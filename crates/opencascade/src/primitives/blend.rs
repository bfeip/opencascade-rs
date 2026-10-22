//! Blends over their faces' extensions: a fillet or chamfer that carries on
//! across holes in its faces and past their far bounds, then is trimmed back
//! to the shape.

use std::cell::OnceCell;

use glam::DVec3;

use crate::bounding_box::aabb;
use crate::primitives::{Compound, Edge, EdgeCorner, EdgeType, Face, FaceType, Shape, ShapeElements};
use crate::{Error, FilletError};

/// How far past a blend's setback holes and narrow faces are in its way, as a
/// multiple of the setback.
const REACH: f64 = 1.5;

/// More faces than this behind one inner wire are not a feature.
const MAX_FEATURE_FACES: usize = 64;

/// Distance within which points and planes coincide.
const COINCIDENCE: f64 = 1e-6;

impl Shape {
    /// Rounds `edges` like [`fillet_edges`](Self::fillet_edges), but over
    /// their faces' extensions: the round carries on across holes in the
    /// faces and past their far bounds, and is trimmed back to the shape.
    ///
    /// Only straight, convex edges between planes are rounded this way; any
    /// other selection is rounded as `fillet_edges` would.
    pub fn fillet_edges_extended<T: AsRef<Edge>>(
        &self,
        radius: f64,
        edges: impl IntoIterator<Item = T>,
    ) -> Result<Self, Error> {
        self.blend_extended(Blend::Fillet(radius), edges)
    }

    /// Bevels `edges` like [`chamfer_edges`](Self::chamfer_edges), over their
    /// faces' extensions as [`fillet_edges_extended`](Self::fillet_edges_extended)
    /// rounds them.
    pub fn chamfer_edges_extended<T: AsRef<Edge>>(
        &self,
        distance: f64,
        edges: impl IntoIterator<Item = T>,
    ) -> Result<Self, Error> {
        self.blend_extended(Blend::Chamfer(distance), edges)
    }

    fn blend_extended<T: AsRef<Edge>>(
        &self,
        blend: Blend,
        edges: impl IntoIterator<Item = T>,
    ) -> Result<Self, Error> {
        let edges: Vec<T> = edges.into_iter().collect();
        let elements = ShapeElements::new(self);
        let frames: Option<Vec<EdgeFrame>> =
            edges.iter().map(|edge| EdgeFrame::new(&elements, edge.as_ref())).collect();
        let Some(frames) = frames.filter(|frames| frames.iter().all(|frame| frame.corner.convex == Some(true)))
        else {
            return blend.apply(self, &edges);
        };

        let reach = REACH * frames.iter().map(|frame| blend.setback(&frame.corner)).fold(0.0, f64::max);
        let holes = holes_in_reach(&elements, &frames, reach);
        let pushes = far_faces(&elements, &frames, reach);
        if holes.is_empty() && pushes.is_empty() {
            return blend.apply(self, &edges);
        }

        let mut extended = match holes.is_empty() {
            true => self.deep_copy(),
            false => self.remove_features(holes.iter().map(|&face| &elements.faces()[face]))?.0,
        };
        for push in &pushes {
            let face = face_in_plane(&extended, push.point, push.normal).ok_or(LOST)?;
            extended = extended.tweak_faces([&face], translation(push.normal * push.by))?;
        }

        let found: Option<Vec<Edge>> = frames
            .iter()
            .map(|frame| straight_edge_through(&extended, frame.corner.point, frame.corner.tangent))
            .collect();
        let blended = blend.apply(&extended, found.ok_or(LOST)?)?;

        // A convex blend only takes material away, so the shape's own bounds
        // trim it.
        self.deep_copy().intersect(&blended)?.shape.clean()
    }
}

/// An extended face or edge that cannot be found again.
const LOST: Error = Error::FilletFailed(FilletError::Unknown);

#[derive(Clone, Copy, Debug)]
enum Blend {
    Fillet(f64),
    Chamfer(f64),
}

impl Blend {
    /// How far the blend reaches along each face of `corner`.
    fn setback(self, corner: &EdgeCorner) -> f64 {
        match self {
            Blend::Fillet(radius) => radius * (corner.normal_angle() / 2.0).tan(),
            Blend::Chamfer(distance) => distance,
        }
    }

    fn apply<T: AsRef<Edge>>(self, shape: &Shape, edges: impl IntoIterator<Item = T>) -> Result<Shape, Error> {
        match self {
            Blend::Fillet(radius) => shape.fillet_edges(radius, edges),
            Blend::Chamfer(distance) => shape.chamfer_edges(distance, edges),
        }
    }
}

/// A straight edge between two planes, and the corner they make.
struct EdgeFrame {
    start: DVec3,
    end: DVec3,
    corner: EdgeCorner,
}

impl EdgeFrame {
    /// The frame of `edge`, unless it is anything but a straight edge between
    /// two planes.
    fn new(elements: &ShapeElements, edge: &Edge) -> Option<Self> {
        if edge.edge_type() != EdgeType::Line {
            return None;
        }
        let corner = elements.corner(edge)?;
        let planes = corner.faces.iter().all(|&face| elements.faces()[face].face_type() == FaceType::Plane);
        planes.then(|| Self { start: edge.start_point(), end: edge.end_point(), corner })
    }
}

/// The faces of the holes and pockets through the frames' faces that come
/// within `reach` of their edges.
fn holes_in_reach(elements: &ShapeElements, frames: &[EdgeFrame], reach: f64) -> Vec<usize> {
    let supports: Vec<usize> = frames.iter().flat_map(|frame| frame.corner.faces).collect();
    let zone = frames.iter().fold((DVec3::MAX, DVec3::MIN), |(low, high), frame| {
        (low.min(frame.start.min(frame.end) - reach), high.max(frame.start.max(frame.end) + reach))
    });
    let shape = elements.shape();
    let volume = shape.volume();
    inner_wire_features(elements, &supports)
        .into_iter()
        .filter(|feature| overlaps(bounds(elements, feature), zone))
        // Filling a hole adds material; filling around a boss takes it away.
        .filter(|feature| {
            shape
                .remove_features(feature.iter().map(|&face| &elements.faces()[face]))
                .is_ok_and(|(filled, _)| filled.volume() > volume * (1.0 + 1e-9))
        })
        .flatten()
        .collect()
}

/// The holes, pockets and bosses behind inner wires of `supports`, each as a
/// connected group of faces. A group that also borders a support along its
/// outer boundary belongs to the corner itself and is left out.
fn inner_wire_features(elements: &ShapeElements, supports: &[usize]) -> Vec<Vec<usize>> {
    let faces = elements.faces();
    let rims: Vec<OnceCell<Vec<Edge>>> = faces.iter().map(|_| OnceCell::new()).collect();
    let rim = |face: usize| -> &[Edge] {
        rims[face].get_or_init(|| faces[face].inner_wires().flat_map(|wire| wire.edges()).collect())
    };
    // A face a feature pierces hosts it.
    let pierces = |face: usize, edge: &Edge| rim(face).iter().any(|rim_edge| rim_edge.is_same(edge));

    let mut features = Vec::new();
    let mut visited: Vec<usize> = Vec::new();
    for &support in supports {
        for rim_edge in rim(support) {
            for &start in elements.adjacent_faces(rim_edge) {
                if supports.contains(&start) || visited.contains(&start) {
                    continue;
                }
                let mut feature = vec![start];
                let mut attached = false;
                let mut next = 0;
                while next < feature.len() && feature.len() <= MAX_FEATURE_FACES {
                    let face = feature[next];
                    next += 1;
                    for edge in faces[face].edges() {
                        for &neighbour in elements.adjacent_faces(&edge) {
                            if feature.contains(&neighbour) {
                                continue;
                            }
                            let pierced = pierces(neighbour, &edge);
                            if supports.contains(&neighbour) {
                                attached |= !pierced;
                            } else if !pierced {
                                feature.push(neighbour);
                            }
                        }
                    }
                }
                visited.extend(&feature);
                if !attached && feature.len() <= MAX_FEATURE_FACES {
                    features.push(feature);
                }
            }
        }
    }
    features
}

/// A planar face to move out along its normal.
struct Push {
    point: DVec3,
    normal: DVec3,
    by: f64,
}

/// The planar faces across the far side of each frame's faces closer to its
/// edge than `reach`, which would cut the blend short, with how far each must
/// move out.
fn far_faces(elements: &ShapeElements, frames: &[EdgeFrame], reach: f64) -> Vec<Push> {
    let faces = elements.faces();
    let ends: Vec<DVec3> = frames.iter().flat_map(|frame| [frame.start, frame.end]).collect();
    let supports: Vec<usize> = frames.iter().flat_map(|frame| frame.corner.faces).collect();
    let mut pushes: Vec<(usize, Push)> = Vec::new();
    for frame in frames {
        for support in frame.corner.faces {
            for edge in faces[support].outer_wire().edges() {
                let touches = [edge.start_point(), edge.end_point()]
                    .iter()
                    .any(|point| ends.iter().any(|end| end.distance(*point) < COINCIDENCE));
                if touches {
                    continue;
                }
                for &far in elements.adjacent_faces(&edge) {
                    let face = &faces[far];
                    // A piece of a support's own plane carries it on.
                    if supports.contains(&far)
                        || face.face_type() != FaceType::Plane
                        || supports.iter().any(|&support| same_plane(&faces[support], face))
                    {
                        continue;
                    }
                    let point = face.center_of_mass();
                    let Ok(normal) = face.normal_at(point) else { continue };
                    let distance = (point - frame.corner.point).dot(normal);
                    if distance <= COINCIDENCE || distance >= reach {
                        continue;
                    }
                    let by = reach - distance;
                    match pushes.iter_mut().find(|(index, _)| *index == far) {
                        Some((_, push)) => push.by = push.by.max(by),
                        None => pushes.push((far, Push { point, normal, by })),
                    }
                }
            }
        }
    }
    pushes.into_iter().map(|(_, push)| push).collect()
}

/// The corners of the box around `faces`.
fn bounds(elements: &ShapeElements, faces: &[usize]) -> (DVec3, DVec3) {
    let compound = Compound::from_shapes(faces.iter().map(|&face| Shape::from(&elements.faces()[face])));
    let bbox = aabb(&Shape::from(compound));
    (bbox.min(), bbox.max())
}

fn overlaps(a: (DVec3, DVec3), b: (DVec3, DVec3)) -> bool {
    a.0.cmple(b.1).all() && b.0.cmple(a.1).all()
}

fn same_plane(a: &Face, b: &Face) -> bool {
    if a.face_type() != FaceType::Plane || b.face_type() != FaceType::Plane {
        return false;
    }
    let (point, other) = (a.center_of_mass(), b.center_of_mass());
    match (a.normal_at(point), b.normal_at(other)) {
        (Ok(n), Ok(m)) => n.dot(m).abs() > 1.0 - 1e-9 && (other - point).dot(n).abs() < COINCIDENCE,
        _ => false,
    }
}

fn translation(by: DVec3) -> [[f64; 4]; 4] {
    #[rustfmt::skip]
    let matrix = [
        [1.0, 0.0, 0.0, by.x],
        [0.0, 1.0, 0.0, by.y],
        [0.0, 0.0, 1.0, by.z],
        [0.0, 0.0, 0.0, 1.0],
    ];
    matrix
}

/// The planar face of `shape` in the plane through `point` facing `normal`.
fn face_in_plane(shape: &Shape, point: DVec3, normal: DVec3) -> Option<Face> {
    shape.faces().find(|face| {
        face.face_type() == FaceType::Plane
            && (face.center_of_mass() - point).dot(normal).abs() < COINCIDENCE
            && face.normal_at(point).is_ok_and(|n| n.dot(normal) > 1.0 - 1e-9)
    })
}

/// The straight edge of `shape` running along `direction` through `point`.
fn straight_edge_through(shape: &Shape, point: DVec3, direction: DVec3) -> Option<Edge> {
    let direction = direction.normalize();
    shape.edges().find(|edge| {
        if edge.edge_type() != EdgeType::Line {
            return false;
        }
        let (start, end) = (edge.start_point(), edge.end_point());
        let length = start.distance(end);
        if length <= COINCIDENCE {
            return false;
        }
        let axis = (end - start) / length;
        let along = (point - start).dot(axis).clamp(0.0, length);
        axis.dot(direction).abs() > 1.0 - 1e-9 && (start + axis * along).distance(point) < COINCIDENCE
    })
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;
    use std::f64::consts::TAU;
    use std::time::{Duration, Instant};

    use glam::{dvec3, DMat3, DQuat};

    use oracles::{arc, chamfer, corner_area, holed_fillet_volume};
    use super::*;
    use crate::primitives::{ShapeType, Wire};

    const SIDE: f64 = 20.0;

    fn block(x: f64, y: f64, z: f64) -> Shape {
        Shape::box_from_corners(DVec3::ZERO, dvec3(x, y, z))
    }

    fn cube() -> Shape {
        block(SIDE, SIDE, SIDE)
    }

    /// A vertical hole of `radius` at `(x, y)`, through a body `top` tall or
    /// `depth` down from its top.
    fn bore(x: f64, y: f64, radius: f64, top: f64, depth: Option<f64>) -> Shape {
        let (bottom, height) = match depth {
            None => (-1.0, top + 2.0),
            Some(depth) => (top - depth, depth + 1.0),
        };
        Shape::cylinder(dvec3(x, y, bottom), radius, DVec3::Z, height)
    }

    fn solid(shape: Shape) -> Shape {
        if shape.shape_type() != ShapeType::Compound {
            return shape;
        }
        let mut children = shape.sub_shapes();
        match (children.next(), children.next()) {
            (Some(only), None) if only.shape_type() == ShapeType::Solid => only,
            _ => shape,
        }
    }

    fn cut(body: &Shape, tool: &Shape) -> Shape {
        solid(body.subtract(tool).expect("the cut builds").shape)
    }

    /// The straight edge through `point` along `direction`.
    fn edge(body: &Shape, point: DVec3, direction: DVec3) -> Edge {
        straight_edge_through(body, point, direction).expect("an edge runs there")
    }

    /// The edge along x on top of the front (y = 0) face.
    fn top_front(body: &Shape) -> Edge {
        let top = aabb(body).max().z;
        edge(body, dvec3(10.0, 0.0, top), DVec3::X)
    }

    /// `edges` of `body` blended the plain way, on a copy.
    fn plain(body: &Shape, edges: impl Fn(&Shape) -> Vec<Edge>, blend: Blend) -> Result<Shape, Error> {
        let copy = body.deep_copy();
        blend.apply(&copy, edges(&copy)).map(solid)
    }

    fn extended(body: &Shape, edges: &[Edge], blend: Blend) -> Shape {
        solid(body.blend_extended(blend, edges).expect("the extended blend builds"))
    }

    /// Whether every face is an elementary surface: no B-spline strip from a
    /// ball riding a rim.
    fn elementary(shape: &Shape) -> bool {
        shape.faces().all(|face| {
            matches!(
                face.face_type(),
                FaceType::Plane | FaceType::Cylinder | FaceType::Cone | FaceType::Sphere | FaceType::Torus
            )
        })
    }

    /// How far `shape`'s volume is from `expected`, relatively, unless it is
    /// not a valid single solid at all.
    fn volume_error(shape: &Shape, expected: f64) -> Result<f64, String> {
        if shape.shape_type() != ShapeType::Solid {
            return Err(format!("not a single solid: {:?}", shape.shape_type()));
        }
        if !shape.is_valid() {
            return Err("invalid".into());
        }
        Ok((shape.volume() - expected).abs() / expected)
    }

    #[track_caller]
    fn assert_volume(shape: &Shape, expected: f64) {
        match volume_error(shape, expected) {
            Ok(error) => assert!(error < 1e-5, "volume {} vs expected {expected}", shape.volume()),
            Err(why) => panic!("{why}"),
        }
    }

    #[test]
    fn the_hole_oracle_matches_occt_where_occt_can_blend() {
        let body = cube();
        let filleted = plain(&body, |b| vec![top_front(b)], Blend::Fillet(6.0)).unwrap();
        let expected = cut(&filleted, &bore(10.0, 3.0, 2.0, SIDE, None)).volume();
        let integrated = holed_fillet_volume(SIDE, SIDE, SIDE, 6.0, 3.0, 2.0);
        assert!((integrated - expected).abs() < 1e-6 * expected, "{integrated} vs {expected}");
    }

    /// A bore spanning y = 3..9 under a fillet whose contact line, at y = r,
    /// crosses it. The plain fillet's ball rides the rim at r = 4 (a B-spline
    /// strip), fails to walk at r = 6 and builds an invalid body at r = 10.
    #[test]
    fn a_fillet_runs_over_a_hole_in_its_face() {
        let body = cut(&cube(), &bore(10.0, 6.0, 3.0, SIDE, None));
        for radius in [4.0, 6.0, 10.0] {
            let plain = plain(&body, |b| vec![top_front(b)], Blend::Fillet(radius));
            assert!(plain.map_or(true, |shape| !elementary(&shape)), "r={radius}: the plain fillet is clean");

            let shape = extended(&body, &[top_front(&body)], Blend::Fillet(radius));
            assert!(elementary(&shape), "r={radius}");
            assert_volume(&shape, holed_fillet_volume(SIDE, SIDE, SIDE, radius, 6.0, 3.0));
        }
    }

    #[test]
    fn a_fillet_runs_over_blind_holes_and_pockets() {
        let blend = Blend::Fillet(6.0);
        let filleted = plain(&cube(), |b| vec![top_front(b)], blend).unwrap();
        let tools = [
            bore(10.0, 6.0, 3.0, SIDE, Some(8.0)),
            Shape::box_from_corners(dvec3(7.0, 3.0, 15.0), dvec3(13.0, 9.0, 21.0)),
        ];
        for tool in tools {
            let body = cut(&cube(), &tool);
            assert_volume(&extended(&body, &[top_front(&body)], blend), cut(&filleted, &tool).volume());
        }
    }

    /// The rim's own round goes with the hole, so all that is left of it is
    /// what lies below the new fillet.
    #[test]
    fn a_fillet_runs_over_a_hole_with_a_rounded_rim() {
        let body = cut(&cube(), &bore(10.0, 6.0, 3.0, SIDE, None));
        let rim = body
            .edges()
            .find(|e| e.edge_type() == EdgeType::Circle && (e.start_point().z - SIDE).abs() < 1e-9)
            .expect("the top rim");
        let rimmed = solid(body.fillet_edges(1.0, [&rim]).expect("the rim rounds"));

        let shape = extended(&rimmed, &[top_front(&rimmed)], Blend::Fillet(6.0));
        assert!(shape.is_valid() && elementary(&shape));
        // The rim's round lies between the bore and a bore one wider.
        let narrow = holed_fillet_volume(SIDE, SIDE, SIDE, 6.0, 6.0, 3.0);
        let wide = holed_fillet_volume(SIDE, SIDE, SIDE, 6.0, 6.0, 4.0);
        let volume = shape.volume();
        assert!(wide < volume && volume < narrow, "{wide} < {volume} < {narrow}");
    }

    /// Short of the front face, exactly as tall as it, past it, and past the
    /// top face too: the round ends where the block does.
    #[test]
    fn a_fillet_runs_past_its_faces() {
        let deep = || block(20.0, 40.0, 20.0);
        for (body, radius) in [(deep(), 19.8), (deep(), 20.0), (deep(), 30.0), (cube(), 30.0)] {
            let size = aabb(&body).max();
            let shape = extended(&body, &[top_front(&body)], Blend::Fillet(radius));
            assert_volume(&shape, body.volume() - size.x * corner_area(arc(radius), size.y, size.z));
        }
    }

    /// As in the screenshot this started from: the fillet takes the whole
    /// front face and crosses a hole on the way.
    #[test]
    fn a_full_height_fillet_runs_over_a_hole() {
        let body = cut(&block(20.0, 30.0, 20.0), &bore(10.0, 20.0, 4.0, SIDE, None));
        let shape = extended(&body, &[top_front(&body)], Blend::Fillet(20.0));
        assert!(elementary(&shape));
        assert_volume(&shape, holed_fillet_volume(20.0, 30.0, SIDE, 20.0, 20.0, 4.0));
    }

    /// Two fillets meeting at a corner, and three rounding it into a sphere,
    /// with a hole in their way.
    #[test]
    fn corners_keep_their_vertex_blends() {
        let hole = bore(5.0, 5.0, 2.0, SIDE, None);
        let body = cut(&cube(), &hole);
        let corner = |body: &Shape| {
            vec![
                top_front(body),
                edge(body, dvec3(0.0, 10.0, SIDE), DVec3::Y),
                edge(body, dvec3(0.0, 0.0, 10.0), DVec3::Z),
            ]
        };
        for count in [2, 3] {
            let edges = |body: &Shape| -> Vec<Edge> { corner(body).into_iter().take(count).collect() };
            let shape = extended(&body, &edges(&body), Blend::Fillet(6.0));
            let filleted = plain(&cube(), edges, Blend::Fillet(6.0)).unwrap();
            assert_volume(&shape, cut(&filleted, &hole).volume());
        }
    }

    #[test]
    fn with_nothing_in_the_way_the_blend_is_the_plain_one() {
        let body = cut(&cube(), &bore(10.0, 15.0, 2.0, SIDE, None));
        let shape = extended(&body, &[top_front(&body)], Blend::Fillet(5.0));
        let plain = plain(&body, |b| vec![top_front(b)], Blend::Fillet(5.0)).unwrap();
        assert_eq!(shape.faces().count(), plain.faces().count());
        assert!((shape.volume() - plain.volume()).abs() < 1e-9);
    }

    /// The fillet over a hole, past both faces, and full height over a hole,
    /// as chamfers.
    #[test]
    fn chamfers_run_over_holes_and_past_their_faces() {
        let hole = bore(10.0, 6.0, 3.0, SIDE, None);
        let body = cut(&cube(), &hole);
        let chamfered = plain(&cube(), |b| vec![top_front(b)], Blend::Chamfer(6.0)).unwrap();
        assert_volume(&extended(&body, &[top_front(&body)], Blend::Chamfer(6.0)), cut(&chamfered, &hole).volume());

        let body = cube();
        let expected = body.volume() - SIDE * corner_area(chamfer(30.0), SIDE, SIDE);
        assert_volume(&extended(&body, &[top_front(&body)], Blend::Chamfer(30.0)), expected);

        // A chamfer as tall as the front face leaves a wedge, with the hole
        // through it.
        let hole = bore(10.0, 20.0, 4.0, SIDE, None);
        let body = cut(&block(20.0, 30.0, 20.0), &hole);
        let section = [dvec3(0.0, 0.0, 0.0), dvec3(0.0, 20.0, 20.0), dvec3(0.0, 30.0, 20.0), dvec3(0.0, 30.0, 0.0)];
        let wedge = Face::from_wire(&Wire::from_ordered_points(section).unwrap()).unwrap().extrude(DVec3::X * 20.0);
        let expected = cut(&Shape::from(wedge), &hole).volume();
        assert_volume(&extended(&body, &[top_front(&body)], Blend::Chamfer(20.0)), expected);
    }

    /// Concave edges and curved ones are left to the plain blend.
    #[test]
    fn other_edges_blend_the_plain_way() {
        let slab = block(20.0, 20.0, 10.0);
        let wall = Shape::box_from_corners(dvec3(0.0, 10.0, 10.0), dvec3(20.0, 20.0, 20.0));
        let step = solid(slab.union(&wall).unwrap().shape.clean().unwrap());
        let inner = |body: &Shape| vec![edge(body, dvec3(10.0, 10.0, 10.0), DVec3::X)];
        let shape = extended(&step, &inner(&step), Blend::Fillet(3.0));
        let plain = plain(&step, inner, Blend::Fillet(3.0)).unwrap();
        assert!(shape.volume() > step.volume(), "a concave fillet adds material");
        assert!((shape.volume() - plain.volume()).abs() < 1e-9);

        let body = cut(&cube(), &bore(10.0, 6.0, 3.0, SIDE, None));
        let rim = body
            .edges()
            .find(|e| e.edge_type() == EdgeType::Circle && (e.start_point().z - SIDE).abs() < 1e-9)
            .expect("the top rim");
        let shape = extended(&body, &[rim], Blend::Fillet(1.0));
        assert!(shape.volume() < body.volume());
    }

    /// Similarity transforms placing a body in general position: fixed
    /// pseudo-random rotations and offsets, the same every run.
    fn placements(count: usize) -> Vec<[[f64; 4]; 4]> {
        let mut seed: u64 = 0x2545_f491_4f6c_dd1d;
        let mut next = move || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        (0..count)
            .map(|_| {
                let axis = dvec3(next() * 2.0 - 1.0, next() * 2.0 - 1.0, next() * 2.0 - 1.0).normalize();
                let r = DMat3::from_quat(DQuat::from_axis_angle(axis, next() * TAU));
                let t = dvec3(next(), next(), next()) * 100.0 - 50.0;
                #[rustfmt::skip]
                let m = [
                    [r.x_axis.x, r.y_axis.x, r.z_axis.x, t.x],
                    [r.x_axis.y, r.y_axis.y, r.z_axis.y, t.y],
                    [r.x_axis.z, r.y_axis.z, r.z_axis.z, t.z],
                    [0.0, 0.0, 0.0, 1.0],
                ];
                m
            })
            .collect()
    }

    fn place(m: &[[f64; 4]; 4], p: DVec3) -> DVec3 {
        dvec3(
            m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3],
            m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3],
            m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3],
        )
    }

    /// Extended blends over a sweep of sizes and 24 placements in general
    /// position, against hand-worked volumes (volume does not care where the
    /// body sits). Faces bounded by approximated intersection curves integrate
    /// a few 1e-6 off, so 1e-5 passes; a mis-trim is off by ~1e-2.
    ///
    /// `cargo test -p opencascade --release blend::tests::stress_report -- --ignored --nocapture`
    #[test]
    #[ignore = "a report, not a check: run with --release --ignored --nocapture"]
    fn stress_report() {
        const TOLERANCE: f64 = 1e-5;
        let placements = placements(24);
        let holed = cut(&cube(), &bore(10.0, 6.0, 3.0, SIDE, None));
        let deep_holed = cut(&block(20.0, 30.0, 20.0), &bore(10.0, 20.0, 4.0, SIDE, None));

        // Per case: runs, passes, worst error, total time, failures.
        let mut tallies: BTreeMap<&str, (usize, usize, f64, Duration, Vec<String>)> = BTreeMap::new();
        for factor in [0.1, 0.3, 0.5, 0.75, 0.99, 1.0, 1.25, 1.5, 2.0] {
            let size = factor * SIDE;
            let cases = [
                ("fillet over a hole", &holed, Blend::Fillet(size), holed_fillet_volume(SIDE, SIDE, SIDE, size, 6.0, 3.0)),
                ("fillet past the faces", &cube(), Blend::Fillet(size), SIDE.powi(3) - SIDE * corner_area(arc(size), SIDE, SIDE)),
                ("chamfer past the faces", &cube(), Blend::Chamfer(size), SIDE.powi(3) - SIDE * corner_area(chamfer(size), SIDE, SIDE)),
                ("screenshot 2", &deep_holed, Blend::Fillet(size), holed_fillet_volume(20.0, 30.0, SIDE, size, 20.0, 4.0)),
            ];
            for (name, body, blend, oracle) in cases {
                // A blend that takes the whole body has nothing to measure.
                if oracle < 0.01 * body.volume() {
                    continue;
                }
                let tally = tallies.entry(name).or_default();
                for (index, m) in placements.iter().enumerate() {
                    let clock = Instant::now();
                    let result = body.transformed(*m).map_err(|e| e.to_string()).and_then(|placed| {
                        let along = place(m, DVec3::X) - place(m, DVec3::ZERO);
                        let edge = straight_edge_through(&placed, place(m, dvec3(10.0, 0.0, SIDE)), along)
                            .ok_or("placed edge lost")?;
                        let shape = placed.blend_extended(blend, [&edge]).map_err(|e| e.to_string())?;
                        volume_error(&solid(shape), oracle)
                    });
                    tally.0 += 1;
                    tally.3 += clock.elapsed();
                    match result {
                        Ok(error) if error <= TOLERANCE => {
                            tally.1 += 1;
                            tally.2 = tally.2.max(error);
                        },
                        Ok(error) => {
                            tally.2 = tally.2.max(error);
                            tally.4.push(format!("f={factor} p={index}: volume off by {error:.1e}"));
                        },
                        Err(why) => tally.4.push(format!("f={factor} p={index}: {why}")),
                    }
                }
            }
        }

        println!("\n{:<24} {:>9} {:>9} {:>9}", "case", "passed", "worst", "mean ms");
        for (name, (runs, passed, worst, time, _)) in &tallies {
            let mean = time.as_secs_f64() * 1e3 / *runs as f64;
            println!("{name:<24} {passed:>4}/{runs:<4} {worst:>9.1e} {mean:>9.1}");
        }
        for (name, (.., failures)) in &tallies {
            for failure in failures {
                println!("{name}: {failure}");
            }
        }
    }

    /// Volumes of blended blocks worked out by hand, without OCCT.
    mod oracles {
        use std::f64::consts::{FRAC_PI_2, PI};

        /// Area a blend takes out of a right-angled corner whose faces are
        /// `width_a` and `width_b` wide. `section` traces the blend from its
        /// contact on face A (t = 0) to its contact on face B (t = 1), in the
        /// corner's own coordinates: along face A, then along face B.
        pub fn corner_area(section: impl Fn(f64) -> [f64; 2], width_a: f64, width_b: f64) -> f64 {
            const SAMPLES: usize = 4000;
            let mut polygon = vec![[0.0, 0.0]];
            polygon.extend((0..=SAMPLES).map(|i| section(i as f64 / SAMPLES as f64)));
            for (axis, limit) in [(0, width_a), (1, width_b)] {
                polygon = clip(&polygon, axis, limit);
            }
            shoelace(&polygon).abs()
        }

        /// A fillet's quarter circle.
        pub fn arc(radius: f64) -> impl Fn(f64) -> [f64; 2] {
            move |t| {
                let angle = t * FRAC_PI_2;
                [radius - radius * angle.sin(), radius - radius * angle.cos()]
            }
        }

        /// A chamfer's straight cut.
        pub fn chamfer(distance: f64) -> impl Fn(f64) -> [f64; 2] {
            move |t| [distance * (1.0 - t), distance * t]
        }

        /// Volume of an `x`×`y`×`h` block whose top-front edge (along x) is
        /// rounded to `radius`, past the faces if need be, with a vertical
        /// through hole of radius `hole` centred `hole_y` back from the front.
        pub fn holed_fillet_volume(x: f64, y: f64, h: f64, radius: f64, hole_y: f64, hole: f64) -> f64 {
            // Material left above a point `depth` back from the front: under
            // the arc, centred `radius` back and `radius` down from the edge.
            let column = |depth: f64| match depth < radius {
                true => (h - radius + (radius * radius - (depth - radius).powi(2)).sqrt()).clamp(0.0, h),
                false => h,
            };
            // Simpson's rule over the hole's disc, at depth hole_y + hole·sin θ
            // where its chord is 2·hole·cos θ.
            const STEPS: usize = 200_000;
            let step = PI / STEPS as f64;
            let strip = |i: usize| {
                let theta = -FRAC_PI_2 + i as f64 * step;
                2.0 * (hole * theta.cos()).powi(2) * column(hole_y + hole * theta.sin())
            };
            let weight = |i: usize| match i {
                0 => 1.0,
                i if i == STEPS => 1.0,
                i if i % 2 == 1 => 4.0,
                _ => 2.0,
            };
            let bored = (0..=STEPS).map(|i| weight(i) * strip(i)).sum::<f64>() * step / 3.0;
            x * y * h - x * corner_area(arc(radius), y, h) - bored
        }

        /// `polygon` clipped to `point[axis] <= limit` (Sutherland–Hodgman).
        fn clip(polygon: &[[f64; 2]], axis: usize, limit: f64) -> Vec<[f64; 2]> {
            let inside = |p: &[f64; 2]| p[axis] <= limit;
            let mut clipped = Vec::new();
            for i in 0..polygon.len() {
                let (current, next) = (polygon[i], polygon[(i + 1) % polygon.len()]);
                if inside(&current) {
                    clipped.push(current);
                }
                if inside(&current) != inside(&next) {
                    let t = (limit - current[axis]) / (next[axis] - current[axis]);
                    clipped.push([current[0] + (next[0] - current[0]) * t, current[1] + (next[1] - current[1]) * t]);
                }
            }
            clipped
        }

        fn shoelace(polygon: &[[f64; 2]]) -> f64 {
            let doubled: f64 = (0..polygon.len())
                .map(|i| {
                    let (a, b) = (polygon[i], polygon[(i + 1) % polygon.len()]);
                    a[0] * b[1] - b[0] * a[1]
                })
                .sum();
            doubled / 2.0
        }
    }
}
