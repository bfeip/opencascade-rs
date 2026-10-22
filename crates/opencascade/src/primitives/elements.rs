use std::sync::OnceLock;

use cxx::UniquePtr;
use glam::DVec3;
use opencascade_sys::ffi;

use crate::{
    primitives::{Edge, Face, Shape, Vertex, Wire},
    Error,
};

/// A shape's faces, edges and vertices as a user sees them, numbered.
///
/// Each kind is numbered from 0 in the order OCCT first meets them exploring the
/// shape. Seams, degenerate edges and the vertices only they need are left out,
/// and the edges a seam split out of one curve are one [`EdgeElement`].
pub struct ShapeElements {
    shape: Shape,
    faces: Vec<Face>,
    edges: Vec<EdgeElement>,
    vertices: Vec<Vertex>,
    seams: Vec<Edge>,
    face_lookup: Lookup,
    edge_lookup: Lookup,
    vertex_lookup: Lookup,
    /// Per sub-shape slot of `edge_lookup`: the faces that edge bounds.
    edge_faces: OnceLock<Vec<Vec<usize>>>,
}

/// An edge as a user sees it: one of the shape's edges, or several where a seam
/// split one curve.
pub struct EdgeElement {
    pieces: Vec<Edge>,
    /// The pieces joined into one, when there are several.
    joined: Option<Edge>,
}

impl EdgeElement {
    /// The shape's edges this element covers, in order along its curve: one, or
    /// more where a seam split it.
    pub fn pieces(&self) -> &[Edge] {
        &self.pieces
    }

    /// This element as one edge: the shape's own edge, or a new edge joining the
    /// pieces that is not part of the shape.
    pub fn curve(&self) -> &Edge {
        self.joined.as_ref().unwrap_or(&self.pieces[0])
    }
}

impl ShapeElements {
    pub fn new(shape: &Shape) -> Self {
        let mut splits = SeamSplits::new(shape);

        let mut face_map = ffi::new_indexed_map_of_shape();
        ffi::map_shapes(&shape.inner, ffi::TopAbs_ShapeEnum::TopAbs_FACE, face_map.pin_mut());
        let faces = (1..=face_map.Extent())
            .map(|i| Face::from_face(ffi::TopoDS_cast_to_face(face_map.FindKey(i))))
            .collect();

        let mut edges = Vec::new();
        for run in std::mem::take(&mut splits.chains) {
            let pieces: Vec<Edge> = run.iter().map(|end| splits.edge(end.edge)).collect();
            if pieces.len() == 1 {
                edges.push(EdgeElement { pieces, joined: None });
                continue;
            }
            match join(&pieces) {
                Some(joined) => edges.push(EdgeElement { pieces, joined: Some(joined) }),
                // Pieces that make no one edge are edges of their own, meeting at
                // vertices of their own.
                None => {
                    for end in &run {
                        splits.reveal(end.other());
                    }
                    edges.extend(
                        pieces
                            .into_iter()
                            .map(|piece| EdgeElement { pieces: vec![piece], joined: None }),
                    );
                },
            }
        }

        let vertices = (0..splits.hidden.len())
            .filter(|&v| !splits.hidden[v])
            .map(|v| splits.vertex(v))
            .collect();
        let seams = (0..splits.seams.len())
            .filter(|&e| splits.seams[e] && !splits.degenerate[e])
            .map(|e| splits.edge(e))
            .collect();
        Self::indexed(shape.clone(), faces, edges, vertices, seams)
    }

    /// A deep copy of the shape ([`Shape::deep_copy`]) with these elements
    /// carried onto it, numbered as here.
    ///
    /// A joined [`EdgeElement::curve`] is shared: it is part of neither shape.
    #[must_use]
    pub fn deep_copy(&self) -> Self {
        let mut copier = ffi::BRepBuilderAPI_Copy_ctor(&self.shape.inner);
        let shape = Shape::from_shape(copier.pin_mut().Shape());
        let copied = |sub_shape: &ffi::TopoDS_Shape| {
            ffi::BRepBuilderAPI_Copy_ModifiedShape(&copier, sub_shape)
        };
        let face = |face: &Face| {
            Face::from_face(ffi::TopoDS_cast_to_face(&copied(ffi::cast_face_to_shape(&face.inner))))
        };
        let edge = |edge: &Edge| {
            Edge::from_edge(ffi::TopoDS_cast_to_edge(&copied(ffi::cast_edge_to_shape(&edge.inner))))
        };
        let vertex = |vertex: &Vertex| {
            Vertex::from_vertex(ffi::TopoDS_cast_to_vertex(&copied(ffi::cast_vertex_to_shape(
                &vertex.inner,
            ))))
        };

        let faces = self.faces.iter().map(face).collect();
        let edges = self
            .edges
            .iter()
            .map(|element| EdgeElement {
                pieces: element.pieces.iter().map(edge).collect(),
                joined: element.joined.as_ref().map(|joined| Edge::from_edge(&joined.inner)),
            })
            .collect();
        let vertices = self.vertices.iter().map(vertex).collect();
        let seams = self.seams.iter().map(edge).collect();
        Self::indexed(shape, faces, edges, vertices, seams)
    }

    /// The shape these are the elements of.
    pub fn shape(&self) -> &Shape {
        &self.shape
    }

    pub fn faces(&self) -> &[Face] {
        &self.faces
    }

    pub fn edges(&self) -> &[EdgeElement] {
        &self.edges
    }

    pub fn vertices(&self) -> &[Vertex] {
        &self.vertices
    }

    pub fn face(&self, index: usize) -> Result<&Face, Error> {
        self.faces.get(index).ok_or(Error::NoSuchElement("face", index))
    }

    pub fn edge(&self, index: usize) -> Result<&EdgeElement, Error> {
        self.edges.get(index).ok_or(Error::NoSuchElement("edge", index))
    }

    pub fn vertex(&self, index: usize) -> Result<&Vertex, Error> {
        self.vertices.get(index).ok_or(Error::NoSuchElement("vertex", index))
    }

    /// The shape's seam edges: no elements, but drawn to debug a surface's
    /// parametrization.
    pub fn seams(&self) -> &[Edge] {
        &self.seams
    }

    /// The element that is `face` of the shape.
    pub fn face_index(&self, face: &Face) -> Option<usize> {
        self.face_lookup.get(ffi::cast_face_to_shape(&face.inner))
    }

    /// The element that covers `edge` of the shape.
    pub fn edge_index(&self, edge: &Edge) -> Option<usize> {
        self.edge_lookup.get(ffi::cast_edge_to_shape(&edge.inner))
    }

    /// The element that is `vertex` of the shape.
    pub fn vertex_index(&self, vertex: &Vertex) -> Option<usize> {
        self.vertex_lookup.get(ffi::cast_vertex_to_shape(&vertex.inner))
    }

    /// The faces `edge` of the shape bounds, by index, each once: two for an
    /// edge between faces, one for a seam or free edge, none for an edge that
    /// is not part of the shape.
    pub fn adjacent_faces(&self, edge: &Edge) -> &[usize] {
        let edge_faces = self.edge_faces.get_or_init(|| self.map_edge_faces());
        match self.edge_lookup.slot(ffi::cast_edge_to_shape(&edge.inner)) {
            Some(slot) => &edge_faces[slot],
            None => &[],
        }
    }

    /// The corner `edge` of the shape makes between the two faces it bounds,
    /// halfway along it; `None` unless it bounds two.
    pub fn corner(&self, edge: &Edge) -> Option<EdgeCorner> {
        let &[a, b] = self.adjacent_faces(edge) else { return None };
        let (point, tangent) = edge.length_midpoint().ok()?;
        let normals = [self.faces[a].normal_at(point).ok()?, self.faces[b].normal_at(point).ok()?];
        let step = edge.length() * EdgeCorner::PROBE;
        let convex =
            into_face(&self.faces[a], point, normals[0], tangent, step).map(|into_a| normals[1].dot(into_a) < 0.0);
        Some(EdgeCorner { point, tangent, faces: [a, b], normals, convex })
    }

    fn map_edge_faces(&self) -> Vec<Vec<usize>> {
        let mut ancestors = ffi::new_indexed_data_map_of_shape_list_of_shape();
        ffi::map_shapes_and_unique_ancestors(
            &self.shape.inner,
            ffi::TopAbs_ShapeEnum::TopAbs_EDGE,
            ffi::TopAbs_ShapeEnum::TopAbs_FACE,
            ancestors.pin_mut(),
        );
        let mut edge_faces = vec![Vec::new(); self.edge_lookup.elements.len()];
        for index in 1..=ancestors.Extent() {
            let Some(slot) = self.edge_lookup.slot(ancestors.FindKey(index)) else { continue };
            for face in ffi::shape_list_to_vector(ancestors.FindFromIndex(index)).iter() {
                let faces: &mut Vec<usize> = &mut edge_faces[slot];
                if let Some(face) = self.face_lookup.get(face).filter(|face| !faces.contains(face)) {
                    faces.push(face);
                }
            }
        }
        edge_faces
    }

    /// The elements, with lookups from `shape`'s sub-shapes to them.
    fn indexed(
        shape: Shape,
        faces: Vec<Face>,
        edges: Vec<EdgeElement>,
        vertices: Vec<Vertex>,
        seams: Vec<Edge>,
    ) -> Self {
        let mut face_lookup = Lookup::new(&shape, ffi::TopAbs_ShapeEnum::TopAbs_FACE);
        for (index, face) in faces.iter().enumerate() {
            face_lookup.set(ffi::cast_face_to_shape(&face.inner), index);
        }
        let mut edge_lookup = Lookup::new(&shape, ffi::TopAbs_ShapeEnum::TopAbs_EDGE);
        for (index, element) in edges.iter().enumerate() {
            for piece in &element.pieces {
                edge_lookup.set(ffi::cast_edge_to_shape(&piece.inner), index);
            }
        }
        let mut vertex_lookup = Lookup::new(&shape, ffi::TopAbs_ShapeEnum::TopAbs_VERTEX);
        for (index, vertex) in vertices.iter().enumerate() {
            vertex_lookup.set(ffi::cast_vertex_to_shape(&vertex.inner), index);
        }
        Self {
            shape,
            faces,
            edges,
            vertices,
            seams,
            face_lookup,
            edge_lookup,
            vertex_lookup,
            edge_faces: OnceLock::new(),
        }
    }
}

/// The corner two faces make along an edge between them, at a point on it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EdgeCorner {
    /// Where on the edge it is measured.
    pub point: DVec3,
    /// The edge's unit tangent there.
    pub tangent: DVec3,
    /// The two faces, by index.
    pub faces: [usize; 2],
    /// The faces' outward normals there.
    pub normals: [DVec3; 2],
    /// Whether the material between the faces makes an outside corner;
    /// `None` where it cannot be told which side of the edge the first face
    /// lies on.
    pub convex: Option<bool>,
}

impl EdgeCorner {
    /// How far across the edge, as a share of its length, a point is taken to
    /// find which side of it a face lies on.
    const PROBE: f64 = 1e-3;

    /// The angle between the faces' outward normals: zero across a flat seam,
    /// a right angle at a box's edge.
    pub fn normal_angle(&self) -> f64 {
        self.normals[0].angle_between(self.normals[1])
    }
}

/// The direction from `point`, on an edge running along `tangent`, into
/// `face`: whichever way across the edge lands on the face a `step` away.
fn into_face(face: &Face, point: DVec3, normal: DVec3, tangent: DVec3, step: f64) -> Option<DVec3> {
    /// OCCT's `Precision::Confusion`.
    const CONFUSION: f64 = 1e-7;

    let across = normal.cross(tangent).normalize();
    let face = Shape::from(face);
    [across, -across].into_iter().find(|&direction| {
        let probe = point + direction * step;
        face.faces_along_line(probe + normal * step, -normal, CONFUSION)
            .iter()
            .any(|hit| hit.point.distance(probe) < step / 2.0)
    })
}

/// `pieces`, in order along one curve, as one edge; `None` if they make no single
/// edge.
fn join(pieces: &[Edge]) -> Option<Edge> {
    let wire = Wire::from_edges(pieces).ok()?;
    let joined = Shape::from(wire).clean().ok()?;
    let mut edges = joined.edges();
    match (edges.next(), edges.next()) {
        (Some(edge), None) => Some(edge),
        _ => None,
    }
}

/// Which element each of a shape's sub-shapes of one type is, or belongs to.
struct Lookup {
    map: UniquePtr<ffi::TopTools_IndexedMapOfShape>,
    elements: Vec<Option<usize>>,
}

impl Lookup {
    fn new(shape: &Shape, ty: ffi::TopAbs_ShapeEnum) -> Self {
        let mut map = ffi::new_indexed_map_of_shape();
        ffi::map_shapes(&shape.inner, ty, map.pin_mut());
        let elements = vec![None; map.Extent() as usize];
        Self { map, elements }
    }

    fn slot(&self, sub_shape: &ffi::TopoDS_Shape) -> Option<usize> {
        (self.map.FindIndex(sub_shape) as usize).checked_sub(1)
    }

    fn set(&mut self, sub_shape: &ffi::TopoDS_Shape, element: usize) {
        if let Some(slot) = self.slot(sub_shape) {
            self.elements[slot] = Some(element);
        }
    }

    fn get(&self, sub_shape: &ffi::TopoDS_Shape) -> Option<usize> {
        self.elements[self.slot(sub_shape)?]
    }
}

/// One end of an edge of a [`SeamSplits`], by the edge's index.
#[derive(Clone, Copy)]
struct EdgeEnd {
    edge: usize,
    /// The end at the curve's first parameter, rather than its last.
    start: bool,
}

impl EdgeEnd {
    /// Position of this end in a per-end table.
    fn slot(self) -> usize {
        2 * self.edge + usize::from(!self.start)
    }

    fn other(self) -> Self {
        Self { start: !self.start, ..self }
    }
}

/// A shape's edges and vertices as they read with its seams hidden: the
/// vertices that only parametrize a surface, and the runs of edges a seam split
/// out of one curve. Edges and vertices are numbered from 0, in the order of
/// OCCT's maps of them.
struct SeamSplits {
    edges: UniquePtr<ffi::TopTools_IndexedMapOfShape>,
    /// Each vertex, with the edges it bounds.
    vertex_edges: UniquePtr<ffi::TopTools_IndexedDataMapOfShapeListOfShape>,
    /// Per edge: it is a seam of some face.
    seams: Vec<bool>,
    /// Per edge: it has no 3D curve.
    degenerate: Vec<bool>,
    /// Per vertex: it is a seam vertex.
    hidden: Vec<bool>,
    /// Runs of edges joined through seam vertices, each in order along its
    /// curve, entered by the listed end. Every edge that is neither a seam nor
    /// degenerate is in exactly one run.
    chains: Vec<Vec<EdgeEnd>>,
    /// Per edge: the run it belongs to.
    chain_of: Vec<Option<usize>>,
}

impl SeamSplits {
    /// Edges leaving a vertex within this angle (radians) of opposite directions
    /// carry one curve through it.
    const SMOOTH_ANGLE: f64 = 1e-3;

    fn new(shape: &Shape) -> Self {
        let mut edges = ffi::new_indexed_map_of_shape();
        ffi::map_shapes(&shape.inner, ffi::TopAbs_ShapeEnum::TopAbs_EDGE, edges.pin_mut());
        let mut vertex_edges = ffi::new_indexed_data_map_of_shape_list_of_shape();
        ffi::map_shapes_and_unique_ancestors(
            &shape.inner,
            ffi::TopAbs_ShapeEnum::TopAbs_VERTEX,
            ffi::TopAbs_ShapeEnum::TopAbs_EDGE,
            vertex_edges.pin_mut(),
        );
        let (edge_count, vertex_count) = (edges.Extent() as usize, vertex_edges.Extent() as usize);

        let mut splits = Self {
            edges,
            vertex_edges,
            seams: vec![false; edge_count],
            degenerate: vec![false; edge_count],
            hidden: vec![false; vertex_count],
            chains: Vec::new(),
            chain_of: vec![None; edge_count],
        };
        for index in 0..edge_count {
            splits.degenerate[index] = splits.edge(index).is_degenerated();
        }
        for face in shape.faces() {
            for edge in face.edges() {
                if let Some(index) = splits.index_of(&edge).filter(|_| edge.is_seam(&face)) {
                    splits.seams[index] = true;
                }
            }
        }
        let joins = splits.find_seam_vertices();
        splits.chain(&joins);
        splits
    }

    fn edge(&self, index: usize) -> Edge {
        Edge::from_edge(ffi::TopoDS_cast_to_edge(self.edges.FindKey(index as i32 + 1)))
    }

    fn index_of(&self, edge: &Edge) -> Option<usize> {
        (self.edges.FindIndex(ffi::cast_edge_to_shape(&edge.inner)) as usize).checked_sub(1)
    }

    fn vertex(&self, index: usize) -> Vertex {
        Vertex::from_vertex(ffi::TopoDS_cast_to_vertex(self.vertex_edges.FindKey(index as i32 + 1)))
    }

    /// Whether the edge only parametrizes its face.
    fn parametric(&self, index: usize) -> bool {
        self.seams[index] || self.degenerate[index]
    }

    /// Marks the seam vertices, and returns, per [`EdgeEnd::slot`], the edge end
    /// that continues each end through one.
    fn find_seam_vertices(&mut self) -> Vec<Option<EdgeEnd>> {
        let mut joins = vec![None; 2 * self.seams.len()];
        for v in 0..self.hidden.len() {
            let vertex = self.vertex_edges.FindKey(v as i32 + 1);
            let mut parametric = false;
            let mut ends = Vec::new();
            for shape in
                ffi::shape_list_to_vector(self.vertex_edges.FindFromIndex(v as i32 + 1)).iter()
            {
                let Some(index) = (self.edges.FindIndex(shape) as usize).checked_sub(1) else {
                    continue;
                };
                if self.parametric(index) {
                    parametric = true;
                    continue;
                }
                // A closed edge has both its ends here.
                let edge = ffi::TopoDS_cast_to_edge(shape);
                if ffi::cast_vertex_to_shape(&ffi::TopExp_FirstVertex(edge)).IsSame(vertex) {
                    ends.push(EdgeEnd { edge: index, start: true });
                }
                if ffi::cast_vertex_to_shape(&ffi::TopExp_LastVertex(edge)).IsSame(vertex) {
                    ends.push(EdgeEnd { edge: index, start: false });
                }
            }

            self.hidden[v] = parametric
                && match ends[..] {
                    [] => true,
                    [a, b] => self.continues(a, b),
                    _ => false,
                };
            if let (&[a, b], true) = (ends.as_slice(), self.hidden[v]) {
                joins[a.slot()] = Some(b);
                joins[b.slot()] = Some(a);
            }
        }
        joins
    }

    /// Whether the edges leaving a vertex by `a` and `b` carry one smooth curve
    /// straight through it.
    fn continues(&self, a: EdgeEnd, b: EdgeEnd) -> bool {
        let away = |end: EdgeEnd| {
            let edge = self.edge(end.edge);
            if end.start {
                edge.start_tangent()
            } else {
                -edge.end_tangent()
            }
        };
        away(a).dot(away(b)) < -Self::SMOOTH_ANGLE.cos()
    }

    /// Groups the edges that are neither seams nor degenerate into runs along
    /// `joins`.
    fn chain(&mut self, joins: &[Option<EdgeEnd>]) {
        for edge in 0..self.seams.len() {
            if self.parametric(edge) || self.chain_of[edge].is_some() {
                continue;
            }
            // Back up to where the run begins; a closed run begins anywhere.
            let mut entry = EdgeEnd { edge, start: true };
            while let Some(previous) = joins[entry.slot()].filter(|previous| previous.edge != edge)
            {
                entry = previous.other();
            }

            let chain = self.chains.len();
            let mut run = Vec::new();
            loop {
                self.chain_of[entry.edge] = Some(chain);
                run.push(entry);
                match joins[entry.other().slot()] {
                    Some(next) if self.chain_of[next.edge].is_none() => entry = next,
                    _ => break,
                }
            }
            self.chains.push(run);
        }
    }

    /// Shows the vertex at `end` again.
    fn reveal(&mut self, end: EdgeEnd) {
        let edge = self.edge(end.edge);
        let vertex = match end.start {
            true => ffi::TopExp_FirstVertex(&edge.inner),
            false => ffi::TopExp_LastVertex(&edge.inner),
        };
        if let Some(v) = (self.vertex_edges.FindIndex(ffi::cast_vertex_to_shape(&vertex)) as usize)
            .checked_sub(1)
        {
            self.hidden[v] = false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::EdgeType;
    use glam::{dvec3, DVec3};

    /// A ball with a bite out of its side, where the seam meridian runs through
    /// the bite: the seam ends on the bite's rim twice, splitting the rim circle.
    fn ball_bitten_across_its_seam() -> Shape {
        let ball = Shape::sphere(2.0).build();
        let bite = Shape::sphere(1.0).at(dvec3(2.0, 0.0, 0.3)).build();
        ball.subtract(&bite).unwrap().shape.clean().unwrap()
    }

    fn polyline_length(points: &[DVec3]) -> f64 {
        points.windows(2).map(|pair| pair[0].distance(pair[1])).sum()
    }

    fn counts(elements: &ShapeElements) -> (usize, usize, usize, usize) {
        (
            elements.faces().len(),
            elements.edges().len(),
            elements.vertices().len(),
            elements.seams().len(),
        )
    }

    #[test]
    fn a_seam_split_rim_is_one_edge() {
        let elements = ShapeElements::new(&ball_bitten_across_its_seam());
        // Every vertex is a pole or a seam crossing; the seam is in two pieces.
        assert_eq!(counts(&elements), (2, 1, 0, 2));

        let rim = &elements.edges()[0];
        assert_eq!(rim.pieces().len(), 2, "the seam splits the rim in two");
        for piece in rim.pieces() {
            assert_eq!(elements.edge_index(piece), Some(0));
        }
        let length: f64 = rim.pieces().iter().map(Edge::length).sum();
        assert_eq!(rim.curve().edge_type(), EdgeType::Circle);
        assert!(
            (rim.curve().length() - length).abs() < 1e-6,
            "expected {length}, got {}",
            rim.curve().length()
        );
    }

    #[test]
    fn a_seam_split_rim_draws_as_one_closed_loop() {
        let elements = ShapeElements::new(&ball_bitten_across_its_seam());
        let rim = &elements.edges()[0];
        let circumference: f64 = rim.pieces().iter().map(Edge::length).sum();

        let points = rim.curve().polyline(0.1);
        assert!(
            points.first().unwrap().distance(*points.last().unwrap()) < 1e-7,
            "the loop is open"
        );
        assert!((polyline_length(&points) - circumference).abs() < 0.01 * circumference);
    }

    #[test]
    fn a_sphere_is_one_face() {
        let elements = ShapeElements::new(&Shape::sphere(1.0).build());
        assert_eq!(counts(&elements), (1, 0, 0, 1), "the poles and the seam are no elements");
    }

    /// A cylinder's rims start where its seam meets them: those vertices are no
    /// elements, and each rim is one edge though both of its faces list it.
    #[test]
    fn a_cylinder_is_three_faces_and_two_rims() {
        let elements = ShapeElements::new(&Shape::cylinder_radius_height(1.0, 2.0));
        assert_eq!(counts(&elements), (3, 2, 0, 1));
    }

    #[test]
    fn a_box_numbers_each_face_edge_and_corner_once() {
        let cube = Shape::cube(1.0);
        let elements = ShapeElements::new(&cube);
        assert_eq!(counts(&elements), (6, 12, 8, 0));

        for (index, element) in elements.edges().iter().enumerate() {
            let [piece] = element.pieces() else { panic!("edge {index} is split") };
            assert!(element.curve().is_same(piece), "edge {index} is its own curve");
        }
        // Every occurrence of an edge finds its one element.
        for edge in cube.edges() {
            let index = elements.edge_index(&edge).expect("every box edge is an element");
            assert!(elements.edges()[index].pieces()[0].is_same(&edge));
        }
        for (index, face) in elements.faces().iter().enumerate() {
            assert_eq!(elements.face_index(face), Some(index));
        }
        for (index, vertex) in elements.vertices().iter().enumerate() {
            assert_eq!(elements.vertex_index(vertex), Some(index));
        }
    }

    /// A V-shaped notch cut across a cylinder's top, its point on the seam: the
    /// notch's sides meet there at a corner, which stays a vertex.
    #[test]
    fn a_corner_on_a_seam_stays_a_vertex() {
        let cylinder = Shape::cylinder_radius_height(1.0, 2.0);
        let v = Wire::from_ordered_points([
            dvec3(-2.0, 0.0, 1.5),
            dvec3(-2.0, 2.0, 3.5),
            dvec3(-2.0, -2.0, 3.5),
        ])
        .unwrap();
        let notch = Shape::from(Face::from_wire(&v).unwrap().extrude(dvec3(4.0, 0.0, 0.0)));
        let notched = cylinder.subtract(&notch).unwrap().shape;
        let elements = ShapeElements::new(&notched);

        // The seam runs along +X, so the notch's point on the cylinder's surface
        // there is on the seam.
        let point = dvec3(1.0, 0.0, 1.5);
        let ends_at_point =
            |s: &Edge| s.start_point().distance(point).min(s.end_point().distance(point)) < 1e-7;
        assert!(elements.seams().iter().any(ends_at_point), "the seam misses the notch");
        assert!(
            elements.vertices().iter().any(|v| v.point().distance(point) < 1e-7),
            "the notch's point is hidden"
        );
        for element in elements.edges() {
            assert_eq!(element.pieces().len(), 1, "nothing on the notch was split");
        }
    }

    #[test]
    fn an_index_past_the_elements_is_an_error() {
        let elements = ShapeElements::new(&Shape::cube(1.0));
        assert!(elements.face(5).is_ok());
        assert!(matches!(elements.face(6), Err(Error::NoSuchElement("face", 6))));
        assert!(matches!(elements.edge(12), Err(Error::NoSuchElement("edge", 12))));
        assert!(matches!(elements.vertex(8), Err(Error::NoSuchElement("vertex", 8))));
    }

    #[test]
    fn a_deep_copy_numbers_the_copy_as_the_original() {
        let original = ShapeElements::new(&ball_bitten_across_its_seam());
        let copy = original.deep_copy();
        assert_eq!(counts(&copy), counts(&original));

        for (index, face) in copy.faces().iter().enumerate() {
            assert_eq!(copy.face_index(face), Some(index));
            assert!(!face.is_same(&original.faces()[index]), "face {index} is shared, not copied");
            assert!((face.surface_area() - original.faces()[index].surface_area()).abs() < 1e-9);
        }
        let (rim, original_rim) = (&copy.edges()[0], &original.edges()[0]);
        for (piece, original_piece) in rim.pieces().iter().zip(original_rim.pieces()) {
            assert_eq!(copy.edge_index(piece), Some(0));
            assert_eq!(
                copy.edge_index(original_piece),
                None,
                "the original's edge is not the copy's"
            );
        }
        assert_eq!(rim.curve().edge_type(), EdgeType::Circle);
    }

    #[test]
    fn arcs_of_one_circle_join_into_it() {
        let halves = [
            Edge::arc(dvec3(1.0, 0.0, 0.0), dvec3(0.0, 1.0, 0.0), dvec3(-1.0, 0.0, 0.0)).unwrap(),
            Edge::arc(dvec3(-1.0, 0.0, 0.0), dvec3(0.0, -1.0, 0.0), dvec3(1.0, 0.0, 0.0)).unwrap(),
        ];
        let circle = join(&halves).expect("the halves join");
        assert_eq!(circle.edge_type(), EdgeType::Circle);
        assert!((circle.length() - 2.0 * std::f64::consts::PI).abs() < 1e-9);
    }

    /// A line running smoothly into an arc is two curves: they don't join.
    #[test]
    fn a_line_and_a_tangent_arc_do_not_join() {
        let line = Edge::segment(dvec3(-1.0, 0.0, 0.0), DVec3::ZERO).unwrap();
        let arc = Edge::arc(DVec3::ZERO, dvec3(1.0, 1.0, 0.0), dvec3(0.0, 2.0, 0.0)).unwrap();
        assert!(join(&[line, arc]).is_none());
    }

    /// The edge of `elements`' shape through `point`.
    fn edge_at(elements: &ShapeElements, point: DVec3) -> Edge {
        elements
            .shape()
            .edges()
            .find(|edge| edge.midpoint().distance(point) < 1e-9)
            .expect("an edge runs through the point")
    }

    /// The circular edge of `elements`' shape at height `z`.
    fn circle_at(elements: &ShapeElements, z: f64) -> Edge {
        elements
            .shape()
            .edges()
            .find(|edge| edge.edge_type() == EdgeType::Circle && (edge.start_point().z - z).abs() < 1e-9)
            .expect("a circle at that height")
    }

    #[test]
    fn adjacent_faces_are_the_faces_an_edge_bounds() {
        let elements = ShapeElements::new(&Shape::box_centered(2.0, 2.0, 2.0));
        for edge in elements.shape().edges() {
            let faces = elements.adjacent_faces(&edge);
            assert_eq!(faces.len(), 2);
            for &face in faces {
                assert!(elements.faces()[face].edges().any(|bounding| bounding.is_same(&edge)));
            }
        }

        let cylinder = ShapeElements::new(&Shape::cylinder_radius_height(1.0, 2.0));
        let [seam] = cylinder.seams() else { panic!("a cylinder has one seam") };
        assert_eq!(cylinder.adjacent_faces(seam).len(), 1);

        let elsewhere = Edge::segment(DVec3::ZERO, DVec3::X).unwrap();
        assert!(elements.adjacent_faces(&elsewhere).is_empty());
    }

    #[test]
    fn a_box_edge_is_an_outside_right_angle_corner() {
        let elements = ShapeElements::new(&Shape::box_centered(2.0, 2.0, 2.0));
        let edge = edge_at(&elements, dvec3(0.0, -1.0, 1.0));
        let corner = elements.corner(&edge).expect("an edge between two faces");

        assert!(corner.point.distance(dvec3(0.0, -1.0, 1.0)) < 1e-9);
        assert!((corner.tangent.x.abs() - 1.0).abs() < 1e-12);
        assert!((corner.normal_angle() - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        for (face, normal) in corner.faces.iter().zip(corner.normals) {
            let expected = elements.faces()[*face].normal_at_center().unwrap();
            assert!((normal - expected).length() < 1e-12, "the normals are the faces' own, outward");
        }
        assert_eq!(corner.convex, Some(true));
    }

    /// Where a wall stands on a slab, the edge along its foot is an inside
    /// corner.
    #[test]
    fn the_foot_of_a_wall_is_an_inside_corner() {
        let slab = Shape::box_from_corners(DVec3::ZERO, dvec3(2.0, 2.0, 1.0));
        let wall = Shape::box_from_corners(dvec3(0.0, 1.0, 1.0), dvec3(2.0, 2.0, 2.0));
        let step = ShapeElements::new(&slab.union(&wall).unwrap().shape.clean().unwrap());
        let corner = step.corner(&edge_at(&step, dvec3(1.0, 1.0, 1.0))).expect("an edge between two faces");
        assert_eq!(corner.convex, Some(false));
    }

    /// The corner test works across curved faces: a rod's rim, a bore's rim
    /// and the foot of a boss.
    #[test]
    fn curved_corners_tell_outside_from_inside() {
        let rod = ShapeElements::new(&Shape::cylinder_radius_height(1.0, 2.0));
        assert_eq!(rod.corner(&circle_at(&rod, 2.0)).and_then(|corner| corner.convex), Some(true));

        let block = Shape::box_from_corners(dvec3(-2.0, -2.0, 0.0), dvec3(2.0, 2.0, 2.0));
        let bore = Shape::cylinder(dvec3(0.0, 0.0, -1.0), 1.0, DVec3::Z, 4.0);
        let bored = ShapeElements::new(&block.subtract(&bore).unwrap().shape);
        assert_eq!(bored.corner(&circle_at(&bored, 2.0)).and_then(|corner| corner.convex), Some(true));

        let boss = Shape::cylinder(dvec3(0.0, 0.0, 2.0), 1.0, DVec3::Z, 1.0);
        let bossed = ShapeElements::new(&block.union(&boss).unwrap().shape.clean().unwrap());
        assert_eq!(bossed.corner(&circle_at(&bossed, 2.0)).and_then(|corner| corner.convex), Some(false));
    }

    #[test]
    fn a_sheet_edge_has_no_corner() {
        let sheet: Shape = Face::from_wire(&Wire::rect(2.0, 2.0).unwrap()).unwrap().into();
        let elements = ShapeElements::new(&sheet);
        let edge = elements.shape().edges().next().unwrap();
        assert_eq!(elements.corner(&edge), None);
    }
}
