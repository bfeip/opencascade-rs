use cxx::UniquePtr;
use opencascade_sys::ffi;

use crate::primitives::{Face, Shape, ShapeType};

/// Sub-shape history of a completed operation: which input sub-shapes were
/// modified into, generated in, or removed from the result. Detached from the
/// algorithm that produced it (refcounted), so it can be stored long after the
/// operation ran.
///
/// Only vertices, edges, faces, and solids are tracked; queries about other
/// shape kinds answer "no relation". A sub-shape with no recorded relation and
/// not removed passed through the operation unchanged.
pub struct ShapeHistory {
    pub(crate) inner: UniquePtr<ffi::HandleBRepTools_History>,
}

impl ShapeHistory {
    /// An empty history: no relations, nothing removed.
    pub fn new() -> Self {
        Self { inner: ffi::BRepTools_History_ctor() }
    }

    pub(crate) fn from_handle(inner: UniquePtr<ffi::HandleBRepTools_History>) -> Self {
        Self { inner }
    }

    /// Result shapes modified from `shape`.
    pub fn modified(&self, shape: &Shape) -> Vec<Shape> {
        list_to_shapes(ffi::BRepTools_History_modified(&self.inner, &shape.inner))
    }

    /// Result shapes generated from `shape`.
    pub fn generated(&self, shape: &Shape) -> Vec<Shape> {
        list_to_shapes(ffi::BRepTools_History_generated(&self.inner, &shape.inner))
    }

    /// Whether `shape` was removed by the operation (no image in the result).
    pub fn is_removed(&self, shape: &Shape) -> bool {
        ffi::BRepTools_History_is_removed(&self.inner, &shape.inner)
    }

    /// Result faces modified from `face`.
    pub fn modified_faces(&self, face: &Face) -> Vec<Face> {
        let list = ffi::BRepTools_History_modified(&self.inner, ffi::cast_face_to_shape(&face.inner));
        list_to_faces(list)
    }

    /// Result faces generated from `face`.
    pub fn generated_faces(&self, face: &Face) -> Vec<Face> {
        let list =
            ffi::BRepTools_History_generated(&self.inner, ffi::cast_face_to_shape(&face.inner));
        list_to_faces(list)
    }

    /// Chain `next` after this history: if this maps A→B and `next` maps B→C,
    /// this becomes A→C.
    pub fn merge(&mut self, next: &ShapeHistory) {
        ffi::BRepTools_History_merge(self.inner.pin_mut(), &next.inner);
    }

    /// The face of `input` whose image in the result is `result_face`.
    /// 
    /// The inverse of [`Self::modified_faces`].
    pub fn source_face(&self, input: &Shape, result_face: &Face) -> Option<Face> {
        let target = ffi::cast_face_to_shape(&result_face.inner);
        for face in input.faces() {
            if face.is_same(result_face) {
                return Some(face);
            }
            let face_shape = ffi::cast_face_to_shape(&face.inner);
            let modified = ffi::shape_list_to_vector(ffi::BRepTools_History_modified(&self.inner, face_shape));
            if modified.iter().any(|image| image.IsSame(target)) {
                return Some(face);
            }
            let generated = ffi::shape_list_to_vector(ffi::BRepTools_History_generated(&self.inner, face_shape));
            if generated.iter().any(|image| image.IsSame(target)) {
                return Some(face);
            }
        }
        None
    }
}

impl Default for ShapeHistory {
    fn default() -> Self {
        Self::new()
    }
}

fn list_to_shapes(list: &ffi::TopTools_ListOfShape) -> Vec<Shape> {
    ffi::shape_list_to_vector(list).iter().map(Shape::from_shape).collect()
}

fn list_to_faces(list: &ffi::TopTools_ListOfShape) -> Vec<Face> {
    ffi::shape_list_to_vector(list)
        .iter()
        .filter(|shape| ShapeType::from(shape.ShapeType()) == ShapeType::Face)
        .map(|shape| Face::from_face(ffi::TopoDS_cast_to_face(shape)))
        .collect()
}

#[cfg(test)]
mod tests {
    use glam::dvec3;

    use crate::primitives::{Face, IntoShape, Shape};

    /// The face of `shape` with the largest area — a cylinder's lateral wall.
    fn widest_face(shape: &Shape) -> Face {
        shape
            .faces()
            .max_by(|a, b| a.surface_area().total_cmp(&b.surface_area()))
            .expect("shape has faces")
    }

    /// The face of `shape` whose center has the largest y.
    fn top_face(shape: &Shape) -> Face {
        shape
            .faces()
            .max_by(|a, b| a.center_of_mass().y.total_cmp(&b.center_of_mass().y))
            .expect("shape has faces")
    }

    #[test]
    fn boolean_history_maps_modified_faces() {
        let cube = Shape::cube(2.0);
        let drill = Shape::cylinder(dvec3(1.0, -1.0, 1.0), 0.5, dvec3(0.0, 1.0, 0.0), 4.0);
        let result = cube.subtract(&drill).expect("cut succeeds");

        // The drilled-through top face keeps exactly one image: itself with a hole.
        let images = result.history.modified_faces(&top_face(&cube));
        assert_eq!(images.len(), 1, "the drilled top face must have exactly one image");

        // The hole wall comes from the drill's lateral face; the inverse query
        // must round-trip back to it.
        let side = widest_face(&drill);
        let walls = result.history.modified_faces(&side);
        assert!(!walls.is_empty(), "the drill's lateral face must leave an image in the cut");
        let back = result.history.source_face(&drill, &walls[0]).expect("source face found");
        assert!(back.is_same(&side));
    }

    #[test]
    fn fused_away_face_reports_removed() {
        let cube = Shape::cube(2.0);
        let inner = Shape::box_from_corners(dvec3(0.5, 0.5, 0.5), dvec3(1.5, 1.5, 1.5));
        let result = cube.union(&inner).expect("fuse succeeds");

        let swallowed = inner.faces().next().expect("box has faces");
        assert!(
            result.history.is_removed(&swallowed.into_shape()),
            "a face of a fully-enclosed fused solid must be removed"
        );
    }
}
