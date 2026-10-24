use cxx::UniquePtr;
use opencascade_sys::ffi;

use crate::primitives::{Shape, SubShape};

/// A deep copy of a shape ([`Shape::deep_copy`]) that can find the copy of each
/// of the original's faces, edges and vertices.
pub struct ShapeCopy {
    copier: UniquePtr<ffi::BRepBuilderAPI_Copy>,
    shape: Shape,
}

impl ShapeCopy {
    pub fn new(shape: &Shape) -> Self {
        let mut copier = ffi::BRepBuilderAPI_Copy_ctor(&shape.inner);
        let shape = Shape::from_shape(copier.pin_mut().Shape());
        Self { copier, shape }
    }

    /// The copy.
    pub fn shape(&self) -> &Shape {
        &self.shape
    }

    /// The copy of `original`, or `None` if it is no sub-shape of the shape
    /// copied.
    pub fn copy_of<T: SubShape>(&self, original: &T) -> Option<T> {
        let copied = ffi::BRepBuilderAPI_Copy_ModifiedShape(&self.copier, original.as_shape()).ok()?;
        Some(T::from_shape(&copied))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::{Edge, Face};

    #[test]
    fn each_sub_shape_has_a_copy_of_its_own() {
        let cube = Shape::cube(1.0);
        let copy = ShapeCopy::new(&cube);

        for face in cube.faces() {
            let copied = copy.copy_of(&face).expect("every face is copied");
            assert!(!copied.is_same(&face), "the face is shared, not copied");
            assert!(copy.shape().faces().any(|f| f.is_same(&copied)), "the copy is the copy's own");
            assert!((copied.surface_area() - face.surface_area()).abs() < 1e-12);
        }
        for edge in cube.edges() {
            let copied = copy.copy_of(&edge).expect("every edge is copied");
            assert!(copy.shape().edges().any(|e| e.is_same(&copied)));
        }
    }

    #[test]
    fn a_foreign_sub_shape_has_no_copy() {
        let copy = ShapeCopy::new(&Shape::cube(1.0));
        let elsewhere: Face = Shape::cube(2.0).faces().next().unwrap();
        assert!(copy.copy_of(&elsewhere).is_none());
        let line = Edge::segment(glam::DVec3::ZERO, glam::DVec3::X).unwrap();
        assert!(copy.copy_of(&line).is_none());
    }
}
