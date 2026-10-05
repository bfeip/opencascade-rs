use cxx::UniquePtr;
use opencascade_sys::ffi;

use crate::primitives::{Face, Wire};

pub struct Shell {
    pub(crate) inner: UniquePtr<ffi::TopoDS_Shell>,
}

impl AsRef<Shell> for Shell {
    fn as_ref(&self) -> &Shell {
        self
    }
}

impl Shell {
    pub(crate) fn from_shell(shell: &ffi::TopoDS_Shell) -> Self {
        let inner = ffi::TopoDS_Shell_to_owned(shell);

        Self { inner }
    }

    /// A shell of `faces`, joined wherever they share edges.
    pub fn from_faces<T: AsRef<Face>>(faces: impl IntoIterator<Item = T>) -> Self {
        let mut shell = ffi::TopoDS_Shell_ctor();
        let builder = ffi::BRep_Builder_ctor();
        let builder = ffi::BRep_Builder_upcast_to_topods_builder(&builder);
        builder.MakeShell(shell.pin_mut());
        let mut shell_shape = ffi::TopoDS_Shell_as_shape(shell);

        for face in faces.into_iter() {
            builder.Add(shell_shape.pin_mut(), ffi::cast_face_to_shape(&face.as_ref().inner));
        }

        let shell = ffi::TopoDS_cast_to_shell(&shell_shape);
        Self::from_shell(shell)
    }

    pub fn loft<T: AsRef<Wire>>(wires: impl IntoIterator<Item = T>) -> Self {
        let is_solid = false;
        let mut make_loft = ffi::BRepOffsetAPI_ThruSections_ctor(is_solid);

        for wire in wires.into_iter() {
            make_loft.pin_mut().AddWire(&wire.as_ref().inner);
        }

        // Set CheckCompatibility to `true` to avoid twisted results.
        make_loft.pin_mut().CheckCompatibility(true);

        let shape = make_loft.pin_mut().Shape();
        let shell = ffi::TopoDS_cast_to_shell(shape);

        Self::from_shell(shell)
    }
}

#[cfg(test)]
mod tests {
    use super::Shell;
    use crate::primitives::{Shape, ShapeType};

    /// Two faces of a box make one shell, sharing the edge between them.
    #[test]
    fn faces_sharing_an_edge_make_one_shell() {
        let cube = Shape::box_centered(2.0, 2.0, 2.0);
        let edge = cube.edges().next().unwrap();

        let shell = Shape::from(Shell::from_faces(cube.adjacent_faces(&edge)));
        assert_eq!(shell.shape_type(), ShapeType::Shell);
        assert_eq!(shell.faces().count(), 2);
        assert_eq!(shell.unique_sub_shape_count(ShapeType::Edge), 7);
    }
}
