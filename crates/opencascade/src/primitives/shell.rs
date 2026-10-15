use cxx::UniquePtr;
use opencascade_sys::ffi;

use crate::primitives::{Edge, Face, Shape, Wire};
use crate::Error;

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

/// Faces sewn together wherever their edges meet within a tolerance
/// (`BRepBuilderAPI_Sewing`), kept to trace each input to what it became.
pub struct Sewing {
    inner: UniquePtr<ffi::BRepBuilderAPI_Sewing>,
}

impl Sewing {
    /// Sews the faces of `shapes` together wherever their edges lie within
    /// `tolerance` of each other.
    pub fn new<T: AsRef<Shape>>(shapes: impl IntoIterator<Item = T>, tolerance: f64) -> Result<Self, Error> {
        let mut inner = ffi::BRepBuilderAPI_Sewing_ctor(tolerance, true, true, true, false);
        for shape in shapes {
            inner.pin_mut().Add(&shape.as_ref().inner);
        }
        inner
            .pin_mut()
            .Perform(&ffi::Message_ProgressRange_ctor())
            .map_err(|e| Error::SewingFailed(e.what().to_string()))?;
        if ffi::TopoDS_Shape_IsNull(inner.SewedShape()) {
            return Err(Error::SewingFailed("nothing was sewn".to_string()));
        }
        Ok(Self { inner })
    }

    /// The sewn faces: a shell or a lone face, or a compound of one such per
    /// group of faces that touch.
    pub fn shape(&self) -> Shape {
        Shape::from_shape(self.inner.SewedShape())
    }

    /// The edges of the result that bound a single face.
    pub fn free_edges(&self) -> Vec<Edge> {
        (1..=self.inner.NbFreeEdges()).map(|index| Edge::from_edge(self.inner.FreeEdge(index))).collect()
    }

    /// What `sub_shape`, part of an input, became in the result: itself if
    /// sewing left it alone.
    pub fn sewn(&self, sub_shape: &Shape) -> Shape {
        let sewn = ffi::BRepBuilderAPI_Sewing_ModifiedSubShape(&self.inner, &sub_shape.inner);
        Shape::from_shape(&sewn)
    }
}

#[cfg(test)]
mod tests {
    use super::{Sewing, Shell};
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

    /// Each face of `shape` copied on its own, sharing no edge with the others.
    fn loose_faces(shape: &Shape) -> Vec<Shape> {
        shape.faces().map(|face| Shape::from(face).deep_copy()).collect()
    }

    #[test]
    fn loose_faces_of_a_cube_sew_into_a_closed_shell() {
        let sewing = Sewing::new(loose_faces(&Shape::cube(2.0)), 1e-6).expect("sewing succeeds");

        let shell = sewing.shape();
        assert_eq!(shell.shape_type(), ShapeType::Shell);
        assert!(shell.is_closed());
        assert!(sewing.free_edges().is_empty());
        let solid = shell.to_solid().expect("a closed shell bounds a solid");
        assert_eq!(solid.shape_type(), ShapeType::Solid);
        assert!((solid.volume() - 8.0).abs() < 1e-9, "got {}", solid.volume());
    }

    /// The side of a cylinder sews to itself along its seam as well as to its
    /// caps.
    #[test]
    fn a_cylinder_sews_shut_across_its_seam() {
        let sewing =
            Sewing::new(loose_faces(&Shape::cylinder_radius_height(1.0, 2.0)), 1e-6).expect("sewing succeeds");

        let solid = sewing.shape().to_solid().expect("the cylinder closes");
        let expected = 2.0 * std::f64::consts::PI;
        assert!((solid.volume() - expected).abs() < 1e-6, "got {}", solid.volume());
    }

    /// Two opposite sides of a cube touch nowhere: each stays a face of its
    /// own, every edge free.
    #[test]
    fn faces_apart_stay_apart_with_their_edges_free() {
        let cube = Shape::cube(2.0);
        let sides: Vec<_> = loose_faces(&cube)
            .into_iter()
            .filter(|face| (face.faces().next().unwrap().center_of_mass().x - 1.0).abs() > 0.5)
            .collect();
        assert_eq!(sides.len(), 2);

        let sewing = Sewing::new(&sides, 1e-6).expect("sewing succeeds");
        let sewn = sewing.shape();
        assert_eq!(sewn.shape_type(), ShapeType::Compound);
        assert_eq!(sewn.sub_shapes().count(), 2);
        assert_eq!(sewing.free_edges().len(), 8);
    }

    #[test]
    fn sewing_traces_each_face_to_its_sewn_self() {
        let faces = loose_faces(&Shape::cube(2.0));
        let sewing = Sewing::new(&faces, 1e-6).expect("sewing succeeds");

        let shell = sewing.shape();
        for face in &faces {
            let sewn = sewing.sewn(face);
            assert!(shell.faces().any(|f| Shape::from(f).is_same(&sewn)));
        }
    }

    /// A box missing its top sews into an open shell rimmed by four free
    /// edges, which bounds no solid.
    #[test]
    fn an_open_shell_is_not_closed() {
        let faces: Vec<_> = loose_faces(&Shape::cube(2.0))
            .into_iter()
            .filter(|face| face.faces().next().unwrap().center_of_mass().z < 1.5)
            .collect();
        assert_eq!(faces.len(), 5);

        let sewing = Sewing::new(&faces, 1e-6).expect("sewing succeeds");
        let shell = sewing.shape();
        assert_eq!(shell.shape_type(), ShapeType::Shell);
        assert!(!shell.is_closed());
        assert_eq!(sewing.free_edges().len(), 4);
        assert!(matches!(shell.to_solid(), Err(crate::Error::NotAClosedShell)));
    }
}
