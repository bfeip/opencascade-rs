use crate::{
    primitives::{boolean_shape, BooleanShape, Compound, Edge, Face, Wire},
    Error,
};
use cxx::UniquePtr;
use glam::{dvec3, DVec3};
use opencascade_sys::ffi;

pub struct Solid {
    pub(crate) inner: UniquePtr<ffi::TopoDS_Solid>,
}

impl AsRef<Solid> for Solid {
    fn as_ref(&self) -> &Solid {
        self
    }
}

impl Solid {
    pub(crate) fn from_solid(solid: &ffi::TopoDS_Solid) -> Self {
        let inner = ffi::TopoDS_Solid_to_owned(solid);

        Self { inner }
    }

    // TODO(bschwind) - Do some cool stuff from this link:
    // https://neweopencascade.wordpress.com/2018/10/17/lets-talk-about-fillets/
    // Key takeaway: Use the `SectionEdges` function to retrieve edges that were
    // the result of combining two shapes.
    #[must_use]
    pub fn fillet_edge(&self, radius: f64, edge: &Edge) -> Compound {
        let inner_shape = ffi::cast_solid_to_shape(&self.inner);

        let mut make_fillet = ffi::BRepFilletAPI_MakeFillet_ctor(inner_shape);
        make_fillet.pin_mut().add_edge(radius, &edge.inner);

        let filleted_shape = make_fillet.pin_mut().Shape();

        let compound = ffi::TopoDS_cast_to_compound(filleted_shape);

        Compound::from_compound(compound)
    }

    pub fn loft<T: AsRef<Wire>>(wires: impl IntoIterator<Item = T>) -> Self {
        let is_solid = true;
        let mut make_loft = ffi::BRepOffsetAPI_ThruSections_ctor(is_solid);

        for wire in wires.into_iter() {
            make_loft.pin_mut().AddWire(&wire.as_ref().inner);
        }

        // Set to CheckCompatibility to `true` to avoid twisted results.
        make_loft.pin_mut().CheckCompatibility(true);

        let shape = make_loft.pin_mut().Shape();
        let solid = ffi::TopoDS_cast_to_solid(shape);

        Self::from_solid(solid)
    }

    pub fn subtract(&self, other: &Solid) -> Result<BooleanShape, Error> {
        boolean_shape::cut(
            ffi::cast_solid_to_shape(&self.inner),
            ffi::cast_solid_to_shape(&other.inner),
            0.0,
        )
    }

    pub fn union(&self, other: &Solid) -> Result<BooleanShape, Error> {
        boolean_shape::fuse(
            ffi::cast_solid_to_shape(&self.inner),
            ffi::cast_solid_to_shape(&other.inner),
            0.0,
        )
    }

    pub fn intersect(&self, other: &Solid) -> Result<BooleanShape, Error> {
        boolean_shape::common(
            ffi::cast_solid_to_shape(&self.inner),
            ffi::cast_solid_to_shape(&other.inner),
            0.0,
        )
    }

    /// Purposefully underpowered for now, this simply takes a list of points,
    /// creates a face out of them, and then extrudes it by h in the positive Z
    /// direction.
    pub fn extrude_polygon(
        points: impl IntoIterator<Item = DVec3>,
        h: f64,
    ) -> Result<Solid, Error> {
        let wire = Wire::from_ordered_points(points)?;
        Ok(Face::from_wire(&wire)?.extrude(dvec3(0.0, 0.0, h)))
    }
}
