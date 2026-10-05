use crate::{
    angle::Angle,
    law_function::law_function_from_graph,
    make_pipe_shell::make_pipe_shell_with_law_function,
    primitives::{
        make_axis_1, make_point, make_vec, EdgeIterator, JoinType, Shape, Solid, Surface, Wire,
    },
    workplane::Workplane,
    Error,
};
use cxx::UniquePtr;
use glam::{dvec3, DVec3};
use opencascade_sys::ffi;

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum FaceType {
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Torus,
    BezierSurface,
    BSplineSurface,
    SurfaceOfRevolution,
    SurfaceOfExtrusion,
    OffsetSurface,
    OtherSurface,
}

impl From<ffi::GeomAbs_SurfaceType> for FaceType {
    fn from(surface_type: ffi::GeomAbs_SurfaceType) -> Self {
        match surface_type {
            ffi::GeomAbs_SurfaceType::GeomAbs_Plane => Self::Plane,
            ffi::GeomAbs_SurfaceType::GeomAbs_Cylinder => Self::Cylinder,
            ffi::GeomAbs_SurfaceType::GeomAbs_Cone => Self::Cone,
            ffi::GeomAbs_SurfaceType::GeomAbs_Sphere => Self::Sphere,
            ffi::GeomAbs_SurfaceType::GeomAbs_Torus => Self::Torus,
            ffi::GeomAbs_SurfaceType::GeomAbs_BezierSurface => Self::BezierSurface,
            ffi::GeomAbs_SurfaceType::GeomAbs_BSplineSurface => Self::BSplineSurface,
            ffi::GeomAbs_SurfaceType::GeomAbs_SurfaceOfRevolution => Self::SurfaceOfRevolution,
            ffi::GeomAbs_SurfaceType::GeomAbs_SurfaceOfExtrusion => Self::SurfaceOfExtrusion,
            ffi::GeomAbs_SurfaceType::GeomAbs_OffsetSurface => Self::OffsetSurface,
            ffi::GeomAbs_SurfaceType::GeomAbs_OtherSurface => Self::OtherSurface,
            ffi::GeomAbs_SurfaceType { repr } => panic!("Unexpected surface type: {repr}"),
        }
    }
}

pub struct Face {
    pub(crate) inner: UniquePtr<ffi::TopoDS_Face>,
}

impl AsRef<Face> for Face {
    fn as_ref(&self) -> &Face {
        self
    }
}

impl Face {
    pub(crate) fn from_face(face: &ffi::TopoDS_Face) -> Self {
        let inner = ffi::TopoDS_Face_to_owned(face);

        Self { inner }
    }

    fn from_make_face(make_face: UniquePtr<ffi::BRepBuilderAPI_MakeFace>) -> Result<Self, Error> {
        if !make_face.IsDone() {
            return Err(Error::FaceFailed(make_face.Error().into()));
        }

        Ok(Self::from_face(make_face.Face()))
    }

    pub fn from_wire(wire: &Wire) -> Result<Self, Error> {
        let only_plane = false;
        let make_face = ffi::BRepBuilderAPI_MakeFace_wire(&wire.inner, only_plane);

        Self::from_make_face(make_face)
    }

    pub fn from_surface(surface: &Surface) -> Result<Self, Error> {
        const EDGE_TOLERANCE: f64 = 0.0001;

        let make_face = ffi::BRepBuilderAPI_MakeFace_surface(&surface.inner, EDGE_TOLERANCE);

        Self::from_make_face(make_face)
    }

    /// Topological identity (same underlying TShape and location, ignoring
    /// orientation) — the `TopoDS_Shape::IsSame` test.
    pub fn is_same(&self, other: &Face) -> bool {
        ffi::cast_face_to_shape(&self.inner).IsSame(ffi::cast_face_to_shape(&other.inner))
    }

    #[must_use]
    pub fn extrude(&self, dir: DVec3) -> Solid {
        self.extrude_with_caps(dir).0
    }

    /// [`extrude`](Self::extrude), also returning the prism's two caps: the
    /// face where the sweep starts, and its copy where the sweep ends.
    #[must_use]
    pub fn extrude_with_caps(&self, dir: DVec3) -> (Solid, Face, Face) {
        let prism_vec = make_vec(dir);

        let copy = false;
        let canonize = true;

        let inner_shape = ffi::cast_face_to_shape(&self.inner);
        let mut make_solid =
            ffi::BRepPrimAPI_MakePrism_ctor(inner_shape, &prism_vec, copy, canonize);
        let solid = Solid::from_solid(ffi::TopoDS_cast_to_solid(make_solid.pin_mut().Shape()));
        let first = ffi::BRepPrimAPI_MakePrism_FirstShape(make_solid.pin_mut());
        let last = ffi::BRepPrimAPI_MakePrism_LastShape(make_solid.pin_mut());

        (
            solid,
            Face::from_face(ffi::TopoDS_cast_to_face(&first)),
            Face::from_face(ffi::TopoDS_cast_to_face(&last)),
        )
    }

    #[must_use]
    pub fn extrude_to_face(&self, shape_with_face: &Shape, face: &Face) -> Shape {
        let profile_base = &self.inner;
        let sketch_base = ffi::TopoDS_Face_ctor();
        let angle = 0.0;
        let fuse = 1; // 0 = subtractive, 1 = additive
        let modify = false;

        let mut make_prism = ffi::BRepFeat_MakeDPrism_ctor(
            &shape_with_face.inner,
            profile_base,
            &sketch_base,
            angle,
            fuse,
            modify,
        );

        let until_face = ffi::cast_face_to_shape(&face.inner);
        make_prism.pin_mut().perform_until_face(until_face);

        Shape::from_shape(make_prism.pin_mut().Shape())
    }

    #[must_use]
    pub fn subtractive_extrude(&self, shape_with_face: &Shape, height: f64) -> Shape {
        let profile_base = &self.inner;
        let sketch_base = ffi::TopoDS_Face_ctor();
        let angle = 0.0;
        let fuse = 0; // 0 = subtractive, 1 = additive
        let modify = false;

        let mut make_prism = ffi::BRepFeat_MakeDPrism_ctor(
            &shape_with_face.inner,
            profile_base,
            &sketch_base,
            angle,
            fuse,
            modify,
        );

        make_prism.pin_mut().perform_with_height(height);

        Shape::from_shape(make_prism.pin_mut().Shape())
    }

    #[must_use]
    pub fn revolve(&self, origin: DVec3, axis: DVec3, angle: Option<Angle>) -> Solid {
        let revol_vec = make_axis_1(origin, axis);

        let angle = angle.map(Angle::radians).unwrap_or(std::f64::consts::PI * 2.0);
        let copy = false;

        let inner_shape = ffi::cast_face_to_shape(&self.inner);
        let mut make_solid = ffi::BRepPrimAPI_MakeRevol_ctor(inner_shape, &revol_vec, angle, copy);
        let revolved_shape = make_solid.pin_mut().Shape();
        let solid = ffi::TopoDS_cast_to_solid(revolved_shape);

        Solid::from_solid(solid)
    }

    /// Fillets the face edges by a given radius at each vertex
    #[must_use]
    pub fn fillet(&self, radius: f64) -> Self {
        let mut make_fillet = ffi::BRepFilletAPI_MakeFillet2d_ctor(&self.inner);

        let face_shape = ffi::cast_face_to_shape(&self.inner);

        // We use a shape map here to avoid duplicates.
        let mut shape_map = ffi::new_indexed_map_of_shape();
        ffi::map_shapes(face_shape, ffi::TopAbs_ShapeEnum::TopAbs_VERTEX, shape_map.pin_mut());

        for i in 1..=shape_map.Extent() {
            let vertex = ffi::TopoDS_cast_to_vertex(shape_map.FindKey(i));
            ffi::BRepFilletAPI_MakeFillet2d_add_fillet(make_fillet.pin_mut(), vertex, radius);
        }

        make_fillet.pin_mut().Build(&ffi::Message_ProgressRange_ctor());

        let result_shape = make_fillet.pin_mut().Shape();
        let result_face = ffi::TopoDS_cast_to_face(result_shape);

        Self::from_face(result_face)
    }

    /// Chamfer the wire edges at each vertex by a given distance
    #[must_use]
    pub fn chamfer(&self, distance_1: f64) -> Self {
        // TODO - Support asymmetric chamfers.
        let distance_2 = distance_1;

        let face_shape = ffi::cast_face_to_shape(&self.inner);

        let mut make_fillet = ffi::BRepFilletAPI_MakeFillet2d_ctor(&self.inner);

        let mut vertex_map = ffi::new_indexed_map_of_shape();
        ffi::map_shapes(face_shape, ffi::TopAbs_ShapeEnum::TopAbs_VERTEX, vertex_map.pin_mut());

        // Get map of vertices to edges so we can find the edges connected to each vertex.
        let mut data_map = ffi::new_indexed_data_map_of_shape_list_of_shape();
        ffi::map_shapes_and_ancestors(
            face_shape,
            ffi::TopAbs_ShapeEnum::TopAbs_VERTEX,
            ffi::TopAbs_ShapeEnum::TopAbs_EDGE,
            data_map.pin_mut(),
        );

        // Chamfer at vertex of all edges.
        for i in 1..=vertex_map.Extent() {
            let edges = ffi::shape_list_to_vector(data_map.FindFromIndex(i));
            let edge_1 = edges.get(0).expect("Vertex has no edges");
            let edge_2 = edges.get(1).expect("Vertex has only one edge");
            ffi::BRepFilletAPI_MakeFillet2d_add_chamfer(
                make_fillet.pin_mut(),
                ffi::TopoDS_cast_to_edge(edge_1),
                ffi::TopoDS_cast_to_edge(edge_2),
                distance_1,
                distance_2,
            );
        }

        let filleted_shape = make_fillet.pin_mut().Shape();
        let result_face = ffi::TopoDS_cast_to_face(filleted_shape);

        Self::from_face(result_face)
    }

    /// Offset the face by a given distance and join settings
    #[must_use]
    pub fn offset(&self, distance: f64, join_type: JoinType) -> Self {
        let mut make_offset =
            ffi::BRepOffsetAPI_MakeOffset_face_ctor(&self.inner, join_type.into());
        make_offset.pin_mut().Perform(distance, 0.0);

        let offset_shape = make_offset.pin_mut().Shape();
        let result_wire = ffi::TopoDS_cast_to_wire(offset_shape);
        let wire = Wire::from_wire(result_wire);

        wire.to_face().unwrap()
    }

    /// Sweep the face along a path to produce a solid
    #[must_use]
    pub fn sweep_along(&self, path: &Wire) -> Solid {
        let profile_shape = ffi::cast_face_to_shape(&self.inner);
        let mut make_pipe = ffi::BRepOffsetAPI_MakePipe_ctor(&path.inner, profile_shape);

        let pipe_shape = make_pipe.pin_mut().Shape();
        let result_solid = ffi::TopoDS_cast_to_solid(pipe_shape);

        Solid::from_solid(result_solid)
    }

    /// Sweep the face along a path, modulated by a function, to produce a solid
    #[must_use]
    pub fn sweep_along_with_radius_values(
        &self,
        path: &Wire,
        radius_values: impl IntoIterator<Item = (f64, f64)>,
    ) -> Solid {
        let law_function = law_function_from_graph(radius_values);
        let law_handle = ffi::Law_Function_to_handle(law_function);

        let profile_wire = ffi::outer_wire(&self.inner);
        let mut make_pipe_shell =
            make_pipe_shell_with_law_function(&profile_wire, &path.inner, &law_handle);

        make_pipe_shell.pin_mut().Build(&ffi::Message_ProgressRange_ctor());
        make_pipe_shell.pin_mut().MakeSolid();
        let pipe_shape = make_pipe_shell.pin_mut().Shape();
        let result_solid = ffi::TopoDS_cast_to_solid(pipe_shape);

        Solid::from_solid(result_solid)
    }

    pub fn edges(&self) -> EdgeIterator {
        let explorer = ffi::TopExp_Explorer_ctor(
            ffi::cast_face_to_shape(&self.inner),
            ffi::TopAbs_ShapeEnum::TopAbs_EDGE,
        );

        EdgeIterator { explorer }
    }

    pub fn center_of_mass(&self) -> DVec3 {
        let mut props = ffi::GProp_GProps_ctor();

        let inner_shape = ffi::cast_face_to_shape(&self.inner);
        ffi::BRepGProp_SurfaceProperties(inner_shape, props.pin_mut());

        let center = ffi::GProp_GProps_CentreOfMass(&props);

        dvec3(center.X(), center.Y(), center.Z())
    }

    pub fn normal_at(&self, pos: DVec3) -> Result<DVec3, Error> {
        let surface = ffi::BRep_Tool_Surface(&self.inner);
        let projector = ffi::GeomAPI_ProjectPointOnSurf_ctor(&make_point(pos), &surface);
        let mut u: f64 = 0.0;
        let mut v: f64 = 0.0;

        projector
            .LowerDistanceParameters(&mut u, &mut v)
            .map_err(|e| Error::SurfaceProjectionFailed(e.what().to_string()))?;

        let mut p = ffi::new_point(0.0, 0.0, 0.0);
        let mut normal = ffi::new_vec(0.0, 1.0, 0.0);

        let face = ffi::BRepGProp_Face_ctor(&self.inner);
        face.Normal(u, v, p.pin_mut(), normal.pin_mut());

        Ok(dvec3(normal.X(), normal.Y(), normal.Z()))
    }

    /// The point halfway through the face's parameter ranges. Unlike the
    /// centre of mass, it lies on the surface, curved or closed (though it may
    /// fall in a hole or notch of a trimmed face).
    pub fn midpoint(&self) -> DVec3 {
        let face = ffi::BRepGProp_Face_ctor(&self.inner);
        let (mut u1, mut u2, mut v1, mut v2) = (0.0, 0.0, 0.0, 0.0);
        face.Bounds(&mut u1, &mut u2, &mut v1, &mut v2);

        let mut point = ffi::new_point(0.0, 0.0, 0.0);
        let mut normal = ffi::new_vec(0.0, 1.0, 0.0);
        face.Normal(0.5 * (u1 + u2), 0.5 * (v1 + v2), point.pin_mut(), normal.pin_mut());

        dvec3(point.X(), point.Y(), point.Z())
    }

    pub fn normal_at_center(&self) -> Result<DVec3, Error> {
        let center = self.center_of_mass();
        self.normal_at(center)
    }

    pub fn workplane(&self) -> Workplane {
        const NORMAL_DIFF_TOLERANCE: f64 = 0.0001;

        let center = self.center_of_mass();
        let normal = self.normal_at(center).expect("face has no well-defined normal at its center");
        let mut x_dir = dvec3(0.0, 0.0, 1.0).cross(normal);

        if x_dir.length() < NORMAL_DIFF_TOLERANCE {
            // The normal of this face is too close to the same direction
            // as the global Z axis. Use the global X axis for X instead.
            x_dir = dvec3(1.0, 0.0, 0.0);
        }

        let mut workplane = Workplane::new(x_dir, normal);
        workplane.set_translation(center);
        workplane
    }

    pub fn union(&self, other: &Face) -> CompoundFace {
        let inner_shape = ffi::cast_face_to_shape(&self.inner);
        let other_inner_shape = ffi::cast_face_to_shape(&other.inner);

        let mut fuse_operation = ffi::BRepAlgoAPI_Fuse_ctor(inner_shape, other_inner_shape);

        let fuse_shape = fuse_operation.pin_mut().Shape();

        let compound = ffi::TopoDS_cast_to_compound(fuse_shape);

        CompoundFace::from_compound(compound)
    }

    #[must_use]
    pub fn intersect(&self, other: &Face) -> CompoundFace {
        let inner_shape = ffi::cast_face_to_shape(&self.inner);
        let other_inner_shape = ffi::cast_face_to_shape(&other.inner);

        let mut common_operation = ffi::BRepAlgoAPI_Common_ctor(inner_shape, other_inner_shape);

        let common_shape = common_operation.pin_mut().Shape();

        let compound = ffi::TopoDS_cast_to_compound(common_shape);

        CompoundFace::from_compound(compound)
    }

    pub fn subtract(&self, other: &Face) -> CompoundFace {
        let inner_shape = ffi::cast_face_to_shape(&self.inner);
        let other_inner_shape = ffi::cast_face_to_shape(&other.inner);

        let mut fuse_operation = ffi::BRepAlgoAPI_Cut_ctor(inner_shape, other_inner_shape);

        let cut_shape = fuse_operation.pin_mut().Shape();

        let compound = ffi::TopoDS_cast_to_compound(cut_shape);

        CompoundFace::from_compound(compound)
    }

    pub fn surface_area(&self) -> f64 {
        let mut props = ffi::GProp_GProps_ctor();

        let inner_shape = ffi::cast_face_to_shape(&self.inner);
        ffi::BRepGProp_SurfaceProperties(inner_shape, props.pin_mut());

        // Returns surface area, obviously.
        props.Mass()
    }

    pub fn orientation(&self) -> FaceOrientation {
        FaceOrientation::from(self.inner.Orientation())
    }

    pub fn face_type(&self) -> FaceType {
        FaceType::from(self.adaptor().GetType())
    }

    /// Radius of a cylindrical face.
    pub fn cylinder_radius(&self) -> Option<f64> {
        let surface = self.adaptor_of(FaceType::Cylinder)?;
        Some(ffi::BRepAdaptor_Surface_Cylinder(&surface).Radius())
    }

    /// Reference radius and semi-angle (radians) of a conical face.
    pub fn cone_dimensions(&self) -> Option<(f64, f64)> {
        let surface = self.adaptor_of(FaceType::Cone)?;
        let cone = ffi::BRepAdaptor_Surface_Cone(&surface);
        Some((cone.RefRadius(), cone.SemiAngle()))
    }

    /// Radius of a spherical face.
    pub fn sphere_radius(&self) -> Option<f64> {
        let surface = self.adaptor_of(FaceType::Sphere)?;
        Some(ffi::BRepAdaptor_Surface_Sphere(&surface).Radius())
    }

    /// Major and minor radii of a toroidal face.
    pub fn torus_radii(&self) -> Option<(f64, f64)> {
        let surface = self.adaptor_of(FaceType::Torus)?;
        let torus = ffi::BRepAdaptor_Surface_Torus(&surface);
        Some((torus.MajorRadius(), torus.MinorRadius()))
    }

    fn adaptor(&self) -> UniquePtr<ffi::BRepAdaptor_Surface> {
        ffi::BRepAdaptor_Surface_ctor(&self.inner, true)
    }

    /// The face's surface adaptor, if the surface is of type `ty`. The
    /// adaptor's analytic accessors throw on any other type.
    fn adaptor_of(&self, ty: FaceType) -> Option<UniquePtr<ffi::BRepAdaptor_Surface>> {
        let surface = self.adaptor();
        (FaceType::from(surface.GetType()) == ty).then_some(surface)
    }

    #[must_use]
    pub fn outer_wire(&self) -> Wire {
        let inner = ffi::outer_wire(&self.inner);

        Wire { inner }
    }
}

pub struct CompoundFace {
    inner: UniquePtr<ffi::TopoDS_Compound>,
}

impl AsRef<CompoundFace> for CompoundFace {
    fn as_ref(&self) -> &CompoundFace {
        self
    }
}

impl From<Face> for CompoundFace {
    fn from(face: Face) -> Self {
        let face = ffi::cast_face_to_shape(&face.inner);
        let mut compound = ffi::TopoDS_Compound_ctor();
        let brep_builder = ffi::BRep_Builder_ctor();
        let topo_builder = ffi::BRep_Builder_upcast_to_topods_builder(&brep_builder);
        topo_builder.MakeCompound(compound.pin_mut());
        let mut compound_shape = ffi::TopoDS_Compound_as_shape(compound);
        topo_builder.Add(compound_shape.pin_mut(), face);
        Self::from_compound(ffi::TopoDS_cast_to_compound(&compound_shape))
    }
}

impl CompoundFace {
    pub(crate) fn from_compound(compound: &ffi::TopoDS_Compound) -> Self {
        let inner = ffi::TopoDS_Compound_to_owned(compound);

        Self { inner }
    }

    pub fn clean(&self) -> Result<Self, Error> {
        let shape = ffi::cast_compound_to_shape(&self.inner);
        let shape = Shape::from_shape(shape).clean()?;

        let compound = ffi::TopoDS_cast_to_compound(&shape.inner);

        Ok(Self::from_compound(compound))
    }

    #[must_use]
    pub fn extrude(&self, dir: DVec3) -> Shape {
        let prism_vec = make_vec(dir);

        let copy = false;
        let canonize = true;

        let inner_shape = ffi::cast_compound_to_shape(&self.inner);

        let mut make_solid =
            ffi::BRepPrimAPI_MakePrism_ctor(inner_shape, &prism_vec, copy, canonize);
        let extruded_shape = make_solid.pin_mut().Shape();

        Shape::from_shape(extruded_shape)
    }

    #[must_use]
    pub fn revolve(&self, origin: DVec3, axis: DVec3, angle: Option<Angle>) -> Shape {
        let revol_axis = make_axis_1(origin, axis);

        let angle = angle.map(Angle::radians).unwrap_or(std::f64::consts::PI * 2.0);
        let copy = false;

        let inner_shape = ffi::cast_compound_to_shape(&self.inner);

        let mut make_solid = ffi::BRepPrimAPI_MakeRevol_ctor(inner_shape, &revol_axis, angle, copy);
        let revolved_shape = make_solid.pin_mut().Shape();

        Shape::from_shape(revolved_shape)
    }

    #[must_use]
    pub fn union(&self, other: &CompoundFace) -> CompoundFace {
        let inner_shape = ffi::cast_compound_to_shape(&self.inner);
        let other_inner_shape = ffi::cast_compound_to_shape(&other.inner);

        let mut fuse_operation = ffi::BRepAlgoAPI_Fuse_ctor(inner_shape, other_inner_shape);

        let fuse_shape = fuse_operation.pin_mut().Shape();

        let compound = ffi::TopoDS_cast_to_compound(fuse_shape);

        CompoundFace::from_compound(compound)
    }

    #[must_use]
    pub fn intersect(&self, other: &CompoundFace) -> CompoundFace {
        let inner_shape = ffi::cast_compound_to_shape(&self.inner);
        let other_inner_shape = ffi::cast_compound_to_shape(&other.inner);

        let mut common_operation = ffi::BRepAlgoAPI_Common_ctor(inner_shape, other_inner_shape);

        let common_shape = common_operation.pin_mut().Shape();

        let compound = ffi::TopoDS_cast_to_compound(common_shape);

        CompoundFace::from_compound(compound)
    }

    #[must_use]
    pub fn subtract(&self, other: &CompoundFace) -> CompoundFace {
        let inner_shape = ffi::cast_compound_to_shape(&self.inner);
        let other_inner_shape = ffi::cast_compound_to_shape(&other.inner);

        let mut fuse_operation = ffi::BRepAlgoAPI_Cut_ctor(inner_shape, other_inner_shape);

        let cut_shape = fuse_operation.pin_mut().Shape();

        let compound = ffi::TopoDS_cast_to_compound(cut_shape);

        CompoundFace::from_compound(compound)
    }

    pub fn set_global_translation(&mut self, translation: DVec3) {
        let shape = ffi::cast_compound_to_shape(&self.inner);
        let mut shape = Shape::from_shape(shape);

        shape.set_global_translation(translation);

        let compound = ffi::TopoDS_cast_to_compound(&shape.inner);
        *self = Self::from_compound(compound);
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum FaceOrientation {
    Forward,
    Reversed,
    Internal,
    External,
}

impl From<ffi::TopAbs_Orientation> for FaceOrientation {
    fn from(orientation: ffi::TopAbs_Orientation) -> Self {
        match orientation {
            ffi::TopAbs_Orientation::TopAbs_FORWARD => Self::Forward,
            ffi::TopAbs_Orientation::TopAbs_REVERSED => Self::Reversed,
            ffi::TopAbs_Orientation::TopAbs_INTERNAL => Self::Internal,
            ffi::TopAbs_Orientation::TopAbs_EXTERNAL => Self::External,
            ffi::TopAbs_Orientation { repr } => {
                panic!("TopAbs_Orientation had an unrepresentable value: {repr}")
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn analytic_faces_report_their_type_and_dimensions() {
        let cylinder = Shape::cylinder_radius_height(5.0, 3.0);
        let side = cylinder.faces().find(|f| f.face_type() == FaceType::Cylinder).unwrap();
        assert!((side.cylinder_radius().unwrap() - 5.0).abs() < 1e-9);
        assert_eq!(side.sphere_radius(), None, "a cylinder is not a sphere");
        assert_eq!(cylinder.faces().filter(|f| f.face_type() == FaceType::Plane).count(), 2);

        let sphere = Shape::sphere(4.0).build();
        let face = sphere.faces().next().unwrap();
        assert_eq!(face.face_type(), FaceType::Sphere);
        assert!((face.sphere_radius().unwrap() - 4.0).abs() < 1e-9);

        let cone = Shape::cone().bottom_radius(2.0).top_radius(1.0).height(1.0).build();
        let side = cone.faces().find(|f| f.face_type() == FaceType::Cone).unwrap();
        let (ref_radius, semi_angle) = side.cone_dimensions().unwrap();
        assert!((ref_radius - 2.0).abs() < 1e-9, "ref radius {ref_radius}");
        assert!((semi_angle.abs() - std::f64::consts::FRAC_PI_4).abs() < 1e-9, "{semi_angle}");

        let torus = Shape::torus().radius_1(20.0).radius_2(10.0).build();
        let face = torus.faces().next().unwrap();
        let (major, minor) = face.torus_radii().unwrap();
        assert!((major - 20.0).abs() < 1e-9 && (minor - 10.0).abs() < 1e-9);
    }

    /// A full cylinder's centre of mass sits on its axis, off the surface; its
    /// midpoint lies on the surface, halfway up, where the normal is defined.
    #[test]
    fn a_face_midpoint_lies_on_the_surface() {
        let cylinder = Shape::cylinder_radius_height(5.0, 3.0);
        let side = cylinder.faces().find(|f| f.face_type() == FaceType::Cylinder).unwrap();
        let middle = side.midpoint();
        assert!((middle.truncate().length() - 5.0).abs() < 1e-9, "{middle}");
        assert!((middle.z - 1.5).abs() < 1e-9, "{middle}");
        let normal = side.normal_at(middle).unwrap().normalize();
        assert!((normal - middle.truncate().extend(0.0) / 5.0).length() < 1e-9, "{normal}");

        let rect = Workplane::xy().rect(7.0, 5.0).to_face().unwrap();
        assert!(rect.midpoint().distance(rect.center_of_mass()) < 1e-9);
    }

    #[test]
    fn test_add() {
        let face = Workplane::xy().rect(7.0, 5.0).to_face().unwrap();
        assert!(
            (face.surface_area() - 35.0).abs() <= 0.00001,
            "Expected surface_area() to be ~35.0, was actually {}",
            face.surface_area()
        );
    }

    #[test]
    fn extrude_with_caps_returns_the_profile_and_its_swept_copy() {
        let face = Workplane::xy().rect(2.0, 3.0).to_face().unwrap();
        let (solid, first, last) = face.extrude_with_caps(dvec3(0.0, 0.0, 4.0));

        assert!(first.is_same(&face), "the prism starts on the profile itself");
        let offset = last.center_of_mass() - face.center_of_mass();
        assert!((offset - dvec3(0.0, 0.0, 4.0)).length() < 1e-9, "offset {offset}");

        let solid: Shape = solid.into();
        for cap in [&first, &last] {
            assert!(solid.faces().any(|f| f.is_same(cap)), "a cap is not a face of the prism");
        }
    }
}
