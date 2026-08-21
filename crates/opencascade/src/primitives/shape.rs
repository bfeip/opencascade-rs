use crate::{
    history::ShapeHistory,
    mesh::{FaceRange, Mesh, Mesher},
    primitives::{
        boolean_shape, make_axis_1, make_axis_2, make_dir, make_point, make_point2d, make_vec,
        BooleanShape, Compound, Edge, EdgeIterator, Face, FaceIterator, ShapeType, Shell, Solid,
        SubShapeIterator, Vertex, VertexIterator, Wire, WireIterator,
    },
    Error,
};
use cxx::UniquePtr;
use glam::{dvec2, dvec3, DVec3};
use opencascade_sys::ffi;
use std::path::Path;

pub struct Shape {
    pub(crate) inner: UniquePtr<ffi::TopoDS_Shape>,
}

impl AsRef<Shape> for Shape {
    fn as_ref(&self) -> &Shape {
        self
    }
}

impl From<Vertex> for Shape {
    fn from(vertex: Vertex) -> Self {
        let shape = ffi::cast_vertex_to_shape(&vertex.inner);

        Self::from_shape(shape)
    }
}

impl From<&Vertex> for Shape {
    fn from(vertex: &Vertex) -> Self {
        let shape = ffi::cast_vertex_to_shape(&vertex.inner);

        Self::from_shape(shape)
    }
}

impl From<Edge> for Shape {
    fn from(edge: Edge) -> Self {
        let shape = ffi::cast_edge_to_shape(&edge.inner);

        Self::from_shape(shape)
    }
}

impl From<&Edge> for Shape {
    fn from(edge: &Edge) -> Self {
        let shape = ffi::cast_edge_to_shape(&edge.inner);

        Self::from_shape(shape)
    }
}

impl From<Wire> for Shape {
    fn from(wire: Wire) -> Self {
        let shape = ffi::cast_wire_to_shape(&wire.inner);

        Self::from_shape(shape)
    }
}

impl From<&Wire> for Shape {
    fn from(wire: &Wire) -> Self {
        let shape = ffi::cast_wire_to_shape(&wire.inner);

        Self::from_shape(shape)
    }
}

impl From<Face> for Shape {
    fn from(face: Face) -> Self {
        let shape = ffi::cast_face_to_shape(&face.inner);

        Self::from_shape(shape)
    }
}

impl From<&Face> for Shape {
    fn from(face: &Face) -> Self {
        let shape = ffi::cast_face_to_shape(&face.inner);

        Self::from_shape(shape)
    }
}

impl From<Shell> for Shape {
    fn from(shell: Shell) -> Self {
        let shape = ffi::cast_shell_to_shape(&shell.inner);

        Self::from_shape(shape)
    }
}

impl From<&Shell> for Shape {
    fn from(shell: &Shell) -> Self {
        let shape = ffi::cast_shell_to_shape(&shell.inner);

        Self::from_shape(shape)
    }
}

impl From<Solid> for Shape {
    fn from(solid: Solid) -> Self {
        let shape = ffi::cast_solid_to_shape(&solid.inner);

        Self::from_shape(shape)
    }
}

impl From<&Solid> for Shape {
    fn from(solid: &Solid) -> Self {
        let shape = ffi::cast_solid_to_shape(&solid.inner);

        Self::from_shape(shape)
    }
}

impl From<Compound> for Shape {
    fn from(compound: Compound) -> Self {
        let shape = ffi::cast_compound_to_shape(&compound.inner);

        Self::from_shape(shape)
    }
}

impl From<&Compound> for Shape {
    fn from(compound: &Compound) -> Self {
        let shape = ffi::cast_compound_to_shape(&compound.inner);

        Self::from_shape(shape)
    }
}

impl From<BooleanShape> for Shape {
    fn from(boolean_shape: BooleanShape) -> Self {
        boolean_shape.shape
    }
}

pub struct SphereBuilder {
    center: DVec3,
    radius: f64,
    z_angle: f64,
    axis: DVec3,
}

impl SphereBuilder {
    pub fn build(self) -> Shape {
        let axis = make_axis_2(self.center, self.axis);
        let mut make_shere = ffi::BRepPrimAPI_MakeSphere_ctor(&axis, self.radius, self.z_angle);

        Shape::from_shape(make_shere.pin_mut().Shape())
    }

    pub fn at(mut self, center: DVec3) -> Self {
        self.center = center;
        self
    }

    /// Polar axis of the sphere's parametrization: the
    /// poles lie along it and the seam meridian in a plane containing it.
    pub fn axis(mut self, axis: DVec3) -> Self {
        self.axis = axis;
        self
    }

    pub fn z_angle(mut self, z_angle: f64) -> Self {
        self.z_angle = z_angle;
        self
    }
}

pub struct ConeBuilder {
    pos: DVec3,
    height: f64,
    bottom_radius: f64,
    top_radius: f64,
    z_angle: f64,
}

impl ConeBuilder {
    pub fn build(self) -> Shape {
        let axis = make_axis_2(self.pos, DVec3::Z);
        let mut make_cone = ffi::BRepPrimAPI_MakeCone_ctor(
            &axis,
            self.bottom_radius,
            self.top_radius,
            self.height,
            self.z_angle,
        );

        Shape::from_shape(make_cone.pin_mut().Shape())
    }

    pub fn at(mut self, pos: DVec3) -> Self {
        self.pos = pos;
        self
    }

    pub fn bottom_radius(mut self, bottom_radius: f64) -> Self {
        self.bottom_radius = bottom_radius;
        self
    }

    pub fn top_radius(mut self, top_radius: f64) -> Self {
        self.top_radius = top_radius;
        self
    }

    pub fn height(mut self, height: f64) -> Self {
        self.height = height;
        self
    }

    pub fn z_angle(mut self, z_angle: f64) -> Self {
        self.z_angle = z_angle;
        self
    }
}

pub struct TorusBuilder {
    pos: DVec3,
    z_axis: DVec3,
    radius_1: f64,
    radius_2: f64,
    angle_1: f64,
    angle_2: f64,
    z_angle: f64,
}

impl TorusBuilder {
    pub fn build(self) -> Shape {
        let axis = make_axis_2(self.pos, self.z_axis);
        let mut make_torus = ffi::BRepPrimAPI_MakeTorus_ctor(
            &axis,
            self.radius_1,
            self.radius_2,
            self.angle_1,
            self.angle_2,
            self.z_angle,
        );

        Shape::from_shape(make_torus.pin_mut().Shape())
    }

    pub fn at(mut self, pos: DVec3) -> Self {
        self.pos = pos;
        self
    }

    pub fn z_axis(mut self, z_axis: DVec3) -> Self {
        self.z_axis = z_axis;
        self
    }

    pub fn radius_1(mut self, radius_1: f64) -> Self {
        self.radius_1 = radius_1;
        self
    }

    pub fn radius_2(mut self, radius_2: f64) -> Self {
        self.radius_2 = radius_2;
        self
    }

    pub fn angle_1(mut self, angle_1: f64) -> Self {
        self.angle_1 = angle_1;
        self
    }

    pub fn angle_2(mut self, angle_2: f64) -> Self {
        self.angle_2 = angle_2;
        self
    }

    pub fn z_angle(mut self, z_angle: f64) -> Self {
        self.z_angle = z_angle;
        self
    }
}

impl Clone for Shape {
    /// OCCT shapes share underlying B-Rep data via reference counting, so cloning is cheap.
    /// I.e. this is shallow copy.
    fn clone(&self) -> Self {
        Self::from_shape(&self.inner)
    }
}

impl Shape {
    pub(crate) fn from_shape(shape: &ffi::TopoDS_Shape) -> Self {
        let inner = ffi::TopoDS_Shape_to_owned(shape);

        Self { inner }
    }

    /// Topological identity (same underlying TShape and location, ignoring
    /// orientation) — the `TopoDS_Shape::IsSame` test.
    pub fn is_same(&self, other: &Shape) -> bool {
        self.inner.IsSame(&other.inner)
    }

    /// A deep copy (`BRepBuilderAPI_Copy`): fresh TShapes sharing no B-Rep data
    /// with `self`, unlike [`clone`](Clone::clone), which is shallow. Cached
    /// triangulations are not carried over.
    #[must_use]
    pub fn deep_copy(&self) -> Shape {
        let mut copy = ffi::BRepBuilderAPI_Copy_ctor(&self.inner);
        Self::from_shape(copy.pin_mut().Shape())
    }

    /// Enclosed volume (`BRepGProp::VolumeProperties`). Zero for shapes with
    /// no solid content; negative for inside-out shells.
    pub fn volume(&self) -> f64 {
        let mut props = ffi::GProp_GProps_ctor();
        ffi::BRepGProp_VolumeProperties(&self.inner, props.pin_mut());
        props.Mass()
    }

    /// Make a shape that models empty space.
    pub fn empty() -> Self {
        // NOTE: It may seem like using `TopoDS_Shape()` directly should work,
        //       but shape operations such as union fail on actual "null shapes".

        // Construct an empty compound
        let mut compound = ffi::TopoDS_Compound_ctor();
        let builder = ffi::BRep_Builder_ctor();
        let topods_builder = ffi::BRep_Builder_upcast_to_topods_builder(&builder);
        topods_builder.MakeCompound(compound.pin_mut());

        let inner = ffi::TopoDS_Compound_as_shape(compound);

        Self { inner }
    }

    /// Make a box with one corner at corner_1, and the opposite corner
    /// at corner_2.
    pub fn box_from_corners(corner_1: DVec3, corner_2: DVec3) -> Self {
        let min_corner = corner_1.min(corner_2);
        let max_corner = corner_1.max(corner_2);

        let point = ffi::new_point(min_corner.x, min_corner.y, min_corner.z);
        let diff = max_corner - min_corner;
        let mut my_box = ffi::BRepPrimAPI_MakeBox_ctor(&point, diff.x, diff.y, diff.z);

        Self::from_shape(my_box.pin_mut().Shape())
    }

    /// Make a box with `width` (x), `depth` (y), and `height` (z)
    /// centered around the origin.
    pub fn box_centered(width: f64, depth: f64, height: f64) -> Self {
        let half_width = width / 2.0;
        let half_depth = depth / 2.0;
        let half_height = height / 2.0;

        let corner_1 = dvec3(-half_width, -half_depth, -half_height);
        let corner_2 = dvec3(half_width, half_depth, half_height);
        Self::box_from_corners(corner_1, corner_2)
    }

    /// Make a box with `width` (x), `depth` (y), and `height` (z)
    /// extending into the positive axes
    pub fn box_with_dimensions(width: f64, depth: f64, height: f64) -> Self {
        let corner_1 = DVec3::ZERO;
        let corner_2 = dvec3(width, depth, height);
        Self::box_from_corners(corner_1, corner_2)
    }

    /// Make a cube with side length of `size`
    /// extending into the positive axes
    pub fn cube(size: f64) -> Self {
        Self::box_with_dimensions(size, size, size)
    }

    /// Make a centered cube with side length of `size`
    pub fn cube_centered(size: f64) -> Self {
        Self::box_centered(size, size, size)
    }

    /// Make a cylinder with base at point `p`, radius `r`, and height `h`.
    /// Extends from `p` along axis `dir`.
    pub fn cylinder(p: DVec3, r: f64, dir: DVec3, h: f64) -> Self {
        let cylinder_coord_system = make_axis_2(p, dir);
        let mut cylinder = ffi::BRepPrimAPI_MakeCylinder_ctor(&cylinder_coord_system, r, h);

        Self::from_shape(cylinder.pin_mut().Shape())
    }

    /// Make a "default" cylinder with radius `r` and height `h`.
    /// The base is at the coordinate origin, and extends along the Z axis.
    pub fn cylinder_radius_height(r: f64, h: f64) -> Self {
        Self::cylinder(DVec3::ZERO, r, DVec3::Z, h)
    }

    /// Make a cylinder from start point `p1` and end point `p2`,
    /// with radius `r`.
    pub fn cylinder_from_points(p1: DVec3, p2: DVec3, r: f64) -> Self {
        let dir = p2 - p1;
        Self::cylinder(p1, r, dir, dir.length())
    }

    /// Make a cylinder centered at point `p`, with radius `r`, and height `h`.
    /// Extends along axis `dir`.
    pub fn cylinder_centered(p: DVec3, r: f64, dir: DVec3, h: f64) -> Self {
        let p = p - (dir.normalize() * (h / 2.0));
        Self::cylinder(p, r, dir, h)
    }

    pub fn sphere(radius: f64) -> SphereBuilder {
        SphereBuilder {
            center: DVec3::ZERO,
            radius,
            z_angle: std::f64::consts::TAU,
            axis: DVec3::Z,
        }
    }

    pub fn cone() -> ConeBuilder {
        ConeBuilder {
            pos: DVec3::ZERO,
            height: 1.0,
            bottom_radius: 1.0,
            top_radius: 0.0,
            z_angle: std::f64::consts::TAU,
        }
    }

    pub fn torus() -> TorusBuilder {
        TorusBuilder {
            pos: DVec3::ZERO,
            z_axis: DVec3::Z,
            radius_1: 20.0,
            radius_2: 10.0,
            angle_1: -std::f64::consts::PI,
            angle_2: std::f64::consts::PI,
            z_angle: std::f64::consts::TAU,
        }
    }

    pub fn shape_type(&self) -> ShapeType {
        self.inner.ShapeType().into()
    }

    /// Returns `true` if this shape contains at least one sub-shape of `ty`.
    /// A shape of type `ty` contains itself.
    pub fn contains_type(&self, ty: ShapeType) -> bool {
        ffi::TopExp_Explorer_ctor(&self.inner, ty.into()).More()
    }

    #[must_use]
    pub fn fillet_edge(&self, radius: f64, edge: &Edge) -> Self {
        self.fillet_edges(radius, [edge])
    }

    #[must_use]
    pub fn variable_fillet_edge(
        &self,
        radius_values: impl IntoIterator<Item = (f64, f64)>,
        edge: &Edge,
    ) -> Self {
        self.variable_fillet_edges(radius_values, [edge])
    }

    #[must_use]
    pub fn chamfer_edge(&self, distance: f64, edge: &Edge) -> Self {
        self.chamfer_edges(distance, [edge])
    }

    #[must_use]
    pub fn fillet_edges<T: AsRef<Edge>>(
        &self,
        radius: f64,
        edges: impl IntoIterator<Item = T>,
    ) -> Self {
        let mut make_fillet = ffi::BRepFilletAPI_MakeFillet_ctor(&self.inner);

        for edge in edges.into_iter() {
            make_fillet.pin_mut().add_edge(radius, &edge.as_ref().inner);
        }

        Self::from_shape(make_fillet.pin_mut().Shape())
    }

    #[must_use]
    pub fn variable_fillet_edges<T: AsRef<Edge>>(
        &self,
        radius_values: impl IntoIterator<Item = (f64, f64)>,
        edges: impl IntoIterator<Item = T>,
    ) -> Self {
        let radius_values: Vec<_> = radius_values.into_iter().collect();
        let mut array = ffi::TColgp_Array1OfPnt2d_ctor(1, radius_values.len() as i32);

        for (index, (t, radius)) in radius_values.into_iter().enumerate() {
            array.pin_mut().SetValue(index as i32 + 1, &make_point2d(dvec2(t, radius)));
        }

        let mut make_fillet = ffi::BRepFilletAPI_MakeFillet_ctor(&self.inner);

        for edge in edges.into_iter() {
            make_fillet.pin_mut().variable_add_edge(&array, &edge.as_ref().inner);
        }

        Self::from_shape(make_fillet.pin_mut().Shape())
    }

    #[must_use]
    pub fn chamfer_edges<T: AsRef<Edge>>(
        &self,
        distance: f64,
        edges: impl IntoIterator<Item = T>,
    ) -> Self {
        let mut make_chamfer = ffi::BRepFilletAPI_MakeChamfer_ctor(&self.inner);

        for edge in edges.into_iter() {
            make_chamfer.pin_mut().add_edge(distance, &edge.as_ref().inner);
        }

        Self::from_shape(make_chamfer.pin_mut().Shape())
    }

    /// Performs fillet of `radius` on all edges of the shape
    #[must_use]
    pub fn fillet(&self, radius: f64) -> Self {
        self.fillet_edges(radius, self.edges())
    }

    /// Performs chamfer of `distance` on all edges of the shape
    #[must_use]
    pub fn chamfer(&self, distance: f64) -> Self {
        self.chamfer_edges(distance, self.edges())
    }

    pub fn subtract(&self, other: &Shape) -> Result<BooleanShape, Error> {
        boolean_shape::cut(&self.inner, &other.inner, 0.0)
    }

    /// [`subtract`](Self::subtract) with an additional intersection tolerance:
    /// geometry closer than `fuzz` is treated as coincident.
    /// 
    /// Use when the inputs' placement precision is coarser than OCCT's default
    /// 1e-7 (e.g. f32-derived coordinates), where near-coincident seams/faces
    /// otherwise derail the classification.
    pub fn subtract_with_fuzz(&self, other: &Shape, fuzz: f64) -> Result<BooleanShape, Error> {
        boolean_shape::cut(&self.inner, &other.inner, fuzz)
    }

    pub fn read_step_from_file(path: impl AsRef<Path>) -> Result<Self, Error> {
        let mut reader = ffi::STEPControl_Reader_ctor();

        let status =
            ffi::read_step_from_file(reader.pin_mut(), path.as_ref().to_string_lossy().to_string());

        if status != ffi::IFSelect_ReturnStatus::IFSelect_RetDone {
            return Err(Error::StepReadFailed);
        }

        reader.pin_mut().TransferRoots(&ffi::Message_ProgressRange_ctor());

        let inner = ffi::one_shape_step(&reader);

        Ok(Self { inner })
    }

    pub fn read_step_from_str(s: &str) -> Result<Self, Error> {
        let mut reader = ffi::STEPControl_Reader_ctor();

        let status = ffi::read_step_from_str(reader.pin_mut(), s);

        if status != ffi::IFSelect_ReturnStatus::IFSelect_RetDone {
            return Err(Error::StepReadFailed);
        }

        reader.pin_mut().TransferRoots(&ffi::Message_ProgressRange_ctor());

        let inner = ffi::one_shape_step(&reader);

        Ok(Self { inner })
    }

    pub fn write_step_to_file(&self, path: impl AsRef<Path>) -> Result<(), Error> {
        let mut writer = ffi::STEPControl_Writer_ctor();

        let status = ffi::transfer_shape(writer.pin_mut(), &self.inner);

        if status != ffi::IFSelect_ReturnStatus::IFSelect_RetDone {
            return Err(Error::StepWriteFailed);
        }

        let status =
            ffi::write_step_to_file(writer.pin_mut(), path.as_ref().to_string_lossy().to_string());

        if status != ffi::IFSelect_ReturnStatus::IFSelect_RetDone {
            return Err(Error::StepWriteFailed);
        }

        Ok(())
    }

    pub fn write_step_to_string(&self) -> Result<String, Error> {
        let mut writer = ffi::STEPControl_Writer_ctor();

        let status = ffi::transfer_shape(writer.pin_mut(), &self.inner);

        if status != ffi::IFSelect_ReturnStatus::IFSelect_RetDone {
            return Err(Error::StepWriteFailed);
        }

        Ok(ffi::write_step_to_string(writer.pin_mut()))
    }

    pub fn read_iges_from_file(path: impl AsRef<Path>) -> Result<Self, Error> {
        let mut reader = ffi::IGESControl_Reader_ctor();

        let status =
            ffi::read_iges_from_file(reader.pin_mut(), path.as_ref().to_string_lossy().to_string());

        reader.pin_mut().TransferRoots(&ffi::Message_ProgressRange_ctor());

        if status != ffi::IFSelect_ReturnStatus::IFSelect_RetDone {
            return Err(Error::IgesReadFailed);
        }

        let inner = ffi::one_shape_iges(&reader);

        Ok(Self { inner })
    }

    pub fn read_iges_from_str(s: &str) -> Result<Self, Error> {
        let mut reader = ffi::IGESControl_Reader_ctor();

        let status = ffi::read_iges_from_str(reader.pin_mut(), s);

        reader.pin_mut().TransferRoots(&ffi::Message_ProgressRange_ctor());

        if status != ffi::IFSelect_ReturnStatus::IFSelect_RetDone {
            return Err(Error::IgesReadFailed);
        }

        let inner = ffi::one_shape_iges(&reader);

        Ok(Self { inner })
    }

    pub fn write_iges_to_file(&self, path: impl AsRef<Path>) -> Result<(), Error> {
        let mut writer = ffi::IGESControl_Writer_ctor();

        let success = ffi::add_shape(writer.pin_mut(), &self.inner);

        if !success {
            return Err(Error::IgesWriteFailed);
        }

        ffi::compute_model(writer.pin_mut());
        let success =
            ffi::write_iges_to_file(writer.pin_mut(), path.as_ref().to_string_lossy().to_string());

        if success {
            Ok(())
        } else {
            Err(Error::IgesWriteFailed)
        }
    }

    pub fn write_iges_to_string(&self) -> Result<String, Error> {
        let mut writer = ffi::IGESControl_Writer_ctor();

        let success = ffi::add_shape(writer.pin_mut(), &self.inner);

        if !success {
            return Err(Error::IgesWriteFailed);
        }

        ffi::compute_model(writer.pin_mut());
        Ok(ffi::write_iges_to_string(writer.pin_mut()))
    }

    pub fn write_brep_text(&self, path: impl AsRef<Path>) -> Result<(), Error> {
        let success =
            ffi::write_brep_text(&self.inner, path.as_ref().to_string_lossy().to_string());

        if success {
            Ok(())
        } else {
            Err(Error::BrepWriteFailed)
        }
    }

    pub fn read_brep_text(path: impl AsRef<Path>) -> Result<Self, Error> {
        let inner = ffi::read_brep_text(path.as_ref().to_string_lossy().to_string());

        if inner.is_null() {
            Err(Error::BrepReadFailed)
        } else {
            Ok(Self { inner })
        }
    }

    pub fn write_brep_bin(&self, path: impl AsRef<Path>) -> Result<(), Error> {
        let success = ffi::write_brep_bin(&self.inner, path.as_ref().to_string_lossy().to_string());

        if success {
            Ok(())
        } else {
            Err(Error::BrepWriteFailed)
        }
    }

    pub fn read_brep_bin(path: impl AsRef<Path>) -> Result<Self, Error> {
        let inner = ffi::read_brep_bin(path.as_ref().to_string_lossy().to_string());

        if inner.is_null() {
            Err(Error::BrepReadFailed)
        } else {
            Ok(Self { inner })
        }
    }

    pub fn union(&self, other: &Shape) -> Result<BooleanShape, Error> {
        boolean_shape::fuse(&self.inner, &other.inner, 0.0)
    }

    /// [`union`](Self::union) with an additional intersection tolerance; see
    /// [`subtract_with_fuzz`](Self::subtract_with_fuzz).
    pub fn union_with_fuzz(&self, other: &Shape, fuzz: f64) -> Result<BooleanShape, Error> {
        boolean_shape::fuse(&self.inner, &other.inner, fuzz)
    }

    pub fn intersect(&self, other: &Shape) -> Result<BooleanShape, Error> {
        boolean_shape::common(&self.inner, &other.inner, 0.0)
    }

    /// [`intersect`](Self::intersect) with an additional intersection
    /// tolerance; see [`subtract_with_fuzz`](Self::subtract_with_fuzz).
    pub fn intersect_with_fuzz(&self, other: &Shape, fuzz: f64) -> Result<BooleanShape, Error> {
        boolean_shape::common(&self.inner, &other.inner, fuzz)
    }

    pub fn write_stl<P: AsRef<Path>>(&self, path: P) -> Result<(), Error> {
        self.write_stl_with_tolerance(path, 0.001)
    }

    pub fn write_stl_with_tolerance<P: AsRef<Path>>(
        &self,
        path: P,
        triangulation_tolerance: f64,
    ) -> Result<(), Error> {
        let mut stl_writer = ffi::StlAPI_Writer_ctor();
        let mesher = Mesher::try_new(self, triangulation_tolerance)?;
        let success = ffi::write_stl(
            stl_writer.pin_mut(),
            mesher.inner.Shape(),
            path.as_ref().to_string_lossy().to_string(),
        );

        if success {
            Ok(())
        } else {
            Err(Error::StlWriteFailed)
        }
    }

    /// Merges faces that lie on the same surface and edges that lie on the same curve.
    pub fn clean(&self) -> Result<Self, Error> {
        let mut upgrader = ffi::ShapeUpgrade_UnifySameDomain_ctor(&self.inner, true, true, true);
        upgrader.pin_mut().AllowInternalEdges(false);
        upgrader.pin_mut().Build().map_err(|e| Error::CleanFailed(e.what().to_string()))?;

        Ok(Self::from_shape(upgrader.Shape()))
    }

    pub fn set_global_translation(&mut self, translation: DVec3) {
        let mut transform = ffi::new_transform();
        let translation_vec = make_vec(translation);
        transform.pin_mut().set_translation_vec(&translation_vec);

        let location = ffi::TopLoc_Location_from_transform(&transform);

        self.inner.pin_mut().set_global_translation(&location, false);
    }

    pub fn mesh(&self) -> Result<Mesh, Error> {
        self.mesh_with_tolerance(0.01)
    }

    pub fn mesh_with_tolerance(&self, triangulation_tolerance: f64) -> Result<Mesh, Error> {
        let mesher = Mesher::try_new(self, triangulation_tolerance)?;
        mesher.mesh()
    }

    /// Tessellates the shape and returns both the flat mesh and per-face index ranges.
    ///
    /// Each [`FaceRange`] records the start triangle and count for one B-Rep face.
    pub fn mesh_with_tolerance_and_ranges(
        &self,
        triangulation_tolerance: f64,
    ) -> Result<(Mesh, Vec<FaceRange>), Error> {
        let mesher = Mesher::try_new(self, triangulation_tolerance)?;
        mesher.mesh_with_face_ranges()
    }

    pub fn edges(&self) -> EdgeIterator {
        let explorer = ffi::TopExp_Explorer_ctor(&self.inner, ffi::TopAbs_ShapeEnum::TopAbs_EDGE);
        EdgeIterator { explorer }
    }

    pub fn faces(&self) -> FaceIterator {
        let explorer = ffi::TopExp_Explorer_ctor(&self.inner, ffi::TopAbs_ShapeEnum::TopAbs_FACE);
        FaceIterator { explorer }
    }

    /// Edges that occur as a seam on at least one face of this shape.
    pub fn seam_edges(&self) -> Vec<Edge> {
        let mut seams: Vec<Edge> = Vec::new();
        for face in self.faces() {
            for edge in face.edges() {
                if edge.is_seam(&face) && !seams.iter().any(|s| s.is_same(&edge)) {
                    seams.push(edge);
                }
            }
        }
        seams
    }

    pub fn vertices(&self) -> VertexIterator {
        let explorer = ffi::TopExp_Explorer_ctor(&self.inner, ffi::TopAbs_ShapeEnum::TopAbs_VERTEX);
        VertexIterator { explorer }
    }

    pub fn wires(&self) -> WireIterator {
        let explorer = ffi::TopExp_Explorer_ctor(&self.inner, ffi::TopAbs_ShapeEnum::TopAbs_WIRE);
        WireIterator { explorer }
    }

    /// Returns `true` if this shape has no underlying topology (null `myTShape`).
    /// Calling most methods on a null shape is undefined behavior.
    pub fn is_null(&self) -> bool {
        ffi::TopoDS_Shape_IsNull(&self.inner)
    }

    /// Iterates the direct children of this shape.
    /// Only meaningful when `shape_type() == ShapeType::Compound`.
    /// This does not recurse — it yields the immediate sub-shapes.
    pub fn sub_shapes(&self) -> SubShapeIterator {
        SubShapeIterator { inner: ffi::TopoDS_Iterator_ctor(&self.inner) }
    }

    /// Returns the 4×4 row-major transform matrix from this shape's `TopLoc_Location`.
    /// `mat[row][col]` with rows and cols 0-indexed.
    pub fn location_as_matrix(&self) -> [[f64; 4]; 4] {
        let loc = ffi::TopoDS_Shape_Location(&self.inner);
        if ffi::TopLoc_Location_IsIdentity(&loc) {
            return [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ];
        }
        let trsf = ffi::TopLoc_Location_Transformation(&loc);
        let mut mat = [[0.0f64; 4]; 4];
        for row in 0..3 {
            for col in 0..4 {
                mat[row][col] = trsf.Value((row + 1) as i32, (col + 1) as i32);
            }
        }
        mat[3] = [0.0, 0.0, 0.0, 1.0];
        mat
    }

    /// Immutably applies `mat`, preserving surface/curve types (a plane stays
    /// a plane, a sphere a sphere).
    ///
    /// Only possible for similarity transforms (rotation + translation + uniform
    /// scale); a general affine matrix must go through [`Self::gtransform`],
    /// which converts all geometry to B-splines.
    pub fn transformed(&self, mat: [[f64; 4]; 4]) -> Result<Shape, Error> {
        Ok(self.transformed_with_history(mat)?.0)
    }

    /// Like [`Self::transformed`], additionally returning the sub-shape
    /// history (input sub-shapes → transformed sub-shapes).
    pub fn transformed_with_history(
        &self,
        mat: [[f64; 4]; 4],
    ) -> Result<(Shape, ShapeHistory), Error> {
        // Pre-validate: gp_Trsf::SetValues aborts (uncatchable across cxx) on
        // a non-similarity matrix.
        if !is_similarity(&mat) {
            return Err(Error::NotASimilarityTransform);
        }

        let mut transform = ffi::new_transform();
        #[rustfmt::skip]
        transform.pin_mut().SetValues(
            mat[0][0], mat[0][1], mat[0][2], mat[0][3],
            mat[1][0], mat[1][1], mat[1][2], mat[1][3],
            mat[2][0], mat[2][1], mat[2][2], mat[2][3],
        );

        let mut builder = ffi::BRepBuilderAPI_Transform_ctor(&self.inner, &transform, true);
        let shape = Self::from_shape(builder.pin_mut().Shape());

        let mut inputs = ffi::new_list_of_shape();
        ffi::shape_list_append_shape(inputs.pin_mut(), &self.inner);
        let history = ShapeHistory::from_handle(ffi::BRepBuilderAPI_Transform_history(
            builder.pin_mut(),
            &inputs,
        ));

        Ok((shape, history))
    }

    /// Immutably applies a general transform, returning the new transformed shape.
    /// `mat` is row-major and zero-indexed (`mat[row][col]`).
    ///
    /// This converts every surface and curve to B-splines (an OCCT
    /// `GTransform` constraint), degrading downstream modeling on the result;
    /// prefer [`Self::transformed`] whenever the matrix is a similarity.
    #[must_use]
    pub fn gtransform(&self, mat: [[f64; 4]; 4]) -> Shape {
        self.gtransform_with_history(mat).0
    }

    /// Like [`Self::gtransform`], additionally returning the sub-shape history
    /// (input sub-shapes → transformed sub-shapes).
    #[must_use]
    pub fn gtransform_with_history(&self, mat: [[f64; 4]; 4]) -> (Shape, ShapeHistory) {
        // Flatten any transforms pre-existing on the shape, they seem to be applied
        // twice, possible OCCT bug?
        let identity = ffi::new_transform();
        let mut flatten = ffi::BRepBuilderAPI_Transform_ctor(&self.inner, &identity, true);
        let flat = Self::from_shape(flatten.pin_mut().Shape());

        let mut inputs = ffi::new_list_of_shape();
        ffi::shape_list_append_shape(inputs.pin_mut(), &self.inner);
        let mut history = ShapeHistory::from_handle(ffi::BRepBuilderAPI_Transform_history(
            flatten.pin_mut(),
            &inputs,
        ));

        let mut transform = ffi::new_gp_GTrsf();
        for row in 0..3 {
            for col in 0..4 {
                transform.pin_mut().SetValue((row + 1) as i32, (col + 1) as i32, mat[row][col]);
            }
        }

        // The constructor performs the transform; `Shape()` returns the result.
        let mut builder = ffi::BRepBuilderAPI_GTransform_ctor(&flat.inner, &transform, true);
        let result = Self::from_shape(builder.pin_mut().Shape());

        // The history must chain across both stages, keyed by the flattened
        // intermediate's sub-shapes.
        let mut stage_inputs = ffi::new_list_of_shape();
        ffi::shape_list_append_shape(stage_inputs.pin_mut(), &flat.inner);
        let stage = ShapeHistory::from_handle(ffi::BRepBuilderAPI_GTransform_history(
            builder.pin_mut(),
            &stage_inputs,
        ));
        history.merge(&stage);

        (result, history)
    }

    /// Transforms `faces` (sub-shapes of this solid) and re-solves the body
    /// around them — a direct-modeling "tweak". Neighboring faces are extended
    /// and re-intersected; the rest of the boundary is preserved.
    ///
    /// `mat` is row-major and zero-indexed, and must be a similarity transform
    /// (rotation + translation + uniform scale).
    ///
    /// Errs when the matrix is not a similarity, or when the body cannot be
    /// re-solved around the moved faces (a face moved past its neighbors,
    /// tangent junctions that cannot re-intersect, ...).
    ///
    /// On a compound, the tweak is applied to the child solid owning the
    /// faces (all faces must belong to one child) and the siblings are kept.
    pub fn tweak_faces<T: AsRef<Face>>(
        &self,
        faces: impl IntoIterator<Item = T>,
        mat: [[f64; 4]; 4],
    ) -> Result<Shape, Error> {
        Ok(self.tweak_faces_with_history(faces, mat)?.0)
    }

    /// Like [`Self::tweak_faces`], additionally returning the face history of
    /// the re-solve (input faces → their images in the result; untouched faces
    /// that vanished are removed). Only faces are tracked.
    pub fn tweak_faces_with_history<T: AsRef<Face>>(
        &self,
        faces: impl IntoIterator<Item = T>,
        mat: [[f64; 4]; 4],
    ) -> Result<(Shape, ShapeHistory), Error> {
        // Pre-validate: gp_Trsf::SetValues aborts (uncatchable across cxx) on
        // a non-similarity matrix.
        if !is_similarity(&mat) {
            return Err(Error::NotASimilarityTransform);
        }

        if self.shape_type() == ShapeType::Compound {
            let faces: Vec<T> = faces.into_iter().collect();
            return self.tweak_faces_in_compound(&faces, mat);
        }

        let mut face_list = ffi::new_list_of_shape();
        for face in faces.into_iter() {
            ffi::shape_list_append_face(face_list.pin_mut(), &face.as_ref().inner);
        }

        let mut transform = ffi::new_transform();
        #[rustfmt::skip]
        transform.pin_mut().SetValues(
            mat[0][0], mat[0][1], mat[0][2], mat[0][3],
            mat[1][0], mat[1][1], mat[1][2], mat[1][3],
            mat[2][0], mat[2][1], mat[2][2], mat[2][3],
        );

        let mut history = ffi::BRepTools_History_ctor();
        let inner = ffi::shape_tweak_faces_with_history(
            &self.inner,
            &face_list,
            &transform,
            history.pin_mut(),
        )
        .map_err(|e| Error::TweakFailed(e.what().to_string()))?;
        Ok((Shape { inner }, ShapeHistory::from_handle(history)))
    }

    /// Tweak faces of one child of a compound and reassemble it with the
    /// untouched siblings. The returned history is the child re-solve's — it
    /// keys on the same face instances the compound contains.
    fn tweak_faces_in_compound<T: AsRef<Face>>(
        &self,
        faces: &[T],
        mat: [[f64; 4]; 4],
    ) -> Result<(Shape, ShapeHistory), Error> {
        let children: Vec<Shape> = self.sub_shapes().collect();

        let mut owner = None;
        for face in faces {
            let face = face.as_ref();
            let index = children
                .iter()
                .position(|child| child.faces().any(|f| f.is_same(face)))
                .ok_or_else(|| {
                    Error::TweakFailed("tweak: a face does not belong to the shape".to_string())
                })?;
            match owner {
                None => owner = Some(index),
                Some(previous) if previous != index => {
                    return Err(Error::TweakFailed(
                        "tweak: faces span multiple children of a compound".to_string(),
                    ));
                },
                Some(_) => {},
            }
        }
        let owner =
            owner.ok_or_else(|| Error::TweakFailed("tweak: no faces given".to_string()))?;

        let (tweaked, history) =
            children[owner].tweak_faces_with_history(faces.iter().map(|f| f.as_ref()), mat)?;
        let rebuilt = children
            .iter()
            .enumerate()
            .map(|(i, child)| if i == owner { tweaked.clone() } else { child.clone() });
        Ok((Compound::from_shapes(rebuilt).into(), history))
    }

    // TODO(bschwind) - Convert the return type to an iterator.
    pub fn faces_along_line(&self, line_origin: DVec3, line_dir: DVec3) -> Vec<LineFaceHitPoint> {
        let mut intersector = ffi::BRepIntCurveSurface_Inter_ctor();
        let tolerance = 0.0001;
        intersector.pin_mut().Init(
            &self.inner,
            &ffi::gp_Lin_ctor(&make_point(line_origin), &make_dir(line_dir)),
            tolerance,
        );

        let mut results = vec![];

        while intersector.More() {
            let face = ffi::BRepIntCurveSurface_Inter_face(&intersector);
            let face = Face::from_face(&face);
            let point = ffi::BRepIntCurveSurface_Inter_point(&intersector);

            results.push(LineFaceHitPoint {
                face,
                t: intersector.W(),
                u: intersector.U(),
                v: intersector.V(),
                point: dvec3(point.X(), point.Y(), point.Z()),
            });

            intersector.pin_mut().Next();
        }

        results
    }

    #[must_use]
    pub fn hollow<T: AsRef<Face>>(
        &self,
        offset: f64,
        faces_to_remove: impl IntoIterator<Item = T>,
    ) -> Self {
        let mut faces_list = ffi::new_list_of_shape();

        for face in faces_to_remove.into_iter() {
            ffi::shape_list_append_face(faces_list.pin_mut(), &face.as_ref().inner);
        }

        let mut solid_maker = ffi::BRepOffsetAPI_MakeThickSolid_ctor();
        ffi::MakeThickSolidByJoin(solid_maker.pin_mut(), &self.inner, &faces_list, offset, 0.001);

        Self::from_shape(solid_maker.pin_mut().Shape())
    }

    #[must_use]
    pub fn offset_surface(&self, offset: f64) -> Self {
        let faces_to_remove: [Face; 0] = [];
        self.hollow(offset, faces_to_remove)
    }

    /// Drill a cylindrical hole along the line defined by point `p`
    /// and direction `dir`, with `radius`.
    #[must_use]
    pub fn drill_hole(&self, p: DVec3, dir: DVec3, radius: f64) -> Self {
        let hole_axis = make_axis_1(p, dir);

        let mut make_hole = ffi::BRepFeat_MakeCylindricalHole_ctor();
        make_hole.pin_mut().Init(&self.inner, &hole_axis);

        make_hole.pin_mut().Perform(radius);
        make_hole.pin_mut().Build();

        Self::from_shape(make_hole.pin_mut().Shape())
    }
}

/// Information about a point where a line hits (i.e. intersects) a face
pub struct LineFaceHitPoint {
    /// The face that is hit
    pub face: Face,
    /// The T parameter along the line
    pub t: f64,
    /// The U parameter on the face
    pub u: f64,
    /// The V parameter on the face
    pub v: f64,
    /// The intersection point
    pub point: DVec3,
}

/// Whether `mat` is affine (bottom row `0 0 0 1`) with a 3x3 part that is a
/// rotation combined with a positive uniform scale i.e. no shear, no non-uniform
/// scale, no mirror.
fn is_similarity(mat: &[[f64; 4]; 4]) -> bool {
    const EPSILON: f64 = 1e-7;

    let affine = [0.0, 0.0, 0.0, 1.0]
        .iter()
        .zip(mat[3].iter())
        .all(|(expected, actual)| (expected - actual).abs() < EPSILON);
    if !affine {
        return false;
    }

    let col = |k: usize| DVec3::new(mat[0][k], mat[1][k], mat[2][k]);
    let (c0, c1, c2) = (col(0), col(1), col(2));
    let scale_sq = (c0.length_squared() + c1.length_squared() + c2.length_squared()) / 3.0;
    if scale_sq < EPSILON {
        return false;
    }

    let uniform = [c0, c1, c2]
        .iter()
        .all(|c| (c.length_squared() - scale_sq).abs() < EPSILON * scale_sq);
    let orthogonal = [c0.dot(c1), c1.dot(c2), c2.dot(c0)]
        .iter()
        .all(|dot| dot.abs() < EPSILON * scale_sq);
    let right_handed = c0.cross(c1).dot(c2) > 0.0;

    uniform && orthogonal && right_handed
}

pub struct ChamferMaker {
    inner: UniquePtr<ffi::BRepFilletAPI_MakeChamfer>,
}

impl ChamferMaker {
    pub fn new(shape: &Shape) -> Self {
        let make_chamfer = ffi::BRepFilletAPI_MakeChamfer_ctor(&shape.inner);

        Self { inner: make_chamfer }
    }

    pub fn add_edge(&mut self, distance: f64, edge: &Edge) {
        self.inner.pin_mut().add_edge(distance, &edge.inner);
    }

    pub fn build(mut self) -> Shape {
        Shape::from_shape(self.inner.pin_mut().Shape())
    }
}

#[cfg(test)]
mod tests {
    use super::Shape;
    use crate::primitives::{Compound, Edge, Face, ShapeType, Wire};
    use crate::Error;
    use glam::dvec3;

    fn max_y(shape: &Shape) -> f64 {
        shape
            .mesh()
            .unwrap()
            .vertices
            .iter()
            .map(|v| v.y)
            .fold(f64::NEG_INFINITY, f64::max)
    }

    #[test]
    fn contains_reports_sub_shape_presence() {
        let cube = Shape::cube(2.0);
        assert!(cube.contains_type(ShapeType::Solid));
        assert!(cube.contains_type(ShapeType::Face));
        assert!(cube.contains_type(ShapeType::Edge));

        let wire: Shape = Wire::from_ordered_points([
            dvec3(0.0, 0.0, 0.0),
            dvec3(1.0, 0.0, 0.0),
            dvec3(1.0, 0.0, 1.0),
        ])
        .unwrap()
        .into();
        assert!(!wire.contains_type(ShapeType::Solid));
        assert!(!wire.contains_type(ShapeType::Shell));
        assert!(!wire.contains_type(ShapeType::Face));
        assert!(wire.contains_type(ShapeType::Edge));
    }

    #[test]
    fn deep_copy_shares_no_brep_data() {
        let cube = Shape::cube(2.0);
        let copy = cube.deep_copy();

        assert!(!copy.is_same(&cube));
        for face in copy.faces() {
            assert!(!cube.faces().any(|orig| orig.is_same(&face)));
        }
        // Same geometry, distinct TShapes.
        assert!((max_y(&copy) - max_y(&cube)).abs() < 1e-9);
    }

    /// Extruding a face builds a prism whose top cap is the profile carried by a
    /// `TopLoc_Location` (= the extrude vector). `gtransform` must treat that
    /// located sub-shape consistently with the rest: an identity transform must
    /// not move it. Guards the "popped-off face" regression.
    #[test]
    fn gtransform_identity_preserves_located_prism_cap() {
        let wire = Wire::from_ordered_points([
            dvec3(0.0, 0.0, 0.0),
            dvec3(1.0, 0.0, 0.0),
            dvec3(1.0, 0.0, 1.0),
            dvec3(0.0, 0.0, 1.0),
        ])
        .unwrap();
        let solid: Shape = Face::from_wire(&wire).unwrap().extrude(dvec3(0.0, 2.0, 0.0)).into();

        let before = max_y(&solid);
        let identity = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        let after = max_y(&solid.gtransform(identity));
        assert!(
            (before - after).abs() < 1e-6,
            "identity gtransform moved the located cap: max_y {before} -> {after}"
        );

        // A real (uniform 2×) transform must scale the cap with the body — the cap
        // base at y=2 lands at y=4, not y=6 (which is what a double-applied
        // +Y location would produce).
        let scale2 = [
            [2.0, 0.0, 0.0, 0.0],
            [0.0, 2.0, 0.0, 0.0],
            [0.0, 0.0, 2.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        let scaled = max_y(&solid.gtransform(scale2));
        assert!((scaled - 4.0).abs() < 1e-6, "expected cap at y=4 after 2x scale, got {scaled}");
    }

    #[test]
    fn box_has_one_wire_per_face() {
        let cube = Shape::cube(1.0);
        assert_eq!(cube.wires().count(), 6, "a box has one boundary wire per face");
        assert_eq!(cube.faces().count(), 6);
        // edges() yields one occurrence per incident face (no de-duplication): each of
        // the 12 physical edges is shared by 2 faces, giving 24 occurrences.
        assert_eq!(cube.edges().count(), 24);
    }

    #[test]
    fn edge_is_same_ignores_orientation_across_wires() {
        // Each box edge is shared by two face-boundary wires, reached with opposite
        // orientations. `is_same` must still recognize them as the same physical edge.
        let cube = Shape::cube(1.0);
        let global: Vec<_> = cube.edges().collect();
        for wire in cube.wires() {
            for member in wire.edges() {
                assert!(
                    global.iter().any(|g| g.is_same(&member)),
                    "wire edge had no matching global edge by is_same"
                );
            }
        }
    }

    /// The face of `shape` whose center has the largest y — for a box, the top.
    fn top_face(shape: &Shape) -> Face {
        shape
            .faces()
            .max_by(|a, b| a.center_of_mass().y.total_cmp(&b.center_of_mass().y))
            .expect("shape has faces")
    }

    fn translation(v: glam::DVec3) -> [[f64; 4]; 4] {
        [
            [1.0, 0.0, 0.0, v.x],
            [0.0, 1.0, 0.0, v.y],
            [0.0, 0.0, 1.0, v.z],
            [0.0, 0.0, 0.0, 1.0],
        ]
    }

    /// Rotation about the x-directed axis through `center` by `angle` radians.
    fn x_rotation_about(center: glam::DVec3, angle: f64) -> [[f64; 4]; 4] {
        let (sin, cos) = angle.sin_cos();
        // t = c - R*c keeps `center` fixed.
        let ty = center.y - (cos * center.y - sin * center.z);
        let tz = center.z - (sin * center.y + cos * center.z);
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, cos, -sin, ty],
            [0.0, sin, cos, tz],
            [0.0, 0.0, 0.0, 1.0],
        ]
    }

    #[test]
    fn tweak_translate_top_face_stretches_box() {
        let cube = Shape::cube(2.0);
        let tweaked = cube
            .tweak_faces([top_face(&cube)], translation(dvec3(0.0, 1.0, 0.0)))
            .expect("translating the top face up re-solves");

        assert_eq!(tweaked.shape_type(), ShapeType::Solid);
        assert_eq!(tweaked.faces().count(), 6, "a stretched box is still a box");
        assert!((max_y(&tweaked) - 3.0).abs() < 1e-6, "top must land at y=3");
    }

    #[test]
    fn tweak_rotate_top_face_makes_valid_wedge() {
        let cube = Shape::cube(2.0);
        let top = top_face(&cube);
        let mat = x_rotation_about(top.center_of_mass(), 0.3);
        let tweaked = cube.tweak_faces([top], mat).expect("tilting the top face re-solves");

        assert_eq!(tweaked.shape_type(), ShapeType::Solid);
        assert_eq!(tweaked.faces().count(), 6);
        // Tilting about the face-centered x axis raises one top edge and lowers
        // the other.
        assert!(max_y(&tweaked) > 2.0 + 1e-3, "one top edge must rise above y=2");
    }

    #[test]
    fn tweak_rejects_non_similarity_transform() {
        let cube = Shape::cube(2.0);
        let squash = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.5, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        let result = cube.tweak_faces([top_face(&cube)], squash);
        assert!(
            matches!(result, Err(Error::NotASimilarityTransform)),
            "non-uniform scale must be rejected before reaching OCCT"
        );
    }

    #[test]
    fn tweak_rejects_face_from_another_shape() {
        let cube = Shape::cube(2.0);
        let other = Shape::cube(3.0);
        let result = cube.tweak_faces([top_face(&other)], translation(dvec3(0.0, 1.0, 0.0)));
        assert!(result.is_err(), "a face that isn't a sub-shape of the solid must be rejected");
    }

    #[test]
    fn tweak_rotate_top_face_perpendicular_errors() {
        // Rotated a full 90°, the top plane becomes parallel to two walls and
        // can no longer close a volume with them — the re-solve must fail, not
        // produce garbage.
        let cube = Shape::cube(2.0);
        let top = top_face(&cube);
        let mat = x_rotation_about(top.center_of_mass(), std::f64::consts::FRAC_PI_2);
        assert!(cube.tweak_faces([top], mat).is_err());
    }

    #[test]
    fn gtransform_history_maps_faces_across_both_stages() {
        // gtransform chains two MakeShape stages (identity flatten, then the
        // general transform); the merged history must map input faces straight
        // to their final images or downstream history walks dead-end here.
        let cube = Shape::cube(2.0);
        let top = top_face(&cube);
        let (moved, history) = cube.gtransform_with_history(translation(dvec3(0.0, 1.0, 0.0)));

        assert!((max_y(&moved) - 3.0).abs() < 1e-6);
        let images = history.modified_faces(&top);
        assert_eq!(images.len(), 1, "each input face maps to exactly one transformed face");
        assert!((images[0].center_of_mass().y - 3.0).abs() < 1e-6);

        let back = history.source_face(&cube, &images[0]).expect("source face found");
        assert!(back.is_same(&top));
    }

    #[test]
    fn tweak_history_traces_every_result_face() {
        let cube = Shape::cube(2.0);
        let top = top_face(&cube);
        let (tweaked, history) = cube
            .tweak_faces_with_history([top_face(&cube)], translation(dvec3(0.0, 1.0, 0.0)))
            .expect("translating the top face up re-solves");

        // The moved face's image lands at the new height.
        let images = history.modified_faces(&top);
        assert_eq!(images.len(), 1);
        assert!((images[0].center_of_mass().y - 3.0).abs() < 1e-6);

        // Every result face traces back to an input face (modified or passed
        // through unchanged) — nothing appears from nowhere in a box stretch.
        for face in tweaked.faces() {
            assert!(history.source_face(&cube, &face).is_some());
        }
    }

    #[test]
    fn tweak_with_spherical_neighbor_keeps_cavity() {
        // Regression: a spherical neighbor face cannot be extended (the
        // surface is closed), so its patch must become the full sphere. With a
        // plain ExtendFace the re-solve silently returned the box with the
        // cavity filled in; the face-survival gate must also make that a hard
        // error rather than a silent fill.
        let cube = Shape::cube(2.0);
        let bite = Shape::sphere(1.0).at(dvec3(2.0, 2.0, 2.0)).build();
        let cut = cube.subtract(&bite).expect("cut succeeds");
        let faces_before = cut.faces().count();

        let tweaked = cut
            .shape
            .tweak_faces([top_face(&cut.shape)], translation(dvec3(0.0, 0.5, 0.0)))
            .expect("tweak with a spherical neighbor re-solves");

        assert!((max_y(&tweaked) - 2.5).abs() < 1e-6, "top must land at y=2.5");
        assert_eq!(
            tweaked.faces().count(),
            faces_before,
            "the spherical cavity face must survive the re-solve"
        );
    }

    #[test]
    fn tweak_past_spherical_cavity_keeps_internal_void() {
        // A bowl cavity in the top face, from a sphere poking 0.5 above it.
        let cube = Shape::cube(2.0);
        let bite = Shape::sphere(0.5).at(dvec3(1.0, 2.0, 1.0)).build();
        let cut = cube.subtract(&bite).expect("cut succeeds");

        // Raising the top while the plane still crosses the sphere deepens the
        // bowl.
        let small = cut
            .shape
            .tweak_faces([top_face(&cut.shape)], translation(dvec3(0.0, 0.25, 0.0)));
        assert!(small.is_ok(), "top raised into the sphere must re-solve: {:?}", small.err());

        // Raising it past the sphere turns the cavity into an internal void:
        // the full-sphere patch still bounds a cell, so the re-solve keeps the
        // bubble rather than filling it or erroring.
        let past = cut
            .shape
            .tweak_faces([top_face(&cut.shape)], translation(dvec3(0.0, 1.0, 0.0)))
            .expect("top raised past the sphere keeps the void");
        assert!((max_y(&past) - 3.0).abs() < 1e-6);
        assert_eq!(
            past.faces().count(),
            7,
            "6 box faces + the full internal sphere must remain"
        );
    }

    #[test]
    fn tweak_split_cavity_pieces_re_solve() {
        // A deep bite whose cavity crosses the sphere's seam gets split into
        // multiple faces on the same spherical surface. Their patches must be
        // shared (duplicate coincident patches derail face attribution) so the
        // re-solve still succeeds with the cavity intact.
        let cube = Shape::cube(2.0);
        let bite = Shape::sphere(1.2).at(dvec3(1.2, 1.2, 1.2)).build();
        let cut = cube.subtract(&bite).expect("cut succeeds");
        let faces_before = cut.faces().count();

        let tweaked = cut
            .shape
            .tweak_faces([top_face(&cut.shape)], translation(dvec3(0.0, 0.3, 0.0)))
            .expect("tweak with a seam-split cavity re-solves");
        assert_eq!(tweaked.faces().count(), faces_before, "the split cavity must survive");
    }

    #[test]
    fn transformed_preserves_analytic_surfaces() {
        // A GTransform converts every surface to a B-spline — hostile input
        // for the tweak's extend-and-reintersect (and booleans, fillets, ...).
        // `transformed` must take the type-preserving gp_Trsf path instead:
        // the same corner-sphere tweak that re-solves on a pristine box must
        // still re-solve on a moved one.
        let moved = Shape::cube(2.0)
            .transformed(translation(dvec3(3.0, 1.0, -2.0)))
            .expect("a translation is a similarity");
        let corner = dvec3(5.0, 3.0, 0.0);
        let cut = moved.subtract(&Shape::sphere(1.0).at(corner).build()).expect("cut");

        let tweaked = cut
            .shape
            .tweak_faces([top_face(&cut.shape)], translation(dvec3(0.0, 0.5, 0.0)))
            .expect("tweak on a transformed (still analytic) box re-solves");
        assert_eq!(tweaked.faces().count(), cut.faces().count());

        let squash = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.5, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        assert!(
            matches!(Shape::cube(2.0).transformed(squash), Err(Error::NotASimilarityTransform)),
            "a non-uniform scale must be rejected (gtransform is the fallback)"
        );
    }

    #[test]
    fn tweak_on_extruded_box_with_corner_sphere() {
        // The modeler's box recipe: a rectangle wire on the construction
        // plane, made into a face and extruded along the normal (all analytic
        // planes), with the bite sphere snapped to the f32 corner-vertex
        // position. This is the interactive box→sphere-boolean→move-top-up
        // flow that must keep re-solving.
        let (w, d, h) = (10.0f32, 10.0f32, 5.0f32);
        let corners = [
            dvec3(0.0, 0.0, 0.0),
            dvec3(w as f64, 0.0, 0.0),
            dvec3(w as f64, 0.0, d as f64),
            dvec3(0.0, 0.0, d as f64),
        ];
        let wire = Wire::from_ordered_points(corners).expect("rectangle wire");
        let face = Face::from_wire(&wire).expect("rectangle face");
        let world_box: Shape = face.extrude(dvec3(0.0, h as f64, 0.0)).into();

        let snapped = dvec3(w as f64, h as f64, d as f64);
        for r in [2.5, 3.0, 4.0] {
            for dy in [1.0, 2.5] {
                let bite = Shape::sphere(r).at(snapped).build();
                let cut = world_box.subtract(&bite).expect("cut succeeds");
                let tweaked = cut
                    .shape
                    .tweak_faces([top_face(&cut.shape)], translation(dvec3(0.0, dy, 0.0)))
                    .unwrap_or_else(|e| panic!("tweak r={r} dy={dy} must re-solve: {e}"));
                assert_eq!(
                    tweaked.faces().count(),
                    cut.faces().count(),
                    "cavity must survive (r={r} dy={dy})"
                );
            }
        }
    }

    #[test]
    fn tweak_of_split_face_piece_fails_cleanly() {
        // Bites whose rim splits the top face into pieces: moving a single
        // piece while its siblings anchor in place is beyond the direct
        // re-solve. It must fail cleanly — never return a silently filled or
        // altered body. Re-solving from operation history is the recovery.
        for center in [dvec3(1.5, 1.5, 1.0), dvec3(1.0, 1.6, 1.0)] {
            let cube = Shape::cube(2.0);
            let bite = Shape::sphere(1.2).at(center).build();
            let cut = cube.subtract(&bite).expect("cut succeeds");
            let result = cut
                .shape
                .tweak_faces([top_face(&cut.shape)], translation(dvec3(0.0, 0.3, 0.0)));
            match result {
                Err(Error::TweakFailed(message)) => assert!(
                    message.contains("did not survive"),
                    "unexpected tweak error: {message}"
                ),
                Err(other) => panic!("unexpected error kind: {other:?}"),
                Ok(_) => panic!("moving one piece of a split top must fail the survival gate"),
            }
        }
    }

    #[test]
    fn tweak_in_compound_edits_owner_and_keeps_siblings() {
        let box_a = Shape::cube(2.0);
        let box_b = Shape::box_from_corners(dvec3(3.0, 0.0, 0.0), dvec3(5.0, 2.0, 2.0));
        let compound: Shape = Compound::from_shapes([&box_a, &box_b]).into();

        let tweaked = compound
            .tweak_faces([top_face(&box_a)], translation(dvec3(0.0, 1.0, 0.0)))
            .expect("tweaking one child of a compound re-solves");

        assert_eq!(tweaked.shape_type(), ShapeType::Compound);
        assert_eq!(tweaked.sub_shapes().count(), 2, "the sibling solid must survive");
        assert_eq!(tweaked.faces().count(), 12);
        assert!((max_y(&tweaked) - 3.0).abs() < 1e-6, "tweaked child's top must land at y=3");
    }

    #[test]
    fn tweak_faces_spanning_compound_children_errors() {
        let box_a = Shape::cube(2.0);
        let box_b = Shape::box_from_corners(dvec3(3.0, 0.0, 0.0), dvec3(5.0, 2.0, 2.0));
        let compound: Shape = Compound::from_shapes([&box_a, &box_b]).into();

        let result = compound.tweak_faces(
            [top_face(&box_a), top_face(&box_b)],
            translation(dvec3(0.0, 1.0, 0.0)),
        );
        assert!(result.is_err(), "faces owned by different children must be rejected");
    }

    #[test]
    fn tweak_after_fillet_does_not_panic() {
        // A filleted edge makes the top face's neighbors tangent-joined —
        // exactly the fragile case for extend-and-reintersect. Either outcome
        // (a re-solved body or an error) is acceptable; crashing is not.
        let cube = Shape::cube(2.0);
        let edge = cube.edges().next().expect("cube has edges");
        let filleted = cube.fillet_edge(0.2, &edge);
        let top = top_face(&filleted);
        let mat = x_rotation_about(top.center_of_mass(), 0.2);
        match filleted.tweak_faces([top], mat) {
            Ok(tweaked) => {
                // fillet_edges wraps its result in a compound; tweak keeps that shape.
                assert_eq!(tweaked.shape_type(), ShapeType::Compound);
                let mut children = tweaked.sub_shapes();
                assert_eq!(children.next().map(|c| c.shape_type()), Some(ShapeType::Solid));
                assert!(children.next().is_none());
            },
            Err(err) => println!("fillet-adjacent tweak declined: {err}"),
        }
    }

    /// Deduplicates explorer occurrences (a seam is yielded once per wire traversal).
    fn unique_edges(edges: impl Iterator<Item = Edge>) -> Vec<Edge> {
        let mut unique: Vec<Edge> = Vec::new();
        for edge in edges {
            if !unique.iter().any(|e| e.is_same(&edge)) {
                unique.push(edge);
            }
        }
        unique
    }

    #[test]
    fn sphere_has_one_seam_and_two_degenerate_edges() {
        let sphere = Shape::sphere(1.0).build();

        assert_eq!(sphere.seam_edges().len(), 1);

        let degenerate = unique_edges(sphere.edges().filter(Edge::is_degenerated));
        assert_eq!(degenerate.len(), 2);

        // The sphere's only edges are the seam meridian and the two pole edges.
        assert_eq!(unique_edges(sphere.edges()).len(), 3);
    }

    #[test]
    fn sphere_face_center_normal_errors() {
        // A full spherical face's center of mass is the sphere's center, and
        // projecting that onto the surface has no unique solution. OCCT throws;
        // this must surface as an Err, not escape the FFI and abort.
        let sphere = Shape::sphere(1.0).build();
        let face = sphere.faces().next().expect("sphere has a face");
        assert!(face.normal_at_center().is_err());
    }

    #[test]
    fn planar_face_center_normal() {
        let top = top_face(&Shape::cube(2.0));
        let n = top.normal_at_center().expect("planar face has a center normal");
        assert!(n.x.abs() < 1e-9 && n.z.abs() < 1e-9 && (n.y.abs() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn cylinder_has_one_seam_and_real_rim_edges() {
        let cylinder = Shape::cylinder_radius_height(1.0, 2.0);

        let seams = cylinder.seam_edges();
        assert_eq!(seams.len(), 1);
        assert_eq!(cylinder.edges().filter(Edge::is_degenerated).count(), 0);

        // The two rim circles are real edges, not seams.
        let rims = unique_edges(cylinder.edges())
            .into_iter()
            .filter(|e| !seams.iter().any(|s| s.is_same(e)))
            .count();
        assert_eq!(rims, 2);
    }

    #[test]
    fn box_has_no_seam_edges() {
        assert!(Shape::cube(1.0).seam_edges().is_empty());
    }

    fn total_area(shape: &Shape) -> f64 {
        shape.faces().map(|f| f.surface_area()).sum()
    }

    /// A boolean can split a periodic face along its seam, leaving two faces on
    /// the same sphere with a spurious edge between them. `clean` merges them.
    #[test]
    fn clean_merges_boolean_split_sphere_face() {
        let sphere = Shape::sphere(2.0).at(dvec3(0.5, 1.0, 0.5)).build();
        let cut = Shape::cube(2.0).subtract(&sphere).unwrap().shape;
        // Two of the six faces are the halves of one spherical face.
        assert_eq!(cut.faces().count(), 6);
        assert_eq!(unique_edges(cut.edges()).len(), 12);

        let cleaned = cut.clean().unwrap();
        assert_eq!(cleaned.faces().count(), 5);
        assert_eq!(unique_edges(cleaned.edges()).len(), 9);

        // The solid is unchanged: the halves were merged, not dropped.
        assert!((cleaned.volume() - cut.volume()).abs() < 1e-9);
        // Loose: face areas come from numerical integration, so a merged patch
        // does not reproduce the sum of its halves to the last bit.
        assert!((total_area(&cleaned) - total_area(&cut)).abs() < 1e-6);
    }

    fn half_ball_volume(r: f64) -> f64 {
        2.0 / 3.0 * std::f64::consts::PI * r * r * r
    }

    /// A sphere whose center lies exactly on a box face plane has its seam
    /// meridian and/or pole edges lying in the cutting plane. OCCT handles
    /// this *exact* coincidence (and one-ulp neighbors at this scale) — the
    /// dangerous band is a few microns off the plane at mm scale, covered by
    /// [`subtract_with_fuzz_rescues_near_coincident_seam`].
    #[test]
    fn subtract_sphere_snapped_to_face_removes_volume() {
        let size = 2.0f64;
        let cube = Shape::cube(size);
        let cube_vol = cube.volume();

        let snapped = 2.0f32;
        let offsets = [
            ("coplanar", 0.0f64),
            ("+1ulp", f32::from_bits(snapped.to_bits() + 1) as f64 - snapped as f64),
            ("-1ulp", f32::from_bits(snapped.to_bits() - 1) as f64 - snapped as f64),
        ];
        // Face-plane axis: y = top/bottom (plane contains seam AND both poles),
        // x = side (plane contains both poles), z = side (transversal).
        let axes = [("y", 1usize), ("x", 0), ("z", 2)];
        let centers = [0.7f32, 1.0, 1.3];
        let radii = [0.3f32, 0.45, 0.5, 0.6, 0.65];

        struct SnapCase {
            axis_name: &'static str,
            axis: usize,
            offset_name: &'static str,
            offset: f64,
            cu: f32,
            cv: f32,
            radius: f32,
        }

        impl SnapCase {
            /// Sphere center: snapped onto the face plane along `axis`, the
            /// free in-plane coordinates on the other two.
            fn center(&self, size: f64) -> glam::DVec3 {
                let (u_axis, v_axis) = match self.axis {
                    0 => (1, 2),
                    1 => (0, 2),
                    _ => (0, 1),
                };
                let mut c = [0.0f64; 3];
                c[self.axis] = size + self.offset;
                c[u_axis] = self.cu as f64;
                c[v_axis] = self.cv as f64;
                dvec3(c[0], c[1], c[2])
            }

            fn label(&self) -> String {
                format!(
                    "{}/{} c=({},{}) r={}",
                    self.axis_name, self.offset_name, self.cu, self.cv, self.radius
                )
            }
        }

        let mut cases = Vec::new();
        for (axis_name, axis) in axes {
            for (offset_name, offset) in offsets {
                for cu in centers {
                    for cv in centers {
                        for radius in radii {
                            cases.push(SnapCase {
                                axis_name,
                                axis,
                                offset_name,
                                offset,
                                cu,
                                cv,
                                radius,
                            });
                        }
                    }
                }
            }
        }

        let mut failures = Vec::new();
        for case in &cases {
            let r = case.radius as f64;
            let sphere = Shape::sphere(r).at(case.center(size)).build();
            let expected = half_ball_volume(r);
            match cube.subtract(&sphere) {
                Ok(cut) => {
                    let removed = cube_vol - cut.volume();
                    if removed < 0.25 * expected {
                        failures.push(format!(
                            "{}: removed {removed:.6} of expected {expected:.6}",
                            case.label()
                        ));
                    }
                },
                Err(err) => failures.push(format!("{}: cut errored: {err}", case.label())),
            }
        }

        assert!(
            failures.is_empty(),
            "{} of {} snapped subtracts removed no material:\n{}",
            failures.len(),
            cases.len(),
            failures.join("\n")
        );
    }

    /// The modeler's box recipe: a rectangle wire extruded along its normal
    /// (all analytic planes).
    fn extruded_box(w: f64, h: f64, d: f64) -> Shape {
        let wire = Wire::from_ordered_points([
            dvec3(0.0, 0.0, 0.0),
            dvec3(w, 0.0, 0.0),
            dvec3(w, 0.0, d),
            dvec3(0.0, 0.0, d),
        ])
        .unwrap();
        Face::from_wire(&wire).unwrap().extrude(dvec3(0.0, h, 0.0)).into()
    }

    /// Regression for the imprint-only subtract: a sphere whose seam/pole
    /// edges lie a few microns off the cutting plane derails the BOP
    /// classification. As of the vendored OCCT 7.8.1, eight of these
    /// configurations "succeed" while removing *no* material (only imprinting
    /// the section circle) and two more silently remove the wrong amount (~88%).
    /// A fuzzy value sized to the placement error (a few f32 ulps of the extent)
    /// must make every cut exact.
    #[test]
    fn subtract_with_fuzz_rescues_near_coincident_seam() {
        let world_box = extruded_box(100.0, 50.0, 100.0);
        let box_vol = world_box.volume();
        let fuzz = 4.0 * f32::EPSILON as f64 * 100.0;

        // Sphere centers a few microns off a face plane, all far enough from
        // the face rims that the true overlap is exactly a half ball.
        for (label, center, r) in [
            ("top +4e-6 a", dvec3(40.0, 50.0 + 4e-6, 60.0), 20.0),
            ("top +4e-6 b", dvec3(75.0, 50.0 + 4e-6, 40.0), 22.0),
            ("top -4e-6", dvec3(40.0, 50.0 - 4e-6, 60.0), 20.0),
            ("top +1e-5", dvec3(25.0, 50.0 + 1e-5, 25.0), 15.0),
            ("top -1e-5", dvec3(60.0, 50.0 - 1e-5, 70.0), 18.0),
            ("top +5e-7", dvec3(40.0, 50.0 + 5e-7, 60.0), 20.0),
            ("bottom +4e-6", dvec3(50.0, 4e-6, 50.0), 20.0),
            ("bottom -4e-6", dvec3(50.0, -4e-6, 50.0), 20.0),
            ("side x=0 +4e-6", dvec3(4e-6, 25.0, 50.0), 20.0),
            ("side x=100 -4e-6", dvec3(100.0 - 4e-6, 25.0, 50.0), 20.0),
            ("side x=100 -1e-5", dvec3(100.0 - 1e-5, 20.0, 70.0), 15.0),
        ] {
            let sphere = Shape::sphere(r).at(center).build();
            let cut = world_box.subtract_with_fuzz(&sphere, fuzz).expect("cut is done");
            let removed = box_vol - cut.volume();
            let expected = half_ball_volume(r);
            assert!(
                (removed - expected).abs() < 5e-3 * expected,
                "{label}: removed {removed:.4}, expected {expected:.4}"
            );
        }
    }

    /// Randomized sweep mirroring the modeler's recipe end to end: a
    /// wire-extruded box at mm scale, sphere centers face-snapped through f32
    /// (including the few-microns-off near-coincidence band that derails the
    /// default-axis parametrization — see
    /// [`subtract_with_fuzz_rescues_near_coincident_seam`]), booleans run on
    /// deep copies. With the polar axis skewed off every world axis, no
    /// axis-aligned cutting plane comes near the seam or poles, and every cut
    /// must remove its half ball even without a fuzzy value. Deterministic
    /// LCG seed.
    #[test]
    fn subtract_skewed_axis_sphere_near_coincident_sweep() {
        let (w, h, d) = (100.0f64, 50.0, 100.0);
        let world_box = extruded_box(w, h, d);
        let box_vol = world_box.volume();

        let mut state = 0x2545F4914F6CDD1Du64;
        let mut rand01 = move || {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((state >> 33) as f64) / ((1u64 << 31) as f64)
        };

        let dims = [w, h, d];
        let mut failures = Vec::new();
        let total = 800;
        for case in 0..total {
            // A face plane: axis 0/1/2, near or far side.
            let axis = (rand01() * 3.0) as usize % 3;
            let far = rand01() < 0.5;
            let plane = if far { dims[axis] } else { 0.0 };

            // Radius, then in-plane coordinates at least r from the face rim so
            // the overlap is exactly a half ball. All snapped through f32 like
            // the modeler's picks; the plane coordinate additionally lands in
            // the near-coincidence band.
            let r = (5.0 + rand01() * 20.0) as f32 as f64;
            let (u_axis, v_axis) = match axis {
                0 => (1, 2),
                1 => (0, 2),
                _ => (0, 1),
            };
            let near = [0.0, 1e-7, -1e-7, 5e-7, -5e-7, 1e-6, -1e-6, 4e-6, -4e-6, 1e-5, -1e-5];
            let offset = near[(rand01() * near.len() as f64) as usize % near.len()];
            let mut c = [0.0f64; 3];
            c[axis] = plane as f32 as f64 + offset;
            c[u_axis] = (r + rand01() * (dims[u_axis] - 2.0 * r)) as f32 as f64;
            c[v_axis] = (r + rand01() * (dims[v_axis] - 2.0 * r)) as f32 as f64;

            let sphere = Shape::sphere(r)
                .at(dvec3(c[0], c[1], c[2]))
                .axis(dvec3(1.0, 2.0, 3.0).normalize())
                .build();
            let expected = half_ball_volume(r);
            match world_box.deep_copy().subtract(&sphere.deep_copy()) {
                Ok(cut) => {
                    let removed = box_vol - cut.volume();
                    if (removed - expected).abs() > 5e-3 * expected {
                        failures.push(format!(
                            "case {case}: axis {axis} offset {offset:+.0e} c=({:.4},{:.4},{:.4}) r={r:.4}: removed {removed:.4} of expected {expected:.4}",
                            c[0], c[1], c[2]
                        ));
                    }
                },
                Err(err) => failures.push(format!("case {case}: cut errored: {err}")),
            }
        }

        assert!(
            failures.is_empty(),
            "{} of {} snapped subtracts came out wrong (first 10):\n{}",
            failures.len(),
            total,
            failures.iter().take(10).map(String::as_str).collect::<Vec<_>>().join("\n")
        );
    }
}
