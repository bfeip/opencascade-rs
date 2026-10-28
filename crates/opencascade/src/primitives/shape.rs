use crate::{
    history::ShapeHistory,
    mesh::{FaceRange, Mesh, Mesher},
    primitives::{
        boolean_shape, make_axis_1, make_axis_2, make_dir, make_point, make_point2d, make_vec,
        AncestorMap, BooleanShape, Compound, Edge, EdgeIterator, Face, FaceIterator, JoinType,
        ShapeType, Shell, Solid, SubShapeIterator, Vertex, VertexIterator, Wire, WireIterator,
    },
    DraftError, Error, FilletError, OffsetError,
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
    /// triangulations are not carried over. To find the copy of one of `self`'s
    /// faces, edges or vertices, use a [`ShapeCopy`](crate::primitives::ShapeCopy).
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

    /// Total area of the shape's faces (`BRepGProp::SurfaceProperties`).
    pub fn surface_area(&self) -> f64 {
        let mut props = ffi::GProp_GProps_ctor();
        ffi::BRepGProp_SurfaceProperties(&self.inner, props.pin_mut());
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

    /// Number of distinct sub-shapes of type `ty` (`TopExp::MapShapes`). Unlike
    /// [`edges`](Self::edges) and its siblings, a shared sub-shape counts once.
    pub fn unique_sub_shape_count(&self, ty: ShapeType) -> usize {
        let mut map = ffi::new_indexed_map_of_shape();
        ffi::map_shapes(&self.inner, ty.into(), map.pin_mut());
        map.Extent() as usize
    }

    pub fn fillet_edge(&self, radius: f64, edge: &Edge) -> Result<Self, Error> {
        self.fillet_edges(radius, [edge])
    }

    pub fn variable_fillet_edge(
        &self,
        radius_values: impl IntoIterator<Item = (f64, f64)>,
        edge: &Edge,
    ) -> Result<Self, Error> {
        self.variable_fillet_edges(radius_values, [edge])
    }

    pub fn chamfer_edge(&self, distance: f64, edge: &Edge) -> Result<Self, Error> {
        self.chamfer_edges(distance, [edge])
    }

    /// Rounds `edges` to a constant `radius` (`BRepFilletAPI_MakeFillet`).
    ///
    /// Each edge carries the chain of edges tangent to it along with it.
    pub fn fillet_edges<T: AsRef<Edge>>(
        &self,
        radius: f64,
        edges: impl IntoIterator<Item = T>,
    ) -> Result<Self, Error> {
        let mut make_fillet = ffi::BRepFilletAPI_MakeFillet_ctor(&self.inner);

        for edge in edges.into_iter() {
            make_fillet
                .pin_mut()
                .add_edge(radius, &edge.as_ref().inner)
                .map_err(|_| Error::FilletFailed(FilletError::Unknown))?;
        }

        fillet_result(make_fillet)
    }

    /// Rounds `edges` to a radius that varies along each of them, given as
    /// `(t, radius)` pairs with `t` running from 0 to 1.
    pub fn variable_fillet_edges<T: AsRef<Edge>>(
        &self,
        radius_values: impl IntoIterator<Item = (f64, f64)>,
        edges: impl IntoIterator<Item = T>,
    ) -> Result<Self, Error> {
        let radius_values: Vec<_> = radius_values.into_iter().collect();
        let mut array = ffi::TColgp_Array1OfPnt2d_ctor(1, radius_values.len() as i32);

        for (index, (t, radius)) in radius_values.into_iter().enumerate() {
            array.pin_mut().SetValue(index as i32 + 1, &make_point2d(dvec2(t, radius)));
        }

        let mut make_fillet = ffi::BRepFilletAPI_MakeFillet_ctor(&self.inner);

        for edge in edges.into_iter() {
            make_fillet
                .pin_mut()
                .variable_add_edge(&array, &edge.as_ref().inner)
                .map_err(|_| Error::FilletFailed(FilletError::Unknown))?;
        }

        fillet_result(make_fillet)
    }

    /// Bevels `edges`, cutting `distance` back along both faces of each
    /// (`BRepFilletAPI_MakeChamfer`).
    ///
    /// Each edge carries the chain of edges tangent to it along with it.
    pub fn chamfer_edges<T: AsRef<Edge>>(
        &self,
        distance: f64,
        edges: impl IntoIterator<Item = T>,
    ) -> Result<Self, Error> {
        let mut make_chamfer = ffi::BRepFilletAPI_MakeChamfer_ctor(&self.inner);

        for edge in edges.into_iter() {
            make_chamfer
                .pin_mut()
                .add_edge(distance, &edge.as_ref().inner)
                .map_err(|_| Error::FilletFailed(FilletError::Unknown))?;
        }

        chamfer_result(make_chamfer)
    }

    /// Performs fillet of `radius` on all edges of the shape
    pub fn fillet(&self, radius: f64) -> Result<Self, Error> {
        self.fillet_edges(radius, self.edges())
    }

    /// Performs chamfer of `distance` on all edges of the shape
    pub fn chamfer(&self, distance: f64) -> Result<Self, Error> {
        self.chamfer_edges(distance, self.edges())
    }

    /// The faces of this shape that `edge` bounds, each once — two for an
    /// edge between faces, one for a seam or free edge, none for an edge
    /// that is not part of the shape.
    pub fn adjacent_faces(&self, edge: &Edge) -> Vec<Face> {
        let edge_faces = AncestorMap::<Edge, Face>::new(self);
        edge_faces.index_of(edge).and_then(|index| edge_faces.ancestors(index)).unwrap_or_default()
    }

    /// The corner the two faces of this shape that `edge` bounds make halfway
    /// along it; `None` unless it bounds two, or where one has no normal there.
    pub fn corner(&self, edge: &Edge) -> Option<EdgeCorner> {
        let faces: [Face; 2] = self.adjacent_faces(edge).try_into().ok()?;
        let (point, tangent) = edge.length_midpoint().ok()?;
        let normals = [faces[0].normal_at(point).ok()?, faces[1].normal_at(point).ok()?];
        let step = edge.length() * EdgeCorner::PROBE;
        let convex =
            into_face(&faces[0], point, normals[0], tangent, step).map(|into_a| normals[1].dot(into_a) < 0.0);
        Some(EdgeCorner { faces, point, tangent, normals, convex })
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

    /// Sweeps the shape `angle` radians about the axis through `origin` along
    /// `axis`, counter-clockwise looking down `axis` (`BRepPrimAPI_MakeRevol`).
    /// 
    /// A vertex sweeps into an edge, an edge into a face, a wire into a shell
    /// and a face into a solid. `angle` must lie in `(0, 2π]`, and no edge may
    /// cross the axis, though one may lie along it.
    pub fn revolve(&self, origin: DVec3, axis: DVec3, angle: f64) -> Result<Shape, Error> {
        // OCCT only checks the angle in debug builds (`Standard_ConstructionError_Raise_if`).
        if !(angle > ANGULAR_PRECISION && angle <= std::f64::consts::TAU + ANGULAR_PRECISION) {
            return Err(Error::RevolveFailed(format!("angle {angle} is outside (0, 2π]")));
        }
        let axis = make_axis_1(origin, axis);
        let mut make_revol = ffi::BRepPrimAPI_MakeRevol_ctor(&self.inner, &axis, angle, false)
            .map_err(|e| Error::RevolveFailed(e.what().to_string()))?;
        // `CheckValidity` is all that leaves the algorithm not done.
        if !make_revol.IsDone() {
            return Err(Error::RevolveFailed("an edge crosses the axis".to_string()));
        }
        Ok(Self::from_shape(make_revol.pin_mut().Shape()))
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

    /// Whether the shape passes OCCT's topological and geometric checks
    /// (`BRepCheck_Analyzer`).
    pub fn is_valid(&self) -> bool {
        ffi::BRepCheck_Analyzer_ctor(&self.inner, true, false, false)
            .is_ok_and(|analyzer| analyzer.IsValid())
    }

    /// Whether the shape fails OCCT's validity checks or any two of its
    /// sub-shapes interfere where they share no topology (`BRepAlgoAPI_Check`).
    /// Unlike [`Self::is_valid`] this intersects the faces with each other, so
    /// it catches a body passing through itself; it costs about a boolean.
    pub fn self_intersects(&self) -> bool {
        ffi::BRepAlgoAPI_Check_ctor(&self.inner, false, true, &ffi::Message_ProgressRange_ctor())
            .map_or(true, |mut check| !check.pin_mut().IsValid())
    }

    /// Returns `true` if this shape has no underlying topology (null `myTShape`).
    /// Calling most methods on a null shape is undefined behavior.
    pub fn is_null(&self) -> bool {
        ffi::TopoDS_Shape_IsNull(&self.inner)
    }

    /// Whether the shape is closed: a shell or solid with every edge shared
    /// by two faces, or a wire whose ends meet (`BRep_Tool::IsClosed`).
    pub fn is_closed(&self) -> bool {
        ffi::BRep_Tool_IsClosed_Shape(&self.inner)
    }

    /// The solid a closed shell bounds, with its faces turned outward.
    pub fn to_solid(&self) -> Result<Shape, Error> {
        if self.shape_type() != ShapeType::Shell || !self.is_closed() {
            return Err(Error::NotAClosedShell);
        }
        let mut make_solid = ffi::BRepBuilderAPI_MakeSolid_ctor(ffi::TopoDS_cast_to_shell(&self.inner));
        if !make_solid.IsDone() {
            return Err(Error::SolidFailed);
        }
        let mut solid = ffi::TopoDS_Solid_to_owned(ffi::TopoDS_cast_to_solid(make_solid.pin_mut().Shape()));
        if !ffi::BRepLibOrientClosedSolid(solid.pin_mut()) {
            return Err(Error::SolidFailed);
        }
        let solid = Shape::from_shape(ffi::cast_solid_to_shape(&solid));
        match solid.is_valid() && solid.volume() > 0.0 {
            true => Ok(solid),
            false => Err(Error::SolidFailed),
        }
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
        let transform = similarity(&mat)?;
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

    /// Removes features — holes, pockets, bosses, blends — given as their
    /// faces, closing each gap by extending the faces around it
    /// (`BRepAlgoAPI_Defeaturing`). Returns the result with its history.
    ///
    /// Fails unless every feature comes out: one whose neighbours cannot be
    /// extended to cover it (tangent neighbours, typically) fails the whole
    /// removal.
    pub fn remove_features<T: AsRef<Face>>(
        &self,
        faces: impl IntoIterator<Item = T>,
    ) -> Result<(Shape, ShapeHistory), Error> {
        let mut defeaturing = ffi::BRepAlgoAPI_Defeaturing_ctor();
        defeaturing.pin_mut().SetShape(&self.inner);
        for face in faces {
            defeaturing.pin_mut().AddFaceToRemove(ffi::cast_face_to_shape(&face.as_ref().inner));
        }

        let built = defeaturing.pin_mut().Build(&ffi::Message_ProgressRange_ctor());
        if let Err(error) = built {
            return Err(Error::RemoveFeaturesFailed(error.what().to_string()));
        }
        if !defeaturing.IsDone() || ffi::BRepAlgoAPI_Defeaturing_has_errors(&defeaturing) {
            return Err(Error::RemoveFeaturesFailed("the removal did not complete".to_string()));
        }
        // A feature it could not remove is only a warning to OCCT.
        if ffi::BRepAlgoAPI_Defeaturing_has_warnings(&defeaturing) {
            return Err(Error::RemoveFeaturesFailed(ffi::BRepAlgoAPI_Defeaturing_warnings(
                &defeaturing,
            )));
        }

        let history = ShapeHistory::from_handle(ffi::BRepAlgoAPI_Defeaturing_history(
            defeaturing.pin_mut(),
        ));
        Ok((Shape::from_shape(defeaturing.pin_mut().Shape()), history))
    }

    /// The faces the line through `line_origin` along `line_dir` crosses,
    /// intersecting within `tolerance`.
    // TODO(bschwind) - Convert the return type to an iterator.
    pub fn faces_along_line(
        &self,
        line_origin: DVec3,
        line_dir: DVec3,
        tolerance: f64,
    ) -> Vec<LineFaceHitPoint> {
        let mut intersector = ffi::BRepIntCurveSurface_Inter_ctor();
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

    /// Hollows the shape into walls `offset` thick, open where
    /// `faces_to_remove` were, or closed around an internal void when there
    /// are none. A closed hollow needs a solid with a single shell.
    ///
    /// A negative `offset` builds the walls inside the surface, a positive one
    /// outside it. `join` shapes the walls where offset faces pull apart.
    /// `tolerance` is the offset's coincidence tolerance: an offset no larger
    /// than it is null.
    pub fn hollow<T: AsRef<Face>>(
        &self,
        offset: f64,
        faces_to_remove: impl IntoIterator<Item = T>,
        join: JoinType,
        tolerance: f64,
    ) -> Result<Self, Error> {
        let mut faces_to_remove = faces_to_remove.into_iter().peekable();
        if faces_to_remove.peek().is_none() {
            return self.hollow_closed(offset, join, tolerance);
        }

        let mut faces_list = ffi::new_list_of_shape();
        for face in faces_to_remove {
            ffi::shape_list_append_face(faces_list.pin_mut(), &face.as_ref().inner);
        }

        let mut solid_maker = ffi::BRepOffsetAPI_MakeThickSolid_ctor();
        let built = solid_maker.pin_mut().MakeThickSolidByJoin(
            &self.inner,
            &faces_list,
            offset,
            tolerance,
            ffi::BRepOffset_Mode::BRepOffset_Skin,
            false,
            false,
            join.into(),
            false,
            &ffi::Message_ProgressRange_ctor(),
        );
        thick_solid_result(solid_maker, built)
    }

    /// Walls `offset` thick around a void, with the shape's own surface as
    /// their outside for a negative `offset` and as their inside for a
    /// positive one.
    fn hollow_closed(&self, offset: f64, join: JoinType, tolerance: f64) -> Result<Self, Error> {
        let offset_body = self.offset_surface(offset, join, tolerance)?;
        let (outer, inner) = if offset < 0.0 { (self, &offset_body) } else { (&offset_body, self) };
        // Past the shape's own thickness a curved offset turns inside out into
        // a smaller one, valid but too close to the outside.
        if !walls_hold(outer, inner, offset.abs(), tolerance) {
            return Err(Error::OffsetFailed(OffsetError::InvalidResult));
        }
        let outer = only_shell(outer)?;
        let mut inner = only_shell(inner)?;
        // Inside out, the inner shell faces into the void.
        inner.inner.pin_mut().Reverse();

        let mut make_solid = ffi::BRepBuilderAPI_MakeSolid_ctor(ffi::TopoDS_cast_to_shell(&outer.inner));
        make_solid.pin_mut().Add(ffi::TopoDS_cast_to_shell(&inner.inner));
        if !make_solid.IsDone() {
            return Err(Error::OffsetFailed(OffsetError::Unknown));
        }
        offset_result(Self::from_shape(make_solid.pin_mut().Shape()))
    }

    /// Offsets the whole surface of the shape by `offset`, growing or
    /// shrinking it. A solid stays a solid; a face or open shell becomes a
    /// shell. `tolerance` is the offset's coincidence tolerance: an offset no
    /// larger than it is null.
    pub fn offset_surface(
        &self,
        offset: f64,
        join: JoinType,
        tolerance: f64,
    ) -> Result<Self, Error> {
        self.offset_by_join(offset, join, tolerance, false)
    }

    /// Thickens a face or open shell into a solid `offset` thick, grown along
    /// its normals — or against them, for a negative `offset`. `join` shapes
    /// the solid where offset faces pull apart. `tolerance` is the offset's
    /// coincidence tolerance: an offset no larger than it is null.
    pub fn thicken(&self, offset: f64, join: JoinType, tolerance: f64) -> Result<Self, Error> {
        let thickened = self.offset_by_join(offset, join, tolerance, true)?;
        // A slab that fails to close comes back as a shell or compound.
        if thickened.shape_type() != ShapeType::Solid {
            return Err(Error::OffsetFailed(OffsetError::InvalidResult));
        }
        Ok(thickened)
    }

    /// The shape's surface offset by `offset`, or with `thickening`, the solid
    /// between the surface and that offset (`BRepOffset_MakeOffset`).
    fn offset_by_join(
        &self,
        offset: f64,
        join: JoinType,
        tolerance: f64,
        thickening: bool,
    ) -> Result<Self, Error> {
        let mut make_offset = ffi::BRepOffset_MakeOffset_ctor();
        make_offset.pin_mut().Initialize(
            &self.inner,
            offset,
            tolerance,
            ffi::BRepOffset_Mode::BRepOffset_Skin,
            false,
            false,
            join.into(),
            thickening,
            false,
        );
        // A throw leaves the error code unset.
        if make_offset.pin_mut().MakeOffsetShape(&ffi::Message_ProgressRange_ctor()).is_err() {
            return Err(Error::OffsetFailed(OffsetError::Unknown));
        }
        if !make_offset.IsDone() {
            return Err(Error::OffsetFailed(make_offset.Error().into()));
        }
        offset_result(Self::from_shape(make_offset.Shape()))
    }

    /// Tapers `faces` by `angle` radians, tilting each about the line where it
    /// crosses the neutral plane through `neutral_origin` with normal
    /// `neutral_normal` (`BRepOffsetAPI_DraftAngle`).
    ///
    /// `pull` is the draft direction: a positive `angle` removes material on
    /// the side of the neutral plane it points into, a negative one adds it.
    /// Only planar, cylindrical and conical faces can be drafted, and a
    /// cylinder or cone only along its axis.
    pub fn draft_faces<T: AsRef<Face>>(
        &self,
        faces: impl IntoIterator<Item = T>,
        pull: DVec3,
        angle: f64,
        neutral_origin: DVec3,
        neutral_normal: DVec3,
    ) -> Result<Shape, Error> {
        Ok(self.draft_faces_with_history(faces, pull, angle, neutral_origin, neutral_normal)?.0)
    }

    /// Like [`Self::draft_faces`], additionally returning the sub-shape
    /// history (input sub-shapes → their drafted images).
    pub fn draft_faces_with_history<T: AsRef<Face>>(
        &self,
        faces: impl IntoIterator<Item = T>,
        pull: DVec3,
        angle: f64,
        neutral_origin: DVec3,
        neutral_normal: DVec3,
    ) -> Result<(Shape, ShapeHistory), Error> {
        let pull = make_dir(pull);
        let neutral_plane = ffi::gp_Pln_ctor(&make_point(neutral_origin), &make_dir(neutral_normal));
        let failed = |draft: &ffi::BRepOffsetAPI_DraftAngle| Error::DraftFailed(draft.Status().into());

        let mut draft = ffi::BRepOffsetAPI_DraftAngle_ctor(&self.inner);
        for face in faces.into_iter() {
            let added =
                draft.pin_mut().Add(&face.as_ref().inner, &pull, angle, &neutral_plane, true);
            // Once an add fails, the algorithm refuses any further one.
            if added.is_err() || !draft.AddDone() {
                return Err(failed(&draft));
            }
        }
        let built = draft.pin_mut().Build(&ffi::Message_ProgressRange_ctor());
        if built.is_err() || !draft.IsDone() {
            return Err(failed(&draft));
        }

        let shape = Self::from_shape(draft.pin_mut().Shape());
        let mut inputs = ffi::new_list_of_shape();
        ffi::shape_list_append_shape(inputs.pin_mut(), &self.inner);
        let history = ShapeHistory::from_handle(ffi::BRepOffsetAPI_DraftAngle_history(
            draft.pin_mut(),
            &inputs,
        ));

        Ok((shape, history))
    }

    /// Moves `faces` by `mat` and re-trims the faces around them to meet them,
    /// keeping the topology (`Tweak_MoveFaces`): every face bordering a moved
    /// one keeps its surface, and the edges and vertices where they meet are
    /// re-solved. Faces tangent to a moved face move with it.
    ///
    /// `mat` is row-major and must be a similarity. Errs when an edge or
    /// vertex cannot be re-solved, and when the move would change the topology
    /// (a face shrinking away, a moved face reaching through another).
    pub fn move_faces<T: AsRef<Face>>(
        &self,
        faces: impl IntoIterator<Item = T>,
        mat: [[f64; 4]; 4],
    ) -> Result<Shape, Error> {
        Ok(self.move_faces_with_history(faces, mat)?.0)
    }

    /// Like [`Self::move_faces`], additionally returning the sub-shape
    /// history (input sub-shapes → their images).
    pub fn move_faces_with_history<T: AsRef<Face>>(
        &self,
        faces: impl IntoIterator<Item = T>,
        mat: [[f64; 4]; 4],
    ) -> Result<(Shape, ShapeHistory), Error> {
        let transform = similarity(&mat)?;
        let failed = |op: &ffi::Tweak_MoveFaces| Error::MoveFacesFailed(op.Status().into());

        let mut op = ffi::Tweak_MoveFaces_ctor(&self.inner);
        for face in faces.into_iter() {
            let added = op.pin_mut().Add(&face.as_ref().inner, &transform);
            // Once an add fails, the algorithm refuses any further one.
            if added.is_err() || !op.AddDone() {
                return Err(failed(&op));
            }
        }
        let built = op.pin_mut().Build(&ffi::Message_ProgressRange_ctor());
        if built.is_err() || !op.IsDone() {
            return Err(failed(&op));
        }

        // The topology is kept even where the move needs it to change (a face
        // shrinking away, a moved face reaching through another), which leaves
        // an invalid or self-intersecting body.
        let shape = Self::from_shape(op.pin_mut().Shape());
        let broken = shape.is_null()
            || !shape.is_valid()
            || shape.self_intersects()
            || (shape.contains_type(ShapeType::Solid) && shape.volume() <= 0.0);
        if broken {
            return Err(Error::MoveFacesFailed(DraftError::InvalidResult));
        }

        let mut inputs = ffi::new_list_of_shape();
        ffi::shape_list_append_shape(inputs.pin_mut(), &self.inner);
        let history = ShapeHistory::from_handle(ffi::Tweak_MoveFaces_history(op.pin_mut(), &inputs));

        Ok((shape, history))
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

/// The corner two faces of a shape make along an edge between them, at a point
/// on it.
pub struct EdgeCorner {
    /// The two faces.
    pub faces: [Face; 2],
    /// Where on the edge it is measured.
    pub point: DVec3,
    /// The edge's unit tangent there.
    pub tangent: DVec3,
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

/// The shape a fillet build produced, or why it failed.
fn fillet_result(
    mut make_fillet: UniquePtr<ffi::BRepFilletAPI_MakeFillet>,
) -> Result<Shape, Error> {
    // With no contour to build, OCCT throws rather than failing softly.
    if make_fillet.NbContours() == 0 {
        return Err(Error::FilletFailed(FilletError::NoSuitableEdges));
    }

    let built = make_fillet.pin_mut().Build(&ffi::Message_ProgressRange_ctor());
    if built.is_err() || !make_fillet.IsDone() {
        // StripeStatus dereferences its contour unchecked, so it is only
        // asked about a contour FaultyContour actually named.
        let contour = match make_fillet.NbFaultyContours() {
            0 => 0,
            _ => make_fillet.FaultyContour(1),
        };
        let reason = match contour {
            0 => FilletError::Unknown,
            contour => make_fillet.StripeStatus(contour).into(),
        };
        return Err(Error::FilletFailed(reason));
    }

    valid_result(Shape::from_shape(make_fillet.pin_mut().Shape()))
}

/// The shape a chamfer build produced, or why it failed. The chamfer builder
/// does not say why a contour failed.
fn chamfer_result(
    mut make_chamfer: UniquePtr<ffi::BRepFilletAPI_MakeChamfer>,
) -> Result<Shape, Error> {
    // With no contour to build, OCCT throws rather than failing softly.
    if make_chamfer.NbContours() == 0 {
        return Err(Error::FilletFailed(FilletError::NoSuitableEdges));
    }

    let built = make_chamfer.pin_mut().Build(&ffi::Message_ProgressRange_ctor());
    if built.is_err() || !make_chamfer.IsDone() {
        return Err(Error::FilletFailed(FilletError::Unknown));
    }

    valid_result(Shape::from_shape(make_chamfer.pin_mut().Shape()))
}

/// A finished fillet or chamfer, unless it is broken: OCCT reports blends that
/// overlap one another as done.
fn valid_result(shape: Shape) -> Result<Shape, Error> {
    match shape.is_valid() {
        true => Ok(shape),
        false => Err(Error::FilletFailed(FilletError::InvalidResult)),
    }
}

/// OCCT's `Precision::Angular()`: angles no larger than this are null.
const ANGULAR_PRECISION: f64 = 1e-12;

/// Whether `outer` keeps at least `thickness` away from `void`, within
/// `tolerance`, measured from the middle of each of the void's faces along its
/// outward normal.
fn walls_hold(outer: &Shape, void: &Shape, thickness: f64, tolerance: f64) -> bool {
    void.faces().all(|face| {
        let middle = face.midpoint();
        let Ok(normal) = face.normal_at(middle) else { return true };
        outer
            .faces_along_line(middle, normal.normalize(), tolerance)
            .iter()
            .all(|hit| hit.t <= 0.0 || hit.t >= thickness - tolerance)
    })
}

/// The one shell bounding `solid`, or an error for anything else.
fn only_shell(solid: &Shape) -> Result<Shape, Error> {
    let mut shells = solid.sub_shapes();
    match (solid.shape_type(), shells.next(), shells.next()) {
        (ShapeType::Solid, Some(shell), None) if shell.shape_type() == ShapeType::Shell => Ok(shell),
        _ => Err(Error::OffsetFailed(OffsetError::Unknown)),
    }
}

/// The solid a thick-solid build produced, or why it failed.
fn thick_solid_result(
    mut solid_maker: UniquePtr<ffi::BRepOffsetAPI_MakeThickSolid>,
    built: Result<(), cxx::Exception>,
) -> Result<Shape, Error> {
    if built.is_err() || !solid_maker.IsDone() {
        return Err(Error::OffsetFailed(solid_maker.MakeOffset().Error().into()));
    }
    offset_result(Shape::from_shape(solid_maker.pin_mut().Shape()))
}

/// A finished offset, unless it is broken: OCCT reports offsets past a shape's
/// own thickness as done, with nothing, a collapsed solid or invalid topology.
fn offset_result(shape: Shape) -> Result<Shape, Error> {
    let broken = shape.is_null()
        || !shape.is_valid()
        || (shape.contains_type(ShapeType::Solid) && shape.volume() <= 0.0);
    match broken {
        true => Err(Error::OffsetFailed(OffsetError::InvalidResult)),
        false => Ok(shape),
    }
}

/// `mat`, row-major, as a `gp_Trsf`, unless it is not a similarity.
fn similarity(mat: &[[f64; 4]; 4]) -> Result<UniquePtr<ffi::gp_Trsf>, Error> {
    // gp_Trsf::SetValues aborts (uncatchable across cxx) on any other matrix.
    if !is_similarity(mat) {
        return Err(Error::NotASimilarityTransform);
    }
    let mut transform = ffi::new_transform();
    #[rustfmt::skip]
    transform.pin_mut().SetValues(
        mat[0][0], mat[0][1], mat[0][2], mat[0][3],
        mat[1][0], mat[1][1], mat[1][2], mat[1][3],
        mat[2][0], mat[2][1], mat[2][2], mat[2][3],
    );
    Ok(transform)
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

    pub fn add_edge(&mut self, distance: f64, edge: &Edge) -> Result<(), Error> {
        self.inner
            .pin_mut()
            .add_edge(distance, &edge.inner)
            .map_err(|_| Error::FilletFailed(FilletError::Unknown))
    }

    pub fn build(self) -> Result<Shape, Error> {
        chamfer_result(self.inner)
    }
}

#[cfg(test)]
mod tests {
    use super::Shape;
    use crate::primitives::{Compound, Edge, EdgeType, Face, FaceType, JoinType, ShapeType, Shell, Wire};
    use crate::workplane::Workplane;
    use crate::{DraftError, Error, FilletError, OffsetError};
    use glam::{dvec3, DVec3};

    /// Offset coincidence tolerance for shapes a few units across.
    const OFFSET_TOLERANCE: f64 = 1e-3;

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
    fn a_cube_measures_its_area_and_distinct_topology() {
        let cube = Shape::cube(2.0);
        assert!((cube.surface_area() - 24.0).abs() < 1e-9);
        assert_eq!(cube.unique_sub_shape_count(ShapeType::Face), 6);
        assert_eq!(cube.unique_sub_shape_count(ShapeType::Edge), 12);
        assert_eq!(cube.unique_sub_shape_count(ShapeType::Vertex), 8);
        assert_eq!(cube.edges().count(), 24, "edges() counts each occurrence");
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

    /// The face of `shape` whose center is nearest `point`.
    fn face_near(shape: &Shape, point: DVec3) -> Face {
        shape
            .faces()
            .min_by(|a, b| a.center_of_mass().distance(point).total_cmp(&b.center_of_mass().distance(point)))
            .expect("shape has faces")
    }

    fn assert_volume(shape: &Shape, expected: f64) {
        let volume = shape.volume();
        assert!((volume - expected).abs() <= 1e-6 * expected, "volume {volume}, expected {expected}");
    }

    /// The area a round of `radius` takes off a right-angled corner.
    fn rounded_corner(radius: f64) -> f64 {
        radius * radius * (1.0 - std::f64::consts::FRAC_PI_4)
    }

    /// A 20-unit cube with its top front edge (y = z = 20) rounded to radius 3.
    fn rounded_block() -> Shape {
        let cube = Shape::box_from_corners(DVec3::ZERO, dvec3(20.0, 20.0, 20.0));
        let edge = cube
            .edges()
            .find(|e| {
                [e.start_point(), e.end_point()].iter().all(|p| (p.y - 20.0).abs() < 1e-9 && (p.z - 20.0).abs() < 1e-9)
            })
            .expect("the cube has a top front edge");
        cube.fillet_edge(3.0, &edge).expect("a radius 3 round fits")
    }

    #[test]
    fn move_faces_translate_top_stretches_box() {
        let cube = Shape::cube(2.0);
        let moved = cube
            .move_faces([top_face(&cube)], translation(dvec3(0.0, 1.0, 0.0)))
            .expect("translating the top face up re-solves");

        assert_eq!(moved.shape_type(), ShapeType::Solid);
        assert_eq!(moved.faces().count(), 6, "a stretched box is still a box");
        assert!((max_y(&moved) - 3.0).abs() < 1e-6, "top must land at y=3");
    }

    /// Nothing in the re-solve is tuned to a fixed length.
    #[test]
    fn move_faces_stretches_a_box_two_thousandths_across() {
        let cube = Shape::cube(0.002);
        let moved = cube
            .move_faces([top_face(&cube)], translation(dvec3(0.0, 0.001, 0.0)))
            .expect("translating a small box's top face up re-solves");

        assert_eq!(moved.shape_type(), ShapeType::Solid);
        assert!((moved.volume() - 1.2e-8).abs() < 1e-15, "got {}", moved.volume());
    }

    #[test]
    fn move_faces_rotate_top_makes_valid_wedge() {
        let cube = Shape::cube(2.0);
        let top = top_face(&cube);
        let mat = x_rotation_about(top.center_of_mass(), 0.3);
        let moved = cube.move_faces([top], mat).expect("tilting the top face re-solves");

        assert_eq!(moved.shape_type(), ShapeType::Solid);
        assert_eq!(moved.faces().count(), 6);
        // Tilting about the face-centered x axis raises one top edge and lowers
        // the other, keeping the volume.
        assert!(max_y(&moved) > 2.0 + 1e-3, "one top edge must rise above y=2");
        assert_volume(&moved, 8.0);
    }

    #[test]
    fn move_faces_rejects_non_similarity_transform() {
        let cube = Shape::cube(2.0);
        let squash = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.5, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        let result = cube.move_faces([top_face(&cube)], squash);
        assert!(
            matches!(result, Err(Error::NotASimilarityTransform)),
            "non-uniform scale must be rejected before reaching OCCT"
        );
    }

    #[test]
    fn move_faces_rejects_face_from_another_shape() {
        let cube = Shape::cube(2.0);
        let other = Shape::cube(3.0);
        let result = cube.move_faces([top_face(&other)], translation(dvec3(0.0, 1.0, 0.0)));
        assert!(result.is_err(), "a face that isn't a sub-shape of the solid must be rejected");
    }

    #[test]
    fn move_faces_rotate_top_perpendicular_errors() {
        // Rotated a full 90°, the top plane becomes parallel to two walls and
        // can no longer meet them: the move must fail, not produce garbage.
        let cube = Shape::cube(2.0);
        let top = top_face(&cube);
        let mat = x_rotation_about(top.center_of_mass(), std::f64::consts::FRAC_PI_2);
        assert!(cube.move_faces([top], mat).is_err());
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
    fn move_faces_history_traces_every_result_face() {
        let cube = Shape::cube(2.0);
        let top = top_face(&cube);
        let (moved, history) = cube
            .move_faces_with_history([top_face(&cube)], translation(dvec3(0.0, 1.0, 0.0)))
            .expect("translating the top face up re-solves");

        // The moved face's image lands at the new height.
        let images = history.modified_faces(&top);
        assert_eq!(images.len(), 1);
        assert!((images[0].center_of_mass().y - 3.0).abs() < 1e-6);

        // Every result face traces back to an input face (modified or passed
        // through unchanged): nothing appears from nowhere in a box stretch.
        for face in moved.faces() {
            assert!(history.source_face(&cube, &face).is_some());
        }
    }

    #[test]
    fn move_faces_matches_recutting_a_pocket() {
        // A box cut into a sphere's side, then one of the pocket's walls moved:
        // the same as moving the box's face and cutting again. Every face of
        // the pocket borders the moved wall, so nothing stays put to anchor a
        // rebuild of the body; only re-trimming the neighbours gets it right.
        let ball = Shape::sphere(30.0).build();
        let pocket = |x: f64| Shape::box_from_corners(dvec3(x, 5.0, 5.0), dvec3(60.0, 60.0, 60.0));
        let cut = ball.subtract(&pocket(5.0)).expect("cut succeeds").shape;
        for dx in [3.0, -3.0] {
            let wall = face_near(&cut, dvec3(5.0, 15.0, 15.0));
            let moved = cut.move_faces([wall], translation(dvec3(dx, 0.0, 0.0))).expect("the wall moves");
            assert_volume(&moved, ball.subtract(&pocket(5.0 + dx)).expect("cut succeeds").shape.volume());
        }
    }

    #[test]
    fn move_faces_lengthens_a_bordering_round() {
        let block = rounded_block();
        let left = face_near(&block, dvec3(0.0, 10.0, 10.0));
        let moved = block.move_faces([left], translation(dvec3(-5.0, 0.0, 0.0))).expect("the side moves");
        assert_volume(&moved, block.volume() + 5.0 * (400.0 - rounded_corner(3.0)));
    }

    #[test]
    fn move_faces_carries_a_tangent_round() {
        // The round moves with the face it is tangent to; its other support,
        // moved within its own plane, stays where it is.
        let block = rounded_block();
        let expected = 20.0 * (500.0 - rounded_corner(3.0));
        let front = face_near(&block, dvec3(10.0, 8.0, 20.0));
        let pushed = block.move_faces([front], translation(dvec3(0.0, 0.0, 5.0))).expect("the front moves");
        assert_volume(&pushed, expected);
        let top = face_near(&block, dvec3(10.0, 20.0, 8.0));
        let raised = block.move_faces([top], translation(dvec3(0.0, 5.0, 0.0))).expect("the top moves");
        assert_volume(&raised, expected);
    }

    #[test]
    fn move_faces_retrims_a_spline_neighbour() {
        // A groove swept from a spline runs front to back; pushing the back out
        // lengthens it, without extending the spline past its ends.
        let cube = Shape::box_from_corners(DVec3::ZERO, dvec3(20.0, 20.0, 20.0));
        let floor = Edge::spline_from_points(
            [dvec3(4.0, 22.0, -5.0), dvec3(8.0, 15.0, -5.0), dvec3(12.0, 17.0, -5.0), dvec3(16.0, 22.0, -5.0)],
            None,
            false,
        )
        .expect("the spline interpolates");
        let lid = Edge::segment(dvec3(16.0, 22.0, -5.0), dvec3(4.0, 22.0, -5.0)).expect("a segment");
        let profile = Face::from_wire(&Wire::from_edges([&floor, &lid]).expect("a closed wire")).expect("a face");
        let grooved = cube.subtract(&profile.extrude(dvec3(0.0, 0.0, 30.0)).into()).expect("cut succeeds").shape;

        let back = face_near(&grooved, dvec3(10.0, 8.0, 0.0));
        let moved = grooved.move_faces([back], translation(dvec3(0.0, 0.0, -5.0))).expect("the back moves");
        assert_volume(&moved, grooved.volume() * 25.0 / 20.0);
    }

    #[test]
    fn move_faces_with_spherical_neighbour_keeps_cavity() {
        // A corner bitten out by a sphere whose poles lie off the cavity's rim.
        let cube = Shape::cube(2.0);
        let ball = Shape::sphere(1.0).at(dvec3(2.0, 2.0, 2.0)).axis(dvec3(1.0, 1.0, 1.0)).build();
        let cut = cube.subtract(&ball).expect("cut succeeds");

        let moved = cut
            .shape
            .move_faces([top_face(&cut.shape)], translation(dvec3(0.0, 0.5, 0.0)))
            .expect("the top moves");
        assert_eq!(moved.faces().count(), cut.faces().count(), "the cavity must survive");
        let taller = Shape::box_from_corners(DVec3::ZERO, dvec3(2.0, 2.5, 2.0));
        assert_volume(&moved, taller.subtract(&ball).expect("cut succeeds").shape.volume());
    }

    #[test]
    fn move_faces_deepens_a_bowl_until_it_closes() {
        let cube = Shape::cube(2.0);
        let ball = Shape::sphere(0.5).at(dvec3(1.0, 2.0, 1.0)).axis(DVec3::Y).build();
        let bowl = cube.subtract(&ball).expect("cut succeeds").shape;

        // Raised while it still crosses the sphere, the top deepens the bowl.
        let raised = bowl.move_faces([top_face(&bowl)], translation(dvec3(0.0, 0.25, 0.0))).expect("the top moves");
        let taller = Shape::box_from_corners(DVec3::ZERO, dvec3(2.0, 2.25, 2.0));
        assert_volume(&raised, taller.subtract(&ball).expect("cut succeeds").shape.volume());

        // Raised past it, the bowl would close into a void: a new shell.
        assert!(bowl.move_faces([top_face(&bowl)], translation(dvec3(0.0, 1.0, 0.0))).is_err());
    }

    #[test]
    #[ignore = "a sphere's pole on the cavity rim must move with it, which re-parametrising the sphere would allow"]
    fn move_faces_with_a_pole_on_the_rim() {
        // The modeler's box recipe (a rectangle extruded along the normal),
        // bitten by a sphere snapped to a corner: the sphere's pole lands on the
        // box's upright edge, at a vertex of the cavity's rim.
        let (w, d, h) = (10.0, 10.0, 5.0);
        let corners = [dvec3(0.0, 0.0, 0.0), dvec3(w, 0.0, 0.0), dvec3(w, 0.0, d), dvec3(0.0, 0.0, d)];
        let face = Face::from_wire(&Wire::from_ordered_points(corners).expect("rectangle wire")).expect("a face");
        let block: Shape = face.extrude(dvec3(0.0, h, 0.0)).into();

        for r in [2.5, 3.0, 4.0] {
            let ball = Shape::sphere(r).at(dvec3(w, h, d)).build();
            let cut = block.subtract(&ball).expect("cut succeeds");
            let moved = cut
                .shape
                .move_faces([top_face(&cut.shape)], translation(dvec3(0.0, 1.0, 0.0)))
                .unwrap_or_else(|e| panic!("r={r}: {e}"));
            assert_eq!(moved.faces().count(), cut.faces().count(), "cavity must survive (r={r})");
        }
    }

    #[test]
    fn move_faces_refuses_a_change_of_topology() {
        let cube = Shape::box_from_corners(DVec3::ZERO, dvec3(20.0, 20.0, 20.0));
        let refused = |shape: &Shape, face: Face, by: DVec3| {
            matches!(shape.move_faces([face], translation(by)), Err(Error::MoveFacesFailed(_)))
        };

        // Through the opposite face.
        assert!(refused(&cube, top_face(&cube), dvec3(0.0, -25.0, 0.0)));

        // Into a blind hole from below, which would break through the top.
        let bore = Shape::cylinder(dvec3(10.0, -1.0, 10.0), 3.0, DVec3::Y, 16.0);
        let blind = cube.subtract(&bore).expect("cut succeeds").shape;
        assert!(refused(&blind, top_face(&blind), dvec3(0.0, -8.0, 0.0)));

        // A step raised past the top it rises to.
        let riser = Shape::box_from_corners(dvec3(0.0, 20.0, 0.0), dvec3(10.0, 30.0, 20.0));
        let step = cube.union(&riser).expect("fuse succeeds").shape;
        assert!(refused(&step, face_near(&step, dvec3(15.0, 20.0, 10.0)), dvec3(0.0, 15.0, 0.0)));

        // A side pushed in under a boss on the top.
        let boss = Shape::box_from_corners(dvec3(7.5, 20.0, 7.5), dvec3(12.5, 25.0, 12.5));
        let bossed = cube.union(&boss).expect("fuse succeeds").shape;
        let side = face_near(&bossed, dvec3(0.0, 10.0, 10.0));
        assert!(refused(&bossed, side, dvec3(10.0, 0.0, 0.0)));
        let side = face_near(&bossed, dvec3(0.0, 10.0, 10.0));
        let clear = bossed.move_faces([side], translation(dvec3(5.0, 0.0, 0.0))).expect("short of the boss");
        assert_volume(&clear, bossed.volume() - 2000.0);
    }

    #[test]
    fn move_faces_refuses_a_cavity_leaving_the_sides() {
        // A deep bite reaching the cube's sides: raising the top shrinks the rim
        // until it no longer meets them, which changes the topology.
        let cube = Shape::cube(2.0);
        let bite = Shape::sphere(1.2).at(dvec3(1.2, 1.2, 1.2)).build();
        let cut = cube.subtract(&bite).expect("cut succeeds").shape;
        assert!(cut.move_faces([top_face(&cut)], translation(dvec3(0.0, 0.3, 0.0))).is_err());
    }

    #[test]
    fn move_faces_moves_a_whole_sphere() {
        let ball = Shape::sphere(5.0).build();
        let surface = ball.faces().next().expect("a sphere has a face");
        let moved = ball.move_faces([surface], translation(dvec3(1.0, 2.0, 3.0))).expect("the sphere moves");
        assert_volume(&moved, ball.volume());
        let centre = moved.faces().next().expect("a sphere has a face").center_of_mass();
        assert!(centre.distance(dvec3(1.0, 2.0, 3.0)) < 1e-6, "centre at {centre}");
    }

    #[test]
    fn transformed_preserves_analytic_surfaces() {
        // A GTransform converts every surface to a B-spline, hostile input for
        // booleans, fillets and face moves. `transformed` must take the
        // type-preserving gp_Trsf path instead.
        let moved = Shape::cube(2.0)
            .transformed(translation(dvec3(3.0, 1.0, -2.0)))
            .expect("a translation is a similarity");
        assert!(moved.faces().all(|face| face.face_type() == FaceType::Plane));

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
    fn move_faces_of_split_face_piece_fails_cleanly() {
        // Bites whose rim splits the top face in two: raising one piece while
        // its sibling stays puts a step into the sides they share, which
        // changes the topology.
        for center in [dvec3(1.5, 1.5, 1.0), dvec3(1.0, 1.6, 1.0)] {
            let cube = Shape::cube(2.0);
            let bite = Shape::sphere(1.2).at(center).build();
            let cut = cube.subtract(&bite).expect("cut succeeds");
            let result = cut.shape.move_faces([top_face(&cut.shape)], translation(dvec3(0.0, 0.3, 0.0)));
            assert!(result.is_err(), "moving one piece of a split top must fail");
        }
    }

    #[test]
    fn move_faces_in_compound_keeps_siblings() {
        let box_a = Shape::cube(2.0);
        let box_b = Shape::box_from_corners(dvec3(3.0, 0.0, 0.0), dvec3(5.0, 2.0, 2.0));
        let compound: Shape = Compound::from_shapes([&box_a, &box_b]).into();

        let moved = compound
            .move_faces([top_face(&box_a)], translation(dvec3(0.0, 1.0, 0.0)))
            .expect("moving a face of one child re-solves");

        assert_eq!(moved.shape_type(), ShapeType::Compound);
        assert_eq!(moved.sub_shapes().count(), 2, "the sibling solid must survive");
        assert_eq!(moved.faces().count(), 12);
        assert!((max_y(&moved) - 3.0).abs() < 1e-6, "the moved child's top must land at y=3");
        assert_volume(&moved, 12.0 + 8.0);
    }

    #[test]
    fn move_faces_across_compound_children() {
        let box_a = Shape::cube(2.0);
        let box_b = Shape::box_from_corners(dvec3(3.0, 0.0, 0.0), dvec3(5.0, 2.0, 2.0));
        let compound: Shape = Compound::from_shapes([&box_a, &box_b]).into();

        let moved = compound
            .move_faces([top_face(&box_a), top_face(&box_b)], translation(dvec3(0.0, 1.0, 0.0)))
            .expect("moving a face of each child re-solves");
        assert_volume(&moved, 24.0);
    }

    #[test]
    fn move_faces_after_fillet_does_not_panic() {
        // A rounded edge makes the top face's neighbours tangent-joined. Either
        // outcome (a re-solved body or an error) is acceptable; crashing is not.
        let cube = Shape::cube(2.0);
        let edge = cube.edges().next().expect("cube has edges");
        let filleted = cube.fillet_edge(0.2, &edge).expect("a small fillet on a cube succeeds");
        let top = top_face(&filleted);
        let mat = x_rotation_about(top.center_of_mass(), 0.2);
        if let Ok(moved) = filleted.move_faces([top], mat) {
            // fillet_edges wraps its result in a compound; the move keeps that shape.
            assert_eq!(moved.shape_type(), ShapeType::Compound);
            let mut children = moved.sub_shapes();
            assert_eq!(children.next().map(|c| c.shape_type()), Some(ShapeType::Solid));
            assert!(children.next().is_none());
        }
    }

    #[test]
    fn self_intersects_sees_overlapping_solids() {
        let cube = Shape::cube(2.0);
        assert!(!cube.self_intersects());
        let overlapping = Shape::box_from_corners(dvec3(1.0, 1.0, 1.0), dvec3(3.0, 3.0, 3.0));
        let compound: Shape = Compound::from_shapes([&cube, &overlapping]).into();
        assert!(compound.self_intersects());
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

    /// A prism of `profile` along +Z, and its side faces.
    fn prism_sides(profile: &Face, height: f64) -> (Shape, Vec<Face>, Face, Face) {
        let (solid, first, last) = profile.extrude_with_caps(dvec3(0.0, 0.0, height));
        let prism: Shape = solid.into();
        let sides = prism.faces().filter(|f| !f.is_same(&first) && !f.is_same(&last)).collect();
        (prism, sides, first, last)
    }

    /// Pulled along +Z from a neutral plane at its base, a square prism
    /// drafted by a positive angle narrows into a frustum; a negative one flares.
    #[test]
    fn drafting_the_sides_of_a_prism_tapers_it() {
        let profile = Workplane::xy().rect(2.0, 2.0).to_face().unwrap();
        let (height, angle) = (1.0, 10f64.to_radians());

        for sign in [1.0, -1.0] {
            let (prism, sides, _, last) = prism_sides(&profile, height);
            let (drafted, history) = prism
                .draft_faces_with_history(
                    &sides,
                    dvec3(0.0, 0.0, 1.0),
                    sign * angle,
                    dvec3(0.0, 0.0, 0.0),
                    dvec3(0.0, 0.0, 1.0),
                )
                .unwrap();

            // Every side moves in by h·tanθ at the top.
            let top = 2.0 - 2.0 * sign * height * angle.tan();
            let frustum = height / 3.0 * (4.0 + top * top + 2.0 * top);
            assert!(
                (drafted.volume() - frustum).abs() < 1e-6,
                "sign {sign}: expected {frustum}, got {}",
                drafted.volume()
            );

            // The history carries the top cap to its resized image.
            let images = history.modified_faces(&last);
            assert_eq!(images.len(), 1, "sign {sign}");
            assert!((images[0].surface_area() - top * top).abs() < 1e-6, "sign {sign}");
        }
    }

    /// A spline's swept side is a surface of linear extrusion, which the draft
    /// algorithm cannot tilt: that must come back as an error, not an abort.
    #[test]
    fn drafting_a_spline_side_is_an_error() {
        let spline = Edge::spline_from_points(
            [dvec3(0.0, 0.0, 0.0), dvec3(2.0, 0.5, 0.0), dvec3(1.5, 2.0, 0.0), dvec3(-0.5, 1.0, 0.0)],
            None,
            true,
        )
        .unwrap();
        let profile = Face::from_wire(&Wire::from_edges([&spline]).unwrap()).unwrap();
        let (prism, sides, _, _) = prism_sides(&profile, 1.0);

        let result = prism.draft_faces(
            &sides,
            dvec3(0.0, 0.0, 1.0),
            5f64.to_radians(),
            dvec3(0.0, 0.0, 0.0),
            dvec3(0.0, 0.0, 1.0),
        );
        assert!(
            matches!(result, Err(Error::DraftFailed(DraftError::FaceRecomputation))),
            "got {:?}",
            result.err()
        );
    }

    /// Open at both ends, a square prism hollowed inward is a square tube.
    #[test]
    fn hollowing_a_prism_open_at_both_ends_leaves_a_tube() {
        let profile = Workplane::xy().rect(2.0, 2.0).to_face().unwrap();
        let (prism, _, first, last) = prism_sides(&profile, 2.0);

        let tube =
            prism.hollow(-0.25, [first, last], JoinType::Intersection, OFFSET_TOLERANCE).unwrap();
        let expected = (4.0 - 1.5 * 1.5) * 2.0;
        assert!((tube.volume() - expected).abs() < 1e-6, "got {}", tube.volume());
    }

    #[test]
    fn a_hollow_that_cannot_be_built_is_an_error() {
        let result = Shape::cube(2.0).hollow(0.0, [] as [Face; 0], JoinType::Arc, OFFSET_TOLERANCE);
        assert!(
            matches!(result, Err(Error::OffsetFailed(OffsetError::NullOffset))),
            "got {:?}",
            result.err()
        );
    }

    /// Hollowed with no faces removed, a cube is one solid of two shells: the
    /// walls inside its faces for a negative offset, outside them for a
    /// positive one.
    #[test]
    fn hollowing_without_removing_faces_leaves_a_void() {
        let cube = Shape::cube(2.0);
        for (offset, expected) in [(-0.25, 8.0 - 1.5f64.powi(3)), (0.25, 2.5f64.powi(3) - 8.0)] {
            let hollow = cube
                .hollow(offset, [] as [Face; 0], JoinType::Intersection, OFFSET_TOLERANCE)
                .unwrap();
            assert_eq!(hollow.shape_type(), ShapeType::Solid);
            assert_eq!(hollow.sub_shapes().count(), 2, "offset {offset}: an outer and an inner shell");
            assert!((hollow.volume() - expected).abs() < 1e-6, "offset {offset}: got {}", hollow.volume());
        }
    }

    /// Offset either way, a cube grows or shrinks with sharp corners and stays
    /// a solid with its material inside.
    #[test]
    fn offsetting_a_cube_grows_or_shrinks_it() {
        let cube = Shape::cube(2.0);
        for (offset, expected) in [(0.5, 27.0), (-0.5, 1.0)] {
            let offset_cube =
                cube.offset_surface(offset, JoinType::Intersection, OFFSET_TOLERANCE).unwrap();
            assert_eq!(offset_cube.shape_type(), ShapeType::Solid);
            assert!((offset_cube.volume() - expected).abs() < 1e-6, "offset {offset}: got {}", offset_cube.volume());
        }
    }

    #[test]
    fn offsetting_a_sheet_moves_it_along_its_normal() {
        let sheet: Shape = Workplane::xy().rect(2.0, 3.0).to_face().unwrap().into();
        let normal = sheet.faces().next().unwrap().normal_at_center().unwrap().normalize();

        let moved = sheet.offset_surface(0.5, JoinType::Intersection, OFFSET_TOLERANCE).unwrap();
        assert_eq!(moved.shape_type(), ShapeType::Shell);
        let face = moved.faces().next().unwrap();
        assert!((face.center_of_mass() - normal * 0.5).length() < 1e-6, "got {}", face.center_of_mass());
        assert!((face.surface_area() - 6.0).abs() < 1e-6);
    }

    /// Grown to either side of the sheet, the slab is a solid with its
    /// material inside.
    #[test]
    fn thickening_a_sheet_makes_a_slab_either_way() {
        let sheet: Shape = Workplane::xy().rect(2.0, 3.0).to_face().unwrap().into();
        for offset in [0.5, -0.5] {
            let slab = sheet.thicken(offset, JoinType::Intersection, OFFSET_TOLERANCE).unwrap();
            assert_eq!(slab.shape_type(), ShapeType::Solid);
            assert!((slab.volume() - 3.0).abs() < 1e-6, "offset {offset}: got {}", slab.volume());
        }
    }

    /// The face of `shape` whose outward normal at its centre is `normal`.
    fn face_along(shape: &Shape, normal: DVec3) -> Face {
        shape
            .faces()
            .find(|face| face.normal_at_center().is_ok_and(|n| n.normalize().distance(normal) < 1e-9))
            .expect("a face matches")
    }

    /// A face of a solid thickens out of the material for a positive offset,
    /// whichever way the solid orients it.
    #[test]
    fn a_face_of_a_solid_thickens_out_of_the_material() {
        let cube = Shape::box_centered(2.0, 2.0, 2.0);
        for (index, face) in cube.faces().enumerate() {
            let outward = face.normal_at_center().unwrap().normalize();
            let slab =
                Shape::from(&face).thicken(0.5, JoinType::Intersection, OFFSET_TOLERANCE).unwrap();
            let aabb = crate::bounding_box::aabb(&slab);
            let centre = (aabb.min() + aabb.max()) * 0.5;
            assert!((centre.dot(outward) - 1.25).abs() < 1e-6, "face {index}: centre {centre}");
            assert!((slab.volume() - 2.0).abs() < 1e-6, "face {index}: got {}", slab.volume());
        }
    }

    /// Faces meeting at an edge thicken as one slab, joined across the corner.
    #[test]
    fn faces_meeting_at_an_edge_thicken_into_one_slab() {
        let cube = Shape::box_centered(2.0, 2.0, 2.0);
        let corner = Shell::from_faces([face_along(&cube, DVec3::Y), face_along(&cube, DVec3::X)]);

        let slab =
            Shape::from(corner).thicken(0.5, JoinType::Intersection, OFFSET_TOLERANCE).unwrap();
        // A 2×2×0.5 slab on each face, and the 2×0.5×0.5 block between them.
        assert!((slab.volume() - 4.5).abs() < 1e-6, "got {}", slab.volume());
    }

    /// Past a shape's own thickness, OCCT reports offsets as done with nothing,
    /// a collapsed solid or broken topology. Each is an error, never a shape
    /// to crash on.
    #[test]
    fn offsets_past_the_shapes_own_thickness_are_errors() {
        let cube = Shape::box_centered(2.0, 2.0, 2.0);
        let cylinder = Shape::cylinder_centered(DVec3::ZERO, 1.0, DVec3::Y, 4.0);
        let sphere = Shape::sphere(1.0).build();
        let offset = |shape: &Shape, by| {
            shape.offset_surface(by, JoinType::Intersection, OFFSET_TOLERANCE)
        };
        let hollow = |shape: &Shape, by, faces: &[Face]| {
            shape.hollow(by, faces, JoinType::Intersection, OFFSET_TOLERANCE)
        };
        let cases = [
            ("a cube collapsed to a point", offset(&cube, -1.0)),
            ("a cube turned inside out", offset(&cube, -1.1)),
            ("a box past its thinnest side", offset(&Shape::box_centered(2.0, 3.0, 5.0), -1.4)),
            ("a plate collapsed to a sheet", offset(&Shape::box_centered(4.0, 2.0, 4.0), -1.0)),
            ("a sphere collapsed to a point", offset(&sphere, -1.0)),
            ("a cylinder collapsed to its axis", offset(&cylinder, -1.0)),
            ("cup walls meeting", hollow(&cube, -1.0, &[face_along(&cube, DVec3::Y)])),
            ("closed walls meeting", hollow(&cube, -1.0, &[])),
            ("closed walls past the middle", hollow(&cube, -1.5, &[])),
            ("closed walls past a cylinder's axis", hollow(&cylinder, -1.2, &[])),
            ("closed walls past a sphere's centre", hollow(&sphere, -1.2, &[])),
        ];
        for (case, result) in cases {
            assert!(matches!(result, Err(Error::OffsetFailed(_))), "{case}: got {:?}", result.map(|shape| shape.volume()));
        }
    }

    /// Closed walls short of meeting still build, however close they come.
    #[test]
    fn closed_walls_short_of_meeting_build() {
        let sphere = Shape::sphere(1.0).build();
        let hollow = sphere
            .hollow(-0.9, [] as [Face; 0], JoinType::Intersection, OFFSET_TOLERANCE)
            .unwrap();
        let expected = 4.0 / 3.0 * std::f64::consts::PI * (1.0 - 0.1f64.powi(3));
        assert!((hollow.volume() - expected).abs() < 1e-4, "got {}", hollow.volume());
    }

    #[test]
    fn faces_that_do_not_touch_cannot_thicken_together() {
        let cube = Shape::box_centered(2.0, 2.0, 2.0);
        let apart = Shell::from_faces([face_along(&cube, DVec3::Y), face_along(&cube, DVec3::NEG_Y)]);

        let result = Shape::from(apart).thicken(0.5, JoinType::Intersection, OFFSET_TOLERANCE);
        assert!(
            matches!(result, Err(Error::OffsetFailed(OffsetError::NotConnectedShell))),
            "got {:?}",
            result.err()
        );
    }

    /// Rounded to radius r, an edge of the 2³ cube loses a groove of r² less a
    /// quarter circle, all along its length of 2.
    #[test]
    fn filleting_a_cube_edge_rounds_it() {
        let cube = Shape::cube(2.0);
        let edge = cube.edges().next().unwrap();
        let radius = 0.5;

        let filleted = cube.fillet_edge(radius, &edge).unwrap();
        let expected = 8.0 - 2.0 * radius * radius * (1.0 - std::f64::consts::FRAC_PI_4);
        assert!((filleted.volume() - expected).abs() < 1e-6, "got {}", filleted.volume());
    }

    /// Bevelled back d along both faces, an edge of the 2³ cube loses a
    /// right-angled prism with legs d, all along its length of 2.
    #[test]
    fn chamfering_a_cube_edge_bevels_it() {
        let cube = Shape::cube(2.0);
        let edge = cube.edges().next().unwrap();
        let distance = 0.5;

        let chamfered = cube.chamfer_edge(distance, &edge).unwrap();
        let expected = 8.0 - distance * distance;
        assert!((chamfered.volume() - expected).abs() < 1e-6, "got {}", chamfered.volume());
    }

    /// A radius far wider than the faces beside the edge is reported, not
    /// aborted on.
    #[test]
    fn an_oversized_fillet_is_an_error() {
        let cube = Shape::cube(2.0);
        let edge = cube.edges().next().unwrap();

        let result = cube.fillet_edge(5.0, &edge);
        assert!(matches!(result, Err(Error::FilletFailed(_))), "got {:?}", result.err());
    }

    #[test]
    fn an_oversized_chamfer_is_an_error() {
        let cube = Shape::cube(2.0);
        let edge = cube.edges().next().unwrap();

        let result = cube.chamfer_edge(5.0, &edge);
        assert!(matches!(result, Err(Error::FilletFailed(_))), "got {:?}", result.err());
    }

    /// Past half the cube's side, neighbouring fillets overlap. OCCT reports
    /// the build as done; the broken shape it made must not get out.
    #[test]
    fn overlapping_fillets_are_an_error() {
        let result = Shape::cube(2.0).fillet(1.5);
        assert!(
            matches!(result, Err(Error::FilletFailed(FilletError::InvalidResult))),
            "got {:?}",
            result.err()
        );
    }

    /// A lone face's edges border only that face: there is no corner to round
    /// or bevel.
    #[test]
    fn a_free_edge_cannot_be_filleted_or_chamfered() {
        let sheet: Shape = Workplane::xy().rect(2.0, 2.0).to_face().unwrap().into();
        let edge = sheet.edges().next().unwrap();

        let filleted = sheet.fillet_edge(0.2, &edge);
        assert!(
            matches!(filleted, Err(Error::FilletFailed(FilletError::NoSuitableEdges))),
            "got {:?}",
            filleted.err()
        );
        let chamfered = sheet.chamfer_edge(0.2, &edge);
        assert!(
            matches!(chamfered, Err(Error::FilletFailed(FilletError::NoSuitableEdges))),
            "got {:?}",
            chamfered.err()
        );
    }

    #[test]
    fn an_edge_between_two_faces_has_both() {
        let cube = Shape::cube(2.0);
        let edge = cube.edges().next().unwrap();

        let faces = cube.adjacent_faces(&edge);
        assert_eq!(faces.len(), 2);
        assert!(!faces[0].is_same(&faces[1]));
        assert!(faces.iter().all(|face| face.edges().any(|e| e.is_same(&edge))));
    }

    /// A cylinder's seam runs down its side face twice, but that is one face.
    #[test]
    fn a_seam_edge_has_one_face() {
        let cylinder = Shape::cylinder_radius_height(1.0, 2.0);
        let seam = cylinder.seam_edges().into_iter().next().expect("a cylinder has a seam");

        assert_eq!(cylinder.adjacent_faces(&seam).len(), 1);
    }

    #[test]
    fn an_edge_of_another_shape_has_no_faces() {
        let cube = Shape::cube(2.0);
        let other = Shape::cube(1.0);
        let edge = other.edges().next().unwrap();

        assert!(cube.adjacent_faces(&edge).is_empty());
    }

    /// The edge of `shape` through `point`.
    fn edge_at(shape: &Shape, point: DVec3) -> Edge {
        shape.edges().find(|edge| edge.midpoint().distance(point) < 1e-9).expect("an edge runs through the point")
    }

    /// The circular edge of `shape` at height `z`.
    fn circle_at(shape: &Shape, z: f64) -> Edge {
        shape
            .edges()
            .find(|edge| edge.edge_type() == EdgeType::Circle && (edge.start_point().z - z).abs() < 1e-9)
            .expect("a circle at that height")
    }

    #[test]
    fn a_box_edge_is_an_outside_right_angle_corner() {
        let cube = Shape::box_centered(2.0, 2.0, 2.0);
        let edge = edge_at(&cube, dvec3(0.0, -1.0, 1.0));
        let corner = cube.corner(&edge).expect("an edge between two faces");

        assert!(corner.point.distance(dvec3(0.0, -1.0, 1.0)) < 1e-9);
        assert!((corner.tangent.x.abs() - 1.0).abs() < 1e-12);
        assert!((corner.normal_angle() - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        for (face, normal) in corner.faces.iter().zip(corner.normals) {
            assert!(face.edges().any(|bounding| bounding.is_same(&edge)), "the faces are the edge's own");
            let expected = face.normal_at_center().unwrap();
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
        let step = slab.union(&wall).unwrap().shape.clean().unwrap();
        let corner = step.corner(&edge_at(&step, dvec3(1.0, 1.0, 1.0))).expect("an edge between two faces");
        assert_eq!(corner.convex, Some(false));
    }

    /// The corner test works across curved faces: a rod's rim, a bore's rim
    /// and the foot of a boss.
    #[test]
    fn curved_corners_tell_outside_from_inside() {
        let rod = Shape::cylinder_radius_height(1.0, 2.0);
        assert_eq!(rod.corner(&circle_at(&rod, 2.0)).and_then(|corner| corner.convex), Some(true));

        let block = Shape::box_from_corners(dvec3(-2.0, -2.0, 0.0), dvec3(2.0, 2.0, 2.0));
        let bore = Shape::cylinder(dvec3(0.0, 0.0, -1.0), 1.0, DVec3::Z, 4.0);
        let bored = block.subtract(&bore).unwrap().shape;
        assert_eq!(bored.corner(&circle_at(&bored, 2.0)).and_then(|corner| corner.convex), Some(true));

        let boss = Shape::cylinder(dvec3(0.0, 0.0, 2.0), 1.0, DVec3::Z, 1.0);
        let bossed = block.union(&boss).unwrap().shape.clean().unwrap();
        assert_eq!(bossed.corner(&circle_at(&bossed, 2.0)).and_then(|corner| corner.convex), Some(false));
    }

    #[test]
    fn a_sheet_edge_has_no_corner() {
        let sheet: Shape = Face::from_wire(&Wire::rect(2.0, 2.0).unwrap()).unwrap().into();
        let edge = sheet.edges().next().unwrap();
        assert!(sheet.corner(&edge).is_none());
    }

    /// A unit square in XY, spun a full turn about its edge on the Y axis.
    #[test]
    fn a_square_revolved_about_its_edge_is_a_cylinder() {
        let wire = Wire::from_ordered_points([
            dvec3(0.0, 0.0, 0.0),
            dvec3(1.0, 0.0, 0.0),
            dvec3(1.0, 1.0, 0.0),
            dvec3(0.0, 1.0, 0.0),
        ])
        .unwrap();
        let square: Shape = Face::from_wire(&wire).unwrap().into();

        let full = square.revolve(DVec3::ZERO, DVec3::Y, std::f64::consts::TAU).expect("revolves");
        assert!(full.contains_type(ShapeType::Solid));
        assert!((full.volume() - std::f64::consts::PI).abs() < 1e-6, "{}", full.volume());

        let quarter = square.revolve(DVec3::ZERO, DVec3::Y, std::f64::consts::FRAC_PI_2).expect("revolves");
        assert!((quarter.volume() - std::f64::consts::FRAC_PI_4).abs() < 1e-6, "{}", quarter.volume());
    }

    #[test]
    fn a_zero_revolution_is_refused() {
        let edge: Shape = Edge::segment(dvec3(1.0, 0.0, 0.0), dvec3(1.0, 1.0, 0.0)).unwrap().into();
        assert!(matches!(edge.revolve(DVec3::ZERO, DVec3::Y, 0.0), Err(Error::RevolveFailed(_))));
    }
}
