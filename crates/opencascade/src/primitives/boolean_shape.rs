use cxx::UniquePtr;
use opencascade_sys::ffi;
use std::ops::{Deref, DerefMut};

use crate::history::ShapeHistory;
use crate::primitives::{Edge, Shape};
use crate::Error;

/// The result of running a boolean operation (union, subtraction, intersection)
/// on two shapes.
pub struct BooleanShape {
    pub shape: Shape,
    pub new_edges: Vec<Edge>,
    /// Sub-shape history of the operation: input sub-shapes → result sub-shapes.
    pub history: ShapeHistory,
    /// The algorithm's warning report (`BOPAlgo_Options::DumpWarnings`), or
    /// `None` when it completed clean. Warnings flag degenerate input
    /// configurations whose result may be wrong despite reported success.
    pub warnings: Option<String>,
}

impl Deref for BooleanShape {
    type Target = Shape;

    fn deref(&self) -> &Self::Target {
        &self.shape
    }
}

impl DerefMut for BooleanShape {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.shape
    }
}

impl BooleanShape {
    pub fn new_edges(&self) -> impl Iterator<Item = &Edge> {
        self.new_edges.iter()
    }

    #[must_use]
    pub fn fillet_new_edges(&self, radius: f64) -> Shape {
        self.shape.fillet_edges(radius, &self.new_edges)
    }

    #[must_use]
    pub fn variable_fillet_new_edges(
        &self,
        radius_values: impl IntoIterator<Item = (f64, f64)>,
    ) -> Shape {
        self.shape.variable_fillet_edges(radius_values, &self.new_edges)
    }

    #[must_use]
    pub fn chamfer_new_edges(&self, distance: f64) -> Shape {
        self.shape.chamfer_edges(distance, &self.new_edges)
    }
}

fn edges_from_list(list: &ffi::TopTools_ListOfShape) -> Vec<Edge> {
    ffi::shape_list_to_vector(list)
        .iter()
        .map(|shape| Edge::from_edge(ffi::TopoDS_cast_to_edge(shape)))
        .collect()
}

fn non_empty(report: String) -> Option<String> {
    let trimmed = report.trim();
    (!trimmed.is_empty()).then(|| trimmed.to_owned())
}

/// The shared body of the boolean operations on [`Shape`] and `Solid`.
/// Errs (instead of raising an uncatchable OCCT exception on `Shape()`)
/// when the algorithm could not complete.
///
/// `fuzz` is the additional intersection tolerance
/// (`BOPAlgo_Options::SetFuzzyValue`); values at or below OCCT's default
/// (`Precision::Confusion`, 1e-7) leave the default in place.
pub(crate) fn cut(
    shape: &ffi::TopoDS_Shape,
    tool: &ffi::TopoDS_Shape,
    fuzz: f64,
) -> Result<BooleanShape, Error> {
    let mut operation = ffi::BRepAlgoAPI_Cut_ctor_empty();
    operation.pin_mut().SetArguments(single(shape).as_ref().unwrap());
    operation.pin_mut().SetTools(single(tool).as_ref().unwrap());
    ffi::BRepAlgoAPI_Cut_set_fuzzy_value(operation.pin_mut(), fuzz);
    operation.pin_mut().Build(&ffi::Message_ProgressRange_ctor());
    cut_result(operation)
}

pub(crate) fn fuse(
    shape: &ffi::TopoDS_Shape,
    tool: &ffi::TopoDS_Shape,
    fuzz: f64,
) -> Result<BooleanShape, Error> {
    let mut operation = ffi::BRepAlgoAPI_Fuse_ctor_empty();
    operation.pin_mut().SetArguments(single(shape).as_ref().unwrap());
    operation.pin_mut().SetTools(single(tool).as_ref().unwrap());
    ffi::BRepAlgoAPI_Fuse_set_fuzzy_value(operation.pin_mut(), fuzz);
    operation.pin_mut().Build(&ffi::Message_ProgressRange_ctor());
    fuse_result(operation)
}

pub(crate) fn common(
    shape: &ffi::TopoDS_Shape,
    tool: &ffi::TopoDS_Shape,
    fuzz: f64,
) -> Result<BooleanShape, Error> {
    let mut operation = ffi::BRepAlgoAPI_Common_ctor_empty();
    operation.pin_mut().SetArguments(single(shape).as_ref().unwrap());
    operation.pin_mut().SetTools(single(tool).as_ref().unwrap());
    ffi::BRepAlgoAPI_Common_set_fuzzy_value(operation.pin_mut(), fuzz);
    operation.pin_mut().Build(&ffi::Message_ProgressRange_ctor());
    common_result(operation)
}

/// The outcome of a built cut.
fn cut_result(mut operation: UniquePtr<ffi::BRepAlgoAPI_Cut>) -> Result<BooleanShape, Error> {
    if !operation.IsDone() {
        return Err(Error::BooleanFailed("cut", ffi::BRepAlgoAPI_Cut_errors(&operation)));
    }
    let warnings = non_empty(ffi::BRepAlgoAPI_Cut_warnings(&operation));
    let new_edges = edges_from_list(operation.pin_mut().SectionEdges());
    let shape = Shape::from_shape(operation.pin_mut().Shape());
    let history = ShapeHistory::from_handle(ffi::BRepAlgoAPI_Cut_history(&operation));
    Ok(BooleanShape { shape, new_edges, history, warnings })
}

/// The outcome of a built fuse.
fn fuse_result(mut operation: UniquePtr<ffi::BRepAlgoAPI_Fuse>) -> Result<BooleanShape, Error> {
    if !operation.IsDone() {
        return Err(Error::BooleanFailed("fuse", ffi::BRepAlgoAPI_Fuse_errors(&operation)));
    }
    let warnings = non_empty(ffi::BRepAlgoAPI_Fuse_warnings(&operation));
    let new_edges = edges_from_list(operation.pin_mut().SectionEdges());
    let shape = Shape::from_shape(operation.pin_mut().Shape());
    let history = ShapeHistory::from_handle(ffi::BRepAlgoAPI_Fuse_history(&operation));
    Ok(BooleanShape { shape, new_edges, history, warnings })
}

/// The outcome of a built common.
fn common_result(
    mut operation: UniquePtr<ffi::BRepAlgoAPI_Common>,
) -> Result<BooleanShape, Error> {
    if !operation.IsDone() {
        return Err(Error::BooleanFailed("common", ffi::BRepAlgoAPI_Common_errors(&operation)));
    }
    let warnings = non_empty(ffi::BRepAlgoAPI_Common_warnings(&operation));
    let new_edges = edges_from_list(operation.pin_mut().SectionEdges());
    let shape = Shape::from_shape(operation.pin_mut().Shape());
    let history = ShapeHistory::from_handle(ffi::BRepAlgoAPI_Common_history(&operation));
    Ok(BooleanShape { shape, new_edges, history, warnings })
}

fn single(shape: &ffi::TopoDS_Shape) -> UniquePtr<ffi::TopTools_ListOfShape> {
    let mut list = ffi::new_list_of_shape();
    ffi::shape_list_append_shape(list.pin_mut(), shape);
    list
}

/// Two shapes intersected once (`BOPAlgo_PaveFiller`), from which each boolean
/// between them is built without intersecting them again.
///
/// Like the booleans on [`Shape`], the intersection may modify its inputs
/// (tolerances, added p-curves); pass deep copies to keep the originals intact.
pub struct BooleanPair {
    filler: UniquePtr<ffi::BOPAlgo_PaveFiller>,
    object: Shape,
    tool: Shape,
}

impl BooleanPair {
    /// Intersect `object` with `tool`. `fuzz` is the additional intersection
    /// tolerance, as for [`Shape::subtract_with_fuzz`].
    pub fn new(object: &Shape, tool: &Shape, fuzz: f64) -> Result<Self, Error> {
        let mut arguments = ffi::new_list_of_shape();
        ffi::shape_list_append_shape(arguments.pin_mut(), &object.inner);
        ffi::shape_list_append_shape(arguments.pin_mut(), &tool.inner);

        let mut filler = ffi::BOPAlgo_PaveFiller_ctor();
        filler.pin_mut().SetArguments(&arguments);
        filler.pin_mut().SetFuzzyValue(fuzz);
        filler.pin_mut().Perform(&ffi::Message_ProgressRange_ctor());
        if filler.HasErrors() {
            return Err(Error::BooleanFailed(
                "intersection",
                ffi::BOPAlgo_PaveFiller_errors(&filler),
            ));
        }
        // The operations must be given the very shapes the filler intersected.
        Ok(Self { filler, object: object.clone(), tool: tool.clone() })
    }

    // Each operation borrows the filler, so it is built and dropped within the
    // call that uses it.

    /// `object ∩ tool`.
    pub fn intersect(&self) -> Result<BooleanShape, Error> {
        common_result(ffi::BRepAlgoAPI_Common_ctor_with_filler(
            &self.object.inner,
            &self.tool.inner,
            &self.filler,
        ))
    }

    /// `object − tool`.
    pub fn subtract(&self) -> Result<BooleanShape, Error> {
        cut_result(ffi::BRepAlgoAPI_Cut_ctor_with_filler(
            &self.object.inner,
            &self.tool.inner,
            &self.filler,
            true,
        ))
    }

    /// `tool − object`.
    pub fn subtract_reversed(&self) -> Result<BooleanShape, Error> {
        cut_result(ffi::BRepAlgoAPI_Cut_ctor_with_filler(
            &self.object.inner,
            &self.tool.inner,
            &self.filler,
            false,
        ))
    }

    /// `object ∪ tool`.
    pub fn union(&self) -> Result<BooleanShape, Error> {
        fuse_result(ffi::BRepAlgoAPI_Fuse_ctor_with_filler(
            &self.object.inner,
            &self.tool.inner,
            &self.filler,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::BooleanPair;
    use crate::primitives::Shape;
    use glam::dvec3;
    use std::f64::consts::PI;

    fn assert_volume(shape: &Shape, expected: f64) {
        let volume = shape.volume();
        assert!(
            (volume - expected).abs() < 1e-6 * expected,
            "expected volume {expected}, got {volume}"
        );
    }

    /// A 2-unit cube and a unit sphere centered on its far corner, which share
    /// an eighth of the sphere.
    #[test]
    fn pair_builds_every_boolean_from_one_intersection() {
        let cube = Shape::cube(2.0);
        let sphere = Shape::sphere(1.0).at(dvec3(2.0, 2.0, 2.0)).build();
        let (cube_volume, sphere_volume, shared) = (8.0, 4.0 / 3.0 * PI, PI / 6.0);

        let pair = BooleanPair::new(&cube, &sphere, 0.0).expect("intersection succeeds");

        assert_volume(&pair.intersect().expect("intersect"), shared);
        assert_volume(&pair.subtract().expect("subtract"), cube_volume - shared);
        assert_volume(&pair.subtract_reversed().expect("subtract_reversed"), sphere_volume - shared);
        assert_volume(&pair.union().expect("union"), cube_volume + sphere_volume - shared);
    }
}
