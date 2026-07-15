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
    if !operation.IsDone() {
        return Err(Error::BooleanFailed("cut", ffi::BRepAlgoAPI_Cut_errors(&operation)));
    }
    let warnings = non_empty(ffi::BRepAlgoAPI_Cut_warnings(&operation));
    let new_edges = edges_from_list(operation.pin_mut().SectionEdges());
    let shape = Shape::from_shape(operation.pin_mut().Shape());
    let history = ShapeHistory::from_handle(ffi::BRepAlgoAPI_Cut_history(&operation));
    Ok(BooleanShape { shape, new_edges, history, warnings })
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
    if !operation.IsDone() {
        return Err(Error::BooleanFailed("fuse", ffi::BRepAlgoAPI_Fuse_errors(&operation)));
    }
    let warnings = non_empty(ffi::BRepAlgoAPI_Fuse_warnings(&operation));
    let new_edges = edges_from_list(operation.pin_mut().SectionEdges());
    let shape = Shape::from_shape(operation.pin_mut().Shape());
    let history = ShapeHistory::from_handle(ffi::BRepAlgoAPI_Fuse_history(&operation));
    Ok(BooleanShape { shape, new_edges, history, warnings })
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
    if !operation.IsDone() {
        return Err(Error::BooleanFailed("common", ffi::BRepAlgoAPI_Common_errors(&operation)));
    }
    let warnings = non_empty(ffi::BRepAlgoAPI_Common_warnings(&operation));
    let new_edges = edges_from_list(operation.pin_mut().SectionEdges());
    let shape = Shape::from_shape(operation.pin_mut().Shape());
    let history = ShapeHistory::from_handle(ffi::BRepAlgoAPI_Common_history(&operation));
    Ok(BooleanShape { shape, new_edges, history, warnings })
}

fn single(shape: &ffi::TopoDS_Shape) -> cxx::UniquePtr<ffi::TopTools_ListOfShape> {
    let mut list = ffi::new_list_of_shape();
    ffi::shape_list_append_shape(list.pin_mut(), shape);
    list
}
