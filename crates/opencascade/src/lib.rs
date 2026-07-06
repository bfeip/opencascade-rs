use opencascade_sys::ffi;
use thiserror::Error;

pub mod angle;
pub mod bounding_box;
pub mod kicad;
pub mod mesh;
pub mod primitives;
pub mod section;
pub mod workplane;
pub mod xcaf;

mod law_function;
mod make_pipe_shell;

#[derive(Error, Debug)]
pub enum Error {
    #[error("failed to write STL file")]
    StlWriteFailed,
    #[error("failed to read STEP file")]
    StepReadFailed,
    #[error("failed to read IGES file")]
    IgesReadFailed,
    #[error("failed to read KiCAD PCB file: {0}")]
    KicadReadFailed(#[from] kicad_parser::Error),
    #[error("failed to write STEP file")]
    StepWriteFailed,
    #[error("failed to write IGES file")]
    IgesWriteFailed,
    #[error("failed to read BREP file")]
    BrepReadFailed,
    #[error("failed to write BREP file")]
    BrepWriteFailed,
    #[error("failed to triangulate Shape")]
    TriangulationFailed,
    #[error("encountered a face with no triangulation")]
    UntriangulatedFace,
    #[error("at least 2 points are required for creating a wire")]
    NotEnoughPoints,
    #[error("consecutive spline points at indices {0} and {1} are identical")]
    IdenticalSplinePoints(usize, usize),
    #[error("failed to build edge: {0:?}")]
    EdgeFailed(EdgeError),
    #[error("failed to build wire: {0:?}")]
    WireFailed(WireError),
    #[error("failed to build face: {0:?}")]
    FaceFailed(FaceError),
    #[error("transform is not a similarity (rotation + translation + uniform scale)")]
    NotASimilarityTransform,
    #[error("failed to tweak faces: {0}")]
    TweakFailed(String),
}

/// Reason a `BRepBuilderAPI_MakeEdge` failed to produce an edge.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum EdgeError {
    PointProjectionFailed,
    ParameterOutOfRange,
    DifferentPointsOnClosedCurve,
    PointWithInfiniteParameter,
    DifferentPointAndParameter,
    LineThroughIdenticalPoints,
    /// An error code not recognized by this wrapper (including a spurious "done").
    Unknown,
}

impl From<ffi::BRepBuilderAPI_EdgeError> for EdgeError {
    fn from(error: ffi::BRepBuilderAPI_EdgeError) -> Self {
        use ffi::BRepBuilderAPI_EdgeError as E;
        match error {
            E::BRepBuilderAPI_PointProjectionFailed => Self::PointProjectionFailed,
            E::BRepBuilderAPI_ParameterOutOfRange => Self::ParameterOutOfRange,
            E::BRepBuilderAPI_DifferentPointsOnClosedCurve => Self::DifferentPointsOnClosedCurve,
            E::BRepBuilderAPI_PointWithInfiniteParameter => Self::PointWithInfiniteParameter,
            E::BRepBuilderAPI_DifferentsPointAndParameter => Self::DifferentPointAndParameter,
            E::BRepBuilderAPI_LineThroughIdenticPoints => Self::LineThroughIdenticalPoints,
            _ => Self::Unknown,
        }
    }
}

/// Reason a `BRepBuilderAPI_MakeWire` failed to produce a wire.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum WireError {
    EmptyWire,
    DisconnectedWire,
    NonManifoldWire,
    /// An error code not recognized by this wrapper (including a spurious "done").
    Unknown,
}

impl From<ffi::BRepBuilderAPI_WireError> for WireError {
    fn from(error: ffi::BRepBuilderAPI_WireError) -> Self {
        use ffi::BRepBuilderAPI_WireError as E;
        match error {
            E::BRepBuilderAPI_EmptyWire => Self::EmptyWire,
            E::BRepBuilderAPI_DisconnectedWire => Self::DisconnectedWire,
            E::BRepBuilderAPI_NonManifoldWire => Self::NonManifoldWire,
            _ => Self::Unknown,
        }
    }
}

/// Reason a `BRepBuilderAPI_MakeFace` failed to produce a face.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum FaceError {
    NoFace,
    NotPlanar,
    CurveProjectionFailed,
    ParametersOutOfRange,
    /// An error code not recognized by this wrapper (including a spurious "done").
    Unknown,
}

impl From<ffi::BRepBuilderAPI_FaceError> for FaceError {
    fn from(error: ffi::BRepBuilderAPI_FaceError) -> Self {
        use ffi::BRepBuilderAPI_FaceError as E;
        match error {
            E::BRepBuilderAPI_NoFace => Self::NoFace,
            E::BRepBuilderAPI_NotPlanar => Self::NotPlanar,
            E::BRepBuilderAPI_CurveProjectionFailed => Self::CurveProjectionFailed,
            E::BRepBuilderAPI_ParametersOutOfRange => Self::ParametersOutOfRange,
            _ => Self::Unknown,
        }
    }
}
