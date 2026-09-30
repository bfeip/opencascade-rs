use opencascade_sys::ffi;
use thiserror::Error;

pub mod angle;
pub mod bounding_box;
pub mod history;
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
    #[error("failed to project point onto surface: {0}")]
    SurfaceProjectionFailed(String),
    #[error("transform is not a similarity (rotation + translation + uniform scale)")]
    NotASimilarityTransform,
    #[error("failed to tweak faces: {0}")]
    TweakFailed(String),
    #[error("failed to unify same-domain geometry: {0}")]
    CleanFailed(String),
    #[error("boolean {0} operation failed: {1}")]
    BooleanFailed(&'static str, String),
    #[error("failed to apply draft angle: {0:?}")]
    DraftFailed(DraftError),
    #[error("failed to offset shape: {0:?}")]
    OffsetFailed(OffsetError),
    #[error("failed to fillet or chamfer edges: {0:?}")]
    FilletFailed(FilletError),
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

/// Reason a `BRepOffsetAPI_DraftAngle` failed to taper a face.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum DraftError {
    /// A face could not take the draft. Only planar, cylindrical and conical
    /// faces can, and a cylinder or cone only along its axis.
    FaceRecomputation,
    EdgeRecomputation,
    VertexRecomputation,
    /// An error code not recognized by this wrapper (including a spurious "no error").
    Unknown,
}

impl From<ffi::Draft_ErrorStatus> for DraftError {
    fn from(error: ffi::Draft_ErrorStatus) -> Self {
        use ffi::Draft_ErrorStatus as E;
        match error {
            E::Draft_FaceRecomputation => Self::FaceRecomputation,
            E::Draft_EdgeRecomputation => Self::EdgeRecomputation,
            E::Draft_VertexRecomputation => Self::VertexRecomputation,
            _ => Self::Unknown,
        }
    }
}

/// Reason a `BRepOffset_MakeOffset` failed to offset a shape.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum OffsetError {
    BadNormalsOnGeometry,
    C0Geometry,
    NullOffset,
    NotConnectedShell,
    CannotTrimEdges,
    CannotFuseVertices,
    CannotExtentEdge,
    UserBreak,
    MixedConnectivity,
    /// An unspecified failure, or an error code not recognized by this wrapper
    /// (including a spurious "no error").
    Unknown,
}

impl From<ffi::BRepOffset_Error> for OffsetError {
    fn from(error: ffi::BRepOffset_Error) -> Self {
        use ffi::BRepOffset_Error as E;
        match error {
            E::BRepOffset_BadNormalsOnGeometry => Self::BadNormalsOnGeometry,
            E::BRepOffset_C0Geometry => Self::C0Geometry,
            E::BRepOffset_NullOffset => Self::NullOffset,
            E::BRepOffset_NotConnectedShell => Self::NotConnectedShell,
            E::BRepOffset_CannotTrimEdges => Self::CannotTrimEdges,
            E::BRepOffset_CannotFuseVertices => Self::CannotFuseVertices,
            E::BRepOffset_CannotExtentEdge => Self::CannotExtentEdge,
            E::BRepOffset_UserBreak => Self::UserBreak,
            E::BRepOffset_MixedConnectivity => Self::MixedConnectivity,
            _ => Self::Unknown,
        }
    }
}

/// Reason a `BRepFilletAPI_MakeFillet` or `BRepFilletAPI_MakeChamfer` failed.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum FilletError {
    /// None of the edges borders two faces of the shape.
    NoSuitableEdges,
    /// The blend could not be traced along an edge, typically because it is
    /// too large for the faces beside it.
    WalkingFailure,
    /// No starting section could be found for the blend.
    StartSolutionFailure,
    /// The blend surface would twist over itself.
    TwistedSurface,
    /// The build finished but its result is not a valid shape, typically
    /// because neighboring blends overlap.
    InvalidResult,
    /// An unspecified failure, or an error code not recognized by this wrapper
    /// (including a spurious "no error").
    Unknown,
}

impl From<ffi::ChFiDS_ErrorStatus> for FilletError {
    fn from(error: ffi::ChFiDS_ErrorStatus) -> Self {
        use ffi::ChFiDS_ErrorStatus as E;
        match error {
            E::ChFiDS_WalkingFailure => Self::WalkingFailure,
            E::ChFiDS_StartsolFailure => Self::StartSolutionFailure,
            E::ChFiDS_TwistedSurface => Self::TwistedSurface,
            _ => Self::Unknown,
        }
    }
}
