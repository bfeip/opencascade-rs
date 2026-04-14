use cxx::UniquePtr;
use opencascade_sys::ffi;
use std::path::Path;

use crate::{primitives::Shape, Error};

/// An XDE/XCAF document. Loaded via a `STEPCAFControl_Reader` or `IGESCAFControl_Reader`
/// and provides access to shapes, assembly structure, names, and colors.
pub struct XcafDocument {
    pub(crate) inner: UniquePtr<ffi::HandleTDocStd_Document>,
}

impl XcafDocument {
    /// Create an empty document.
    pub fn new() -> Self {
        Self { inner: ffi::xcaf_new_document() }
    }

    /// Read a STEP file with full metadata (names, colors, layers).
    pub fn read_step(path: impl AsRef<Path>) -> Result<Self, Error> {
        let mut doc = Self::new();
        let path_str = path.as_ref().to_string_lossy().into_owned();

        let mut reader = ffi::new_STEPCAFControl_Reader();
        reader.pin_mut().SetColorMode(true);
        reader.pin_mut().SetNameMode(true);
        reader.pin_mut().SetLayerMode(true);
        reader.pin_mut().SetGDTMode(true);

        let status = ffi::xcaf_step_read_file(reader.pin_mut(), path_str);
        if status != ffi::IFSelect_ReturnStatus::IFSelect_RetDone {
            return Err(Error::StepReadFailed);
        }
        if !ffi::xcaf_step_transfer(reader.pin_mut(), doc.inner.pin_mut()) {
            return Err(Error::StepReadFailed);
        }
        Ok(doc)
    }

    /// Read STEP data from a string with full metadata (names, colors, layers).
    pub fn read_step_from_str(s: &str) -> Result<Self, Error> {
        let mut doc = Self::new();
        let mut reader = ffi::new_STEPCAFControl_Reader();
        reader.pin_mut().SetColorMode(true);
        reader.pin_mut().SetNameMode(true);
        reader.pin_mut().SetLayerMode(true);
        reader.pin_mut().SetGDTMode(true);
        let status = ffi::xcaf_step_read_str(reader.pin_mut(), s);
        if status != ffi::IFSelect_ReturnStatus::IFSelect_RetDone {
            return Err(Error::StepReadFailed);
        }
        if !ffi::xcaf_step_transfer(reader.pin_mut(), doc.inner.pin_mut()) {
            return Err(Error::StepReadFailed);
        }
        Ok(doc)
    }

    /// Read IGES data from a string with full metadata (names, colors).
    pub fn read_iges_from_str(s: &str) -> Result<Self, Error> {
        let mut doc = Self::new();
        let mut reader = ffi::new_IGESCAFControl_Reader();
        reader.pin_mut().SetColorMode(true);
        reader.pin_mut().SetNameMode(true);
        let status = ffi::xcaf_iges_read_str(reader.pin_mut(), s);
        if status != ffi::IFSelect_ReturnStatus::IFSelect_RetDone {
            return Err(Error::IgesReadFailed);
        }
        if !ffi::xcaf_iges_transfer(reader.pin_mut(), doc.inner.pin_mut()) {
            return Err(Error::IgesReadFailed);
        }
        Ok(doc)
    }

    /// Read an IGES file with full metadata (names, colors).
    pub fn read_iges(path: impl AsRef<Path>) -> Result<Self, Error> {
        let mut doc = Self::new();
        let path_str = path.as_ref().to_string_lossy().into_owned();

        let mut reader = ffi::new_IGESCAFControl_Reader();
        reader.pin_mut().SetColorMode(true);
        reader.pin_mut().SetNameMode(true);

        let status = ffi::xcaf_iges_read_file(reader.pin_mut(), path_str);
        if status != ffi::IFSelect_ReturnStatus::IFSelect_RetDone {
            return Err(Error::IgesReadFailed);
        }
        if !ffi::xcaf_iges_transfer(reader.pin_mut(), doc.inner.pin_mut()) {
            return Err(Error::IgesReadFailed);
        }
        Ok(doc)
    }

    pub fn shape_tool(&self) -> XcafShapeTool {
        XcafShapeTool { inner: ffi::xcaf_shape_tool(&self.inner) }
    }

    pub fn color_tool(&self) -> XcafColorTool {
        XcafColorTool { inner: ffi::xcaf_color_tool(&self.inner) }
    }

    pub fn dim_tol_tool(&self) -> XcafDimTolTool {
        XcafDimTolTool { inner: ffi::xcaf_dimtol_tool(&self.inner) }
    }

    pub fn clipping_plane_tool(&self) -> XcafClippingPlaneTool {
        XcafClippingPlaneTool { inner: ffi::xcaf_clipping_plane_tool(&self.inner) }
    }

    pub fn view_tool(&self) -> XcafViewTool {
        XcafViewTool { inner: ffi::xcaf_view_tool(&self.inner) }
    }
}

impl Default for XcafDocument {
    fn default() -> Self {
        Self::new()
    }
}

/// Provides shape-tree traversal and geometry access within an [`XcafDocument`].
pub struct XcafShapeTool {
    inner: UniquePtr<ffi::HandleXCAFDoc_ShapeTool>,
}

impl XcafShapeTool {
    /// Iterate over the top-level ("free") shapes in the document.
    pub fn free_shapes(&self) -> XcafLabelIter {
        let seq = ffi::xcaf_free_shapes(&self.inner);
        let len = ffi::xcaf_seq_len(&seq);
        XcafLabelIter { seq, len, next: 1 }
    }

    /// Iterate over the direct components of an assembly label.
    pub fn components(&self, label: &XcafLabel) -> XcafLabelIter {
        let seq = ffi::xcaf_label_components(&self.inner, &label.inner);
        let len = ffi::xcaf_seq_len(&seq);
        XcafLabelIter { seq, len, next: 1 }
    }

    pub fn is_assembly(&self, label: &XcafLabel) -> bool {
        ffi::xcaf_label_is_assembly(&self.inner, &label.inner)
    }

    pub fn is_reference(&self, label: &XcafLabel) -> bool {
        ffi::xcaf_label_is_reference(&self.inner, &label.inner)
    }

    /// Get the `TopoDS_Shape` associated with a label.
    pub fn shape(&self, label: &XcafLabel) -> Shape {
        Shape { inner: ffi::xcaf_label_shape(&self.inner, &label.inner) }
    }

    /// Extract the placement of a label as a row-major 4×4 matrix.
    ///
    /// Row/column are 0-indexed. The 4th row is always `[0, 0, 0, 1]`.
    /// Returns the identity matrix when the label has no location set.
    pub fn location_matrix(&self, label: &XcafLabel) -> [[f64; 4]; 4] {
        let loc = ffi::xcaf_label_location(&self.inner, &label.inner);
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
}

/// Provides per-label and per-shape color queries within an [`XcafDocument`].
pub struct XcafColorTool {
    inner: UniquePtr<ffi::HandleXCAFDoc_ColorTool>,
}

impl XcafColorTool {
    /// Look up the color assigned to a label. Returns `(r, g, b)` in linear [0, 1].
    /// Tries surface color, then generic color, then curve color.
    pub fn color_of_label(&self, label: &XcafLabel) -> Option<(f32, f32, f32)> {
        let (mut r, mut g, mut b) = (0.0f64, 0.0f64, 0.0f64);
        if ffi::xcaf_color_of_label(&self.inner, &label.inner, &mut r, &mut g, &mut b) {
            Some((r as f32, g as f32, b as f32))
        } else {
            None
        }
    }

    /// Look up the color assigned to a shape. Returns `(r, g, b)` in linear [0, 1].
    pub fn color_of_shape(&self, shape: &Shape) -> Option<(f32, f32, f32)> {
        let (mut r, mut g, mut b) = (0.0f64, 0.0f64, 0.0f64);
        if ffi::xcaf_color_of_shape(&self.inner, &shape.inner, &mut r, &mut g, &mut b) {
            Some((r as f32, g as f32, b as f32))
        } else {
            None
        }
    }
}

/// Provides access to GD&T/PMI annotations (dimensions, geometric tolerances, datums)
/// stored in an [`XcafDocument`].
pub struct XcafDimTolTool {
    inner: UniquePtr<ffi::HandleXCAFDoc_DimTolTool>,
}

impl XcafDimTolTool {
    /// Iterate over all dimension annotation labels.
    pub fn dimension_labels(&self) -> XcafLabelIter {
        let seq = ffi::xcaf_dimtol_dimension_labels(&self.inner);
        let len = ffi::xcaf_seq_len(&seq);
        XcafLabelIter { seq, len, next: 1 }
    }

    /// Iterate over all geometric tolerance annotation labels.
    pub fn geom_tolerance_labels(&self) -> XcafLabelIter {
        let seq = ffi::xcaf_dimtol_geomtol_labels(&self.inner);
        let len = ffi::xcaf_seq_len(&seq);
        XcafLabelIter { seq, len, next: 1 }
    }

    /// Iterate over all datum annotation labels.
    pub fn datum_labels(&self) -> XcafLabelIter {
        let seq = ffi::xcaf_dimtol_datum_labels(&self.inner);
        let len = ffi::xcaf_seq_len(&seq);
        XcafLabelIter { seq, len, next: 1 }
    }
}

/// Provides access to clipping planes stored in an [`XcafDocument`].
pub struct XcafClippingPlaneTool {
    inner: UniquePtr<ffi::HandleXCAFDoc_ClippingPlaneTool>,
}

impl XcafClippingPlaneTool {
    /// Iterate over all clipping plane labels in the document.
    pub fn clipping_plane_labels(&self) -> XcafLabelIter {
        let seq = ffi::xcaf_clipping_plane_labels(&self.inner);
        let len = ffi::xcaf_seq_len(&seq);
        XcafLabelIter { seq, len, next: 1 }
    }

    /// Returns `true` if `label` is a clipping plane definition.
    pub fn is_clipping_plane(&self, label: &XcafLabel) -> bool {
        ffi::xcaf_is_clipping_plane(&self.inner, &label.inner)
    }

    /// Returns the data for a clipping plane label, or `None` if the label is invalid.
    pub fn clipping_plane_data(&self, label: &XcafLabel) -> Option<ClippingPlaneData> {
        let pln = ffi::xcaf_clipping_plane_pln(&self.inner, &label.inner);
        if pln.is_null() {
            return None;
        }
        let loc = pln.Location();
        let norm = pln.Axis().Direction();
        let name = {
            let s = ffi::xcaf_clipping_plane_name(&self.inner, &label.inner);
            if s.is_empty() { None } else { Some(s) }
        };
        let capping = ffi::xcaf_clipping_plane_capping(&self.inner, &label.inner);
        Some(ClippingPlaneData {
            origin: [loc.X(), loc.Y(), loc.Z()],
            normal: [norm.X(), norm.Y(), norm.Z()],
            name,
            capping,
        })
    }
}

/// Data for a clipping plane stored in an [`XcafDocument`].
#[derive(Debug)]
pub struct ClippingPlaneData {
    /// Plane origin in model space.
    pub origin: [f64; 3],
    /// Plane normal direction (unit vector).
    pub normal: [f64; 3],
    /// Display name, if any.
    pub name: Option<String>,
    /// Whether the cutting section is capped (filled).
    pub capping: bool,
}

/// Provides access to views stored in an [`XcafDocument`].
pub struct XcafViewTool {
    inner: UniquePtr<ffi::HandleXCAFDoc_ViewTool>,
}

impl XcafViewTool {
    /// Iterate over all view labels in the document.
    pub fn view_labels(&self) -> XcafLabelIter {
        let seq = ffi::xcaf_view_labels(&self.inner);
        let len = ffi::xcaf_seq_len(&seq);
        XcafLabelIter { seq, len, next: 1 }
    }

    /// Returns `true` if `label` is a view definition.
    pub fn is_view(&self, label: &XcafLabel) -> bool {
        ffi::xcaf_is_view(&self.inner, &label.inner)
    }

    /// Iterate over the shape labels referenced by a view.
    pub fn ref_shapes(&self, label: &XcafLabel) -> XcafLabelIter {
        let seq = ffi::xcaf_view_ref_shapes(&self.inner, &label.inner);
        let len = ffi::xcaf_seq_len(&seq);
        XcafLabelIter { seq, len, next: 1 }
    }

    /// Iterate over the GDT annotation labels referenced by a view.
    pub fn ref_gdts(&self, label: &XcafLabel) -> XcafLabelIter {
        let seq = ffi::xcaf_view_ref_gdts(&self.inner, &label.inner);
        let len = ffi::xcaf_seq_len(&seq);
        XcafLabelIter { seq, len, next: 1 }
    }

    /// Iterate over the clipping plane labels referenced by a view.
    pub fn ref_clipping_planes(&self, label: &XcafLabel) -> XcafLabelIter {
        let seq = ffi::xcaf_view_ref_clipping_planes(&self.inner, &label.inner);
        let len = ffi::xcaf_seq_len(&seq);
        XcafLabelIter { seq, len, next: 1 }
    }

    /// Returns the camera and window data for a view label, or `None` if the label is invalid.
    pub fn view_data(&self, label: &XcafLabel) -> Option<ViewData> {
        let mut obj = ffi::xcaf_view_object(&label.inner);
        if obj.is_null() {
            return None;
        }
        let mut obj = obj.pin_mut();
        let name = {
            let s = ffi::xcaf_view_name(obj.as_mut());
            if s.is_empty() { None } else { Some(s) }
        };
        let pt = ffi::xcaf_view_projection_point(obj.as_mut());
        let dir = ffi::xcaf_view_direction(obj.as_mut());
        let up = ffi::xcaf_view_up_direction(obj.as_mut());
        let projection_type = obj.as_mut().Type();
        let zoom_factor = obj.as_mut().ZoomFactor();
        let window_horizontal_size = obj.as_mut().WindowHorizontalSize();
        let window_vertical_size = obj.as_mut().WindowVerticalSize();
        let has_front = obj.as_mut().HasFrontPlaneClipping();
        let front_plane_distance =
            if has_front { Some(obj.as_mut().FrontPlaneDistance()) } else { None };
        let has_back = obj.as_mut().HasBackPlaneClipping();
        let back_plane_distance =
            if has_back { Some(obj.as_mut().BackPlaneDistance()) } else { None };
        let has_view_volume_sides_clipping = obj.as_mut().HasViewVolumeSidesClipping();
        Some(ViewData {
            name,
            projection_type,
            projection_point: [pt.X(), pt.Y(), pt.Z()],
            view_direction: [dir.X(), dir.Y(), dir.Z()],
            up_direction: [up.X(), up.Y(), up.Z()],
            zoom_factor,
            window_horizontal_size,
            window_vertical_size,
            front_plane_distance,
            back_plane_distance,
            has_view_volume_sides_clipping,
        })
    }
}

/// Camera and window data for a view stored in an [`XcafDocument`].
#[derive(Debug)]
pub struct ViewData {
    /// Display name, if any.
    pub name: Option<String>,
    /// Projection type (parallel or central/perspective).
    pub projection_type: ffi::XCAFView_ProjectionType,
    /// Camera/eye position in model space.
    pub projection_point: [f64; 3],
    /// View direction vector (camera look direction).
    pub view_direction: [f64; 3],
    /// Up direction vector.
    pub up_direction: [f64; 3],
    /// Zoom factor.
    pub zoom_factor: f64,
    /// Horizontal size of the view window.
    pub window_horizontal_size: f64,
    /// Vertical size of the view window.
    pub window_vertical_size: f64,
    /// Front clipping plane distance, if set.
    pub front_plane_distance: Option<f64>,
    /// Back clipping plane distance, if set.
    pub back_plane_distance: Option<f64>,
    /// Whether view-volume sides clipping is active.
    pub has_view_volume_sides_clipping: bool,
}

/// Semantic data for a dimension annotation.
#[derive(Debug)]
pub struct DimensionData {
    pub type_: ffi::XCAFDimTolObjects_DimensionType,
    pub value: f64,
    pub upper_tolerance: f64,
    pub lower_tolerance: f64,
    pub semantic_name: Option<String>,
}

/// Semantic data for a geometric tolerance annotation.
#[derive(Debug)]
pub struct GeomToleranceData {
    pub type_: ffi::XCAFDimTolObjects_GeomToleranceType,
    pub value: f64,
    pub semantic_name: Option<String>,
}

/// Semantic data for a datum annotation.
#[derive(Debug)]
pub struct DatumData {
    /// Datum identifier (e.g. "A", "B", "C").
    pub name: Option<String>,
    pub semantic_name: Option<String>,
}

/// A node in the XCAF label tree. Holds a name, location, and either sub-components
/// (assembly) or geometry (leaf shape).
pub struct XcafLabel {
    pub(crate) inner: UniquePtr<ffi::TDF_Label>,
}

impl XcafLabel {
    /// The part name stored in the `TDataStd_Name` attribute, if any.
    pub fn name(&self) -> Option<String> {
        let s = ffi::xcaf_label_name(&self.inner);
        if s.is_empty() { None } else { Some(s) }
    }

    // --- PMI graphical presentation ---

    /// Returns the graphical presentation shape if this label is a dimension, or `None`.
    pub fn dimension_presentation(&self) -> Option<Shape> {
        let shape = Shape { inner: ffi::xcaf_dimension_presentation(&self.inner) };
        if shape.is_null() { None } else { Some(shape) }
    }

    /// Returns the graphical presentation shape if this label is a geometric tolerance, or `None`.
    pub fn geom_tolerance_presentation(&self) -> Option<Shape> {
        let shape = Shape { inner: ffi::xcaf_geomtol_presentation(&self.inner) };
        if shape.is_null() { None } else { Some(shape) }
    }

    /// Returns the graphical presentation shape if this label is a datum, or `None`.
    pub fn datum_presentation(&self) -> Option<Shape> {
        let shape = Shape { inner: ffi::xcaf_datum_presentation(&self.inner) };
        if shape.is_null() { None } else { Some(shape) }
    }

    // --- PMI attribute presence checks ---

    pub fn is_dimension(&self) -> bool {
        ffi::xcaf_is_dimension(&self.inner)
    }

    pub fn is_geom_tolerance(&self) -> bool {
        ffi::xcaf_is_geomtol(&self.inner)
    }

    pub fn is_datum(&self) -> bool {
        ffi::xcaf_is_datum(&self.inner)
    }

    // --- PMI semantic data ---

    /// Returns the semantic data for a dimension annotation, or `None` if this label is not a dimension.
    pub fn dimension_data(&self) -> Option<DimensionData> {
        if !ffi::xcaf_is_dimension(&self.inner) {
            return None;
        }
        let semantic_name = {
            let s = ffi::xcaf_dimension_semantic_name(&self.inner);
            if s.is_empty() { None } else { Some(s) }
        };
        Some(DimensionData {
            type_: ffi::xcaf_dimension_type(&self.inner),
            value: ffi::xcaf_dimension_value(&self.inner),
            upper_tolerance: ffi::xcaf_dimension_upper_tol(&self.inner),
            lower_tolerance: ffi::xcaf_dimension_lower_tol(&self.inner),
            semantic_name,
        })
    }

    /// Returns the semantic data for a geometric tolerance annotation, or `None`.
    pub fn geom_tolerance_data(&self) -> Option<GeomToleranceData> {
        if !ffi::xcaf_is_geomtol(&self.inner) {
            return None;
        }
        let semantic_name = {
            let s = ffi::xcaf_geomtol_semantic_name(&self.inner);
            if s.is_empty() { None } else { Some(s) }
        };
        Some(GeomToleranceData {
            type_: ffi::xcaf_geomtol_type(&self.inner),
            value: ffi::xcaf_geomtol_value(&self.inner),
            semantic_name,
        })
    }

    /// Returns the semantic data for a datum annotation, or `None` if this label is not a datum.
    pub fn datum_data(&self) -> Option<DatumData> {
        if !ffi::xcaf_is_datum(&self.inner) {
            return None;
        }
        let name = {
            let s = ffi::xcaf_datum_name(&self.inner);
            if s.is_empty() { None } else { Some(s) }
        };
        let semantic_name = {
            let s = ffi::xcaf_datum_semantic_name(&self.inner);
            if s.is_empty() { None } else { Some(s) }
        };
        Some(DatumData { name, semantic_name })
    }
}

/// Iterator over a `TDF_LabelSequence`. Uses 1-based OCCT indexing internally.
pub struct XcafLabelIter {
    seq: UniquePtr<ffi::TDF_LabelSequence>,
    len: i32,
    next: i32,
}

impl Iterator for XcafLabelIter {
    type Item = XcafLabel;

    fn next(&mut self) -> Option<XcafLabel> {
        if self.next > self.len {
            return None;
        }
        let label = ffi::xcaf_seq_get(&self.seq, self.next);
        self.next += 1;
        Some(XcafLabel { inner: label })
    }
}
