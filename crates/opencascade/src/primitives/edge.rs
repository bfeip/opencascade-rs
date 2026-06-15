use crate::primitives::{make_axis_2, make_point, Face};
use crate::Error;
use cxx::UniquePtr;
use glam::{dvec3, DVec3};
use opencascade_sys::ffi;

use super::make_vec;

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum EdgeType {
    Line,
    Circle,
    Ellipse,
    Hyperbola,
    Parabola,
    BezierCurve,
    BSplineCurve,
    OffsetCurve,
    OtherCurve,
}

impl From<ffi::GeomAbs_CurveType> for EdgeType {
    fn from(curve_type: ffi::GeomAbs_CurveType) -> Self {
        match curve_type {
            ffi::GeomAbs_CurveType::GeomAbs_Line => Self::Line,
            ffi::GeomAbs_CurveType::GeomAbs_Circle => Self::Circle,
            ffi::GeomAbs_CurveType::GeomAbs_Ellipse => Self::Ellipse,
            ffi::GeomAbs_CurveType::GeomAbs_Hyperbola => Self::Hyperbola,
            ffi::GeomAbs_CurveType::GeomAbs_Parabola => Self::Parabola,
            ffi::GeomAbs_CurveType::GeomAbs_BezierCurve => Self::BezierCurve,
            ffi::GeomAbs_CurveType::GeomAbs_BSplineCurve => Self::BSplineCurve,
            ffi::GeomAbs_CurveType::GeomAbs_OffsetCurve => Self::OffsetCurve,
            ffi::GeomAbs_CurveType::GeomAbs_OtherCurve => Self::OtherCurve,
            ffi::GeomAbs_CurveType { repr } => panic!("Unexpected curve type: {repr}"),
        }
    }
}

pub struct Edge {
    pub(crate) inner: UniquePtr<ffi::TopoDS_Edge>,
}

impl AsRef<Edge> for Edge {
    fn as_ref(&self) -> &Edge {
        self
    }
}

impl Edge {
    pub(crate) fn from_edge(edge: &ffi::TopoDS_Edge) -> Self {
        let inner = ffi::TopoDS_Edge_to_owned(edge);

        Self { inner }
    }

    fn from_make_edge(
        mut make_edge: UniquePtr<ffi::BRepBuilderAPI_MakeEdge>,
    ) -> Result<Self, Error> {
        if !make_edge.IsDone() {
            return Err(Error::EdgeFailed(make_edge.Error().into()));
        }

        let make_edge_pin = make_edge.pin_mut();
        Ok(Self::from_edge(make_edge_pin.Edge()))
    }

    pub fn segment(p1: DVec3, p2: DVec3) -> Result<Self, Error> {
        let make_edge =
            ffi::BRepBuilderAPI_MakeEdge_gp_Pnt_gp_Pnt(&make_point(p1), &make_point(p2));

        Self::from_make_edge(make_edge)
    }

    pub fn bezier(points: impl IntoIterator<Item = DVec3>) -> Result<Self, Error> {
        let points: Vec<_> = points.into_iter().collect();
        let mut array = ffi::TColgp_HArray1OfPnt_ctor(1, points.len() as i32);
        for (index, point) in points.into_iter().enumerate() {
            array.pin_mut().SetValue(index as i32 + 1, &make_point(point));
        }

        let bezier = ffi::Geom_BezierCurve_ctor_points(&array);
        let bezier_handle = ffi::Geom_BezierCurve_to_handle(bezier);
        let curve_handle = ffi::new_HandleGeomCurve_from_HandleGeom_BezierCurve(&bezier_handle);

        let make_edge = ffi::BRepBuilderAPI_MakeEdge_HandleGeomCurve(&curve_handle);
        Self::from_make_edge(make_edge)
    }

    pub fn circle(center: DVec3, normal: DVec3, radius: f64) -> Result<Self, Error> {
        let axis = make_axis_2(center, normal);

        let make_circle = ffi::gp_Circ_ctor(&axis, radius);
        let make_edge = ffi::BRepBuilderAPI_MakeEdge_circle(&make_circle);

        Self::from_make_edge(make_edge)
    }

    pub fn ellipse() {}

    /// Interpolates a B-spline curve through `points`. When `periodic` is true the
    /// curve is closed smoothly back to the first point; the closing point must NOT
    /// be repeated in `points`. `tangents` constrains the start/end tangents and
    /// only applies to open (non-periodic) curves.
    pub fn spline_from_points(
        points: impl IntoIterator<Item = DVec3>,
        tangents: Option<(DVec3, DVec3)>,
        periodic: bool,
    ) -> Result<Self, Error> {
        let points: Vec<_> = points.into_iter().collect();
        let tolerance = 1.0e-7;

        if points.len() < 2 {
            return Err(Error::NotEnoughPoints);
        }
        for (i, pair) in points.windows(2).enumerate() {
            if pair[0].distance(pair[1]) <= tolerance {
                return Err(Error::IdenticalSplinePoints(i, i + 1));
            }
        }
        if periodic {
            let last = points.len() - 1;
            if points[last].distance(points[0]) <= tolerance {
                return Err(Error::IdenticalSplinePoints(last, 0));
            }
        }

        let mut array = ffi::TColgp_HArray1OfPnt_ctor(1, points.len() as i32);
        for (index, point) in points.iter().enumerate() {
            array.pin_mut().SetValue(index as i32 + 1, &make_point(*point));
        }
        let array_handle = ffi::new_HandleTColgpHArray1OfPnt_from_TColgpHArray1OfPnt(array);

        let mut interpolate = ffi::GeomAPI_Interpolate_ctor(&array_handle, periodic, tolerance);
        if let Some((t_start, t_end)) = tangents {
            interpolate.pin_mut().Load(&make_vec(t_start), &make_vec(t_end), true);
        }

        interpolate.pin_mut().Perform();
        let bspline_handle = ffi::GeomAPI_Interpolate_Curve(&interpolate);
        let curve_handle = ffi::new_HandleGeomCurve_from_HandleGeom_BSplineCurve(&bspline_handle);

        let make_edge = ffi::BRepBuilderAPI_MakeEdge_HandleGeomCurve(&curve_handle);
        Self::from_make_edge(make_edge)
    }

    pub fn arc(p1: DVec3, p2: DVec3, p3: DVec3) -> Result<Self, Error> {
        let make_arc = ffi::GC_MakeArcOfCircle_point_point_point(
            &make_point(p1),
            &make_point(p2),
            &make_point(p3),
        );

        let make_edge = ffi::BRepBuilderAPI_MakeEdge_HandleGeomCurve(
            &ffi::new_HandleGeomCurve_from_HandleGeom_TrimmedCurve(&ffi::GC_MakeArcOfCircle_Value(
                &make_arc,
            )),
        );

        Self::from_make_edge(make_edge)
    }

    pub fn start_point(&self) -> DVec3 {
        let curve = ffi::BRepAdaptor_Curve_ctor(&self.inner);
        let start_param = curve.FirstParameter();
        let point = ffi::BRepAdaptor_Curve_value(&curve, start_param);

        dvec3(point.X(), point.Y(), point.Z())
    }

    pub fn end_point(&self) -> DVec3 {
        let curve = ffi::BRepAdaptor_Curve_ctor(&self.inner);
        let last_param = curve.LastParameter();
        let point = ffi::BRepAdaptor_Curve_value(&curve, last_param);

        dvec3(point.X(), point.Y(), point.Z())
    }

    pub fn approximation_segments(&self) -> ApproximationSegmentIterator {
        let adaptor_curve = ffi::BRepAdaptor_Curve_ctor(&self.inner);
        let approximator = ffi::GCPnts_TangentialDeflection_ctor(&adaptor_curve, 0.1, 0.1);

        ApproximationSegmentIterator { count: 1, approximator }
    }

    pub fn tangent_arc(_p1: DVec3, _tangent: DVec3, _p3: DVec3) {}

    pub fn edge_type(&self) -> EdgeType {
        let curve = ffi::BRepAdaptor_Curve_ctor(&self.inner);

        EdgeType::from(curve.GetType())
    }

    /// Linearly extrudes this edge along `dir`, producing the swept [`Face`].
    ///
    /// This is the 1D→2D analogue of [`Face::extrude`](crate::primitives::Face::extrude):
    /// sweeping an edge along a vector yields a single surface.
    #[must_use]
    pub fn extrude(&self, dir: DVec3) -> Face {
        let prism_vec = make_vec(dir);

        let copy = false;
        let canonize = true;

        let inner_shape = ffi::cast_edge_to_shape(&self.inner);
        let mut make_prism =
            ffi::BRepPrimAPI_MakePrism_ctor(inner_shape, &prism_vec, copy, canonize);
        let face = ffi::TopoDS_cast_to_face(make_prism.pin_mut().Shape());

        Face::from_face(face)
    }
}

pub struct ApproximationSegmentIterator {
    count: usize,
    approximator: UniquePtr<ffi::GCPnts_TangentialDeflection>,
}

impl Iterator for ApproximationSegmentIterator {
    type Item = DVec3;

    fn next(&mut self) -> Option<Self::Item> {
        if self.count <= self.approximator.NbPoints() as usize {
            let point =
                ffi::GCPnts_TangentialDeflection_Value(&self.approximator, self.count as i32);

            self.count += 1;
            Some(dvec3(point.X(), point.Y(), point.Z()))
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::EdgeError;

    #[test]
    fn segment_with_identical_points_reports_error() {
        // A zero-length segment cannot form a valid edge; the builder should
        // surface the specific OCCT error rather than yielding a broken edge.
        let point = dvec3(1.0, 2.0, 3.0);
        let result = Edge::segment(point, point);

        assert!(matches!(
            result,
            Err(Error::EdgeFailed(EdgeError::LineThroughIdenticalPoints))
        ));
    }

    #[test]
    fn spline_through_consecutive_identical_points_reports_error() {
        // Two consecutive identical points would make OCCT's interpolation
        // constructor raise a Standard_ConstructionError and abort the process;
        // the guard must surface this as a recoverable error instead.
        let p = dvec3(1.0, 2.0, 3.0);
        let result = Edge::spline_from_points([p, p, dvec3(4.0, 5.0, 6.0)], None, false);

        assert!(matches!(result, Err(Error::IdenticalSplinePoints(0, 1))));
    }

    #[test]
    fn spline_through_nonconsecutive_duplicate_points_is_accepted() {
        // A spline may legitimately revisit a location, so only *adjacent*
        // duplicates are rejected.
        let p = dvec3(0.0, 0.0, 0.0);
        let result = Edge::spline_from_points(
            [p, dvec3(1.0, 0.0, 0.0), dvec3(1.0, 1.0, 0.0), p],
            None,
            false,
        );

        assert!(result.is_ok());
    }
}
