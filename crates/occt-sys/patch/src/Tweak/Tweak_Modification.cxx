// Derived from Draft_Modification (Jacques GOUSSARD, 1994).
// Copyright (c) 1994-1999 Matra Datavision
// Copyright (c) 1999-2014 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.
//
// The re-solve, from Propagate on, is Draft_Modification's, restructured: the
// draft-only paths (postponed faces, fixed tangent points, invented faces on
// free borders) are gone, rigid moves are taken exactly, and an edge's sense
// is told from the normals of its faces rather than from the old edge.

// Quoted: OCCT's build drops the headers of packages that only exist in the
// patch from its inc folder; the install still ships them.
#include "Tweak_Modification.hxx"

#include <Adaptor3d_CurveOnSurface.hxx>
#include <Approx_CurveOnSurface.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <Draft_EdgeInfo.hxx>
#include <Draft_FaceInfo.hxx>
#include <Draft_VertexInfo.hxx>
#include <ElCLib.hxx>
#include <ElSLib.hxx>
#include <Extrema_ExtCS.hxx>
#include <Extrema_ExtPC.hxx>
#include <Extrema_POnCurv.hxx>
#include <Extrema_POnSurf.hxx>
#include <Geom2d_BezierCurve.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <Geom2dConvert.hxx>
#include <Geom2dConvert_CompCurveToBSplineCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Hyperbola.hxx>
#include <Geom_Line.hxx>
#include <Geom_Parabola.hxx>
#include <Geom_Plane.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_SurfaceOfLinearExtrusion.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomConvert_CompCurveToBSplineCurve.hxx>
#include <GeomInt_IntSS.hxx>
#include <GeomProjLib.hxx>
#include <gp.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec2d.hxx>
#include <IntCurveSurface_HInter.hxx>
#include <IntCurveSurface_IntersectionPoint.hxx>
#include <Precision.hxx>
#include <ProjLib_HCompProjectedCurve.hxx>
#include <Standard_ConstructionError.hxx>
#include <Standard_DomainError.hxx>
#include <Standard_Failure.hxx>
#include <StdFail_NotDone.hxx>
#include <TColGeom_SequenceOfCurve.hxx>
#include <TColgp_Array1OfPnt2d.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>

IMPLEMENT_STANDARD_RTTIEXT(Tweak_Modification, BRepTools_Modification)

static gp_Vec SurfaceNormal(const Handle(Geom_Surface)& S, const gp_Pnt& P);

static Standard_Boolean IsReversed(const Handle(Geom_Curve)& NewC,
                                   const Standard_Real NewPar,
                                   const gp_Pnt& OldP,
                                   const gp_Vec& OldT,
                                   const TopoDS_Face& F1,
                                   const TopoDS_Face& F2,
                                   const Handle(Geom_Surface)& NewS1,
                                   const Handle(Geom_Surface)& NewS2);

static Standard_Boolean SameMove(const gp_Trsf& T1, const gp_Trsf& T2);

static Handle(Geom_Surface) Basis(const Handle(Geom_Surface)& S);

static gp_Vec Normalized(const gp_Vec& V);

static Standard_Boolean OnSurface(const gp_Pnt& P1,
                                  const gp_Pnt& P2,
                                  const Handle(Geom_Surface)& S,
                                  const Standard_Real Tol);

static Standard_Boolean SurfaceParameters(const Handle(Geom_Surface)& S,
                                          const gp_Pnt& P,
                                          Standard_Real& U,
                                          Standard_Real& V);

static Standard_Integer NearestLine(const GeomInt_IntSS& Inter,
                                    const gp_Pnt& First,
                                    const gp_Pnt& Last,
                                    const Handle(Geom_Surface)& Still,
                                    Standard_Real& Param);

static Handle(Geom_Curve) GluedLines(const GeomInt_IntSS& Inter,
                                     const Handle(Geom_Surface)& S1,
                                     const gp_Pnt& First);

static Standard_Boolean NearestMeeting(const Handle(GeomAdaptor_Curve)& C,
                                       const Handle(GeomAdaptor_Surface)& S,
                                       const gp_Pnt& Near,
                                       gp_Pnt& Result);

static Standard_Real ParameterOn(const Handle(Geom_Curve)& C,
                                 const Draft_IndexedDataMapOfFaceFaceInfo& FMap,
                                 Draft_EdgeInfo& EI,
                                 const TopoDS_Edge& E,
                                 const gp_Pnt& P,
                                 Standard_Integer& done);

static Standard_Boolean Choose(const Draft_IndexedDataMapOfFaceFaceInfo& FMap,
                               Draft_IndexedDataMapOfEdgeEdgeInfo& EMap,
                               const TopoDS_Vertex& V,
                               Draft_VertexInfo& VI,
                               GeomAdaptor_Curve& C,
                               GeomAdaptor_Surface& S);

static Standard_Real Parameter(const Handle(Geom_Curve)& C, const gp_Pnt& P, Standard_Integer& done);

static Standard_Real SmartParameter(Draft_EdgeInfo& EI,
                                    const Standard_Real EdgeTol,
                                    const gp_Pnt& Pnt,
                                    const Standard_Integer Sign,
                                    const Handle(Geom_Surface)& S1,
                                    const Handle(Geom_Surface)& S2);

//=======================================================================
//function : Tweak_Modification
//purpose  :
//=======================================================================

Tweak_Modification::Tweak_Modification(const TopoDS_Shape& S)
    : myComp(Standard_False),
      myShape(S),
      errStat(Draft_NoError)
{
  TopExp::MapShapesAndAncestors(myShape, TopAbs_EDGE, TopAbs_FACE, myEFMap);
}

//=======================================================================
//function : Clear
//purpose  :
//=======================================================================

void Tweak_Modification::Clear()
{
  myComp = Standard_False;
  myFMap.Clear();
  myEMap.Clear();
  myVMap.Clear();
  myMoves.Clear();
  badShape.Nullify();
  errStat = Draft_NoError;
}

//=======================================================================
//function : Init
//purpose  :
//=======================================================================

void Tweak_Modification::Init(const TopoDS_Shape& S)
{
  myShape = S;
  Clear();
  myEFMap.Clear();
  TopExp::MapShapesAndAncestors(myShape, TopAbs_EDGE, TopAbs_FACE, myEFMap);
}

//=======================================================================
//function : Add
//purpose  :
//=======================================================================

Standard_Boolean Tweak_Modification::Add(const TopoDS_Face& F, const gp_Trsf& T)
{
  if (!badShape.IsNull()) {
    throw Standard_ConstructionError();
  }
  if (myComp) {
    Clear();
  }
  if (T.IsNegative()) {
    badShape = F;
    errStat = Draft_FaceRecomputation;
    return Standard_False;
  }

  // Faces added with the same move are one group: the edges and vertices
  // between them move rigidly.
  curFace = F;
  for (NCollection_DataMap<TopoDS_Shape, gp_Trsf, TopTools_ShapeMapHasher>::Iterator it(myMoves);
       it.More(); it.Next()) {
    if (SameMove(it.Value(), T)) {
      curFace = TopoDS::Face(it.Key());
      break;
    }
  }
  if (curFace.IsSame(F) && !myMoves.IsBound(F)) {
    myMoves.Bind(F, T);
  }
  return InternalAdd(F, T);
}

//=======================================================================
//function : InternalAdd
//purpose  :
//=======================================================================

Standard_Boolean Tweak_Modification::InternalAdd(const TopoDS_Face& F, const gp_Trsf& T)
{
  if (myFMap.Contains(F)) {
    // A face reached from two moves cannot follow both.
    if (!myFMap.FindFromKey(F).RootFace().IsSame(curFace)) {
      badShape = F;
      errStat = Draft_FaceRecomputation;
    }
    return badShape.IsNull();
  }

  TopLoc_Location Lo;
  Handle(Geom_Surface) S = BRep_Tool::Surface(F, Lo);
  S = Handle(Geom_Surface)::DownCast(S->Transformed(Lo.Transformation()));
  if (S->DynamicType() == STANDARD_TYPE(Geom_RectangularTrimmedSurface)) {
    S = Handle(Geom_RectangularTrimmedSurface)::DownCast(S)->BasisSurface();
  }
  Handle(Geom_Surface) NewS = Handle(Geom_Surface)::DownCast(S->Transformed(T));

  // A cylinder or cone is bounded well past the face, as Draft does to keep
  // clear of trouble with unbounded surfaces, and a cone short of its apex.
  Handle(Geom_ConicalSurface) aCone = Handle(Geom_ConicalSurface)::DownCast(NewS);
  if (!aCone.IsNull() || NewS->DynamicType() == STANDARD_TYPE(Geom_CylindricalSurface)) {
    Standard_Real aUMin, aUMax, aVMin, aVMax;
    BRepTools::UVBounds(F, aUMin, aUMax, aVMin, aVMax);
    if (!Precision::IsNegativeInfinite(aVMin) && !Precision::IsPositiveInfinite(aVMax)) {
      const Standard_Real aMargin = 10. * (aVMax - aVMin);
      const Standard_Real anApex =
        aCone.IsNull() ? 0. : -aCone->Cone().RefRadius() / Sin(aCone->Cone().SemiAngle());
      if (!aCone.IsNull() && aVMin < anApex && aVMax + aMargin > anApex) {
        aVMax = anApex;
        aVMin -= 10. * (aVMax - aVMin);
        aVMax -= Precision::Confusion();
      }
      else if (!aCone.IsNull() && aVMin >= anApex && aVMin - aMargin < anApex) {
        aVMin = anApex;
        aVMax += 10. * (aVMax - aVMin);
        aVMin += Precision::Confusion();
      }
      else {
        aVMin -= aMargin;
        aVMax += aMargin;
      }
      NewS = new Geom_RectangularTrimmedSurface(NewS, 0., 2. * M_PI, aVMin, aVMax);
    }
  }

  Draft_FaceInfo FI(NewS, Standard_True);
  FI.RootFace(curFace);
  myFMap.Add(F, FI);

  // Seams and free edges move with the face; so does a tangent neighbour,
  // which then carries the move on.
  for (TopExp_Explorer aExp(F, TopAbs_EDGE); aExp.More() && badShape.IsNull(); aExp.Next()) {
    const TopoDS_Edge& edg = TopoDS::Edge(aExp.Current());
    if (myEMap.Contains(edg) || BRep_Tool::Degenerated(edg)) {
      continue;
    }
    Standard_Boolean addedg = Standard_False;
    TopoDS_Face OtherF;
    if (BRepTools::IsReallyClosed(edg, F)) {
      addedg = Standard_True;
    }
    else {
      Standard_Integer nbother = 0;
      for (TopTools_ListIteratorOfListOfShape it(myEFMap.FindFromKey(edg)); it.More(); it.Next()) {
        if (!it.Value().IsSame(F)) {
          if (OtherF.IsNull()) {
            OtherF = TopoDS::Face(it.Value());
          }
          nbother++;
        }
      }
      if (nbother >= 2) {
        badShape = edg;
        errStat = Draft_EdgeRecomputation;
      }
      else if (nbother == 0 || BRep_Tool::Continuity(edg, F, OtherF) >= GeomAbs_G1) {
        addedg = Standard_True;
      }
      else {
        OtherF.Nullify();
      }
    }
    if (!addedg) {
      continue;
    }

    Standard_Real f, l;
    TopLoc_Location L;
    Handle(Geom_Curve) C = BRep_Tool::Curve(edg, L, f, l);
    C = Handle(Geom_Curve)::DownCast(C->Transformed(L));
    if (C->DynamicType() == STANDARD_TYPE(Geom_TrimmedCurve)) {
      C = Handle(Geom_TrimmedCurve)::DownCast(C)->BasisCurve();
    }
    Draft_EdgeInfo EInf(Standard_True);
    EInf.ChangeGeometry() = Handle(Geom_Curve)::DownCast(C->Transformed(T));
    EInf.RootFace(curFace);
    myEMap.Add(edg, EInf);
    if (!OtherF.IsNull()) {
      InternalAdd(OtherF, T);
    }
  }
  return badShape.IsNull();
}

//=======================================================================
//function : IsDone
//purpose  :
//=======================================================================

Standard_Boolean Tweak_Modification::IsDone() const
{
  return myComp && badShape.IsNull();
}

//=======================================================================
//function : Error
//purpose  :
//=======================================================================

Draft_ErrorStatus Tweak_Modification::Error() const
{
  return errStat;
}

//=======================================================================
//function : ProblematicShape
//purpose  :
//=======================================================================

const TopoDS_Shape& Tweak_Modification::ProblematicShape() const
{
  return badShape;
}

//=======================================================================
//function : ModifiedFaces
//purpose  :
//=======================================================================

const TopTools_ListOfShape& Tweak_Modification::ModifiedFaces()
{
  if (!badShape.IsNull()) {
    throw StdFail_NotDone();
  }
  conneF.Clear();
  for (Standard_Integer i = 1; i <= myFMap.Extent(); i++) {
    if (myFMap(i).NewGeometry()) {
      conneF.Append(myFMap.FindKey(i));
    }
  }
  return conneF;
}

//=======================================================================
//function : Move
//purpose  :
//=======================================================================

const gp_Trsf* Tweak_Modification::Move(const TopoDS_Face& F) const
{
  if (F.IsNull() || !myFMap.Contains(F)) {
    return NULL;
  }
  const Draft_FaceInfo& FI = myFMap.FindFromKey(F);
  return FI.NewGeometry() ? myMoves.Seek(FI.RootFace()) : NULL;
}

//=======================================================================
//function : RigidMove
//purpose  :
//=======================================================================

const gp_Trsf* Tweak_Modification::RigidMove(const TopoDS_Edge& E) const
{
  if (!myEMap.Contains(E)) {
    return NULL;
  }
  const Draft_EdgeInfo& EI = myEMap.FindFromKey(E);
  const gp_Trsf* aMove = Move(EI.FirstFace());
  if (aMove == NULL || EI.SecondFace().IsNull()) {
    return aMove;
  }
  return Move(EI.SecondFace()) == aMove ? aMove : NULL;
}

//=======================================================================
//function : RigidMove
//purpose  :
//=======================================================================

const gp_Trsf* Tweak_Modification::RigidMove(const TopoDS_Vertex& V)
{
  Draft_VertexInfo& VI = myVMap.ChangeFromKey(V);
  const gp_Trsf* aMove = NULL;
  for (VI.InitEdgeIterator(); VI.MoreEdge(); VI.NextEdge()) {
    const gp_Trsf* anEdgeMove = RigidMove(VI.Edge());
    if (anEdgeMove == NULL || (aMove != NULL && anEdgeMove != aMove)) {
      return NULL;
    }
    aMove = anEdgeMove;
  }
  return aMove;
}

//=======================================================================
//function : NewSurface
//purpose  :
//=======================================================================

Standard_Boolean Tweak_Modification::NewSurface(const TopoDS_Face& F,
                                                Handle(Geom_Surface)& S,
                                                TopLoc_Location& L,
                                                Standard_Real& Tol,
                                                Standard_Boolean& RevWires,
                                                Standard_Boolean& RevFace)
{
  if (!IsDone()) {
    throw Standard_DomainError();
  }
  if (!myFMap.Contains(F) || !myFMap.FindFromKey(F).NewGeometry()) {
    return Standard_False;
  }
  RevWires = Standard_False;
  RevFace = Standard_False;
  Tol = BRep_Tool::Tolerance(F);
  L.Identity();
  S = myFMap.FindFromKey(F).Geometry();
  return Standard_True;
}

//=======================================================================
//function : NewCurve
//purpose  :
//=======================================================================

Standard_Boolean Tweak_Modification::NewCurve(const TopoDS_Edge& E,
                                              Handle(Geom_Curve)& C,
                                              TopLoc_Location& L,
                                              Standard_Real& Tol)
{
  if (!IsDone()) {
    throw Standard_DomainError();
  }
  if (!myEMap.Contains(E)) {
    return Standard_False;
  }
  const Draft_EdgeInfo& Einf = myEMap.FindFromKey(E);
  if (!Einf.NewGeometry() || Einf.Geometry().IsNull()) {
    return Standard_False;
  }
  Tol = Max(Einf.Tolerance(), BRep_Tool::Tolerance(E));
  L.Identity();
  C = Einf.Geometry();
  return Standard_True;
}

//=======================================================================
//function : NewPoint
//purpose  :
//=======================================================================

Standard_Boolean Tweak_Modification::NewPoint(const TopoDS_Vertex& V,
                                              gp_Pnt& P,
                                              Standard_Real& Tol)
{
  if (!IsDone()) {
    throw Standard_DomainError();
  }
  if (!myVMap.Contains(V)) {
    return Standard_False;
  }
  Tol = BRep_Tool::Tolerance(V);
  P = myVMap.FindFromKey(V).Geometry();
  return Standard_True;
}

//=======================================================================
//function : NewCurve2d
//purpose  :
//=======================================================================

Standard_Boolean Tweak_Modification::NewCurve2d(const TopoDS_Edge& E,
                                                const TopoDS_Face& F,
                                                const TopoDS_Edge& NewE,
                                                const TopoDS_Face&,
                                                Handle(Geom2d_Curve)& C,
                                                Standard_Real& Tol)
{
  if (!IsDone()) {
    throw Standard_DomainError();
  }
  if (!myEMap.Contains(E)) {
    return Standard_False;
  }

  // An isometry keeps the parametrisation of the surfaces and curves it
  // moves, so an edge moving with its faces keeps its pcurves.
  const gp_Trsf* aMove = RigidMove(E);
  if (aMove == NULL && BRep_Tool::Degenerated(E)) {
    aMove = Move(F);
  }
  if (aMove != NULL && Abs(aMove->ScaleFactor() - 1.) <= Epsilon(1.)) {
    Standard_Real aFirst, aLast;
    C = Handle(Geom2d_Curve)::DownCast(BRep_Tool::CurveOnSurface(E, F, aFirst, aLast)->Copy());
    Tol = BRep_Tool::Tolerance(E);
    return Standard_True;
  }
  if (BRep_Tool::Degenerated(E)) {
    return Standard_False;
  }

  const Draft_EdgeInfo& anInfo = myEMap.FindFromKey(E);
  Handle(Geom_Surface) aSurface = myFMap.FindFromKey(F).Geometry();
  Tol = BRep_Tool::Tolerance(E);
  Standard_Real aFirst, aLast;
  BRep_Tool::Range(NewE, aFirst, aLast);
  if (anInfo.FirstFace().IsSame(F) && !anInfo.FirstPC().IsNull()) {
    C = anInfo.FirstPC();
  }
  else if (anInfo.SecondFace().IsSame(F) && !anInfo.SecondPC().IsNull()) {
    C = anInfo.SecondPC();
  }
  else {
    // An edge whose curve stays keeps its pcurve while its ends stay within
    // it; otherwise the new curve is projected over the new edge's range.
    if (!anInfo.NewGeometry()) {
      Standard_Real anOldFirst, anOldLast;
      BRep_Tool::Range(E, anOldFirst, anOldLast);
      if (anOldFirst <= aFirst && aFirst <= anOldLast && anOldFirst <= aLast && aLast <= anOldLast) {
        return Standard_False;
      }
    }
    Handle(Geom_TrimmedCurve) aTrimmed = new Geom_TrimmedCurve(anInfo.Geometry(), aFirst, aLast);
    aFirst = aTrimmed->FirstParameter();
    aLast = aTrimmed->LastParameter();
    BRep_Builder().Range(NewE, aFirst, aLast);
    C = GeomProjLib::Curve2d(aTrimmed, aFirst, aLast, aSurface, Tol);
  }

  // On a surface periodic in U, a projected pcurve may land a period away
  // from the old one: shift it back.
  aSurface = Basis(aSurface);
  Handle(Geom_SurfaceOfLinearExtrusion) anExtrusion = Handle(Geom_SurfaceOfLinearExtrusion)::DownCast(aSurface);
  const Handle(Standard_Type)& aType = aSurface->DynamicType();
  if (aType == STANDARD_TYPE(Geom_CylindricalSurface) || aType == STANDARD_TYPE(Geom_SphericalSurface)
      || aType == STANDARD_TYPE(Geom_ConicalSurface)
      || (!anExtrusion.IsNull() && anExtrusion->BasisCurve()->DynamicType() == STANDARD_TYPE(Geom_Circle))) {
    Standard_Real anOldFirst, anOldLast;
    const Handle(Geom2d_Curve)& anOldPCurve = BRep_Tool::CurveOnSurface(E, F, anOldFirst, anOldLast);
    const gp_Pnt2d anOldMid = anOldPCurve->Value(0.5 * (anOldFirst + anOldLast));
    const gp_Pnt2d aNewMid = C->Value(0.5 * (aFirst + aLast));
    const gp_Vec2d aPeriod(2. * M_PI, 0.);
    const Standard_Real aDist2 = aNewMid.SquareDistance(anOldMid);
    if (aNewMid.Translated(aPeriod).SquareDistance(anOldMid) < aDist2) {
      C->Translate(aPeriod);
    }
    else if (aNewMid.Translated(-aPeriod).SquareDistance(anOldMid) < aDist2) {
      C->Translate(-aPeriod);
    }
  }

  Handle(Geom_Curve) aCurve = BRep_Tool::Curve(NewE, aFirst, aLast);
  Tol = BRepTools::EvalAndUpdateTol(NewE, aCurve, C, aSurface, aFirst, aLast);
  return Standard_True;
}

//=======================================================================
//function : NewParameter
//purpose  :
//=======================================================================

Standard_Boolean Tweak_Modification::NewParameter(const TopoDS_Vertex& V,
                                                  const TopoDS_Edge& E,
                                                  Standard_Real& P,
                                                  Standard_Real& Tol)
{
  if (!IsDone()) {
    throw Standard_DomainError();
  }
  if (!myVMap.Contains(V) || !myEMap.Contains(E)) {
    return Standard_False;
  }
  const Handle(Geom_Curve)& aCurve = myEMap.FindFromKey(E).Geometry();
  if (aCurve.IsNull()) {
    return Standard_False;
  }
  P = myVMap.ChangeFromKey(V).Parameter(E);

  // On a closed curve the last vertex comes after the first: a period on,
  // or at the curve's end.
  if (aCurve->IsClosed()) {
    TopoDS_Vertex aFirstVertex = TopExp::FirstVertex(E);
    Standard_Real aFirstParam = myVMap.Contains(aFirstVertex)
                                  ? myVMap.ChangeFromKey(aFirstVertex).Parameter(E)
                                  : BRep_Tool::Parameter(aFirstVertex, E);
    aFirstVertex.Orientation(E.Orientation());
    if (Abs(aFirstParam - aCurve->LastParameter()) <= Precision::PConfusion()) {
      aFirstParam = aCurve->FirstParameter();
      if (V.IsEqual(aFirstVertex)) {
        P = aFirstParam;
      }
    }
    if (!V.IsEqual(aFirstVertex) && P <= aFirstParam) {
      P = aCurve->IsPeriodic() ? P + aCurve->Period() : aCurve->LastParameter();
    }
  }
  Tol = Max(BRep_Tool::Tolerance(V), BRep_Tool::Tolerance(E));
  return Standard_True;
}

//=======================================================================
//function : Continuity
//purpose  :
//=======================================================================

GeomAbs_Shape Tweak_Modification::Continuity(const TopoDS_Edge& E,
                                             const TopoDS_Face& F1,
                                             const TopoDS_Face& F2,
                                             const TopoDS_Edge&,
                                             const TopoDS_Face&,
                                             const TopoDS_Face&)
{
  return BRep_Tool::Continuity(E, F1, F2);
}


//=======================================================================
//function : Propagate
//purpose  : Gathers what the moved faces touch: their edges and vertices,
//           every other edge at those vertices, and the faces along all of
//           those edges, which keep their surfaces.
//=======================================================================

Standard_Boolean Tweak_Modification::Propagate()
{
  if (!badShape.IsNull()) {
    return Standard_False;
  }

  for (Standard_Integer i = 1; i <= myFMap.Extent(); i++) {
    const TopoDS_Face& aFace = myFMap.FindKey(i);
    for (TopExp_Explorer anEdges(aFace, TopAbs_EDGE); anEdges.More(); anEdges.Next()) {
      const TopoDS_Edge& anEdge = TopoDS::Edge(anEdges.Current());
      if (!myEMap.Contains(anEdge)) {
        myEMap.Add(anEdge, Draft_EdgeInfo(Standard_True));
      }
      myEMap.ChangeFromKey(anEdge).Add(aFace);

      for (TopExp_Explorer aVertices(anEdge, TopAbs_VERTEX); aVertices.More(); aVertices.Next()) {
        const TopoDS_Vertex& aVertex = TopoDS::Vertex(aVertices.Current());
        if (!myVMap.Contains(aVertex)) {
          myVMap.Add(aVertex, Draft_VertexInfo());
        }
        Draft_VertexInfo& anInfo = myVMap.ChangeFromKey(aVertex);
        anInfo.Add(anEdge);
        anInfo.ChangeParameter(anEdge) = BRep_Tool::Parameter(aVertex, anEdge);
      }
    }
  }

  // The other edges at those vertices keep their curves; only their ends move.
  TopTools_IndexedDataMapOfShapeListOfShape aVertexEdges;
  TopExp::MapShapesAndAncestors(myShape, TopAbs_VERTEX, TopAbs_EDGE, aVertexEdges);
  for (Standard_Integer i = 1; i <= myVMap.Extent(); i++) {
    const TopoDS_Vertex& aVertex = myVMap.FindKey(i);
    for (TopTools_ListIteratorOfListOfShape it(aVertexEdges.FindFromKey(aVertex)); it.More(); it.Next()) {
      const TopoDS_Edge& anEdge = TopoDS::Edge(it.Value());
      if (!myEMap.Contains(anEdge)) {
        myEMap.Add(anEdge, Draft_EdgeInfo(Standard_False));
      }
      Draft_VertexInfo& anInfo = myVMap.ChangeFromIndex(i);
      anInfo.Add(anEdge);
      anInfo.ChangeParameter(anEdge) = BRep_Tool::Parameter(aVertex, anEdge);
    }
  }

  for (Standard_Integer i = 1; i <= myEMap.Extent(); i++) {
    const TopoDS_Edge& anEdge = myEMap.FindKey(i);
    for (TopTools_ListIteratorOfListOfShape it(myEFMap.FindFromKey(anEdge)); it.More(); it.Next()) {
      const TopoDS_Face& aFace = TopoDS::Face(it.Value());
      if (!myFMap.Contains(aFace)) {
        TopLoc_Location aLoc;
        const Handle(Geom_Surface)& aSurface = BRep_Tool::Surface(aFace, aLoc);
        Handle(Geom_Surface) aStill =
          Handle(Geom_Surface)::DownCast(aSurface->Transformed(aLoc.Transformation()));

        // Bounded well past the face, as for a moved one (see InternalAdd).
        const Handle(Standard_Type)& aType = aSurface->DynamicType();
        if (aType == STANDARD_TYPE(Geom_CylindricalSurface) || aType == STANDARD_TYPE(Geom_ConicalSurface)) {
          Standard_Real aUMin, aUMax, aVMin, aVMax;
          BRepTools::UVBounds(aFace, aUMin, aUMax, aVMin, aVMax);
          if (!Precision::IsNegativeInfinite(aVMin) && !Precision::IsPositiveInfinite(aVMax)) {
            const Standard_Real aMargin = 10. * (aVMax - aVMin);
            aStill = new Geom_RectangularTrimmedSurface(aStill, 0., 2. * M_PI, aVMin - aMargin, aVMax + aMargin);
          }
        }
        myFMap.Add(aFace, Draft_FaceInfo(aStill, Standard_False));
      }
      myEMap.ChangeFromKey(anEdge).Add(aFace);
    }
  }
  return Standard_True;
}

//=======================================================================
//function : Perform
//purpose  :
//=======================================================================

void Tweak_Modification::Perform()
{
  if (!badShape.IsNull()) {
    throw Standard_ConstructionError();
  }
  if (!myComp) {
    myComp = Standard_True;
    if (!Propagate() || !ComputeEdges() || !ComputeVertices()) {
      return;
    }
  }
  OrderVertexParameters();
}

//=======================================================================
//function : ComputeEdges
//purpose  : The new curve of every edge gathered by Propagate.
//=======================================================================

Standard_Boolean Tweak_Modification::ComputeEdges()
{
  for (Standard_Integer i = 1; i <= myEMap.Extent(); i++) {
    const TopoDS_Edge& anEdge = myEMap.FindKey(i);
    Draft_EdgeInfo& anInfo = myEMap.ChangeFromIndex(i);
    // A degenerated edge has no curve; seams, free and tangent edges of the
    // moved faces got theirs in InternalAdd.
    if (BRep_Tool::Degenerated(anEdge) || !anInfo.Geometry().IsNull()) {
      continue;
    }

    TopLoc_Location aLoc;
    Standard_Real aFirst, aLast;
    Handle(Geom_Curve) aCurve = BRep_Tool::Curve(anEdge, aLoc, aFirst, aLast);
    aCurve = Handle(Geom_Curve)::DownCast(aCurve->Transformed(aLoc.Transformation()));
    if (anInfo.NewGeometry()) {
      // Between faces moving alike the edge moves with them; elsewhere it runs
      // where a moved surface now meets one that stays.
      const gp_Trsf* aMove = RigidMove(anEdge);
      if (aMove != NULL) {
        aCurve = Handle(Geom_Curve)::DownCast(aCurve->Transformed(*aMove));
      }
      else {
        aCurve = IntersectionCurve(anEdge, aCurve, anInfo);
        if (aCurve.IsNull()) {
          return Standard_False;
        }
      }
    }
    Handle(Geom_TrimmedCurve) aTrimmed = Handle(Geom_TrimmedCurve)::DownCast(aCurve);
    anInfo.ChangeGeometry() = aTrimmed.IsNull() ? aCurve : aTrimmed->BasisCurve();
  }
  return Standard_True;
}

//=======================================================================
//function : IntersectionCurve
//purpose  : The curve along which the surfaces of the faces of E now meet:
//           the branch nearest E, running E's way, with pcurves where they
//           come for free. Null, with the error set, if they do not meet.
//=======================================================================

Handle(Geom_Curve) Tweak_Modification::IntersectionCurve(const TopoDS_Edge& E,
                                                         const Handle(Geom_Curve)& OldC,
                                                         Draft_EdgeInfo& EI)
{
  const TopoDS_Face& F1 = EI.FirstFace();
  const TopoDS_Face& F2 = EI.SecondFace();
  const Handle(Geom_Surface) S1 = myFMap.FindFromKey(F1).Geometry();
  const Handle(Geom_Surface) S2 = myFMap.FindFromKey(F2).Geometry();
  const Handle(Geom_Surface) aBasis1 = Basis(S1);
  const Handle(Geom_Surface) aBasis2 = Basis(S2);

  // The old edge's ends, and its tangent at the first.
  gp_Pnt aFirst, aLast;
  gp_Vec aFirstTangent;
  OldC->D1(BRep_Tool::Parameter(TopExp::FirstVertex(E), E), aFirst, aFirstTangent);
  OldC->D0(BRep_Tool::Parameter(TopExp::LastVertex(E), E), aLast);

  // A plane across an extrusion of a circle parallel to it meets it along
  // that circle, slid along the extrusion.
  Handle(Geom_SurfaceOfLinearExtrusion) anExtrusion = Handle(Geom_SurfaceOfLinearExtrusion)::DownCast(aBasis1);
  Handle(Geom_Plane) aPlane = Handle(Geom_Plane)::DownCast(aBasis2);
  Standard_Boolean isExtrusionFirst = Standard_True;
  if (anExtrusion.IsNull() || aPlane.IsNull()) {
    anExtrusion = Handle(Geom_SurfaceOfLinearExtrusion)::DownCast(aBasis2);
    aPlane = Handle(Geom_Plane)::DownCast(aBasis1);
    isExtrusionFirst = Standard_False;
  }
  if (!anExtrusion.IsNull() && !aPlane.IsNull()) {
    Handle(Geom_Circle) aCircle = Handle(Geom_Circle)::DownCast(anExtrusion->BasisCurve());
    const gp_Ax3& anAxis = aPlane->Position();
    if (!aCircle.IsNull()
        && aCircle->Position().Direction().IsParallel(anAxis.Direction(), Precision::Angular())) {
      const Standard_Real aHeight =
        gp_Vec(aCircle->Circ().Location(), anAxis.Location()).Dot(anAxis.Direction());
      const Standard_Real aSlide = aHeight / anExtrusion->Direction().Dot(anAxis.Direction());
      Handle(Geom2d_Line) aPCurve = new Geom2d_Line(gp_Pnt2d(0., aSlide), gp::DX2d());
      if (isExtrusionFirst) {
        EI.ChangeFirstPC() = aPCurve;
      }
      else {
        EI.ChangeSecondPC() = aPCurve;
      }
      return Handle(Geom_Curve)::DownCast(aCircle->Translated(aSlide * gp_Vec(anExtrusion->Direction())));
    }
  }

  // Pcurves are projected later, over the edge alone: approximating them
  // along the whole intersection would be wasted, and could break SameRange
  // once the curve is reversed.
  GeomInt_IntSS anInter;
  anInter.Perform(S1, S2, Precision::Confusion(), Standard_True, Standard_False, Standard_False);
  if (!anInter.IsDone() || anInter.NbLines() <= 0) {
    errStat = Draft_EdgeRecomputation;
    badShape = E;
    return NULL;
  }

  Handle(Geom_Curve) aCurve;
  Standard_Real aParam = 0.;
  if (anInter.Line(1)->DynamicType() != STANDARD_TYPE(Geom_BSplineCurve)) {
    // The old edge lies on the surface that stays, which tells its sides.
    Handle(Geom_Surface) aStill;
    if (OnSurface(aFirst, aLast, S1, BRep_Tool::Tolerance(F1))) {
      aStill = aBasis1;
    }
    if (OnSurface(aFirst, aLast, S2, BRep_Tool::Tolerance(F2))) {
      aStill = aBasis2;
    }
    const Standard_Integer aLine = NearestLine(anInter, aFirst, aLast, aStill, aParam);
    if (aLine == 0) {
      errStat = Draft_EdgeRecomputation;
      badShape = E;
      return NULL;
    }
    aCurve = anInter.Line(aLine);
    const Standard_Boolean isReversed = IsReversed(aCurve, aParam, aFirst, aFirstTangent, F1, F2, S1, S2);
    if (isReversed) {
      aCurve->Reverse();
    }
    if (anInter.HasLineOnS1(aLine)) {
      EI.ChangeFirstPC() = anInter.LineOnS1(aLine);
      if (isReversed) {
        EI.ChangeFirstPC()->Reverse();
      }
    }
    if (anInter.HasLineOnS2(aLine)) {
      EI.ChangeSecondPC() = anInter.LineOnS2(aLine);
      if (isReversed) {
        EI.ChangeSecondPC()->Reverse();
      }
    }
  }
  else {
    // An approximated intersection comes in pieces: glue them into one,
    // starting from the piece nearest the old edge's first end.
    aCurve = GluedLines(anInter, S1, aFirst);
    if (aCurve.IsNull()) {
      errStat = Draft_EdgeRecomputation;
      badShape = E;
      return NULL;
    }
    GeomAdaptor_Curve anAdaptor(aCurve);
    Extrema_ExtPC anExt(aFirst, anAdaptor);
    Standard_Real aDist2Min = RealLast();
    for (Standard_Integer i = 1; i <= anExt.NbExt(); i++) {
      if (anExt.IsMin(i) && anExt.SquareDistance(i) < aDist2Min) {
        aDist2Min = anExt.SquareDistance(i);
        aParam = anExt.Point(i).Parameter();
      }
    }
    if (IsReversed(aCurve, aParam, aFirst, aFirstTangent, F1, F2, S1, S2)) {
      aCurve->Reverse();
    }
  }
  EI.Tolerance(Max(EI.Tolerance(), anInter.TolReached3d()));
  return aCurve;
}

//=======================================================================
//function : ComputeVertices
//purpose  : The new point of every vertex gathered by Propagate, and its
//           parameters on the new curves of its edges.
//=======================================================================

Standard_Boolean Tweak_Modification::ComputeVertices()
{
  Handle(GeomAdaptor_Curve) aCurve = new GeomAdaptor_Curve;
  Handle(GeomAdaptor_Surface) aSurface = new GeomAdaptor_Surface;
  for (Standard_Integer i = 1; i <= myVMap.Extent(); i++) {
    const TopoDS_Vertex& aVertex = myVMap.FindKey(i);
    Draft_VertexInfo& anInfo = myVMap.ChangeFromIndex(i);
    gp_Pnt aPnt;
    const gp_Trsf* aMove = RigidMove(aVertex);
    if (aMove != NULL) {
      aPnt = BRep_Tool::Pnt(aVertex).Transformed(*aMove);
    }
    else if (Choose(myFMap, myEMap, aVertex, anInfo, *aCurve, *aSurface)) {
      if (!NearestMeeting(aCurve, aSurface, BRep_Tool::Pnt(aVertex), aPnt)) {
        errStat = Draft_VertexRecomputation;
        badShape = aVertex;
        return Standard_False;
      }
    }
    else {
      if (!AlignVertex(aVertex, anInfo)) {
        errStat = Draft_VertexRecomputation;
        badShape = aVertex;
        return Standard_False;
      }
      continue;
    }
    anInfo.ChangeGeometry() = aPnt;

    for (anInfo.InitEdgeIterator(); anInfo.MoreEdge(); anInfo.NextEdge()) {
      const TopoDS_Edge& anEdge = anInfo.Edge();
      Draft_EdgeInfo& anEdgeInfo = myEMap.ChangeFromKey(anEdge);
      if (anEdgeInfo.Geometry().IsNull()) {
        continue;
      }
      const Standard_Real anOldParam = anInfo.Parameter(anEdge);
      Standard_Integer done;
      const Standard_Real aParam = ParameterOn(anEdgeInfo.Geometry(), myFMap, anEdgeInfo, anEdge, aPnt, done);
      // A vertex moved along a trimmed curve may leave its trim: the edge
      // takes the untrimmed curve instead.
      if (done == 0 && Abs(anOldParam - aParam) > Precision::PConfusion()) {
        TopLoc_Location aLoc;
        Standard_Real aFirst, aLast;
        if (BRep_Tool::Curve(anEdge, aLoc, aFirst, aLast)->DynamicType() == STANDARD_TYPE(Geom_TrimmedCurve)) {
          anEdgeInfo.SetNewGeometry(Standard_True);
        }
      }
      anInfo.ChangeParameter(anEdge) = aParam;
    }
  }
  return Standard_True;
}

//=======================================================================
//function : AlignVertex
//purpose  : Places vertex V where no edge at it crosses another, between
//           edges running on from one another: on the first edge's curve
//           nearest V, or at the second's point there if that is nearer.
//=======================================================================

Standard_Boolean Tweak_Modification::AlignVertex(const TopoDS_Vertex& V, Draft_VertexInfo& VI)
{
  VI.InitEdgeIterator();
  while (VI.MoreEdge() && myEMap.FindFromKey(VI.Edge()).Geometry().IsNull()) {
    VI.NextEdge();
  }
  if (!VI.MoreEdge()) {
    return Standard_False;
  }
  const TopoDS_Edge& anEdge1 = VI.Edge();
  Draft_EdgeInfo& anInfo1 = myEMap.ChangeFromKey(anEdge1);
  const gp_Pnt anOld = BRep_Tool::Pnt(V);
  gp_Pnt aPnt = GeomAPI_ProjectPointOnCurve(anOld, anInfo1.Geometry()).NearestPoint();
  const Standard_Real aDist2 = aPnt.SquareDistance(anOld);

  VI.NextEdge();
  while (VI.MoreEdge() && myEMap.FindFromKey(VI.Edge()).Geometry().IsNull()) {
    VI.NextEdge();
  }
  Standard_Integer done;
  if (VI.MoreEdge()) {
    const TopoDS_Edge& anEdge2 = VI.Edge();
    Draft_EdgeInfo& anInfo2 = myEMap.ChangeFromKey(anEdge2);
    gp_Pnt aPnt2;
    anInfo2.Geometry()->D0(VI.Parameter(anEdge2), aPnt2);
    if (aPnt2.SquareDistance(anOld) < aDist2) {
      aPnt = aPnt2;
    }
    VI.ChangeParameter(anEdge2) = ParameterOn(anInfo2.Geometry(), myFMap, anInfo2, anEdge2, aPnt, done);
  }
  VI.ChangeGeometry() = aPnt;
  VI.ChangeParameter(anEdge1) = ParameterOn(anInfo1.Geometry(), myFMap, anInfo1, anEdge1, aPnt, done);
  return Standard_True;
}

//=======================================================================
//function : OrderVertexParameters
//purpose  : Makes every edge run from its first vertex to its last along
//           its new curve, unrolling closed curves. An edge whose ends have
//           crossed is an error: the move needs the topology to change.
//=======================================================================

void Tweak_Modification::OrderVertexParameters()
{
  for (Standard_Integer i = 1; i <= myEMap.Extent(); i++) {
    const TopoDS_Edge& anEdge = myEMap.FindKey(i);
    TopoDS_Vertex aVF, aVL;
    TopExp::Vertices(anEdge, aVF, aVL);
    if (anEdge.Orientation() == TopAbs_REVERSED) {
      aVF.Reverse();
      aVL.Reverse();
    }

    // An intersection curve running against the old edge at its ends is
    // reversed. (A rigidly moved curve is exact; the old directions are no
    // guide to it once the move turns it far.)
    if (myVMap.Contains(aVF) && myVMap.Contains(aVL) && RigidMove(anEdge) == NULL) {
      const Standard_Real aParF = myVMap.ChangeFromKey(aVF).Parameter(anEdge);
      const Standard_Real aParL = myVMap.ChangeFromKey(aVL).Parameter(anEdge);
      if (aParL < aParF) {
        Draft_EdgeInfo& anInfo = myEMap.ChangeFromKey(anEdge);
        Standard_Real aFirst = 0., aLast = 0.;
        const Handle(Geom_Curve) anOldCurve = BRep_Tool::Curve(anEdge, aFirst, aLast);
        Handle(Geom_Curve) aNewCurve = anInfo.Geometry();
        gp_Pnt aPF, aPL;
        gp_Vec anOldDirF, anOldDirL, aNewDirF, aNewDirL;
        anOldCurve->D1(BRep_Tool::Parameter(aVF, anEdge), aPF, anOldDirF);
        anOldCurve->D1(BRep_Tool::Parameter(aVL, anEdge), aPL, anOldDirL);
        aNewCurve->D1(aParF, aPF, aNewDirF);
        aNewCurve->D1(aParL, aPL, aNewDirL);
        const Standard_Real aCosF = Normalized(aNewDirF).Dot(Normalized(anOldDirF));
        const Standard_Real aCosL = Normalized(aNewDirL).Dot(Normalized(anOldDirL));
        if ((Abs(aCosF) > Abs(aCosL) ? aCosF : aCosL) < 0.) {
          aNewCurve->Reverse();
          anInfo.ChangeGeometry() = aNewCurve;
          Standard_Integer anErr = 0;
          Standard_Real aPar = Parameter(aNewCurve, aPF, anErr);
          if (anErr == 0) {
            myVMap.ChangeFromKey(aVF).ChangeParameter(anEdge) = aPar;
          }
          aPar = Parameter(aNewCurve, aPL, anErr);
          if (anErr == 0) {
            myVMap.ChangeFromKey(aVL).ChangeParameter(anEdge) = aPar;
          }
        }
      }
    }

    Standard_Real aPF, aPL, aTol;
    if (!NewParameter(aVF, anEdge, aPF, aTol)) {
      aPF = BRep_Tool::Parameter(aVF, anEdge);
    }
    if (!NewParameter(aVL, anEdge, aPL, aTol)) {
      aPL = BRep_Tool::Parameter(aVL, anEdge);
    }
    if (aPL <= aPF) {
      const Handle(Geom_Curve)& aCurve = myEMap.FindFromKey(anEdge).Geometry();
      if (aCurve->IsClosed()) {
        const Standard_Real aStart = aCurve->FirstParameter(), anEnd = aCurve->LastParameter();
        if (Abs(aPF - anEnd) <= Precision::PConfusion()) {
          aPF = aStart;
        }
        else if (Abs(aPL - aStart) <= Precision::PConfusion()) {
          aPL = anEnd;
        }
        if (aPL <= aPF) {
          aPL += anEnd - aStart;
        }
      }
      if (aPL <= aPF) {
        errStat = Draft_EdgeRecomputation;
        badShape = anEdge;
        return;
      }
    }
    if (myVMap.Contains(aVF)) {
      myVMap.ChangeFromKey(aVF).ChangeParameter(anEdge) = aPF;
    }
    if (myVMap.Contains(aVL)) {
      myVMap.ChangeFromKey(aVL).ChangeParameter(anEdge) = aPL;
    }
  }
}

//=======================================================================
//function : SurfaceNormal
//purpose  : The normal of S at its point nearest P, unnormalised; null if
//           P does not project onto S.
//=======================================================================

static gp_Vec SurfaceNormal(const Handle(Geom_Surface)& S, const gp_Pnt& P)
{
  GeomAPI_ProjectPointOnSurf aProj(P, S);
  if (!aProj.IsDone() || aProj.NbPoints() == 0) {
    return gp_Vec();
  }
  Standard_Real aU, aV;
  aProj.LowerDistanceParameters(aU, aV);
  gp_Pnt aPnt;
  gp_Vec aDU, aDV;
  S->D1(aU, aV, aPnt, aDU, aDV);
  return aDU.Crossed(aDV);
}

//=======================================================================
//function : IsReversed
//purpose  : Whether NewC, at parameter NewPar, runs against the edge it
//           replaces, which passes OldP with tangent OldT between faces F1
//           and F2, now on surfaces NewS1 and NewS2. An edge keeps its
//           sense relative to the normals of the faces it joins, however
//           far they move; where that sense is undefined, the tangents
//           are compared.
//=======================================================================

static Standard_Boolean IsReversed(const Handle(Geom_Curve)& NewC,
                                   const Standard_Real NewPar,
                                   const gp_Pnt& OldP,
                                   const gp_Vec& OldT,
                                   const TopoDS_Face& F1,
                                   const TopoDS_Face& F2,
                                   const Handle(Geom_Surface)& NewS1,
                                   const Handle(Geom_Surface)& NewS2)
{
  gp_Pnt aNewP;
  gp_Vec aNewT;
  NewC->D1(NewPar, aNewP, aNewT);

  const gp_Vec anOldAcross =
    SurfaceNormal(BRep_Tool::Surface(F1), OldP).Crossed(SurfaceNormal(BRep_Tool::Surface(F2), OldP));
  const gp_Vec aNewAcross = SurfaceNormal(NewS1, aNewP).Crossed(SurfaceNormal(NewS2, aNewP));
  const Standard_Real anOldSense = OldT.Dot(anOldAcross);
  const Standard_Real aNewSense = aNewT.Dot(aNewAcross);
  const Standard_Real anOldTiny = Precision::Angular() * OldT.Magnitude() * anOldAcross.Magnitude();
  const Standard_Real aNewTiny = Precision::Angular() * aNewT.Magnitude() * aNewAcross.Magnitude();
  if (Abs(anOldSense) <= anOldTiny || Abs(aNewSense) <= aNewTiny) {
    return OldT.Dot(aNewT) < 0.;
  }
  return (anOldSense > 0.) != (aNewSense > 0.);
}

//=======================================================================
//function : SameMove
//purpose  : Whether two transforms are the same move.
//=======================================================================

static Standard_Boolean SameMove(const gp_Trsf& T1, const gp_Trsf& T2)
{
  for (Standard_Integer aRow = 1; aRow <= 3; aRow++) {
    for (Standard_Integer aCol = 1; aCol <= 4; aCol++) {
      if (Abs(T1.Value(aRow, aCol) - T2.Value(aRow, aCol)) > Precision::Confusion()) {
        return Standard_False;
      }
    }
  }
  return Standard_True;
}

//=======================================================================
//function : Basis
//purpose  : S, or the surface it trims.
//=======================================================================

static Handle(Geom_Surface) Basis(const Handle(Geom_Surface)& S)
{
  Handle(Geom_RectangularTrimmedSurface) aTrimmed = Handle(Geom_RectangularTrimmedSurface)::DownCast(S);
  return aTrimmed.IsNull() ? S : aTrimmed->BasisSurface();
}

//=======================================================================
//function : Normalized
//purpose  : V scaled to unit length, or as it is when too short to tell.
//=======================================================================

static gp_Vec Normalized(const gp_Vec& V)
{
  const Standard_Real aSqMagn = V.SquareMagnitude();
  return aSqMagn > Precision::SquareConfusion() ? V / sqrt(aSqMagn) : V;
}

//=======================================================================
//function : OnSurface
//purpose  : Whether P1 and P2 both lie on S, projecting within Tol.
//=======================================================================

static Standard_Boolean OnSurface(const gp_Pnt& P1,
                                  const gp_Pnt& P2,
                                  const Handle(Geom_Surface)& S,
                                  const Standard_Real Tol)
{
  GeomAPI_ProjectPointOnSurf aProj1(P1, S, Tol);
  GeomAPI_ProjectPointOnSurf aProj2(P2, S, Tol);
  return aProj1.IsDone() && aProj2.IsDone() && aProj1.LowerDistance() <= Precision::Confusion()
         && aProj2.LowerDistance() <= Precision::Confusion();
}

//=======================================================================
//function : SurfaceParameters
//purpose  : The parameters of P on S, if S is a plane or a cylinder.
//=======================================================================

static Standard_Boolean SurfaceParameters(const Handle(Geom_Surface)& S,
                                          const gp_Pnt& P,
                                          Standard_Real& U,
                                          Standard_Real& V)
{
  Handle(Geom_Plane) aPlane = Handle(Geom_Plane)::DownCast(S);
  if (!aPlane.IsNull()) {
    ElSLib::Parameters(aPlane->Pln(), P, U, V);
    return Standard_True;
  }
  Handle(Geom_CylindricalSurface) aCylinder = Handle(Geom_CylindricalSurface)::DownCast(S);
  if (!aCylinder.IsNull()) {
    ElSLib::Parameters(aCylinder->Cylinder(), P, U, V);
    return Standard_True;
  }
  return Standard_False;
}

//=======================================================================
//function : NearestLine
//purpose  : The line of Inter nearest First, the old edge's first end,
//           with the parameter there; 0 if no line could be measured.
//           Where a line passes two points about equally near First, the
//           one on First's side of the old edge wins, sides being told in
//           the parameters of Still, the surface the old edge lies on.
//=======================================================================

static Standard_Integer NearestLine(const GeomInt_IntSS& Inter,
                                    const gp_Pnt& First,
                                    const gp_Pnt& Last,
                                    const Handle(Geom_Surface)& Still,
                                    Standard_Real& Param)
{
  Standard_Real aBestDist2 = RealLast();
  Standard_Integer aBest = 0;
  // Once sides cannot be told on Still, the nearest point decides throughout.
  Standard_Boolean isSidesUnknown = Standard_False;
  for (Standard_Integer i = 1; i <= Inter.NbLines(); i++) {
    const Handle(Geom_Curve)& aLine = Inter.Line(i);
    GeomAdaptor_Curve anAdaptor(aLine);
    Extrema_ExtPC anExt(First, anAdaptor);
    if (!anExt.IsDone()) {
      continue;
    }
    const Standard_Integer aNbExt = anExt.NbExt();
    Standard_Real aDist2 = RealLast(), aParam = 0.;
    if (aNbExt >= 1) {
      aDist2 = anExt.SquareDistance(1);
      aParam = anExt.Point(1).Parameter();
    }
    if (aNbExt == 2 && aDist2 > Precision::SquareConfusion()) {
      // Two candidates: the clearly nearer, or else the one on First's side.
      const Standard_Real aDist2A = anExt.SquareDistance(1), aDist2B = anExt.SquareDistance(2);
      if (aDist2A > 1.21 * aDist2B) {
        aDist2 = aDist2B;
        aParam = anExt.Point(2).Parameter();
      }
      else if (aDist2B > 1.21 * aDist2A) {
        aDist2 = aDist2A;
        aParam = anExt.Point(1).Parameter();
      }
      else if (!Still.IsNull() && !isSidesUnknown) {
        const gp_Pnt aPntA = aLine->Value(anExt.Point(1).Parameter());
        const gp_Pnt aPntB = aLine->Value(anExt.Point(2).Parameter());
        Standard_Real aUF, aVF, aUL, aVL, aUA, aVA, aUB, aVB;
        if (SurfaceParameters(Still, First, aUF, aVF) && SurfaceParameters(Still, Last, aUL, aVL)
            && SurfaceParameters(Still, aPntA, aUA, aVA) && SurfaceParameters(Still, aPntB, aUB, aVB)) {
          const gp_Vec2d anAcross(aVF - aVA, aUA - aUF);
          const gp_Vec2d aToLast(aUL - aUA, aVL - aVA);
          const gp_Vec2d aToB(aUB - aUA, aVB - aVA);
          if (aToLast.Dot(anAcross) * aToB.Dot(anAcross) < 0.) {
            aDist2 = aDist2B;
            aParam = anExt.Point(2).Parameter();
          }
        }
        else {
          isSidesUnknown = Standard_True;
        }
      }
    }
    if (aNbExt >= 1 && (aNbExt != 2 || isSidesUnknown)) {
      aDist2 = anExt.SquareDistance(1);
      aParam = anExt.Point(1).Parameter();
      for (Standard_Integer j = 2; j <= aNbExt; j++) {
        if (anExt.SquareDistance(j) < aDist2) {
          aDist2 = anExt.SquareDistance(j);
          aParam = anExt.Point(j).Parameter();
        }
      }
    }
    else if (aNbExt < 1) {
      // Nearest at an end of the line.
      Standard_Real aDist2First, aDist2Last;
      gp_Pnt aPntFirst, aPntLast;
      anExt.TrimmedSquareDistances(aDist2First, aDist2Last, aPntFirst, aPntLast);
      if (aDist2First < aDist2Last) {
        aDist2 = aDist2First;
        aParam = anAdaptor.FirstParameter();
      }
      else {
        aDist2 = aDist2Last;
        aParam = anAdaptor.LastParameter();
      }
    }

    if (aDist2 < aBestDist2) {
      aBestDist2 = aDist2;
      aBest = i;
      Param = aParam;
    }
  }
  return aBest;
}

//=======================================================================
//function : GluedLines
//purpose  : The B-spline pieces of Inter glued end to end into one curve,
//           starting from the piece nearest First. On a cylinder or cone
//           S1, pieces starting or ending on its seam are preferred.
//=======================================================================

static Handle(Geom_Curve) GluedLines(const GeomInt_IntSS& Inter,
                                     const Handle(Geom_Surface)& S1,
                                     const gp_Pnt& First)
{
  TColGeom_SequenceOfCurve aCandidates;
  if (S1->DynamicType() == STANDARD_TYPE(Geom_CylindricalSurface)
      || S1->DynamicType() == STANDARD_TYPE(Geom_ConicalSurface)) {
    for (Standard_Integer i = 1; i <= Inter.NbLines(); i++) {
      const Handle(Geom_Curve)& aLine = Inter.Line(i);
      Standard_Real aU, aV;
      GeomAPI_ProjectPointOnSurf aProj(aLine->Value(aLine->FirstParameter()), S1, Precision::Confusion());
      aProj.LowerDistanceParameters(aU, aV);
      if (Abs(aU) <= Precision::Confusion() || Abs(aU - 2. * M_PI) <= Precision::Confusion()) {
        aCandidates.Append(aLine);
        continue;
      }
      aProj.Init(aLine->Value(aLine->LastParameter()), S1, Precision::Confusion());
      aProj.LowerDistanceParameters(aU, aV);
      if (Abs(aU) <= Precision::Confusion() || Abs(aU - 2. * M_PI) <= Precision::Confusion()) {
        aLine->Reverse();
        aCandidates.Append(aLine);
      }
    }
  }
  if (aCandidates.IsEmpty()) {
    for (Standard_Integer i = 1; i <= Inter.NbLines(); i++) {
      aCandidates.Append(Inter.Line(i));
    }
  }

  Handle(Geom_Curve) aStart = aCandidates(1);
  if (aCandidates.Length() > 1) {
    aStart.Nullify();
    Standard_Real aDistMin = Precision::Infinite();
    for (Standard_Integer i = 1; i <= aCandidates.Length(); i++) {
      const Handle(Geom_Curve)& aCandidate = aCandidates(i);
      const Standard_Real aDist = aCandidate->Value(aCandidate->FirstParameter()).Distance(First);
      if (aDist - aDistMin < -Precision::Confusion()) {
        aDistMin = aDist;
        aStart = aCandidate;
      }
    }
    if (aStart.IsNull()) {
      return aStart;
    }
  }

  TColGeom_SequenceOfCurve aRest;
  for (Standard_Integer i = 1; i <= Inter.NbLines(); i++) {
    if (aStart != Inter.Line(i)) {
      aRest.Append(Inter.Line(i));
    }
  }
  GeomConvert_CompCurveToBSplineCurve aGlued(Handle(Geom_BSplineCurve)::DownCast(aStart));
  gp_Pnt anEnd = aStart->Value(aStart->LastParameter());
  for (Standard_Boolean isAdded = Standard_True; isAdded;) {
    isAdded = Standard_False;
    for (Standard_Integer i = 1; i <= aRest.Length(); i++) {
      Handle(Geom_Curve) aPiece = aRest(i);
      const gp_Pnt aPieceFirst = aPiece->Value(aPiece->FirstParameter());
      const gp_Pnt aPieceLast = aPiece->Value(aPiece->LastParameter());
      if (aPieceFirst.Distance(anEnd) <= Precision::Confusion()) {
        anEnd = aPieceLast;
      }
      else if (aPieceLast.Distance(anEnd) <= Precision::Confusion()) {
        aPiece->Reverse();
        anEnd = aPieceFirst;
      }
      else {
        continue;
      }
      aGlued.Add(Handle(Geom_BSplineCurve)::DownCast(aPiece), Precision::Confusion(), Standard_True);
      aRest.Remove(i);
      isAdded = Standard_True;
      break;
    }
  }
  return aGlued.BSplineCurve();
}

//=======================================================================
//function : NearestMeeting
//purpose  : Where C meets S nearest Near; failing a meeting, the point of
//           C nearest S. False if neither can be found.
//=======================================================================

static Standard_Boolean NearestMeeting(const Handle(GeomAdaptor_Curve)& C,
                                       const Handle(GeomAdaptor_Surface)& S,
                                       const gp_Pnt& Near,
                                       gp_Pnt& Result)
{
  IntCurveSurface_HInter anInter;
  anInter.Perform(C, S);
  if (!anInter.IsDone()) {
    return Standard_False;
  }
  Standard_Real aDist2Min = RealLast();
  if (anInter.NbPoints() > 0) {
    for (Standard_Integer i = 1; i <= anInter.NbPoints(); i++) {
      const gp_Pnt& aPnt = anInter.Point(i).Pnt();
      if (aPnt.SquareDistance(Near) < aDist2Min) {
        aDist2Min = aPnt.SquareDistance(Near);
        Result = aPnt;
      }
    }
    return Standard_True;
  }

  Extrema_ExtCS anExt(*C, *S, Precision::PConfusion(), Precision::PConfusion());
  if (!anExt.IsDone() || anExt.NbExt() == 0) {
    return Standard_False;
  }
  Extrema_POnCurv anOnCurve;
  Extrema_POnSurf anOnSurface;
  for (Standard_Integer i = 1; i <= anExt.NbExt(); i++) {
    anExt.Points(i, anOnCurve, anOnSurface);
    if (anOnCurve.Value().SquareDistance(Near) < aDist2Min) {
      aDist2Min = anOnCurve.Value().SquareDistance(Near);
      Result = anOnCurve.Value();
    }
  }
  return Standard_True;
}

//=======================================================================
//function : ParameterOn
//purpose  : The parameter of P on C (the curve of edge E, old or new). If
//           P lies past an end, E's new curve is extended to reach it (see
//           SmartParameter) and done tells which end.
//=======================================================================

static Standard_Real ParameterOn(const Handle(Geom_Curve)& C,
                                 const Draft_IndexedDataMapOfFaceFaceInfo& FMap,
                                 Draft_EdgeInfo& EI,
                                 const TopoDS_Edge& E,
                                 const gp_Pnt& P,
                                 Standard_Integer& done)
{
  const Standard_Real aParam = Parameter(C, P, done);
  if (done == 0) {
    return aParam;
  }
  return SmartParameter(EI,
                        BRep_Tool::Tolerance(E),
                        P,
                        done,
                        FMap.FindFromKey(EI.FirstFace()).Geometry(),
                        FMap.FindFromKey(EI.SecondFace()).Geometry());
}

//=======================================================================
//function : Choose
//purpose  : A curve C and a surface S whose meeting places vertex V: the
//           new curve of an edge at V (preferring a free or smooth one),
//           and the new surface of a face across a sharp edge at V that
//           runs another way. False if no edge at V runs another way.
//=======================================================================

static Standard_Boolean Choose(const Draft_IndexedDataMapOfFaceFaceInfo& FMap,
                               Draft_IndexedDataMapOfEdgeEdgeInfo& EMap,
                               const TopoDS_Vertex& V,
                               Draft_VertexInfo& VI,
                               GeomAdaptor_Curve& C,
                               GeomAdaptor_Surface& S)
{
  for (VI.InitEdgeIterator(); VI.MoreEdge(); VI.NextEdge()) {
    const TopoDS_Edge& anEdge = VI.Edge();
    const Draft_EdgeInfo& anInfo = EMap.FindFromKey(anEdge);
    if (!anInfo.Geometry().IsNull()
        && (anInfo.SecondFace().IsNull()
            || BRep_Tool::Continuity(anEdge, anInfo.FirstFace(), anInfo.SecondFace()) >= GeomAbs_G1)) {
      break;
    }
  }
  if (!VI.MoreEdge()) {
    VI.InitEdgeIterator();
    while (VI.MoreEdge() && EMap.FindFromKey(VI.Edge()).Geometry().IsNull()) {
      VI.NextEdge();
    }
    if (!VI.MoreEdge()) {
      return Standard_False;
    }
  }

  const TopoDS_Edge& aRef = VI.Edge();
  Draft_EdgeInfo& aRefInfo = EMap.ChangeFromKey(aRef);
  C.Load(aRefInfo.Geometry());

  // The old edge's direction at V.
  const gp_Pnt anOld = BRep_Tool::Pnt(V);
  TopLoc_Location aLoc;
  Standard_Real aFirst, aLast;
  Handle(Geom_Curve) anOldCurve = BRep_Tool::Curve(aRef, aLoc, aFirst, aLast);
  anOldCurve = Handle(Geom_Curve)::DownCast(anOldCurve->Transformed(aLoc.Transformation()));
  Standard_Integer done;
  gp_Pnt aPnt;
  gp_Vec aRefTangent;
  anOldCurve->D1(ParameterOn(anOldCurve, FMap, aRefInfo, aRef, anOld, done), aPnt, aRefTangent);

  for (VI.InitEdgeIterator(); VI.MoreEdge(); VI.NextEdge()) {
    const TopoDS_Edge& anEdge = VI.Edge();
    if (anEdge.IsSame(aRef)) {
      continue;
    }
    Draft_EdgeInfo& anInfo = EMap.ChangeFromKey(anEdge);
    if (anInfo.SecondFace().IsNull()
        || BRep_Tool::Continuity(anEdge, anInfo.FirstFace(), anInfo.SecondFace()) > GeomAbs_C0) {
      continue;
    }
    anOldCurve = BRep_Tool::Curve(anEdge, aLoc, aFirst, aLast);
    anOldCurve = Handle(Geom_Curve)::DownCast(anOldCurve->Transformed(aLoc.Transformation()));
    gp_Vec aTangent;
    anOldCurve->D1(ParameterOn(anOldCurve, FMap, anInfo, anEdge, anOld, done), aPnt, aTangent);
    if (aTangent.CrossMagnitude(aRefTangent) > Precision::Confusion()) {
      break;
    }
  }
  if (!VI.MoreEdge()) {
    return Standard_False;
  }

  // The face of the crossing edge that is not along the reference edge.
  const Draft_EdgeInfo& aCrossing = EMap.FindFromKey(VI.Edge());
  const Standard_Boolean isFirstShared =
    aCrossing.FirstFace().IsSame(aRefInfo.FirstFace())
    || (!aRefInfo.SecondFace().IsNull() && aCrossing.FirstFace().IsSame(aRefInfo.SecondFace()));
  S.Load(FMap.FindFromKey(isFirstShared ? aCrossing.SecondFace() : aCrossing.FirstFace()).Geometry());
  return Standard_True;
}

//=======================================================================
//function : Parameter
//purpose  : The parameter of P on C. If P lies past an end of C, done is
//           -1 for the first end and 1 for the last, else 0.
//=======================================================================

static Standard_Real Parameter(const Handle(Geom_Curve)& C, const gp_Pnt& P, Standard_Integer& done)
{
  done = 0;
  Handle(Geom_Curve) aBasis = C;
  Handle(Geom_TrimmedCurve) aTrimmed = Handle(Geom_TrimmedCurve)::DownCast(C);
  if (!aTrimmed.IsNull()) {
    aBasis = aTrimmed->BasisCurve();
  }
  const Handle(Standard_Type)& aType = aBasis->DynamicType();

  Standard_Real aParam;
  if (aType == STANDARD_TYPE(Geom_Line)) {
    aParam = ElCLib::Parameter(Handle(Geom_Line)::DownCast(aBasis)->Lin(), P);
  }
  else if (aType == STANDARD_TYPE(Geom_Circle)) {
    aParam = ElCLib::Parameter(Handle(Geom_Circle)::DownCast(aBasis)->Circ(), P);
    if (Abs(2. * M_PI - aParam) <= Epsilon(2. * M_PI)) {
      aParam = 0.;
    }
  }
  else if (aType == STANDARD_TYPE(Geom_Ellipse)) {
    aParam = ElCLib::Parameter(Handle(Geom_Ellipse)::DownCast(aBasis)->Elips(), P);
    if (Abs(2. * M_PI - aParam) <= Epsilon(2. * M_PI)) {
      aParam = 0.;
    }
  }
  else if (aType == STANDARD_TYPE(Geom_Parabola)) {
    aParam = ElCLib::Parameter(Handle(Geom_Parabola)::DownCast(aBasis)->Parab(), P);
  }
  else if (aType == STANDARD_TYPE(Geom_Hyperbola)) {
    aParam = ElCLib::Parameter(Handle(Geom_Hyperbola)::DownCast(aBasis)->Hypr(), P);
  }
  else {
    GeomAdaptor_Curve anAdaptor(C);
    Extrema_ExtPC anExt(P, anAdaptor);
    if (!anExt.IsDone()) {
      throw Standard_Failure("Tweak_Modification::Parameter: ExtremaPC not done.");
    }
    if (anExt.NbExt() >= 1) {
      Standard_Integer aNearest = 1;
      for (Standard_Integer j = 2; j <= anExt.NbExt(); j++) {
        if (anExt.SquareDistance(j) < anExt.SquareDistance(aNearest)) {
          aNearest = j;
        }
      }
      aParam = anExt.Point(aNearest).Parameter();
    }
    else {
      Standard_Real aDist2First, aDist2Last;
      gp_Pnt aPntFirst, aPntLast;
      anExt.TrimmedSquareDistances(aDist2First, aDist2Last, aPntFirst, aPntLast);
      if (aDist2First < aDist2Last) {
        done = -1;
        aParam = anAdaptor.FirstParameter();
      }
      else {
        done = 1;
        aParam = anAdaptor.LastParameter();
      }
    }

    if (aBasis->IsPeriodic()
        && Abs(aBasis->Period() - aParam) <= Precision::Parametric(Precision::Confusion())) {
      aParam = 0.;
    }
  }
  return aParam;
}

//=======================================================================
//function : SmartParameter
//purpose  : Extends the new curve of an edge to reach Pnt, which lies
//           past its first end (Sign -1) or last (Sign 1): the pcurve on S1
//           is extended to Pnt's parameters, projected onto S2, and the 3D
//           curve rebuilt from the two. Returns Pnt's parameter, the end.
//=======================================================================

static Standard_Real SmartParameter(Draft_EdgeInfo& EI,
                                    const Standard_Real EdgeTol,
                                    const gp_Pnt& Pnt,
                                    const Standard_Integer Sign,
                                    const Handle(Geom_Surface)& S1,
                                    const Handle(Geom_Surface)& S2)
{
  const Standard_Real aTol = Precision::Confusion();
  Standard_Real anEdgeTol = EdgeTol;

  if (EI.FirstPC().IsNull()) {
    const Handle(Geom_Curve)& aCurve = EI.Geometry();
    EI.ChangeFirstPC() =
      GeomProjLib::Curve2d(aCurve, aCurve->FirstParameter(), aCurve->LastParameter(), S1, anEdgeTol);
  }
  if (EI.SecondPC().IsNull()) {
    const Handle(Geom_Curve)& aCurve = EI.Geometry();
    EI.ChangeSecondPC() =
      GeomProjLib::Curve2d(aCurve, aCurve->FirstParameter(), aCurve->LastParameter(), S2, anEdgeTol);
  }

  Standard_Real aU, aV;
  GeomAPI_ProjectPointOnSurf(Pnt, S1).LowerDistanceParameters(aU, aV);
  const gp_Pnt2d aPnt2d(aU, aV);

  Handle(Geom2d_Curve) aPCurve = EI.FirstPC();
  Handle(Geom2d_TrimmedCurve) aTrimmed = Handle(Geom2d_TrimmedCurve)::DownCast(aPCurve);
  if (!aTrimmed.IsNull()) {
    aPCurve = aTrimmed->BasisCurve();
  }

  // Join the point on with a straight piece, if it is not on the pcurve yet.
  Geom2dAPI_ProjectPointOnCurve aProj2d(aPnt2d, aPCurve);
  if (aProj2d.NbPoints() == 0 || aProj2d.LowerDistance() > aTol) {
    Handle(Geom2d_BSplineCurve) aBSpline = Handle(Geom2d_BSplineCurve)::DownCast(aPCurve);
    if (aBSpline.IsNull()) {
      aBSpline = Geom2dConvert::CurveToBSplineCurve(aPCurve);
    }
    TColgp_Array1OfPnt2d aPoles(1, 2);
    aPoles(1) = Sign == -1 ? aPnt2d : aBSpline->Pole(aBSpline->NbPoles());
    aPoles(2) = Sign == -1 ? aBSpline->Pole(1) : aPnt2d;
    Geom2dConvert_CompCurveToBSplineCurve aJoined(aBSpline, Convert_QuasiAngular);
    aJoined.Add(new Geom2d_BezierCurve(aPoles), aTol, Sign != -1);
    aPCurve = aJoined.BSplineCurve();
  }
  EI.ChangeFirstPC() = aPCurve;

  Handle(Geom2dAdaptor_Curve) aPCurveAdaptor = new Geom2dAdaptor_Curve(aPCurve);
  Handle(GeomAdaptor_Surface) aSurface1 = new GeomAdaptor_Surface(S1);
  Handle(Adaptor3d_CurveOnSurface) anOnS1 = new Adaptor3d_CurveOnSurface(aPCurveAdaptor, aSurface1);
  Handle(GeomAdaptor_Surface) aSurface2 = new GeomAdaptor_Surface(S2);
  Handle(ProjLib_HCompProjectedCurve) aProjected =
    new ProjLib_HCompProjectedCurve(aSurface2, anOnS1, aTol, aTol);
  Standard_Real aStart, anEnd;
  aProjected->Bounds(1, aStart, anEnd);
  Approx_CurveOnSurface anApprox(aProjected, aSurface2, aStart, anEnd, aTol);
  anApprox.Perform(20 + aProjected->NbIntervals(GeomAbs_C3), 10, GeomAbs_C1, Standard_False, Standard_False);
  EI.ChangeSecondPC() = anApprox.Curve2d();
  EI.ChangeGeometry() = anApprox.Curve3d();
  EI.SetNewGeometry(Standard_True);

  return Sign == -1 ? EI.Geometry()->FirstParameter() : EI.Geometry()->LastParameter();
}
