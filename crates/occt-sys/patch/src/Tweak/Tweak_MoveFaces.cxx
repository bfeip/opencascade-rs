// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.

// Quoted: OCCT's build drops the headers of packages that only exist in the
// patch from its inc folder; the install still ships them.
#include "Tweak_MoveFaces.hxx"
#include "Tweak_Modification.hxx"

#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepLib.hxx>
#include <BRepLib_MakeEdge.hxx>
#include <BRepTools_ReShape.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <Geom_Circle.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <gp_Circ.hxx>
#include <Geom_SphericalSurface.hxx>
#include <gp_Ax3.hxx>
#include <NCollection_DataMap.hxx>
#include <NCollection_Vector.hxx>
#include <Precision.hxx>
#include <ShapeAnalysis_Edge.hxx>
#include <ShapeFix_Edge.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>

namespace
{
//! The history methods of a BRepTools_Modifier, for BRepTools_History.
class ModifierHistory
{
public:
  ModifierHistory(BRepTools_Modifier& theModifier)
      : myModifier(theModifier)
  {
  }

  Standard_Boolean IsDeleted(const TopoDS_Shape&) { return Standard_False; }

  const TopTools_ListOfShape& Modified(const TopoDS_Shape& theShape)
  {
    myList.Clear();
    myList.Append(myModifier.ModifiedShape(theShape));
    return myList;
  }

  const TopTools_ListOfShape& Generated(const TopoDS_Shape&)
  {
    myList.Clear();
    return myList;
  }

private:
  BRepTools_Modifier& myModifier;
  TopTools_ListOfShape myList;
};
} // namespace

//! Angle between samples of a boundary, much finer than MIN_CLEARANCE so
//! that a seam cannot slip between two of them unseen.
static const Standard_Real SAMPLE_SPACING = M_PI / 180.;

//! The least angle from a re-seated sphere's poles and seam to its boundary.
static const Standard_Real MIN_CLEARANCE = 5. * M_PI / 180.;

//! How far past the measured deviation of a projected pcurve the tolerance
//! of its edge reaches.
static const Standard_Real TOLERANCE_MARGIN = 1.5;

static Handle(Geom_SphericalSurface) Sphere(const TopoDS_Face& F);

static void MovingSamples(const TopoDS_Face& F,
                          const gp_Sphere& S,
                          const TopTools_IndexedMapOfShape& MovedVertices,
                          const NCollection_List<gp_Trsf>& Moves,
                          NCollection_Vector<gp_Dir>& Directions);

static Standard_Boolean IsCrowded(const TopoDS_Face& F,
                                  const TopTools_IndexedMapOfShape& MovedVertices,
                                  const NCollection_List<gp_Trsf>& Moves);

static void Reseat(const TopoDS_Face& F,
                   const TopTools_IndexedDataMapOfShapeListOfShape& EdgeFaces,
                   const NCollection_Vector<gp_Dir>& Moving,
                   BRepTools_ReShape& ReShape);

//=======================================================================
//function : Tweak_MoveFaces
//purpose  :
//=======================================================================

Tweak_MoveFaces::Tweak_MoveFaces(const TopoDS_Shape& S)
{
  myInitialShape = S;
  myModification = new Tweak_Modification(S);
}

//=======================================================================
//function : Add
//purpose  :
//=======================================================================

void Tweak_MoveFaces::Add(const TopoDS_Face& F, const gp_Trsf& T)
{
  myFaces.Append(F);
  myMoves.Append(T);
  Handle(Tweak_Modification)::DownCast(myModification)->Add(F, T);
}

//=======================================================================
//function : AddDone
//purpose  :
//=======================================================================

Standard_Boolean Tweak_MoveFaces::AddDone() const
{
  return Handle(Tweak_Modification)::DownCast(myModification)->ProblematicShape().IsNull();
}

//=======================================================================
//function : ProblematicShape
//purpose  :
//=======================================================================

const TopoDS_Shape& Tweak_MoveFaces::ProblematicShape() const
{
  return Handle(Tweak_Modification)::DownCast(myModification)->ProblematicShape();
}

//=======================================================================
//function : Status
//purpose  :
//=======================================================================

Draft_ErrorStatus Tweak_MoveFaces::Status() const
{
  return Handle(Tweak_Modification)::DownCast(myModification)->Error();
}

//=======================================================================
//function : ModifiedFaces
//purpose  :
//=======================================================================

const TopTools_ListOfShape& Tweak_MoveFaces::ModifiedFaces() const
{
  return Handle(Tweak_Modification)::DownCast(myModification)->ModifiedFaces();
}

//=======================================================================
//function : Build
//purpose  :
//=======================================================================

void Tweak_MoveFaces::Build(const Message_ProgressRange& /*theRange*/)
{
  Handle(Tweak_Modification) aModification = Handle(Tweak_Modification)::DownCast(myModification);
  const TopoDS_Shape anInitial = myInitialShape;
  TopTools_ListOfShape anInitials;
  anInitials.Append(anInitial);
  myHistory = new BRepTools_History();

  // The spheres bordering the moved faces whose poles or seam crowd the
  // edges about to move.
  TopTools_IndexedDataMapOfShapeListOfShape anEdgeFaces;
  TopExp::MapShapesAndAncestors(anInitial, TopAbs_EDGE, TopAbs_FACE, anEdgeFaces);
  TopTools_IndexedMapOfShape aMoved, aMovedVertices, aCrowded;
  for (TopTools_ListIteratorOfListOfShape aFaces(myFaces); aFaces.More(); aFaces.Next()) {
    aMoved.Add(aFaces.Value());
    TopExp::MapShapes(aFaces.Value(), TopAbs_VERTEX, aMovedVertices);
  }
  for (Standard_Integer i = 1; i <= aMoved.Extent(); i++) {
    for (TopExp_Explorer anEdges(aMoved(i), TopAbs_EDGE); anEdges.More(); anEdges.Next()) {
      const TopTools_ListOfShape* aNeighbours = anEdgeFaces.Seek(anEdges.Current());
      if (aNeighbours == NULL) {
        continue;
      }
      for (TopTools_ListIteratorOfListOfShape it(*aNeighbours); it.More(); it.Next()) {
        if (!aMoved.Contains(it.Value()) && IsCrowded(TopoDS::Face(it.Value()), aMovedVertices, myMoves)) {
          aCrowded.Add(it.Value());
        }
      }
    }
  }

  // Re-seated on a copy: re-seating adds pcurves to edges the input shares.
  TopoDS_Shape aShape = anInitial;
  if (!aCrowded.IsEmpty()) {
    BRepBuilderAPI_Copy aCopy(anInitial);
    BRepTools_ReShape aReShape;
    TopTools_IndexedDataMapOfShapeListOfShape aCopyEdgeFaces;
    TopExp::MapShapesAndAncestors(aCopy.Shape(), TopAbs_EDGE, TopAbs_FACE, aCopyEdgeFaces);
    for (Standard_Integer i = 1; i <= aCrowded.Extent(); i++) {
      const TopoDS_Face aFace = TopoDS::Face(aCopy.ModifiedShape(aCrowded(i)).Oriented(TopAbs_FORWARD));
      // Where the moving stretch of its boundary goes counts as boundary too.
      NCollection_Vector<gp_Dir> aMoving;
      MovingSamples(TopoDS::Face(aCrowded(i)), Sphere(aFace)->Sphere(), aMovedVertices, myMoves, aMoving);
      Reseat(aFace, aCopyEdgeFaces, aMoving, aReShape);
    }
    aShape = aReShape.Apply(aCopy.Shape());
    myHistory->Merge(anInitials, aCopy);
    myHistory->Merge(aReShape.History());

    aModification->Init(aShape);
    NCollection_List<gp_Trsf>::Iterator aMoves(myMoves);
    for (TopTools_ListIteratorOfListOfShape aFaces(myFaces); aFaces.More(); aFaces.Next(), aMoves.Next()) {
      const TopTools_ListOfShape& anImages = myHistory->Modified(aFaces.Value());
      const TopoDS_Shape aFace = anImages.IsEmpty() ? aFaces.Value() : anImages.First();
      if (!aModification->Add(TopoDS::Face(aFace), aMoves.Value())) {
        NotDone();
        return;
      }
    }
  }

  aModification->Perform();
  if (!aModification->IsDone()) {
    NotDone();
    return;
  }
  DoModif(aShape);
  if (IsDone()) {
    TopTools_ListOfShape aShapes;
    aShapes.Append(aShape);
    ModifierHistory aModifierHistory(myModifier);
    myHistory->Merge(aShapes, aModifierHistory);
  }
}

//=======================================================================
//function : Modified
//purpose  :
//=======================================================================

const TopTools_ListOfShape& Tweak_MoveFaces::Modified(const TopoDS_Shape& S)
{
  return myHistory.IsNull() ? BRepBuilderAPI_ModifyShape::Modified(S) : myHistory->Modified(S);
}

//=======================================================================
//function : Generated
//purpose  :
//=======================================================================

const TopTools_ListOfShape& Tweak_MoveFaces::Generated(const TopoDS_Shape& S)
{
  return myHistory.IsNull() ? BRepBuilderAPI_ModifyShape::Generated(S) : myHistory->Generated(S);
}

//=======================================================================
//function : IsDeleted
//purpose  :
//=======================================================================

Standard_Boolean Tweak_MoveFaces::IsDeleted(const TopoDS_Shape& S)
{
  return myHistory.IsNull() ? BRepBuilderAPI_ModifyShape::IsDeleted(S) : myHistory->IsRemoved(S);
}

//=======================================================================
//function : Sphere
//purpose  : The sphere F lies on, if it does.
//=======================================================================

static Handle(Geom_SphericalSurface) Sphere(const TopoDS_Face& F)
{
  Handle(Geom_Surface) aSurface = BRep_Tool::Surface(F);
  Handle(Geom_RectangularTrimmedSurface) aTrimmed = Handle(Geom_RectangularTrimmedSurface)::DownCast(aSurface);
  if (!aTrimmed.IsNull()) {
    aSurface = aTrimmed->BasisSurface();
  }
  return Handle(Geom_SphericalSurface)::DownCast(aSurface);
}

//=======================================================================
//function : Clearance
//purpose  : The angle from direction Q, out of a sphere's centre, to the
//           nearest of the sphere's poles and seam, its poles lying along
//           Axis and its seam the half meridian through X.
//=======================================================================

static Standard_Real Clearance(const gp_Dir& Q, const gp_Dir& Axis, const gp_Dir& X)
{
  const Standard_Real aPoles = ACos(Min(1., Abs(Q.Dot(Axis))));
  if (Q.Dot(X) < 0.) {
    return aPoles;
  }
  const Standard_Real aSeam = ASin(Min(1., Abs(Q.Dot(Axis.Crossed(X)))));
  return Min(aPoles, aSeam);
}

//=======================================================================
//function : Samples
//purpose  : Directions out of the centre of sphere S to points along E,
//           close enough together that a seam cannot pass between two of
//           them unseen, and to where each of Moves takes them.
//=======================================================================

static void Samples(const TopoDS_Edge& E,
                    const gp_Sphere& S,
                    const NCollection_List<gp_Trsf>& Moves,
                    NCollection_Vector<gp_Dir>& Directions)
{
  BRepAdaptor_Curve aCurve(E);
  const Standard_Real anArc = GCPnts_AbscissaPoint::Length(aCurve) / S.Radius();
  const Standard_Integer aNbSamples = Max(16, Standard_Integer(anArc / SAMPLE_SPACING) + 1);
  for (Standard_Integer i = 0; i <= aNbSamples; i++) {
    const Standard_Real aParam =
      aCurve.FirstParameter() + (aCurve.LastParameter() - aCurve.FirstParameter()) * i / aNbSamples;
    const gp_Pnt aPnt = aCurve.Value(aParam);
    const gp_Vec anOut(S.Location(), aPnt);
    if (anOut.Magnitude() > Precision::Confusion()) {
      Directions.Append(gp_Dir(anOut));
    }
    for (NCollection_List<gp_Trsf>::Iterator aMove(Moves); aMove.More(); aMove.Next()) {
      const gp_Vec aMovedOut(S.Location(), aPnt.Transformed(aMove.Value()));
      if (aMovedOut.Magnitude() > Precision::Confusion()) {
        Directions.Append(gp_Dir(aMovedOut));
      }
    }
  }
}

//=======================================================================
//function : MovingSamples
//purpose  : Samples (with Moves) of the edges of F, on sphere S, that end
//           at one of MovedVertices: the stretch of F's boundary that moves.
//=======================================================================

static void MovingSamples(const TopoDS_Face& F,
                          const gp_Sphere& S,
                          const TopTools_IndexedMapOfShape& MovedVertices,
                          const NCollection_List<gp_Trsf>& Moves,
                          NCollection_Vector<gp_Dir>& Directions)
{
  for (TopExp_Explorer anEdges(F, TopAbs_EDGE); anEdges.More(); anEdges.Next()) {
    const TopoDS_Edge& anEdge = TopoDS::Edge(anEdges.Current());
    TopoDS_Vertex aFirst, aLast;
    TopExp::Vertices(anEdge, aFirst, aLast);
    if (!BRep_Tool::Degenerated(anEdge) && !BRep_Tool::IsClosed(anEdge, F)
        && (MovedVertices.Contains(aFirst) || MovedVertices.Contains(aLast))) {
      Samples(anEdge, S, Moves, Directions);
    }
  }
}

//=======================================================================
//function : IsCrowded
//purpose  : Whether F lies on a sphere whose poles or seam come within
//           MIN_CLEARANCE of the edges of F ending at one of MovedVertices,
//           where they are or where Moves take them: as they move, they
//           could have to cross them.
//=======================================================================

static Standard_Boolean IsCrowded(const TopoDS_Face& F,
                                  const TopTools_IndexedMapOfShape& MovedVertices,
                                  const NCollection_List<gp_Trsf>& Moves)
{
  const Handle(Geom_SphericalSurface) aSphere = Sphere(F);
  if (aSphere.IsNull()) {
    return Standard_False;
  }
  const gp_Ax3& aFrame = aSphere->Position();
  NCollection_Vector<gp_Dir> aSamples;
  MovingSamples(F, aSphere->Sphere(), MovedVertices, Moves, aSamples);
  for (Standard_Integer i = 0; i < aSamples.Length(); i++) {
    if (Clearance(aSamples(i), aFrame.Direction(), aFrame.XDirection()) < MIN_CLEARANCE) {
      return Standard_True;
    }
  }
  return Standard_False;
}

//=======================================================================
//function : AddPCurve
//purpose  : Projects edge E onto Face for a new pcurve, re-parametrised to
//           follow E's curve, E's tolerance grown to cover what is left.
//           That is measured densely: a move may cut E down to the stretch
//           where its pcurve strays most.
//=======================================================================

static void AddPCurve(const TopoDS_Edge& E, const TopoDS_Face& Face)
{
  BRep_Builder aBuilder;
  ShapeFix_Edge aFixEdge;
  aFixEdge.FixAddPCurve(E, Face, Standard_False);
  aBuilder.SameParameter(E, Standard_False);
  BRepLib::SameParameter(E, BRep_Tool::Tolerance(E));
  Standard_Real aDeviation = 0.;
  ShapeAnalysis_Edge().CheckSameParameter(E, Face, aDeviation, 101);
  if (TOLERANCE_MARGIN * aDeviation > BRep_Tool::Tolerance(E)) {
    aBuilder.UpdateEdge(E, TOLERANCE_MARGIN * aDeviation);
  }
  aFixEdge.FixVertexTolerance(E, Face);
}

//=======================================================================
//function : OtherFace
//purpose  : The face along E besides F, if there is one.
//=======================================================================

static TopoDS_Face OtherFace(const TopoDS_Edge& E,
                             const TopoDS_Face& F,
                             const TopTools_IndexedDataMapOfShapeListOfShape& EdgeFaces)
{
  const TopTools_ListOfShape* aFaces = EdgeFaces.Seek(E);
  if (aFaces != NULL) {
    for (TopTools_ListIteratorOfListOfShape it(*aFaces); it.More(); it.Next()) {
      if (!it.Value().IsSame(F)) {
        return TopoDS::Face(it.Value());
      }
    }
  }
  return TopoDS_Face();
}

//=======================================================================
//function : Circle
//purpose  : The circle E runs along, if it does.
//=======================================================================

static Handle(Geom_Circle) Circle(const TopoDS_Edge& E)
{
  Standard_Real aFirst, aLast;
  Handle(Geom_Curve) aCurve = BRep_Tool::Curve(E, aFirst, aLast);
  Handle(Geom_TrimmedCurve) aTrimmed = Handle(Geom_TrimmedCurve)::DownCast(aCurve);
  if (!aTrimmed.IsNull()) {
    aCurve = aTrimmed->BasisCurve();
  }
  return Handle(Geom_Circle)::DownCast(aCurve);
}

//=======================================================================
//function : Fused
//purpose  : The edge running along both A and B, which follow each other
//           in that order along one circle, with the same orientation; it
//           takes that orientation. Null if they cannot be fused.
//=======================================================================

static TopoDS_Edge Fused(const TopoDS_Edge& A, const TopoDS_Edge& B)
{
  const Handle(Geom_Circle) aCircleA = Circle(A), aCircleB = Circle(B);
  if (aCircleA.IsNull() || aCircleB.IsNull() || A.Orientation() != B.Orientation()) {
    return TopoDS_Edge();
  }
  const gp_Circ aCircA = aCircleA->Circ(), aCircB = aCircleB->Circ();
  if (Abs(aCircA.Radius() - aCircB.Radius()) > Precision::Confusion()
      || aCircA.Location().Distance(aCircB.Location()) > Precision::Confusion()
      || !aCircA.Axis().Direction().IsEqual(aCircB.Axis().Direction(), Precision::Angular())) {
    return TopoDS_Edge();
  }
  // Along the circle, A comes first unless both run against it.
  const Standard_Boolean isForward = A.Orientation() == TopAbs_FORWARD;
  const TopoDS_Edge& aFirst = isForward ? A : B;
  const TopoDS_Edge& aLast = isForward ? B : A;
  BRepLib_MakeEdge aMaker(aCircleA, TopExp::FirstVertex(aFirst), TopExp::LastVertex(aLast));
  if (!aMaker.IsDone()) {
    return TopoDS_Edge();
  }
  return TopoDS::Edge(aMaker.Edge().Oriented(A.Orientation()));
}

//=======================================================================
//function : Reseat
//purpose  : Remakes face F, FORWARD on a sphere, on the same sphere with
//           its poles and seam as far as they can be from its boundary and
//           from Moving, directions to where the moving stretch of it goes:
//           both then lie inside F or both outside it, never on its
//           boundary. Records it, and the fusing of boundary pieces that the
//           old seam split, in ReShape. Leaves F as it is if they cannot
//           keep MIN_CLEARANCE from all that, F has no boundary besides seam
//           and poles, or the remade face would not be valid.
//=======================================================================

static void Reseat(const TopoDS_Face& F,
                   const TopTools_IndexedDataMapOfShapeListOfShape& EdgeFaces,
                   const NCollection_Vector<gp_Dir>& Moving,
                   BRepTools_ReShape& ReShape)
{
  const Handle(Geom_SphericalSurface) aSphere = Sphere(F);
  const gp_Sphere aGeom = aSphere->Sphere();
  const gp_Pnt aCentre = aGeom.Location();

  // The boundary, seams and poles aside, and directions out to points on it.
  TopTools_ListOfShape aBoundary;
  TopTools_IndexedMapOfShape aSeamVertices;
  NCollection_Vector<gp_Dir> aSamples;
  for (TopExp_Explorer aWires(F, TopAbs_WIRE); aWires.More(); aWires.Next()) {
    for (BRepTools_WireExplorer anEdges(TopoDS::Wire(aWires.Current()), F); anEdges.More(); anEdges.Next()) {
      const TopoDS_Edge& anEdge = anEdges.Current();
      if (BRep_Tool::Degenerated(anEdge)) {
        continue;
      }
      if (BRep_Tool::IsClosed(anEdge, F)) {
        TopExp::MapShapes(anEdge, TopAbs_VERTEX, aSeamVertices);
        continue;
      }
      aBoundary.Append(anEdge);
      Samples(anEdge, aGeom, NCollection_List<gp_Trsf>(), aSamples);
    }
  }
  if (aBoundary.IsEmpty()) {
    return;
  }
  for (Standard_Integer i = 0; i < Moving.Length(); i++) {
    aSamples.Append(Moving(i));
  }

  // Of directions spread evenly over the sphere, the axis and seam keeping
  // farthest from the boundary.
  const Standard_Integer aNbAxes = 256, aNbSeams = 24;
  const Standard_Real aGoldenAngle = M_PI * (3. - Sqrt(5.));
  Standard_Real aBestClearance = -1.;
  gp_Dir aBestAxis, aBestX;
  for (Standard_Integer i = 0; i < aNbAxes; i++) {
    const Standard_Real aZ = 1. - 2. * (i + 0.5) / aNbAxes;
    const Standard_Real aR = Sqrt(1. - aZ * aZ);
    const gp_Dir anAxis(aR * Cos(i * aGoldenAngle), aR * Sin(i * aGoldenAngle), aZ);
    const gp_Ax2 aPlane(aCentre, anAxis);
    for (Standard_Integer j = 0; j < aNbSeams; j++) {
      const Standard_Real anAngle = 2. * M_PI * j / aNbSeams;
      const gp_Dir aX(gp_Vec(aPlane.XDirection()) * Cos(anAngle) + gp_Vec(aPlane.YDirection()) * Sin(anAngle));
      Standard_Real aClearance = RealLast();
      for (Standard_Integer k = 0; k < aSamples.Length() && aClearance > aBestClearance; k++) {
        aClearance = Min(aClearance, Clearance(aSamples(k), anAxis, aX));
      }
      if (aClearance > aBestClearance) {
        aBestClearance = aClearance;
        aBestAxis = anAxis;
        aBestX = aX;
      }
    }
  }
  // Too close a fit (a rim on a great circle cannot keep a seam joining
  // opposite poles off it) leaves the sphere as it is.
  if (aBestClearance < MIN_CLEARANCE) {
    return;
  }

  // The boundary's loops, chained end to start in their own orientation,
  // which keeps the face on the same side of them.
  NCollection_List<TopTools_ListOfShape> aLoops;
  while (!aBoundary.IsEmpty()) {
    TopTools_ListOfShape aLoop;
    TopoDS_Edge anEdge = TopoDS::Edge(aBoundary.First());
    aBoundary.RemoveFirst();
    const TopoDS_Vertex aStart = TopExp::FirstVertex(anEdge, Standard_True);
    for (;;) {
      aLoop.Append(anEdge);
      const TopoDS_Vertex anEnd = TopExp::LastVertex(anEdge, Standard_True);
      if (anEnd.IsSame(aStart)) {
        break;
      }
      TopTools_ListIteratorOfListOfShape aNext(aBoundary);
      while (aNext.More() && !TopExp::FirstVertex(TopoDS::Edge(aNext.Value()), Standard_True).IsSame(anEnd)) {
        aNext.Next();
      }
      if (!aNext.More()) {
        return;
      }
      anEdge = TopoDS::Edge(aNext.Value());
      aBoundary.Remove(aNext);
    }
    aLoops.Append(aLoop);
  }

  // Where the old seam split a loop, the pieces either side are fused when
  // they run on along one circle by one other face: a vertex moving along
  // the boundary may have to pass where the seam met it.
  NCollection_DataMap<TopoDS_Shape, TopTools_ListOfShape, TopTools_ShapeMapHasher> aPieces;
  for (NCollection_List<TopTools_ListOfShape>::Iterator aLoopIt(aLoops); aLoopIt.More(); aLoopIt.Next()) {
    TopTools_ListOfShape& aLoop = aLoopIt.ChangeValue();
    for (Standard_Boolean isFused = Standard_True; isFused && aLoop.Extent() > 1;) {
      isFused = Standard_False;
      for (TopTools_ListIteratorOfListOfShape it(aLoop); it.More(); it.Next()) {
        TopTools_ListIteratorOfListOfShape aNextIt = it;
        aNextIt.Next();
        const TopoDS_Edge anA = TopoDS::Edge(it.Value());
        const TopoDS_Edge aB = TopoDS::Edge(aNextIt.More() ? aNextIt.Value() : aLoop.First());
        if (anA.IsSame(aB) || !aSeamVertices.Contains(TopExp::LastVertex(anA, Standard_True))) {
          continue;
        }
        const TopTools_ListOfShape* aPiecesA = aPieces.Seek(anA);
        const TopTools_ListOfShape* aPiecesB = aPieces.Seek(aB);
        const TopoDS_Edge anOriginalA = TopoDS::Edge(aPiecesA != NULL ? aPiecesA->First() : anA);
        const TopoDS_Edge anOriginalB = TopoDS::Edge(aPiecesB != NULL ? aPiecesB->First() : aB);
        const TopoDS_Face anOther = OtherFace(anOriginalA, F, EdgeFaces);
        if (anOther.IsNull() || !anOther.IsSame(OtherFace(anOriginalB, F, EdgeFaces))) {
          continue;
        }
        const TopoDS_Edge aFused = Fused(anA, aB);
        if (aFused.IsNull()) {
          continue;
        }
        TopTools_ListOfShape aFusedPieces = aPiecesA != NULL ? *aPiecesA : TopTools_ListOfShape();
        if (aPiecesA == NULL) {
          aFusedPieces.Append(anA);
        }
        if (aPiecesB == NULL) {
          aFusedPieces.Append(aB);
        }
        else {
          for (TopTools_ListIteratorOfListOfShape aPiece(*aPiecesB); aPiece.More(); aPiece.Next()) {
            aFusedPieces.Append(aPiece.Value());
          }
        }
        aPieces.UnBind(anA);
        aPieces.UnBind(aB);
        aPieces.Bind(aFused, aFusedPieces);
        TopTools_ListOfShape aRebuilt;
        for (TopTools_ListIteratorOfListOfShape anEdge(aLoop); anEdge.More(); anEdge.Next()) {
          if (anEdge.Value().IsSame(anA)) {
            aRebuilt.Append(aFused);
          }
          else if (!anEdge.Value().IsSame(aB)) {
            aRebuilt.Append(anEdge.Value());
          }
        }
        aLoop = aRebuilt;
        isFused = Standard_True;
        break;
      }
    }
  }

  // The seam joins the poles without crossing the boundary, so if one pole
  // lies inside F the whole natural bound of the sphere does.
  gp_Ax3 aFrame(aCentre, aBestAxis, aBestX);
  if (!aGeom.Position().Direct()) {
    aFrame.YReverse();
  }
  Handle(Geom_SphericalSurface) aReseated = new Geom_SphericalSurface(aFrame, aGeom.Radius());
  BRep_Builder aBuilder;
  TopoDS_Face aFace;
  const gp_Pnt aPole = aCentre.Translated(gp_Vec(aBestAxis) * aGeom.Radius());
  if (BRepClass_FaceClassifier(F, aPole, Precision::Confusion()).State() == TopAbs_IN) {
    aFace = BRepBuilderAPI_MakeFace(aReseated, Precision::Confusion()).Face();
  }
  else {
    aBuilder.MakeFace(aFace, aReseated, BRep_Tool::Tolerance(F));
  }
  for (NCollection_List<TopTools_ListOfShape>::Iterator aLoopIt(aLoops); aLoopIt.More(); aLoopIt.Next()) {
    TopoDS_Wire aWire;
    aBuilder.MakeWire(aWire);
    for (TopTools_ListIteratorOfListOfShape it(aLoopIt.Value()); it.More(); it.Next()) {
      const TopoDS_Edge& anEdge = TopoDS::Edge(it.Value());
      AddPCurve(anEdge, aFace);
      const TopTools_ListOfShape* aFusedPieces = aPieces.Seek(anEdge);
      if (aFusedPieces != NULL) {
        AddPCurve(anEdge, OtherFace(TopoDS::Edge(aFusedPieces->First()), F, EdgeFaces));
      }
      aBuilder.Add(aWire, anEdge);
    }
    aWire.Closed(Standard_True);
    aBuilder.Add(aFace, aWire);
  }
  if (!BRepCheck_Analyzer(aFace).IsValid()) {
    return;
  }

  // Each fused edge takes the place of its first piece, running its way;
  // the other pieces go.
  ReShape.Replace(F, aFace);
  for (NCollection_DataMap<TopoDS_Shape, TopTools_ListOfShape, TopTools_ShapeMapHasher>::Iterator it(aPieces);
       it.More(); it.Next()) {
    TopTools_ListIteratorOfListOfShape aPiece(it.Value());
    ReShape.Replace(aPiece.Value().Oriented(it.Key().Orientation()), it.Key());
    for (aPiece.Next(); aPiece.More(); aPiece.Next()) {
      ReShape.Remove(aPiece.Value());
    }
  }
}
