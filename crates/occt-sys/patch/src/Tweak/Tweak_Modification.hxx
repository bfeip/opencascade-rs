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

#ifndef _Tweak_Modification_HeaderFile
#define _Tweak_Modification_HeaderFile

#include <BRepTools_Modification.hxx>
#include <Draft_ErrorStatus.hxx>
#include <Draft_IndexedDataMapOfEdgeEdgeInfo.hxx>
#include <Draft_IndexedDataMapOfFaceFaceInfo.hxx>
#include <Draft_IndexedDataMapOfVertexVertexInfo.hxx>
#include <GeomAbs_Shape.hxx>
#include <gp_Trsf.hxx>
#include <NCollection_DataMap.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_ShapeMapHasher.hxx>

class Tweak_Modification;
DEFINE_STANDARD_HANDLE(Tweak_Modification, BRepTools_Modification)

//! Moves faces of a shape and re-solves the edges and vertices around them,
//! keeping the topology: each face bordering a moved one keeps its surface
//! and is re-trimmed where it meets the moved surface. Faces tangent to a
//! moved face move with it.
//!
//! A move that would change the topology (a face shrinking away, a moved
//! face reaching past a neighbour) is not detected here; the modified shape
//! must be checked.
class Tweak_Modification : public BRepTools_Modification
{
public:
  Standard_EXPORT Tweak_Modification(const TopoDS_Shape& S);

  //! Resets on the same shape.
  Standard_EXPORT void Clear();

  //! Changes the basis shape and resets.
  Standard_EXPORT void Init(const TopoDS_Shape& S);

  //! Adds the face F, moved by T, and propagates the move to its tangent
  //! neighbours. T must be a similarity that keeps orientation. Returns
  //! False, with ProblematicShape set, when F cannot be moved.
  Standard_EXPORT Standard_Boolean Add(const TopoDS_Face& F, const gp_Trsf& T);

  //! Re-solves the edges and vertices around the moved faces.
  Standard_EXPORT void Perform();

  Standard_EXPORT Standard_Boolean IsDone() const;

  Standard_EXPORT Draft_ErrorStatus Error() const;

  //! The face, edge or vertex on which an error occurred.
  Standard_EXPORT const TopoDS_Shape& ProblematicShape() const;

  //! All the faces that move, the added ones and their tangent neighbours.
  Standard_EXPORT const TopTools_ListOfShape& ModifiedFaces();

  Standard_EXPORT Standard_Boolean NewSurface(const TopoDS_Face& F,
                                              Handle(Geom_Surface)& S,
                                              TopLoc_Location& L,
                                              Standard_Real& Tol,
                                              Standard_Boolean& RevWires,
                                              Standard_Boolean& RevFace) Standard_OVERRIDE;

  Standard_EXPORT Standard_Boolean NewCurve(const TopoDS_Edge& E,
                                            Handle(Geom_Curve)& C,
                                            TopLoc_Location& L,
                                            Standard_Real& Tol) Standard_OVERRIDE;

  Standard_EXPORT Standard_Boolean NewPoint(const TopoDS_Vertex& V,
                                            gp_Pnt& P,
                                            Standard_Real& Tol) Standard_OVERRIDE;

  Standard_EXPORT Standard_Boolean NewCurve2d(const TopoDS_Edge& E,
                                              const TopoDS_Face& F,
                                              const TopoDS_Edge& NewE,
                                              const TopoDS_Face& NewF,
                                              Handle(Geom2d_Curve)& C,
                                              Standard_Real& Tol) Standard_OVERRIDE;

  Standard_EXPORT Standard_Boolean NewParameter(const TopoDS_Vertex& V,
                                                const TopoDS_Edge& E,
                                                Standard_Real& P,
                                                Standard_Real& Tol) Standard_OVERRIDE;

  Standard_EXPORT GeomAbs_Shape Continuity(const TopoDS_Edge& E,
                                           const TopoDS_Face& F1,
                                           const TopoDS_Face& F2,
                                           const TopoDS_Edge& NewE,
                                           const TopoDS_Face& NewF1,
                                           const TopoDS_Face& NewF2) Standard_OVERRIDE;

  DEFINE_STANDARD_RTTIEXT(Tweak_Modification, BRepTools_Modification)

private:
  Standard_Boolean InternalAdd(const TopoDS_Face& F, const gp_Trsf& T);

  Standard_Boolean Propagate();

  Standard_Boolean ComputeEdges();

  Handle(Geom_Curve) IntersectionCurve(const TopoDS_Edge& E,
                                       const Handle(Geom_Curve)& OldC,
                                       Draft_EdgeInfo& EI);

  Standard_Boolean ComputeVertices();

  Standard_Boolean AlignVertex(const TopoDS_Vertex& V, Draft_VertexInfo& VI);

  void OrderVertexParameters();

  //! The transform moving every face around E alike, if there is one: E
  //! then moves rigidly with them.
  const gp_Trsf* RigidMove(const TopoDS_Edge& E) const;

  //! The transform moving every face around V alike, if there is one.
  const gp_Trsf* RigidMove(const TopoDS_Vertex& V);

  //! The transform moving F, if F moves.
  const gp_Trsf* Move(const TopoDS_Face& F) const;

  Draft_IndexedDataMapOfFaceFaceInfo myFMap;
  Draft_IndexedDataMapOfEdgeEdgeInfo myEMap;
  Draft_IndexedDataMapOfVertexVertexInfo myVMap;
  //! Per added face (the root of the faces moving with it), its transform.
  NCollection_DataMap<TopoDS_Shape, gp_Trsf, TopTools_ShapeMapHasher> myMoves;
  Standard_Boolean myComp;
  TopoDS_Shape myShape;
  TopoDS_Shape badShape;
  Draft_ErrorStatus errStat;
  TopoDS_Face curFace;
  TopTools_ListOfShape conneF;
  TopTools_IndexedDataMapOfShapeListOfShape myEFMap;
};

#endif // _Tweak_Modification_HeaderFile
