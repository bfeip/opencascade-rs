// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.

#ifndef _Tweak_MoveFaces_HeaderFile
#define _Tweak_MoveFaces_HeaderFile

#include <BRepBuilderAPI_ModifyShape.hxx>
#include <BRepTools_History.hxx>
#include <Draft_ErrorStatus.hxx>
#include <gp_Trsf.hxx>
#include <NCollection_List.hxx>
#include <TopTools_ListOfShape.hxx>

class TopoDS_Face;
class TopoDS_Shape;

//! Moves faces of a shape, re-trimming the faces around them to meet them
//! (see Tweak_Modification). The topology of the shape is kept, except that
//! a sphere bordering a moved face, whose poles or seam lie near the edges
//! about to move, first has them turned away from its boundary: a moving
//! edge could otherwise have to cross them.
//!
//! Usage mirrors BRepOffsetAPI_DraftAngle: Add each face with its move,
//! check AddDone, then Build and check IsDone.
class Tweak_MoveFaces : public BRepBuilderAPI_ModifyShape
{
public:
  DEFINE_STANDARD_ALLOC

  Standard_EXPORT Tweak_MoveFaces(const TopoDS_Shape& S);

  //! Adds the face F, moved by T, with its tangent neighbours. Faces added
  //! with the same move move as one.
  Standard_EXPORT void Add(const TopoDS_Face& F, const gp_Trsf& T);

  //! False once a face could not be added; ProblematicShape tells which.
  Standard_EXPORT Standard_Boolean AddDone() const;

  //! The face, edge or vertex on which an error occurred.
  Standard_EXPORT const TopoDS_Shape& ProblematicShape() const;

  Standard_EXPORT Draft_ErrorStatus Status() const;

  //! All the faces that move, the added ones and their tangent neighbours.
  Standard_EXPORT const TopTools_ListOfShape& ModifiedFaces() const;

  Standard_EXPORT virtual void Build(
    const Message_ProgressRange& theRange = Message_ProgressRange()) Standard_OVERRIDE;

  Standard_EXPORT virtual const TopTools_ListOfShape& Modified(const TopoDS_Shape& S) Standard_OVERRIDE;

  Standard_EXPORT virtual const TopTools_ListOfShape& Generated(const TopoDS_Shape& S) Standard_OVERRIDE;

  Standard_EXPORT virtual Standard_Boolean IsDeleted(const TopoDS_Shape& S) Standard_OVERRIDE;

private:
  //! The added faces, and their moves.
  TopTools_ListOfShape myFaces;
  NCollection_List<gp_Trsf> myMoves;
  //! From the initial shape to the result, through the re-seated spheres.
  Handle(BRepTools_History) myHistory;
};

#endif // _Tweak_MoveFaces_HeaderFile
