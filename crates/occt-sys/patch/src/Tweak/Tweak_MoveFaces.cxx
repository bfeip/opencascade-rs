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

#include <gp_Trsf.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

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
  aModification->Perform();
  if (!aModification->IsDone()) {
    NotDone();
  }
  else {
    DoModif(myInitialShape);
  }
}
