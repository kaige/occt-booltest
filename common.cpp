#include "common.hpp"

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax2d.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Dir2d.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GC_MakeSegment.hxx>
#include <GC_MakeSegment2d.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Compound.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepLib.hxx>
#include <STEPControl_Writer.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_StepModelType.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <NCollection_List.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <cmath>
#include <cstdio>

// ============================================================
// MakeBottle — verbatim from the OCCT tutorial (dox/tutorial)
// ============================================================
TopoDS_Shape MakeBottle(double theWidth, double theHeight,
                        double theThickness)
{
    gp_Pnt aPnt1(-theWidth / 2., 0, 0);
    gp_Pnt aPnt2(-theWidth / 2., -theThickness / 4., 0);
    gp_Pnt aPnt3(0, -theThickness / 2., 0);
    gp_Pnt aPnt4(theWidth / 2., -theThickness / 4., 0);
    gp_Pnt aPnt5(theWidth / 2., 0, 0);

    occ::handle<Geom_TrimmedCurve> anArcOfCircle = GC_MakeArcOfCircle(aPnt2,aPnt3,aPnt4);
    occ::handle<Geom_TrimmedCurve> aSegment1 = GC_MakeSegment(aPnt1, aPnt2);
    occ::handle<Geom_TrimmedCurve> aSegment2 = GC_MakeSegment(aPnt4, aPnt5);

    TopoDS_Edge anEdge1 = BRepBuilderAPI_MakeEdge(aSegment1);
    TopoDS_Edge anEdge2 = BRepBuilderAPI_MakeEdge(anArcOfCircle);
    TopoDS_Edge anEdge3 = BRepBuilderAPI_MakeEdge(aSegment2);
    TopoDS_Wire aWire  = BRepBuilderAPI_MakeWire(anEdge1, anEdge2, anEdge3);

    gp_Ax1 xAxis = gp::OX();
    gp_Trsf aTrsf;
    aTrsf.SetMirror(xAxis);
    BRepBuilderAPI_Transform aBRepTrsf(aWire, aTrsf);
    TopoDS_Shape aMirroredShape = aBRepTrsf.Shape();
    TopoDS_Wire aMirroredWire = TopoDS::Wire(aMirroredShape);

    BRepBuilderAPI_MakeWire aMkWire;
    aMkWire.Add(aWire);
    aMkWire.Add(aMirroredWire);
    TopoDS_Wire aWireProfile = aMkWire.Wire();

    TopoDS_Face aFaceProfile = BRepBuilderAPI_MakeFace(aWireProfile);
    gp_Vec aPrismVec(0, 0, theHeight);
    TopoDS_Shape aBody = BRepPrimAPI_MakePrism(aFaceProfile, aPrismVec);

    BRepFilletAPI_MakeFillet aMkFillet(aBody);
    TopExp_Explorer anEdgeExplorer(aBody, TopAbs_EDGE);
    while(anEdgeExplorer.More()){
        TopoDS_Edge anEdge = TopoDS::Edge(anEdgeExplorer.Current());
        aMkFillet.Add(theThickness / 12., anEdge);
        anEdgeExplorer.Next();
    }
    aBody = aMkFillet.Shape();

    gp_Pnt aNeckLocation(0, 0, theHeight);
    gp_Dir aNeckAxis = gp::DZ();
    gp_Ax2 neckAx2(aNeckLocation, aNeckAxis);

    double aNeckRadius = theThickness / 4.;
    double aNeckHeight = theHeight / 10.;

    BRepPrimAPI_MakeCylinder aMkCylinder(neckAx2, aNeckRadius, aNeckHeight);
    TopoDS_Shape aNeck = aMkCylinder.Shape();

    BRepAlgoAPI_Fuse aFuser(aBody, aNeck);
    if (aFuser.IsDone())
        aBody = aFuser.Shape();

    TopoDS_Face   aFaceToRemove;
    double aZMax = -1;
    for(TopExp_Explorer aFaceExplorer(aBody, TopAbs_FACE); aFaceExplorer.More(); aFaceExplorer.Next()){
        TopoDS_Face aFace = TopoDS::Face(aFaceExplorer.Current());
        occ::handle<Geom_Surface> aSurface = BRep_Tool::Surface(aFace);
        if(!aSurface.IsNull() && aSurface->DynamicType() == STANDARD_TYPE(Geom_Plane)){
            occ::handle<Geom_Plane> aPlane = occ::down_cast<Geom_Plane>(aSurface);
            if (!aPlane.IsNull()) {
                gp_Pnt aPnt = aPlane->Location();
                double aZ   = aPnt.Z();
                if(aZ > aZMax){ aZMax = aZ; aFaceToRemove = aFace; }
            }
        }
    }

    NCollection_List<TopoDS_Shape> aFacesToRemove;
    aFacesToRemove.Append(aFaceToRemove);
    BRepOffsetAPI_MakeThickSolid aSolidMaker;
    aSolidMaker.MakeThickSolidByJoin(aBody, aFacesToRemove, -theThickness / 50, 1.e-3);
    aBody = aSolidMaker.Shape();

    occ::handle<Geom_CylindricalSurface> aCyl1 = new Geom_CylindricalSurface(neckAx2, aNeckRadius * 0.99);
    occ::handle<Geom_CylindricalSurface> aCyl2 = new Geom_CylindricalSurface(neckAx2, aNeckRadius * 1.05);

    gp_Pnt2d aPnt(2. * M_PI, aNeckHeight / 2.);
    gp_Dir2d aDir(2. * M_PI, aNeckHeight / 4.);
    gp_Ax2d anAx2d(aPnt, aDir);

    double aMajor = 2. * M_PI;
    double aMinor = aNeckHeight / 10;

    occ::handle<Geom2d_Ellipse> anEllipse1 = new Geom2d_Ellipse(anAx2d, aMajor, aMinor);
    occ::handle<Geom2d_Ellipse> anEllipse2 = new Geom2d_Ellipse(anAx2d, aMajor, aMinor / 4);
    occ::handle<Geom2d_TrimmedCurve> anArc1 = new Geom2d_TrimmedCurve(anEllipse1, 0, M_PI);
    occ::handle<Geom2d_TrimmedCurve> anArc2 = new Geom2d_TrimmedCurve(anEllipse2, 0, M_PI);
    gp_Pnt2d anEllipsePnt1 = anEllipse1->Value(0);
    gp_Pnt2d anEllipsePnt2 = anEllipse1->Value(M_PI);

    occ::handle<Geom2d_TrimmedCurve> aSegment = GC_MakeSegment2d(anEllipsePnt1, anEllipsePnt2);
    TopoDS_Edge anEdge1OnSurf1 = BRepBuilderAPI_MakeEdge(anArc1, aCyl1);
    TopoDS_Edge anEdge2OnSurf1 = BRepBuilderAPI_MakeEdge(aSegment, aCyl1);
    TopoDS_Edge anEdge1OnSurf2 = BRepBuilderAPI_MakeEdge(anArc2, aCyl2);
    TopoDS_Edge anEdge2OnSurf2 = BRepBuilderAPI_MakeEdge(aSegment, aCyl2);
    TopoDS_Wire aThreadingWire1 = BRepBuilderAPI_MakeWire(anEdge1OnSurf1, anEdge2OnSurf1);
    TopoDS_Wire aThreadingWire2 = BRepBuilderAPI_MakeWire(anEdge1OnSurf2, anEdge2OnSurf2);
    BRepLib::BuildCurves3d(aThreadingWire1);
    BRepLib::BuildCurves3d(aThreadingWire2);

    BRepOffsetAPI_ThruSections aTool(true);
    aTool.AddWire(aThreadingWire1);
    aTool.AddWire(aThreadingWire2);
    aTool.CheckCompatibility(false);
    TopoDS_Shape aThreading = aTool.Shape();

    TopoDS_Compound aRes;
    BRep_Builder aBuilder;
    aBuilder.MakeCompound (aRes);
    aBuilder.Add (aRes, aBody);
    aBuilder.Add (aRes, aThreading);
    return aRes;
}

// ============================================================
// Geometry helpers
// ============================================================
double shapeVolume(const TopoDS_Shape& s)
{
    GProp_GProps props;
    BRepGProp::VolumeProperties(s, props);
    return props.Mass();
}

int countSub(const TopoDS_Shape& s, TopAbs_ShapeEnum t)
{
    int n = 0;
    for (TopExp_Explorer ex(s, t); ex.More(); ex.Next()) n++;
    return n;
}

gp_Pnt bboxCenter(const TopoDS_Shape& s, double& diag)
{
    Bnd_Box box;
    BRepBndLib::Add(s, box);
    double xmin, ymin, zmin, xmax, ymax, zmax;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    diag = std::sqrt((xmax-xmin)*(xmax-xmin) + (ymax-ymin)*(ymax-ymin) + (zmax-zmin)*(zmax-zmin));
    return gp_Pnt((xmin+xmax)/2, (ymin+ymax)/2, (zmin+zmax)/2);
}

// ============================================================
// STEP file I/O
// ============================================================
bool writeSTEP(const TopoDS_Shape& s, const std::string& path)
{
    STEPControl_Writer w;
    IFSelect_ReturnStatus st = w.Transfer(s, STEPControl_AsIs);
    if (st != IFSelect_RetDone) return false;
    st = w.Write(path.c_str());
    return st == IFSelect_RetDone;
}

TopoDS_Shape readSTEP(const std::string& path, bool& ok)
{
    ok = false;
    STEPControl_Reader r;
    IFSelect_ReturnStatus st = r.ReadFile(path.c_str());
    if (st != IFSelect_RetDone) return TopoDS_Shape();
    if (r.NbRootsForTransfer() < 1) return TopoDS_Shape();
    if (!r.TransferRoots()) return TopoDS_Shape();
    TopoDS_Shape s = r.OneShape();
    if (s.IsNull()) return TopoDS_Shape();
    ok = true;
    return s;
}

// ============================================================
// Boolean intersection (shared, validated)
// ============================================================
// NOTE: OCCT 8 BRepAlgoAPI_Common silently returns an EMPTY result
// when an argument is a compound (even one holding nothing but valid
// solids). Feeding the solids as a flat argument/tool list works
// correctly — so extract solids first.
TopoDS_Shape intersectSolids(const TopoDS_Shape& a, const TopoDS_Shape& b,
                             std::string& err)
{
    err.clear();
    try {
        NCollection_List<TopoDS_Shape> args, tools;
        for (TopExp_Explorer e(a, TopAbs_SOLID); e.More(); e.Next())
            args.Append(e.Current());
        for (TopExp_Explorer e(b, TopAbs_SOLID); e.More(); e.Next())
            tools.Append(e.Current());
        if (args.IsEmpty() || tools.IsEmpty()) {
            err = "input has no solids to intersect";
            return TopoDS_Shape();
        }
        BRepAlgoAPI_Common common;
        common.SetArguments(args);
        common.SetTools(tools);
        common.Build();
        if (!common.IsDone() || common.HasErrors()) {
            err = "Common not done / has errors";
            return TopoDS_Shape();
        }
        TopoDS_Shape result = common.Shape();
        if (result.IsNull()) {
            err = "result is null shape";
            return TopoDS_Shape();
        }
        return result;
    } catch (const Standard_Failure& e) {
        err = std::string("OCCT exception: ") + e.GetMessageString();
    } catch (const std::exception& e) {
        err = std::string("std exception: ") + e.what();
    } catch (...) {
        err = "unknown exception";
    }
    return TopoDS_Shape();
}

std::string volumePlausibilityError(double V, double vin1, double vin3)
{
    const double vinMin = vin1 < vin3 ? vin1 : vin3;
    if (V <= 1e-9) {
        char vb[96];
        snprintf(vb, sizeof(vb), "invalid volume (V=%.6g): must be > 0", V);
        return vb;
    }
    if (V > vinMin * (1.0 + 1e-6)) {
        char vb[192];
        snprintf(vb, sizeof(vb),
                 "impossible volume: V=%.6g > min input %.6g (Vin1=%.6g, Vin3=%.6g)",
                 V, vinMin, vin1, vin3);
        return vb;
    }
    return std::string();
}

std::string jesc(const std::string& s)
{
    std::string o;
    for (char c : s) {
        if (c == '"' || c == '\\') { o += '\\'; o += c; }
        else if (c == '\n') o += "\\n";
        else o += c;
    }
    return o;
}
