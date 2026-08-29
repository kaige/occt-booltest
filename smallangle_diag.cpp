// ============================================================
// smallangle_diag — minimal repro for small-angle self-intersect
//
// Question: solid bottle ∩ (bottle rotated 0.5° about X through
// bbox center) returns a 0.17 mm³ sliver. Physically the lens
// should be ~97% of the body. Isolate:
//   1. plain box(50,30,70) — no modeling features at all
//   2. solid bottle BODY only (no threading solid)
//   3. full solid bottle compound
// each at 0.5°/1°/2°/5°, with and without SetFuzzyValue.
// ============================================================
#include <cstdio>
#include <string>
#include <vector>

#include <TopoDS_Shape.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <gp_Trsf.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <NCollection_List.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Version.hxx>

#include "common.hpp"

static TopoDS_Shape rotX(const TopoDS_Shape& s, double deg)
{
    double diag; gp_Pnt c = bboxCenter(s, diag);
    gp_Trsf rot;
    rot.SetRotation(gp_Ax1(c, gp_Dir(1, 0, 0)), deg * M_PI / 180.0);
    return BRepBuilderAPI_Transform(s, rot).Shape();
}

static double vol(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

static TopoDS_Shape commonOf(const TopoDS_Shape& a, const TopoDS_Shape& b,
                             double fuzzy, std::string& err)
{
    err.clear();
    NCollection_List<TopoDS_Shape> args, tools;
    args.Append(a);
    tools.Append(b);
    BRepAlgoAPI_Common op;
    op.SetArguments(args);
    op.SetTools(tools);
    if (fuzzy > 0) op.SetFuzzyValue(fuzzy);
    try {
        op.Build();
    } catch (const Standard_Failure& e) {
        err = std::string("exception: ") + e.what();
        return TopoDS_Shape();
    }
    if (!op.IsDone() || op.HasErrors()) { err = "not done / has errors"; return TopoDS_Shape(); }
    TopoDS_Shape r = op.Shape();
    if (r.IsNull()) { err = "null shape"; return TopoDS_Shape(); }
    return r;
}

int main()
{
    printf("OCCT version: %s\n", OCC_VERSION_COMPLETE);
    TopoDS_Shape box = BRepPrimAPI_MakeBox(50., 30., 70.).Shape();
    const double Vbox = vol(box);

    // solid bottle, split into body solid (first) and threading solid (rest)
    TopoDS_Shape fullBottle = MakeBottleSolid(50., 70., 30.);
    TopoDS_Shape bodySolid, threadSolid;
    int si = 0;
    for (TopExp_Explorer ex(fullBottle, TopAbs_SOLID); ex.More(); ex.Next(), si++)
        if (si == 0) bodySolid = ex.Current();
        else if (si == 1) threadSolid = ex.Current();
    const double Vbody = vol(bodySolid);
    const double Vthread = vol(threadSolid);
    const double Vfull = vol(fullBottle);

    struct ShapeCase { const char* name; TopoDS_Shape s; double V; };
    std::vector<ShapeCase> shapes = {
        {"box(50x30x70)",        box,         Vbox   },
        {"bottle BODY solid",    bodySolid,   Vbody  },
        {"bottle THREAD solid",  threadSolid, Vthread},
        {"bottle full(+thread)", fullBottle,  Vfull  },
    };

    const double thetas[] = {0.5, 1.0, 2.0, 5.0};
    const double fuzzies[] = {0, 1e-7, 1e-6, 1e-5, 1e-4};

    for (auto& sc : shapes) {
        printf("\n===== %s  (V0=%.3f) =====\n", sc.name, sc.V);
        for (double deg : thetas) {
            TopoDS_Shape rot = rotX(sc.s, deg);
            for (double fz : fuzzies) {
                std::string err;
                TopoDS_Shape r = commonOf(sc.s, rot, fz, err);
                if (!err.empty()) {
                    printf("  deg=%-4.1f fuzzy=%-6.0e  FAILED  (%s)\n", deg, fz, err.c_str());
                    continue;
                }
                double V = vol(r);
                printf("  deg=%-4.1f fuzzy=%-6.0e  V=%14.4f  (%7.4f%% of V0)\n",
                       deg, fz, V, 100.0 * V / sc.V);
            }
        }
    }
    return 0;
}
