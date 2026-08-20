// diagnose: why does Common(bottle, rotated bottle) report zero volume?
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS.hxx>
#include <NCollection_List.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <cstdio>

static void report(const char* tag, const TopoDS_Shape& s)
{
    int solids=0, shells=0, faces=0;
    for (TopExp_Explorer e(s, TopAbs_SOLID); e.More(); e.Next()) solids++;
    for (TopExp_Explorer e(s, TopAbs_SHELL); e.More(); e.Next()) shells++;
    for (TopExp_Explorer e(s, TopAbs_FACE); e.More(); e.Next()) faces++;
    GProp_GProps props;
    BRepGProp::VolumeProperties(s, props);
    printf("%-12s: solids=%d shells=%d faces=%d volume=%.4f null=%d empty-type=%d\n",
           tag, solids, shells, faces, props.Mass(), s.IsNull(), s.ShapeType());
}

int main()
{
    TopoDS_Shape body1, body3;
    BRepTools::Read(body1, "/tmp/booltest-smoke/failures/axisz_theta45_it1/body1.brep", BRep_Builder());
    BRepTools::Read(body3, "/tmp/booltest-smoke/failures/axisz_theta45_it1/body3.brep", BRep_Builder());
    report("body1", body1);
    report("body3", body3);

    // validity of inputs
    BRepCheck_Analyzer a1(body1), a3(body3);
    printf("valid: body1=%d body3=%d\n", a1.IsValid(), a3.IsValid());

    // experiment 1: plain Common as in booltest
    {
        BRepAlgoAPI_Common common(body3, body1);
        report("common1", common.Shape());
    }

    // experiment 2: solids only, explicit Build
    TopoDS_Compound c1, c3;
    BRep_Builder bb;
    bb.MakeCompound(c1); bb.MakeCompound(c3);
    for (TopExp_Explorer e(body1, TopAbs_SOLID); e.More(); e.Next()) bb.Add(c1, e.Current());
    for (TopExp_Explorer e(body3, TopAbs_SOLID); e.More(); e.Next()) bb.Add(c3, e.Current());
    report("c1-solids", c1);
    report("c3-solids", c3);
    {
        BRepAlgoAPI_Common common;
        NCollection_List<TopoDS_Shape> args, tools;
        args.Append(c3); tools.Append(c1);
        common.SetArguments(args);
        common.SetTools(tools);
        common.Build();
        printf("common2 IsDone=%d HasErrors=%d\n", common.IsDone(), common.HasErrors());
        report("common2", common.Shape());
    }

    // experiment 3: single solid from body1 vs rotated single solid
    {
        TopExp_Explorer e1(body1, TopAbs_SOLID), e3(body3, TopAbs_SOLID);
        TopoDS_Shape s1 = e1.Current(), s3 = e3.Current();
        report("s1", s1); report("s3", s3);
        BRepAlgoAPI_Common common(s3, s1);
        report("common3", common.Shape());
    }
    // experiment 4: flat lists of solids (no wrapping compound)
    {
        BRepAlgoAPI_Common common;
        NCollection_List<TopoDS_Shape> args, tools;
        for (TopExp_Explorer e(body3, TopAbs_SOLID); e.More(); e.Next()) tools.Append(e.Current());
        for (TopExp_Explorer e(body1, TopAbs_SOLID); e.More(); e.Next()) args.Append(e.Current());
        common.SetArguments(args);
        common.SetTools(tools);
        common.Build();
        printf("common4 IsDone=%d HasErrors=%d\n", common.IsDone(), common.HasErrors());
        report("common4", common.Shape());
    }
    return 0;
}
