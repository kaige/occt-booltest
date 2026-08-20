// ============================================================
// occt-booltest — OCCT boolean (intersection) stress-test generator
//
// Algorithm (per user spec):
//   1. body1 = the tutorial bottle solid
//   2. body2 = copy of body1 (coincident); rotate body2 about its
//      bbox center, around axis `axis`, by `theta` -> body3
//   3. body4 = body3 ∩ body1 (BRepAlgoAPI_Common).
//      SUCCESS = the operation completes AND result has volume > 0.
//      FAILURE = exception / not done / null / no-volume result;
//      record the failing pair (body3, body1, bool-intersect) as
//      .brep files + info, for later replay.
//      Result plausibility is validated too: the intersection of two
//      solids can never be larger than either input, so
//      SUCCESS additionally requires 0 < V <= min(Vin1, Vin3)*(1+1e-6).
//      A "successfully built" result violating this is recorded as
//      FAILED with an "impossible volume" error (OCCT silently returns
//      such garbage in near-degenerate configs).
//   4. On success body1 <- body4 and repeat until:
//        - a boolean failure occurs, or
//        - volume < V0/1000 (V0 = original bottle volume), or
//        - iterations > 500 (configurable), or
//        - per-case wall-clock cap (engineering guard, configurable).
//
// Test cases are named  axis<A>_theta<T>_it<N>  (loop count in name).
// I/O modes (--io):
//   memory (default) - bodies are handed between iterations in memory
//                      (TopoDS_Shape handle assignment, lossless).
//   step             - every boolean input is read back from STEP:
//                      round starts from the canonical bottle.step, each
//                      successful result body4.step is re-read as the next
//                      round's body1. This exercises serialization
//                      round-trips and makes the archived per-iteration
//                      body1.step/body3.step pairs a kernel-independent
//                      benchmark corpus for other geometry kernels.
// Screenshots: iteration 1, every 25th, final result; failure pair
// shapes are exported as BREP.
// Output: <outdir>/summary.json + summary.csv + pngs + failures/.
// ============================================================

#include <QApplication>
#include <QOpenGLWidget>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QSurfaceFormat>
#include <QTimer>
#include <QThread>
#include <QImage>
#include <QMatrix4x4>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <cstdio>
#include <cmath>
#include <algorithm>

// --- OCCT ---
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
#include <TopoDS_Shape.hxx>
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
#include <BRepTools.hxx>
#include <STEPControl_Writer.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_StepModelType.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <NCollection_List.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <Poly_Triangulation.hxx>
#include <TopLoc_Location.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>

// ============================================================
// MakeBottle — verbatim from the OCCT tutorial (dox/tutorial)
// ============================================================
static TopoDS_Shape MakeBottle(double theWidth, double theHeight,
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
static double shapeVolume(const TopoDS_Shape& s)
{
    GProp_GProps props;
    BRepGProp::VolumeProperties(s, props);
    return props.Mass();
}

static int countSub(const TopoDS_Shape& s, TopAbs_ShapeEnum t)
{
    int n = 0;
    for (TopExp_Explorer ex(s, t); ex.More(); ex.Next()) n++;
    return n;
}

static gp_Pnt bboxCenter(const TopoDS_Shape& s, double& diag)
{
    Bnd_Box box;
    BRepBndLib::Add(s, box);
    double xmin, ymin, zmin, xmax, ymax, zmax;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    diag = std::sqrt((xmax-xmin)*(xmax-xmin) + (ymax-ymin)*(ymax-ymin) + (zmax-zmin)*(zmax-zmin));
    return gp_Pnt((xmin+xmax)/2, (ymin+ymax)/2, (zmin+zmax)/2);
}

// STEP export (case inputs are archived in STEP per requirement)
static bool writeSTEP(const TopoDS_Shape& s, const std::string& path)
{
    STEPControl_Writer w;
    IFSelect_ReturnStatus st = w.Transfer(s, STEPControl_AsIs);
    if (st != IFSelect_RetDone) return false;
    st = w.Write(path.c_str());
    return st == IFSelect_RetDone;
}

// STEP import (for --io step mode: every boolean input starts as a STEP
// read-back, making the corpus a kernel-independent benchmark set)
static TopoDS_Shape readSTEP(const std::string& path, bool& ok)
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
// Tessellation + offscreen viewer (reused from bottle-tutorial,
// generalized with setMesh for repeated snapshots)
// ============================================================
struct Mesh {
    std::vector<float> triVerts, lineVerts;
    int triCount = 0, lineCount = 0;
};

static Mesh tessellate(const TopoDS_Shape& shape, double defl)
{
    Mesh m;
    BRepMesh_IncrementalMesh mesher(shape, defl, false, 0.5, true);
    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
        TopoDS_Face face = TopoDS::Face(ex.Current());
        TopLoc_Location loc;
        occ::handle<Poly_Triangulation> tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull() || tri->NbTriangles() == 0) continue;
        const gp_Trsf& T = loc.Transformation();
        const int n = (int)tri->NbNodes();
        std::vector<gp_Pnt> P(n);
        std::vector<gp_Vec> N(n, gp_Vec(0, 0, 0));
        for (int i = 0; i < n; i++) P[i] = tri->Node(i + 1).Transformed(T);
        for (int i = 1; i <= (int)tri->NbTriangles(); i++) {
            int a, b, c; tri->Triangle(i).Get(a, b, c);
            gp_Vec fn = gp_Vec(P[a-1], P[b-1]).Crossed(gp_Vec(P[a-1], P[c-1]));
            if (face.Orientation() == TopAbs_REVERSED) fn.Reverse();
            N[a-1] += fn; N[b-1] += fn; N[c-1] += fn;
        }
        for (int i = 1; i <= (int)tri->NbTriangles(); i++) {
            int a, b, c; tri->Triangle(i).Get(a, b, c);
            int idx[3] = {a-1, b-1, c-1};
            for (int k = 0; k < 3; k++) {
                gp_Vec nrm = N[idx[k]];
                if (nrm.Magnitude() < 1e-12) nrm = gp_Vec(0, 0, 1);
                nrm.Normalize();
                if (face.Orientation() == TopAbs_REVERSED) nrm.Reverse();
                const gp_Pnt& p = P[idx[k]];
                m.triVerts.push_back((float)p.X()); m.triVerts.push_back((float)p.Y()); m.triVerts.push_back((float)p.Z());
                m.triVerts.push_back((float)nrm.X()); m.triVerts.push_back((float)nrm.Y()); m.triVerts.push_back((float)nrm.Z());
            }
            m.triCount += 3;
        }
    }
    for (TopExp_Explorer ex(shape, TopAbs_EDGE); ex.More(); ex.Next()) {
        TopoDS_Edge edge = TopoDS::Edge(ex.Current());
        if (BRep_Tool::Degenerated(edge)) continue;
        BRepAdaptor_Curve curve(edge);
        GCPnts_QuasiUniformDeflection discr(curve, defl);
        if (!discr.IsDone() || discr.NbPoints() < 2) continue;
        gp_Pnt prev = discr.Value(1);
        for (int i = 2; i <= discr.NbPoints(); i++) {
            gp_Pnt p = discr.Value(i);
            m.lineVerts.push_back((float)prev.X()); m.lineVerts.push_back((float)prev.Y()); m.lineVerts.push_back((float)prev.Z());
            m.lineVerts.push_back((float)p.X()); m.lineVerts.push_back((float)p.Y()); m.lineVerts.push_back((float)p.Z());
            prev = p;
            m.lineCount += 2;
        }
    }
    return m;
}

class ShapeView : public QOpenGLWidget {
public:
    ShapeView(QWidget* parent = nullptr) : QOpenGLWidget(parent) {}

    void setShape(const TopoDS_Shape& shape, double deflScale)
    {
        double diag;
        bboxCenter(shape, diag);
        double defl = std::max(0.05, diag * deflScale);
        Mesh m = tessellate(shape, defl);
        m_centerRadii = m;
        makeCurrent();
        upload(std::move(m));
        doneCurrent();
        update();
    }

protected:
    void initializeGL() override {
        glEnable(GL_DEPTH_TEST);
        prog.addShaderFromSourceCode(QOpenGLShader::Vertex,
            "#version 330 core\n"
            "layout(location=0) in vec3 pos;\n"
            "layout(location=1) in vec3 nrm;\n"
            "uniform mat4 mvp;\n"
            "out vec3 vN; out vec3 vP;\n"
            "void main(){ vN=nrm; vP=pos; gl_Position=mvp*vec4(pos,1.0); }");
        prog.addShaderFromSourceCode(QOpenGLShader::Fragment,
            "#version 330 core\n"
            "in vec3 vN; in vec3 vP; out vec4 frag;\n"
            "uniform vec3 eye;\n"
            "void main(){\n"
            "  vec3 N = normalize(vN);\n"
            "  if(!gl_FrontFacing) N = -N;\n"
            "  vec3 base = vec3(0.36, 0.50, 0.62);\n"
            "  vec3 L1 = normalize(vec3(0.45, -0.75, 0.55));\n"
            "  vec3 L2 = normalize(vec3(-0.60, 0.35, 0.30));\n"
            "  float d1 = max(dot(N,L1), 0.0);\n"
            "  float d2 = max(dot(N,L2), 0.0);\n"
            "  vec3 V = normalize(eye - vP);\n"
            "  vec3 H = normalize(L1 + V);\n"
            "  float sp = pow(max(dot(N,H), 0.0), 48.0) * 0.35;\n"
            "  vec3 c = base * (0.32 + 0.60*d1 + 0.16*d2) + vec3(sp);\n"
            "  frag = vec4(c, 1.0);\n"
            "}");
        prog.link();
        lprog.addShaderFromSourceCode(QOpenGLShader::Vertex,
            "#version 330 core\n"
            "layout(location=0) in vec3 pos;\n"
            "uniform mat4 mvp;\n"
            "void main(){ gl_Position=mvp*vec4(pos,1.0); }");
        lprog.addShaderFromSourceCode(QOpenGLShader::Fragment,
            "#version 330 core\n"
            "uniform vec3 color; out vec4 frag;\n"
            "void main(){ frag=vec4(color,1.0); }");
        lprog.link();
        vboTri.create(); vaoTri.create();
        vboLine.create(); vaoLine.create();
    }

    void resizeGL(int w, int h) override {
        glViewport(0, 0, w, h);
        (void)w; (void)h;
    }

    void paintGL() override {
        glClearColor(0.937f, 0.945f, 0.953f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (triCountTotal == 0) return;

        int w = width(), h = height();
        QMatrix4x4 proj;
        proj.perspective(22.0f, (float)w / (float)h, 10.0f, 4000.0f);
        QMatrix4x4 view;
        eye = m_center + QVector3D(0.85f, -1.65f, 0.75f).normalized() * (m_radius * 7.5f);
        view.lookAt(eye, m_center, QVector3D(0, 0, 1));
        QMatrix4x4 mvp = proj * view;

        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0f, 1.0f);
        prog.bind();
        prog.setUniformValue("mvp", mvp);
        prog.setUniformValue("eye", eye);
        vaoTri.bind();
        glDrawArrays(GL_TRIANGLES, 0, triCountTotal);
        vaoTri.release();
        prog.release();
        glDisable(GL_POLYGON_OFFSET_FILL);

        lprog.bind();
        lprog.setUniformValue("mvp", mvp);
        lprog.setUniformValue("color", 0.10f, 0.13f, 0.20f);
        vaoLine.bind();
        glDrawArrays(GL_LINES, 0, lineCountTotal);
        vaoLine.release();
        lprog.release();
    }

private:
    void upload(Mesh&& m)
    {
        vboTri.bind();
        vboTri.allocate(m.triVerts.data(), (int)(m.triVerts.size() * sizeof(float)));
        vboTri.release();
        vaoTri.bind();
        vboTri.bind();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 24, (void*)0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 24, (void*)12);
        glEnableVertexAttribArray(0); glEnableVertexAttribArray(1);
        vaoTri.release();

        vboLine.bind();
        vboLine.allocate(m.lineVerts.data(), (int)(m.lineVerts.size() * sizeof(float)));
        vboLine.release();
        vaoLine.bind();
        vboLine.bind();
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 12, (void*)0);
        glEnableVertexAttribArray(0);
        vaoLine.release();

        triCountTotal = m.triCount;
        lineCountTotal = m.lineCount;
    }

    QVector3D m_center, eye;
    float m_radius = 1.0f;
    Mesh m_centerRadii; // placeholder (kept for API symmetry)
    int triCountTotal = 0, lineCountTotal = 0;
    QOpenGLShaderProgram prog, lprog;
    QOpenGLBuffer vboTri{QOpenGLBuffer::VertexBuffer}, vboLine{QOpenGLBuffer::VertexBuffer};
    QOpenGLVertexArrayObject vaoTri, vaoLine;

public:
    void setViewFrame(const QVector3D& center, float radius) {
        m_center = center; m_radius = std::max(radius, 1.0f);
    }
};

// ============================================================
// Test harness
// ============================================================
// per-iteration record (successes AND failures are logged per requirement)
struct IterRec {
    int iter = 0;
    bool ok = false;
    double V = 0;              // result volume (0 on failure)
    double vinMin = 0;         // min(input volumes) — plausibility upper bound
    int faces = 0, edges = 0;  // result topology (0 on failure)
    double tSec = 0;           // elapsed at end of this iteration
    std::string note;          // error message on failure
    std::string dir;           // iteration dir holding STEP files
};

struct CaseResult {
    std::string axisName;
    double thetaDeg = 0;
    int itersDone = 0;             // successful boolean iterations
    std::string termReason;        // boolean-failed | volume-threshold | max-iterations | time-cap
    double finalVolume = 0;
    double V0 = 0;
    double durationSec = 0;
    int finalFaces = 0, finalEdges = 0;
    std::string failDetail;        // exception message etc.
    std::vector<std::string> shots;
    std::vector<IterRec> iterLog;  // one record per executed iteration
};

static std::string jesc(const std::string& s)
{
    std::string o;
    for (char c : s) {
        if (c == '"' || c == '\\') { o += '\\'; o += c; }
        else if (c == '\n') o += "\\n";
        else o += c;
    }
    return o;
}

int main(int argc, char* argv[])
{
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setDepthBufferSize(24);
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);

    // ---- args ----
    std::string outDir = "output";
    std::vector<std::string> axes = {"x", "y", "z", "diag"};
    std::vector<double> thetas = {1, 2, 5, 15, 45, 90, 180};
    int maxIters = 500;
    double caseCapSec = 240.0;
    std::string ioMode = "memory";   // memory: in-memory handoff | step: STEP read-back each iteration
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
        if (a == "--out") outDir = next();
        else if (a == "--axes") {
            axes.clear();
            std::stringstream ss(next());
            std::string item;
            while (std::getline(ss, item, ',')) if (!item.empty()) axes.push_back(item);
        }
        else if (a == "--thetas") {
            thetas.clear();
            std::stringstream ss(next());
            std::string item;
            while (std::getline(ss, item, ',')) if (!item.empty()) thetas.push_back(atof(item.c_str()));
        }
        else if (a == "--max-iters") maxIters = atoi(next().c_str());
        else if (a == "--case-seconds") caseCapSec = atof(next().c_str());
        else if (a == "--io") ioMode = next();
    }
    const bool stepIO = (ioMode == "step");
    if (!stepIO && ioMode != "memory") {
        printf("unknown --io mode '%s' (use memory|step)\n", ioMode.c_str());
        return 2;
    }

    std::system(("mkdir -p " + outDir + "/failures " + outDir + "/png "
                 + outDir + "/iterations").c_str());

    printf("== occt-booltest ==\n");
    printf("cases: %zu axes x %zu thetas, max %d iters/case, cap %.0fs/case, io: %s\n",
           axes.size(), thetas.size(), maxIters, caseCapSec, ioMode.c_str());

    // ---- base model ----
    printf("building bottle...\n");
    TopoDS_Shape bottle = MakeBottle(50., 70., 30.);
    const double V0 = shapeVolume(bottle);
    printf("V0 = %.3f mm^3 (threshold V0/1000 = %.3f)\n", V0, V0 / 1000.0);

    // canonical benchmark input: every case starts from this STEP file
    if (stepIO) {
        if (!writeSTEP(bottle, outDir + "/bottle.step")) {
            printf("FATAL: cannot write %s/bottle.step\n", outDir.c_str());
            return 2;
        }
        bool okR = false;
        TopoDS_Shape s = readSTEP(outDir + "/bottle.step", okR);
        if (!okR) { printf("FATAL: cannot read back bottle.step\n"); return 2; }
        printf("bottle.step roundtrip: V=%.3f (delta %.3g%%)\n",
               shapeVolume(s), 100.0 * (shapeVolume(s) - V0) / V0);
    }

    ShapeView view;
    view.resize(1100, 850);
    view.show();
    QApplication::processEvents();

    auto snapshot = [&](const TopoDS_Shape& shape, const std::string& name, double deflScale) -> std::string {
        double diag; gp_Pnt c = bboxCenter(shape, diag);
        view.setViewFrame(QVector3D(c.X(), c.Y(), c.Z()), (float)diag / 2);
        view.setShape(shape, deflScale);
        QApplication::processEvents();
        QThread::msleep(40);
        QApplication::processEvents();
        std::string path = outDir + "/png/" + name + ".png";
        QImage fb = view.grabFramebuffer();
        fb.save(QString::fromStdString(path));
        printf("  [shot] %s\n", path.c_str());
        return path;
    };

    std::vector<CaseResult> results;

    for (auto& axName : axes) {
        gp_Dir axisDir(1, 0, 0);
        if (axName == "x") axisDir = gp_Dir(1, 0, 0);
        else if (axName == "y") axisDir = gp_Dir(0, 1, 0);
        else if (axName == "z") axisDir = gp_Dir(0, 0, 1);
        else if (axName == "diag") axisDir = gp_Dir(1, 1, 1);
        else { printf("unknown axis '%s', skipped\n", axName.c_str()); continue; }

        for (double thetaDeg : thetas) {
            char buf[64];
            snprintf(buf, sizeof(buf), "axis%s_theta%g", axName.c_str(), thetaDeg);
            std::string caseName = buf;
            printf("\n>>> case %s\n", caseName.c_str());

            auto t0 = std::chrono::steady_clock::now();
            CaseResult res;
            res.axisName = axName;
            res.thetaDeg = thetaDeg;
            res.V0 = V0;

            TopoDS_Shape body1 = bottle;   // fresh start each case
            if (stepIO) {
                bool okR = false;
                TopoDS_Shape s = readSTEP(outDir + "/bottle.step", okR);
                if (!okR) {
                    printf("  bottle.step read failed -> skip case\n");
                    res.termReason = "input-read-failed";
                    results.push_back(res);
                    continue;
                }
                body1 = s;
            }
            double thetaRad = thetaDeg * M_PI / 180.0;
            int iter = 0;
            bool stop = false;

            while (!stop) {
                double elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t0).count();
                if (elapsed > caseCapSec) {
                    res.termReason = "time-cap";
                    printf("  elapsed %.1fs > cap -> stop\n", elapsed);
                    break;
                }
                if (iter >= maxIters) {
                    res.termReason = "max-iterations";
                    printf("  reached max iterations (%d) -> stop\n", maxIters);
                    break;
                }
                // Face-explosion guard: iterated self-intersection grows the
                // face count super-linearly; once a body exceeds this budget
                // a single Common() can take >10 minutes. Record and stop —
                // this is a performance finding, not an algorithm failure.
                {
                    int curFaces = countSub(body1, TopAbs_FACE);
                    if (curFaces > 4000) {
                        res.termReason = "face-explosion-guard";
                        printf("  body1 has %d faces (>4000) -> stop (perf guard)\n", curFaces);
                        break;
                    }
                }
                iter++;

                // ---- archive this iteration's INPUT bodies as STEP ----
                char itd[160];
                snprintf(itd, sizeof(itd), "%s/iterations/%s/it%03d",
                         outDir.c_str(), caseName.c_str(), iter);
                std::string itDir = itd;
                std::system(("mkdir -p " + itDir).c_str());

                // step 2: copy body1, rotate about bbox center
                double diag;
                gp_Pnt center = bboxCenter(body1, diag);
                gp_Trsf rot;
                rot.SetRotation(gp_Ax1(center, axisDir), thetaRad);
                BRepBuilderAPI_Transform xform(body1, rot, /*copy=*/Standard_True);
                TopoDS_Shape body3 = xform.Shape();

                writeSTEP(body1, itDir + "/body1.step");
                writeSTEP(body3, itDir + "/body3.step");

                // input volumes for the physical-plausibility check:
                // Common(body1, body3) must satisfy 0 < V <= min(Vin1, Vin3).
                // (Rotation preserves volume mathematically; measure both
                // anyway — it is cheap next to the boolean itself.)
                const double vin1 = shapeVolume(body1);
                const double vin3 = shapeVolume(body3);
                const double vinMin = vin1 < vin3 ? vin1 : vin3;

                // step 3: boolean intersection
                // NOTE: OCCT 8 BRepAlgoAPI_Common silently returns an EMPTY
                // result when an argument is a compound (even one holding
                // nothing but valid solids). Feeding the solids as a flat
                // argument/tool list works correctly — so extract solids.
                TopoDS_Shape body4;
                double V = -1;
                bool ok = false;
                std::string errMsg;
                try {
                    NCollection_List<TopoDS_Shape> args, tools;
                    for (TopExp_Explorer e(body1, TopAbs_SOLID); e.More(); e.Next())
                        args.Append(e.Current());
                    for (TopExp_Explorer e(body3, TopAbs_SOLID); e.More(); e.Next())
                        tools.Append(e.Current());
                    if (args.IsEmpty() || tools.IsEmpty()) {
                        errMsg = "input has no solids to intersect";
                    } else {
                        BRepAlgoAPI_Common common;
                        common.SetArguments(args);
                        common.SetTools(tools);
                        common.Build();
                        if (!common.IsDone() || common.HasErrors()) {
                            errMsg = "Common not done / has errors";
                        } else {
                            body4 = common.Shape();
                            if (body4.IsNull()) {
                                errMsg = "result is null shape";
                            } else {
                                V = shapeVolume(body4);
                                // --- result plausibility validation ---
                                // (1) positive: catches empty results and
                                //     negative volumes (inverted orientation)
                                // (2) physically possible: an intersection
                                //     cannot exceed either input volume
                                if (V <= 1e-9) {
                                    char vb[96];
                                    snprintf(vb, sizeof(vb),
                                             "invalid volume (V=%.6g): must be > 0", V);
                                    errMsg = vb;
                                } else if (V > vinMin * (1.0 + 1e-6)) {
                                    char vb[192];
                                    snprintf(vb, sizeof(vb),
                                             "impossible volume: V=%.6g > min input %.6g "
                                             "(Vin1=%.6g, Vin3=%.6g)",
                                             V, vinMin, vin1, vin3);
                                    errMsg = vb;
                                } else {
                                    ok = true;
                                }
                            }
                        }
                    }
                } catch (const Standard_Failure& e) {
                    errMsg = std::string("OCCT exception: ") + e.GetMessageString();
                } catch (const std::exception& e) {
                    errMsg = std::string("std exception: ") + e.what();
                } catch (...) {
                    errMsg = "unknown exception";
                }

                double iterStart = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t0).count();

                if (!ok) {
                    // record the failing case: (body3, body1, bool-intersect)
                    res.termReason = "boolean-failed";
                    res.failDetail = errMsg;
                    BRepTools::Write(body3, (itDir + "/body3.brep").c_str());
                    BRepTools::Write(body1, (itDir + "/body1.brep").c_str());
                    std::ofstream info(itDir + "/info.txt");
                    info << "case: " << caseName << "\n"
                         << "failed at iteration: " << iter << "\n"
                         << "status: FAILED\n"
                         << "operation: boolean-intersect (BRepAlgoAPI_Common)\n"
                         << "axis: " << axName << "  theta: " << thetaDeg << " deg\n"
                         << "error: " << errMsg << "\n"
                         << "input volumes: Vin1=" << vin1 << "  Vin3=" << vin3
                         << "  (plausible result must be in (0, " << vinMin << "])\n"
                         << "volume before this op: "
                         << (iter == 1 ? V0 : res.finalVolume) << "\n";
                    info.close();
                    std::string itDirF = itDir + "_failed";
                    std::system(("mv " + itDir + " " + itDirF).c_str());
                    IterRec rec;
                    rec.iter = iter; rec.ok = false;
                    rec.V = 0; rec.vinMin = vinMin; rec.faces = 0; rec.edges = 0;
                    rec.tSec = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - t0).count();
                    rec.note = errMsg; rec.dir = itDirF;
                    res.iterLog.push_back(rec);
                    snapshot(body1, caseName + "_it" + std::to_string(iter) + "_fail_body1", 0.006);
                    printf("  it%d FAILED: %s (saved %s)\n", iter, errMsg.c_str(), itDirF.c_str());
                    break;
                }

                // success
                res.itersDone = iter;
                res.finalVolume = V;
                res.finalFaces = countSub(body4, TopAbs_FACE);
                res.finalEdges = countSub(body4, TopAbs_EDGE);
                writeSTEP(body4, itDir + "/body4.step");
                std::string itDirS = itDir + "_success";
                std::system(("mv " + itDir + " " + itDirS).c_str());
                IterRec rec;
                rec.iter = iter; rec.ok = true; rec.V = V; rec.vinMin = vinMin;
                rec.faces = res.finalFaces; rec.edges = res.finalEdges;
                rec.tSec = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t0).count() - iterStart;
                rec.dir = itDirS;
                res.iterLog.push_back(rec);
                printf("  it%3d  V=%12.3f  (%.2f%% of V0)  faces=%d  edges=%d  [%.1fs]\n",
                       iter, V, 100.0 * V / V0, res.finalFaces, res.finalEdges,
                       std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());

                // screenshot policy: it1, every 25th, final (decided after loop)
                if (iter == 1 || iter % 25 == 0) {
                    double defl = 0.004 * (1.0 + iter / 25.0);
                    char nb[96];
                    snprintf(nb, sizeof(nb), "%s_it%03d", caseName.c_str(), iter);
                    res.shots.push_back(snapshot(body4, nb, defl));
                }

                if (V < V0 / 1000.0) {
                    res.termReason = "volume-threshold";
                    char nb[96];
                    snprintf(nb, sizeof(nb), "%s_it%03d_final", caseName.c_str(), iter);
                    res.shots.push_back(snapshot(body4, nb, 0.006));
                    printf("  V < V0/1000 -> stop\n");
                    stop = true;
                }
                if (stop) {
                    // no further iteration — skip the STEP read-back
                } else if (stepIO) {
                    // hand off through STEP: next boolean reads this file back
                    bool okR = false;
                    TopoDS_Shape rb = readSTEP(itDirS + "/body4.step", okR);
                    if (!okR) {
                        res.termReason = "step-read-failed";
                        printf("  body4.step read-back FAILED at it%d -> stop\n", iter);
                        break;
                    }
                    body1 = rb;
                } else {
                    body1 = body4;   // step 4: iterate on the intersection
                }
            }

            if (res.termReason.empty()) res.termReason = "max-iterations";
            // final snapshot for non-threshold terminations (if not already taken)
            if (res.termReason != "volume-threshold" && res.termReason != "boolean-failed" && res.itersDone > 0) {
                char nb[96];
                snprintf(nb, sizeof(nb), "%s_it%03d_final", caseName.c_str(), res.itersDone);
                res.shots.push_back(snapshot(body1, nb, 0.006));
            }
            res.durationSec = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            // per-case iteration log (successes AND failures)
            {
                std::ofstream ic(outDir + "/iterations/" + caseName + "_iterations.csv");
                ic << "case,iteration,status,volume,vin_min,volume_pct_of_V0,faces,edges,duration_sec,dir\n";
                for (auto& q : res.iterLog) {
                    ic << caseName << "," << q.iter << ","
                       << (q.ok ? "success" : "failed") << ","
                       << std::setprecision(10) << q.V << ","
                       << std::setprecision(10) << q.vinMin << ","
                       << std::setprecision(6) << 100.0 * q.V / V0 << ","
                       << q.faces << "," << q.edges << ","
                       << std::setprecision(3) << q.tSec << ","
                       << q.dir << "\n";
                }
            }
            results.push_back(res);
            printf("  => %s: %d iters, term=%s, Vfinal=%.4f, %.1fs\n",
                   caseName.c_str(), res.itersDone, res.termReason.c_str(),
                   res.finalVolume, res.durationSec);
            QApplication::processEvents();
        }
    }

    // ---- summary ----
    std::ofstream json(outDir + "/summary.json");
    json << "{\n  \"V0\": " << V0 << ",\n  \"io_mode\": \"" << ioMode << "\",\n  \"cases\": [\n";
    for (size_t i = 0; i < results.size(); i++) {
        auto& r = results[i];
        json << "    {\"case\": \"axis" << r.axisName << "_theta" << r.thetaDeg
             << "\", \"axis\": \"" << r.axisName << "\", \"theta_deg\": " << r.thetaDeg
             << ", \"iterations\": " << r.itersDone
             << ", \"termination\": \"" << r.termReason << "\""
             << ", \"final_volume\": " << std::setprecision(10) << r.finalVolume
             << ", \"duration_sec\": " << std::setprecision(3) << r.durationSec
             << ", \"final_faces\": " << r.finalFaces
             << ", \"final_edges\": " << r.finalEdges;
        if (!r.failDetail.empty())
            json << ", \"fail_detail\": \"" << jesc(r.failDetail) << "\"";
        int okN = 0, badN = 0;
        for (auto& q : r.iterLog) { if (q.ok) okN++; else badN++; }
        json << ", \"success_iterations\": " << okN
             << ", \"failed_iterations\": " << badN
             << ", \"iterations\": [";
        for (size_t k = 0; k < r.iterLog.size(); k++) {
            auto& q = r.iterLog[k];
            json << (k ? ", " : "")
                 << "{\"it\": " << q.iter
                 << ", \"status\": \"" << (q.ok ? "success" : "failed") << "\""
                 << ", \"volume\": " << std::setprecision(10) << q.V
                 << ", \"dir\": \"" << jesc(q.dir) << "\"}";
        }
        json << "]";
        json << ", \"snapshots\": [";
        for (size_t k = 0; k < r.shots.size(); k++)
            json << (k ? ", " : "") << "\"" << jesc(r.shots[k]) << "\"";
        json << "]}";
        if (i + 1 < results.size()) json << ",";
        json << "\n";
    }
    json << "  ]\n}\n";
    json.close();

    std::ofstream csv(outDir + "/summary.csv");
    csv << "io_mode,case,axis,theta_deg,iterations,termination,final_volume,duration_sec,final_faces,final_edges,success_iterations,failed_iterations\n";
    for (auto& r : results) {
        int okN = 0, badN = 0;
        for (auto& q : r.iterLog) { if (q.ok) okN++; else badN++; }
        csv << ioMode << ","
            << "axis" << r.axisName << "_theta" << r.thetaDeg << ","
            << r.axisName << "," << r.thetaDeg << ","
            << r.itersDone << "," << r.termReason << ","
            << std::setprecision(10) << r.finalVolume << ","
            << std::setprecision(3) << r.durationSec << ","
            << r.finalFaces << "," << r.finalEdges << ","
            << okN << "," << badN << "\n";
    }
    csv.close();

    // global iteration log across all cases
    {
        std::ofstream gi(outDir + "/all_iterations.csv");
        gi << "case,iteration,status,volume,volume_pct_of_V0,faces,edges,duration_sec,dir\n";
        for (auto& r : results)
            for (auto& q : r.iterLog)
                gi << "axis" << r.axisName << "_theta" << r.thetaDeg << ","
                   << q.iter << "," << (q.ok ? "success" : "failed") << ","
                   << std::setprecision(10) << q.V << ","
                   << std::setprecision(6) << 100.0 * q.V / r.V0 << ","
                   << q.faces << "," << q.edges << ","
                   << std::setprecision(3) << q.tSec << ","
                   << q.dir << "\n";
    }

    // console table
    printf("\n==== SUMMARY ====\n");
    printf("%-24s %5s %5s  %-17s %14s %8s\n", "case", "iters", "fails", "termination", "final_volume", "sec");
    for (auto& r : results) {
        char cn[64];
        snprintf(cn, sizeof(cn), "axis%s_theta%g", r.axisName.c_str(), r.thetaDeg);
        int badN = 0;
        for (auto& q : r.iterLog) if (!q.ok) badN++;
        printf("%-24s %5d %5d  %-17s %14.4f %8.1f\n", cn, r.itersDone, badN,
               r.termReason.c_str(), r.finalVolume, r.durationSec);
    }
    printf("wrote %s/summary.json, summary.csv, all_iterations.csv, iterations/, png/\n", outDir.c_str());
    return 0;
}
