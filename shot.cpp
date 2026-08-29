// ============================================================
// shot — standalone offscreen shape renderer (reuses booltest's
// tessellate() + ShapeView verbatim).
// usage: shot <model.step> <out.png> [deflScale=0.006]
// ============================================================

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

// --- OCCT (viewer/tessellation only; modeling & IO via common.hpp) ---
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax1.hxx>
#include <gp_Trsf.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <Poly_Triangulation.hxx>
#include <TopLoc_Location.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <BRepGProp.hxx>

#include "common.hpp"


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
// shot main: read STEP -> offscreen render -> PNG
// ============================================================
#include <cstdlib>

int main(int argc, char* argv[])
{
    if (argc < 3) {
        printf("usage: shot <model.step> <out.png> [deflScale=0.006]\n");
        return 2;
    }
    std::string inPath = argv[1], outPath = argv[2];
    double deflScale = (argc > 3) ? atof(argv[3]) : 0.006;

    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setDepthBufferSize(24);
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);

    bool ok = false;
    TopoDS_Shape shape = readSTEP(inPath, ok);
    if (!ok) { printf("cannot read '%s'\n", inPath.c_str()); return 2; }
    const double V = shapeVolume(shape);
    double diag; gp_Pnt c = bboxCenter(shape, diag);
    printf("V=%.4f mm^3  diag=%.2f  faces=%d  edges=%d  defl=%.3f\n",
           V, diag, countSub(shape, TopAbs_FACE), countSub(shape, TopAbs_EDGE),
           std::max(0.05, diag * deflScale));

    ShapeView view;
    view.resize(1100, 850);
    view.show();
    QApplication::processEvents();

    view.setViewFrame(QVector3D(c.X(), c.Y(), c.Z()), (float)diag / 2 / 3.0f);
    view.setShape(shape, deflScale);
    QApplication::processEvents();
    QThread::msleep(40);
    QApplication::processEvents();

    QImage fb = view.grabFramebuffer();
    if (!fb.save(QString::fromStdString(outPath))) {
        printf("save failed: %s\n", outPath.c_str());
        return 1;
    }
    printf("wrote %s (%dx%d)\n", outPath.c_str(), fb.width(), fb.height());
    return 0;
}
