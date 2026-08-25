// ============================================================
// boolstep — occt-booltest STEP 模式（与原生造型模式完全分离）
//
// 输入就是一个 STEP 模型（例如用 mkbottle 预先以造型方法准备好的
// bottle.step）。每条测试都是严格的"文件到文件"链：
//
//     读入 STEP → 拷贝 → 旋转变换 → 布尔 ∩ → 写出 STEP（或报告失败）
//
// 成功写出的 body4.step 即下一轮迭代的输入文件，因此整条链路
// 完整压测 OCCT 的 STEP 读写回环 + 布尔内核；每轮迭代目录里的
// (body1.step, body3.step) 输入对是内核无关的基准语料。
//
// usage:
//   boolstep --in bottle.step [--out dir] [--axes x,y,z,diag]
//            [--thetas 1,2,5,...] [--max-iters 500] [--case-seconds 240]
// ============================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Trsf.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Transform.hxx>

#include "common.hpp"

namespace fs = std::filesystem;

// ---- per-iteration record (successes AND failures are logged) ----
struct IterRec {
    int iter = 0;
    bool ok = false;
    double V = 0, vinMin = 0;
    int faces = 0, edges = 0;
    double tSec = 0;
    std::string note, dir;
};

struct CaseResult {
    std::string axisName;
    double thetaDeg = 0;
    int itersDone = 0;
    std::string termReason;
    double finalVolume = 0, V0 = 0, durationSec = 0;
    int finalFaces = 0, finalEdges = 0;
    std::string failDetail;
    std::vector<IterRec> iterLog;
};

static gp_Dir axisDirOf(const std::string& name, bool& valid)
{
    valid = true;
    if (name == "x") return gp_Dir(1, 0, 0);
    if (name == "y") return gp_Dir(0, 1, 0);
    if (name == "z") return gp_Dir(0, 0, 1);
    if (name == "diag") return gp_Dir(1, 1, 1);
    valid = false;
    return gp_Dir(1, 0, 0);
}

int main(int argc, char* argv[])
{
    // ---- args ----
    std::string inPath, outDir = "output_step";
    std::vector<std::string> axes = {"x", "y", "z", "diag"};
    std::vector<double> thetas = {1, 2, 5, 15, 45, 90, 180};
    int maxIters = 500;
    double caseCapSec = 240.0;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
        if (a == "--in") inPath = next();
        else if (a == "--out") outDir = next();
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
        else {
            printf("unknown option '%s'\n"
                   "usage: boolstep --in <model.step> [--out dir] [--axes x,y,z,diag]\n"
                   "                 [--thetas 1,2,...] [--max-iters N] [--case-seconds S]\n",
                   a.c_str());
            return 2;
        }
    }
    if (inPath.empty()) {
        printf("missing --in <model.step>\n"
               "first prepare one with the modeling tool: ./mkbottle bottle.step\n");
        return 2;
    }

    fs::create_directories(outDir + "/iterations");

    // ---- V0: volume of the input model (read once, just for thresholds) ----
    bool okRead = false;
    TopoDS_Shape inputShape = readSTEP(inPath, okRead);
    if (!okRead) { printf("FATAL: cannot read input '%s'\n", inPath.c_str()); return 2; }
    const double V0 = shapeVolume(inputShape);
    fs::copy_file(inPath, outDir + "/input.step", fs::copy_options::overwrite_existing);

    printf("== boolstep (STEP mode) ==\n");
    printf("input: %s  (V0 = %.3f mm^3, threshold V0/1000 = %.3f)\n",
           inPath.c_str(), V0, V0 / 1000.0);
    printf("cases: %zu axes x %zu thetas, max %d iters/case, cap %.0fs/case\n",
           axes.size(), thetas.size(), maxIters, caseCapSec);

    // every case restarts from the archived input file
    const std::string inputFile = outDir + "/input.step";
    std::vector<CaseResult> results;

    for (auto& axName : axes) {
        bool validAxis = false;
        gp_Dir axisDir = axisDirOf(axName, validAxis);
        if (!validAxis) { printf("unknown axis '%s', skipped\n", axName.c_str()); continue; }

        for (double thetaDeg : thetas) {
            char cb[64];
            snprintf(cb, sizeof(cb), "axis%s_theta%g", axName.c_str(), thetaDeg);
            std::string caseName = cb;
            printf("\n>>> case %s\n", caseName.c_str());

            auto t0 = std::chrono::steady_clock::now();
            CaseResult res;
            res.axisName = axName;
            res.thetaDeg = thetaDeg;
            res.V0 = V0;
            std::string curFile = inputFile;   // 每条测试都从输入 STEP 开始
            const double thetaRad = thetaDeg * M_PI / 180.0;
            int iter = 0;

            while (true) {
                double elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t0).count();
                if (elapsed > caseCapSec) { res.termReason = "time-cap"; break; }
                if (iter >= maxIters)    { res.termReason = "max-iterations"; break; }
                iter++;

                fs::path itDir = fs::path(outDir) / "iterations" / caseName
                                 / ("it" + std::to_string(iter));
                fs::create_directories(itDir);

                // ---------- 1. 读入 STEP ----------
                TopoDS_Shape body1 = readSTEP(curFile, okRead);
                if (!okRead) {
                    res.termReason = "input-read-failed";
                    res.failDetail = "cannot read " + curFile;
                    break;
                }
                // face-explosion guard (perf, not an algorithm failure)
                if (countSub(body1, TopAbs_FACE) > 4000) {
                    res.termReason = "face-explosion-guard";
                    break;
                }
                // archive this iteration's input verbatim (self-contained pair)
                fs::copy_file(curFile, itDir / "body1.step",
                              fs::copy_options::overwrite_existing);

                // ---------- 2. 拷贝 ----------
                TopoDS_Shape body2 = BRepBuilderAPI_Copy(body1).Shape();

                // ---------- 3. 旋转变换（绕包围盒中心） ----------
                double diag;
                gp_Pnt center = bboxCenter(body1, diag);
                gp_Trsf rot;
                rot.SetRotation(gp_Ax1(center, axisDir), thetaRad);
                TopoDS_Shape body3 = BRepBuilderAPI_Transform(body2, rot).Shape();
                writeSTEP(body3, (itDir / "body3.step").string());

                // ---------- 4. 布尔 ∩ ----------
                const double vin1 = shapeVolume(body1);
                const double vin3 = shapeVolume(body3);
                std::string errMsg;
                TopoDS_Shape body4 = intersectSolids(body1, body3, errMsg);
                double V = body4.IsNull() ? 0.0 : shapeVolume(body4);
                if (errMsg.empty())
                    errMsg = volumePlausibilityError(V, vin1, vin3);

                // ---------- 5. 写出 STEP（或报告失败） ----------
                if (!errMsg.empty()) {
                    std::ofstream info(itDir / "info.txt");
                    info << "case: " << caseName << "\n"
                         << "failed at iteration: " << iter << "\n"
                         << "status: FAILED\n"
                         << "operation: boolean-intersect (BRepAlgoAPI_Common)\n"
                         << "axis: " << axName << "  theta: " << thetaDeg << " deg\n"
                         << "inputs: body1.step + body3.step (this directory)\n"
                         << "error: " << errMsg << "\n"
                         << "input volumes: Vin1=" << vin1 << "  Vin3=" << vin3 << "\n";
                    info.close();
                    fs::path itDirF = itDir; itDirF += "_failed";
                    fs::rename(itDir, itDirF);
                    IterRec rec;
                    rec.iter = iter; rec.ok = false; rec.vinMin = std::min(vin1, vin3);
                    rec.note = errMsg; rec.dir = itDirF.string();
                    rec.tSec = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - t0).count();
                    res.iterLog.push_back(rec);
                    res.termReason = "boolean-failed";
                    res.failDetail = errMsg;
                    printf("  it%d FAILED: %s (saved %s)\n",
                           iter, errMsg.c_str(), rec.dir.c_str());
                    break;
                }
                if (!writeSTEP(body4, (itDir / "body4.step").string())) {
                    res.termReason = "step-write-failed";
                    res.failDetail = "cannot write " + (itDir / "body4.step").string();
                    break;
                }

                res.itersDone = iter;
                res.finalVolume = V;
                res.finalFaces = countSub(body4, TopAbs_FACE);
                res.finalEdges = countSub(body4, TopAbs_EDGE);
                fs::path itDirS = itDir; itDirS += "_success";
                fs::rename(itDir, itDirS);
                IterRec rec;
                rec.iter = iter; rec.ok = true; rec.V = V; rec.vinMin = std::min(vin1, vin3);
                rec.faces = res.finalFaces; rec.edges = res.finalEdges;
                rec.dir = itDirS.string();
                rec.tSec = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t0).count();
                res.iterLog.push_back(rec);
                printf("  it%3d  V=%12.3f  (%.2f%% of V0)  faces=%d  edges=%d  [%.1fs]\n",
                       iter, V, 100.0 * V / V0, res.finalFaces, res.finalEdges, rec.tSec);

                curFile = (itDirS / "body4.step").string();   // 链到下一轮
                if (V < V0 / 1000.0) { res.termReason = "volume-threshold"; break; }
            }

            if (res.termReason.empty()) res.termReason = "max-iterations";
            res.durationSec = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();

            // per-case iteration log
            {
                std::ofstream ic(fs::path(outDir) / "iterations" / (caseName + "_iterations.csv"));
                ic << "case,iteration,status,volume,vin_min,volume_pct_of_V0,faces,edges,duration_sec,dir\n";
                for (auto& q : res.iterLog)
                    ic << caseName << "," << q.iter << ","
                       << (q.ok ? "success" : "failed") << ","
                       << std::setprecision(10) << q.V << ","
                       << std::setprecision(10) << q.vinMin << ","
                       << std::setprecision(6) << 100.0 * q.V / V0 << ","
                       << q.faces << "," << q.edges << ","
                       << std::setprecision(3) << q.tSec << ","
                       << q.dir << "\n";
            }
            results.push_back(res);
            printf("  => %s: %d iters, term=%s, Vfinal=%.4f, %.1fs\n",
                   caseName.c_str(), res.itersDone, res.termReason.c_str(),
                   res.finalVolume, res.durationSec);
        }
    }

    // ---- summary ----
    std::ofstream json(outDir + "/summary.json");
    json << "{\n  \"mode\": \"step\",\n  \"input\": \"" << jesc(inPath)
         << "\",\n  \"V0\": " << V0 << ",\n  \"cases\": [\n";
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
             << ", \"failed_iterations\": " << badN << ", \"iterations\": [";
        for (size_t k = 0; k < r.iterLog.size(); k++) {
            auto& q = r.iterLog[k];
            json << (k ? ", " : "")
                 << "{\"it\": " << q.iter
                 << ", \"status\": \"" << (q.ok ? "success" : "failed") << "\""
                 << ", \"volume\": " << std::setprecision(10) << q.V
                 << ", \"dir\": \"" << jesc(q.dir) << "\"}";
        }
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
        csv << "step,axis" << r.axisName << "_theta" << r.thetaDeg << ","
            << r.axisName << "," << r.thetaDeg << ","
            << r.itersDone << "," << r.termReason << ","
            << std::setprecision(10) << r.finalVolume << ","
            << std::setprecision(3) << r.durationSec << ","
            << r.finalFaces << "," << r.finalEdges << ","
            << okN << "," << badN << "\n";
    }
    csv.close();

    printf("\n==== SUMMARY ====\n");
    printf("%-24s %5s %5s  %-22s %14s %8s\n",
           "case", "iters", "fails", "termination", "final_volume", "sec");
    for (auto& r : results) {
        char cn[64];
        snprintf(cn, sizeof(cn), "axis%s_theta%g", r.axisName.c_str(), r.thetaDeg);
        int badN = 0;
        for (auto& q : r.iterLog) if (!q.ok) badN++;
        printf("%-24s %5d %5d  %-22s %14.4f %8.1f\n", cn, r.itersDone, badN,
               r.termReason.c_str(), r.finalVolume, r.durationSec);
    }
    printf("wrote %s/summary.json, summary.csv, iterations/\n", outDir.c_str());
    return 0;
}
