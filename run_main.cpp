// ============================================================
// boolrun — occt-booltest 纯语料回放器 (pure corpus runner)
//
// 输入就是已归档的测试用例：任何同时包含 body1.step 和
// body3.step 的目录（booltest / boolstep 每轮迭代都会归档这样
// 一对文件；失败用例目录另带 info.txt 记录当时的错误）。
// 不造型、不拷贝、不旋转、不写出任何几何——每条用例只做三件事：
//
//     读入 STP ×2 → 布尔 ∩ → 记录测试状态
//
// 状态判定复用与生成端完全相同的校验（intersectSolids 错误 +
// volumePlausibilityError 合理性），因此回放结果与归档时的成败
// 具备直接可比性：info.txt 里有 status: FAILED 的用例预期复现
// 失败，其余预期通过；汇总时逐条对照，不一致即回归。
//
// usage:
//   boolrun <root>... [--out dir] [--repeat N] [--max-faces N]
//
// root 可以是用例目录本身，也可以是递归扫描的树根
// （如 output_step/iterations 或解包后的语料目录）。
// --repeat N 同一进程内重复回放每条用例 N 次，用于观察
// README 记载的跨运行非确定性；结果不一致记为 FLAKY。
// --max-faces N 读入后 face 数超过 N 的用例记为 SKIP 不做布尔
// （与 boolstep 的 face-explosion guard 同口径，0 = 不设限）。
// 退出码：0 = 全部与预期一致；1 = 存在回归/FLAKY/读取失败；
// 2 = 用法错误。
// ============================================================

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>

#include <TopAbs_ShapeEnum.hxx>

#include "common.hpp"

namespace fs = std::filesystem;

// ---- per-run record ----
struct RunOut {
    int run = 0;
    std::string status;              // success / failed / read-fail / skipped
    double V = 0, vin1 = 0, vin3 = 0;
    int faces = 0, edges = 0;
    double sec = 0;
    std::string err;
};

// ---- per-case record ----
struct CaseRec {
    std::string name;                // relative to its scan root
    std::string dir;
    std::string expected;            // "FAILED" (info.txt) / "PASS" (default)
    std::string expectedErr;         // error recorded in info.txt, if any
    std::vector<RunOut> runs;
    double sec = 0;                  // total wall time across runs

    std::string status() const       // aggregated across runs
    {
        if (runs.empty()) return "NO-RUNS";
        auto map = [](const std::string& s) {
            if (s == "success") return "PASS";
            if (s == "failed")  return "FAIL";
            if (s == "read-fail") return "READ-FAIL";
            return "SKIP";
        };
        std::string first = map(runs[0].status);
        for (const auto& r : runs)
            if (map(r.status) != first) return "FLAKY";
        return first;
    }
    bool matches() const             // vs recorded expectation
    {
        std::string st = status();
        if (expected == "FAILED") return st == "FAIL";
        return st == "PASS";         // default expectation: PASS
    }
};

// ---- discover case dirs (containing body1.step + body3.step) ----
static bool isCaseDir(const fs::path& d)
{
    std::error_code ec;
    return fs::exists(d / "body1.step", ec) && fs::exists(d / "body3.step", ec);
}

static std::vector<CaseRec> discoverCases(const std::vector<fs::path>& roots)
{
    std::set<std::string> seen;      // dedup by canonical path
    std::vector<CaseRec> cases;
    auto addCase = [&](const fs::path& dir, const fs::path& root) {
        std::error_code ec;
        fs::path canon = fs::weakly_canonical(dir, ec);
        std::string key = ec ? dir.string() : canon.string();
        if (!seen.insert(key).second) return;
        CaseRec c;
        c.dir = key;
        fs::path rel = fs::relative(dir, root, ec);
        c.name = (!ec && rel.generic_string() != ".") ? rel.generic_string()
                                                      : dir.filename().string();
        cases.push_back(std::move(c));
    };
    for (const auto& root : roots) {
        std::error_code ec;
        if (isCaseDir(root)) { addCase(root, root); continue; }
        fs::recursive_directory_iterator it(root, fs::directory_options::skip_permission_denied, ec);
        if (ec) { printf("warn: cannot scan '%s': %s\n", root.string().c_str(), ec.message().c_str()); continue; }
        for (auto end = fs::recursive_directory_iterator(); it != end; it.increment(ec)) {
            if (ec) break;
            std::error_code ec2;
            if (it->is_directory(ec2) && !ec2 && isCaseDir(it->path()))
                addCase(it->path(), root);
        }
    }
    std::sort(cases.begin(), cases.end(),
              [](const CaseRec& a, const CaseRec& b) { return a.dir < b.dir; });
    return cases;
}

// ---- expectation from archived info.txt (failed cases) ----
static std::string trim(const std::string& s)
{
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

static void readExpectation(CaseRec& c)
{
    c.expected = "PASS";             // default: success dirs have no info.txt
    std::ifstream f(fs::path(c.dir) / "info.txt");
    if (!f) return;
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("status:", 0) == 0)
            c.expected = trim(line.substr(7));
        else if (line.rfind("error:", 0) == 0)
            c.expectedErr = trim(line.substr(6));
    }
    if (c.expected != "FAILED") c.expected = "PASS";
}

// ---- one replay run of one case: read ×2 → boolean → status ----
static RunOut replayOnce(int rep, const fs::path& p1, const fs::path& p3, long maxFaces)
{
    RunOut r; r.run = rep;
    auto t0 = std::chrono::steady_clock::now();

    bool ok1 = false, ok3 = false;
    TopoDS_Shape b1 = readSTEP(p1.string(), ok1);
    TopoDS_Shape b3;
    if (ok1) b3 = readSTEP(p3.string(), ok3);
    if (!ok1 || !ok3) {
        r.status = "read-fail";
        r.err = std::string("cannot read ") + (!ok1 ? p1.string() : p3.string());
    } else {
        int nf = countSub(b1, TopAbs_FACE);
        if (maxFaces > 0 && nf > maxFaces) {
            r.status = "skipped";
            r.err = "face count " + std::to_string(nf) + " > max-faces "
                  + std::to_string(maxFaces);
        } else {
            r.vin1 = shapeVolume(b1);
            r.vin3 = shapeVolume(b3);
            std::string errMsg;
            TopoDS_Shape b4 = intersectSolids(b1, b3, errMsg);
            if (errMsg.empty() && b4.IsNull())
                errMsg = "intersect returned null without error";
            if (errMsg.empty()) {
                r.V = shapeVolume(b4);
                errMsg = volumePlausibilityError(r.V, r.vin1, r.vin3);
            }
            if (!errMsg.empty()) {
                r.status = "failed";
                r.err = errMsg;
            } else {
                r.status = "success";
                r.faces = countSub(b4, TopAbs_FACE);
                r.edges = countSub(b4, TopAbs_EDGE);
            }
        }
    }
    r.sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return r;
}

int main(int argc, char* argv[])
{
    std::vector<fs::path> roots;
    std::string outDir = "output_boolrun";
    long repeat = 1, maxFaces = 4000;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
        if (a == "--out") outDir = next();
        else if (a == "--repeat") repeat = atol(next().c_str());
        else if (a == "--max-faces") maxFaces = atol(next().c_str());
        else if (a == "--help" || a == "-h") {
            printf("usage: boolrun <root>... [--out dir] [--repeat N] [--max-faces N]\n"
                   "  root: a case dir (body1.step + body3.step) or a tree to scan\n");
            return 0;
        }
        else if (!a.empty() && a[0] == '-') {
            printf("unknown option '%s'\n", a.c_str());
            return 2;
        }
        else roots.push_back(a);
    }
    if (roots.empty()) {
        printf("missing <root>...\n"
               "usage: boolrun <root>... [--out dir] [--repeat N] [--max-faces N]\n"
               "a case = any dir containing body1.step + body3.step\n");
        return 2;
    }
    if (repeat < 1) repeat = 1;

    std::vector<CaseRec> cases = discoverCases(roots);
    fs::create_directories(outDir);

    printf("== boolrun (pure corpus runner) ==\n");
    printf("roots:");
    for (auto& r : roots) printf(" %s", r.string().c_str());
    printf("\ncases: %zu  repeat: %ld  max-faces: %ld%s\n",
           cases.size(), repeat, maxFaces, maxFaces == 0 ? " (off)" : "");

    int done = 0;
    for (auto& c : cases) {
        readExpectation(c);
        for (long rep = 1; rep <= repeat; ++rep) {
            RunOut r = replayOnce((int)rep,
                                  fs::path(c.dir) / "body1.step",
                                  fs::path(c.dir) / "body3.step",
                                  maxFaces);
            c.sec += r.sec;
            c.runs.push_back(std::move(r));
        }
        done++;

        std::string st = c.status();
        bool ok = c.matches();
        printf("[%3d/%zu] %-9s %7.2fs  %s%s\n",
               done, cases.size(), st.c_str(), c.sec, c.name.c_str(),
               ok ? "" : "  <= MISMATCH");
        if (st == "FAIL" || st == "READ-FAIL" || st == "SKIP" || st == "FLAKY") {
            printf("         err: %s\n", c.runs.back().err.c_str());
            if (!c.expectedErr.empty() && c.expectedErr != c.runs.back().err)
                printf("         archived: %s\n", c.expectedErr.c_str());
        }
    }

    // ---- totals ----
    int nPass = 0, nFail = 0, nFlaky = 0, nSkip = 0, nReadFail = 0, nOther = 0;
    std::vector<const CaseRec*> mismatches;
    double totalSec = 0;
    for (const auto& c : cases) {
        std::string st = c.status();
        if (st == "PASS") nPass++;
        else if (st == "FAIL") nFail++;
        else if (st == "FLAKY") nFlaky++;
        else if (st == "SKIP") nSkip++;
        else if (st == "READ-FAIL") nReadFail++;
        else nOther++;
        totalSec += c.sec;
        if (!c.matches()) mismatches.push_back(&c);
    }

    printf("\n==== SUMMARY ====\n");
    printf("cases: %zu  (PASS %d, FAIL %d, FLAKY %d, SKIP %d, READ-FAIL %d%s)\n",
           cases.size(), nPass, nFail, nFlaky, nSkip, nReadFail,
           nOther ? ", other" : "");
    printf("total boolean+io time: %.1fs\n", totalSec);
    if (mismatches.empty()) {
        printf("expectation check: all %zu cases match archived outcome\n", cases.size());
    } else {
        printf("expectation check: %zu MISMATCH(ES):\n", mismatches.size());
        for (const auto* c : mismatches)
            printf("  - %s: expected %s, got %s (%s)\n", c->name.c_str(),
                   c->expected.c_str(), c->status().c_str(),
                   c->runs.back().err.c_str());
    }

    // ---- summary.json ----
    {
        std::ofstream json(outDir + "/boolrun_summary.json");
        json << "{\n  \"mode\": \"boolrun\",\n  \"roots\": [";
        for (size_t i = 0; i < roots.size(); i++)
            json << (i ? ", " : "") << "\"" << jesc(roots[i].string()) << "\"";
        json << "],\n  \"repeat\": " << repeat
             << ",\n  \"max_faces\": " << maxFaces
             << ",\n  \"totals\": {\"cases\": " << cases.size()
             << ", \"pass\": " << nPass << ", \"fail\": " << nFail
             << ", \"flaky\": " << nFlaky << ", \"skip\": " << nSkip
             << ", \"read_fail\": " << nReadFail
             << ", \"mismatches\": " << mismatches.size() << "}"
             << ",\n  \"cases\": [\n";
        for (size_t i = 0; i < cases.size(); i++) {
            const auto& c = cases[i];
            json << "    {\"case\": \"" << jesc(c.name)
                 << "\", \"dir\": \"" << jesc(c.dir)
                 << "\", \"expected\": \"" << c.expected
                 << "\", \"status\": \"" << c.status()
                 << "\", \"match\": " << (c.matches() ? "true" : "false")
                 << ", \"sec\": " << std::setprecision(3) << c.sec
                 << ", \"runs\": [";
            for (size_t k = 0; k < c.runs.size(); k++) {
                const auto& r = c.runs[k];
                json << (k ? ", " : "")
                     << "{\"run\": " << r.run
                     << ", \"status\": \"" << r.status << "\""
                     << ", \"volume\": " << std::setprecision(10) << r.V
                     << ", \"vin1\": " << std::setprecision(10) << r.vin1
                     << ", \"vin3\": " << std::setprecision(10) << r.vin3
                     << ", \"faces\": " << r.faces
                     << ", \"edges\": " << r.edges
                     << ", \"sec\": " << std::setprecision(3) << r.sec;
                if (!r.err.empty())
                    json << ", \"error\": \"" << jesc(r.err) << "\"";
                json << "}";
            }
            json << "]}";
            if (i + 1 < cases.size()) json << ",";
            json << "\n";
        }
        json << "  ]\n}\n";
    }

    // ---- summary.csv ----
    {
        std::ofstream csv(outDir + "/boolrun_summary.csv");
        csv << "case,status,expected,match,pass_runs,fail_runs,skip_runs,"
               "volume,vin1,vin3,faces,edges,sec,error,dir\n";
        for (const auto& c : cases) {
            int pR = 0, fR = 0, sR = 0;
            std::string lastErr;
            double V = 0, v1 = 0, v3 = 0;
            int faces = 0, edges = 0;
            for (const auto& r : c.runs) {
                if (r.status == "success") { pR++; V = r.V; v1 = r.vin1; v3 = r.vin3;
                                             faces = r.faces; edges = r.edges; }
                else if (r.status == "failed") fR++;
                else sR++;
                if (!r.err.empty()) lastErr = r.err;
            }
            csv << c.name << "," << c.status() << "," << c.expected << ","
                << (c.matches() ? "yes" : "NO") << ","
                << pR << "," << fR << "," << sR << ","
                << std::setprecision(10) << V << ","
                << std::setprecision(10) << v1 << ","
                << std::setprecision(10) << v3 << ","
                << faces << "," << edges << ","
                << std::setprecision(3) << c.sec << ",\""
                << lastErr << "\",\"" << c.dir << "\"\n";
        }
    }

    printf("wrote %s/boolrun_summary.json, boolrun_summary.csv\n", outDir.c_str());
    return mismatches.empty() ? 0 : 1;
}
