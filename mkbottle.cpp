// ============================================================
// mkbottle — 用造型方法（OCCT 教程瓶子）预先准备 STEP 模式
// 的输入模型。boolstep 的输入就是这个 STEP 文件，运行期不再
// 调用任何造型代码。
//
// usage: mkbottle [output.step] [--solid] [--split body.step thread.step]
//   --split: write the two solids of the solid bottle (body / decorative
//   threading) as two separate single-solid STEP files instead of one
//   compound; combines with --solid.
// ============================================================
#include <cstdio>
#include <string>
#include <TopExp_Explorer.hxx>
#include "common.hpp"

int main(int argc, char* argv[])
{
    const char* out = "bottle.step";
    const char* splitBody = nullptr;
    const char* splitThread = nullptr;
    bool solid = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--solid") solid = true;
        else if (a == "--split" && i + 2 < argc) {
            splitBody = argv[++i];
            splitThread = argv[++i];
        }
        else out = argv[i];
    }
    printf("modeling bottle (%s 50 x 70 x 30)...\n",
           solid ? "MakeBottleSolid" : "MakeBottle");
    TopoDS_Shape bottle = solid ? MakeBottleSolid(50., 70., 30.)
                                : MakeBottle(50., 70., 30.);

    if (splitBody && splitThread) {
        TopoDS_Shape parts[2];
        int si = 0;
        for (TopExp_Explorer ex(bottle, TopAbs_SOLID); ex.More() && si < 2; ex.Next(), si++)
            parts[si] = ex.Current();
        if (si < 2) { printf("FATAL: expected 2 solids, found %d\n", si); return 1; }
        if (!writeSTEP(parts[0], splitBody) || !writeSTEP(parts[1], splitThread)) {
            printf("FATAL: split write failed\n");
            return 1;
        }
        printf("wrote %s  (body,  V = %.3f mm^3)\n", splitBody,  shapeVolume(parts[0]));
        printf("wrote %s  (thread, V = %.3f mm^3)\n", splitThread, shapeVolume(parts[1]));
        return 0;
    }

    const double V = shapeVolume(bottle);
    if (!writeSTEP(bottle, out)) {
        printf("FATAL: cannot write %s\n", out);
        return 1;
    }
    printf("wrote %s  (V = %.3f mm^3)\n", out, V);
    return 0;
}
