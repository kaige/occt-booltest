// ============================================================
// mkbottle — 用造型方法（OCCT 教程瓶子）预先准备 STEP 模式
// 的输入模型。boolstep 的输入就是这个 STEP 文件，运行期不再
// 调用任何造型代码。
//
// usage: mkbottle [output.step] [--solid]
//   默认 = 教程瓶子（瓶身抽壳，壁厚 T/50）
//   --solid = 几何相同的实心瓶（不抽壳）
// ============================================================
#include <cstdio>
#include <string>
#include "common.hpp"

int main(int argc, char* argv[])
{
    const char* out = "bottle.step";
    bool solid = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--solid") solid = true;
        else out = argv[i];
    }
    printf("modeling bottle (%s 50 x 70 x 30)...\n",
           solid ? "MakeBottleSolid" : "MakeBottle");
    TopoDS_Shape bottle = solid ? MakeBottleSolid(50., 70., 30.)
                                : MakeBottle(50., 70., 30.);
    const double V = shapeVolume(bottle);
    if (!writeSTEP(bottle, out)) {
        printf("FATAL: cannot write %s\n", out);
        return 1;
    }
    printf("wrote %s  (V = %.3f mm^3)\n", out, V);
    return 0;
}
