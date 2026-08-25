// ============================================================
// mkbottle — 用造型方法（OCCT 教程瓶子）预先准备 STEP 模式
// 的输入模型。boolstep 的输入就是这个 STEP 文件，运行期不再
// 调用任何造型代码。
//
// usage: mkbottle [output.step]   (default: bottle.step)
// ============================================================
#include <cstdio>
#include "common.hpp"

int main(int argc, char* argv[])
{
    const char* out = (argc > 1) ? argv[1] : "bottle.step";
    printf("modeling bottle (MakeBottle 50 x 70 x 30)...\n");
    TopoDS_Shape bottle = MakeBottle(50., 70., 30.);
    const double V = shapeVolume(bottle);
    if (!writeSTEP(bottle, out)) {
        printf("FATAL: cannot write %s\n", out);
        return 1;
    }
    printf("wrote %s  (V = %.3f mm^3)\n", out, V);
    return 0;
}
