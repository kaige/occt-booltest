#!/bin/bash
# corpus size analysis
cd ~/Projects/occt-booltest
echo "== 总量 =="
du -sh output_step/ | awk '{print $1}'
echo "== 按文件类型 =="
for t in body1 body3 body4; do
  sz=$(find output_step/iterations -name "$t.step" -print0 | xargs -0 stat -f%z | awk '{s+=$1} END {printf "%.2f GB", s/1e9}')
  n=$(find output_step/iterations -name "$t.step" | wc -l | tr -d ' ')
  echo "  $t.step: $n 个, $sz"
done
echo "== 失败归档(brep+info+png) =="
find output_step/iterations -name "*.brep" -print0 | xargs -0 stat -f%z | awk '{s+=$1} END {printf "  brep: %.1f MB\n", s/1e6}'
echo "== 顶层杂项 =="
du -sh output_step/png output_step/bottle.step output_step/summary.* output_step/all_iterations.csv 2>/dev/null
echo "== 最大单个文件 TOP5 =="
find output_step -name "*.step" -exec stat -f "%z %N" {} \; | sort -rn | head -5 | awk '{printf "  %.0f MB  %s\n", $1/1e6, $2}'
echo "== 压缩率抽样（取3个大文件+20个小文件） =="
find output_step -name "*.step" -size +50M | head -3 > /tmp/zlist.txt
find output_step -name "*.step" -size -1M | head -20 >> /tmp/zlist.txt
raw=$(cat /tmp/zlist.txt | tr '\n' '\0' | xargs -0 stat -f%z | awk '{s+=$1} END {print s}')
zip -q -9 /tmp/zprobe.zip $(cat /tmp/zlist.txt | head -23) 2>/dev/null || tar -czf /tmp/zprobe.tgz -T /tmp/zlist.txt
[ -f /tmp/zprobe.zip ] && z=$(stat -f%z /tmp/zprobe.zip) || z=$(stat -f%z /tmp/zprobe.tgz)
awk -v r=$raw -v z=$z 'BEGIN {printf "  抽样 %d MB -> %d MB, 压缩到 %.0f%%\n", r/1e6, z/1e6, z/r*100}'
rm -f /tmp/zprobe.zip /tmp/zprobe.tgz /tmp/zlist.txt
