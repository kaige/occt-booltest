#!/bin/bash
# Package the benchmark corpus for publication.
set -e
cd ~/Projects/occt-booltest
mkdir -p corpus
rm -f corpus/*.zip corpus/SHA256SUMS

# ---- 1. index files into git-friendly location ----
mkdir -p corpus-index
cp output_step/all_iterations.csv corpus-index/all_iterations.csv
cp output_step/summary.csv corpus-index/summary.csv
cp output_step/summary.json corpus-index/summary.json
cp output_step/bottle.step corpus-index/bottle.step

# ---- 2. core zip: everything needed to understand + rerun key vectors ----
# bottle + summaries + pngs + ALL failure archives + first iteration of every case
CORE=/tmp/corpus_core_staging
rm -rf $CORE && mkdir -p $CORE
cp output_step/bottle.step output_step/all_iterations.csv output_step/summary.csv output_step/summary.json $CORE/
cp -R output_step/png $CORE/png
for d in output_step/iterations/*/; do
  case=$(basename $d)
  # failures: *_failed dirs
  for f in $d*_failed; do [ -d "$f" ] && cp -R "$f" "$CORE/$(basename $f)"; done
  # first success iteration
  for f in $d*_success; do
    if [ -d "$f" ] && [ "$(basename $f)" = "it001_success" ]; then cp -R "$f" "$CORE/${case}_it001_success"; fi
  done
done
(cd $CORE && zip -q -r ~/Projects/occt-booltest/corpus/booltest-corpus-step-v1-core.zip .)
rm -rf $CORE

# ---- 3. full zip: the whole step-mode corpus ----
(cd output_step && zip -q -r ../corpus/booltest-corpus-step-v1-full.zip .)

# ---- 4. checksums ----
(cd corpus && shasum -a 256 *.zip > SHA256SUMS)
ls -lh corpus/
