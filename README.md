# occt-booltest

OpenCASCADE (OCCT 8) boolean intersection stress-test harness and
kernel-independent benchmark corpus.

## What it does

Per test case (`axis` × `theta`):

1. `body1` = the OCCT tutorial bottle (MakeBottle 50×70×30, V0 = 6960.775 mm³)
2. `body3` = `body1` rotated by `theta` about `axis` through its bbox center
3. `body4` = `BRepAlgoAPI_Common(body1 solids, body3 solids)`
4. on success `body1 ← body4`, repeat until boolean failure / volume <
   V0/1000 / 500 iterations / face-explosion guard (>4000 faces) / 300 s cap

Results are validated for physical plausibility: `0 < V ≤ min(Vin1, Vin3)`.
OCCT silently returns garbage (empty, negative, oversized results) in
near-degenerate configurations — those are recorded as failures with the
offending shapes archived.

## I/O modes

- `--io memory` (default): bodies handed between iterations in memory
  (lossless TopoDS_Shape handle assignment)
- `--io step`: every boolean input is read back from STEP — each round
  starts from the canonical `bottle.step`, each result `body4.step` is
  re-read as the next round's `body1`. Exercises serialization
  round-trips (STEP sewing/tolerance healing).

## Build & run

```bash
cmake -B build && cmake --build build
./build/booltest --out output --io step \
    --axes x,y,z,diag --thetas 1,2,5,15,45,90,180 \
    --max-iters 500 --case-seconds 300
```

Dependencies: OCCT (homebrew `opencascade`), Qt6 (offscreen screenshots
via QOpenGLWidget `grabFramebuffer`).

## Key findings (see corpus `summary.csv` for raw data)

- **Exact-coincidence geometry kills the boolean**: rotating about z
  (near-symmetry axis of the bottle) fails 7/7 in memory mode. A STEP
  round-trip (volume perturbation ~4e-8%) revives 4 of those 7 —
  "mathematically identical faces" are harder than "nearly identical".
- **Silent invalid results**: intersections larger than both inputs
  (up to 9.5× input volume), negative volumes, and near-zero slivers
  are returned with status OK. The plausibility check catches them.
- **Cross-run nondeterminism**: identical binaries + identical case can
  end differently in different processes (archived evidence in corpus).
- **STEP round-trip is a double-edged sword**: merges coplanar faces
  (4363 → 34 faces on axisx_theta15) and shrinks some results by >99%,
  but adds noise that breaks thin slivers that succeeded in memory.

## Benchmark corpus (STEP mode)

Published as GitHub Release assets (not in git):

| asset | contents |
|---|---|
| `booltest-corpus-step-v1-core.zip` | bottle.step, summaries, all failure archives (brep + info + png), first iteration of every case |
| `booltest-corpus-step-v1-full.zip` | everything: 2389 input pairs (body1+body3 per iteration), 2378 results (body4) |

To benchmark **your** kernel: read each iteration's `body1.step` +
`body3.step`, intersect, compare your volume against `body4.step` /
`all_iterations.csv`. `summary.csv` rows carry `io_mode` so memory-mode
and step-mode runs are never confused.

License: code MIT; corpus data CC-BY-4.0.
