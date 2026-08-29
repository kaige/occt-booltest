// ============================================================
// occt-booltest — shared helpers (used by booltest / boolstep /
// mkbottle): the tutorial bottle model, geometry queries, STEP
// read/write, and the validated boolean intersection.
// ============================================================
#pragma once

#include <string>
#include <gp_Pnt.hxx>
#include <TopoDS_Shape.hxx>
#include <TopAbs_ShapeEnum.hxx>

// --- modeling: the OCCT tutorial bottle (verbatim) ---
TopoDS_Shape MakeBottle(double theWidth, double theHeight, double theThickness);
// Same geometry, body kept SOLID (no MakeThickSolidByJoin hollowing)
TopoDS_Shape MakeBottleSolid(double theWidth, double theHeight, double theThickness);

// --- geometry queries ---
double shapeVolume(const TopoDS_Shape& s);
int    countSub(const TopoDS_Shape& s, TopAbs_ShapeEnum t);
gp_Pnt bboxCenter(const TopoDS_Shape& s, double& diag);

// --- STEP file I/O ---
bool writeSTEP(const TopoDS_Shape& s, const std::string& path);
// returns null shape and ok=false on any failure
TopoDS_Shape readSTEP(const std::string& path, bool& ok);

// --- boolean intersection with validation ---
// Intersects the SOLIDs of a and b (flat solid lists — see note in
// common.cpp about the OCCT 8 compound pitfall). On failure returns a
// null shape and sets err. Does NOT check result volume plausibility;
// use volumePlausibilityError() on the caller side.
TopoDS_Shape intersectSolids(const TopoDS_Shape& a, const TopoDS_Shape& b,
                             std::string& err);

// An intersection of two solids must satisfy 0 < V <= min(Vin1, Vin3).
// Returns "" if plausible, else a human-readable violation message
// (OCCT silently returns garbage volumes in near-degenerate configs).
std::string volumePlausibilityError(double V, double vin1, double vin3);

// --- misc ---
std::string jesc(const std::string& s);   // JSON string escape
