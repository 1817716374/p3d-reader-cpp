#pragma once
#include "internal.hpp"
namespace p3d::loft_detail {
using H = std::array<double, 4>;
// Homogeneous working storage; check() enforces the narrower loft contract.
struct Curve {
    unsigned degree = 1;
    bool rational = false;
    std::vector<double> knots;
    std::vector<H> poles;
    static Curve from_bspline(const BsplineCurve &, unsigned limit);
    // Isocurve storage can retain zero W; Coons working curves must deweight.
    void check(unsigned limit, bool require_cartesian = true) const;
    void insert(double t, unsigned multiplicity, unsigned limit);
    void elevate(unsigned degree, unsigned limit);
    double polygon_length() const;
    double knot_tolerance() const;
    Json table() const;
};
Point3 cartesian(H);
Curve open_periodic(const BsplineCurve &, unsigned limit, Json *report = nullptr);
// Boundary opening retains native weights, discontinuities and special-seam domains.
Curve open_periodic_boundary(const BsplineCurve &, unsigned limit, Json *report = nullptr);
struct CurveClosure {
    Curve curve;
    bool success = false, closed = false;
    Json report;
};
// Native closure for normalized, clamped, continuous working curves. Retains
// homogeneous controls (including zero W); does not reopen a periodic result.
CurveClosure close_normalized_curve(Curve, unsigned limit);
// Full curve-close entry: preserves already-closed/two-pole copies, normalizes
// only after endpoint agreement, and permits non-clamped/discontinuous storage.
CurveClosure close_native_curve(const BsplineCurve &, unsigned limit);
Curve close_reopen(Curve, unsigned limit, Json &report);
void compatible(std::vector<Curve *> curves, unsigned limit);
Curve append(Curve a, Curve b, bool length_weighted, unsigned limit);
} // namespace p3d::loft_detail
