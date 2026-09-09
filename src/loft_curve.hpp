#pragma once
#include "internal.hpp"
namespace p3d::loft_detail {
using H = std::array<double, 4>;
// Open, clamped, normalized curves. Full knot multiplicities are retained.
struct Curve {
    unsigned degree = 1;
    bool rational = false;
    std::vector<double> knots;
    std::vector<H> poles;
    static Curve from_bspline(const BsplineCurve &, unsigned limit);
    void check(unsigned limit) const;
    void insert(double t, unsigned multiplicity, unsigned limit);
    void elevate(unsigned degree, unsigned limit);
    double polygon_length() const;
    double knot_tolerance() const;
    Json table() const;
};
Point3 cartesian(H);
void compatible(std::vector<Curve *> curves, unsigned limit);
Curve append(Curve a, Curve b, bool length_weighted, unsigned limit);
} // namespace p3d::loft_detail
