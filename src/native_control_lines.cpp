#include "native_control_lines.hpp"
namespace p3d::swept_detail {
namespace {
double finite(double x) {
    require(std::isfinite(x), "native control-line pair nonfinite arithmetic");
    return x;
}
Point3 subtract(const Point3 &a, const Point3 &b) {
    return {finite(a[0] - b[0]), finite(a[1] - b[1]), finite(a[2] - b[2])};
}
double dot(const Point3 &a, const Point3 &b) {
    return finite((a[1] * b[1] + a[0] * b[0]) + a[2] * b[2]);
}
int location(double f, double tolerance) {
    if (-tolerance > f || f > 1 + tolerance)
        return 0;
    if (tolerance > f)
        return 1;
    return 1 - tolerance > f ? 2 : 4;
}
} // namespace
NativeControlLinePair native_control_line_pair(const Point3 &a0, const Point3 &a1, const Point3 &b0,
                                               const Point3 &b1, double tolerance) {
    require(std::isfinite(tolerance) && tolerance >= 0,
            "native line tolerance must be finite and nonnegative");
    const auto aa = subtract(a1, a0), bb = subtract(b1, b0), cc = subtract(b0, a0);
    const double a = dot(aa, cc), b = dot(aa, aa), c = dot(aa, bb), d = dot(bb, cc),
                 f = dot(bb, bb), denominator = finite(f * b - c * c);
    NativeControlLinePair out;
    if (!(std::abs(denominator) > 1e-12))
        return out;
    out.first_fraction = finite((f * a - d * c) / denominator);
    out.second_fraction = finite((c * a - d * b) / denominator);
    for (unsigned k = 0; k < 3; ++k) {
        out.first[k] = finite(out.first_fraction * aa[k] + a0[k]);
        out.second[k] = finite(out.second_fraction * bb[k] + b0[k]);
    }
    out.first_location = location(out.first_fraction, finite(tolerance / std::sqrt(b)));
    out.second_location = location(out.second_fraction, finite(tolerance / std::sqrt(f)));
    out.success = true;
    return out;
}
} // namespace p3d::swept_detail
