#include "internal.hpp"

namespace p3d {
namespace {
double finite(double x) {
    require(std::isfinite(x), "B-spline native frame: non-finite arithmetic");
    return x;
}
double magnitude(const Point3 &p) {
    return finite(std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]));
}
Point3 cross(const Point3 &a, const Point3 &b) {
    return {finite(a[1] * b[2] - a[2] * b[1]), finite(a[2] * b[0] - a[0] * b[2]),
            finite(a[0] * b[1] - a[1] * b[0])};
}
// This path uses GePoint3d::normalize: a zero-length vector stays unchanged.
// GeVec3d::normalize has a different fallback and must not be substituted.
double normalize(Point3 &p) {
    const double length = magnitude(p);
    if (length > 0) {
        const double inverse = finite(1 / length);
        for (auto &x : p)
            x = finite(x * inverse);
    }
    return length;
}
Point3 polygon_normal(const std::vector<Point3> &p) {
    Point3 result{};
    const Point3 *previous = &p.back();
    for (const auto &current : p) {
        for (unsigned axis = 0; axis < 3; ++axis) {
            const unsigned a = (axis + 1) % 3, b = (axis + 2) % 3;
            result[axis] = finite(result[axis] +
                                  ((*previous)[a] - current[a]) * ((*previous)[b] + current[b]));
        }
        previous = &current;
    }
    return result;
}
} // namespace

Json BsplineCurve::native_frame_at(double fraction) const {
    auto evaluation = native_bspline_evaluate(*this, fraction, 3);
    const auto &h = evaluation.homogeneous;
    Point3 position{}, first{}, second{};
    if (rational()) {
        const double w = h[0][3], dw = h[1][3], ddw = h[2][3];
        require(w != 0, "B-spline native frame: zero evaluated weight");
        const double w2 = finite(w * w), w3 = finite(w2 * w);
        require(w2 != 0 && w3 != 0, "B-spline native frame: weight powers underflow");
        const double dw2 = finite(dw * dw), dw_w = finite(dw * w), ddw_w = finite(ddw * w);
        for (unsigned axis = 0; axis < 3; ++axis) {
            position[axis] = finite(h[0][axis] / w);
            first[axis] = finite((w * h[1][axis] - dw * h[0][axis]) / w2);
            // Preserve the native chain-rule expansion, not the separate
            // computeDerivatives interface's recursive quotient evaluation.
            second[axis] =
                finite((((h[0][axis] * 2) * dw2 - h[0][axis] * ddw_w - (h[1][axis] * 2) * dw_w) +
                        h[2][axis] * w2) /
                       w3);
        }
    } else {
        std::copy_n(h[0].begin(), 3, position.begin());
        std::copy_n(h[1].begin(), 3, first.begin());
        std::copy_n(h[2].begin(), 3, second.begin());
    }
    Point3 tangent = first, normal{0, 1, 0}, binormal{0, 0, 1};
    const double tangent_length = normalize(tangent);
    bool degenerate = magnitude(second) < 1e-5;
    if (!degenerate) {
        binormal = cross(first, second);
        degenerate = normalize(binormal) < 1e-5;
        normal = cross(binormal, tangent);
    }
    double curvature = 0;
    bool fallback = order() < 3 || degenerate;
    if (!fallback) {
        const double speed = magnitude(first);
        require(speed != 0, "B-spline native frame: zero curvature denominator");
        curvature = finite(((magnitude(cross(first, second)) / speed) / speed) / speed);
        fallback = curvature < 1e-12;
    }
    std::string method = "derivative_frame";
    if (fallback) {
        if (rational())
            for (std::size_t i = 0; i < evaluation.working_poles.size(); ++i) {
                const double inverse = finite(1 / weights()[i]);
                for (auto &x : evaluation.working_poles[i])
                    x = finite(x * inverse);
            }
        Point3 reference = polygon_normal(evaluation.working_poles);
        if (normalize(reference) < 1e-5) {
            reference = std::abs(tangent[0]) < .01 && std::abs(tangent[1]) < .01 ? Point3{0, 1, 0}
                                                                                 : Point3{0, 0, 1};
            method = "axis_fallback_frame";
        } else
            method = "control_polygon_frame";
        normal = cross(reference, tangent);
        normalize(normal);
        binormal = cross(tangent, normal);
    }
    Matrix4 frame{};
    for (unsigned axis = 0; axis < 3; ++axis) {
        frame[axis] = {tangent[axis], normal[axis], binormal[axis], position[axis]};
    }
    frame[3][3] = 1;
    const bool singular =
        magnitude(tangent) == 0 || magnitude(normal) == 0 || magnitude(binormal) == 0;
    return {{"status", "computed"},
            {"profile", "native_bspline_frenet_frame"},
            {"method", method},
            {"frame", frame},
            {"basis_status", singular ? "degenerate" : "nondegenerate"},
            {"tangent_magnitude", tangent_length},
            {"curvature", fallback ? 0. : curvature}};
}
} // namespace p3d
