// SPDX-License-Identifier: Apache-2.0
// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// Adapted from imodel-native BsplineCurveFit.cpp and MSBsplineCurve_Modify.cpp.
// Changes: specialize the verified P3D degree-one LineString route; preserve
// native tolerances, immutable input, source indices and bounded work/storage.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_tube.hpp"
#include <numeric>

namespace p3d::swept_detail {
namespace {
constexpr double knot_tolerance = 1e-10, fit_tolerance = 1e-5, huge = 1e37;
void charge(TubeBudget &b, std::size_t n) {
    require(b.work <= b.max_work && n <= b.max_work - b.work,
            "native LineString fit work budget exceeded");
    b.work += n;
}
double finite(double x) {
    require(std::isfinite(x), "native LineString fit nonfinite arithmetic");
    return x;
}
double distance(const Point3 &a, const Point3 &b) {
    const double x = finite(a[0] - b[0]), y = finite(a[1] - b[1]), z = finite(a[2] - b[2]);
    return finite(std::sqrt((x * x + y * y) + z * z));
}
// Native oneBasisFuncs (left convention), specialized to order two. Its
// near-right-support exclusion is retained even for strictly increasing knots.
double basis(int i, const std::vector<double> &u, double t) {
    const int n = int(u.size()) - 3;
    require(i >= 0 && i <= n, "native linear fit basis index");
    if (std::abs(t - u[1]) < knot_tolerance)
        return i == 0 ? 1. : 0.;
    if (std::abs(t - u[n + 1]) < knot_tolerance)
        return i == n ? 1. : 0.;
    if (t < u[i] || t > u[i + 2] || std::abs(t - u[i + 2]) < knot_tolerance)
        return 0;
    const bool a = (t > u[i] || std::abs(t - u[i]) < knot_tolerance) && t < u[i + 1];
    const bool b = (t > u[i + 1] || std::abs(t - u[i + 1]) < knot_tolerance) && t < u[i + 2];
    const double left = a ? finite((t - u[i]) / (u[i + 1] - u[i])) : 0.;
    const double right = b ? finite((u[i + 2] - t) / (u[i + 2] - u[i + 1])) : 0.;
    return finite(left + right);
}
double removal_bound(const std::vector<Point3> &p, const std::vector<double> &u, int r, int s) {
    require(s >= 1 && s <= 3 && r >= 2 && r < int(p.size()),
            "native linear fit removal support is not representable");
    if (s == 3)
        return 0; // Both native temporary-control indices are zero.
    if (s == 2)
        return distance(p[r - 2], p[r - 1]);
    const double a = finite((u[r] - u[r - 1]) / (u[r + 1] - u[r - 1])), b = 1 - a;
    Point3 combined{};
    for (unsigned k = 0; k < 3; ++k)
        combined[k] = finite(b * p[r - 2][k] + a * p[r][k]);
    return distance(p[r - 1], combined);
}
} // namespace

TubeCurve fit_tube_linestring(const std::vector<Point3> &points, TubeBudget &budget) {
    const auto count = points.size();
    require(count >= 2 && count <= budget.max_control_points && count <= INT32_MAX - 2,
            "native LineString fit point count/budget");
    for (unsigned i = 0; i < 12; ++i)
        charge(budget, count);
    for (const auto &p : points)
        for (double x : p)
            finite(x);
    std::vector<Point3> poles = points;
    std::vector<double> params(count, 0), errors(count, 0), trial(count, 0);
    params.back() = 1;
    std::vector<std::size_t> origins(count);
    std::iota(origins.begin(), origins.end(), 0);
    Json attempts = Json::array();
    std::vector<double> knots{0, 0, 1, 1};
    double length = 0;
    if (count > 2) {
        std::vector<double> distances(count, 0);
        for (std::size_t i = 1; i < count; ++i) {
            distances[i] = distance(points[i - 1], points[i]);
            length = finite(length + distances[i]);
        }
        if (length > fit_tolerance)
            for (std::size_t i = 1; i + 1 < count; ++i)
                params[i] = finite(params[i - 1] + distances[i] / length);
        // The order-two interpolation matrix has one diagonal per distinct
        // interior parameter. Duplicate parameters or endpoint snapping make
        // it singular; native fitting fails rather than deduplicating inputs.
        for (std::size_t i = 1; i + 1 < count; ++i)
            require(params[i] > params[i - 1] && params[i] < params[i + 1] &&
                        params[i] >= knot_tolerance && 1 - params[i] >= knot_tolerance,
                    "native LineString interpolation is singular");
        knots.clear();
        knots.push_back(0);
        knots.insert(knots.end(), params.begin(), params.end());
        knots.push_back(1);
        // Preserve nonZeroBasisFuncs' reciprocal-then-multiply rounding and
        // the diagonal band solve; algebraic cancellation would change poles.
        for (std::size_t i = 1; i + 1 < count; ++i) {
            const double gap = params[i + 1] - params[i];
            const double diagonal = finite(gap * finite(1 / gap));
            const double factor = finite(1 / diagonal);
            for (auto &x : poles[i])
                x = finite(x * factor);
        }
        int n = int(count) - 1, mu = n;
        std::vector<double> bounds(count + 2, huge);
        std::vector<int> multiplicity(count + 2, 0), rejected(count + 2, 0), left(count),
            right(count);
        for (int r = 2; r <= n; ++r) {
            const int begin = r;
            while (r <= n && knots[r + 1] - knots[r] < knot_tolerance)
                ++r;
            multiplicity[r] = r - begin + 1;
            bounds[r] = removal_bound(poles, knots, r, multiplicity[r]);
            require(bounds[r] < huge, "native linear fit error bound exceeds native sentinel");
        }
        int j = 1;
        for (int i = 0; i <= n; ++i) {
            left[i] = j;
            while (j < mu && params[j] > knots[i] && params[j] <= knots[i + 1])
                ++j;
            int k = j;
            while (k < mu && params[k] < knots[i + 2])
                ++k;
            right[i] = k - 1;
        }
        while (n > 1) {
            // Linear scans, trial errors, vector shifts and bound updates are
            // charged before each attempt; rejected candidates stay rejected.
            for (unsigned k = 0; k < 8; ++k)
                charge(budget, count);
            int r = 2;
            for (int k = 3; k <= n; ++k)
                if (bounds[k] < bounds[r])
                    r = k;
            if (rejected[r])
                break;
            const int s = multiplicity[r];
            require(s >= 1 && s <= 3, "native linear fit invalid removal multiplicity");
            const double bound = bounds[r];
            const bool odd = (1 + s) % 2;
            const int i = r - (odd ? (2 + s) / 2 : (1 + s) / 2);
            const int lo = left[i], hi = right[i + (odd ? 1 : 0)];
            double alpha = 0, omb = 0, lam = 0, oml = 0;
            if (odd) {
                alpha = finite((knots[r] - knots[i]) / (knots[i + 2] - knots[i]));
                const double beta =
                    finite((knots[r] - knots[i + 1]) / (knots[i + 3] - knots[i + 1]));
                omb = 1 - beta;
                lam = finite(alpha / (alpha + beta));
                oml = 1 - lam;
            }
            bool removable = true;
            for (int k = lo; k <= hi; ++k) {
                double coefficient = basis(i, knots, params[k]);
                if (odd)
                    coefficient = std::abs(lam * alpha * coefficient -
                                           oml * omb * basis(i + 1, knots, params[k]));
                trial[k] = finite(errors[k] + coefficient * bound);
                if (trial[k] > fit_tolerance) {
                    removable = false;
                    break;
                }
            }
            // floor((2*r-s-1)/2), without overflowing signed native indices.
            const int removed = r - (s + 2) / 2;
            attempts.push_back({{"knot_index", r},
                                {"multiplicity", s},
                                {"bound", bound},
                                {"control_source_index", origins[removed]},
                                {"removed", removable}});
            if (!removable) {
                bounds[r] = huge;
                rejected[r] = 1;
                continue;
            }
            for (int k = lo; k <= hi; ++k)
                errors[k] = trial[k];
            // Degree one has no interior control-recovery iterations. For
            // near-multiple knots the native odd temporary is not written
            // back to the control array; retain that behavior as well.
            if (s > 1)
                multiplicity[r - 1] = s - 1;
            bounds.erase(bounds.begin() + r);
            rejected.erase(rejected.begin() + r);
            multiplicity.erase(multiplicity.begin() + r);
            knots.erase(knots.begin() + r);
            poles.erase(poles.begin() + removed);
            origins.erase(origins.begin() + removed);
            --n;
            if (n == 1)
                break;
            for (int k = std::max(r - 1, 2); k <= std::min(n, r + 1 - s); ++k)
                if (knots[k] != knots[k + 1] && !rejected[k]) {
                    bounds[k] = removal_bound(poles, knots, k, multiplicity[k]);
                    require(bounds[k] < huge,
                            "native linear fit error bound exceeds native sentinel");
                }
            for (int k = r - 2; k <= r - s; ++k) {
                int next = right[k] + 1;
                while (next <= mu && params[next] < knots[k + 2])
                    ++next;
                right[k] = next - 1;
            }
            for (int k = r - s + 1; k <= n; ++k) {
                left[k] = left[k + 1];
                right[k] = right[k + 1];
            }
        }
    }
    Json xyz = Json::array();
    for (const auto &p : poles)
        for (double x : p)
            xyz.push_back(x);
    auto result = BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                           {"order", 2},
                                           {"closed", false},
                                           {"poles", std::move(xyz)},
                                           {"weights", nullptr},
                                           {"knots", knots}});
    return {std::move(result),
            {{"scope", "native_swept_linestring_fit"},
             {"method", count == 2 ? "two_point_line" : "degree_one_fit"},
             {"fit_tolerance", fit_tolerance},
             {"source_parameters", params},
             {"source_polyline_length", count == 2 ? Json() : Json(length)},
             {"accumulated_errors", errors},
             {"attempts", std::move(attempts)},
             {"control_source_indices", origins},
             {"work_used", budget.work},
             {"source_geometry_reused", false}}};
}
} // namespace p3d::swept_detail
