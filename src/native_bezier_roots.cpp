// Copyright (c) Bentley Systems, Incorporated. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Adapted from bezroot.cpp, bezeval.cpp, quadeqn.cpp and bezierDPoint4d.cpp.
// Changes: native P3D tolerances/arithmetic, immutable inputs and bounded work.
// See THIRD_PARTY.md and third_party/BENTLEY_GEOMETRY_LICENSE.md.
#include "native_bezier_roots.hpp"

namespace p3d::curve_detail {
namespace {
double finite(double x) {
    require(std::isfinite(x), "native Bezier root nonfinite arithmetic");
    return x;
}
using Coefficients = std::vector<double>;
const std::array<std::array<double, 78>, 78> &pascal() {
    static const auto rows = [] {
        std::array<std::array<double, 78>, 78> a{};
        a[0][0] = 1;
        for (unsigned n = 1; n < 78; ++n) {
            a[n][0] = a[n][n] = 1;
            for (unsigned k = 1; k < n; ++k)
                a[n][k] = a[n - 1][k - 1] + a[n - 1][k];
        }
        return a;
    }();
    return rows;
}
// Native scalar evaluation uses explicit Horner forms through degree five,
// then a Pascal-row power sum. It is not the homogeneous de Casteljau kernel.
std::array<double, 2> evaluate(const Coefficients &a, double u, bool derivative, BezierWork work) {
    work.charge(16 * a.size());
    const std::size_t d = a.size() - 1;
    const double v = finite(1 - u);
    std::array<double, 78> vn{};
    vn[0] = 1;
    for (std::size_t i = 1; i <= d; ++i)
        vn[i] = finite(vn[i - 1] * v);
    double f = a[0], df = 0;
    if (d == 1) {
        f = finite(v * a[0] + u * a[1]);
        if (derivative)
            df = finite(a[1] - a[0]);
    } else if (d > 1 && d <= 5) {
        f = a[d];
        for (std::size_t j = d; j-- > 0;) {
            const double term =
                j == 0 ? finite(a[0] * vn[d]) : finite((pascal()[d][j] * a[j]) * vn[d - j]);
            f = finite(term + u * f);
        }
        if (derivative) {
            df = finite(a[d] - a[d - 1]);
            for (std::size_t j = d - 1; j-- > 0;) {
                const double delta = finite(a[j + 1] - a[j]);
                const double term = j == 0 ? finite(delta * vn[d - 1])
                                           : finite((pascal()[d - 1][j] * delta) * vn[d - 1 - j]);
                df = finite(term + u * df);
            }
            df = finite(double(d) * df);
        }
    } else if (d > 5) {
        f = finite(vn[d] * a[0]);
        double un = u;
        for (std::size_t i = 1; i < d; ++i) {
            f = finite(f + ((un * vn[d - i]) * pascal()[d][i]) * a[i]);
            un = finite(un * u);
        }
        f = finite(f + un * a[d]);
        if (derivative) {
            df = finite((a[1] - a[0]) * vn[d - 1]);
            un = u;
            for (std::size_t i = 1; i < d - 1; ++i) {
                df = finite(df +
                            ((un * vn[d - 1 - i]) * pascal()[d - 1][i]) * finite(a[i + 1] - a[i]));
                un = finite(un * u);
            }
            df = finite(double(d) * finite(df + un * finite(a[d] - a[d - 1])));
        }
    }
    return {f, df};
}
std::pair<Coefficients, Coefficients> split(const Coefficients &a, double u, BezierWork work) {
    work.charge(a.size() * a.size());
    auto left = a;
    Coefficients right(a.size());
    const auto d = a.size() - 1;
    const double v = finite(1 - u);
    right[d] = left[d];
    for (std::size_t j = 1; j <= d; ++j) {
        for (std::size_t k = d; k >= j; --k)
            left[k] = finite(v * left[k - 1] + u * left[k]);
        right[d - j] = left[d];
    }
    return {std::move(left), std::move(right)};
}
bool monotonic(double &root, Coefficients &a, BezierWork work) {
    work.charge(32 + a.size());
    const auto d = a.size() - 1;
    const double da = finite(a[d] - a[0]);
    const double da0 = finite(a[1] - a[0]), da1 = finite(a[d] - a[d - 1]);
    const double tolerance = finite(1e-22 * std::abs(da));
    if (finite(da * da0) < 0)
        a[1] = a[0];
    if (finite(da * da1) < 0)
        a[d - 1] = a[d];
    if (std::abs(a[0]) < tolerance) {
        root = 0;
        return true;
    }
    if (std::abs(a[d]) < tolerance) {
        root = 1;
        return true;
    }
    if (finite(a[d] * a[0]) > 0)
        return false;
    bool found = false;
    for (std::size_t i = 1; i <= d; ++i)
        if (finite(a[i - 1] * a[i]) <= 0) {
            const double numerator = -a[i - 1], denominator = finite(a[i] - a[i - 1]);
            const double fraction = std::abs(denominator) > 1e-15 * std::abs(numerator)
                                        ? finite(numerator / denominator)
                                        : 0.;
            root = finite((double(i - 1) + fraction) / double(d));
            found = true;
            break;
        }
    if (!found)
        return false;
    auto value = evaluate(a, root, true, work);
    if (std::abs(value[0]) < tolerance)
        return true;
    double low = a[0] < 0 ? 0. : 1., high = a[0] < 0 ? 1. : 0.;
    double du = 1, previous = 1;
    for (unsigned i = 0; i < 55; ++i) {
        work.charge(32);
        const double f = value[0], df = value[1];
        const double outside =
            finite(finite((root - high) * df - f) * finite((root - low) * df - f));
        const double saved = du;
        if (outside > 0 || std::abs(finite(2 * f)) > std::abs(finite(previous * df))) {
            du = finite(.5 * (high - low));
            root = finite(low + du);
        } else {
            du = finite(f / df);
            root = finite(root - du);
        }
        previous = saved;
        if (std::abs(du) < 1e-12)
            return true;
        value = evaluate(a, root, true, work);
        (value[0] < 0 ? low : high) = root;
    }
    return false;
}
bool interval(double &root, Coefficients &a, double low, double high, BezierWork work) {
    double local = 0;
    if (low == 0) {
        if (high == 1)
            return monotonic(root, a, work);
        auto halves = split(a, high, work);
        if (!monotonic(local, halves.first, work))
            return false;
        root = finite(local * high);
    } else {
        auto halves = split(a, low, work);
        if (high == 1) {
            if (!monotonic(local, halves.second, work))
                return false;
            root = finite(low + local * (1 - low));
        } else {
            const double fraction = finite((high - low) / (1 - low));
            auto inner = split(halves.second, fraction, work);
            if (!monotonic(local, inner.first, work))
                return false;
            root = finite(low + local * (high - low));
        }
    }
    return true;
}
Coefficients quadratic(const Coefficients &a, BezierWork work) {
    work.charge(64);
    const double a00 = a[0], a01 = finite(2 * a[1]), a11 = a[2];
    // Native binary uses 1e-14, unlike the newer upstream 1e-10.
    const double epsilon = finite(1e-14 * ((std::abs(a00) + std::abs(a01)) + std::abs(a11)));
    const double A = finite((a00 - a01) + a11);
    Coefficients roots;
    if (std::abs(A) > epsilon) {
        const double b2 = finite(a01 * a01);
        const double disc = finite(b2 - finite((4 * a00) * a11));
        const double dt = std::abs(finite(1e-14 * b2));
        const double B = finite(a01 - (a00 + a00));
        if (disc < -dt)
            return roots;
        if (disc > dt) {
            const double delta = std::sqrt(disc);
            if (B > 0) {
                const double q = finite((-B - delta) * .5);
                roots = {finite(q / A), finite(a00 / q)};
            } else {
                const double q = finite((delta - B) * .5);
                roots = {finite(a00 / q), finite(q / A)};
            }
        } else
            roots = {finite((-.5 * B) / A)};
    } else {
        const double B = finite(a00 - a11);
        if (std::abs(B) > epsilon)
            roots = {finite(a00 / B)};
    }
    roots.erase(std::remove_if(roots.begin(), roots.end(), [](double r) { return r < 0 || r > 1; }),
                roots.end());
    if (roots.size() == 2) {
        if (roots[0] == roots[1])
            roots.resize(1);
        else if (roots[0] > roots[1])
            std::swap(roots[0], roots[1]);
    }
    return roots;
}
bool roots(Coefficients &out, Coefficients &a, BezierWork work) {
    work.charge(4 * a.size());
    out.clear();
    const auto n = a.size(), d = n - 1;
    double minimum = a[0], maximum = a[0];
    std::size_t up = 0, down = 0, zero = 0;
    for (std::size_t i = 1; i < n; ++i) {
        minimum = std::min(minimum, a[i]);
        maximum = std::max(maximum, a[i]);
        const double delta = finite(a[i] - a[i - 1]);
        if (delta > 0)
            ++up;
        else if (delta < 0)
            ++down;
        else
            ++zero;
    }
    if (finite(minimum * maximum) > 0)
        return true;
    if (zero == d) {
        out.resize(n);
        out[0] = 0;
        out[d] = 1;
        for (std::size_t i = 1; i < d; ++i)
            out[i] = double(i) / double(d);
    } else if (n == 2) {
        out.push_back(a[0] == 0 ? 0. : a[1] == 0 ? 1. : finite(-a[0] / (a[1] - a[0])));
    } else if (n == 3) {
        out = quadratic(a, work);
    } else if (up == 0 || down == 0) {
        double root = 0;
        if (!monotonic(root, a, work))
            return false;
        out.push_back(root);
    } else {
        const double scale = finite(maximum - minimum);
        const double tol = finite(1e-8 * scale), tol1 = finite(1e-12 * scale);
        Coefficients derivative(d), split_roots;
        for (std::size_t i = 0; i < d; ++i)
            derivative[i] = finite(a[i + 1] - a[i]);
        // Native outer routine intentionally ignores derivative convergence status.
        roots(split_roots, derivative, work);
        if (split_roots.empty() || split_roots.back() < 1)
            split_roots.push_back(1);
        Coefficients values;
        for (double r : split_roots)
            values.push_back(evaluate(a, r, false, work)[0]);
        double low = 0, last = -1, f0 = a[0];
        for (std::size_t i = split_roots[0] == 0 ? 1 : 0;
             low < 1 && i < split_roots.size() && out.size() < n; ++i) {
            double high = split_roots[i], f1 = values[i];
            bool double_root = false;
            if (i + 1 < split_roots.size() && low < high && high < split_roots[i + 1]) {
                const double f2 = values[i + 1];
                if (finite(f0 * f2) > 0 && std::abs(f1) < (finite(f0 * f1) > 0 ? tol : tol1)) {
                    out.push_back(last = high);
                    high = split_roots[++i];
                    f1 = f2;
                    double_root = true;
                }
            }
            double root = 0;
            if (!double_root && interval(root, a, low, high, work) && root != last)
                out.push_back(last = root);
            low = high;
            f0 = f1;
        }
    }
    return true;
}
void endpoint(Coefficients &out, const Coefficients &a, bool end, BezierWork work) {
    work.charge(a.size() + out.size() + 16);
    if (out.size() >= a.size() - 1)
        return;
    double s = end ? 1. : 0.;
    const double low = end ? 1 - 1e-10 : 1e-10;
    const double high = end ? 1 + 1e-10 : -1e-10;
    for (double r : out)
        if (finite((r - low) * (r - high)) <= 0)
            return;
    const auto n = a.size();
    const double a0 = a[end ? n - 1 : 0], a1 = a[end ? n - 2 : 1];
    if (!(std::abs(a0) < 1e-10 * std::abs(a1)))
        return;
    for (unsigned i = 0; i < 5; ++i) {
        const auto v = evaluate(a, s, true, work);
        if (std::abs(v[0]) >= 1e-4 * std::abs(v[1]))
            return;
        const double step = finite(v[0] / v[1]);
        s = finite(s - step);
        // Native compares signed step, not abs(step).
        if (step < 2.5e-11) {
            if (finite((s - low) * (s - high)) <= 0)
                out.push_back(s);
            return;
        }
    }
}
} // namespace
std::vector<double> native_bezier_product(const std::vector<double> &a,
                                          const std::vector<double> &b, BezierWork work) {
    require(!a.empty() && !b.empty() && a.size() <= 78 && b.size() <= 78 &&
                a.size() + b.size() <= 79,
            "native Bezier product order exceeded");
    work.charge(8 * a.size() * b.size());
    for (double x : a)
        finite(x);
    for (double x : b)
        finite(x);
    const auto da = a.size() - 1, db = b.size() - 1, d = da + db;
    Coefficients out(d + 1);
    if (a.size() == 1) {
        for (std::size_t i = 0; i < b.size(); ++i)
            out[i] = finite(a[0] * b[i]);
    } else if (a.size() <= 4 && b.size() <= 4) {
        // Native explicit formulas prescale degree-2/3 middle ordinates.
        // The 3x3 convolution accumulates in descending A-index order.
        auto aa = a, bb = b;
        for (std::size_t i = 1; i < da; ++i)
            aa[i] = finite(pascal()[da][i] * aa[i]);
        for (std::size_t j = 1; j < db; ++j)
            bb[j] = finite(pascal()[db][j] * bb[j]);
        for (std::size_t k = 0; k <= d; ++k) {
            const auto low = k > db ? k - db : 0, high = std::min(k, da);
            const bool reverse = a.size() == 3 && b.size() == 3;
            double sum = 0;
            bool first = true;
            for (std::size_t step = 0; step <= high - low; ++step) {
                const auto i = reverse ? high - step : low + step;
                const double term = finite(aa[i] * bb[k - i]);
                sum = first ? term : finite(sum + term);
                first = false;
            }
            out[k] = finite(sum * (1. / pascal()[d][k]));
        }
    } else {
        for (std::size_t i = 0; i <= da; ++i)
            for (std::size_t j = 0; j <= db; ++j)
                out[i + j] =
                    finite(out[i + j] + ((pascal()[da][i] * pascal()[db][j]) * a[i]) * b[j]);
        for (std::size_t k = 0; k <= d; ++k)
            out[k] = finite(out[k] / pascal()[d][k]);
    }
    return out;
}
BezierRoots native_bezier_roots(const std::vector<double> &source, BezierWork work,
                                bool add_endpoint_roots) {
    require(source.size() >= 2 && source.size() <= 78, "native Bezier root order must be 2..78");
    work.charge(source.size());
    for (double x : source)
        finite(x);
    BezierRoots result;
    result.working_coefficients = source;
    result.success = roots(result.parameters, result.working_coefficients, work);
    if (add_endpoint_roots) {
        endpoint(result.parameters, result.working_coefficients, false, work);
        endpoint(result.parameters, result.working_coefficients, true, work);
    }
    return result;
}
BezierPlaneIntersections native_bezier_plane_intersections(const std::vector<BezierPole> &poles,
                                                           const BezierPole &plane,
                                                           std::size_t max_output,
                                                           BezierWork work) {
    require(poles.size() >= 2 && poles.size() <= 26,
            "native plane intersection order must be 2..26");
    work.charge(8 * poles.size());
    for (double x : plane)
        finite(x);
    Coefficients coefficients;
    for (const auto &p : poles) {
        for (double x : p)
            finite(x);
        coefficients.push_back(
            finite(((plane[0] * p[0] + plane[1] * p[1]) + plane[2] * p[2]) + plane[3] * p[3]));
    }
    const auto solutions = native_bezier_roots(coefficients, work, true);
    BezierPlaneIntersections out;
    out.all_parameters = solutions.parameters.size() == poles.size();
    if (!out.all_parameters)
        for (double u : solutions.parameters) {
            if (out.parameters.size() >= max_output)
                return out;
            work.charge(4 * poles.size() * poles.size());
            auto p = poles;
            const double v = finite(1 - u);
            for (std::size_t j = 1; j < p.size(); ++j)
                for (std::size_t k = p.size() - 1; k >= j; --k)
                    for (unsigned axis = 0; axis < 4; ++axis)
                        p[k][axis] = finite(v * p[k - 1][axis] + u * p[k][axis]);
            out.parameters.push_back(u);
            out.points.push_back(p.back());
        }
    // Plane wrapper ignores the scalar-root convergence status.
    out.success = true;
    return out;
}
} // namespace p3d::curve_detail
