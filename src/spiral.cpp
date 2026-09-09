#include "internal.hpp"
namespace p3d {
namespace {
constexpr double pi = 3.1415926535897932384626433832795;
double number(const Json &v) {
    require(v.is_number(), "spiral value is not numeric");
    const double x = v.get<double>();
    require(std::isfinite(x), "spiral value is not finite");
    return x;
}
double finite(double x) {
    require(std::isfinite(x), "spiral arithmetic overflow");
    return x;
}
} // namespace
TransitionSpiral TransitionSpiral::from_bgfb(const Json &table) {
    require(table.at("_type") == "TransitionSpiral", "expected BGFB TransitionSpiral table");
    const auto &d = table.at("detail");
    TransitionSpiral s;
    const auto &kind = d.at("spiralType");
    require(kind.is_number_integer() && kind >= 10 && kind <= 14,
            "native BGFB spiral factory only supports types 10 through 14");
    s.type_ = kind.get<int>();
    s.bearing_ = number(d.at("bearing0Radians"));
    const double end_bearing = number(d.at("bearing1Radians")),
                 delta = finite(end_bearing - s.bearing_), a = std::abs(number(d.at("curvature0"))),
                 b = std::abs(number(d.at("curvature1"))), average = finite(a + b) * .5;
    require(average >= std::abs(delta) * 1e-20, "native spiral bearing/curvature limits rejected");
    // Native factory determines both curvature signs from the bearing order,
    // does not wrap angles, and ignores the signs serialized for the curvatures.
    const double sign = end_bearing > s.bearing_ ? 1 : -1;
    s.curvature0_ = sign * a;
    s.curvature1_ = sign * b;
    require(average > 0, "native spiral length is undefined for zero mean curvature");
    s.length_ = finite(std::abs(delta / average));
    s.start_ = number(d.at("fractionA"));
    s.end_ = number(d.at("fractionB"));
    finite(s.end_ - s.start_);
    const std::array<const char *, 12> names{"axx", "axy", "axz", "axw", "ayx", "ayy",
                                             "ayz", "ayw", "azx", "azy", "azz", "azw"};
    for (unsigned i = 0; i < 12; ++i)
        s.transform_[i] = number(d.at("transform").at(names[i]));
    s.source_ = table;
    const std::array<const char *, 5> profiles{"linear", "cubic", "piecewise_quadratic", "cosine",
                                               "sine_corrected_linear"};
    s.report_ = {{"status", "valid"},
                 {"representation", "source_transition_spiral"},
                 {"curvature_profile", profiles[s.type_ - 10]},
                 {"effective_curvature0", s.curvature0_},
                 {"effective_curvature1", s.curvature1_},
                 {"local_length", s.length_},
                 {"active_fraction_range", {s.start_, s.end_}},
                 {"native_bspline_conversion", "not_evaluated"},
                 {"bgfb_inactive_fields",
                  Json::array({"detail/constructionHint", "extraData", "directDetail"})}};
    return s;
}
double TransitionSpiral::angle(double t) const {
    if (length_ == 0)
        return bearing_;
    const double delta = curvature1_ - curvature0_;
    double integral = 0;
    switch (type_) {
    case 10:
        integral = .5 * t * t;
        break;
    case 11:
        integral = t * t * t * (1 - .5 * t);
        break;
    case 12:
        if (t <= .5)
            integral = (2. / 3) * t * t * t;
        else {
            const double u = 1 - t;
            integral = .5 - u + (2. / 3) * u * u * u;
        }
        break;
    case 13:
        integral = .5 * t - std::sin(pi * t) / (2 * pi);
        break;
    case 14:
        integral = .5 * t * t + (std::cos(2 * pi * t) - 1) / (4 * pi * pi);
        break;
    }
    return finite(bearing_ + length_ * (curvature0_ * t + delta * integral));
}
double TransitionSpiral::curvature(double t) const {
    if (length_ == 0)
        t = 0;
    double blend = 0;
    switch (type_) {
    case 10:
        blend = t;
        break;
    case 11:
        blend = t * t * (3 - 2 * t);
        break;
    case 12:
        blend = t <= .5 ? 2 * t * t : 1 - 2 * (1 - t) * (1 - t);
        break;
    case 13:
        blend = .5 * (1 - std::cos(pi * t));
        break;
    case 14:
        blend = t - std::sin(2 * pi * t) / (2 * pi);
        break;
    }
    return finite(curvature0_ + (curvature1_ - curvature0_) * blend);
}
double TransitionSpiral::integrand_fourth_bound(double a, double b) const {
    // Bounds for g, g', g'', g''' on an interval, where k=k0+(k1-k0)g(t).
    // These also hold for active ranges outside [0,1]. Type12 is split at .5.
    const double r = std::max(std::abs(a), std::abs(b));
    double g = 0, g1 = 0, g2 = 0, g3 = 0;
    switch (type_) {
    case 10:
        g = r;
        g1 = 1;
        break;
    case 11:
        g = r * r * (3 + 2 * r);
        g1 = 6 * r * (1 + r);
        g2 = 6 + 12 * r;
        g3 = 12;
        break;
    case 12: {
        const double q = std::max({r, std::abs(1 - a), std::abs(1 - b)});
        g = 1 + 2 * q * q;
        g1 = 4 * q;
        g2 = 4;
        break;
    }
    case 13:
        g = 1;
        g1 = pi / 2;
        g2 = pi * pi / 2;
        g3 = pi * pi * pi / 2;
        break;
    case 14:
        g = r + 1 / (2 * pi);
        g1 = 2;
        g2 = 2 * pi;
        g3 = 4 * pi * pi;
        break;
    }
    const double d = std::abs(curvature1_ - curvature0_),
                 p = length_ * (std::abs(curvature0_) + d * g), q = length_ * d * g1,
                 u = length_ * d * g2, v = length_ * d * g3;
    // Norm of d^4/dt^4 (cos(theta),sin(theta)); triangle inequality on complex form.
    return finite(v + 4 * p * u + 3 * q * q + 6 * p * p * q + p * p * p * p);
}
SpiralEvaluation TransitionSpiral::evaluate(double fraction, double tolerance,
                                            unsigned budget) const {
    require(std::isfinite(fraction) && fraction >= 0 && fraction <= 1,
            "spiral fraction must be in [0,1]");
    require(std::isfinite(tolerance) && tolerance > 0 && budget > 0,
            "spiral integration tolerance/budget");
    const double t = fraction == 0   ? start_
                     : fraction == 1 ? end_
                                     : finite((1 - fraction) * start_ + fraction * end_);
    SpiralEvaluation out;
    out.local_bearing = angle(t);
    out.local_curvature = curvature(t);
    const double speed = finite(length_ * (end_ - start_));
    Point2 local{};
    // Frobenius norm bounds the affine XY linear map; translation has no effect
    // on quadrature truncation, but can still dominate floating-point roundoff.
    double frame = 0;
    for (unsigned k = 0; k < 3; ++k)
        frame = std::hypot(frame, std::hypot(transform_[4 * k], transform_[4 * k + 1]));
    const double scale = finite(length_ * frame);
    if (t != 0 && scale != 0) {
        struct Interval {
            double a, b;
            unsigned depth;
        };
        std::vector<Interval> stack;
        const double low = std::min(0., t), high = std::max(0., t), span = high - low;
        if (type_ == 12 && low < .5 && high > .5) {
            stack.push_back({.5, high, 0});
            stack.push_back({low, .5, 0});
        } else
            stack.push_back({low, high, 0});
        while (!stack.empty()) {
            const auto item = stack.back();
            stack.pop_back();
            const double h = item.b - item.a, middle = item.a + h * .5;
            const double error =
                finite(scale * integrand_fourth_bound(item.a, item.b) * (h * h * h * h * h / 2880));
            if (error > tolerance * (h / span)) {
                require(item.depth < 60 && middle > item.a && middle < item.b,
                        "spiral integration precision exhausted");
                require(std::size_t(out.intervals) + stack.size() + 2 <= budget,
                        "spiral integration interval budget exceeded");
                stack.push_back({middle, item.b, item.depth + 1});
                stack.push_back({item.a, middle, item.depth + 1});
                continue;
            }
            require(out.intervals < budget, "spiral integration interval budget exceeded");
            const double aa = angle(item.a), bb = angle(middle), cc = angle(item.b);
            local[0] += h / 6 * (std::cos(aa) + 4 * std::cos(bb) + std::cos(cc));
            local[1] += h / 6 * (std::sin(aa) + 4 * std::sin(bb) + std::sin(cc));
            out.quadrature_error_bound += error;
            ++out.intervals;
        }
        for (double &v : local)
            v = finite(v * length_ * (t < 0 ? -1 : 1));
    }
    for (unsigned k = 0; k < 3; ++k) {
        out.point[k] = finite(transform_[4 * k] * local[0] + transform_[4 * k + 1] * local[1] +
                              transform_[4 * k + 3]);
        out.derivative[k] = finite(speed * (transform_[4 * k] * std::cos(out.local_bearing) +
                                            transform_[4 * k + 1] * std::sin(out.local_bearing)));
    }
    return out;
}
Json TransitionSpiral::native_fit_input(unsigned budget) const {
    require(budget > 0, "native spiral integration budget");
    const double inner = std::sqrt((3 - 2 * std::sqrt(6. / 5)) / 7),
                 outer = std::sqrt((3 + 2 * std::sqrt(6. / 5)) / 7),
                 wi = (18 + std::sqrt(30.)) / 36, wo = (18 - std::sqrt(30.)) / 36;
    const std::array<double, 4> nodes{-inner, inner, -outer, outer}, weights{wi, wi, wo, wo};
    auto count = [&](double a, double b) {
        const double raw = finite(std::abs(angle(b) - angle(a)) / .04 + .9999999999);
        require(raw < std::numeric_limits<int>::max() - 1., "native spiral sample count overflow");
        auto n = unsigned(raw);
        if (n & 1)
            ++n;
        return std::max(1u, n);
    };
    auto gauss = [&](double a, double b, unsigned pieces) {
        Point2 sum{};
        const double step = (b - a) / pieces;
        for (unsigned i = 0; i < pieces; ++i) {
            const double left = a + i * step, right = a + (i + 1) * step,
                         half = (right - left) * .5;
            for (unsigned j = 0; j < 4; ++j) {
                const double theta =
                    angle(length_ == 0 ? 0 : (left + (nodes[j] + 1) * half) / length_);
                const double weight = half * weights[j];
                sum[0] += weight * std::cos(theta);
                sum[1] += weight * std::sin(theta);
            }
        }
        return sum;
    };
    const unsigned intervals = count(start_, end_), offset_intervals = count(0, start_);
    require(intervals < 2000, "native spiral stroke exceeds its 2000 point buffer");
    require(std::uint64_t(intervals) + offset_intervals <= budget,
            "native spiral integration interval budget exceeded");
    auto stroke = [&](double a, double b, unsigned n, bool save, std::vector<Point3> &points,
                      std::vector<double> &fractions, double &estimate) {
        const double start = finite(a * length_), end = finite(b * length_),
                     step = finite(end - start) / n;
        Point2 sum{};
        if (save) {
            points.push_back({0, 0, 0});
            fractions.push_back(a);
        }
        for (unsigned i = 0; i < n; ++i) {
            const double left = start + i * step, right = start + (i + 1) * step;
            const auto coarse = gauss(left, right, 1), fine = gauss(left, right, 2);
            Point2 difference{fine[0] - coarse[0], fine[1] - coarse[1]};
            for (unsigned k = 0; k < 2; ++k)
                sum[k] = finite(sum[k] + fine[k] + difference[k] / 255);
            estimate =
                finite(estimate + std::max(std::abs(difference[0]), std::abs(difference[1])) / 255);
            if (save) {
                points.push_back({sum[0], sum[1], 0});
                // Parameter provenance for the generated samples, not an array
                // serialized in BGFB. It also remains defined for zero length.
                fractions.push_back(finite(a + (b - a) * (double(i + 1) / n)));
            }
        }
        return sum;
    };
    std::vector<Point3> points;
    std::vector<double> fractions;
    double main_error = 0, offset_error = 0;
    stroke(start_, end_, intervals, true, points, fractions, main_error);
    const auto offset = stroke(0, start_, offset_intervals, false, points, fractions, offset_error);
    for (auto &p : points)
        for (unsigned k = 0; k < 2; ++k)
            p[k] = finite(p[k] + offset[k]);
    auto bearing = [&](std::size_t first, std::size_t second, double t) {
        double theta = angle(t);
        const double dot = finite((points[second][0] - points[first][0]) * std::cos(theta) +
                                  (points[second][1] - points[first][1]) * std::sin(theta));
        if (dot < 0)
            theta = finite(theta + pi);
        return theta;
    };
    const double k0 = curvature(start_), k1 = curvature(end_);
    return {{"local_points", points},
            {"source_fractions", fractions},
            {"local_origin_offset", {offset[0], offset[1], 0.}},
            {"endpoint_bearings",
             {bearing(0, 1, start_), bearing(points.size() - 2, points.size() - 1, end_)}},
            {"endpoint_radii", {k0 == 0 ? 0 : finite(1 / k0), k1 == 0 ? 0 : finite(1 / k1)}},
            {"angular_step", .04},
            {"stroke_intervals", intervals},
            {"origin_intervals", offset_intervals},
            {"stroke_error_estimate", main_error},
            {"origin_error_estimate", offset_error},
            {"error_estimate_kind", "native_gauss4_richardson_local_max_component"},
            {"fit_point_limit", 998},
            {"fits_native_point_limit", points.size() <= 998},
            {"bspline_fit_status", "not_evaluated"}};
}
} // namespace p3d
