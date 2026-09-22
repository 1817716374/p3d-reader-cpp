#include "internal.hpp"
#include "p3d/pcurve.hpp"

namespace p3d {
namespace {
Point3 interpolate(const Point3 &a, double f, const Point3 &b) {
    Point3 p{};
    for (unsigned k = 0; k < 3; ++k)
        p[k] = a[k] + f * (b[k] - a[k]);
    return p;
}
Point3 split_point(const Point3 &a, double f, const Point3 &b) {
    if (f <= .5)
        return interpolate(a, f, b);
    Point3 p{};
    for (unsigned k = 0; k < 3; ++k)
        p[k] = b[k] + (f - 1) * (b[k] - a[k]);
    return p;
}
double dot(const Point3 &a, const Point3 &b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Point3 subtract(const Point3 &a, const Point3 &b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
double distance(const Point3 &a, const Point3 &b) {
    const auto d = subtract(a, b);
    const double result = std::sqrt(dot(d, d));
    require(std::isfinite(result), "native PCurve distance overflow");
    return result;
}
void finite(const Point3 &p) {
    for (double x : p)
        require(std::isfinite(x), "native PCurve non-finite coordinate");
}
BsplineSurface normalized_surface(const BsplineSurface &source) {
    auto knots = [](const BsplineDirection &d) {
        auto values = d.knots();
        const auto domain = d.knot_domain();
        for (double &v : values)
            v = (v - domain[0]) / (domain[1] - domain[0]);
        return values;
    };
    std::vector<double> poles;
    for (const auto &p : source.poles())
        poles.insert(poles.end(), p.begin(), p.end());
    return BsplineSurface::from_bgfb(
        {{"_type", "BsplineSurface"},
         {"numPolesU", source.u().pole_count()},
         {"numPolesV", source.v().pole_count()},
         {"orderU", source.u().order()},
         {"orderV", source.v().order()},
         {"closedU", source.u().closed()},
         {"closedV", source.v().closed()},
         {"numRulesU", source.num_rules_u()},
         {"numRulesV", source.num_rules_v()},
         {"poles", poles},
         {"weights", source.rational() ? Json(source.weights()) : Json(nullptr)},
         {"knotsU", knots(source.u())},
         {"knotsV", knots(source.v())},
         {"holeOrigin", source.hole_origin()},
         {"boundaries", nullptr}});
}
// Native knot compression compares against the representative, preserving
// the exact active endpoints when their source indices are encountered.
std::vector<double> high_knots(const BsplineDirection &direction) {
    const auto &knots = direction.knots();
    const auto domain = direction.knot_domain();
    const double span = domain[1] - domain[0];
    std::vector<double> values;
    std::vector<unsigned> counts;
    std::size_t low = 0, high = 0;
    for (std::size_t i = 0; i < knots.size(); ++i) {
        const double v = (knots[i] - domain[0]) / span;
        require(std::isfinite(v), "native PCurve knot normalization overflow");
        if (!values.empty() &&
            std::abs(values.back() - v) < (std::abs(values.back()) + 1 + std::abs(v)) * 1e-14)
            ++counts.back();
        else {
            values.push_back(v);
            counts.push_back(1);
        }
        if (i == direction.order() - 1) {
            low = values.size() - 1;
            values.back() = v;
        }
        if (i == knots.size() - direction.order()) {
            high = values.size() - 1;
            values.back() = v;
        }
    }
    std::vector<double> result;
    const unsigned threshold = direction.order() > 2 ? 2 : 1;
    for (std::size_t i = low; i <= high; ++i)
        if (counts[i] >= threshold)
            result.push_back(values[i]);
    return result;
}
struct Stroke {
    const BsplineSurface *surface;
    const PCurveStrokeOptions &options;
    std::vector<PCurveSample> points;
    Json intervals = Json::array();
    unsigned evaluations = 0, omitted_starts = 0;
    double uv_tolerance, spatial_tolerance;

    PCurveSample evaluate(Point3 uv) {
        finite(uv);
        require(evaluations < options.max_evaluations, "native PCurve evaluation budget exhausted");
        ++evaluations;
        return {uv, surface->point_at(uv[0], uv[1])};
    }
    void append(const PCurveSample &p) {
        require(points.size() < options.max_points, "native PCurve point budget exhausted");
        points.push_back(p);
    }
    void start(const PCurveSample &p, double uv_limit) {
        if (!points.empty() &&
            (distance(points.back().parameter, p.parameter) <= uv_limit ||
             distance(points.back().position, p.position) <= spatial_tolerance * 1e-5)) {
            ++omitted_starts;
            return;
        }
        append(p);
    }
    double error(std::size_t base, bool world) const {
        double maximum = 0;
        for (std::size_t i = base; i + 1 < points.size(); ++i) {
            const auto &a = world ? points[i - 1].position : points[i - 1].parameter;
            const auto &b = world ? points[i].position : points[i].parameter;
            const auto &c = world ? points[i + 1].position : points[i + 1].parameter;
            const auto ab = subtract(b, a), ac = subtract(c, a);
            const double numerator = dot(ab, ac), denominator = dot(ac, ac);
            require(std::isfinite(numerator) && std::isfinite(denominator),
                    "native PCurve error projection overflow");
            double f = 0.5;
            if (std::abs(denominator) > std::abs(numerator) * 1e-12) {
                const double candidate = numerator / denominator;
                if (candidate >= 1e-5 && candidate <= 0.99999)
                    f = candidate;
            }
            maximum = std::max(maximum, distance(b, interpolate(a, f, c)));
        }
        return maximum;
    }
    template <class Evaluate>
    void interval(double offset, double length, bool linear, Evaluate at) {
        const auto base = points.size();
        require(base != 0, "native PCurve missing preceding sample");
        unsigned count = 2;
        for (;;) {
            const double step = length / count;
            for (unsigned i = 1; i <= count; ++i)
                append(evaluate(at(offset + double(i) * step)));
            const double uv_error = linear ? 0 : error(base, false);
            const double spatial_error = error(base, true);
            const bool met = uv_error <= uv_tolerance && spatial_error <= spatial_tolerance;
            if (met || count >= 2048 || step < 1e-5) {
                intervals.push_back({{"offset", offset},
                                     {"length", length},
                                     {"subdivisions", count},
                                     {"sampled_uv_error", linear ? Json(nullptr) : Json(uv_error)},
                                     {"sampled_spatial_error", spatial_error},
                                     {"sampled_tolerances_met", met},
                                     {"stop", met             ? "sampled_tolerances"
                                              : count >= 2048 ? "native_subdivision_limit"
                                                              : "native_parameter_step_limit"}});
                return;
            }
            const double ratio =
                std::max(spatial_error / spatial_tolerance, linear ? 0 : uv_error / uv_tolerance);
            unsigned next = unsigned(std::ceil(std::min(16., std::sqrt(ratio)) * count));
            next = std::min(2048u, next);
            if (next == count)
                ++next;
            points.resize(base); // Replace the attempted interval, not the earlier stream.
            count = next;
        }
    }
    void line(Point3 a, Point3 b) {
        for (unsigned k = 0; k < 2; ++k) {
            a[k] = std::clamp(a[k], 0., 1.);
            b[k] = std::clamp(b[k], 0., 1.);
        }
        start(evaluate(a), 1e-8);
        const auto at = [&](double f) { return interpolate(a, f, b); };
        // The linear branch always starts with two independent half intervals.
        interval(0, .5, true, at);
        interval(.5, .5, true, at);
    }
};
} // namespace

PCurveStrokes sample_native_pcurve(const BsplineSurface &surface, const BsplineCurve &curve,
                                   const PCurveStrokeOptions &options,
                                   const std::optional<PCurveSample> &previous) {
    require(std::isfinite(options.uv_tolerance) && std::isfinite(options.spatial_tolerance),
            "native PCurve tolerances must be finite");
    require(std::isfinite(options.start_fraction) && std::isfinite(options.end_fraction) &&
                options.start_fraction >= 0 && options.start_fraction <= 1 &&
                options.end_fraction >= 0 && options.end_fraction <= 1,
            "native PCurve fraction interval must be in [0,1]");
    require(options.max_points > 0 && options.max_evaluations > 0,
            "native PCurve budgets must be positive");
    std::optional<BsplineSurface> prepared_surface;
    Stroke stroke{&surface,
                  options,
                  {},
                  Json::array(),
                  0,
                  0,
                  options.uv_tolerance > 0 ? options.uv_tolerance : .001,
                  options.spatial_tolerance > 0 ? options.spatial_tolerance : 1e-7};
    PCurveStrokes result;
    result.report = {{"status", "incomplete"},
                     {"algorithm", "native_append_pcurve_strokes"},
                     {"parameter_coordinates", "surface_fractions"},
                     {"curve_order", curve.order()},
                     {"continuous_error_bound", nullptr},
                     {"uv_tolerance", stroke.uv_tolerance},
                     {"spatial_tolerance", stroke.spatial_tolerance}};
    try {
        const Point2 unit_domain{0, 1};
        if (surface.u().knot_domain() != unit_domain || surface.v().knot_domain() != unit_domain) {
            prepared_surface = normalized_surface(surface);
            stroke.surface = &*prepared_surface;
        }
        result.report["surface_knot_preparation"] =
            prepared_surface ? "normalized_copy" : "already_normalized";
        if (previous) {
            finite(previous->parameter);
            finite(previous->position);
            stroke.points.push_back(*previous);
        }
        if (curve.order() == 2) {
            const auto u = high_knots(stroke.surface->u()), v = high_knots(stroke.surface->v());
            result.report["surface_split_knots"] = {{"u", u}, {"v", v}};
            result.report["interval_parameter"] = "linear_subsegment_fraction";
            const auto &poles = curve.poles();
            const auto n = poles.size() - (curve.closed() ? 0 : 1);
            for (std::size_t i = 0; i < n; ++i) {
                const auto &a = poles[i], &b = poles[(i + 1) % poles.size()];
                std::vector<double> splits{0, 1};
                for (unsigned k = 0; k < 2; ++k) {
                    const double delta = b[k] - a[k];
                    if (std::abs(delta) <= 1e-15)
                        continue;
                    const double inverse = 1 / delta;
                    for (double knot : k == 0 ? u : v) {
                        const double f = (knot - a[k]) * inverse;
                        if (f > 0 && f < 1)
                            splits.push_back(f);
                    }
                }
                std::sort(splits.begin(), splits.end());
                std::vector<double> distinct{splits.front()};
                for (double f : splits)
                    if (f > distinct.back() + 1e-8)
                        distinct.push_back(f);
                if (distinct.back() < 1)
                    distinct.back() = 1;
                result.report["linear_pole_weights_applied"] = false;
                for (std::size_t j = 1; j < distinct.size(); ++j) {
                    const auto begin = stroke.intervals.size();
                    stroke.line(split_point(a, distinct[j - 1], b), split_point(a, distinct[j], b));
                    for (std::size_t k = begin; k < stroke.intervals.size(); ++k) {
                        stroke.intervals[k]["source_pole_segment"] = i;
                        stroke.intervals[k]["pole_fraction_range"] = {distinct[j - 1], distinct[j]};
                    }
                }
            }
            result.report["fraction_interval_applied"] = false;
        } else {
            result.report["interval_parameter"] = "curve_fraction";
            const auto count = std::max(2u, options.minimum_points);
            require(count <= options.max_points, "native PCurve minimum points exceed budget");
            const auto at = [&](double f) { return curve.point_at(f); };
            stroke.start(stroke.evaluate(at(options.start_fraction)), stroke.uv_tolerance * 1e-5);
            const double length = (options.end_fraction - options.start_fraction) / (count - 1);
            for (unsigned i = 0; i + 1 < count; ++i)
                stroke.interval(options.start_fraction + double(i) * length, length, false, at);
            result.report["fraction_interval_applied"] = true;
        }
        result.samples = std::move(stroke.points);
        if (previous)
            result.samples.erase(result.samples.begin());
        result.report["status"] = "complete";
    } catch (const std::exception &e) {
        result.samples.clear();
        result.report["reason"] = e.what();
    }
    result.report["evaluations"] = stroke.evaluations;
    result.report["omitted_starts"] = stroke.omitted_starts;
    result.report["intervals"] = std::move(stroke.intervals);
    result.report["sampled_tolerances_met"] =
        result.report["status"] == "complete" &&
        std::all_of(result.report["intervals"].begin(), result.report["intervals"].end(),
                    [](const Json &v) { return v.at("sampled_tolerances_met").get<bool>(); });
    return result;
}
} // namespace p3d
