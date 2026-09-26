#include "native_tube_caps.hpp"
#include "native_surface_iso.hpp"
#include "native_knot_normalize.hpp"
#include "loft_curve.hpp"

namespace p3d::swept_detail {
namespace {
curve_detail::BezierWork work(TubeBudget &b) {
    return {b.work, b.max_work};
}
Json variant(Json value) {
    return {{"_type", "VariantGeometry"}, {"geometry", std::move(value)}};
}
Json region(unsigned type, Json curves = Json::array()) {
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", std::move(curves)}};
}
Json table(const BsplineCurve &c) {
    Json points = Json::array();
    for (const auto &p : c.poles())
        for (double x : p)
            points.push_back(x);
    return {{"_type", "BsplineCurve"},
            {"order", c.order()},
            {"closed", c.closed()},
            {"poles", std::move(points)},
            {"weights", c.rational() ? Json(c.weights()) : Json(nullptr)},
            {"knots", c.knots()}};
}
Json reverse_closed_curve(const BsplineCurve &source, TubeBudget &b, Json &report) {
    require(source.closed() && source.order() <= 26 && b.max_control_points <= UINT32_MAX,
            "native cap reversal requires a supported closed curve");
    for (unsigned i = 0; i < 8 * (source.order() + 1); ++i)
        work(b).charge(source.poles().size());
    Json opening;
    auto c = loft_detail::open_periodic_boundary(source, unsigned(b.max_control_points), &opening);
    work(b).charge(c.knots.size());
    for (unsigned i = 0; i < 16; ++i)
        work(b).charge(c.poles.size());
    std::reverse(c.poles.begin(), c.poles.end());
    std::reverse(c.knots.begin(), c.knots.end());
    const bool normalized =
        curve_detail::normalize_native_knots(c.knots, c.poles.size(), source.order(), false);
    // The native primitive wrapper ignores the curve-operation status. Do not
    // silently reclose a result when native endpoint checks have rejected it.
    require(normalized, "native cap reversal produced unevaluable descending knots");
    work(b).charge(4 * std::size_t(source.order()) * source.order());
    auto closed = loft_detail::close_native_curve(BsplineCurve::from_bgfb(c.table()),
                                                  unsigned(b.max_control_points));
    auto result = closed.curve.table();
    result["closed"] = closed.closed;
    report = {{"opening", std::move(opening)},
              {"knots_normalized", normalized},
              {"closure", std::move(closed.report)},
              {"closure_succeeded", closed.success},
              {"closed", closed.closed},
              {"native_primitive_result", true}};
    return result;
}
} // namespace

TubeCaps tube_cap_regions(const std::vector<Json> &surfaces, TubeBudget &b) {
    TubeCaps out;
    out.report = {{"scope", "native_swept_cap_regions"},
                  {"native_result", false},
                  {"visited_surfaces", 0},
                  {"completed_rings", 0},
                  {"start_reversed", false},
                  {"stop_reason", "empty_surface_list"},
                  {"rings", Json::array()},
                  {"work_used", b.work}};
    work(b).charge(surfaces.size());
    if (surfaces.empty()) {
        out.report["work_used"] = b.work;
        return out;
    }
    if (surfaces.size() > 1)
        out.start = out.end = region(4);
    std::vector<BsplineCurve> starts;
    std::size_t controls = 0;
    for (std::size_t i = 0; i < surfaces.size(); ++i) {
        const auto &j = surfaces[i];
        require(j.is_object() && j.contains("poles") && j["poles"].is_array() &&
                    j["poles"].size() % 3 == 0 && j["poles"].size() / 3 <= b.max_control_points,
                "native cap surface layout/control budget exceeded");
        for (unsigned k = 0; k < 8; ++k)
            work(b).charge(j["poles"].size() / 3);
        const auto surface = BsplineSurface::from_bgfb(j);
        auto first = detail::native_iso_v_curve(surface, 0, work(b), b.max_control_points);
        auto last = detail::native_iso_v_curve(surface, 1, work(b), b.max_control_points);
        out.report["visited_surfaces"] = i + 1;
        if (!first.curve.closed() || !last.curve.closed()) {
            out.report["stop_reason"] = "open_isocurve";
            out.report["work_used"] = b.work;
            return out;
        }
        for (const auto *c : {&first.curve, &last.curve}) {
            require(c->poles().size() <= b.max_control_points - controls,
                    "native cap total control budget exceeded");
            controls += c->poles().size();
            work(b).charge(8 * c->poles().size());
        }
        const unsigned type = i == 0 ? 2 : 3;
        auto a = region(type, Json::array({variant(table(first.curve))}));
        auto z = region(type, Json::array({variant(table(last.curve))}));
        if (surfaces.size() == 1) {
            out.start = std::move(a);
            out.end = std::move(z);
        } else {
            out.start["curves"].push_back(variant(std::move(a)));
            out.end["curves"].push_back(variant(std::move(z)));
        }
        starts.push_back(std::move(first.curve));
        out.report["rings"].push_back({{"source_surface_index", i},
                                       {"boundary_type", type},
                                       {"start_zero_weight_fallbacks", first.zero_weight_fallbacks},
                                       {"end_zero_weight_fallbacks", last.zero_weight_fallbacks},
                                       {"reversal", nullptr}});
        out.report["completed_rings"] = i + 1;
    }
    // CurveVector reversal retains type-4 child order. Each child here contains
    // one curve; type-2/3 member reversal therefore changes only its direction.
    for (std::size_t i = 0; i < starts.size(); ++i) {
        Json reversal;
        auto curve = reverse_closed_curve(starts[i], b, reversal);
        const auto count = curve.at("poles").size() / 3;
        controls -= starts[i].poles().size();
        require(count <= b.max_control_points - controls,
                "native reversed cap total control budget exceeded");
        controls += count;
        auto &r = surfaces.size() == 1 ? out.start : out.start["curves"][i]["geometry"];
        r["curves"][0]["geometry"] = std::move(curve);
        out.report["rings"][i]["reversal"] = std::move(reversal);
    }
    out.report["start_reversed"] = true;
    out.report["native_result"] = true;
    out.report["stop_reason"] = "completed";
    out.report["control_points"] = controls;
    out.report["work_used"] = b.work;
    return out;
}

CappedTube prepare_swept_tube_with_caps(const Json &profile, const Json &path, bool capped,
                                        TubeBudget &budget) {
    CappedTube out{prepare_swept_tube_surfaces(profile, path, budget), {}, {}};
    const bool sides_ok = out.sides.report.at("native_generation_result");
    const bool source_closed = out.sides.report.at("path_conversion").at("source_endpoint_closed");
    const bool requested = sides_ok && capped && !source_closed;
    out.report = {{"scope", "native_swept_side_and_cap_preparation"},
                  {"native_result", sides_ok},
                  {"source_capped", capped},
                  {"source_path_endpoint_closed", source_closed},
                  {"caps_requested", requested},
                  {"cap_generation", nullptr},
                  {"native_face_indices", nullptr}};
    if (requested) {
        auto caps = tube_cap_regions(out.sides.surfaces, budget);
        const bool ok = caps.report.at("native_result");
        out.report["native_result"] = ok;
        out.report["cap_generation"] = std::move(caps.report);
        if (ok) {
            out.caps.push_back(std::move(caps.start));
            out.caps.push_back(std::move(caps.end));
        }
    }
    out.report["work_used"] = budget.work;
    return out;
}
} // namespace p3d::swept_detail
