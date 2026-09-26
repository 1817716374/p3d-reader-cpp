#include "native_tube_caps.hpp"
#include "native_surface_iso.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
Json surface(unsigned order = 2, bool rational = false, bool closed = true) {
    const auto n = std::max(4u, order + 1);
    Json poles = Json::array(), weights = Json::array();
    for (unsigned row = 0; row < 2; ++row)
        for (unsigned i = 0; i < n; ++i) {
            const double a = 6.283185307179586 * i / n;
            const double w = rational ? 1 + .1 * i + .2 * row : 1;
            poles.push_back(2 * std::cos(a) * w);
            poles.push_back(std::sin(a) * w);
            poles.push_back(3. * row * w);
            weights.push_back(w);
        }
    return {{"_type", "BsplineSurface"},
            {"orderU", order},
            {"orderV", 2},
            {"numPolesU", n},
            {"numPolesV", 2},
            {"closedU", closed},
            {"closedV", false},
            {"knotsU", nullptr},
            {"knotsV", {-4., -2., 3., 5.}},
            {"poles", poles},
            {"weights", rational ? weights : Json(nullptr)},
            {"boundaries", nullptr},
            {"numRulesU", 0},
            {"numRulesV", 0},
            {"holeOrigin", 0}};
}
const Json &cap_curve(const Json &cap, std::size_t ring = 0) {
    if (cap.at("type") == 4)
        return cap.at("curves").at(ring).at("geometry").at("curves").at(0).at("geometry");
    return cap.at("curves").at(0).at("geometry");
}
Json group(unsigned type, std::initializer_list<Json> values) {
    Json curves = Json::array();
    for (const auto &v : values)
        curves.push_back({{"geometry", v}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", curves}};
}
Json line() {
    return {{"_type", "LineSegment"},
            {"segment",
             {{"point0X", 0.},
              {"point0Y", 0.},
              {"point0Z", 0.},
              {"point1X", 0.},
              {"point1Y", 0.},
              {"point1Z", 3.}}}};
}
Json polygon() {
    return {{"_type", "LineString"},
            {"points", {0., 0., 0., 2., 0., 0., 2., 2., 0., 0., 2., 0., 0., 0., 0.}}};
}
bool near(const Point3 &a, const Point3 &b) {
    for (unsigned i = 0; i < 3; ++i)
        if (std::abs(a[i] - b[i]) > 2e-8 * std::max({1., std::abs(a[i]), std::abs(b[i])}))
            return false;
    return true;
}
} // namespace
unsigned native_tube_caps_tests() {
    unsigned count = 0;
    auto check = [&](bool ok, const char *why) {
        ++count;
        require(ok, why);
    };
    auto rejects = [&](auto fn, const char *why) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, why);
    };
    for (unsigned order : {2u, 3u, 4u, 8u})
        for (bool rational : {false, true}) {
            const auto input = surface(order, rational);
            const auto saved = input;
            const auto source = BsplineSurface::from_bgfb(input);
            TubeBudget b;
            const auto caps = tube_cap_regions({input}, b);
            check(caps.report["native_result"] == true && caps.start["type"] == 2 &&
                      caps.end["type"] == 2 && caps.report["start_reversed"] == true,
                  "one closed side yields two native outer-loop caps");
            const auto start = BsplineCurve::from_bgfb(cap_curve(caps.start));
            const auto end = BsplineCurve::from_bgfb(cap_curve(caps.end));
            for (unsigned i = 0; i <= 30; ++i) {
                const double u = double(i) / 30;
                check(near(start.point_at(u), source.point_at(1 - u, 0)),
                      "start cap agrees with independently evaluated reversed periodic side");
                check(near(end.point_at(u), source.point_at(u, 1)),
                      "end cap agrees with independently evaluated forward side");
            }
            check(input == saved && end.knots() == source.u().knots() &&
                      end.closed() == source.u().closed(),
                  "cap generation copies source and retains end curve full node layout");
        }
    auto nonunit = surface(3, true);
    const auto default_surface = BsplineSurface::from_bgfb(nonunit);
    Json shifted_knots = Json::array();
    for (double k : default_surface.u().knots())
        shifted_knots.push_back(4 * k - 2);
    nonunit["knotsU"] = shifted_knots;
    const auto shifted_surface = BsplineSurface::from_bgfb(nonunit);
    TubeBudget shift_budget;
    const auto shifted_caps = tube_cap_regions({nonunit}, shift_budget);
    const auto shifted_start = BsplineCurve::from_bgfb(cap_curve(shifted_caps.start));
    for (unsigned i = 0; i <= 30; ++i) {
        const double u = double(i) / 30;
        double source_u = .5 - u;
        if (source_u < 0)
            source_u += 1;
        check(near(shifted_start.point_at(u), shifted_surface.point_at(source_u, 0)),
              "periodic cap reversal opens at native knot zero inside a nonunit domain");
    }
    check(cap_curve(shifted_caps.end)["knots"] == shifted_knots,
          "end cap retains source domain instead of inheriting start normalization");
    auto imperfect = surface(3, true);
    const double root3 = std::sqrt(3.) / 2;
    Json row{1., 0.,  0.,     .5, root3, 0.,     -.5, root3,    0., -1., 0.,
             0., -.5, -root3, 0., .5,    -root3, 0.,  1.000001, 0., 0.};
    Json row_weights{1., .5, 1., .5, 1., .5, 1.};
    imperfect["numPolesU"] = 7;
    imperfect["knotsU"] = {-1. / 3, 0., 0., 0., 1. / 3, 1. / 3, 2. / 3, 2. / 3, 1., 1., 1., 4. / 3};
    imperfect["poles"] = row;
    imperfect["weights"] = row_weights;
    for (std::size_t i = 0; i < row.size(); ++i)
        imperfect["poles"].push_back(i % 3 == 2 ? 3 * row_weights[i / 3].get<double>()
                                                : row[i].get<double>());
    for (const auto &w : row_weights)
        imperfect["weights"].push_back(w);
    TubeBudget imperfect_budget;
    const auto imperfect_caps = tube_cap_regions({imperfect}, imperfect_budget);
    check(
        imperfect_caps.report["native_result"] == true &&
            imperfect_caps.report["rings"][0]["reversal"]["closure_succeeded"] == false &&
            cap_curve(imperfect_caps.start)["closed"] == false &&
            cap_curve(imperfect_caps.end)["closed"] == true,
        "native cap caller ignores reverse primitive closure failure without repairing its result");
    const auto original = surface();
    auto displaced = original;
    for (std::size_t i = 0; i < displaced["poles"].size(); i += 3)
        displaced["poles"][i] = displaced["poles"][i].get<double>() + 7;
    TubeBudget mb;
    const auto multi = tube_cap_regions({original, displaced, original}, mb);
    check(multi.report["native_result"] == true && multi.start["type"] == 4 &&
              multi.end["type"] == 4 && multi.start["curves"].size() == 3,
          "multiple native sides produce parity regions without deduplication");
    for (std::size_t i = 0; i < 3; ++i) {
        check(multi.start["curves"][i]["geometry"]["type"] == (i ? 3 : 2) &&
                  multi.end["curves"][i]["geometry"]["type"] == (i ? 3 : 2),
              "source first ring is outer and later rings inner on both ends");
        const auto c = BsplineCurve::from_bgfb(cap_curve(multi.start, i));
        const auto s = BsplineSurface::from_bgfb(i == 1 ? displaced : original);
        check(near(c.point_at(.23), s.point_at(.77, 0)),
              "parity reversal retains ring order rather than reversing children");
    }
    auto job = std::async(std::launch::async, [&] {
        TubeBudget b;
        return tube_cap_regions({original, displaced, original}, b);
    });
    const auto concurrent = job.get();
    check(concurrent.start == multi.start && concurrent.end == multi.end &&
              concurrent.report == multi.report,
          "cap generation is immutable and thread independent");
    const auto open = surface(2, false, false);
    for (const auto &inputs :
         {std::vector<Json>{}, std::vector<Json>{open}, std::vector<Json>{original, open},
          std::vector<Json>{open, original}}) {
        TubeBudget b;
        const auto failed = tube_cap_regions(inputs, b);
        check(failed.report["native_result"] == false && failed.report["start_reversed"] == false,
              "empty list or open isocurve fails before start-cap reversal");
        if (inputs.size() <= 1)
            check(failed.start.is_null() && failed.end.is_null(),
                  "single-side failure leaves both cap pointers empty");
        else {
            const std::size_t n = inputs.front() == open ? 0 : 1;
            check(failed.start["curves"].size() == n && failed.end["curves"].size() == n,
                  "multi-side failure retains previously assembled regions including empty parity");
            if (n) {
                const auto c = BsplineCurve::from_bgfb(cap_curve(failed.start));
                check(near(c.point_at(.19), BsplineSurface::from_bgfb(original).point_at(.19, 0)),
                      "partial start cap has not been reversed before later isocurve failure");
            }
        }
    }
    const auto profile = group(2, {polygon()}), path = group(1, {line()});
    Json imperfect_path{{"_type", "BsplineCurve"}, {"order", 3},
                        {"closed", true},          {"poles", row},
                        {"weights", row_weights},  {"knots", imperfect["knotsU"]}};
    TubeBudget mismatch_budget;
    const auto mismatch =
        prepare_swept_tube_with_caps(profile, group(1, {imperfect_path}), true, mismatch_budget);
    check(
        mismatch.sides.trace.closed() && mismatch.report["source_path_endpoint_closed"] == false &&
            mismatch.report["caps_requested"] == true,
        "outer cap decision uses source endpoints even when converted trace carries a closed flag");
    TubeBudget bb;
    const auto body = prepare_swept_tube_with_caps(profile, path, true, bb);
    check(body.report["native_result"] == true && body.report["caps_requested"] == true &&
              body.caps.size() == 2 && body.sides.surfaces.size() == 1,
          "outer swept caller appends two cap regions for capped open source path");
    TubeBudget off;
    const auto no_caps = prepare_swept_tube_with_caps(profile, path, false, off);
    check(no_caps.report["native_result"] == true && no_caps.caps.empty() &&
              no_caps.report["cap_generation"].is_null() &&
              no_caps.sides.surfaces == body.sides.surfaces,
          "uncapped preparation preserves side result and skips cap work");
    TubeBudget fb;
    const auto bad = prepare_swept_tube_with_caps(group(1, {line()}), path, true, fb);
    check(bad.report["native_result"] == false && bad.caps.empty() && !bad.sides.surfaces.empty(),
          "outer caller retains generated sides but does not append failed cap regions");
    TubeBudget cb;
    const auto closed = prepare_swept_tube_with_caps(profile, group(1, {polygon()}), true, cb);
    check(closed.report["source_path_endpoint_closed"] == true && closed.caps.empty() &&
              closed.report["caps_requested"] == false,
          "closed source path skips caps despite capped flag");
    rejects(
        [&] {
            TubeBudget b;
            b.max_work = mb.work - 1;
            tube_cap_regions({original, displaced, original}, b);
        },
        "cap extraction opening reversal and closure share a cumulative work budget");
    rejects(
        [&] {
            TubeBudget b;
            b.max_control_points = 15;
            tube_cap_regions({original, displaced, original}, b);
        },
        "cap control count is bounded across all rings and endpoints");
    return count;
}
