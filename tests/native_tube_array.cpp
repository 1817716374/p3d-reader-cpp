#include "native_tube.hpp"
#include "native_curve_conversion.hpp"
#include "loft_curve.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
Json spline(unsigned order, Json poles, Json knots, Json weights = nullptr, bool closed = false) {
    return {{"_type", "BsplineCurve"}, {"order", order}, {"closed", closed},
            {"poles", poles},          {"knots", knots}, {"weights", weights}};
}
Json line(double a, double b) {
    return {{"_type", "LineSegment"},
            {"segment",
             {{"point0X", a},
              {"point0Y", 0},
              {"point0Z", 0},
              {"point1X", b},
              {"point1Y", 0},
              {"point1Z", 0}}}};
}
Json group(unsigned type, std::initializer_list<Json> geometries) {
    Json entries = Json::array();
    for (const auto &g : geometries)
        entries.push_back({{"geometry", g}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", entries}};
}
Json polygon(double origin = 0) {
    return {
        {"_type", "LineString"},
        {"points", {origin, 0, 0, origin + 2, 0, 0, origin + 2, 2, 0, origin, 2, 0, origin, 0, 0}}};
}
} // namespace
unsigned native_tube_array_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
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
    auto convert = [](const Json &v) {
        TubeBudget b;
        return convert_tube_curve_array(v, b);
    };
    auto profile = [](const Json &v) {
        TubeBudget b;
        return convert_tube_profile(v, b);
    };
    for (unsigned type = 0; type <= 3; ++type) {
        const auto c = convert(group(type, {line(0, 2), line(2, 8)}));
        check(!c.curve.closed() && c.curve.knots() == std::vector<double>({0, 0, .25, 1, 1}) &&
                  c.report["source_endpoint_closed"] == false,
              "source group type does not force closure");
        const auto r = convert(group(type, {polygon()}));
        check(r.curve.closed() && r.curve.poles().size() == 4 &&
                  r.report["closure"]["success"] == true,
              "all simple group types use source endpoints to request native periodic closure");
    }
    const auto mixed =
        convert(group(1, {line(0, 2), spline(3, {2, 0, 0, 3, 1, 0, 4, 0, 0}, {0, 0, 0, 1, 1, 1})}));
    check(mixed.curve.order() == 3 && mixed.report["joins"][0]["left_elevation"].is_object(),
          "complete group pipeline converts and elevates native primitives before joining");
    const auto gapped = convert(group(2, {line(0, 1), line(3, 4), line(6, 0)}));
    check(gapped.report["source_endpoint_closed"] == true && gapped.curve.closed() &&
              gapped.report["joins"][0]["combination"]["contiguous"] == false,
          "source first/last agreement does not validate or repair intermediate gaps");
    check(gapped.curve.poles().size() == 5,
          "closed discontinuous linear group removes only final duplicate control");
    const auto weighted = spline(3, {0, 0, 0, 1, 1, 0, 0, 0, 0}, {2, 2, 2, 7, 7, 7}, {1, 1, 2});
    const auto failed = convert(group(2, {weighted}));
    check(failed.report["source_endpoint_closed"] == true &&
              failed.report["closure"]["success"] == false && !failed.curve.closed() &&
              failed.curve.knots() == std::vector<double>({2, 2, 2, 7, 7, 7}) &&
              failed.curve.weights() == std::vector<double>({1, 1, 2}),
          "ignored native closure failure keeps all original curve storage and reports failure");
    const auto repeated_line = convert(group(2, {line(5, 5)}));
    check(repeated_line.report["closure"]["success"] == true && !repeated_line.curve.closed(),
          "two-pole line closure succeeds by copy but stays open");
    const auto multi =
        group(4, {group(3, {polygon(10)}), group(2, {polygon(-5)}), group(3, {polygon(10)})});
    const auto rings = profile(multi);
    check(
        rings.curves.size() == 3 && rings.curves[0].poles().front()[0] == 10 &&
            rings.curves[1].poles().front()[0] == -5 &&
            rings.curves[2].poles() == rings.curves[0].poles(),
        "parity profile preserves ring order and duplicate rings without spatial reclassification");
    check(rings.report["rings"][0]["boundary_type"] == 3 &&
              rings.report["rings"][1]["boundary_type"] == 2,
          "source outer/inner labels retained independently of order");
    const auto notactuallyclosed = profile(group(4, {group(2, {weighted})}));
    check(notactuallyclosed.curves.size() == 1 && !notactuallyclosed.curves[0].closed() &&
              notactuallyclosed.report["rings"][0]["closure"]["success"] == false,
          "parity ring uses source closure predicate, not final spline closed flag");
    check(profile(group(0, {line(0, 1)})).curves.size() == 1,
          "simple profile may be an open source group");
    rejects([&] { profile(group(4, {group(2, {line(0, 1)})})); },
            "parity ring with open source endpoints rejected");
    rejects([&] { profile(group(4, {group(1, {polygon()})})); },
            "parity child must be type two or three");
    rejects([&] { profile(group(5, {group(2, {polygon()})})); }, "union profile rejected");
    rejects([&] { convert(group(4, {group(2, {polygon()})})); },
            "path group cannot be parity collection");
    rejects([&] { convert(group(1, {})); }, "empty source group rejected");
    rejects([&] { convert(group(1, {Json(nullptr)})); }, "null primitive conversion rejected");
    rejects([&] { convert(group(1, {group(1, {line(0, 1)})})); },
            "nested primitive array is not an accepted basic swept primitive");
    rejects([&] { convert(group(1, {{{"_type", "PointString"}, {"points", {0, 0, 0}}}})); },
            "PointString not swept source");
    rejects(
        [&] {
            TubeBudget b;
            b.max_work = 1;
            convert_tube_profile(multi, b);
        },
        "parity members share cumulative budget");
    rejects(
        [&] {
            TubeBudget b;
            b.max_control_points = 6;
            convert_tube_profile(multi, b);
        },
        "parity profile bounds total controls across individually valid rings");
    // General close entry retains failure/copy domains; successful conversion
    // normalizes source knots even if non-clamped or internally discontinuous.
    auto close = [](const Json &j) {
        return loft_detail::close_native_curve(BsplineCurve::from_bgfb(j), 100);
    };
    const auto raw = spline(2, {0, 0, 0, 1, 0, 0, 0, 0, 0}, {0, 2, 4, 6, 8});
    auto rc = close(raw);
    check(rc.success && rc.closed && rc.curve.knots == std::vector<double>({-.5, 0, .5, 1, 1.5}) &&
              rc.curve.poles.size() == 2,
          "linear closure normalizes nonclamped domain then builds periodic end intervals");
    const auto broken = spline(3, {0, 0, 0, 1, 1, 0, 2, 0, 0, 3, 0, 0, 4, 1, 0, 0, 0, 0},
                               {0, 0, 0, .5, .5, .5, 1, 1, 1});
    const auto bc = close(broken);
    check(bc.success && bc.closed,
          "higher-order closure accepts internal full-multiplicity discontinuity");
    auto bt = bc.curve.table();
    bt["closed"] = true;
    check(BsplineCurve::from_bgfb(bt).closed(),
          "general periodic closure result has valid storage");
    auto already = close(bt);
    check(already.report["method"] == "already_closed_copy" &&
              already.curve.knots == bc.curve.knots,
          "already-closed curves copy without normalizing");
    const auto different = close(spline(2, {0, 0, 0, 1, 1, 0, 2, 0, 0}, {2, 2, 4, 6, 6}));
    check(!different.success && different.curve.knots == std::vector<double>({2, 2, 4, 6, 6}),
          "endpoint mismatch is evaluated before node normalization");
    auto zero = close(spline(3, {0, 0, 0, 1, 1, 0, 0, 0, 0}, {0, 0, 0, 1, 1, 1}, {0, -1, 0}));
    check(zero.success && zero.closed,
          "stored homogeneous endpoint closure does not divide zero weights");
    auto arc = Json{{"_type", "EllipticArc"},
                    {"arc",
                     {{"centerX", 0},
                      {"centerY", 0},
                      {"centerZ", 0},
                      {"vector0X", 2},
                      {"vector0Y", 0},
                      {"vector0Z", 0},
                      {"vector90X", 0},
                      {"vector90Y", 1},
                      {"vector90Z", 0},
                      {"startRadians", 0},
                      {"sweepRadians", 6.283185307179586}}}};
    auto ac = convert(group(2, {arc}));
    check(ac.curve.closed() &&
              ac.report["closure"]["conversion"]["method"] == "already_closed_copy",
          "whole ellipse conversion remains in native closed representation");
    arc["arc"]["startRadians"] = .7;
    arc["arc"]["sweepRadians"] = .5;
    auto endpoints = curve_detail::primitive_endpoints(arc);
    check(endpoints && (*endpoints)[0] == Point3({2 * std::cos(.7), std::sin(.7), 0}),
          "source ellipse closure uses analytic source angle, not adjusted converted angle");
    for (double d : {0., 1e-11, 1e-10, 2e-10})
        check(curve_detail::endpoint_pair_closed({0, 0, 0}, {d, 0, 0}) == (d < 1e-10),
              "source closure uses strict squared relative tolerance");
    const auto original = multi.dump();
    std::vector<std::future<TubeProfile>> jobs;
    for (unsigned i = 0; i < 8; ++i)
        jobs.push_back(std::async(std::launch::async, [&] { return profile(multi); }));
    for (auto &job : jobs) {
        auto r = job.get();
        check(r.report == rings.report, "parallel profile conversions are deterministic");
    }
    check(multi.dump() == original, "source curve arrays and repeated rings remain immutable");
    return n;
}
