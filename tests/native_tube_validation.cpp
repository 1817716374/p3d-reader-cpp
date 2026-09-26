#include "native_tube.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
Json group(Json curves, int type = 1) {
    Json members = Json::array();
    for (const auto &c : curves)
        members.push_back({{"geometry", c}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", members}};
}
Json line() {
    return {{"_type", "LineSegment"},
            {"segment",
             {{"point0X", 0.},
              {"point0Y", 0.},
              {"point0Z", 0.},
              {"point1X", 0.},
              {"point1Y", 0.},
              {"point1Z", 4.}}}};
}
Json arc() {
    return {{"_type", "EllipticArc"},
            {"arc",
             {{"centerX", 0.},
              {"centerY", 0.},
              {"centerZ", 0.},
              {"vector0X", 2.},
              {"vector0Y", 0.},
              {"vector0Z", 0.},
              {"vector90X", 0.},
              {"vector90Y", 1.},
              {"vector90Z", 0.},
              {"startRadians", 0.},
              {"sweepRadians", 1.}}}};
}
Json spline() {
    return {{"_type", "BsplineCurve"}, {"order", 2},
            {"closed", false},         {"poles", {0., 0., 0., 1., 0., 0.}},
            {"weights", {1., -2.}},    {"knots", {0., 0., 1., 1.}}};
}
Json validate(const Json &profile, const Json &path) {
    TubeBudget b;
    return validate_tube_sources(profile, path, b);
}
} // namespace
unsigned native_tube_validation_tests() {
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
    const auto path = group({line()}), profile = group({arc()});
    const auto valid = validate(profile, path);
    check(valid["accepted"] == true && valid["geometry_valid"] == true &&
              valid["numeric_valid"] == true && valid["failure_path"].is_null() &&
              valid["geometry_visits"] == 4 && valid["numeric_visits"] == 4 &&
              valid["scalar_checks"] == 17,
          "native validators traverse both sources in two passes");
    for (bool bad_path : {true, false}) {
        for (const auto &bad : {Json(nullptr), group(Json::array()), group({nullptr}),
                                group({group(Json::array())})}) {
            const auto r = bad_path ? validate(profile, bad) : validate(bad, path);
            check(r["accepted"] == false && r["geometry_valid"] == false &&
                      r["numeric_valid"].is_null() && r["numeric_visits"] == 0 &&
                      r["failure_path"].get<std::string>().find(bad_path ? "/path" : "/profile") ==
                          0,
                  "null/empty/nested-empty groups stop geometry before numeric validation");
        }
    }
    const double nan = std::numeric_limits<double>::quiet_NaN();
    auto bad_line = line();
    bad_line["segment"]["point0X"] = nan;
    const auto priority = validate(group(Json::array()), group({bad_line}));
    check(priority["failure_path"] == "/profile" && priority["numeric_visits"] == 0,
          "all geometry checks precede path numeric checks");
    for (const auto &primitive : {line(), arc()}) {
        const std::string key = primitive["_type"] == "LineSegment" ? "segment" : "arc";
        for (auto i = primitive.at(key).begin(); i != primitive.at(key).end(); ++i) {
            for (double bad : {nan, std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity()}) {
                auto changed = primitive;
                changed[key][i.key()] = bad;
                const auto r = validate(profile, group({changed}));
                check(r["geometry_valid"] == true && r["numeric_valid"] == false &&
                          r["failure_path"] == "/path/curves/0/geometry/" + key + "/" + i.key(),
                      "every native fixed-detail scalar is checked without evaluating geometry");
            }
        }
    }
    for (const char *type : {"LineString", "PointString", "AkimaCurve"}) {
        Json primitive{{"_type", type}, {"points", Json::array()}};
        check(validate(profile, group({primitive}))["geometry_valid"] == false,
              "point-based native geometry requires at least one stored point");
        primitive["points"] = {0., 0., 0.};
        check(validate(profile, group({primitive}))["accepted"] == true,
              "native validation does not impose conversion minimum point counts");
        for (unsigned k = 0; k < 3; ++k) {
            auto bad = primitive;
            bad["points"][k] = nan;
            check(validate(profile, group({bad}))["numeric_valid"] == false,
                  "stored point components are checked independently");
        }
    }
    for (const char *key : {"poles", "weights", "knots"}) {
        const auto c = spline();
        for (std::size_t i = 0; i < c.at(key).size(); ++i) {
            auto bad = c;
            bad[key][i] = nan;
            const auto r = validate(profile, group({bad}));
            check(r["geometry_valid"] == true && r["numeric_valid"] == false &&
                      r["failure_path"] ==
                          "/path/curves/0/geometry/" + std::string(key) + "/" + std::to_string(i),
                  "B-spline validity checks all stored homogeneous controls weights and knots");
        }
    }
    auto c = spline();
    c["weights"] = {0., -2.};
    c["knots"] = {1., 0., -1., -2.};
    c["poles"][0] = std::numeric_limits<double>::max();
    const auto original = c;
    check(validate(profile, group({c}))["accepted"] == true && c == original,
          "finite invalid-order knots and signed/zero weights pass without division or repairs");
    c["weights"] = nullptr;
    c["knots"] = nullptr;
    check(validate(profile, group({c}))["accepted"] == true,
          "implicit weights and knots are not treated as null geometry");
    c["poles"] = Json::array();
    check(validate(profile, group({c}))["accepted"] == true,
          "B-spline validation is not a control-count or evaluation certificate");
    auto zero_line = line();
    zero_line["segment"]["point1Z"] = 0.;
    auto zero_arc = arc();
    for (auto &v : zero_arc["arc"])
        v = 0.;
    check(validate(group({zero_arc}), group({zero_line}))["accepted"] == true,
          "finite degenerate line and arc are not rejected by native validation");
    for (int boundary : {0, 1, 2, 3, 4, 5, -1, 200})
        check(validate(group({line()}, boundary), path)["accepted"] == true,
              "validity dispatch does not enforce boundary conversion policy");
    auto nested = group({group({line(), line()}, 3), group({arc()}, 2)}, 4);
    const auto saved = nested;
    const auto multi = validate(nested, path);
    check(multi["accepted"] == true && multi["geometry_visits"] == 8 && nested == saved,
          "nested rings retain source order and duplicates without mutation");
    auto job = std::async(std::launch::async, [&] { return validate(nested, path); });
    check(job.get() == multi, "independent source validation is deterministic across threads");
    TubeBudget integrated;
    auto surface = prepare_swept_tube_surfaces(profile, path, integrated);
    check(surface.report["source_validation"] == valid,
          "sweep preparation executes and preserves the source validation report");
    try {
        TubeBudget b;
        prepare_swept_tube_surfaces(group(Json::array()), group({bad_line}), b);
        check(false, "sweep preparation must reject empty profile");
    } catch (const std::exception &e) {
        check(std::string(e.what()) == "native swept source validation failed at /profile",
              "preparation rejects source geometry before attempting path conversion");
    }
    rejects(
        [&] {
            TubeBudget b;
            b.max_work = multi["work_used"].get<std::size_t>() - 1;
            validate_tube_sources(nested, path, b);
        },
        "validation obeys shared work budget");
    rejects(
        [&] {
            TubeBudget b;
            b.max_control_points = 1;
            validate_tube_sources(group({spline()}), path, b);
        },
        "validation limits source arrays");
    auto deep = line();
    for (unsigned i = 0; i < 82; ++i)
        deep = group({deep});
    rejects([&] { validate(deep, path); }, "validation rejects excessive source nesting");
    rejects([&] { validate(group({{{"_type", "InterpolationCurve"}}}), path); },
            "unimplemented numeric dispatch is not reported valid");
    auto malformed = spline();
    malformed["poles"] = {0., 1.};
    rejects([&] { validate(group({malformed}), path); }, "malformed XYZ is a layout error");
    return count;
}
