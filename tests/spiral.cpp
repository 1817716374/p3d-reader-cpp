#include "blob_internal.hpp"
#include <future>
using namespace p3d;
namespace {
const std::array<const char *, 12> matrix_names{"axx", "axy", "axz", "axw", "ayx", "ayy",
                                                "ayz", "ayw", "azx", "azy", "azz", "azw"};
Json source(int type = 10) {
    Json matrix;
    for (unsigned i = 0; i < 12; ++i)
        matrix[matrix_names[i]] = i == 0 || i == 5 || i == 10 ? 1. : 0.;
    return {{"_type", "TransitionSpiral"},
            {"detail",
             {{"transform", matrix},
              {"fractionA", 0.},
              {"fractionB", 1.},
              {"bearing0Radians", .3},
              {"bearing1Radians", 1.32},
              {"curvature0", .2},
              {"curvature1", 1.},
              {"spiralType", type},
              {"constructionHint", 7}}},
            {"extraData", {3, 9}},
            {"directDetail", nullptr}};
}
bool near(Point3 a, Point3 b, double e = 1e-9) {
    return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) <= e;
}
template <class T> void write(Bytes &b, std::size_t p, T v) {
    std::memcpy(b.data() + p, &v, sizeof(v));
}
Bytes packet(const Json &input) {
    Bytes b(240);
    std::memcpy(b.data(), "bg0001fb", 8);
    write<std::uint32_t>(b, 8, 12); // root table at 20
    write<std::uint16_t>(b, 12, 8);
    write<std::uint16_t>(b, 14, 12);
    write<std::uint16_t>(b, 16, 4);
    write<std::uint16_t>(b, 18, 8);
    write<std::int32_t>(b, 20, 8);
    b[24] = 17;
    write<std::uint32_t>(b, 28, 20);
    write<std::uint16_t>(b, 32, 10);
    write<std::uint16_t>(b, 34, 168);
    write<std::uint16_t>(b, 36, 8);
    write<std::uint16_t>(b, 38, 160); // direct detail omitted
    write<std::int32_t>(b, 48, 16);
    const auto &d = input.at("detail");
    for (unsigned i = 0; i < 12; ++i)
        write<double>(b, 56 + 8 * i, d["transform"][matrix_names[i]].get<double>());
    const std::array<const char *, 6> doubles{"fractionA",       "fractionB",  "bearing0Radians",
                                              "bearing1Radians", "curvature0", "curvature1"};
    for (unsigned i = 0; i < 6; ++i)
        write<double>(b, 152 + 8 * i, d[doubles[i]].get<double>());
    write<std::int32_t>(b, 200, d["spiralType"].get<int>());
    write<std::int32_t>(b, 204, d["constructionHint"].get<int>());
    write<std::uint32_t>(b, 208, 12);
    write<std::uint32_t>(b, 220, 2);
    write<double>(b, 224, 3);
    write<double>(b, 232, 9);
    return b;
}
} // namespace
unsigned spiral_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto rejects = [&](const Json &v, const char *message) {
        bool failed = false;
        try {
            TransitionSpiral::from_bgfb(v);
        } catch (const std::exception &) {
            failed = true;
        }
        check(failed, message);
    };
    // Independent high precision integration reference for varying curvature.
    const std::array<Point3, 5> golden{{{1.2486373378673545, 1.0403167215726249, 0},
                                        {1.2714346048204521, 1.0101641924711157, 0},
                                        {1.2769233921518272, 1.0023976481326069, 0},
                                        {1.2731889183670382, 1.0077188513606445, 0},
                                        {1.2827961282703088, .9940204594498421, 0}}};
    for (int type = 10; type <= 14; ++type) {
        const auto input = source(type);
        const auto s = TransitionSpiral::from_bgfb(input);
        check(s.source() == input && std::abs(s.length() - 1.7) < 1e-14,
              "spiral source and native length preserved");
        const auto endpoint = s.evaluate(1, 1e-9);
        const auto native = s.native_fit_input();
        check(native["stroke_intervals"] == 26 && native["local_points"].size() == 27 &&
                  native["fits_native_point_limit"] == true &&
                  native["bspline_fit_status"] == "not_evaluated",
              "native spiral cache input uses angular count and retains fitter status");
        check(near(native["local_points"].back().get<Point3>(), endpoint.point, 1e-9) &&
                  native["endpoint_radii"] == Json({5., 1.}),
              "native Gauss4 Richardson points agree with independently bounded underlying "
              "integration");
        check(endpoint.quadrature_error_bound <= 1e-9 && endpoint.intervals > 0,
              "integration meets transformed truncation bound");
        check(near(endpoint.point, golden[type - 10], endpoint.quadrature_error_bound + 1e-13),
              "five native profiles match independent high precision integration");
        check(std::abs(endpoint.local_bearing - 1.32) < 1e-14 &&
                  std::abs(endpoint.local_curvature - 1) < 1e-14,
              "native endpoint bearing and curvature");
        const auto mid = s.evaluate(.5, 1e-9);
        const double h = 1e-5;
        const auto a = s.evaluate(.5 - h, 1e-12), b = s.evaluate(.5 + h, 1e-12);
        Point3 difference{};
        for (unsigned k = 0; k < 3; ++k)
            difference[k] = (b.point[k] - a.point[k]) / (2 * h);
        check(near(difference, mid.derivative, 1e-7),
              "evaluated spiral derivative matches point differences");
        auto signed_source = input;
        signed_source["detail"]["curvature0"] = -.2;
        signed_source["detail"]["curvature1"] = -1.;
        check(near(TransitionSpiral::from_bgfb(signed_source).evaluate(1).point, endpoint.point,
                   1e-8),
              "native factory ignores source curvature signs");
        auto reverse = input;
        reverse["detail"]["fractionA"] = 1.;
        reverse["detail"]["fractionB"] = 0.;
        const auto reversed = TransitionSpiral::from_bgfb(reverse);
        const auto native_reverse = reversed.native_fit_input();
        check(near(native_reverse["local_points"][0].get<Point3>(), endpoint.point, 1e-9) &&
                  near(native_reverse["local_points"].back().get<Point3>(), {0, 0, 0}, 1e-9),
              "native reverse samples add the independently stroked origin offset");
        check(
            std::abs(native_reverse["endpoint_bearings"][0].get<double>() -
                     (1.32 + std::acos(-1.))) < 1e-12 &&
                std::abs(native_reverse["endpoint_bearings"][1].get<double>() -
                         (.3 + std::acos(-1.))) < 1e-12 &&
                native_reverse["endpoint_radii"] == Json({1., 5.}),
            "native fitter aligns bearings to chords but does not negate reversed endpoint radii");
        check(near(reversed.evaluate(0, 1e-9).point, endpoint.point) &&
                  near(reversed.evaluate(1).point, {0, 0, 0}),
              "active interval reversal retains full spiral origin");
        Point3 negative{};
        for (unsigned k = 0; k < 3; ++k)
            negative[k] = -mid.derivative[k];
        check(near(reversed.evaluate(.5).derivative, negative),
              "active fraction derivative changes sign on reversal");
        auto circle = input;
        circle["detail"]["bearing0Radians"] = 0.;
        circle["detail"]["bearing1Radians"] = 1.;
        circle["detail"]["curvature0"] = .25;
        circle["detail"]["curvature1"] = .25;
        const auto arc = TransitionSpiral::from_bgfb(circle);
        for (unsigned i = 0; i <= 20; ++i) {
            const double t = i / 20.;
            check(near(arc.evaluate(t, 1e-10).point, {4 * std::sin(t), 4 * (1 - std::cos(t)), 0},
                       1e-10),
                  "constant curvature yields analytic circle for every spiral profile");
        }
        circle["detail"]["bearing1Radians"] = -1.;
        const auto clockwise = TransitionSpiral::from_bgfb(circle).evaluate(1, 1e-10);
        check(near(clockwise.point, {4 * std::sin(1.), -4 * (1 - std::cos(1.)), 0}, 1e-10) &&
                  clockwise.local_curvature == -.25,
              "decreasing bearing determines both effective curvature signs");
        auto decoded = decode_bgfb(packet(input))["geometry"];
        check(
            decoded["_spiral"]["status"] == "valid" && decoded["extraData"] == Json({3, 9}) &&
                decoded["detail"]["constructionHint"] == 7 && decoded["directDetail"].is_null() &&
                near(TransitionSpiral::from_bgfb(decoded).evaluate(1, 1e-9).point, endpoint.point),
            "native BGFB type17 packet reaches evaluator and retains ignored fields");
    }
    auto v = source();
    v["detail"]["fractionA"] = -.4;
    v["detail"]["fractionB"] = 1.4;
    const auto extended = TransitionSpiral::from_bgfb(v);
    check(std::abs(extended.evaluate(0).local_curvature + .12) < 1e-14 &&
              std::abs(extended.evaluate(1).local_curvature - 1.32) < 1e-14,
          "active source interval is not clamped or wrapped");
    v["detail"]["spiralType"] = 12;
    const auto quadratic_extended = TransitionSpiral::from_bgfb(v);
    check(near(quadratic_extended.evaluate(0, 1e-10).point,
               {-.6629848081014693, -.14651848718833245, 0}, 1e-10) &&
              near(quadratic_extended.evaluate(1, 1e-10).point,
                   {1.2269435559610768, 1.6694014435946751, 0}, 1e-10),
          "piecewise curvature integrates negative and extended source intervals without aliasing");
    v = source();
    v["detail"]["fractionA"] = .7;
    v["detail"]["fractionB"] = .7;
    const auto singleton = TransitionSpiral::from_bgfb(v).evaluate(.2);
    check(near(singleton.point, TransitionSpiral::from_bgfb(source()).evaluate(.7).point) &&
              singleton.derivative == Point3{},
          "equal active limits retain the located point with zero active derivative");
    const auto invalid_packet = decode_bgfb(packet(source(55)))["geometry"];
    check(invalid_packet["_spiral"]["status"] == "invalid" &&
              invalid_packet["detail"]["spiralType"] == 55 &&
              invalid_packet["extraData"] == Json({3, 9}),
          "unsupported spiral factory type retains complete decoded source and explicit error");
    v = source();
    v["detail"]["transform"]["axx"] = 2.;
    v["detail"]["transform"]["axy"] = .4;
    v["detail"]["transform"]["axw"] = 10.;
    v["detail"]["transform"]["ayx"] = 1.;
    v["detail"]["transform"]["ayy"] = -3.;
    v["detail"]["transform"]["ayw"] = -20.;
    v["detail"]["transform"]["azx"] = 5.;
    v["detail"]["transform"]["azy"] = -2.;
    v["detail"]["transform"]["azw"] = 30.;
    const auto local = TransitionSpiral::from_bgfb(source()).evaluate(.7, 1e-12);
    const auto world = TransitionSpiral::from_bgfb(v).evaluate(.7, 1e-10);
    check(near(world.point,
               {2 * local.point[0] + .4 * local.point[1] + 10,
                local.point[0] - 3 * local.point[1] - 20,
                5 * local.point[0] - 2 * local.point[1] + 30},
               1e-10),
          "affine spiral transform preserves scale, shear, reflection and translation");
    check(world.quadrature_error_bound <= 1e-10, "quadrature accounts for affine XY magnification");
    v = source();
    v["detail"]["bearing1Radians"] = v["detail"]["bearing0Radians"];
    const auto zero = TransitionSpiral::from_bgfb(v).evaluate(1);
    check(zero.point == Point3{} && zero.derivative == Point3{} && zero.intervals == 0 &&
              zero.local_curvature == -.2,
          "zero length native base evaluates at origin with initial signed curvature");
    for (int type : {0, 9, 15, 55}) {
        v = source(type);
        rejects(v, "unknown native spiral factory type rejected");
    }
    v = source();
    v["detail"]["curvature0"] = 0;
    v["detail"]["curvature1"] = 0;
    rejects(v, "undefined native length rejected");
    v = source();
    v["detail"]["curvature0"] = 1e-30;
    v["detail"]["curvature1"] = 1e-30;
    rejects(v, "native bearing curvature limit threshold enforced");
    v = source();
    v["detail"]["fractionB"] = std::numeric_limits<double>::infinity();
    rejects(v, "nonfinite source fraction rejected");
    const auto s = TransitionSpiral::from_bgfb(source());
    bool failed = false;
    try {
        s.evaluate(1, 1e-14, 1);
    } catch (const std::exception &) {
        failed = true;
    }
    check(failed, "insufficient integration budget does not return an unbounded approximation");
    failed = false;
    try {
        s.evaluate(2);
    } catch (const std::exception &) {
        failed = true;
    }
    check(failed, "API fraction is checked separately from source active interval");
    v = source();
    v["detail"]["bearing0Radians"] = 0.;
    v["detail"]["bearing1Radians"] = .080000000001;
    check(TransitionSpiral::from_bgfb(v).native_fit_input()["stroke_intervals"] == 2,
          "native angular count retains its near-integer downward bias");
    v["detail"]["bearing1Radians"] = .0800000001;
    check(TransitionSpiral::from_bgfb(v).native_fit_input()["stroke_intervals"] == 4,
          "native positive angular count rounds upward to an even interval count");
    v["detail"]["bearing1Radians"] = 0.;
    const auto collapsed = TransitionSpiral::from_bgfb(v).native_fit_input();
    check(
        collapsed["stroke_intervals"] == 1 &&
            collapsed["local_points"] == Json({Point3{}, Point3{}}) &&
            collapsed["source_fractions"] == Json({0., 1.}),
        "native zero-angle minimum interval retains both generated samples without deduplication");
    v = source();
    v["detail"]["bearing0Radians"] = 0.;
    v["detail"]["bearing1Radians"] = 40.;
    const auto large = TransitionSpiral::from_bgfb(v).native_fit_input();
    check(large["local_points"].size() == 1001 && large["fits_native_point_limit"] == false,
          "stroke buffer and downstream fitter point limits are independent");
    v["detail"]["bearing1Radians"] = 80.;
    failed = false;
    try {
        TransitionSpiral::from_bgfb(v).native_fit_input();
    } catch (const std::exception &) {
        failed = true;
    }
    check(failed, "native 2000 point stroke buffer includes the initial point");
    failed = false;
    try {
        s.native_fit_input(26);
    } catch (const std::exception &) {
        failed = true;
    }
    check(failed, "caller work budget includes origin integration as well as main stroke");
    for (int type = 10; type <= 14; ++type) {
        const auto spiral = TransitionSpiral::from_bgfb(source(type));
        const auto fit = spiral.native_fit();
        const auto &report = fit.report;
        check(report["iterations"] <= 30 && report["iterations"] > 0 &&
                  report["max_interval_length_change"] < report["convergence_threshold"] &&
                  report["convergence_is_geometric_error_bound"] == false,
              "spiral iterative fit meets native parameter convergence criterion");
        const auto prepared = report["expanded_local_points"].get<std::vector<Point2>>();
        const auto parameters = report["expanded_parameters"].get<std::vector<double>>();
        check(prepared.size() == 29 && fit.curve.poles().size() == 31 &&
                  fit.curve.knots().size() == 35 && fit.curve.order() == 4,
              "native spiral fit preserves two auxiliary nodes and cubic pole ordering");
        for (std::size_t i = 0; i < prepared.size(); ++i)
            check(
                near(fit.curve.point_at(parameters[i]), {prepared[i][0], prepared[i][1], 0}, 1e-12),
                "converted spiral B-spline interpolates every expanded local point");
        const auto samples = spiral.native_fit_input()["local_points"].get<std::vector<Point3>>();
        for (std::size_t i = 1; i + 1 < samples.size(); ++i)
            check(near(fit.curve.point_at(parameters[i + 1]), samples[i], 1e-12),
                  "native source samples retain their positions after arc length iteration");
        const auto &p = fit.curve.poles();
        const auto &knots = fit.curve.knots();
        const double h = knots[4], hnext = knots[5] - knots[4];
        Point2 d{}, dd{};
        for (unsigned k = 0; k < 2; ++k) {
            d[k] = 3 * (p[1][k] - p[0][k]) / h;
            dd[k] = 6 / h * ((p[2][k] - p[1][k]) / (h + hnext) - (p[1][k] - p[0][k]) / h);
        }
        const double speed = std::hypot(d[0], d[1]);
        check(std::abs(std::atan2(d[1], d[0]) - .3) < 1e-11 &&
                  std::abs((d[0] * dd[1] - d[1] * dd[0]) / (speed * speed * speed) - .2) < 1e-9,
              "native fitted poles preserve start bearing and curvature constraints");
    }
    v = source();
    v["detail"]["curvature0"] = .5;
    v["detail"]["curvature1"] = .5;
    const auto circle = TransitionSpiral::from_bgfb(v);
    const auto circle_fit = circle.native_fit();
    for (unsigned i = 0; i <= 50; ++i)
        check(near(circle_fit.curve.point_at(i / 50.), circle.evaluate(i / 50., 1e-11).point, 1e-6),
              "native constant curvature fit agrees with independently integrated circular arc");
    const auto untransformed = s.native_fit();
    v = source();
    v["detail"]["transform"]["axx"] = 2.;
    v["detail"]["transform"]["axy"] = .4;
    v["detail"]["transform"]["axw"] = 10.;
    v["detail"]["transform"]["ayx"] = 1.;
    v["detail"]["transform"]["ayy"] = -3.;
    v["detail"]["transform"]["azx"] = 5.;
    const auto transformed = TransitionSpiral::from_bgfb(v).native_fit();
    check(transformed.report == untransformed.report &&
              transformed.curve.knots() == untransformed.curve.knots(),
          "affine transform does not alter local arc length fit or convergence");
    // High precision dense collocation references. In particular the native
    // three-point case overwrites a boundary row, and reverse fitting retains
    // the source curvature signs; neither is a reversal of the forward poles.
    const std::array<Point3, 4> short_mid{{{.007957700090909245, .00247372025197351, 0},
                                           {.007957722647741739, .0024737272913157088, 0},
                                           {.06346443007144283, .020407181328549653, 0},
                                           {.06347302642506665, .020410008222561137, 0}}};
    for (unsigned i = 0; i < short_mid.size(); ++i) {
        auto input = source();
        input["detail"]["bearing1Radians"] = i < 2 ? .31 : .38;
        if (i & 1) {
            input["detail"]["fractionA"] = 1.;
            input["detail"]["fractionB"] = 0.;
        }
        const auto fitted = TransitionSpiral::from_bgfb(input).native_fit();
        check(fitted.report["source_point_count"] == 3 &&
                  fitted.report["iterations"] == (i == 0 ? 1 : 2) &&
                  near(fitted.curve.point_at(.5), short_mid[i], 1e-13),
              "short forward and reverse spiral fits agree with dense high precision reference");
    }
    auto two_source = source();
    two_source["detail"]["fractionA"] = -.5;
    two_source["detail"]["fractionB"] = 0.;
    const auto two_fit = TransitionSpiral::from_bgfb(two_source).native_fit();
    check(two_fit.report["source_point_count"] == 2 && two_fit.report["iterations"] == 11 &&
              near(two_fit.curve.point_at(.5), {-.3754041580229958, -.10508754307469105, 0}, 1e-12),
          "two-point fit keeps sequential auxiliary resets and native partial arc update");
    for (unsigned i = 0; i <= 10; ++i) {
        const auto p = untransformed.curve.point_at(i / 10.);
        check(near(transformed.curve.point_at(i / 10.),
                   {2 * p[0] + .4 * p[1] + 10, p[0] - 3 * p[1], 5 * p[0]}, 1e-12),
              "source affine transform is applied after native pole fitting");
    }
    auto fit_rejects = [&](const Json &input, const char *message) {
        bool bad = false;
        try {
            TransitionSpiral::from_bgfb(input).native_fit();
        } catch (const std::exception &) {
            bad = true;
        }
        check(bad, message);
    };
    v = source();
    v["detail"]["fractionB"] = 0.;
    fit_rejects(v, "native fit rejects zero active interval without inventing a fallback curve");
    v = source();
    v["detail"]["bearing1Radians"] = .3;
    fit_rejects(v, "zero length underlying spiral does not imply a valid fitted B-spline");
    v = source();
    v["detail"]["bearing1Radians"] = 40.;
    v["detail"]["bearing0Radians"] = 0.;
    fit_rejects(v, "native fit independently enforces source point limit");
    auto variant = [](const Json &geometry) {
        return Json{{"_type", "VariantGeometry"}, {"geometry", geometry}};
    };
    auto line = [](Point3 a, Point3 b) {
        return Json{{"_type", "LineSegment"},
                    {"segment",
                     {{"point0X", a[0]},
                      {"point0Y", a[1]},
                      {"point0Z", a[2]},
                      {"point1X", b[0]},
                      {"point1Y", b[1]},
                      {"point1Z", b[2]}}}};
    };
    auto boundary = [](int type, const Json &curves) {
        return Json{{"_type", "CurveVector"}, {"type", type}, {"curves", curves}};
    };
    auto surface = [](const Json &curves) {
        return BsplineSurface::from_bgfb({{"_type", "BsplineSurface"},
                                          {"numPolesU", 2},
                                          {"numPolesV", 2},
                                          {"orderU", 2},
                                          {"orderV", 2},
                                          {"closedU", false},
                                          {"closedV", false},
                                          {"poles", {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0}},
                                          {"weights", nullptr},
                                          {"knotsU", nullptr},
                                          {"knotsV", nullptr},
                                          {"holeOrigin", 1},
                                          {"numRulesU", 0},
                                          {"numRulesV", 0},
                                          {"boundaries", curves}});
    };
    for (int type = 10; type <= 14; ++type)
        for (bool reversed : {false, true}) {
            auto input = source(type);
            auto &detail = input["detail"];
            detail["bearing0Radians"] = 0.;
            detail["bearing1Radians"] = 3.14159265358979323846 / 2;
            detail["curvature0"] = detail["curvature1"] = 1.;
            detail["transform"]["axx"] = detail["transform"]["ayy"] = .4;
            detail["transform"]["axw"] = detail["transform"]["ayw"] = .2;
            if (reversed) {
                detail["fractionA"] = 1.;
                detail["fractionB"] = 0.;
            }
            const auto decoded = decode_bgfb(packet(input))["geometry"];
            const auto fitted = TransitionSpiral::from_bgfb(decoded).native_fit();
            const auto a = fitted.curve.point_at(0), b = fitted.curve.point_at(1);
            const auto source_boundary = boundary(1, {variant(decoded), variant(line(b, a))});
            const auto surf = surface(source_boundary);
            const auto trim = surf.trim(1e-5);
            check(surf.boundaries() == source_boundary && trim.report()["status"] == "complete" &&
                      trim.loops().size() == 1,
                  "decoded spiral cache closes an Open trim loop without changing source data");
            check(trim.classify({.45, .35}) == TrimLocation::Inside &&
                      trim.classify({.3, .5}) == TrimLocation::Outside,
                  "fitted quarter-circle and chord trim region classifies independent points");
            const auto &report = trim.report();
            check(report["curve_conversions"].size() == 1 &&
                      report["curve_conversions"][0]["source_path"] == "/curves/0/geometry" &&
                      report["curve_conversions"][0]["representation"] == "native_fitted_bspline" &&
                      report["bounds_underlying_spiral_error"] == false,
                  "trim provenance identifies fitted spiral and excludes underlying fit error");
            Json flat = Json::array();
            for (const auto &p : fitted.curve.poles())
                for (double x : p)
                    flat.push_back(x);
            const Json explicit_curve{{"_type", "BsplineCurve"},
                                      {"order", 4},
                                      {"closed", false},
                                      {"poles", flat},
                                      {"knots", fitted.curve.knots()},
                                      {"weights", nullptr}};
            const auto explicit_trim =
                surface(boundary(1, {variant(explicit_curve), variant(line(b, a))})).trim(1e-5);
            check(explicit_trim.loops() == trim.loops() &&
                      explicit_trim.report()["loops"] == report["loops"],
                  "spiral trim conversion and endpoint check use the same fitted B-spline");
            const auto open = surface(boundary(1, Json::array({variant(decoded)}))).trim(1e-5);
            check(open.report()["status"] == "complete" && open.loops().empty() &&
                      open.report()["ignored"][0]["reason"] == "open_boundary_not_closed",
                  "unclosed spiral remains subject to native source-tree closure filtering");
            const auto limited = surf.trim(1e-5, 1);
            check(limited.report()["status"] == "incomplete" &&
                      limited.classify({.45, .35}) == TrimLocation::Indeterminate,
                  "spiral trim preserves segment budget failure and indeterminate queries");
            if (type == 10 && !reversed) {
                const auto duplicates =
                    surface(boundary(4, {variant(source_boundary), variant(source_boundary)}))
                        .trim(1e-5);
                check(duplicates.loops().size() == 2 &&
                          duplicates.report()["curve_conversions"].size() == 2 &&
                          duplicates.classify({.45, .35}) == TrimLocation::Outside,
                      "equal but independent spiral boundary records retain native parity");
                auto first = std::async(std::launch::async, [&surf] { return surf.trim(1e-5); });
                auto second = std::async(std::launch::async, [&surf] { return surf.trim(1e-5); });
                const auto r1 = first.get(), r2 = second.get();
                check(r1.loops() == trim.loops() && r2.loops() == trim.loops() &&
                          r1.report() == trim.report() && r2.report() == trim.report(),
                      "concurrent trim calls on one surface keep independent deterministic caches");
            }
        }
    v = source();
    v["detail"]["fractionB"] = 0.;
    const auto invalid_trim = surface(boundary(2, Json::array({variant(v)}))).trim(1e-5);
    check(invalid_trim.report()["status"] == "incomplete" &&
              invalid_trim.classify({.5, .5}) == TrimLocation::Indeterminate &&
              invalid_trim.report()["errors"][0]["error"].get<std::string>().find(
                  "native spiral fit") != std::string::npos,
          "failed native spiral fit does not become a successful trim or fallback polyline");
    return checks;
}
