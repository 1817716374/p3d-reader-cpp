#include "blob_internal.hpp"
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
    return checks;
}
