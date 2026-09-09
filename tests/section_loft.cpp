#include "loft_curve.hpp"
#include "guided.hpp"
#include "blob_internal.hpp"
#include <cstring>
#include <future>
#include <random>
using namespace p3d;
namespace {
bool near(Point3 a, Point3 b, double e = 2e-10) {
    return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) < e;
}
Json curve(unsigned order, Json xyz, Json knots = nullptr, Json weights = nullptr) {
    return {{"_type", "BsplineCurve"}, {"order", order},    {"closed", false}, {"poles", xyz},
            {"knots", knots},          {"weights", weights}};
}
Json array(std::vector<Json> geometries, int type = 1) {
    Json entries = Json::array();
    for (auto &g : geometries)
        entries.push_back({{"geometry", g}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", entries}};
}
Json line(Point3 a, Point3 b) {
    return curve(2, {a[0], a[1], a[2], b[0], b[1], b[2]});
}
Json source() {
    const Json guides =
        Json::array({array({line({0, 0, 0}, {0, 0, 2})}), array({line({3, 0, 0}, {3, 0, 2})})});
    return {{"_type", "P3DSectionLoft"},
            {"capped", false},
            {"section0",
             array({curve(3, {0, 0, 0, 1, 1, 0, 2, -1, 0, 3, 0, 0}, {0, 0, 0, .3, 1, 1, 1})})},
            {"section1", array({curve(4, {0, 0, 2, .5, -1, 2, 1.5, 2, 2, 2.5, 1, 2, 3, 0, 2},
                                      {0, 0, 0, 0, .6, 1, 1, 1, 1})})},
            {"guide_groups", Json::array({guides})}};
}
// Forward FlatBuffer encoder for this test's actual P3D union and B-spline tables.
struct Wire {
    Bytes b = Bytes(12);
    template <class T> void write(std::size_t p, T v) {
        std::memcpy(b.data() + p, &v, sizeof v);
    }
    std::size_t allocate(std::size_t n) {
        while (b.size() % 8)
            b.push_back(0);
        const auto p = b.size();
        b.resize(p + n);
        return p;
    }
    std::size_t table(unsigned n) {
        auto vt = allocate(4 + 2 * n), t = allocate(4 + 4 * n);
        write<std::uint16_t>(vt, std::uint16_t(4 + 2 * n));
        write<std::uint16_t>(vt + 2, std::uint16_t(4 + 4 * n));
        for (unsigned i = 0; i < n; ++i)
            write<std::uint16_t>(vt + 4 + 2 * i, std::uint16_t(4 + 4 * i));
        write<std::int32_t>(t, std::int32_t(t - vt));
        return t;
    }
    void reference(std::size_t f, std::size_t t) {
        write<std::uint32_t>(f, std::uint32_t(t - f));
    }
    std::size_t refs(std::size_t n) {
        auto p = allocate(4 + 4 * n);
        write<std::uint32_t>(p, std::uint32_t(n));
        return p;
    }
    std::size_t numbers(const Json &a) {
        auto p = allocate(4 + 8 * a.size());
        write<std::uint32_t>(p, std::uint32_t(a.size()));
        for (std::size_t i = 0; i < a.size(); ++i)
            write<double>(p + 4 + 8 * i, a[i].get<double>());
        return p;
    }
    std::size_t geometry(const Json &v) {
        auto t = table(2);
        if (v.at("_type") == "BsplineCurve") {
            b[t + 4] = 3;
            reference(t + 8, spline(v));
        } else {
            const bool arc = v.at("_type") == "EllipticArc";
            b[t + 4] = arc ? 2 : 1;
            const std::vector<std::string> keys =
                arc ? std::vector<std::string>{"centerX",      "centerY",     "centerZ",
                                               "vector0X",     "vector0Y",    "vector0Z",
                                               "vector90X",    "vector90Y",   "vector90Z",
                                               "startRadians", "sweepRadians"}
                    : std::vector<std::string>{"point0X", "point0Y", "point0Z",
                                               "point1X", "point1Y", "point1Z"};
            const auto vt = allocate(6), body = allocate(8 + 8 * keys.size());
            write<std::uint16_t>(vt, 6);
            write<std::uint16_t>(vt + 2, std::uint16_t(8 + 8 * keys.size()));
            write<std::uint16_t>(vt + 4, 8);
            write<std::int32_t>(body, std::int32_t(body - vt));
            reference(t + 8, body);
            for (std::size_t i = 0; i < keys.size(); ++i)
                write<double>(body + 8 + 8 * i,
                              v.at(arc ? "arc" : "segment").at(keys[i]).get<double>());
        }
        return t;
    }
    std::size_t spline(const Json &v) {
        auto t = table(5);
        write<std::int32_t>(t + 4, v.at("order").get<int>());
        b[t + 8] = v.at("closed").get<bool>();
        reference(t + 12, numbers(v.at("poles")));
        reference(t + 16, numbers(v.at("weights")));
        reference(t + 20, numbers(v.at("knots")));
        return t;
    }
    std::size_t curves(const Json &v) {
        auto t = table(2);
        write<std::int32_t>(t + 4, v.at("type").get<int>());
        const auto &a = v.at("curves");
        auto p = refs(a.size());
        reference(t + 8, p);
        for (std::size_t i = 0; i < a.size(); ++i)
            reference(p + 4 + 4 * i, geometry(a[i].at("geometry")));
        return t;
    }
    Bytes encode(const Json &v) {
        std::memcpy(b.data(), "bg0001fb", 8);
        auto root = table(2);
        reference(8, root);
        b[root + 4] = 21;
        auto t = table(4);
        reference(root + 8, t);
        reference(t + 4, curves(v.at("section0")));
        reference(t + 8, curves(v.at("section1")));
        const auto &groups = v.at("guide_groups");
        auto p = refs(groups.size());
        reference(t + 12, p);
        for (std::size_t i = 0; i < groups.size(); ++i) {
            auto q = refs(groups[i].size());
            reference(p + 4 + 4 * i, q);
            for (std::size_t j = 0; j < groups[i].size(); ++j)
                reference(q + 4 + 4 * j, curves(groups[i][j]));
        }
        b[t + 16] = v.at("capped").get<bool>();
        return b;
    }
};
} // namespace
unsigned section_loft_tests() {
    using loft_detail::Curve;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto rejects = [&](auto fn, const char *message) {
        bool threw = false;
        try {
            fn();
        } catch (const std::exception &) {
            threw = true;
        }
        check(threw, message);
    };
    std::mt19937 random(3401);
    std::uniform_real_distribution<double> coord(-2, 2), weight(.7, 1.3);
    for (unsigned order : {2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u, 26u}) {
        Json xyz = Json::array(), weights = Json::array();
        const unsigned count = order + 4;
        for (unsigned i = 0; i < count; ++i) {
            const double w = weight(random);
            weights.push_back(w);
            for (unsigned k = 0; k < 3; ++k)
                xyz.push_back(coord(random) * w);
        }
        auto input = curve(order, xyz, nullptr, weights);
        input["closed"] = true;
        const auto base = BsplineCurve::from_bgfb(input);
        for (double shift : {0., -.173, -1.}) {
            Json knots = Json::array();
            for (double k : base.knots())
                knots.push_back(k + shift);
            input["knots"] = knots;
            const auto original = BsplineCurve::from_bgfb(input);
            const auto opened = Curve::from_bspline(original, 1000);
            const auto eval = BsplineCurve::from_bgfb(opened.table());
            check(opened.poles.size() == count + order - (shift == -.173 ? 0 : 1),
                  "ordinary periodic opening preserves the native control count");
            Json closure;
            const auto reopened = loft_detail::close_reopen(opened, 1000, closure);
            const auto closed_eval = BsplineCurve::from_bgfb(reopened.table());
            check(reopened.poles.size() == opened.poles.size(),
                  "close/open retains the clamped control count across cyclic seam positions");
            if (order <= 6)
                check(closure["method"] ==
                          (order == 2 ? "linear_periodic_reopened" : "regular_periodic_reopened"),
                      "smooth periodic lifts take the native periodic pole-reduction path");
            for (unsigned i = 0; i <= 100; ++i) {
                const double f = i / 100.;
                double t = f - shift;
                while (t > 1)
                    t -= 1;
                check(near(eval.point_at(f), original.point_at(t), 3e-10),
                      "periodic opening preserves cyclic rational geometry and the absolute zero "
                      "seam");
                check(near(closed_eval.point_at(f), eval.point_at(f), 5e-9),
                      "native unclamping and reclosure preserve rational periodic geometry");
            }
        }
        input["knots"] = Json::array();
        for (double k : base.knots())
            input["knots"].push_back(k + 2);
        rejects([&] { Curve::from_bspline(BsplineCurve::from_bgfb(input), 1000); },
                "native zero-knot opening rejects a domain that excludes zero");
    }
    const double root3 = std::sqrt(3.) / 2;
    auto circle_table = curve(
        3, {1, 0, 0, .5, root3, 0, -.5, root3, 0, -1, 0, 0, -.5, -root3, 0, .5, -root3, 0, 1, 0, 0},
        {-1. / 3, 0, 0, 0, 1. / 3, 1. / 3, 2. / 3, 2. / 3, 1, 1, 1, 4. / 3},
        {1, .5, 1, .5, 1, .5, 1});
    circle_table["closed"] = true;
    const auto circle = BsplineCurve::from_bgfb(circle_table);
    const auto opened_circle = Curve::from_bspline(circle, 1000);
    Json closure;
    const auto reclosed_circle = loft_detail::close_reopen(opened_circle, 1000, closure);
    check(closure["method"] == "special_periodic_reopened" &&
              reclosed_circle.knots == opened_circle.knots &&
              reclosed_circle.poles == opened_circle.poles,
          "C0 rational circle falls back to the special closed knot layout without changing poles");
    for (unsigned order : {3u, 4u, 5u, 6u, 7u, 8u, 9u, 26u}) {
        Json xyz = Json::array(), w = Json::array();
        for (unsigned i = 0; i < order; ++i) {
            const double weight_value = i == 0 || i + 1 == order ? 1 : weight(random);
            w.push_back(weight_value);
            for (unsigned k = 0; k < 3; ++k)
                xyz.push_back(i == 0 || i + 1 == order ? 0. : coord(random) * weight_value);
        }
        const auto short_closed =
            Curve::from_bspline(BsplineCurve::from_bgfb(curve(order, xyz, nullptr, w)), order);
        const auto result = loft_detail::close_reopen(short_closed, order, closure);
        check(
            closure["method"] == "special_periodic_reopened" &&
                result.knots == short_closed.knots && result.poles == short_closed.poles,
            "short closed Bezier curves keep every homogeneous pole through odd/even knot padding");
    }
    auto box = Curve::from_bspline(
        BsplineCurve::from_bgfb(curve(3, {1, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, 0, 0, 0, 1, 0, 0},
                                      {0, 0, 0, .25, .5, .75, 1, 1, 1})),
        100);
    auto changed = box;
    changed.poles.back()[0] += 5e-11;
    auto recovered = loft_detail::close_reopen(changed, 100, closure);
    check(closure["method"] == "regular_periodic_reopened" && closure["closed_pole_count"] == 4 &&
              recovered.poles == box.poles,
          "accepted periodic overlap drops only the native redundant end poles");
    for (auto &h : changed.poles)
        for (unsigned k = 0; k < 3; ++k)
            h[k] -= 10;
    recovered = loft_detail::close_reopen(changed, 100, closure);
    check(closure["method"] == "special_periodic_reopened" && recovered.poles == changed.poles,
          "negative stored coordinates do not acquire an invented absolute overlap tolerance");
    changed = box;
    changed.rational = true;
    changed.poles.back()[3] += 7.5e-11;
    recovered = loft_detail::close_reopen(changed, 100, closure);
    check(closure["method"] == "regular_periodic_reopened" && recovered.poles == box.poles,
          "signed periodic overlap weight test accepts a larger trailing weight");
    changed.poles.back()[3] = 1 - 7.5e-11;
    recovered = loft_detail::close_reopen(changed, 100, closure);
    check(closure["method"] == "special_periodic_reopened" && recovered.poles == changed.poles,
          "signed periodic overlap weight test rejects the opposite difference");
    changed.poles.back()[3] = 1 + 2e-10;
    recovered = loft_detail::close_reopen(changed, 100, closure);
    check(closure["method"] == "retained_open" && closure["reason"] == "endpoint_weight_mismatch" &&
              recovered.poles == changed.poles,
          "failed native endpoint closure retains the input instead of aborting the guide");
    changed = box;
    changed.poles.back()[0] += 1e-6;
    recovered = loft_detail::close_reopen(changed, 100, closure);
    check(closure["method"] == "retained_open" &&
              closure["reason"] == "endpoint_position_mismatch" && recovered.poles == changed.poles,
          "endpoint closure tests stored XYZ before trying any periodic conversion");
    auto closed_line = Curve::from_bspline(
        BsplineCurve::from_bgfb(curve(2, {0, 0, 0, 1, 0, 0, 0, 0, 5e-11}, {0, 0, .3, 1, 1})), 3);
    recovered = loft_detail::close_reopen(closed_line, 3, closure);
    check(closure["method"] == "linear_periodic_reopened" && closure["closed_pole_count"] == 2 &&
              recovered.knots == closed_line.knots &&
              recovered.poles.back() == closed_line.poles.front(),
          "linear closure uses the first pole at the reopened end and preserves the internal knot");
    const auto straight =
        Curve::from_bspline(BsplineCurve::from_bgfb(line({0, 0, 0}, {2, 0, 0})), 2);
    recovered = loft_detail::close_reopen(straight, 2, closure);
    check(closure["reason"] == "two_pole_line" && recovered.poles == straight.poles,
          "two-pole line is a native no-op even with unequal endpoints");
    check(opened_circle.poles.size() == 7 &&
              BsplineCurve::from_bgfb(opened_circle.table()).poles() == circle.poles(),
          "special periodic conic strips exterior knots without duplicating or rotating poles");
    for (unsigned i = 0; i <= 90; ++i)
        check(near(BsplineCurve::from_bgfb(opened_circle.table()).point_at(i / 90.),
                   circle.point_at(i / 90.)),
              "special periodic conic keeps its full rational locus and source seam");
    auto imperfect = circle_table;
    imperfect["poles"][18] = 1 + 1e-6;
    check(Curve::from_bspline(BsplineCurve::from_bgfb(imperfect), 1000).poles.back()[0] == 1 + 1e-6,
          "native special seam tolerance preserves an accepted endpoint without snapping");
    imperfect["weights"][6] = 1 + 2e-10;
    Json fallback_note;
    const auto fallback =
        loft_detail::open_periodic(BsplineCurve::from_bgfb(imperfect), 1000, &fallback_note);
    check(fallback_note["method"] == "cyclic_seam_fallback" &&
              fallback_note["pole_rotation"] == 1 && fallback.poles.back()[0] == 1 &&
              fallback.poles[5][0] == 1 + 1e-6 && fallback.poles[5][3] == 1 + 2e-10 &&
              fallback.knots == opened_circle.knots,
          "failed special endpoint weight check takes the native cyclic fallback, retaining and "
          "rotating every stored pole and weight");
    for (unsigned order : {3u, 4u, 5u, 6u, 7u, 8u, 9u, 26u}) {
        const unsigned n = order + 3, shift = order / 2;
        Json xyz = Json::array(), w = Json::array();
        for (unsigned i = 0; i < n; ++i) {
            w.push_back(1 + .01 * i);
            xyz.push_back(double(i));
            xyz.push_back(double(i % 3));
            xyz.push_back(0);
        }
        auto stored = curve(order, xyz, nullptr, w);
        const auto clamped = BsplineCurve::from_bgfb(stored);
        std::vector<double> u(n + 2 * order - 1);
        std::copy_n(clamped.knots().begin() + (order + 1) / 2, n - 1, u.begin() + order);
        for (unsigned i = 0; i < order; ++i) {
            u[i] = u[i + n] - 1;
            u[n + order - 1 + i] = u[order - 1 + i] + 1;
        }
        stored["closed"] = true;
        stored["knots"] = u;
        const auto native =
            loft_detail::open_periodic(BsplineCurve::from_bgfb(stored), n, &fallback_note);
        check(native.knots == clamped.knots() && native.poles.size() == n &&
                  fallback_note["method"] == "cyclic_seam_fallback" &&
                  fallback_note["inserted_knot_count"] == 0 &&
                  fallback_note["pole_rotation"] == shift,
              "full-multiplicity special fallback strips the original exterior knot counts");
        for (unsigned i = 0; i < n; ++i) {
            const auto at = (i + shift) % n;
            check(native.poles[i] == loft_detail::H{double(at), double(at % 3), 0, 1 + .01 * at},
                  "special fallback rotates weighted XYZ and weights together across odd/even "
                  "orders");
        }
    }
    auto exterior =
        curve(3, {0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0}, {-.5, -.1, 0, .25, .5, .75, 1, 1.25, 1.5});
    exterior["closed"] = true;
    const auto external_open =
        loft_detail::open_periodic(BsplineCurve::from_bgfb(exterior), 6, &fallback_note);
    check(
        external_open.knots == std::vector<double>({0, 0, 0, .25, .5, .75, 1, 1, 1}) &&
            std::abs(external_open.poles.front()[0] - 4. / 7) < 1e-15 &&
            external_open.poles.back() == external_open.poles.front() &&
            fallback_note["inserted_knot_count"] == 2,
        "native cyclic insertion uses supplied exterior knots instead of forcing a periodic lift");
    for (double sign : {-1., 1.}) {
        auto near_seam =
            curve(3, {0, 0, 0, 1, 0, 0, 2, 1, 0, 2, 2, 0, 1, 3, 0, 0, 3, 0, -1, 2, 0, -1, 1, 0});
        near_seam["closed"] = true;
        const auto unshifted = BsplineCurve::from_bgfb(near_seam);
        near_seam["knots"] = Json::array();
        for (double k : unshifted.knots())
            near_seam["knots"].push_back(k - .5 + sign * 1e-12);
        const auto original = BsplineCurve::from_bgfb(near_seam);
        const auto prepared = loft_detail::open_periodic(original, 10, &fallback_note);
        const auto evaluated = BsplineCurve::from_bgfb(prepared.table());
        check(fallback_note["inserted_knot_count"] == 2 &&
                  std::abs(fallback_note["effective_seam_knot"].get<double>() - sign * 1e-12) <
                      1e-18 &&
                  prepared.poles.size() == 10,
              "a single nearby seam knot is retained and filled to native multiplicity, not "
              "duplicated");
        for (unsigned i = 0; i <= 80; ++i) {
            const double f = i / 80.;
            check(near(evaluated.point_at(f), original.point_at(std::fmod(.5 + f, 1.)), 5e-10),
                  "the reported native snapped seam preserves its cyclic parameterization");
        }
    }
    auto scaled = curve(3, {0, 0, 0, 1, 0, 0, 2, 0, 0, 1, 1, 0, 0, 1, 0},
                        {-2e8, -1.5e8, -1e8, 1e-4, 1.2e-4, 3e8, 6e8, 9e8, 1.1e9, 1.2e9});
    scaled["closed"] = true;
    loft_detail::open_periodic(BsplineCurve::from_bgfb(scaled), 8, &fallback_note);
    check(
        std::abs(fallback_note["knot_tolerance"].get<double>() - 2e-6) < 1e-18 &&
            fallback_note["effective_seam_knot"] == 0 && fallback_note["inserted_knot_count"] == 3,
        "large knot domains reduce tolerance using native active-span differences before snapping");
    auto compact = Curve::from_bspline(
        BsplineCurve::from_bgfb(curve(3, {1e8, 0, 0, 1e8 + 1e-4, 0, 0, 1e8 + 4e-8, 0, 0})), 3);
    const auto compact_open = loft_detail::close_reopen(compact, 3, closure);
    check(closure["method"] == "special_periodic_reopened" &&
              closure["opening"]["method"] == "cyclic_seam_fallback" &&
              compact_open.poles[0] == compact.poles[1] &&
              compact_open.poles[2] == compact.poles[0],
          "a relative closure match can take the special opening fallback when the local range is "
          "small");
    for (unsigned order = 2; order <= 9; ++order) {
        const unsigned n = order + 6, p = order - 1;
        std::vector<double> cycle(n + 1);
        Json xyz = Json::array(), weights = Json::array();
        for (unsigned i = 0; i < n; ++i) {
            cycle[i + 1] = cycle[i] + (order > 2 && i == 2 ? 0 : weight(random));
            const double w = weight(random);
            weights.push_back(w);
            for (unsigned axis = 0; axis < 3; ++axis)
                xyz.push_back(w * coord(random));
        }
        const double total = cycle.back();
        for (auto &k : cycle)
            k /= total;
        for (double shift : {0., -.237, -1.}) {
            Json knots = Json::array();
            for (unsigned i = 0; i < n + 2 * order - 1; ++i) {
                const int relative = int(i) - int(p);
                const int turn = relative < 0 ? -1 : relative / int(n);
                const unsigned j = unsigned(relative - turn * int(n));
                knots.push_back(cycle[j] + turn + shift);
            }
            auto stored = curve(order, xyz, knots, weights);
            stored["closed"] = true;
            const auto original = BsplineCurve::from_bgfb(stored);
            const auto prepared = loft_detail::open_periodic(original, 100, &fallback_note);
            const auto evaluated = BsplineCurve::from_bgfb(prepared.table());
            for (unsigned i = 0; i <= 70; ++i) {
                const double f = i / 70.;
                double source_parameter = f - shift;
                while (source_parameter > 1)
                    source_parameter -= 1;
                check(near(evaluated.point_at(f), original.point_at(source_parameter), 2e-9),
                      "native local cyclic insertion preserves nonuniform repeated-knot rational "
                      "curves");
            }
        }
    }
    auto fallback_input = source();
    const auto bottom_fallback = BsplineCurve::from_bgfb(fallback.table());
    auto lifted = imperfect;
    for (std::size_t i = 0; i < lifted["weights"].size(); ++i)
        lifted["poles"][3 * i + 2] = 2 * lifted["weights"][i].get<double>();
    fallback_input["section0"] = array({imperfect});
    fallback_input["section1"] = array({lifted});
    const auto f0 = bottom_fallback.point_at(0), f1 = bottom_fallback.point_at(1);
    fallback_input["guide_groups"][0] =
        Json::array({array({line(f0, {f0[0], f0[1], 2})}), array({line(f1, {f1[0], f1[1], 2})})});
    const auto fallback_loft =
        SectionLoft::from_bgfb(decode_bgfb(Wire{}.encode(fallback_input)).at("geometry"));
    check(fallback_loft.report()["curve_openings"][0]["method"] == "cyclic_seam_fallback" &&
              fallback_loft.report()["curve_openings"][0]["pole_rotation"] == 1 &&
              fallback_loft.source()["section0"]["curves"][0]["geometry"]["weights"] ==
                  imperfect["weights"],
          "BGFB loft reports the actual fallback opening method and keeps the original weights");
    for (unsigned i = 0; i <= 40; ++i) {
        const auto point = bottom_fallback.point_at(i / 40.);
        check(near(fallback_loft.sides()[0].surface.point_at(i / 40., 0), point) &&
                  near(fallback_loft.sides()[0].surface.point_at(i / 40., 1),
                       {point[0], point[1], 2}),
              "special fallback controls reach both rational side boundaries without substitution");
    }
    auto periodic_input = source();
    auto clustered = curve(3, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {-.5, -.25, 0, 1e-12, .5, .75, 1, 1.25, 1.5});
    clustered["closed"] = true;
    rejects([&] { loft_detail::open_periodic(BsplineCurve::from_bgfb(clustered), 100); },
            "distinct near seam knots are not silently collapsed to create a clamped loft curve");
    auto tiny_domain = exterior;
    for (auto &k : tiny_domain["knots"])
        k = k.get<double>() * 1e-11;
    rejects([&] { loft_detail::open_periodic(BsplineCurve::from_bgfb(tiny_domain), 100); },
            "a knot domain below the native normalization threshold is not silently rescaled");
    periodic_input["section0"] = array({circle_table});
    auto raised = circle_table;
    for (std::size_t i = 0; i < raised["weights"].size(); ++i)
        raised["poles"][3 * i + 2] = 2 * raised["weights"][i].get<double>();
    periodic_input["section1"] = array({raised});
    periodic_input["guide_groups"][0] =
        Json::array({array({line({1, 0, 0}, {1, 0, 2})}), array({line({1, 0, 0}, {1, 0, 2})})});
    const auto cylinder = SectionLoft::from_bgfb(periodic_input);
    check(cylinder.source() == periodic_input && cylinder.report()["curve_openings"].size() == 2 &&
              cylinder.report()["curve_openings"][0]["method"] == "strip_exterior_knots" &&
              cylinder.report()["curve_openings"][0]["opened_pole_count"] == 7,
          "source periodic openings report provenance without changing source identity or arrays");
    const auto decoded_cylinder =
        SectionLoft::from_bgfb(decode_bgfb(Wire{}.encode(periodic_input)).at("geometry"));
    check(decoded_cylinder.sides()[0].surface.poles() == cylinder.sides()[0].surface.poles(),
          "periodic type 21 source crosses the actual BGFB decoder and loft opening path");
    for (unsigned i = 0; i <= 60; ++i) {
        const auto p = cylinder.sides()[0].surface.point_at(i / 60., .4);
        check(std::abs(p[0] * p[0] + p[1] * p[1] - 1) < 2e-11 && std::abs(p[2] - .8) < 2e-11,
              "closed BGFB B-spline sections now reach the native rational cylindrical side");
    }
    for (unsigned order = 2; order <= 9; ++order) {
        const unsigned n = order + 2;
        Json xyz = Json::array(), knots = Json::array();
        for (unsigned i = 0; i < n; ++i)
            for (unsigned k = 0; k < 3; ++k)
                xyz.push_back(i == n - 1 ? 0. : double(i + k));
        xyz[0] = xyz[1] = xyz[2] = 0.;
        for (unsigned i = order / 2; i > 0; --i)
            knots.push_back(-double(i));
        for (unsigned i = 0; i < order; ++i)
            knots.push_back(0);
        knots.push_back(1. / 3);
        knots.push_back(2. / 3);
        for (unsigned i = 0; i < order; ++i)
            knots.push_back(1);
        for (unsigned i = 0; i < order - 1 - order / 2; ++i)
            knots.push_back(2. + i);
        auto table = curve(order, xyz, knots);
        table["closed"] = true;
        const auto original = BsplineCurve::from_bgfb(table),
                   opened = BsplineCurve::from_bgfb(Curve::from_bspline(original, 1000).table());
        check(
            opened.poles() == original.poles() && opened.poles().size() == n,
            "special odd and even orders preserve source poles with asymmetric exterior trimming");
        for (unsigned i = 0; i <= 30; ++i)
            check(
                near(opened.point_at(i / 30.), original.point_at(i / 30.)),
                "special opening agrees with the native cyclic index shift for every tested order");
    }
    auto periodic_guide = curve(3, {0, 0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2});
    periodic_guide["closed"] = true;
    auto right_guide = periodic_guide;
    for (unsigned i = 0; i < 4; ++i)
        right_guide["poles"][3 * i] = 3;
    periodic_input = source();
    periodic_input["section0"] = periodic_input["section1"] = array({line({0, 1, 0}, {3, 1, 0})});
    periodic_input["guide_groups"][0] =
        Json::array({array({periodic_guide}), array({right_guide})});
    const auto guide_loft = SectionLoft::from_bgfb(periodic_input);
    const auto guide_eval = BsplineCurve::from_bgfb(periodic_guide);
    check(guide_loft.sides()[0].surface.v().pole_count() == 6,
          "ordinary periodic guides are opened before global compatibility");
    for (unsigned i = 0; i <= 30; ++i) {
        const auto p = guide_eval.point_at(i / 30.);
        check(near(guide_loft.sides()[0].surface.point_at(.3, i / 30.), {.9, p[1], p[2]}),
              "periodic guide loft matches an independently evaluated ruled cyclic surface");
    }
    auto repeated = curve(3, {0, 0, 0, 2, 0, 0, 2, 1, 0, 2, 2, 0, 0, 2, 0, -1, 1, 0},
                          {-.5, -.25, 0, 0, .25, .5, .5, .75, 1, 1, 1.25});
    repeated["closed"] = true;
    const auto repeated_eval = BsplineCurve::from_bgfb(repeated);
    const auto repeated_open = Curve::from_bspline(repeated_eval, 1000);
    check(repeated_open.poles.size() == 7,
          "seam multiplicity two creates only one additional native pole");
    for (unsigned i = 0; i <= 40; ++i)
        check(
            near(BsplineCurve::from_bgfb(repeated_open.table()).point_at(i / 40.),
                 repeated_eval.point_at(i / 40.)),
            "periodic opening preserves repeated interior knots without changing their continuity");
    rejects([&] { Curve::from_bspline(repeated_eval, 6); },
            "periodic preparation respects its final control budget");
    auto repeated_end = repeated;
    for (auto &k : repeated_end["knots"])
        k = k.get<double>() - 1;
    rejects(
        [&] { loft_detail::open_periodic(BsplineCurve::from_bgfb(repeated_end), 100); },
        "unsupported repeated end-seam insertion is rejected before an out-of-range control copy");
    auto shifted_curve = periodic_guide;
    shifted_curve["knots"] = Json::array();
    for (double k : guide_eval.knots())
        shifted_curve["knots"].push_back(k - .173);
    const auto shifted_eval = BsplineCurve::from_bgfb(shifted_curve);
    const auto seam = shifted_eval.point_at(.173);
    auto shifted_top = shifted_curve;
    for (unsigned i = 0; i < 4; ++i)
        shifted_top["poles"][3 * i] = 3;
    periodic_input = source();
    periodic_input["section0"] = array({shifted_curve});
    periodic_input["section1"] = array({shifted_top});
    periodic_input["guide_groups"][0] = Json::array(
        {array({line(seam, {3, seam[1], seam[2]})}), array({line(seam, {3, seam[1], seam[2]})})});
    const auto shifted_loft = SectionLoft::from_bgfb(periodic_input);
    check(std::abs(
              shifted_loft.report()["curve_openings"][0]["source_fraction_at_seam"].get<double>() -
              .173) < 1e-15 &&
              shifted_loft.report()["curve_openings"][0]["source_path"] ==
                  "/section0/curves/0/geometry" &&
              near(shifted_loft.sides()[0].surface.point_at(0, 0), seam),
          "loft uses and reports native knot zero instead of silently using source fraction zero");
    const auto original_start = shifted_eval.point_at(0);
    auto detour = seam;
    detour[2] += 1;
    periodic_input["section0"] = periodic_input["section1"] =
        array({line(seam, {3, seam[1], seam[2]})});
    periodic_input["guide_groups"][0] = Json::array(
        {array({shifted_curve, line(seam, detour), line(detour, seam)}), array({shifted_top})});
    const auto shifted_guides = SectionLoft::from_bgfb(periodic_input);
    check(!near(original_start, seam) && !shifted_guides.report().contains("guide_closures"),
          "closure uses source fractions zero and one, not the opened periodic seam, and does not "
          "reclose an already closed single source");
    periodic_input["guide_groups"][0][0]["curves"][2]["geometry"] = line(detour, original_start);
    periodic_input["section1"] = array({line(original_start, {3, seam[1], seam[2]})});
    const auto source_closed = SectionLoft::from_bgfb(periodic_input);
    check(source_closed.report()["guide_closures"].size() == 1 &&
              source_closed.report()["guide_closures"][0]["method"] == "retained_open" &&
              source_closed.report()["guide_closures"][0]["reason"] == "endpoint_position_mismatch",
          "source closure can trigger a failed reclosure after periodic opening without discarding "
          "the valid open guide");
    periodic_input["section0"] = periodic_input["section1"] =
        array({shifted_curve, curve(2, {seam[0], seam[1], seam[2], detour[0], detour[1], detour[2],
                                        seam[0], seam[1], seam[2]})},
              2);
    bool original_section_endpoints = false;
    try {
        SectionLoft::from_bgfb(periodic_input);
    } catch (const std::exception &e) {
        original_section_endpoints =
            std::string(e.what()).find("closed section endpoints") != std::string::npos;
    }
    check(original_section_endpoints,
          "closed section arrays use the source endpoints before the individual curves are opened");
    for (unsigned p = 1; p <= 6; ++p)
        for (unsigned q = p + 1; q <= 9; ++q)
            for (unsigned m = 1; m <= p; ++m) {
                Json knots = Json::array(), xyz = Json::array(), weights = Json::array();
                for (unsigned i = 0; i <= p; ++i)
                    knots.push_back(0);
                for (double k : {.17, .44, .86})
                    for (unsigned i = 0; i < m; ++i)
                        knots.push_back(k);
                for (unsigned i = 0; i <= p; ++i)
                    knots.push_back(1);
                for (std::size_t i = 0; i < knots.size() - p - 1; ++i) {
                    const double w = weight(random);
                    weights.push_back(w);
                    for (unsigned k = 0; k < 3; ++k)
                        xyz.push_back(coord(random) * w);
                }
                const auto original = BsplineCurve::from_bgfb(curve(p + 1, xyz, knots, weights));
                auto c = Curve::from_bspline(original, 1000);
                c.elevate(q, 1000);
                const auto elevated = BsplineCurve::from_bgfb(c.table());
                check(c.poles.size() == 3 * (m + q - p) + q + 1,
                      "native degree elevation preserves continuity rather than Bezier expansion");
                for (unsigned i = 0; i <= 40; ++i)
                    check(near(original.point_at(i / 40.), elevated.point_at(i / 40.), 2e-8),
                          "elevation preserves rational nonuniform geometry");
                for (double t : {.17, .44, .86})
                    check(near(original.point_at(t), elevated.point_at(t), 2e-8),
                          "elevation uses the left polynomial at repeated knots");
            }
    auto constant = [&](double knot, unsigned m = 1) {
        Json u = {0, 0, 0}, xyz = Json::array();
        for (unsigned i = 0; i < m; ++i)
            u.push_back(knot);
        for (unsigned i = 0; i < 3; ++i)
            u.push_back(1);
        for (unsigned i = 0; i < m + 3; ++i)
            for (unsigned j = 0; j < 3; ++j)
                xyz.push_back(0);
        return Curve::from_bspline(BsplineCurve::from_bgfb(curve(3, xyz, u)), 1000);
    };
    auto a = constant(.5), b = constant(.5 + 5e-11);
    const auto poles = a.poles;
    loft_detail::compatible({&a, &b}, 1000);
    check(a.poles == poles && b.poles == poles && a.knots == b.knots &&
              a.knots[3] == (.5 + (.5 + 5e-11)) / 2,
          "compatible matching groups average close knots without moving poles");
    a = constant(.5);
    b = constant(.5 + 6e-11);
    auto c = constant(.5 + 12e-11);
    loft_detail::compatible({&a, &b, &c}, 1000);
    check(a.knots == b.knots && b.knots == c.knots && a.poles.size() == 4 &&
              std::abs(a.knots[3] - (.5 + 6e-11)) < 1e-15,
          "general knot merge clusters by adjacent differences, not distance to first");
    a = constant(.25);
    b = constant(.7, 2);
    loft_detail::compatible({&a, &b}, 1000);
    check(a.knots == std::vector<double>({0, 0, 0, .25, .7, .7, 1, 1, 1}) && a.knots == b.knots,
          "general compatibility retains the maximum multiplicity per source knot");
    a = constant(.5);
    a.elevate(25, 1000);
    check(a.degree == 25 && a.poles.size() == 50 &&
              BsplineCurve::from_bgfb(a.table()).point_at(.31) == Point3{},
          "native maximum degree 25 is supported");
    rejects([&] { a.elevate(26, 1000); }, "native degree ceiling is enforced");
    auto segment = [&](double lo, double hi) {
        return Curve::from_bspline(BsplineCurve::from_bgfb(line({lo, 0, 0}, {hi, 0, 0})), 1000);
    };
    a = loft_detail::append(loft_detail::append(segment(0, 1), segment(1, 3), false, 1000),
                            segment(3, 4), false, 1000);
    check(a.knots == std::vector<double>({0, 0, .25, .5, 1, 1}),
          "three composite pieces retain sequential half-domains");
    a = loft_detail::append(segment(0, 1), segment(1, 3), true, 1000);
    check(a.knots[2] == 1. / 3,
          "different source member counts select control polygon length weighting");
    a = loft_detail::append(segment(0, 1e-13), segment(1e-13, 1), true, 1000);
    check(a.poles.size() == 2 && a.poles.front()[0] == 1e-13,
          "native negligible preceding guide is omitted only by its confirmed length rule");
    auto input = source();
    const auto saved = input;
    const auto loft = SectionLoft::from_bgfb(input);
    const auto &s = loft.sides()[0].surface;
    check(loft.source() == saved && input == saved && loft.sides().size() == 1 &&
              loft.sides()[0].loop_index == 0,
          "source loft reconstruction keeps source data and patch provenance");
    check(s.u().order() == 4 && s.u().pole_count() == 7 && s.v().order() == 2 &&
              s.v().pole_count() == 2,
          "mixed source degrees retain original C1/C2 knots in the Coons control net");
    check(s.u().knots() == std::vector<double>({0, 0, 0, 0, .3, .3, .6, 1, 1, 1, 1}),
          "loft knot union does not introduce full Bezier multiplicities");
    const auto bottom = BsplineCurve::from_bgfb(input["section0"]["curves"][0]["geometry"]),
               top = BsplineCurve::from_bgfb(input["section1"]["curves"][0]["geometry"]);
    for (unsigned i = 0; i <= 80; ++i) {
        const double u = i / 80.;
        const auto x = bottom.point_at(u), y = top.point_at(u);
        check(near(s.point_at(u, 0), x) && near(s.point_at(u, 1), y),
              "derived source loft has the original boundary curves");
        for (double v : {.23, .51, .9})
            check(near(s.point_at(u, v),
                       {x[0] * (1 - v) + y[0] * v, x[1] * (1 - v) + y[1] * v, 2 * v}),
                  "straight guides give an independently evaluated ruled surface");
    }
    const auto decoded = decode_bgfb(Wire{}.encode(input)).at("geometry");
    const auto from_wire = SectionLoft::from_bgfb(decoded);
    check(from_wire.sides()[0].surface.poles() == s.poles() &&
              from_wire.sides()[0].surface.u().knots() == s.u().knots(),
          "actual BGFB type 21 decoding reaches parametric loft reconstruction");
    input = source();
    input["section0"] = array({line({0, 0, 0}, {3, 0, 0})});
    input["section1"] = array({line({0, 0, 2}, {3, 0, 2})});
    const auto left_table =
        curve(3, {0, 0, 0, -1, 1, .5, 1, -1, 1.5, 0, 0, 2}, {0, 0, 0, .37, 1, 1, 1});
    const auto right_table =
        curve(4, {3, 0, 0, 2, -1, .3, 4, 2, 1, 3, 1, 1.8, 3, 0, 2}, {0, 0, 0, 0, .72, 1, 1, 1, 1});
    input["guide_groups"][0] = Json::array({array({left_table}), array({right_table})});
    const auto bent = SectionLoft::from_bgfb(input);
    const auto l = BsplineCurve::from_bgfb(left_table), r = BsplineCurve::from_bgfb(right_table);
    check(bent.sides()[0].surface.v().knots() ==
              std::vector<double>({0, 0, 0, 0, .37, .37, .72, 1, 1, 1, 1}),
          "all guides are made compatible before side construction");
    for (unsigned i = 0; i <= 40; ++i) {
        const auto x = l.point_at(i / 40.), y = r.point_at(i / 40.);
        for (double u : {0., .3, .5, .7, 1.})
            check(near(bent.sides()[0].surface.point_at(u, i / 40.),
                       {(1 - u) * x[0] + u * y[0], (1 - u) * x[1] + u * y[1],
                        (1 - u) * x[2] + u * y[2]}),
                  "nonuniform B-spline guides preserve independently evaluated boundaries and "
                  "ruled interior");
    }
    std::vector<Json> low_loops, high_loops;
    Json guide_groups = Json::array();
    for (unsigned ring = 0; ring < 2; ++ring) {
        const double lo = ring ? .5 : 0, hi = ring ? 1.5 : 2;
        const std::vector<Point3> corners =
            ring ? std::vector<Point3>{{lo, lo, 0}, {lo, hi, 0}, {hi, hi, 0}, {hi, lo, 0}}
                 : std::vector<Point3>{{lo, lo, 0}, {hi, lo, 0}, {hi, hi, 0}, {lo, hi, 0}};
        std::vector<Json> low, high;
        Json rails = Json::array();
        for (unsigned i = 0; i < 4; ++i) {
            auto x = corners[i], y = corners[(i + 1) % 4], z = x, t = y;
            z[2] = 3;
            t[2] = 3;
            low.push_back(line(x, y));
            high.push_back(line(z, t));
            rails.push_back(array({line(x, z)}));
        }
        low_loops.push_back(array(low, ring ? 3 : 2));
        high_loops.push_back(array(high, ring ? 3 : 2));
        guide_groups.push_back(rails);
    }
    input = {{"_type", "P3DSectionLoft"},
             {"capped", true},
             {"section0", array(low_loops, 4)},
             {"section1", array(high_loops, 4)},
             {"guide_groups", guide_groups}};
    const auto parity = SectionLoft::from_bgfb(input);
    check(parity.sides().size() == 8 && parity.sides()[4].loop_index == 1 &&
              parity.sides()[4].primitive_index == 0 &&
              near(parity.sides()[4].surface.point_at(.5, .5), {.5, 1, 1.5}),
          "parity loft retains native outer/inner and source side order");
    for (unsigned ring = 0; ring < 2; ++ring)
        for (unsigned i = 0; i < 4; ++i)
            check(near(parity.sides()[4 * ring + i].surface.point_at(1, .4),
                       parity.sides()[4 * ring + (i + 1) % 4].surface.point_at(0, .4)),
                  "closed source loops connect the final side to their own first guide");
    input["section0"]["curves"][0]["geometry"]["type"] = 3;
    rejects([&] { SectionLoft::from_bgfb(input); },
            "parity loops cannot be relabelled or sorted into validity");
    // A quarter arc below a three-span full arc exposes the order of the
    // native three-pole elevation and knot compatibility operations.
    auto arc = [](double sweep, double z) {
        return Json{{"_type", "EllipticArc"},
                    {"arc",
                     {{"centerX", 0},
                      {"centerY", 0},
                      {"centerZ", z},
                      {"vector0X", 1},
                      {"vector0Y", 0},
                      {"vector0Z", 0},
                      {"vector90X", 0},
                      {"vector90Y", 1},
                      {"vector90Z", 0},
                      {"startRadians", 0},
                      {"sweepRadians", sweep}}}};
    };
    constexpr double pi = 3.1415926535897932384626433832795;
    input = source();
    input["section0"] = array({arc(pi / 2, 0)});
    input["section1"] = array({arc(2 * pi, 2)});
    input["guide_groups"][0] =
        Json::array({array({line({1, 0, 0}, {1, 0, 2})}), array({line({0, 1, 0}, {1, 0, 2})})});
    const auto arcs = SectionLoft::from_bgfb(input);
    input["guide_groups"][0][0] = array({{{"_type", "LineSegment"},
                                          {"segment",
                                           {{"point0X", 1},
                                            {"point0Y", 0},
                                            {"point0Z", 0},
                                            {"point1X", 1},
                                            {"point1Y", 0},
                                            {"point1Z", 2}}}}});
    const auto arcs_from_wire =
        SectionLoft::from_bgfb(decode_bgfb(Wire{}.encode(input)).at("geometry"));
    check(arcs_from_wire.sides()[0].surface.poles() == arcs.sides()[0].surface.poles(),
          "BGFB arc and line structs use their actual flattened coordinate fields");
    for (unsigned i = 0; i <= 60; ++i) {
        const auto lower = arcs.sides()[0].surface.point_at(i / 60., 0);
        const auto upper = arcs.sides()[0].surface.point_at(i / 60., 1);
        check(
            std::abs(lower[0] * lower[0] + lower[1] * lower[1] - 1) < 2e-11 &&
                std::abs(upper[0] * upper[0] + upper[1] * upper[1] - 1) < 2e-11 &&
                std::abs(lower[2]) < 2e-11 && std::abs(upper[2] - 2) < 2e-11,
            "rational loft boundaries retain both exact circle loci after native degree elevation");
    }
    check(arcs.sides()[0].surface.u().order() == 4 &&
              arcs.sides()[0].surface.u().pole_count() == 10,
          "three-pole conic elevates both source sections before their spans are aligned");
    GuidedBoundary ga, gb, gl, gr;
    ga.ellipse = gb.ellipse = true;
    ga.axis_x = gb.axis_x = {1, 0, 0};
    ga.axis_y = gb.axis_y = {0, 1, 0};
    ga.sweep = pi / 2;
    gb.sweep = 2 * pi;
    gb.center = {0, 0, 2};
    gl.points = {{1, 0, 0}, {1, 0, 2}};
    gr.points = {{0, 1, 0}, {1, 0, 2}};
    const auto command_mesh = guided_surface({ga}, {gb}, {gl, gr}, Tessellation{}, false);
    check(command_mesh.note["patches"][0]["u_degree"] == 3 &&
              command_mesh.note["patches"][0]["u_poles"] == 10,
          "command stream follows the same corrected native elevation order as BGFB lofts");
    for (const auto *type : {"AkimaCurve", "InterpolationCurve", "TransitionSpiral"}) {
        input = source();
        input["section0"]["curves"][0]["geometry"] = {{"_type", type}};
        bool native_failure = false;
        try {
            SectionLoft::from_bgfb(input);
        } catch (const std::exception &e) {
            native_failure = std::string(e.what()).find(
                                 "native loft conversion returns no curve") != std::string::npos;
        }
        check(native_failure,
              "loft does not substitute a proxy cache when the native accessor returns null");
    }
    input = source();
    input["guide_groups"][0][0] = array({line({0, 0, 0}, {0, 0, .5}), line({0, 0, .5}, {0, 0, 2})});
    const auto composite = SectionLoft::from_bgfb(input);
    check(composite.report()["loops"][0]["guide_parameterization"] == "control_polygon_length" &&
              composite.sides()[0].surface.v().knots() ==
                  std::vector<double>({0, 0, 0, .25, .25, 1, 1, 1}),
          "actual source member counts select length-weighted joining before global guide "
          "compatibility");
    for (unsigned i = 0; i <= 20; ++i)
        check(near(composite.sides()[0].surface.point_at(0, i / 20.), {0, 0, 2 * i / 20.}),
              "composite guide has source length parameterization after three-pole elevation");
    input["guide_groups"][0][0] = array({line({0, 0, 0}, {0, 0, 1}), line({0, 0, 1}, {0, 0, 0})});
    bool corner_mismatch = false;
    try {
        SectionLoft::from_bgfb(input);
    } catch (const std::exception &e) {
        corner_mismatch = std::string(e.what()).find("corner mismatch") != std::string::npos;
    }
    check(corner_mismatch, "closed guide still must match the original section corners");
    input["section0"] = input["section1"] = array({line({0, 0, 0}, {3, 0, 0})});
    input["guide_groups"][0][1] = array({curve(2, {3, 0, 0, 4, 0, 1, 3, 0, 0}, {0, 0, .5, 1, 1})});
    const auto closed_composite = SectionLoft::from_bgfb(input);
    check(closed_composite.source() == input &&
              closed_composite.report()["guide_closures"].size() == 2 &&
              closed_composite.report()["guide_closures"][0]["method"] ==
                  "linear_periodic_reopened" &&
              closed_composite.report()["guide_closures"][1]["source_path"] == "/guide_groups/0/1",
          "unequal source guide counts activate native closure and preserve source provenance");
    const auto decoded_closed =
        SectionLoft::from_bgfb(decode_bgfb(Wire{}.encode(input)).at("geometry"));
    check(decoded_closed.sides()[0].surface.poles() == closed_composite.sides()[0].surface.poles(),
          "closed composite guides roundtrip through the actual BGFB union decoder");
    for (unsigned i = 0; i <= 40; ++i) {
        const double t = i / 40., z = t <= .5 ? 2 * t : 2 * (1 - t);
        check(near(closed_composite.sides()[0].surface.point_at(0, t), {0, 0, z}) &&
                  near(closed_composite.sides()[0].surface.point_at(1, t), {3 + z, 0, z}),
              "reclosed composite guides remain exact side boundaries after global compatibility");
    }
    input = source();
    input["section0"] = array({{{"_type", "LineString"}, {"points", {0, 0, 0, 1, 1, 0, 3, 0, 0}}}});
    const auto unequal = SectionLoft::from_bgfb(input);
    check(
        unequal.sides()[0].surface.u().order() == 5,
        "three-pole rule elevates each section once before compatibility even when degrees differ");
    input = source();
    input["capped"] = true;
    check(SectionLoft::from_bgfb(input).report()["cap_status"] == "not_reconstructed",
          "requested caps are explicitly distinguished from completed side reconstruction");
    rejects([&] { SectionLoft::from_bgfb(input, 5); },
            "total surface control budget cannot return a partial loft");
    input["guide_groups"][0][0] = array({line({9, 0, 0}, {0, 0, 2})});
    rejects([&] { SectionLoft::from_bgfb(input); },
            "misordered source guides are not searched or silently reversed");
    input = source();
    input["section0"]["curves"][0]["geometry"]["closed"] = true;
    rejects([&] { SectionLoft::from_bgfb(input); },
            "closed source still requires the correct periodic knot array length");
    input = source();
    input["section0"] =
        array({curve(2, {0, 0, 0, 1, 0, 0, 2, 0, 0, 3, 0, 0}, {0, 0, .5, .5, 1, 1})});
    rejects([&] { SectionLoft::from_bgfb(input); },
            "discontinuous source spline cannot produce a false continuous side");
    input = source();
    input["section0"]["curves"][0]["geometry"]["weights"] = {1, 0, 1, 1};
    rejects([&] { SectionLoft::from_bgfb(input); },
            "unsupported zero control weights are not deweighted as finite points");
    input = source();
    auto task = std::async(std::launch::async, [&] { return SectionLoft::from_bgfb(input); });
    const auto concurrent = SectionLoft::from_bgfb(input);
    check(task.get().sides()[0].surface.poles() == concurrent.sides()[0].surface.poles() &&
              input == saved,
          "source loft reconstruction is const and independent across concurrent calls");
    return checks;
}
