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
        b[t + 8] = 0;
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
    bool closure_pending = false;
    try {
        SectionLoft::from_bgfb(input);
    } catch (const std::exception &e) {
        closure_pending = std::string(e.what()).find("close/reopen") != std::string::npos;
    }
    check(closure_pending,
          "closed length-weighted guides cannot bypass the native reclosure branch");
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
            "unconfirmed periodic opening is explicitly rejected");
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
