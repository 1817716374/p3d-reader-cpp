#include "blob_internal.hpp"
#include <cmath>
#include <limits>
using namespace p3d;
namespace {
template <class T> void append(Bytes &b, T v) {
    const auto p = b.size();
    b.resize(p + sizeof(v));
    std::memcpy(b.data() + p, &v, sizeof(v));
}
struct FlatSurface {
    Bytes bytes = Bytes(12);
    template <class T> void write(std::size_t p, T v) {
        std::memcpy(bytes.data() + p, &v, sizeof(v));
    }
    void ref(std::size_t p, std::size_t target) {
        write(p, std::uint32_t(target - p));
    }
    void align(unsigned n, unsigned offset = 0) {
        while ((bytes.size() + offset) % n)
            bytes.push_back(0);
    }
    std::size_t table(std::vector<std::uint16_t> fields, std::uint16_t size) {
        align(4);
        const auto vt = bytes.size();
        append<std::uint16_t>(bytes, std::uint16_t(4 + 2 * fields.size()));
        append(bytes, size);
        for (auto off : fields)
            append(bytes, off);
        align(4);
        const auto p = bytes.size();
        bytes.resize(p + size);
        write(p, std::int32_t(p - vt));
        return p;
    }
    std::size_t boundary(const Json &source) {
        const auto &curves = source.at("curves");
        const auto p = table({4, std::uint16_t(curves.is_null() ? 0 : 8)}, 12);
        write(p + 4, source.at("type").get<std::int32_t>());
        if (curves.is_null())
            return p;
        align(4);
        const auto vector = bytes.size();
        append<std::uint32_t>(bytes, unsigned(curves.size()));
        bytes.resize(bytes.size() + 4 * curves.size());
        ref(p + 8, vector);
        for (std::size_t i = 0; i < curves.size(); ++i) {
            const auto &geometry = curves[i].at("geometry");
            const auto root = table({4, 8}, 12);
            ref(vector + 4 + 4 * i, root);
            if (geometry.at("_type") == "CurveVector") {
                bytes[root + 4] = 5;
                ref(root + 8, boundary(geometry));
            } else if (geometry.at("_type") == "InterpolationCurve") {
                bytes[root + 4] = 16;
                const auto curve = table({4, 8, 12, 16, 20, 24, 32, 56, 80, 84}, 88);
                ref(root + 8, curve);
                write(curve + 4, geometry.at("order").get<std::int32_t>());
                bytes[curve + 8] = geometry.at("closed").get<bool>();
                const std::array<const char *, 4> flags{"isChordLenKnots", "isColinearTangents",
                                                        "isChordLenTangents", "isNaturalTangents"};
                for (unsigned i = 0; i < 4; ++i)
                    write(curve + 12 + 4 * i, geometry.at(flags[i]).get<std::int32_t>());
                for (unsigned i = 0; i < 2; ++i)
                    for (unsigned k = 0; k < 3; ++k)
                        write(curve + 32 + 24 * i + 8 * k,
                              geometry.at(i ? "endTangent" : "startTangent")
                                  .at(std::array<const char *, 3>{"x", "y", "z"}[k])
                                  .get<double>());
                for (unsigned i = 0; i < 2; ++i) {
                    const auto &a = geometry.at(i ? "knots" : "fitPoints");
                    align(8, 4);
                    const auto v = bytes.size();
                    append<std::uint32_t>(bytes, unsigned(a.size()));
                    for (const auto &x : a)
                        append<double>(bytes, x.get<double>());
                    ref(curve + 80 + 4 * i, v);
                }
            } else {
                require(geometry.at("_type") == "LineString" ||
                            geometry.at("_type") == "PointString" ||
                            geometry.at("_type") == "AkimaCurve",
                        "fixture trim curve type");
                bytes[root + 4] = geometry.at("_type") == "LineString"    ? 4
                                  : geometry.at("_type") == "PointString" ? 18
                                                                          : 19;
                const auto line = table({4}, 8);
                ref(root + 8, line);
                align(8, 4);
                const auto points = bytes.size();
                append<std::uint32_t>(bytes, unsigned(geometry.at("points").size()));
                for (const auto &v : geometry.at("points"))
                    append<double>(bytes, v.get<double>());
                ref(line + 4, points);
            }
        }
        return p;
    }
    explicit FlatSurface(const Json &source) {
        std::memcpy(bytes.data(), "bg0001fb", 8);
        const auto root = table({4, 8}, 12);
        ref(8, root);
        bytes[root + 4] = 14;
        std::vector<std::uint16_t> fields{4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 53};
        for (unsigned i = 0; i < 4; ++i)
            if (source.at(std::array<const char *, 4>{"poles", "weights", "knotsU", "knotsV"}[i])
                    .is_null())
                fields[i] = 0;
        if (source["boundaries"].is_null())
            fields[11] = 0;
        if (!source["closedU"].get<bool>())
            fields[12] = 0;
        if (!source["closedV"].get<bool>())
            fields[13] = 0;
        const auto surface = table(fields, 56);
        ref(root + 8, surface);
        const std::array<const char *, 7> scalar{"numPolesU", "numPolesV", "orderU",    "orderV",
                                                 "numRulesU", "numRulesV", "holeOrigin"};
        for (unsigned i = 0; i < scalar.size(); ++i)
            write(surface + 20 + 4 * i, source[scalar[i]].get<std::int32_t>());
        bytes[surface + 52] = source["closedU"].get<bool>();
        bytes[surface + 53] = source["closedV"].get<bool>();
        for (unsigned i = 0; i < 4; ++i) {
            const auto &a =
                source[std::array<const char *, 4>{"poles", "weights", "knotsU", "knotsV"}[i]];
            if (a.is_null())
                continue;
            align(8, 4);
            const auto p = bytes.size();
            append<std::uint32_t>(bytes, unsigned(a.size()));
            for (const auto &v : a)
                append<double>(bytes, v.get<double>());
            ref(surface + 4 + 4 * i, p);
        }
        if (!source["boundaries"].is_null()) {
            ref(surface + 48, boundary(source["boundaries"]));
        }
    }
};
Json source(unsigned nu, unsigned nv, unsigned ku, unsigned kv) {
    return {{"_type", "BsplineSurface"},
            {"numPolesU", nu},
            {"numPolesV", nv},
            {"orderU", ku},
            {"orderV", kv},
            {"closedU", false},
            {"closedV", false},
            {"numRulesU", 4},
            {"numRulesV", 7},
            {"holeOrigin", 0},
            {"boundaries", nullptr},
            {"poles", Json::array()},
            {"weights", nullptr},
            {"knotsU", nullptr},
            {"knotsV", nullptr}};
}
bool near(Point3 a, Point3 b, double tol = 1e-11) {
    for (unsigned i = 0; i < 3; ++i)
        if (std::abs(a[i] - b[i]) > tol)
            return false;
    return true;
}
} // namespace
unsigned bspline_surface_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *message) {
        ++checks;
        require(value, message);
    };
    auto rejects = [&](auto function, const char *message) {
        bool threw = false;
        try {
            function();
        } catch (const std::exception &) {
            threw = true;
        }
        check(threw, message);
    };
    auto grid = source(2, 3, 2, 2);
    for (unsigned v = 0; v < 3; ++v)
        for (unsigned u = 0; u < 2; ++u)
            for (double x : Point3{3.0 * u, 5.0 * v, 2.0 * u * v})
                grid["poles"].push_back(x);
    auto plane = BsplineSurface::from_bgfb(grid);
    // Independent tensor Bernstein reference with non-separable weights.
    auto patch = source(3, 4, 3, 4);
    patch["weights"] = Json::array();
    std::vector<Point3> patch_points;
    for (unsigned v = 0; v < 4; ++v)
        for (unsigned u = 0; u < 3; ++u) {
            const double weight = 0.7 + 0.2 * u + 0.3 * v + 0.1 * u * v * v;
            const Point3 p{double(u * u + v), double(v * v - 2.0 * u), std::sin(double(u + v))};
            patch_points.push_back(p);
            patch["weights"].push_back(weight);
            for (double x : p)
                patch["poles"].push_back(weight * x);
        }
    const auto tensor = BsplineSurface::from_bgfb(patch);
    for (unsigned j = 0; j <= 8; ++j)
        for (unsigned i = 0; i <= 8; ++i) {
            const double u = i / 8.0, v = j / 8.0;
            const std::array<double, 3> bu{(1 - u) * (1 - u), 2 * u * (1 - u), u * u};
            const std::array<double, 4> bv{std::pow(1 - v, 3), 3 * v * (1 - v) * (1 - v),
                                           3 * v * v * (1 - v), v * v * v};
            Point3 expected{};
            double weight = 0;
            for (unsigned b = 0; b < 4; ++b)
                for (unsigned a = 0; a < 3; ++a) {
                    const auto index = b * 3 + a;
                    const double term = bu[a] * bv[b] * patch["weights"][index].get<double>();
                    weight += term;
                    for (unsigned k = 0; k < 3; ++k)
                        expected[k] += term * patch_points[index][k];
                }
            for (auto &x : expected)
                x /= weight;
            check(near(tensor.point_at(u, v), expected),
                  "surface agrees with independent rational tensor Bernstein polynomial");
        }
    auto ordinary = source(4, 3, 3, 2);
    ordinary["closedU"] = ordinary["closedV"] = true;
    for (unsigned v = 0; v < 3; ++v)
        for (unsigned u = 0; u < 4; ++u)
            for (double x : Point3{double(u), double(v), double(u * v)})
                ordinary["poles"].push_back(x);
    auto periodic = BsplineSurface::from_bgfb(ordinary);
    check(periodic.u().periodic_pole_shift() == 0 && periodic.v().periodic_pole_shift() == 0 &&
              near(periodic.point_at(0, 0.37), periodic.point_at(1, 0.37)) &&
              near(periodic.point_at(0.37, 0), periodic.point_at(0.37, 1)),
          "ordinary periodic surface reuses each source direction without the special seam shift");
    check(plane.poles().size() == 6 && plane.u().pole_count() == 2 && plane.v().pole_count() == 3 &&
              near(plane.point_at(0.2, 0.35), {0.6, 3.5, 0.28}),
          "surface control grid is U-fastest, including a non-square grid");
    check(plane.num_rules_u() == 4 && plane.num_rules_v() == 7 && plane.outer_boundary_active(),
          "source direction rules and zero hole origin are retained");
    check(plane.u().source_knots().empty() &&
              plane.v().knots() == std::vector<double>{0, 0, 0.5, 1, 1},
          "surface directions generate omitted knots independently");
    auto nonuniform = grid;
    nonuniform["knotsV"] = {0, 0, 0.2, 1, 1};
    check(near(BsplineSurface::from_bgfb(nonuniform).point_at(0.2, 0.1), {0.6, 2.5, 0.2}),
          "nonuniform V parameter does not use a uniformly spaced control grid");
    auto shifted = grid;
    shifted["knotsU"] = {2, 2, 6, 6};
    shifted["knotsV"] = {-4, -4, -1, 2, 2};
    auto shifted_surface = BsplineSurface::from_bgfb(shifted);
    check(shifted_surface.u().knot_domain() == std::array<double, 2>{2, 6} &&
              shifted_surface.v().knot_domain() == std::array<double, 2>{-4, 2} &&
              near(shifted_surface.point_at(0.2, 0.35), plane.point_at(0.2, 0.35)),
          "public surface fractions cover each source active knot domain without changing stored "
          "knots");
    auto tiny = grid;
    tiny["knotsU"] = {0, 0, 1e-310, 1e-310};
    check(near(BsplineSurface::from_bgfb(tiny).point_at(0.5, 0.5), plane.point_at(0.5, 0.5)),
          "narrow finite knot span does not overflow an intermediate inverse interval");
    const double w = std::sqrt(0.5);
    auto cylinder = source(3, 2, 3, 2);
    cylinder["weights"] = Json::array();
    for (unsigned v = 0; v < 2; ++v)
        for (unsigned u = 0; u < 3; ++u) {
            const double wu = std::array<double, 3>{1, w, 1}[u], wv = v + 1;
            Point3 p{std::array<double, 3>{1, w, 0}[u] * wv, std::array<double, 3>{0, w, 1}[u] * wv,
                     6.0 * v * wu * wv};
            for (double x : p)
                cylinder["poles"].push_back(x);
            cylinder["weights"].push_back(wu * wv);
        }
    auto c = BsplineSurface::from_bgfb(cylinder);
    check(c.rational() && near(c.point_at(0.5, 0.5), {w, w, 4}),
          "surface blends weighted XYZ and weights in both directions before dividing");
    for (unsigned v = 0; v <= 8; ++v)
        for (unsigned u = 0; u <= 8; ++u) {
            const double t = v / 8.0;
            const auto p = c.point_at(u / 8.0, t);
            check(std::abs(p[0] * p[0] + p[1] * p[1] - 1) < 1e-11 &&
                      std::abs(p[2] - 12 * t / (1 + t)) < 1e-11,
                  "rational tensor cylinder stays on analytic cylinder with rational height "
                  "parameter");
        }
    const double s = std::sqrt(3.0) / 2;
    const std::array<Point3, 7> circle{{{1, 0, 1},
                                        {0.5, s, 0.5},
                                        {-0.5, s, 1},
                                        {-1, 0, 0.5},
                                        {-0.5, -s, 1},
                                        {0.5, -s, 0.5},
                                        {1, 0, 1}}};
    auto torus = source(7, 7, 3, 3);
    torus["closedU"] = torus["closedV"] = true;
    torus["knotsU"] =
        torus["knotsV"] = {-1.0 / 3, 0, 0, 0, 1.0 / 3, 1.0 / 3, 2.0 / 3, 2.0 / 3, 1, 1, 1, 4.0 / 3};
    torus["weights"] = Json::array();
    for (const auto &v : circle)
        for (const auto &u : circle) {
            for (double x : Point3{(3 * v[2] + v[0]) * u[0], (3 * v[2] + v[0]) * u[1], v[1] * u[2]})
                torus["poles"].push_back(x);
            torus["weights"].push_back(v[2] * u[2]);
        }
    auto t = BsplineSurface::from_bgfb(torus);
    check(
        t.u().periodic_pole_shift() == -1 && t.v().periodic_pole_shift() == -1 &&
            t.poles().size() == 49,
        "both periodic directions apply native seam shifts without deduplicating the source grid");
    for (unsigned v = 0; v <= 12; ++v)
        for (unsigned u = 0; u <= 12; ++u) {
            const auto p = t.point_at(u / 12.0, v / 12.0);
            const double radial = std::hypot(p[0], p[1]) - 3;
            check(std::abs(radial * radial + p[2] * p[2] - 1) < 1e-11,
                  "doubly-periodic rational tensor surface stays on the analytic torus");
        }
    check(near(t.point_at(0, 0.3), t.point_at(1, 0.3)) &&
              near(t.point_at(0.4, 0), t.point_at(0.4, 1)),
          "both native torus seams close at original source positions");
    auto trim = grid;
    trim["holeOrigin"] = -2;
    trim["boundaries"] = {{"_type", "CurveVector"}, {"type", 3}, {"curves", nullptr}};
    auto trimmed = BsplineSurface::from_bgfb(trim);
    check(!trimmed.outer_boundary_active() && trimmed.hole_origin() == -2 &&
              trimmed.boundaries() == trim["boundaries"] &&
              near(trimmed.point_at(0.3, 0.4), plane.point_at(0.3, 0.4)),
          "nonzero hole origin disables outer boundary without deleting trim data or changing "
          "underlying evaluation");
    auto singular = source(2, 2, 2, 2);
    singular["poles"] = {1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0};
    singular["weights"] = {1, -1, 1, -1};
    auto inf = BsplineSurface::from_bgfb(singular);
    check(inf.homogeneous_at(0.5, 0.5)[3] == 0,
          "surface retains zero evaluated homogeneous weight");
    rejects([&] { inf.point_at(0.5, 0.5); }, "singular surface cannot return a fake finite point");
    for (const auto &kv :
         std::vector<std::pair<std::string, Json>>{{"numPolesU", INT32_MAX},
                                                   {"numPolesV", -3},
                                                   {"orderU", 1},
                                                   {"orderV", 4},
                                                   {"weights", Json({1})},
                                                   {"knotsU", Json({0, 1, 0, 1})},
                                                   {"knotsV", Json({0, 0, 1, 1})},
                                                   {"closedU", 1},
                                                   {"poles", Json({0, 0, 0, 1})},
                                                   {"boundaries", Json::array()}}) {
        auto bad = grid;
        bad[kv.first] = kv.second;
        rejects([&] { BsplineSurface::from_bgfb(bad); },
                "bad surface arrays and dimensions are rejected");
    }
    rejects([&] { t.point_at(1.1, 0.5); }, "periodic U fraction is not implicitly wrapped");
    rejects([&] { t.point_at(0.5, std::numeric_limits<double>::quiet_NaN()); },
            "NaN V fraction rejected");
    auto decoded = decode_bgfb(FlatSurface(torus).bytes)["geometry"];
    check(decoded["_spline"]["pole_order"] == "u_fastest" &&
              decoded["_spline"]["u"]["periodic_pole_shift"] == -1 &&
              decoded["_spline"]["v"]["periodic_pole_shift"] == -1 &&
              decoded["poles"] == torus["poles"] &&
              near(BsplineSurface::from_bgfb(decoded).point_at(0.5, 0.5), {-2, 0, 0}),
          "BGFB surface parser reaches public periodic evaluator with unchanged source poles");
    // U fraction .5 is on the negative X side of the torus.
    check(near(t.point_at(0.5, 0.5), {-2, 0, 0}),
          "torus half parameters keep native seam orientation");
    auto decoded_trim = decode_bgfb(FlatSurface(trim).bytes)["geometry"];
    check(decoded_trim["_spline"]["outer_boundary_active"] == false &&
              decoded_trim["_spline"]["trim_region_evaluation"] == "not_evaluated" &&
              decoded_trim["boundaries"]["type"] == 3 && decoded_trim["closedU"] == false,
          "surface trim and omitted closed flags have explicit semantic metadata");
    auto invalid = grid;
    invalid["weights"] = {1};
    auto bad_decoded = decode_bgfb(FlatSurface(invalid).bytes)["geometry"];
    check(bad_decoded["_spline"]["status"] == "invalid" && bad_decoded["weights"] == Json({1}),
          "invalid surface semantics retain the original decoded arrays");
    Bytes packet(32);
    torus["holeOrigin"] = 1;
    const Json trim_line = {{"_type", "AkimaCurve"},
                            {"points", {.8, .8, 0,  .2, .8, 0,  .2, .2, 0,  .8, .2, 0,  .8, .8,
                                        0,  .2, .8, 0,  .2, .2, 0,  .8, .2, 0,  .8, .8, 0}}};
    Json trim_loop = {{"_type", "CurveVector"},
                      {"type", 2},
                      {"curves", Json::array({Json{{"geometry", trim_line}}})}};
    trim_loop["curves"].push_back({{"geometry", trim_loop}});
    trim_loop["curves"].push_back(
        {{"geometry", {{"_type", "PointString"}, {"points", {.4, .4, 0, .6, .6, 0}}}}});
    torus["boundaries"] = {{"_type", "CurveVector"},
                           {"type", 4},
                           {"curves", Json::array({Json{{"geometry", trim_loop}}})}};
    const auto bgfb = FlatSurface(torus).bytes;
    append<std::uint64_t>(packet, bgfb.size());
    packet.insert(packet.end(), bgfb.begin(), bgfb.end());
    auto component_for = [](const Bytes &packet) {
        Bytes body(142);
        body[0] = 1;
        body[134] = 1;
        append<std::uint32_t>(body, unsigned(packet.size()));
        body.insert(body.end(), packet.begin(), packet.end());
        append<std::int32_t>(body, 7);
        body.push_back(3);
        append<std::uint32_t>(body, 0);
        append<std::uint64_t>(body, 0);
        Bytes component(11);
        append<std::uint32_t>(component, 1);
        component.resize(component.size() + 11);
        append<std::uint32_t>(component, unsigned(body.size()));
        component.insert(component.end(), body.begin(), body.end());
        component.resize(component.size() + 25);
        return component;
    };
    const auto component = component_for(packet);
    const auto parsed = complex_blob("ParaCmptInstance", component);
    const auto &entry = parsed["instances"][0]["geometry_packets"][0];
    check(!entry.contains("decode_error") && entry["raw_base64"] == base64(packet) &&
              near(BsplineSurface::from_bgfb(entry["geometry"]["geometry"]).point_at(0, 0),
                   {4, 0, 0}),
          "native component packet feeds surface evaluator and retains complete packet bytes");
    const auto packet_surface = BsplineSurface::from_bgfb(entry["geometry"]["geometry"]);
    const auto packet_trim = packet_surface.trim(1e-6);
    check(packet_trim.report()["status"] == "complete" &&
              packet_trim.report()["loops"][0]["source_path"] == "/curves/0/geometry" &&
              packet_trim.classify({.5, .5}) == TrimLocation::Inside &&
              packet_trim.classify({.1, .1}) == TrimLocation::Outside,
          "complete component packet and nested BGFB trim tree reach native-parity UV "
          "classification");
    check(packet_trim.report()["ignored"].size() == 2 &&
              packet_trim.report()["ignored"][0]["source_type"] == "CurveVector" &&
              packet_trim.report()["ignored"][1]["source_type"] == "PointString" &&
              packet_surface.boundaries()["curves"][0]["geometry"]["curves"].size() == 3,
          "full packet retains direct child-array and point-string source members while native "
          "trim conversion excludes them");
    const auto &packet_akima =
        packet_surface.boundaries()["curves"][0]["geometry"]["curves"][0]["geometry"];
    check(packet_akima["_type"] == "AkimaCurve" && packet_akima["_akima"]["status"] == "valid" &&
              packet_akima["points"] == trim_line["points"] &&
              packet_akima["_akima"]["source_point_count"] == 9,
          "native BGFB type19 point vector survives full component decoding with Akima semantics");
    auto invalid_trim = torus;
    invalid_trim["boundaries"]["curves"][0]["geometry"]["curves"][0]["geometry"]["points"] =
        Json::array({0., 0., 0.});
    const auto invalid_packet = decode_bgfb(FlatSurface(invalid_trim).bytes);
    const auto &bad_akima =
        invalid_packet["geometry"]["boundaries"]["curves"][0]["geometry"]["curves"][0]["geometry"];
    check(bad_akima["_akima"]["status"] == "invalid" &&
              bad_akima["points"] == Json::array({0., 0., 0.}),
          "invalid BGFB Akima input retains complete points and explicit conversion error");
    Json interpolation = {{"_type", "InterpolationCurve"},
                          {"order", 7},
                          {"closed", true},
                          {"isChordLenKnots", 1},
                          {"isColinearTangents", -1},
                          {"isChordLenTangents", 2},
                          {"isNaturalTangents", 3},
                          {"startTangent", {{"x", 1}, {"y", 2}, {"z", 3}}},
                          {"endTangent", {{"x", 4}, {"y", 5}, {"z", 6}}},
                          {"fitPoints", {.2, .2, 0, .8, .2, 0, .8, .8, 0, .2, .8, 0}},
                          {"knots", {9, 8, 7}}};
    torus["boundaries"] = {{"_type", "CurveVector"},
                           {"type", 1},
                           {"curves", Json::array({{{"geometry", interpolation}}})}};
    const auto interpolation_fb = FlatSurface(torus).bytes;
    Bytes interpolation_packet(32);
    append<std::uint64_t>(interpolation_packet, interpolation_fb.size());
    interpolation_packet.insert(interpolation_packet.end(), interpolation_fb.begin(),
                                interpolation_fb.end());
    const auto ic = complex_blob("ParaCmptInstance", component_for(interpolation_packet));
    const auto &ip = ic["instances"][0]["geometry_packets"][0];
    const auto isurface = BsplineSurface::from_bgfb(ip["geometry"]["geometry"]);
    const auto &it = isurface.boundaries()["curves"][0]["geometry"];
    check(ip["raw_base64"] == base64(interpolation_packet) && it["_type"] == "InterpolationCurve" &&
              it["_interpolation"]["status"] == "valid" && it["order"] == 7 &&
              it["knots"] == interpolation["knots"] && it["startTangent"]["x"] == 1 &&
              it["endTangent"]["z"] == 6 && it["fitPoints"] == interpolation["fitPoints"],
          "full component BGFB type16 decoding retains inactive order, knots and tangents "
          "alongside conversion report");
    const auto itrim = isurface.trim(1e-5);
    check(itrim.report()["status"] == "complete" &&
              itrim.report()["loops"][0]["effective_boundary_type"] == 2 &&
              itrim.classify({.5, .5}) == TrimLocation::Inside &&
              itrim.classify({.01, .01}) == TrimLocation::Outside,
          "periodic interpolation endpoints and spans reach native Open promotion and trim "
          "classification");
    torus["boundaries"]["curves"][0]["geometry"]["fitPoints"] = {0., 0., 0.};
    const auto invalid_interpolation =
        decode_bgfb(FlatSurface(torus).bytes)["geometry"]["boundaries"]["curves"][0]["geometry"];
    check(invalid_interpolation["_interpolation"]["status"] == "invalid" &&
              invalid_interpolation["fitPoints"] == Json({0., 0., 0.}),
          "invalid interpolation retains source with explicit conversion error");
    return checks;
}
