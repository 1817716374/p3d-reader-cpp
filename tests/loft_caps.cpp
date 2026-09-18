#include "internal.hpp"
#include <future>
using namespace p3d;
namespace {
Json array(Json geometries, unsigned type) {
    Json curves = Json::array();
    for (auto &g : geometries)
        curves.push_back({{"_type", "VariantGeometry"}, {"geometry", std::move(g)}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", curves}};
}
Json line(Point3 a, Point3 b) {
    return {{"_type", "BsplineCurve"}, {"order", 2},
            {"closed", false},         {"knots", nullptr},
            {"weights", nullptr},      {"poles", {a[0], a[1], a[2], b[0], b[1], b[2]}}};
}
Json source(unsigned loops = 1, unsigned type = 2, double gap = 0) {
    Json lower = Json::array(), upper = Json::array(), groups = Json::array();
    for (unsigned ring = 0; ring < loops; ++ring) {
        const double a = ring ? .5 : 0, b = ring ? 1.5 : 2;
        std::vector<Point3> points = {{a, a, 0}, {b, a, 0}, {b, b, 0}, {a, b, 0}, {a + gap, a, 0}};
        if (ring)
            std::reverse(points.begin(), points.end());
        Json bottom = Json::array(), top = Json::array(), guides = Json::array();
        for (unsigned i = 0; i < 4; ++i) {
            auto x = points[i], y = points[i + 1], z = x, w = y;
            z[2] = w[2] = 3;
            bottom.push_back(line(x, y));
            top.push_back(line(z, w));
            guides.push_back(array(Json::array({line(x, z)}), 1));
        }
        if (type == 1) {
            auto x = points.back(), y = x;
            y[2] = 3;
            guides.push_back(array(Json::array({line(x, y)}), 1));
        }
        lower.push_back(array(bottom, ring ? 3 : type));
        upper.push_back(array(top, ring ? 3 : type));
        groups.push_back(guides);
    }
    return {{"_type", "P3DSectionLoft"},
            {"capped", true},
            {"section0", loops == 1 ? lower[0] : array(lower, 4)},
            {"section1", loops == 1 ? upper[0] : array(upper, 4)},
            {"guide_groups", groups}};
}
bool near(Point3 a, Point3 b) {
    return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) < 3e-12;
}
Json near_knots(bool rational, bool unsafe = false, unsigned type = 2, double delta = 1.5e-14) {
    auto input = source(1, type);
    for (auto &entry : input["section1"]["curves"])
        for (std::size_t i = 2; i < entry["geometry"]["poles"].size(); i += 3)
            entry["geometry"]["poles"][i] = 3000000;
    for (auto &guide : input["guide_groups"][0]) {
        auto &curve = guide["curves"][0]["geometry"];
        const double x = curve["poles"][0], y = curve["poles"][1];
        const unsigned count = unsafe ? 6 : 5;
        Json poles = Json::array(), weights = Json::array();
        for (unsigned row = 0; row < count; ++row) {
            const double w = rational ? row + 1 : 1;
            poles.push_back(x * w);
            poles.push_back(y * w);
            poles.push_back((3000000.0 * row / (count - 1)) * w);
            weights.push_back(w);
        }
        curve["poles"] = poles;
        curve["weights"] = rational ? weights : Json(nullptr);
        curve["knots"] = unsafe ? Json{0, 0, .25, .25 + delta, .75, .75 + delta, 1, 1}
                                : Json{0, 0, .5, .5 + delta, .75, 1, 1};
    }
    return input;
}
} // namespace
unsigned loft_caps_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *message) {
        ++checks;
        require(value, message);
    };
    auto verify = [&](const SectionLoft &loft) {
        const auto result = loft.cap_regions();
        check(result.report.at("status") == "complete", "loft cap regions complete");
        check(result.report.at("boundary_curves").size() == loft.sides().size() * 2,
              "each cap curve has a source side correspondence");
        for (const auto &link : result.report.at("boundary_curves")) {
            const auto &region = link.at("cap") == "bottom" ? result.bottom : result.top;
            const auto &table =
                region.at(Json::json_pointer(link.at("region_path").get<std::string>()));
            const auto curve = BsplineCurve::from_bgfb(table);
            const auto &side = loft.sides().at(link.at("side_index").get<std::size_t>());
            require(link.at("loop_index") == side.loop_index &&
                        link.at("primitive_index") == side.primitive_index,
                    "cap provenance indices");
            for (unsigned i = 0; i <= 16; ++i) {
                const double t = i / 16.0;
                require(near(curve.point_at(t),
                             side.surface.point_at(link.at("u_reversed").get<bool>() ? 1 - t : t,
                                                   link.at("surface_v").get<double>())),
                        "cap curve matches actual side isocurve");
            }
        }
        check(result.report.at("mesh_status") == "not_evaluated" &&
                  result.report.at("planarity_status") == "not_evaluated",
              "cap boundary success is not a meshed or planar solid claim");
        return result;
    };
    const auto input = source();
    const auto loft = SectionLoft::from_bgfb(input);
    const auto caps = verify(loft);
    check(caps.bottom.at("type") == 2 && caps.top.at("type") == 2,
          "single source loop produces outer cap regions");
    auto signed_area = [&](const Json &region) {
        double sum = 0;
        for (const auto &entry : region.at("curves")) {
            auto c = BsplineCurve::from_bgfb(entry.at("geometry"));
            const auto a = c.point_at(0), b = c.point_at(1);
            sum += a[0] * b[1] - a[1] * b[0];
        }
        return sum / 2;
    };
    check(signed_area(caps.bottom) == -4 && signed_area(caps.top) == 4,
          "native bottom reversal and top orientation");
    check(caps.report["boundary_curves"][0]["side_index"] == 3 &&
              caps.report["boundary_curves"][4]["side_index"] == 0,
          "bottom primitive order reversed without reordering source sides");
    verify(SectionLoft::from_bgfb(source(1, 1)));
    const auto inner = verify(SectionLoft::from_bgfb(source(1, 3)));
    check(inner.bottom["type"] == 2, "single inner-labelled source is still an outer cap");
    const auto holes = verify(SectionLoft::from_bgfb(source(2)));
    const std::vector<std::array<std::int64_t, 3>> expected_faces = {
        {-1, 0, 0}, {-1, 1, 0}, {0, 0, 0}, {0, 1, 0}, {0, 2, 0},
        {0, 3, 0},  {0, 4, 0},  {0, 5, 0}, {0, 6, 0}, {0, 7, 0}};
    const auto hole_faces = SectionLoft::from_bgfb(source(2)).face_indices();
    check(hole_faces.report["status"] == "complete" && hole_faces.indices == expected_faces,
          "native cap-first face enumeration uses one side counter across loops");
    check(hole_faces.report["cap_count"] == 2 && hole_faces.report["side_count"] == 8 &&
              hole_faces.report["material_part_mapping"] == "not_established",
          "a parity cap is one face, and face identity is not a material part mapping");
    auto composite = source(2);
    for (auto &group : composite["guide_groups"])
        for (auto &guide : group) {
            const auto poles = guide["curves"][0]["geometry"]["poles"];
            Point3 a{poles[0], poles[1], poles[2]}, b{poles[3], poles[4], poles[5]};
            Json segments = Json::array();
            for (unsigned i = 0; i < 4; ++i) {
                auto x = a, y = a;
                for (unsigned k = 0; k < 3; ++k) {
                    x[k] += (b[k] - a[k]) * i / 4;
                    y[k] += (b[k] - a[k]) * (i + 1) / 4;
                }
                segments.push_back(line(x, y));
            }
            guide = array(segments, 1);
        }
    const auto composite_loft = SectionLoft::from_bgfb(composite);
    check(composite_loft.sides()[0].surface.v().order() == 2 &&
              composite_loft.sides()[0].surface.v().pole_count() > 4 &&
              composite_loft.face_indices().indices == expected_faces,
          "linear-V composite guides do not create additional native face identities");
    check(holes.bottom["type"] == 4 && holes.top["type"] == 4 &&
              holes.bottom["curves"][0]["geometry"]["type"] == 2 &&
              holes.bottom["curves"][1]["geometry"]["type"] == 3,
          "native parity cap structure preserves outer then inner loop order");
    check(signed_area(holes.bottom["curves"][0]["geometry"]) == -4 &&
              signed_area(holes.bottom["curves"][1]["geometry"]) == 1,
          "bottom reversal applies inside each loop, not to region child order");
    auto curved = source();
    for (const auto name : {"section0", "section1"}) {
        auto &c = curved[name]["curves"][0]["geometry"];
        const double z = std::string(name) == "section0" ? 0 : 3;
        c["order"] = 3;
        c["knots"] = {0, 0, 0, .3, 1, 1, 1};
        c["weights"] = {1, 2, .75, 1};
        c["poles"] = {0, 0, z, 1, -1, 2 * z, 1.125, -.375, .75 * z, 2, 0, z};
    }
    const auto rational = verify(SectionLoft::from_bgfb(curved));
    const auto reversed = BsplineCurve::from_bgfb(rational.bottom["curves"][3]["geometry"]);
    check(reversed.rational() && reversed.knots()[3] == .7,
          "nonuniform cap reversal mirrors knots and preserves rational representation");
    auto nonplanar = curved;
    nonplanar["section0"]["curves"][0]["geometry"]["poles"][5] = 1;
    verify(SectionLoft::from_bgfb(nonplanar));
    check(SectionLoft::from_bgfb(nonplanar).face_indices().report["status"] == "complete",
          "native face enumeration does not require planar cap triangulation");
    auto altered = source();
    // Coons construction accepts this corner discrepancy. Extracted cap must
    // follow the final side, not return the unmodified source profile.
    altered["section0"]["curves"][0]["geometry"]["poles"][3] = 2.000001;
    const auto changed = verify(SectionLoft::from_bgfb(altered));
    const auto cap_curve = BsplineCurve::from_bgfb(changed.bottom["curves"][3]["geometry"]);
    check(near(cap_curve.point_at(0), {2, 0, 0}), "cap follows constructed side endpoint");
    verify(SectionLoft::from_bgfb(source(1, 1, 1e-11)));
    const auto failed = SectionLoft::from_bgfb(source(1, 1, 1e-9)).cap_regions();
    const auto failed_faces = SectionLoft::from_bgfb(source(1, 1, 1e-9)).face_indices();
    check(failed_faces.indices.empty() && failed_faces.report["status"] == "native_failure",
          "native requested cap failure prevents even side face enumeration");
    check(failed.report["status"] == "native_failure" && failed.bottom.is_null() &&
              failed.top.is_null(),
          "open cap rejected without a synthetic closing segment or partial region");
    auto top_gap = source(1, 1);
    top_gap["section1"]["curves"][3]["geometry"]["poles"][3] = 1e-8;
    top_gap["guide_groups"][0][4]["curves"][0]["geometry"]["poles"][3] = 1e-8;
    const auto failed_top = SectionLoft::from_bgfb(top_gap).cap_regions();
    check(failed_top.report["status"] == "native_failure" &&
              failed_top.report["failed_end"] == "top" && failed_top.bottom.is_null(),
          "failed top discards completed bottom region");
    auto no_caps = source(1, 1, 1e-9);
    no_caps["capped"] = false;
    const auto none = SectionLoft::from_bgfb(no_caps).cap_regions();
    const auto uncapped_faces = SectionLoft::from_bgfb(no_caps).face_indices();
    check(uncapped_faces.report["status"] == "complete" &&
              uncapped_faces.report["cap_status"] == "not_requested" &&
              uncapped_faces.indices ==
                  std::vector<std::array<std::int64_t, 3>>{
                      {0, 0, 0}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}},
          "uncapped native face enumeration ignores an unclosed end profile");
    check(none.report["status"] == "not_requested" && none.bottom.is_null(),
          "uncapped source does not attempt cap closure");
    const auto budget = loft.cap_regions(15);
    const auto budget_faces = loft.face_indices(15);
    check(budget_faces.report["status"] == "incomplete" && budget_faces.indices.empty(),
          "budget-limited cap verification cannot fabricate a complete face index set");
    check(budget.report["status"] == "incomplete" && budget.bottom.is_null() &&
              budget.top.is_null(),
          "total cap control budget does not expose partial success");
    bool invalid = false;
    try {
        loft.cap_regions(0);
    } catch (const std::exception &) {
        invalid = true;
    }
    check(invalid, "zero cap budget rejected");
    invalid = false;
    try {
        loft.face_indices(0);
    } catch (const std::exception &) {
        invalid = true;
    }
    check(invalid, "zero face-enumeration cap budget rejected");
    auto face_future = std::async(std::launch::async, [&] { return loft.face_indices(); });
    const auto concurrent_faces = face_future.get();
    check(concurrent_faces.indices == loft.face_indices().indices &&
              concurrent_faces.report == loft.face_indices().report,
          "native face enumeration is concurrent and repeatable");
    const auto plain_native = loft.native_faces();
    check(plain_native.report["status"] == "complete" &&
              plain_native.report["side_adjustments"].empty() &&
              plain_native.caps.bottom == caps.bottom,
          "ordinary native faces preserve unmodified side and cap geometry");
    const auto narrow_loft = SectionLoft::from_bgfb(near_knots(false));
    const auto narrow = narrow_loft.native_faces();
    check(narrow.report["status"] == "complete" && narrow.sides.size() == 4 &&
              narrow.sides[0].surface.v().pole_count() == 4 &&
              narrow.sides[0].surface.v().knots() == std::vector<double>{0, 0, .5, .75, 1, 1},
          "native relative knot compression removes the later near-coincident linear row");
    check(narrow.sides[0].surface.poles()[4] == narrow_loft.sides()[0].surface.poles()[6] &&
              narrow_loft.sides()[0].surface.v().pole_count() == 5,
          "native cleaned geometry and original side remain separate");
    const auto outside = SectionLoft::from_bgfb(near_knots(false, false, 2, 4e-14)).native_faces();
    check(outside.report["status"] == "complete" && outside.report["side_adjustments"].empty(),
          "knots outside the native strict relative tolerance stay distinct");
    const auto weighted_loft = SectionLoft::from_bgfb(near_knots(true));
    const auto weighted = weighted_loft.native_faces();
    check(weighted.report["status"] == "complete" &&
              weighted.sides[0].surface.weights() == std::vector<double>{1, 1, 2, 2, 3, 4, 5, 5} &&
              weighted.report["side_adjustments"][0]["removed_source_weight_indices"] == Json{4, 6},
          "native weight erasure has independent moving-array indices");
    const auto &native_surface = weighted.sides[0].surface;
    check(native_surface.poles()[4][2] == weighted_loft.sides()[0].surface.poles()[6][2] * 3 &&
              native_surface.point_at(0, 1)[2] == 15000000 &&
              weighted_loft.sides()[0].surface.point_at(0, 1)[2] == 3000000,
          "native replacement constructor reweights stored poles and changes evaluated geometry");
    const auto native_top_curve =
        BsplineCurve::from_bgfb(weighted.caps.top["curves"][0]["geometry"]);
    check(native_top_curve.point_at(0)[2] == 15000000,
          "native caps come from cleaned surfaces rather than the original sides");
    const auto unsafe_loft = SectionLoft::from_bgfb(near_knots(true, true));
    const auto unsafe_native = unsafe_loft.native_faces();
    check(unsafe_native.report["status"] == "incomplete" && unsafe_native.sides.empty() &&
              unsafe_native.caps.bottom.is_null() && unsafe_loft.face_indices().indices.empty(),
          "out-of-range native weight removal is reported without partial geometry or identities");
    auto open_weighted = near_knots(true, false, 1);
    auto &last = open_weighted["guide_groups"][0][4]["curves"][0]["geometry"];
    last["weights"][4] = 6;
    last["poles"][14] = 18000000;
    const auto open_loft = SectionLoft::from_bgfb(open_weighted);
    const auto open_native = open_loft.native_faces();
    check(open_loft.cap_regions().report["status"] == "complete" &&
              open_native.report["status"] == "native_failure" && open_native.sides.empty() &&
              open_loft.face_indices().indices.empty(),
          "native cleanup can invalidate closure although derived side caps close");
    LoftMeshOptions mapping_options;
    mapping_options.max_uv_edge = .5;
    const auto derived_mesh = unsafe_loft.mesh(mapping_options);
    check(derived_mesh.report["status"] == "complete" &&
              derived_mesh.report["native_face_indices"]["status"] == "incomplete" &&
              std::all_of(derived_mesh.parts.begin(), derived_mesh.parts.end(),
                          [](const LoftMeshPart &part) { return !part.native_face_indices; }),
          "derived mesh cannot claim native face indices when native reconstruction fails");
    check(weighted_loft.source() == near_knots(true),
          "native face extraction preserves source data");
    const auto small_budget = weighted_loft.native_faces(1);
    check(small_budget.report["status"] == "incomplete" && small_budget.sides.empty(),
          "native cap budget failure clears the whole requested face set");
    auto native_future =
        std::async(std::launch::async, [&] { return weighted_loft.native_faces(); });
    check(native_future.get().caps.top == weighted.caps.top,
          "native face reconstruction is concurrent");
    auto future = std::async(std::launch::async, [&] { return loft.cap_regions(); });
    check(future.get().bottom == caps.bottom && loft.source() == input &&
              loft.cap_regions().report == caps.report,
          "cap reconstruction is immutable and concurrent");
    return checks;
}
