#include "native_tube_facet_groups.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
Json line(Point3 a, Point3 b) {
    Json p;
    for (unsigned i = 0; i < 3; ++i) {
        p[std::string("point0") + "XYZ"[i]] = a[i];
        p[std::string("point1") + "XYZ"[i]] = b[i];
    }
    return {{"_type", "LineSegment"}, {"segment", p}};
}
Json group(Json values, unsigned type = 1) {
    Json members = Json::array();
    for (const auto &v : values)
        members.push_back({{"geometry", v}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", members}};
}
std::vector<Point3> vertices(double size, bool reverse) {
    std::vector<Point3> points{
        {-size, -size, 0}, {size, -size, 0}, {size, size, 0}, {-size, size, 0}};
    if (reverse)
        std::reverse(points.begin(), points.end());
    return points;
}
Json square(double size, bool reverse) {
    const auto points = vertices(size, reverse);
    Json lines = Json::array();
    for (unsigned i = 0; i < 4; ++i)
        lines.push_back(line(points[i], points[(i + 1) % 4]));
    return group(lines, 2);
}
bool near(double a, double b) {
    return std::abs(a - b) < 2e-11;
}
} // namespace
unsigned native_tube_facet_groups_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto f, const char *why) {
        bool caught = false;
        try {
            f();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, why);
    };
    const auto path = group({line({0, 0, 0}, {0, 0, 4})});
    auto generate = [&](const Json &profile, const Json &trace) {
        TubeBudget b;
        return generate_tube_facet_groups(profile, trace, b);
    };
    for (bool reverse : {false, true}) {
        const auto profile = square(1, reverse);
        const auto saved = profile;
        const auto result = generate(profile, path);
        check(result.success && result.groups.size() == 1 && result.groups[0].size() == 4 &&
                  result.preparation.orientation["flags"] == Json::array({reverse}),
              "complete source profile and path reach native member surface groups");
        check(result.report["reversed_groups"] == (reverse ? Json::array({0}) : Json::array()),
              "source orientation reverses exactly the selected member list");
        const auto points = vertices(1, reverse);
        for (unsigned j = 0; j < 4; ++j) {
            const auto &member = result.groups[0][j];
            const auto index = reverse ? 3 - j : j;
            check(member.source_member == index && member.surfaces.size() == 1,
                  "source member provenance survives positional member-list reversal");
            const auto surface = BsplineSurface::from_bgfb(member.surfaces[0].geometry);
            const auto a = points[reverse ? (index + 1) % 4 : index];
            const auto z = points[reverse ? index : (index + 1) % 4];
            for (double u : {0., .3, 1.})
                for (double v : {0., .4, 1.}) {
                    const auto actual = surface.point_at(u, v);
                    check(near(actual[0], (1 - u) * a[0] + u * z[0]) &&
                              near(actual[1], (1 - u) * a[1] + u * z[1]) && near(actual[2], 4 * v),
                          "full source pipeline agrees with independent rectangular prism side "
                          "formula");
                }
        }
        check(profile == saved, "full grouping leaves source profile immutable");
    }
    const auto parity = group({square(2, false), square(1, true)}, 4);
    auto rings = generate(parity, path);
    check(rings.success && rings.groups.size() == 2 && rings.groups[0].size() == 4 &&
              rings.groups[1].size() == 4,
          "native profile rings remain distinct groups with all their members");
    check(rings.groups[0][0].surfaces[0].geometry != rings.groups[1][0].surfaces[0].geometry,
          "different native rings are not merged into one profile surface");
    const auto bent = group({line({0, 0, 0}, {0, 0, 4}), line({0, 0, 4}, {2, 0, 4})});
    const auto corner = generate(square(1, true), bent);
    check(corner.success && corner.groups[0].size() == 4,
          "multi-member reversed profile completes actual bent-path facet generation");
    for (const auto &m : corner.groups[0]) {
        check(m.surfaces.size() == 2, "each bent-path member retains both native facets");
        const auto a = BsplineSurface::from_bgfb(m.surfaces[0].geometry);
        const auto z = BsplineSurface::from_bgfb(m.surfaces[1].geometry);
        check(near(a.point_at(0, 0)[2], 0),
              "member-list reversal does not reverse facet traversal");
        for (double u : {0., .5, 1.}) {
            const auto ap = a.point_at(u, 1), zp = z.point_at(u, 0);
            check(near(ap[0], zp[0]) && near(ap[1], zp[1]) && near(ap[2], zp[2]),
                  "actual member facets meet at the projected native seam");
        }
    }
    TubeBudget preparation_budget;
    auto prepared = prepare_tube_facet_inputs(square(1, false), path, preparation_budget);
    const auto weighted = BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                                   {"order", 2},
                                                   {"closed", false},
                                                   {"poles", {1. / 37., .7, .2, 1.1, 2.3, .4}},
                                                   {"weights", {45. / 23., 2.9}},
                                                   {"knots", nullptr}});
    auto &branches = prepared.placement->branches;
    const auto reused_index = *branches.suffix.reused_curve_index;
    branches.path.selection.curves[reused_index] = weighted;
    TubeBudget working_budget;
    const auto working = generate_tube_facet_groups(prepared, working_budget);
    const auto &final = working.preparation.placement->branches;
    double expected = 1. / 37., reset_each_member = expected;
    const double weight = 45. / 23.;
    // Each linear rational source-frame query has two actual homogeneous
    // round trips. Four ordered members must accumulate all eight round trips.
    for (unsigned i = 0; i < 8; ++i) {
        expected = (expected * (1 / weight)) * weight;
        if (i == 1)
            reset_each_member = expected;
    }
    check(working.success && final.suffix.reused_curve_index == reused_index &&
              final.path.selection.curves[reused_index].poles()[0][0] == expected &&
              expected != reset_each_member,
          "source-index-reused path carries exact working mutations across every profile member");
    check(branches.path.selection.curves[reused_index].poles() == weighted.poles(),
          "working state in generated groups does not alter input preparation");
    auto shared = prepared;
    shared.placement->prefix_transform = shared.placement->suffix_transform;
    shared.placement->branches.prefix.reused_curve_index = reused_index;
    TubeBudget shared_budget;
    const auto shared_groups = generate_tube_facet_groups(shared, shared_budget);
    double shared_expected = 1. / 37.;
    for (unsigned i = 0; i < 16; ++i)
        shared_expected = (shared_expected * (1 / weight)) * weight;
    const auto &shared_paths = shared_groups.preparation.placement->branches;
    check(shared_groups.success && shared_groups.report["shared_path"] == true &&
              shared_paths.prefix.reused_curve_index == shared_paths.suffix.reused_curve_index &&
              shared_paths.path.selection.curves[reused_index].poles()[0][0] == shared_expected,
          "aliased path branches carry both ordered mutations through every source member");
    for (const auto &m : shared_groups.groups[0])
        check(m.surfaces.size() == 2,
              "shared native path identity does not merge separately generated branch surfaces");
    auto empty = prepared;
    empty.profile.groups = {{}};
    TubeBudget empty_budget;
    const auto empty_group = generate_tube_facet_groups(empty, empty_budget);
    check(empty_group.success && empty_group.groups.size() == 1 && empty_group.groups[0].empty() &&
              empty_group.report["first_member_available"] == false,
          "empty partition group remains explicit and is not mistaken for a final entity");
    auto bad_flags = prepared;
    bad_flags.orientation["flags"] = Json::array();
    rejects(
        [&] {
            TubeBudget b;
            generate_tube_facet_groups(bad_flags, b);
        },
        "missing native positional orientation flag is never inferred");
    auto invalid_path = prepared;
    invalid_path.placement->branches.suffix.reused_curve_index = 999;
    rejects(
        [&] {
            TubeBudget b;
            generate_tube_facet_groups(invalid_path, b);
        },
        "invalid reused native path index is rejected");
    auto unavailable = generate(square(1, false), nullptr);
    check(!unavailable.success && unavailable.groups.empty() &&
              unavailable.report["failure_step"] == "input_preparation",
          "source validation failure produces no partial surface groups");
    rejects(
        [&] {
            TubeBudget b;
            b.max_work = 0;
            generate_tube_facet_groups(prepared, b);
        },
        "group generation obeys shared work budget");
    auto other = std::async(std::launch::async, [&] { return generate(parity, path); });
    const auto concurrent = generate(parity, path);
    const auto parallel = other.get();
    check(parallel.report == concurrent.report && parallel.groups[1][2].surfaces[0].geometry ==
                                                      concurrent.groups[1][2].surfaces[0].geometry,
          "source-to-group generation is deterministic with concurrent immutable inputs");
    return n;
}
