#include "native_tube_facet_output.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
BsplineCurve curve(Json poles) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", 2},
                                    {"closed", false},
                                    {"poles", poles},
                                    {"knots", nullptr},
                                    {"weights", nullptr}});
}
bool near(double a, double b) {
    return std::abs(a - b) < 2e-11;
}
} // namespace
unsigned native_tube_facet_output_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    const auto path = curve({0, 0, 0, 0, 0, 2});
    const auto section = curve({0, 0, 0, .2, 0, 0});
    std::vector<TubeFacetSurface> surfaces;
    std::vector<bool> flags;
    TubeBudget b;
    auto output =
        append_tube_facet_surfaces(surfaces, &flags, nullptr, nullptr, &path, &section, b);
    check(output.report["native_return_value"] == true && surfaces.size() == 1 &&
              flags == std::vector<bool>{false} && surfaces[0].boundaries.empty(),
          "actual straight generation is copied with native classifier-two output flag");
    check(surfaces[0].geometry == output.generation.trimmed.surfaces[0],
          "explicit source knots and generated geometry survive native surface copying");
    const auto original = surfaces[0].geometry;
    output.generation.trimmed.surfaces[0]["poles"][0] = 900;
    check(surfaces[0].geometry == original, "output geometry does not alias generation storage");
    auto empty =
        append_tube_facet_surfaces(surfaces, &flags, nullptr, nullptr, nullptr, nullptr, b);
    check(empty.report["native_return_value"] == true &&
              empty.report["generation_status"] == "native_failure" &&
              empty.report["appended_surface_count"] == 0 && surfaces.size() == 1 &&
              flags == std::vector<bool>{false},
          "empty failed inner generation completes outer wrapper and preserves existing outputs");
    auto missing =
        append_tube_facet_surfaces(surfaces, &flags, &path, nullptr, nullptr, nullptr, b);
    check(missing.report["native_return_value"] == true && surfaces[0].geometry == original,
          "missing section is an inner failure rather than an outer surface-copy failure");
    const auto bent = curve({0, 0, 0, 1, 0, 0, 1, 1, 0});
    const auto corners =
        append_tube_facet_surfaces(surfaces, &flags, nullptr, nullptr, &bent, &section, b);
    check(surfaces.size() == 3 && flags == std::vector<bool>({false, true, false}) &&
              corners.report["initial_surface_count"] == 1 &&
              corners.report["appended_surface_count"] == 2,
          "native wrapper appends projected corner facets and aligned flags in source order");
    const auto a = BsplineSurface::from_bgfb(surfaces[1].geometry);
    const auto z = BsplineSurface::from_bgfb(surfaces[2].geometry);
    for (double u : {0., .25, 1.}) {
        const auto ap = a.point_at(u, 1), zp = z.point_at(u, 0);
        check(near(ap[0], zp[0]) && near(ap[1], zp[1]) && near(ap[2], zp[2]),
              "copied projected surfaces retain independently evaluated seam positions");
    }
    append_tube_facet_surfaces(surfaces, nullptr, nullptr, nullptr, &path, &section, b);
    check(surfaces.size() == 4 && flags.size() == 3,
          "omitted flag collection remains untouched while surface output is appended");
    surfaces[3].geometry["poles"][0] = 700;
    check(surfaces[0].geometry == original,
          "equal generated surfaces from separate calls remain independently owned");
    const auto joined =
        BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                 {"order", 3},
                                 {"closed", false},
                                 {"poles", {0, 0, 0, 1, 0, 0, 1, 1, 0, 2, 1, 0, 2, 2, 0}},
                                 {"knots", {0, 0, 0, .5, .5, 1, 1, 1}},
                                 {"weights", nullptr}});
    std::vector<TubeFacetSurface> trimmed;
    std::vector<bool> trim_flags;
    const auto high =
        append_tube_facet_surfaces(trimmed, &trim_flags, nullptr, nullptr, &joined, &section, b);
    check(trimmed.size() == 2 && trim_flags == std::vector<bool>({true, false}) &&
              trimmed[0].boundaries.size() == 1 && trimmed[1].boundaries.size() == 1 &&
              high.generation.report["all_trim_samples_succeeded"] == true,
          "actual high-order extension and trim reach final independent surface copies");
    check(trimmed[0].geometry["holeOrigin"] == 1 && trimmed[0].geometry["boundaries"].is_null() &&
              trimmed[0].boundaries[0] == high.generation.trimmed.boundaries[0].allocated[0],
          "copied native UV boundary stays separate from BGFB trim-tree encoding");

    auto table = original;
    table["knotsU"] = nullptr;
    table["knotsV"] = nullptr;
    table["weights"] = {1., 2., 3., 4.};
    table["holeOrigin"] = 7;
    TubeFacetUvBoundaryState storage{
        {{{-.2, .3}, {.4, 1.8}, {-.2, .3}}, {{9., 10.}, {11., 12.}}, {{800., 900.}}}, 2};
    const auto saved = storage.allocated;
    auto copy = copy_tube_facet_surface(table, storage, b);
    check(copy.geometry["knotsU"] == Json({0., 0., 1., 1.}) &&
              copy.geometry["knotsV"] == Json({0., 0., 1., 1.}) && table["knotsU"].is_null(),
          "missing knots are generated only in independent output");
    check(copy.geometry["poles"] == table["poles"] &&
              copy.geometry["weights"] == table["weights"] && copy.geometry["holeOrigin"] == 7,
          "surface copying preserves weighted controls and raw hole-origin state");
    check(copy.boundaries.size() == 2 && copy.boundaries[0] == saved[0] &&
              copy.boundaries[1] == saved[1],
          "only active boundaries are copied without normalizing, welding or closing them");
    copy.boundaries[0][0] = {88., 99.};
    check(storage.allocated == saved, "copied UV storage is independent of source allocations");
    storage.active_count = 0;
    auto inactive = copy_tube_facet_surface(table, storage, b);
    check(inactive.boundaries.empty() && inactive.geometry["holeOrigin"] == 7,
          "inactive allocation slots are discarded without rewriting hole-origin state");
    table["knotsU"] = {2., 2., 5., 5.};
    table["knotsV"] = {-3., -3., 8., 8.};
    auto nonunit = copy_tube_facet_surface(table, storage, b);
    check(nonunit.geometry == table, "explicit nonunit knots are copied without normalization");
    table["knotsU"] = Json::array();
    auto no_knots = copy_tube_facet_surface(table, storage, b);
    check(no_knots.geometry["knotsU"] == Json({0., 0., 1., 1.}) &&
              no_knots.geometry["knotsV"] == table["knotsV"],
          "empty source knot buffer is materialized without changing the other direction");
    storage.active_count = 4;
    bool threw = false;
    try {
        copy_tube_facet_surface(table, storage, b);
    } catch (const std::exception &) {
        threw = true;
    }
    check(threw, "invalid active count cannot silently expand allocated boundary storage");
    storage.active_count = 0;
    table["boundaries"] = Json::object();
    threw = false;
    try {
        copy_tube_facet_surface(table, storage, b);
    } catch (const std::exception &) {
        threw = true;
    }
    check(threw, "source trim trees are not misinterpreted as generated raw UV boundaries");
    TubeBudget measured;
    generate_tube_facet_boundaries(nullptr, nullptr, &path, &section, false, measured);
    TubeBudget limited;
    limited.max_work = measured.work;
    threw = false;
    try {
        append_tube_facet_surfaces(surfaces, &flags, nullptr, nullptr, &path, &section, limited);
    } catch (const std::exception &) {
        threw = true;
    }
    check(threw && surfaces.empty() && flags.empty(),
          "copy failure clears all previous output entries while resource failure remains an "
          "exception");
    TubeBudget measurement;
    const auto source_pair =
        generate_tube_facet_boundaries(nullptr, nullptr, &bent, &section, false, measurement);
    copy_tube_facet_surface(source_pair.trimmed.surfaces[0], source_pair.trimmed.boundaries[0],
                            measurement);
    TubeBudget second_copy_limit;
    second_copy_limit.max_work = measurement.work;
    surfaces.push_back({original, {}});
    flags.push_back(false);
    threw = false;
    try {
        append_tube_facet_surfaces(surfaces, &flags, nullptr, nullptr, &bent, &section,
                                   second_copy_limit);
    } catch (const std::exception &) {
        threw = true;
    }
    check(threw && surfaces.empty() && flags.empty(),
          "later copy failure clears preexisting and newly appended surfaces and flags together");
    auto run = [&] {
        TubeBudget local;
        std::vector<TubeFacetSurface> s;
        auto r = append_tube_facet_surfaces(s, nullptr, nullptr, nullptr, &joined, &section, local);
        return std::make_pair(s[0].geometry, r.report);
    };
    auto future = std::async(std::launch::async, run);
    check(run() == future.get(), "independent output collections are deterministic across threads");
    return n;
}
