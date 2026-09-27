#include "native_tube_facet_generation.hpp"
#include "native_curve_affine.hpp"
#include "bspline_frame.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
BsplineCurve curve(Json poles, unsigned order = 2, Json weights = nullptr) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", order},
                                    {"closed", false},
                                    {"poles", poles},
                                    {"knots", nullptr},
                                    {"weights", weights}});
}
bool near(double a, double b) {
    return std::abs(a - b) < 2e-11;
}
} // namespace
unsigned native_tube_facet_generation_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto generate = [](const BsplineCurve *p, const BsplineCurve *ps, const BsplineCurve *s,
                       const BsplineCurve *ss) {
        TubeBudget b;
        return generate_tube_facet_boundaries(p, ps, s, ss, false, b);
    };
    const auto path = curve({0, 0, 0, 0, 0, 2});
    const auto prefix = curve({0, 0, 0, 0, 0, -2});
    const auto section = curve({0, 0, 0, .2, 0, 0});
    const auto suffix = generate(nullptr, nullptr, &path, &section);
    check(suffix.status == TubeFacetSeamStatus::complete &&
              suffix.report["native_return_code"] == 0 && suffix.trimmed.surfaces.size() == 1 &&
              suffix.composition.report["seams_applied"] == true,
          "single suffix reaches native composition, seams and final boundary processing");
    check(suffix.report["trim_profile_source"] == 3 &&
              suffix.trimmed.report["path_preparation"]["method"] == "suffix_copy" &&
              suffix.report["all_trim_samples_succeeded"] == true,
          "suffix profile and combined path are selected by the native caller");
    const auto surface = BsplineSurface::from_bgfb(suffix.trimmed.surfaces[0]);
    for (double u : {0., .25, 1.})
        for (double v : {0., .4, 1.}) {
            const auto a = surface.point_at(u, v), base = surface.point_at(u, 0);
            check(
                near(a[0], base[0]) && near(a[1], base[1]) && near(a[2] - base[2], 2 * v),
                "generated straight sweep independently translates each profile point along path");
        }
    check(suffix.trimmed.boundaries[0].active_count == 0,
          "straight open sweep keeps natural tensor surface boundary instead of inventing trim");
    for (double height : {0., .1, 1.}) {
        const auto bent = curve({0, 0, 0, 1, 0, 0, 1, 1, height});
        const auto corner = generate(nullptr, nullptr, &bent, &section);
        check(corner.status == TubeFacetSeamStatus::complete &&
                  corner.trimmed.surfaces.size() == 2 &&
                  corner.composition.seams[0].classifier == 0,
              "bent linear path completes native corner projection");
        const auto a = BsplineSurface::from_bgfb(corner.trimmed.surfaces[0]);
        const auto z = BsplineSurface::from_bgfb(corner.trimmed.surfaces[1]);
        for (double u : {0., .25, 1.}) {
            const auto ap = a.point_at(u, 1), zp = z.point_at(u, 0);
            check(near(ap[0], zp[0]) && near(ap[1], zp[1]) && near(ap[2], zp[2]),
                  "independent evaluations meet across the projected linear seam");
        }
        const auto quadratic = curve({0, 0, 0, 1, 0, 0, 1, 1, height}, 3);
        const auto smooth = generate(nullptr, nullptr, &quadratic, &section);
        check(smooth.status == TubeFacetSeamStatus::complete && smooth.trimmed.surfaces.size() == 1,
              "quadratic path completes native curved facet generation");
        const auto curved = BsplineSurface::from_bgfb(smooth.trimmed.surfaces[0]);
        for (double v : {0., .25, .5, 1.}) {
            const auto p = curved.point_at(0, v);
            check(near(p[0], 2 * v - v * v) && near(p[1], v * v) && near(p[2], height * v * v),
                  "zero-offset profile edge follows independent quadratic path formula");
        }
    }
    const auto joined =
        BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                 {"order", 3},
                                 {"closed", false},
                                 {"poles", {0, 0, 0, 1, 0, 0, 1, 1, 0, 2, 1, 0, 2, 2, 0}},
                                 {"knots", {0, 0, 0, .5, .5, 1, 1, 1}},
                                 {"weights", nullptr}});
    const auto extended = generate(nullptr, nullptr, &joined, &section);
    check(extended.status == TubeFacetSeamStatus::complete &&
              extended.report["all_trim_samples_succeeded"] == true &&
              extended.trimmed.surfaces.size() == 2 &&
              extended.composition.seams[0].classifier == 1 &&
              extended.trimmed.boundaries[0].active_count == 1 &&
              extended.trimmed.boundaries[1].active_count == 1,
          "joined quadratics complete actual extension, seam sampling and both UV boundaries");
    const auto first = BsplineSurface::from_bgfb(extended.trimmed.surfaces[0]);
    const auto second = BsplineSurface::from_bgfb(extended.trimmed.surfaces[1]);
    const auto &plane = extended.composition.seams[0].plane->value;
    const auto &sampling = extended.trimmed.report["visits"][0]["sampling"];
    auto verify_seam = [&](const Json &query) {
        const double u = query.at("fraction");
        const auto &parameters = query.at("parameters");
        const auto a = first.point_at(u, parameters.at(0));
        const auto z = second.point_at(u, parameters.at(1));
        double distance = 0;
        for (unsigned axis = 0; axis < 3; ++axis)
            distance += (a[axis] - plane[0][axis]) * plane[1][axis];
        check(near(a[0], z[0]) && near(a[1], z[1]) && near(a[2], z[2]) &&
                  std::abs(distance) < 2e-11,
              "actual trimmed seam parameters reproject to coincident points on seam plane");
    };
    for (const auto &query : sampling.at("endpoint_queries"))
        verify_seam(query);
    for (const auto &interval : sampling.at("intervals"))
        for (const auto &query : interval.at("queries"))
            verify_seam(query);
    const auto pre = generate(&prefix, &section, nullptr, nullptr);
    check(pre.status == TubeFacetSeamStatus::complete && pre.report["trim_profile_source"] == 1 &&
              pre.trimmed.report["path_preparation"]["method"] == "reversed_prefix_copy",
          "single prefix uses reversed combined path and prefix section");
    const auto both = generate(&prefix, &section, &path, &section);
    check(both.status == TubeFacetSeamStatus::complete && both.trimmed.surfaces.size() == 2 &&
              both.report["trim_profile_source"] == 1 &&
              both.trimmed.report["path_preparation"]["method"] == "reversed_prefix_then_suffix",
          "both branches retain ordered independent patches through the complete native pipeline");
    check(both.composition.working_sources[1] == both.composition.working_sources[3] &&
              both.composition.working_sources[0] != both.composition.working_sources[2],
          "shared profile and separate paths retain original native identities");
    const auto unused_prefix = generate(nullptr, &section, &path, &section);
    check(unused_prefix.status == TubeFacetSeamStatus::complete &&
              unused_prefix.report["trim_profile_source"] == 1 &&
              unused_prefix.composition.working_sources[1] ==
                  unused_prefix.composition.working_sources[3],
          "nonnull prefix section remains selected even when prefix path is absent");
    const auto weighted = curve({.3, .7, .2, 1.1, 2.3, .4}, 2, {1.7, 2.9});
    const auto expected = native_bspline_frame_working(weighted, 0);
    const auto alias = generate(nullptr, nullptr, &weighted, &weighted);
    check(alias.status == TubeFacetSeamStatus::complete &&
              alias.composition.working_sources[2] == alias.composition.working_sources[3] &&
              alias.composition.working_sources[3]->poles() == expected.working_poles &&
              alias.report["trim_profile_source"] == 3,
          "selected section observes the preceding frame mutation when aliased to its path");
    const auto weighted_copy = weighted;
    const auto independent = generate(nullptr, nullptr, &weighted, &weighted_copy);
    check(independent.composition.working_sources[2] !=
                  independent.composition.working_sources[3] &&
              independent.composition.working_sources[2]->poles() == expected.working_poles &&
              independent.composition.working_sources[3]->poles() == weighted_copy.poles(),
          "equal independent section is not overwritten by path frame processing");
    check(weighted.poles()[0] == Point3{.3, .7, .2}, "original shared curve remains immutable");
    const auto absent = generate(nullptr, nullptr, nullptr, nullptr);
    const auto missing = generate(&path, nullptr, nullptr, nullptr);
    check(absent.status == TubeFacetSeamStatus::native_failure &&
              missing.status == TubeFacetSeamStatus::native_failure &&
              absent.composition.nodes.empty() && missing.trimmed.surfaces.empty(),
          "native missing source failures do not expose uninitialized or partial surfaces");
    TubeBudget b;
    b.max_work = 0;
    bool threw = false;
    try {
        generate_tube_facet_boundaries(nullptr, nullptr, &path, &section, false, b);
    } catch (const std::exception &) {
        threw = true;
    }
    check(threw, "resource failure is not disguised as a native generation return code");
    auto future = std::async(std::launch::async,
                             [&] { return generate(&prefix, &section, &path, &section); });
    const auto now = generate(&prefix, &section, &path, &section), other = future.get();
    check(now.report == other.report && now.trimmed.surfaces == other.trimmed.surfaces &&
              now.composition.report == other.composition.report,
          "complete generation with shared immutable inputs is deterministic across threads");
    return n;
}
