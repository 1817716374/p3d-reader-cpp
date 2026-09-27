#include "native_tube_mesh_sampling.hpp"
#include <p3d/swept_patches.hpp>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_tube_mesh_sampling_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    const Json path{{"_type", "LineSegment"},
                    {"segment",
                     {{"point0X", 0},
                      {"point0Y", 0},
                      {"point0Z", 0},
                      {"point1X", 0},
                      {"point1Y", 0},
                      {"point1Z", 4}}}};
    const Json square{{"_type", "LineString"},
                      {"points", {0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, 0, 0, 0}}};
    auto surface = [&](const Json &profile) {
        const auto r = reconstruct_bgfb_swept_body_patches(
            {{"_type", "P3DSweptBody"},
             {"profile",
              {{"_type", "CurveVector"},
               {"type", 2},
               {"curves", Json::array({{{"geometry", profile}}})}}},
             {"path",
              {{"_type", "CurveVector"},
               {"type", 1},
               {"curves", Json::array({{{"geometry", path}}})}}}});
        require(r.status == "reconstructed" && r.groups.size() == 1 &&
                    r.groups[0].patches.size() == 1,
                "mesh sampling fixture reconstructs native whole-section patch");
        return BsplineSurface::from_bgfb(r.groups[0].patches[0].geometry);
    };
    const auto patch = surface(square);
    TubeBudget b;
    auto s = sample_tube_mesh_section(patch, true, .01, .2, 100000, b);
    check(s.success && s.interval_samples.size() == 4 && s.end_discontinuity &&
              s.break_set == std::vector<double>{0, .25, .5, .75, 1},
          "whole square section retains all four corners including closing corner");
    check(s.discontinuity_intervals == std::vector<std::size_t>{0, 1, 2, 3},
          "source compressed knot intervals carry native discontinuity marks");
    for (const auto &samples : s.interval_samples)
        check(samples == std::vector<double>{0, 1},
              "straight section span emits normalized endpoints");
    check(s.report["mesh_generated"] == false && s.reference->poles().size() == 5,
          "section sampling preserves complete U reference without claiming whole mesh");
    const Json circle{{"_type", "EllipticArc"},
                      {"arc",
                       {{"centerX", 0},
                        {"centerY", 0},
                        {"centerZ", 0},
                        {"vector0X", 2},
                        {"vector0Y", 0},
                        {"vector0Z", 0},
                        {"vector90X", 0},
                        {"vector90Y", 2},
                        {"vector90Z", 0},
                        {"startRadians", 0},
                        {"sweepRadians", 2 * std::acos(-1.)}}}};
    const auto cylinder = surface(circle);
    TubeBudget cb, ob;
    auto closed = sample_tube_mesh_section(cylinder, true, .01, .2, 100000, cb);
    auto open = sample_tube_mesh_section(cylinder, false, .01, .2, 100000, ob);
    check(closed.success && closed.break_set.empty() && !closed.end_discontinuity &&
              closed.report["end_tangents_parallel"] == true,
          "smooth closed profile suppresses both endpoint discontinuities");
    check(open.success && open.break_set == std::vector<double>{0, 1} && open.end_discontinuity,
          "original profile closure flag controls suppression independently of surface closure");
    check(closed.interval_samples == open.interval_samples && closed.interval_samples.size() == 3,
          "endpoint classification does not alter independent Bezier sampling");
    for (const auto &samples : closed.interval_samples)
        check(samples.size() > 2 && samples.front() == 0 && samples.back() == 1,
              "curved ring spans use normalized adaptive interior samples");
    TubeBudget failure_budget;
    auto failure = sample_tube_mesh_section(patch, true, 0, 0, 100000, failure_budget);
    check(!failure.success && failure.interval_samples.empty() &&
              failure.report["failure_step"] == "adaptive_sampling",
          "native sampling failure is not reported as complete section");
    bool caught = false;
    try {
        TubeBudget limited;
        limited.max_work = 0;
        sample_tube_mesh_section(patch, true, .01, .2, 1000, limited);
    } catch (const std::exception &) {
        caught = true;
    }
    check(caught, "section preparation uses caller cumulative work budget");
    return n;
}
