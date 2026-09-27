#include "native_tube_facet_trim.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
BsplineCurve curve(Json poles = {0, 0, 0, 1, 0, 0}) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", 2},
                                    {"closed", false},
                                    {"poles", poles},
                                    {"knots", nullptr},
                                    {"weights", nullptr}});
}
Json surface(double z = 0) {
    return {{"_type", "BsplineSurface"},
            {"numPolesU", 2},
            {"numPolesV", 2},
            {"orderU", 2},
            {"orderV", 2},
            {"closedU", false},
            {"closedV", false},
            {"knotsU", nullptr},
            {"knotsV", nullptr},
            {"weights", nullptr},
            {"poles", {0, -1, z, 1, -1, z, 0, 1, z, 1, 1, z}},
            {"boundaries", nullptr},
            {"holeOrigin", 0},
            {"numRulesU", 0},
            {"numRulesV", 0}};
}
TubeFacetComposition chain(const std::vector<int> &classes) {
    TubeFacetComposition out;
    out.prepared = true;
    out.report = {{"seams_applied", true}};
    out.working_suffix_path = curve();
    for (int value : classes) {
        out.nodes.push_back({surface(), {}});
        TubeFacetSeamReferences ref;
        ref.classifier = value;
        ref.plane = std::make_shared<TubeFacetSeamStorage<std::array<Point3, 2>>>();
        ref.plane->value = {{{0, 0, 0}, {0, 1, 0}}};
        out.seams.push_back(std::move(ref));
    }
    return out;
}
} // namespace
unsigned native_tube_facet_trim_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto fn) {
        bool failed = false;
        try {
            fn();
        } catch (const std::exception &) {
            failed = true;
        }
        check(failed, "facet trim rejects invalid native state or exhausted resources");
    };
    auto trim = [](const TubeFacetComposition &c, const BsplineCurve &p) {
        TubeBudget b;
        return trim_tube_facet_chain(c, p, curve(), b);
    };
    const auto path = curve();
    const auto empty = trim(chain({}), path);
    check(!empty.success && empty.report["reason"] == "empty_chain", "empty native chain fails");
    const auto noop = trim(chain({2, 0, -2}), path);
    check(noop.success && noop.report["writes"].empty() && noop.boundaries.size() == 3,
          "non-cutting classes without carried segments do not invent rectangular boundaries");
    for (unsigned i = 0; i < 3; ++i)
        check(noop.boundaries[i].allocated.empty() && noop.surfaces[i]["holeOrigin"] == 0,
              "untouched surfaces retain original boundary state and holeOrigin");
    const auto ordinary_input = chain({1, 2});
    const auto ordinary = trim(ordinary_input, path);
    const std::vector<Point2> bottom{{0, 0},   {1, 0},    {1, .5}, {.75, .5},
                                     {.5, .5}, {.25, .5}, {0, .5}, {0, 0}};
    const std::vector<Point2> top{{0, .5}, {.25, .5}, {.5, .5}, {.75, .5},
                                  {1, .5}, {1, 1},    {0, 1},   {0, .5}};
    check(
        ordinary.success && ordinary.report["failed_samples"] == 0 &&
            ordinary.boundaries[0].allocated == std::vector<std::vector<Point2>>{bottom} &&
            ordinary.boundaries[1].allocated == std::vector<std::vector<Point2>>{top},
        "ordinary cut and carried-only successor match independent analytic half-strip boundaries");
    check(ordinary.boundaries[0].active_count == 1 && ordinary.boundaries[1].active_count == 1 &&
              ordinary.surfaces[0]["holeOrigin"] == 1 && ordinary.surfaces[1]["holeOrigin"] == 1,
          "writes explicitly activate first raw UV boundary and set native holeOrigin");
    check(ordinary.surfaces[0]["boundaries"].is_null() &&
              ordinary.report["boundary_storage"] == "separate_native_uv_polylines",
          "raw native boundary storage is not mislabeled as a BGFB curve tree");
    check(ordinary_input.nodes[0].surface == surface() &&
              ordinary_input.nodes[1].surface == surface(),
          "finalization leaves source geometry tables untouched");
    const auto negative = trim(chain({-1, 2}), path);
    check(negative.boundaries[0].allocated == ordinary.boundaries[0].allocated,
          "negative class one uses the same sampling route");
    const auto wrapped = trim(chain({1, 1}), path);
    check(wrapped.success && wrapped.report["visits"][0]["action"] == "head_boundary_deferred" &&
              wrapped.report["writes"][0]["node"] == 1 && wrapped.report["writes"][1]["node"] == 0,
          "head cut is deferred until tail wrap, with tail written before head");
    check(wrapped.boundaries[0].active_count == 1 && wrapped.boundaries[1].active_count == 1 &&
              wrapped.boundaries[0].allocated == wrapped.boundaries[1].allocated,
          "symmetric wrap consumes both saved head and carried tail segments");
    const auto single = trim(chain({1}), path);
    check(single.success && single.report["writes"].size() == 2 &&
              single.boundaries[0].allocated == std::vector<std::vector<Point2>>{bottom, top} &&
              single.boundaries[0].active_count == 1,
          "one-node wrap writes twice but native numBounds selects the first allocation only");
    auto partial_input = chain({1, 1, 2});
    partial_input.nodes[2].surface = surface(1);
    const auto partial = trim(partial_input, path);
    check(partial.success && partial.report["failed_samples"] == 1 &&
              partial.report["all_samples_succeeded"] == false &&
              partial.boundaries[0].active_count == 1 && partial.boundaries[1].active_count == 0 &&
              partial.boundaries[2].active_count == 0,
          "failed later seam preserves earlier write and clears carried segments without failing "
          "chain");
    auto missed_wrap = chain({1, 1});
    missed_wrap.seams.back().plane->value[0][1] = 3;
    const auto missed = trim(missed_wrap, path);
    check(missed.success && missed.report["failed_samples"] == 1 && missed.report["writes"].empty(),
          "failed wrap leaves deferred head unwritten instead of inventing a fallback boundary");
    const auto spatial_path = curve({0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1});
    auto separate = chain({1, 1});
    separate.nodes[1].surface = surface(1);
    const auto nonplanar = trim(separate, spatial_path), planar = trim(separate, path);
    check(nonplanar.report["path_planarity"]["planar"] == false &&
              nonplanar.report["visits"][0]["require_same_point"] == true &&
              nonplanar.report["visits"][1]["require_same_point"] == false,
          "path planarity controls only wrap coincidence, not ordinary adjacent seam checks");
    check(
        nonplanar.success && nonplanar.report["failed_samples"] == 1 &&
            nonplanar.report["writes"].size() == 2 && planar.report["failed_samples"] == 2 &&
            planar.report["writes"].empty(),
        "nonplanar wrap accepts separate plane intersections while planar wrap requests fallback");
    TubeBudget final_budget;
    const auto finalized = finalize_tube_facet_boundaries(ordinary_input, curve(), final_budget);
    check(finalized.success &&
              finalized.boundaries[0].allocated == ordinary.boundaries[0].allocated &&
              finalized.report["path_preparation"]["method"] == "suffix_copy",
          "f95b0 combined-path preparation is connected to complete f81c0 boundary processing");
    auto no_seams = chain({1});
    no_seams.report["seams_applied"] = false;
    rejects([&] { trim(no_seams, path); });
    auto dead = chain({1});
    dead.seams[0].plane->alive = false;
    rejects([&] { trim(dead, path); });
    auto absent = chain({1});
    absent.seams[0].plane.reset();
    rejects([&] { trim(absent, path); });
    TubeBudget work;
    work.max_work = 0;
    rejects([&] { trim_tube_facet_chain(ordinary_input, path, curve(), work); });
    TubeBudget points;
    points.max_control_points = 15;
    rejects([&] { trim_tube_facet_chain(ordinary_input, path, curve(), points); });
    auto concurrent = std::async(std::launch::async, [&] { return trim(ordinary_input, path); });
    const auto now = trim(ordinary_input, path), other = concurrent.get();
    check(now.report == other.report && now.surfaces == other.surfaces &&
              now.boundaries[0].allocated == other.boundaries[0].allocated,
          "independent finalization of shared immutable source chain is deterministic");
    return n;
}
