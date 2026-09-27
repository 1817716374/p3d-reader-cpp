#include "native_tube_mesh_group.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_tube_mesh_group_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *why) {
        ++checks;
        require(value, why);
    };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "mesh group rejects invalid preparation or budget");
    };
    auto surface = [](double z) {
        return Json{{"_type", "BsplineSurface"},
                    {"orderU", 2},
                    {"orderV", 2},
                    {"numPolesU", 3},
                    {"numPolesV", 2},
                    {"closedU", false},
                    {"closedV", false},
                    {"poles", {0, 0, z, 1, 0, z, 1, 1, z, 0, 0, z + 1, 1, 0, z + 1, 1, 1, z + 1}},
                    {"weights", nullptr},
                    {"knotsU", {0, 0, .5, 1, 1}},
                    {"knotsV", nullptr},
                    {"numRulesU", 0},
                    {"numRulesV", 0},
                    {"holeOrigin", 0},
                    {"boundaries", nullptr}};
    };
    SweptBodyPatchGroup group;
    group.patches = {{surface(0), {}}, {surface(1), {}}};
    TubeBudget b;
    auto prepared = prepare_tube_mesh_grid(group, false, .01, .1, 10000, b);
    check(prepared.success && prepared.section.interval_samples.size() == 2,
          "two native knot strips in each of two path patches");
    TubeMeshGroupOptions options;
    options.collect_cap_boundaries = true;
    auto result = emit_tube_mesh_group(prepared, options, b);
    check(result.traversal_completed && result.strips_complete && result.strips.size() == 4,
          "entire patch-major group emits four strips");
    check(result.start_boundary == std::vector<Point3>{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}} &&
              result.end_boundary == std::vector<Point3>{{0, 0, 2}, {1, 0, 2}, {1, 1, 2}},
          "native cap arguments remove only adjacent strip endpoint duplicates");
    const std::vector<std::vector<Point3>> expected{{{0, 0, 0}, {1, 0, 0}, {0, 0, 1}, {1, 0, 1}},
                                                    {{1, 0, 0}, {1, 1, 0}, {1, 0, 1}, {1, 1, 1}},
                                                    {{0, 0, 1}, {1, 0, 1}, {0, 0, 2}, {1, 0, 2}},
                                                    {{1, 0, 1}, {1, 1, 1}, {1, 0, 2}, {1, 1, 2}}};
    for (std::size_t i = 0; i < 4; ++i) {
        const auto &m = result.strips[i];
        check(m.points == expected[i], "group preserves original per-strip coordinate pools");
        check(m.normal_indices == std::vector<std::int32_t>{1, 2, 3, 0, 2, 4, 3, 0} &&
                  m.parameter_indices == m.normal_indices,
              "independent local attribute indices copied before hidden-edge signs");
        const double u = (i % 2) * .5, v = (i / 2) * .5;
        check(m.parameters ==
                  std::vector<Point2>{{u, v}, {u + .5, v}, {u, v + .5}, {u + .5, v + .5}},
              "original U intervals and patch-major V fractions preserved");
    }
    // Exact input discontinuities drive visibility, independently of seam flags.
    auto flags = prepared;
    flags.section.discontinuity_intervals = {1};
    flags.section.end_discontinuity = false;
    auto hidden = emit_tube_mesh_group(flags, options, b);
    check(hidden.strips[0].point_indices == std::vector<std::int32_t>{-1, -2, -3, 0, -2, -4, -3, 0},
          "smooth first interval has no visible selected column");
    check(hidden.strips[1].point_indices == std::vector<std::int32_t>{-1, -2, 3, 0, -2, -4, -3, 0},
          "second strip alone exposes its original first-column edge");
    flags.section.end_discontinuity = true;
    auto last = emit_tube_mesh_group(flags, options, b);
    check(last.strips[1].point_indices == std::vector<std::int32_t>{-1, -2, 3, 0, 2, -4, -3, 0},
          "end discontinuity applies only to the final strip");
    check(last.strips[0].point_indices == hidden.strips[0].point_indices &&
              last.strips[1].normal_indices == hidden.strips[1].normal_indices,
          "last-column visibility does not affect prior strips or attribute indices");
    auto no_attributes = options;
    no_attributes.normals = no_attributes.parameters = false;
    no_attributes.collect_cap_boundaries = false;
    auto bare = emit_tube_mesh_group(prepared, no_attributes, b);
    check(bare.strips_complete && bare.start_boundary.empty() && bare.end_boundary.empty(),
          "resolved no-cap route does not collect cap arguments");
    for (const auto &m : bare.strips)
        check(m.normals.empty() && m.parameters.empty() && m.normal_indices.empty() &&
                  m.parameter_indices.empty(),
              "optional channels remain absent throughout group");
    {
        auto single = prepared;
        single.patches.resize(1);
        auto r = emit_tube_mesh_group(single, options, b);
        check(r.strips_complete && r.start_boundary == result.start_boundary &&
                  r.end_boundary == std::vector<Point3>{{0, 0, 1}, {1, 0, 1}, {1, 1, 1}},
              "single patch removes predecessor endpoints from both cap boundaries");
    }
    // One patch is trimmed to V=.8; the next is rectangular. Shared coordinates
    // come from original predecessor slots, not the next patch's V=0 evaluation.
    group.patches[0].boundary_points = {
        {{0, 0}, {.5, 0}, {1, 0}, {1, .8}, {.5, .8}, {0, .8}, {0, 0}}};
    auto mixed = prepare_tube_mesh_grid(group, false, .01, .1, 10000, b);
    auto joined = emit_tube_mesh_group(mixed, options, b);
    check(joined.strips_complete && joined.strips.size() == 4 &&
              joined.report.at("strip_results")[0].at("trimmed") == true &&
              joined.report.at("strip_results")[2].at("trimmed") == false,
          "trimmed and rectangular routes share one native group state");
    check(joined.strips[2].points[0] == Point3{0, 0, .8} &&
              joined.strips[2].points[1] == Point3{1, 0, .8},
          "later rectangular strip consumes trimmed predecessor coordinates");
    check(joined.end_boundary == result.end_boundary &&
              joined.start_boundary == result.start_boundary,
          "mixed routes retain correct start and end cap argument order");
    // A later native trim-planning failure retains earlier strip diagnostics,
    // but cannot be mistaken for a completed group.
    auto failed = mixed;
    failed.patches.push_back(mixed.patches.front());
    failed.patches.back().path.parameters = {0, .5};
    auto partial = emit_tube_mesh_group(failed, no_attributes, b);
    check(!partial.traversal_completed && !partial.strips_complete && partial.strips.size() == 4 &&
              partial.report.at("failure").at("patch") == 2,
          "late native failure preserves prior strips with explicit incomplete state");
    for (unsigned mask = 0; mask < 4; ++mask) {
        auto closed = no_attributes;
        closed.profile_closed = mask & 1;
        closed.path_closed = mask & 2;
        auto r = emit_tube_mesh_group(prepared, closed, b);
        check(r.traversal_completed && r.strips.size() == 4,
              "explicit native seam flags traverse all patch-strip combinations");
    }
    auto bad = prepared;
    bad.section.discontinuity_intervals = {2};
    rejects([&] { emit_tube_mesh_group(bad, options, b); });
    bad = prepared;
    bad.success = false;
    rejects([&] { emit_tube_mesh_group(bad, options, b); });
    bad = prepared;
    bad.patches.back().preparation.strips.pop_back();
    rejects([&] { emit_tube_mesh_group(bad, options, b); });
    TubeBudget measured;
    emit_tube_mesh_group(prepared, options, measured);
    for (auto limit : {std::size_t(0), measured.work / 2, measured.work - 1}) {
        TubeBudget limited;
        limited.max_work = limit;
        rejects([&] { emit_tube_mesh_group(prepared, options, limited); });
    }
    {
        TubeBudget limited;
        limited.max_control_points = 120;
        rejects([&] { emit_tube_mesh_group(prepared, options, limited); });
    }
    auto task = [=] {
        TubeBudget local;
        auto r = emit_tube_mesh_group(prepared, options, local);
        Json j = {{"report", r.report}, {"start", r.start_boundary}, {"end", r.end_boundary}};
        for (const auto &m : r.strips)
            j["meshes"].push_back({m.points, m.point_indices, m.parameters});
        return j;
    };
    auto reference = task();
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(std::async(std::launch::async, task));
    for (auto &job : jobs)
        check(job.get() == reference, "group calls are independent and deterministic");
    return checks;
}
