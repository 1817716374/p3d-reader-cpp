#include "native_tube_mesh_trim_output.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_tube_mesh_trim_output_tests() {
    unsigned checks = 0;
    auto check = [&](bool x, const char *why) {
        ++checks;
        require(x, why);
    };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "trim output rejects invalid correspondence or exhausted budget");
    };
    Json surface{{"_type", "BsplineSurface"},
                 {"orderU", 2},
                 {"orderV", 2},
                 {"numPolesU", 2},
                 {"numPolesV", 2},
                 {"closedU", false},
                 {"closedV", false},
                 {"poles", {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0}},
                 {"weights", nullptr},
                 {"knotsU", nullptr},
                 {"knotsV", nullptr},
                 {"numRulesU", 0},
                 {"numRulesV", 0},
                 {"holeOrigin", 0},
                 {"boundaries", nullptr}};
    SweptBodyPatchGroup group;
    group.patches = {
        {surface, {{{0, .1}, {.5, .2}, {1, .1}, {1, .9}, {.5, .8}, {0, .9}, {0, .1}}}}};
    TubeBudget b;
    auto prepared = prepare_tube_mesh_grid(group, false, .01, .1, 10000, b);
    auto nodes = evaluate_tube_mesh_trim_vertices(prepared.patches[0], prepared.section, 0, b);
    auto state = make_tube_mesh_edge_state(1, 1, false, false, b);
    auto connected =
        connect_tube_mesh_trim_vertices(nodes, prepared.patches[0].path.parameters, state, {}, b);
    TubeMeshVisibilityLayout layout;
    layout.trimmed = true;
    layout.u_count = nodes.plan.columns.size();
    layout.v_count = prepared.patches[0].path.parameters.size();
    layout.coordinate_count = connected.points.size();
    layout.first_column_count = int(nodes.plan.columns.front().count);
    layout.last_column_count = int(nodes.plan.columns.back().count);
    auto expected = emit_tube_mesh_trimmed_strip(connected, {}, layout, b);
    check(expected.complete && expected.report.at("triangulation").at("native_succeeded") == true,
          "trim bounds, evaluated vertices, native correspondence and facet output connect end to "
          "end");
    check(expected.mesh.points == connected.points && expected.mesh.normals == connected.normals &&
              expected.mesh.parameters == connected.parameters,
          "emission preserves the complete source attribute pools");
    auto area = [&](const TubeMeshRegularMesh &m) {
        double total = 0;
        check(m.point_indices.size() % 4 == 0, "trim output contains zero-terminated triangles");
        for (std::size_t i = 0; i < m.point_indices.size(); i += 4) {
            std::array<Point3, 3> p;
            check(m.point_indices[i + 3] == 0, "triangle delimiter retained");
            for (std::size_t k = 0; k < 3; ++k) {
                auto n = std::abs(m.point_indices[i + k]);
                check(n > 0 && std::size_t(n) <= m.points.size(),
                      "trim output points refer to original array");
                p[k] = m.points[n - 1];
            }
            total += ((p[1][0] - p[0][0]) * (p[2][1] - p[0][1]) -
                      (p[1][1] - p[0][1]) * (p[2][0] - p[0][0])) *
                     .5;
        }
        return total;
    };
    check(std::abs(area(expected.mesh) - .8) < 1e-10,
          "two native U samples form the rectangle between the sampled endpoint bounds");
    {
        // Exercise varying row counts and large boundary polygons with an
        // explicit prepared sample table that includes the boundary bend.
        auto sampled = prepared;
        sampled.section.interval_samples[0] = {0, .5, 1};
        sampled.patches[0].path.parameters = {0,  .1,  .15, .17, .19, .3, .5,
                                              .7, .81, .83, .85, .9,  1};
        auto detailed_nodes =
            evaluate_tube_mesh_trim_vertices(sampled.patches[0], sampled.section, 0, b);
        auto detailed_state = make_tube_mesh_edge_state(1, 1, false, false, b);
        auto detailed = connect_tube_mesh_trim_vertices(
            detailed_nodes, sampled.patches[0].path.parameters, detailed_state, {}, b);
        auto l = layout;
        l.u_count = 3;
        l.v_count = sampled.patches[0].path.parameters.size();
        l.coordinate_count = detailed.points.size();
        l.first_column_count = int(detailed_nodes.plan.columns.front().count);
        l.last_column_count = int(detailed_nodes.plan.columns.back().count);
        bool large = false;
        for (const auto &f : detailed.facets.facets)
            large |= f.indices.size() > 4;
        check(large, "varying row counts produce original large boundary facet arguments");
        auto r = emit_tube_mesh_trimmed_strip(detailed, {}, l, b);
        check(r.complete && std::abs(area(r.mesh) - .7) < 1e-10,
              "large boundary polygons preserve the independently integrated sampled trim area");
    }
    for (auto index : expected.mesh.point_indices)
        check(index <= 0, "no visible columns hides every point-index edge");
    for (unsigned flags = 0; flags < 4; ++flags) {
        auto l = layout;
        l.first_column_visible = flags & 1;
        l.last_column_visible = flags & 2;
        const auto r = emit_tube_mesh_trimmed_strip(connected, {}, l, b);
        check(r.complete && r.mesh.normal_indices == expected.mesh.normal_indices &&
                  r.mesh.parameter_indices == expected.mesh.parameter_indices,
              "point visibility changes do not rewrite prior normal and UV signs");
        for (std::size_t i = 0; i < r.mesh.point_indices.size(); ++i)
            check(std::abs(r.mesh.point_indices[i]) == std::abs(expected.mesh.point_indices[i]),
                  "visibility never changes attribute correspondence");
    }
    {
        const auto r =
            emit_tube_mesh_trimmed_strip(connected, {false, false, false, false}, layout, b);
        check(r.complete && r.mesh.normals.empty() && r.mesh.parameters.empty() &&
                  r.mesh.normal_indices.empty() && r.mesh.parameter_indices.empty(),
              "explicit optional channels remain absent");
        auto missing = connected;
        missing.normals.clear();
        const auto partial = emit_tube_mesh_trimmed_strip(missing, {}, layout, b);
        check(!partial.complete &&
                  partial.report.at("invalid_normal_indices").get<unsigned>() > 0 &&
                  partial.mesh.point_indices == expected.mesh.point_indices,
              "missing attributes remain an explicit incomplete result");
    }
    {
        auto invalid = connected;
        invalid.facets.column_correspondence_valid = false;
        rejects([&] { emit_tube_mesh_trimmed_strip(invalid, {}, layout, b); });
        invalid = connected;
        invalid.facets.facets[0].indices[0] = INT32_MAX;
        rejects([&] { emit_tube_mesh_trimmed_strip(invalid, {}, layout, b); });
        auto l = layout;
        l.coordinate_count++;
        rejects([&] { emit_tube_mesh_trimmed_strip(connected, {}, l, b); });
    }
    {
        TubeMeshTrimConnected partial;
        partial.points = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 0},
                          {4, 4, 0}, {0, 4, 0}, {4, 0, 0}, {0, 0, 0}};
        partial.normals.assign(9, {0, 0, 1});
        partial.parameters.assign(9, {0, 0});
        partial.facets.facets = {
            {0, TubeMeshTrimFacetKind::common_rows, {1, 2, 3, 4}, true, true},
            {1, TubeMeshTrimFacetKind::lower_boundary, {5, 6, 7, 8, 9}, true, true}};
        TubeMeshVisibilityLayout l{2, 2, 9, 4, 5, true, false, false};
        const auto r = emit_tube_mesh_trimmed_strip(partial, {}, l, b);
        check(!r.complete && r.report.at("triangulation").at("failed_facets") == 1 &&
                  r.mesh.point_indices.size() == 8,
              "native strip keeps successful faces while reporting skipped triangulation failures");
        check(r.mesh.normal_indices.size() == 8 && r.mesh.parameter_indices.size() == 8,
              "copied attributes follow the same retained partial geometry");
    }
    TubeBudget measured;
    emit_tube_mesh_trimmed_strip(connected, {}, layout, measured);
    for (auto limit : {std::size_t(0), measured.work / 2, measured.work - 1}) {
        TubeBudget limited;
        limited.max_work = limit;
        rejects([&] { emit_tube_mesh_trimmed_strip(connected, {}, layout, limited); });
        check(connected.points == expected.mesh.points,
              "failed strip emission does not alter connected source data");
    }
    auto task = [=] {
        TubeBudget local;
        auto r = emit_tube_mesh_trimmed_strip(connected, {}, layout, local);
        return Json{{"report", r.report},
                    {"indices", r.mesh.point_indices},
                    {"normals", r.mesh.normal_indices}};
    };
    auto reference = task();
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(std::async(std::launch::async, task));
    for (auto &job : jobs)
        check(job.get() == reference,
              "trim strip emission is deterministic across concurrent calls");
    return checks;
}
