#include "native_tube_mesh_cap.hpp"
#include "native_polyface_prepare.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_tube_mesh_cap_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *why) {
        ++checks;
        require(value, why);
    };
    const std::vector<Point3> square{{10, 20, 3}, {14, 20, 3}, {14, 22, 3}, {10, 22, 3}};
    const std::vector<Point3> hole{{11, 20.5, 3}, {12, 20.5, 3}, {12, 21, 3}, {11, 21, 3}};
    auto run = [&](std::vector<std::vector<Point3>> rings, bool reverse = false,
                   NativeTubeMeshCapOptions options = {}) {
        TubeBudget budget;
        return emit_native_tube_mesh_cap(rings, reverse, options, budget);
    };
    auto area = [](const NativePolyfaceMesh &mesh) {
        double area = 0;
        TubeBudget budget;
        auto visit = visit_native_polyface(mesh, budget, false, 0);
        require(visit.complete, "cap point visitor complete");
        for (const auto &face : visit.facets) {
            require(face.points.size() == 3, "cap emits triangles");
            const auto a = face.points[0], b = face.points[1], c = face.points[2];
            area += ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) * .5;
        }
        return area;
    };
    auto validate = [&](const NativeTubeMeshCap &cap, bool normals = true, bool params = true) {
        const auto &m = cap.mesh;
        check(cap.native_succeeded && cap.triangulation_succeeded && cap.complete,
              "complete native cap");
        check(m.mesh_style == 1 && m.data.num_per_face == 0 && m.data.two_sided,
              "fresh indexed builder layout");
        const auto &indices = m.data.indices.indices;
        check(indices[point_channel].size() % 4 == 0 && !indices[point_channel].empty(),
              "terminated triangles");
        for (std::size_t c = 0; c < polyface_channel_count; ++c) {
            if (c == color_channel || (c == normal_channel && !normals) ||
                ((c == parameter_channel || c == face_channel) && !params)) {
                check(indices[c].empty(), "unrequested index channel remains empty");
                continue;
            }
            check(indices[c].size() == indices[point_channel].size(),
                  "active channel matches faces");
            const auto count = c == point_channel       ? m.data.coordinates.points.size()
                               : c == normal_channel    ? m.data.coordinates.normals.size()
                               : c == parameter_channel ? m.data.coordinates.parameters.size()
                                                        : m.data.face_data.size();
            for (std::size_t i = 0; i < indices[c].size(); ++i) {
                const auto v = static_cast<std::int64_t>(indices[c][i]);
                check(i % 4 == 3 ? v == 0
                                 : (v != 0 && static_cast<std::size_t>(v < 0 ? -v : v) <= count),
                      "valid native pooled references");
                if (c != point_channel)
                    check(v >= 0, "edge visibility sign only belongs to point index");
            }
        }
        check(m.pool_active[native_normal_pool] == normals &&
                  m.pool_active[native_parameter_pool] == params,
              "requested pool flags retained");
        check(!m.pool_active[native_face_data_pool] && !m.edge_chain_active,
              "end face does not invent face-data or edge-chain activity");
        if (params)
            check(m.data.face_data.size() == 1, "cap triangles share one native face record");
        else
            check(m.data.face_data.empty(), "no face record without requested parameters");
    };
    auto cap = run({square});
    validate(cap);
    check(cap.report["polygon_route"] == "projected_triangulation",
          "square uses original triangulator");
    check(std::abs(area(cap.mesh) - 8) < 1e-12, "end cap area and winding");
    check(cap.mesh.data.coordinates.points.size() == 4 &&
              cap.mesh.data.coordinates.normals.size() == 1,
          "native point and normal pools");
    check(cap.mesh.data.coordinates.normals[0] == Point3{0, 0, 1}, "positive end normal");
    check(cap.mesh.data.face_data[0].parameter_distance_range ==
              std::array<Point2, 2>{{{0, 0}, {4, 2}}},
          "unscaled face distance range");
    auto start = run({square}, true);
    validate(start);
    check(std::abs(area(start.mesh) + 8) < 1e-12 &&
              start.mesh.data.coordinates.normals[0] == Point3{0, 0, -1},
          "start rings reversed before projection and normal generation");
    auto multi = run({square, hole});
    validate(multi);
    check(std::abs(area(multi.mesh) - 7.5) < 1e-12, "hole is not capped over");
    check(multi.polygon_points.size() == 12 && multi.mesh.data.coordinates.points.size() == 8,
          "source closure and marker slots reach native point map");
    auto has_hidden = std::any_of(multi.mesh.data.indices.indices[point_channel].begin(),
                                  multi.mesh.data.indices.indices[point_channel].end(),
                                  [](auto v) { return v < 0; });
    check(has_hidden, "internal edges remain hidden");
    auto tri = run({{{2, 3, 4}, {6, 3, 4}, {2, 5, 4}}});
    validate(tri);
    check(tri.report["polygon_route"] == "simple_face" &&
              tri.polygon_indices == std::vector<std::int32_t>{1, 2, 3, 0},
          "three-point cap bypasses VU with original order");
    check(tri.polygon_points == tri.input.points, "simple path avoids projection round trip");
    for (bool normals : {false, true})
        for (bool params : {false, true}) {
            NativeTubeMeshCapOptions options;
            options.normals_required = normals;
            options.parameters_required = params;
            auto requested = run({square}, false, options);
            validate(requested, normals, params);
            check(std::abs(area(requested.mesh) - 8) < 1e-12,
                  "attribute requests preserve geometry");
        }
    for (unsigned mode : {0u, 1u, 2u, 3u}) {
        NativeTubeMeshCapOptions options;
        options.parameter_mode = mode;
        auto mapped = run({square}, false, options);
        validate(mapped);
        const auto &range = mapped.mesh.data.face_data[0].parameter_range;
        check(range[1][0] == (mode >= 2 ? 4 : 1) && range[1][1] == (mode >= 2   ? 2
                                                                    : mode == 1 ? .5
                                                                                : 1),
              "UV mode reaches emitted face record");
    }
    auto tilted = run({{{10, 20, 30}, {14, 20, 30}, {14, 20, 32}, {10, 20, 32}}});
    validate(tilted);
    check(tilted.mesh.data.coordinates.normals[0] == Point3{0, -1, 0},
          "three-dimensional cap normal");
    auto bow = run({{{0, 0, 0}, {4, 4, 0}, {0, 4, 0}, {4, 0, 0}}});
    // Native frame construction may reject the self-crossing region before VU.
    if (bow.complete) {
        validate(bow);
        check(bow.polygon_points.size() > bow.input.points.size(),
              "self intersection point reaches builder");
    } else
        check(!bow.input.preparation_succeeded, "self-crossing frame failure is preserved");
    auto empty = run({});
    check(!empty.native_succeeded && !empty.complete, "no rings means no published cap handle");
    auto degenerate = run({{}});
    check(degenerate.native_succeeded && !degenerate.triangulation_succeeded &&
              !degenerate.complete && degenerate.mesh.data.coordinates.points.empty(),
          "native empty mesh success is not complete geometry");
    auto duplicate = run({{{1, 1, 1}, {1, 1, 1}, {1, 1, 1}}});
    check(duplicate.native_succeeded && !duplicate.complete, "collapsed ring is not repaired");
    auto upper = square;
    for (auto &p : upper)
        p[2] += 5;
    TubeBudget budget;
    auto pair = emit_native_tube_mesh_cap_pair({square}, {upper}, true, {}, budget);
    check(pair.published && pair.complete && pair.start && pair.end, "cap pair publishes together");
    check(area(pair.start->mesh) < 0 && area(pair.end->mesh) > 0,
          "pair controls start and end winding");
    auto assembly =
        assemble_native_prepared_polyfaces({pair.start->mesh, pair.end->mesh}, {}, {}, budget);
    check(assembly.complete && assembly.native_succeeded && assembly.sources.size() == 2,
          "cap pair enters original final-builder preparation and assembly");
    check(assembly.assembled.face_data.size() == 2 &&
              assembly.assembled.coordinates.points.size() == 8 &&
              assembly.assembled.coordinates.normals.size() == 2,
          "assembled cap geometry and independent face records survive");
    auto unequal = emit_native_tube_mesh_cap_pair({square}, {upper, hole}, true, {}, budget);
    check(!unequal.published && !unequal.start && !unequal.end,
          "ring mismatch skips both cap attempts");
    auto disabled = emit_native_tube_mesh_cap_pair({square}, {upper}, false, {}, budget);
    check(!disabled.start && !disabled.end, "eligibility gate prevents attempts");
    auto none = emit_native_tube_mesh_cap_pair({}, {}, true, {}, budget);
    check(none.start && !none.start->native_succeeded && !none.end && !none.published,
          "failed start prevents end attempt");
    auto native_empty = emit_native_tube_mesh_cap_pair({{}}, {{}}, true, {}, budget);
    check(native_empty.published && !native_empty.complete,
          "two empty native handles may publish without completeness");
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "unsafe cap inputs rejected");
    };
    rejects([&] { run({{{NAN, 0, 0}, {1, 0, 0}, {0, 1, 0}}}); });
    rejects([&] {
        TubeBudget small;
        small.max_control_points = 3;
        emit_native_tube_mesh_cap({square}, false, {}, small);
    });
    TubeBudget measured;
    emit_native_tube_mesh_cap({square, hole}, false, {}, measured);
    for (auto limit : {std::size_t(0), measured.work / 2, measured.work - 1})
        rejects([&] {
            TubeBudget small;
            small.max_work = limit;
            emit_native_tube_mesh_cap({square, hole}, false, {}, small);
        });
    auto future = std::async(std::launch::async, [&] { return run({square, hole}); });
    auto parallel = future.get();
    check(parallel.mesh.data.indices.indices == multi.mesh.data.indices.indices &&
              parallel.report == multi.report,
          "isolated deterministic cap calls");
    check(square.front() == Point3{10, 20, 3}, "caller rings unchanged");
    return checks;
}
