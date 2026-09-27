#include "native_polyface_finalize.hpp"
#include "native_polyface_copy.hpp"
#include "native_polyface_face_data.hpp"
#include "native_tube_mesh_cap.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_polyface_finalize_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *why) {
        ++checks;
        require(value, why);
    };
    auto run = [](std::vector<std::int32_t> input) {
        NativePolyfaceIndexState source;
        source.indices[point_channel] = std::move(input);
        TubeBudget budget;
        return resolve_native_polyface_edge_visibility(source, budget);
    };
    // Four occurrences of the same undirected edge, with unrelated other edges.
    // Enumerate every sign/order; do not replace the one-pass rule with global OR.
    for (unsigned mask = 0; mask < 16; ++mask) {
        std::vector<std::int32_t> input, expected;
        int first_visible = -1;
        for (unsigned i = 0; i < 4; ++i) {
            bool visible = mask & (1u << i);
            if (visible && first_visible < 0)
                first_visible = static_cast<int>(i);
            const std::int32_t a = i % 2 ? 2 : 1, b = i % 2 ? 1 : 2;
            input.insert(input.end(), {visible ? a : -a, b, -static_cast<std::int32_t>(10 + i), 0});
        }
        expected = input;
        if (first_visible >= 0) {
            if (first_visible > 0)
                expected[0] = std::abs(expected[0]);
            for (unsigned i = static_cast<unsigned>(first_visible); i < 4; ++i)
                expected[i * 4] = std::abs(expected[i * 4]);
        }
        auto output = run(input);
        check(output.output.indices[point_channel] == expected,
              "native ordered hidden-edge promotion");
        for (unsigned i = 0; i < 4; ++i)
            check(output.output.indices[point_channel][i * 4 + 2] == input[i * 4 + 2],
                  "unrelated hidden closing edge unchanged");
    }
    auto delayed = run({-1, 2, -3, 0, -2, 1, -4, 0, 1, 2, -5, 0});
    check(delayed.output.indices[point_channel][0] == 1 &&
              delayed.output.indices[point_channel][4] == -2,
          "positive third occurrence does not retrospectively fix every hidden duplicate");
    check(delayed.report["promoted_previous"] == 1 && delayed.report["promoted_current"] == 0,
          "backward and current promotions counted separately");
    check(run({-1, 0, 1, 0}).output.indices[point_channel] == std::vector<std::int32_t>{1, 0, 1, 0},
          "single-corner face uses self edge");
    check(run({1, 2, 0, -1, 2}).output.indices[point_channel] ==
              std::vector<std::int32_t>{1, 2, 0, 1, 2},
          "unterminated tail still supplies previous edge endpoint");
    check(run({1, 2, 0, 1, -2}).output.indices[point_channel].back() == -2,
          "last token itself is never processed");
    check(run({0, 0, -1, 2, 0, 1, 2, 0}).output.indices[point_channel][2] == 1,
          "leading consecutive terminators reset first-corner state");
    for (auto input : {std::vector<std::int32_t>{}, std::vector<std::int32_t>{INT32_MIN},
                       std::vector<std::int32_t>{0, INT32_MIN}})
        check(run(input).output.indices[point_channel] == input,
              "unread final magnitude is not inspected");
    NativePolyfaceIndexState channels;
    channels.active = {false, true, true, true, true};
    channels.indices[point_channel] = {-1, 2, 0, 1, 2, 0};
    for (std::size_t c = 1; c < polyface_channel_count; ++c)
        channels.indices[c] = {-7, 9, 0};
    TubeBudget budget;
    auto resolved = resolve_native_polyface_edge_visibility(channels, budget);
    check(resolved.output.active == channels.active, "visibility does not activate channels");
    for (std::size_t c = 1; c < polyface_channel_count; ++c)
        check(resolved.output.indices[c] == channels.indices[c],
              "normal UV color and face indices untouched");
    auto mesh = make_native_polyface_mesh();
    mesh.data.coordinates.points = {{0, 0, 0},     {1, 0, 0},         {0, 1, 0},
                                    {1e-10, 0, 0}, {1 + 1e-10, 0, 0}, {0, -1, 0}};
    mesh.data.indices.indices[point_channel] = {-1, 2, 3, 0, 4, 5, 6, 0};
    mesh.data.coordinates.normals = {{0, 0, 1}, {0, 0, 1}};
    mesh.data.indices.indices[normal_channel] = {1, 1, 1, 0, 2, 2, 2, 0};
    mesh.data.indices.active[normal_channel] = true;
    mesh.pool_active[native_normal_pool] = true;
    mesh.data.coordinates.parameters = {{0, 0}, {0, 0}, {1, 0}, {0, 1}};
    mesh.data.indices.indices[parameter_channel] = {1, 3, 4, 0, 2, 3, 4, 0};
    mesh.data.indices.active[parameter_channel] = true;
    mesh.pool_active[native_parameter_pool] = true;
    mesh.texture_id = 123;
    mesh.illumination_name = u"source";
    mesh.face_material_ids = {33, 44};
    mesh.face_uv_points = {{8, 9}};
    mesh.face_smooth_groups = {7};
    mesh.double_colors = {{.1, .2, .3}};
    mesh.float_colors = {{{.4f, .5f, .6f}}};
    mesh.integer_colors = {0x12345678};
    mesh.color_table = {9};
    mesh.data.face_data = {native_null_face_data()};
    mesh.data.face_data[0].source_index = 999;
    mesh.data.face_data[0].point_range = {{{-9, -8, -7}, {9, 8, 7}}};
    mesh.data.indices.indices[face_channel] = {1, 1, 1, 0, 1, 1, 1, 0};
    mesh.data.edge_chains = {{2, {7, 8}, {0, 3, 5}}};
    mesh.edge_chain_active = true;
    mesh.edge_chain_rows = 5;
    mesh.pool_rows[native_point_pool] = 17;
    mesh.data.coordinates.parameter_scope = 42;
    auto finalized = finalize_native_polyface_mesh(mesh, budget);
    const auto &out = finalized.output;
    check(finalized.combination_applied && out.data.coordinates.points.size() == 4,
          "native finalization combines near point pools");
    check(out.data.indices.indices[point_channel][0] > 0,
          "newly matched edge key is processed after combination");
    check(out.data.coordinates.normals.size() == 1 && out.data.coordinates.parameters.size() == 3,
          "independent active normal and UV pools combined");
    for (auto c : {point_channel, normal_channel, parameter_channel})
        for (std::size_t i = 0; i < mesh.data.indices.indices[c].size(); ++i) {
            const auto old = mesh.data.indices.indices[c][i];
            if (!old) {
                check(out.data.indices.indices[c][i] == 0, "terminators retained");
                continue;
            }
            const auto &map = c == point_channel    ? finalized.point_map
                              : c == normal_channel ? finalized.normal_map
                                                    : finalized.parameter_map;
            check(std::size_t(std::abs(out.data.indices.indices[c][i])) ==
                      map[std::size_t(std::abs(old) - 1)] + 1,
                  "all combined channel magnitudes follow native map");
        }
    check(out.data.face_data[0].point_range == mesh.data.face_data[0].point_range &&
              out.data.face_data[0].source_index == 999 &&
              out.data.indices.indices[face_channel] == mesh.data.indices.indices[face_channel],
          "face data is not recomputed after coordinate replacement");
    check(out.data.edge_chains[0].point_indices == mesh.data.edge_chains[0].point_indices &&
              out.data.edge_chains[0].topology_ids == mesh.data.edge_chains[0].topology_ids &&
              out.edge_chain_active,
          "native unremapped edge chains preserved");
    check(out.texture_id == 123 && out.illumination_name == mesh.illumination_name &&
              out.face_material_ids == mesh.face_material_ids &&
              out.face_uv_points == mesh.face_uv_points &&
              out.face_smooth_groups == mesh.face_smooth_groups &&
              out.double_colors == mesh.double_colors && out.float_colors == mesh.float_colors &&
              out.integer_colors == mesh.integer_colors && out.color_table == mesh.color_table,
          "all independent metadata retained");
    check(out.pool_rows == mesh.pool_rows && out.index_rows == mesh.index_rows &&
              out.edge_chain_rows == 5 && out.pool_active == mesh.pool_active &&
              out.data.coordinates.parameter_scope == 42,
          "source vector layout and caller controls retained");
    check(mesh.data.indices.indices[point_channel][0] == -1 &&
              mesh.data.coordinates.points.size() == 6,
          "source unchanged through combination and visibility");
    auto unindexed = mesh;
    unindexed.mesh_style = 4;
    unindexed.data.indices.indices[point_channel] = {-1, 2, 0, 1, 2, 0};
    auto raw = finalize_native_polyface_mesh(unindexed, budget);
    check(!raw.combination_applied &&
              raw.output.data.coordinates.points == mesh.data.coordinates.points &&
              raw.output.data.indices.indices[point_channel][0] == 1,
          "style skip does not skip later visibility scan");
    auto no_points = unindexed;
    no_points.data.coordinates.points.clear();
    check(!finalize_native_polyface_mesh(no_points, budget).combination_applied,
          "empty point pool skips combine");
    auto cap = emit_native_tube_mesh_cap({{{0, 0, 0}, {4, 0, 0}, {4, 3, 0}, {0, 3, 0}}}, false, {},
                                         budget);
    auto cap_final = finalize_native_polyface_mesh(cap.mesh, budget);
    check(cap.complete && cap_final.combination_applied &&
              cap_final.output.data.face_data.size() == 1,
          "real cap generator output enters native finalization");
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "unsafe or over-budget finalization rejected");
    };
    rejects([&] { run({INT32_MIN, 1}); });
    rejects([&] { run({1, INT32_MIN}); });
    auto bad = mesh;
    bad.data.indices.indices[point_channel][0] = 999;
    rejects([&] { finalize_native_polyface_mesh(bad, budget); });
    TubeBudget measured;
    finalize_native_polyface_mesh(mesh, measured);
    for (auto limit : {std::size_t(0), measured.work / 2, measured.work - 1})
        rejects([&] {
            TubeBudget small;
            small.max_work = limit;
            finalize_native_polyface_mesh(mesh, small);
        });
    rejects([&] {
        TubeBudget small;
        small.max_control_points = 3;
        resolve_native_polyface_edge_visibility(channels, small);
    });
    auto future = std::async(std::launch::async, [&] {
        TubeBudget b;
        return finalize_native_polyface_mesh(mesh, b);
    });
    auto concurrent = future.get();
    check(concurrent.report == finalized.report &&
              concurrent.output.data.indices.indices == out.data.indices.indices,
          "independent finalization calls are deterministic");
    return checks;
}
