#include "native_tube_mesh_assembly.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_tube_mesh_assembly_tests() {
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
        check(caught, "sweep assembly rejects unsafe or over-budget input");
    };
    auto surface = [](double z) {
        return Json{{"_type", "BsplineSurface"},
                    {"orderU", 2},
                    {"orderV", 2},
                    {"numPolesU", 5},
                    {"numPolesV", 2},
                    {"closedU", false},
                    {"closedV", false},
                    {"poles", {0, 0, z,     1, 0, z,     1, 1, z,     0, 1, z,     0, 0, z,
                               0, 0, z + 1, 1, 0, z + 1, 1, 1, z + 1, 0, 1, z + 1, 0, 0, z + 1}},
                    {"weights", nullptr},
                    {"knotsU", {0, 0, .25, .5, .75, 1, 1}},
                    {"knotsV", nullptr},
                    {"numRulesU", 0},
                    {"numRulesV", 0},
                    {"holeOrigin", 0},
                    {"boundaries", nullptr}};
    };
    TubeBudget budget;
    SweptBodyPatchGroup group;
    group.patches = {{surface(0), {}}};
    auto grid = prepare_tube_mesh_grid(group, false, .01, .1, 10000, budget);
    check(grid.success, "closed square side grid prepared");
    NativeTubeMeshAssemblyOptions options;
    options.profile_closed = true;
    options.cap_eligible = true;
    auto result = assemble_native_tube_mesh_groups({grid}, options, budget);
    check(result.complete && result.finalized && result.caps.published,
          "prepared sweep side and both caps reach finalization");
    check(result.sources.size() == 6 && result.groups.size() == 1,
          "four side strips then two caps in original order");
    const auto &mesh = result.finalized->output;
    check(mesh.data.coordinates.points.size() == 8 &&
              mesh.data.indices.indices[point_channel].size() == 48,
          "closed cube retains eight original corners and twelve triangles");
    double volume = 0;
    const auto &points = mesh.data.coordinates.points;
    const auto &idx = mesh.data.indices.indices[point_channel];
    for (std::size_t i = 0; i < idx.size(); i += 4) {
        check(idx[i + 3] == 0, "triangle boundary preserved");
        const auto &a = points.at(std::abs(idx[i]) - 1);
        const auto &b = points.at(std::abs(idx[i + 1]) - 1);
        const auto &c = points.at(std::abs(idx[i + 2]) - 1);
        volume += a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
                  a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    check(std::abs(std::abs(volume / 6) - 1) < 1e-12, "assembled closed cube signed volume");
    for (auto c : {normal_channel, parameter_channel, face_channel}) {
        check(mesh.data.indices.indices[c].size() == idx.size(),
              "complete normal UV and face correspondence survives assembly");
        for (std::size_t i = 0; i < idx.size(); ++i)
            check((idx[i] == 0) == (mesh.data.indices.indices[c][i] == 0),
                  "independent channel terminators remain aligned");
    }
    check(mesh.data.face_data.size() == 6, "four side face records followed by both caps");
    check(result.assembly.assembled.report["sources"][0]["normal_index_source"] ==
                  "point_fallback" &&
              result.assembly.assembled.report["sources"][1]["normal_index_source"] == "explicit" &&
              result.assembly.assembled.report["unrepresented_attribute_indices"] == 0,
          "late settings use fallback once and then retain explicit source attributes");
    const auto &strip = result.sources.front();
    check(strip.pool_rows[native_point_pool] == 2 && strip.index_rows[point_channel] == 3 &&
              strip.index_tags[normal_channel].tag == 1 &&
              strip.index_tags[parameter_channel].tag == 1 && strip.data.num_per_face == 0 &&
              strip.mesh_style == 1 && strip.num_per_row == 0,
          "rectangular strip retains grid row metadata and copied point-index tags");
    // Reconstruct the original pre-triangulation runtime layout independently.
    auto original = make_native_polyface_mesh(3, 5);
    original.data.coordinates = strip.data.coordinates;
    original.pool_rows[native_point_pool] = 2;
    original.num_per_row = 2;
    auto tri = triangulate_native_polyface_mesh(original, budget);
    check(tri.complete && tri.output.pool_rows == strip.pool_rows &&
              tri.output.index_rows == strip.index_rows &&
              tri.output.pool_active == strip.pool_active,
          "adapted strip metadata agrees with actual typed native triangulation");
    for (unsigned mask = 0; mask < 4; ++mask) {
        auto optional = options;
        optional.normals = mask & 1;
        optional.parameters = mask & 2;
        for (bool cap : {false, true}) {
            optional.cap_eligible = cap;
            auto r = assemble_native_tube_mesh_groups({grid}, optional, budget);
            check(r.complete && r.finalized && r.caps.published == cap,
                  "all attribute and cap request combinations complete");
            const auto &m = r.finalized->output;
            check(m.data.coordinates.normals.empty() == !optional.normals &&
                      m.data.coordinates.parameters.empty() == !optional.parameters &&
                      m.data.face_data.empty() == !optional.parameters,
                  "unrequested attribute pools remain absent");
            check(m.data.indices.indices[point_channel].size() == (cap ? 48u : 32u),
                  "cap eligibility controls only the two cap meshes");
        }
    }
    group.patches.push_back({surface(1), {}});
    auto two = prepare_tube_mesh_grid(group, false, .01, .1, 10000, budget);
    auto stacked = assemble_native_tube_mesh_groups({two}, options, budget);
    check(stacked.complete && stacked.sources.size() == 10 &&
              stacked.finalized->output.data.coordinates.points.size() == 12,
          "two path patches assemble in order with only outer caps");
    auto no_caps = options;
    no_caps.cap_eligible = false;
    auto doubled = assemble_native_tube_mesh_groups({grid, grid}, no_caps, budget);
    check(doubled.complete && doubled.sources.size() == 8 &&
              doubled.finalized->output.data.indices.indices[point_channel].size() == 64,
          "equal source groups retain all faces and only native coordinate pooling");
    group.patches.resize(1);
    group.patches[0].boundary_points = {{{0, 0},
                                         {.25, 0},
                                         {.5, 0},
                                         {.75, 0},
                                         {1, 0},
                                         {1, .8},
                                         {.75, .8},
                                         {.5, .8},
                                         {.25, .8},
                                         {0, .8},
                                         {0, 0}}};
    auto trim = prepare_tube_mesh_grid(group, false, .01, .1, 10000, budget);
    auto clipped = assemble_native_tube_mesh_groups({trim}, options, budget);
    check(clipped.complete && clipped.sources.front().index_rows[point_channel] == 1 &&
              clipped.sources.front().index_tags[normal_channel].tag == 1,
          "trimmed source preserves its independent indexed layout");
    double max_z = 0;
    for (const auto &p : clipped.finalized->output.data.coordinates.points)
        max_z = std::max(max_z, p[2]);
    check(std::abs(max_z - .8) < 1e-12, "trimmed end cap follows actual collected boundary");
    auto broken = trim;
    broken.patches[0].path.parameters = {0, .5};
    auto failed = assemble_native_tube_mesh_groups({grid, broken}, no_caps, budget);
    check(!failed.complete && !failed.finalized && failed.groups.size() == 2 &&
              !failed.sources.empty(),
          "late group failure retains diagnostics without final mesh");
    auto empty = assemble_native_tube_mesh_groups({}, no_caps, budget);
    check(empty.complete && empty.finalized && empty.finalized->output.data.two_sided &&
              !empty.finalized->output.data.indices.active[normal_channel] &&
              !empty.finalized->output.data.indices.active[face_channel],
          "empty prepared sequence retains builder construction flags");
    // Separate profile groups form the original ordered outer/inner cap rings.
    SweptBodyPatchGroup inner;
    auto inner_surface = surface(0);
    for (std::size_t i = 0; i < inner_surface["poles"].size(); i += 3) {
        const double x = inner_surface["poles"][i].get<double>();
        const double y = inner_surface["poles"][i + 1].get<double>();
        inner_surface["poles"][i] = .25 + .5 * y;
        inner_surface["poles"][i + 1] = .25 + .5 * x;
    }
    inner.patches = {{inner_surface, {}}};
    auto inner_grid = prepare_tube_mesh_grid(inner, false, .01, .1, 10000, budget);
    auto holed = assemble_native_tube_mesh_groups({grid, inner_grid}, options, budget);
    check(holed.complete && holed.sources.size() == 10 && holed.caps.published,
          "outer and inner source groups retain ordered multi-ring caps");
    double hole_volume = 0;
    const auto &hm = holed.finalized->output.data;
    const auto &hi = hm.indices.indices[point_channel];
    for (std::size_t i = 0; i < hi.size(); i += 4) {
        const auto &a = hm.coordinates.points.at(std::abs(hi[i]) - 1);
        const auto &b = hm.coordinates.points.at(std::abs(hi[i + 1]) - 1);
        const auto &c = hm.coordinates.points.at(std::abs(hi[i + 2]) - 1);
        hole_volume += a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
                       a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    check(std::abs(std::abs(hole_volume / 6) - .75) < 1e-12,
          "multi-ring assembled solid volume excludes inner hole");
    auto curved = surface(0);
    curved["orderU"] = 3;
    curved["numPolesU"] = 3;
    curved["poles"] = {1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 2, 1, 1, 2, 0, 1, 2};
    curved["weights"] = {1, std::sqrt(.5), 1, 1, std::sqrt(.5), 1};
    // BGFB stores homogeneous poles, including weighted Z.
    for (std::size_t i = 0; i < 6; ++i)
        for (std::size_t c = 0; c < 3; ++c)
            curved["poles"][i * 3 + c] = curved["poles"][i * 3 + c].get<double>() *
                                        curved["weights"][i].get<double>();
    curved["knotsU"] = nullptr;
    SweptBodyPatchGroup curved_group;
    curved_group.patches = {{curved, {}}};
    auto curved_grid = prepare_tube_mesh_grid(curved_group, false, .01, .1, 10000, budget);
    auto curved_options = no_caps;
    curved_options.profile_closed = false;
    auto cylinder = assemble_native_tube_mesh_groups({curved_grid}, curved_options, budget);
    check(cylinder.complete, "rational curved side sampling reaches complete assembly");
    for (const auto &p : cylinder.finalized->output.data.coordinates.points)
        check(std::abs(p[0] * p[0] + p[1] * p[1] - 1) < 1e-12,
              "assembled rational cylinder vertices retain analytic radius");
    NativeBuilderPolyface independent;
    independent.coordinates.points = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    independent.coordinates.normals = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    independent.indices.indices[point_channel] = {1, 2, 3, 0};
    independent.indices.indices[normal_channel] = {2, 1, 3, 0};
    NativeBuilderPolyfaceOptions deferred;
    deferred.parameters_required = false;
    deferred.attributes_enabled_at_construction = false;
    auto loss = assemble_native_builder_polyfaces({independent}, deferred, budget);
    check(!loss.complete && loss.report["unrepresented_attribute_indices"] == 4,
          "late enabled mismatching explicit indices stay visibly incomplete");
    deferred.attributes_enabled_at_construction = true;
    check(assemble_native_builder_polyfaces({independent}, deferred, budget).complete,
          "construction-time attribute option retains ordinary matched behavior");
    TubeBudget measured;
    assemble_native_tube_mesh_groups({grid}, options, measured);
    for (auto limit : {std::size_t(0), measured.work / 2, measured.work - 1})
        rejects([&] {
            TubeBudget b;
            b.max_work = limit;
            assemble_native_tube_mesh_groups({grid}, options, b);
        });
    rejects([&] {
        TubeBudget b;
        b.max_control_points = 3;
        assemble_native_tube_mesh_groups({grid}, options, b);
    });
    auto bad = grid;
    bad.success = false;
    rejects([&] { assemble_native_tube_mesh_groups({bad}, options, budget); });
    auto future = std::async(std::launch::async, [&] {
        TubeBudget b;
        return assemble_native_tube_mesh_groups({grid}, options, b);
    });
    check(future.get().finalized->output.data.indices.indices == mesh.data.indices.indices,
          "independent full assembly calls are deterministic");
    return checks;
}
