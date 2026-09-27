#include "native_polyface_connectivity.hpp"
#include "native_polyface_attributes.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_polyface_connectivity_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "connectivity rejects unsafe access and exhausted budgets");
    };
    auto triangle = [] {
        NativePolyfaceFaceDataState s;
        s.mesh.coordinates.points = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        s.mesh.indices.indices[point_channel] = {1, -2, 3, 0};
        return s;
    };
    TubeBudget b;
    const auto source = triangle();
    const auto base = build_native_polyface_connectivity(source, false, b);
    auto verify = [&](const NativePolyfaceConnectivity &r) {
        std::vector<unsigned> face_in(r.nodes.size()), vertex_in(r.nodes.size());
        for (std::size_t i = 0; i < r.nodes.size(); ++i) {
            const auto &n = r.nodes[i];
            check(n.face_successor < r.nodes.size() && n.vertex_successor < r.nodes.size(),
                  "graph successors stay in the node array");
            ++face_in[n.face_successor];
            ++vertex_in[n.vertex_successor];
            check(r.nodes[n.face_successor].vertex_successor == (i ^ 1),
                  "edge mates remain the originally allocated pair after twisting");
            check(n.vertex_index >= 0 && static_cast<std::size_t>(n.vertex_index) < r.points.size(),
                  "vertex labels address original coordinates");
        }
        for (std::size_t i = 0; i < r.nodes.size(); ++i)
            check(face_in[i] == 1 && vertex_in[i] == 1,
                  "both successors are permutations, including exterior loops");
        for (std::size_t i = 0; i < r.half_edges.size(); ++i) {
            const auto &e = r.half_edges[i];
            check(e.node < r.nodes.size() &&
                      r.nodes[e.node].read_index == static_cast<std::int32_t>(e.read_index) &&
                      r.read_to_half_edge[e.read_index] == i,
                  "half edges retain exact read positions and reverse mapping");
        }
    };
    check(base.native_succeeded && base.complete && base.nodes.size() == 6 &&
              base.half_edges.size() == 3 && base.report["boundary_edges"] == 3,
          "one triangle creates three independent boundary edges");
    const std::array<std::size_t, 6> expected_face{4, 3, 0, 5, 2, 1},
        expected_vertex{3, 4, 5, 0, 1, 2};
    const std::array<std::int32_t, 6> expected_reads{0, -1, 2, -1, 1, -1},
        expected_points{0, 1, 2, 0, 1, 2};
    for (std::size_t i = 0; i < 6; ++i) {
        const auto &n = base.nodes[i];
        check(n.face_successor == expected_face[i] && n.vertex_successor == expected_vertex[i] &&
                  n.read_index == expected_reads[i] && n.vertex_index == expected_points[i] &&
                  n.mask == (i % 2 ? 0x203u : 0x202u),
              "triangle has analytical native node order, face loops, vertex rings and boundary "
              "masks");
    }
    check(base.points == source.mesh.coordinates.points && base.points.size() == 4,
          "unused source coordinates remain in original order");
    check(base.half_edges[2].read_index == 1 && !base.half_edges[2].visible &&
              (base.nodes[base.half_edges[2].node].mask & native_mtg_primary),
          "unpaired hidden source edge becomes primary in both native boundary nodes");
    verify(base);
    {
        const auto normal_mesh = build_native_polyface_normals(source, b);
        const auto topology = build_native_polyface_connectivity(normal_mesh.output, false, b);
        check(normal_mesh.complete && topology.complete && topology.report == base.report,
              "native per-face normal output feeds the connectivity stage without changing native "
              "vertex identities");
    }
    {
        auto s = triangle();
        s.mesh.indices.indices[point_channel] = {1, 2, -3, 0, -1, 3, 4, 0};
        auto r = build_native_polyface_connectivity(s, false, b);
        check(r.complete && r.nodes.size() == 10 && r.report["paired_edges"] == 1 &&
                  r.report["boundary_edges"] == 4,
              "two adjacent triangles share one original edge pair");
        for (const auto &e : r.half_edges)
            if (e.read_index == 2 || e.read_index == 4)
                check(r.nodes[e.node].mask == 0 && r.nodes[e.node ^ 1].mask == 0,
                      "paired invisible edges preserve the absence of both primary and boundary "
                      "masks");
        verify(r);
        s.mesh.indices.indices[point_channel] = {1, 3, 2, 0, 1, 2, 4, 0, 2, 3, 4, 0, 3, 1, 4, 0};
        r = build_native_polyface_connectivity(s, false, b);
        check(r.complete && r.nodes.size() == 12 && r.report["paired_edges"] == 6 &&
                  r.report["boundary_edges"] == 0,
              "consistently oriented tetrahedron forms a graph without exterior nodes");
        for (const auto &n : r.nodes)
            check(n.mask == native_mtg_primary, "closed visible mesh has only primary-edge masks");
        verify(r);
        s = triangle();
        s.mesh.coordinates.points.insert(s.mesh.coordinates.points.end(),
                                         {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}});
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 5, 6, 7, 0};
        r = build_native_polyface_connectivity(s, false, b);
        check(r.complete && r.nodes.size() == 12 && r.report["paired_edges"] == 0,
              "equal coordinates with distinct native identities never acquire shared topology");
        verify(r);
    }
    {
        auto s = triangle();
        s.mesh.indices.indices[point_channel] = {1, 2, 2, 0, 2, 1, 3, 0, 1, 2, 4, 0};
        auto r = build_native_polyface_connectivity(s, false, b);
        check(r.complete && r.report["four_edge_degenerate_couplings"] == 1 &&
                  r.report["paired_edges"] == 2 && r.nodes.size() == 14,
              "four-edge class couples through its collapsed triangular neighbor");
        std::size_t polar = 0;
        for (const auto &n : r.nodes)
            if (n.mask & native_mtg_polar_loop) {
                ++polar;
                check(n.mask == 0x1203 && n.read_index == -1,
                      "collapsed unpaired edge marks only its exterior side polar");
            }
        check(polar == 1, "collapsed triangle produces one polar exterior node");
        verify(r);
        auto filtered = build_native_polyface_connectivity(s, true, b);
        check(filtered.complete && filtered.report["skipped_degenerate_faces"] == 1 &&
                  filtered.report["filtered_corners"] == 3 &&
                  filtered.report["four_edge_degenerate_couplings"] == 0 &&
                  filtered.report["paired_edges"] == 1,
              "optional native filter packs repeats and skips the resulting two-point face");
        verify(filtered);
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 2, 1, 4, 0, 1, 2, 4, 0};
        r = build_native_polyface_connectivity(s, false, b);
        check(r.native_succeeded && r.report["four_edge_degenerate_couplings"] == 0 &&
                  r.report["boundary_edges"].get<std::size_t>() >= 3,
              "three-way shared endpoint class is kept as separate native boundary edges");
        verify(r);
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 1, 2, 3, 0};
        r = build_native_polyface_connectivity(s, false, b);
        check(r.native_succeeded && !r.complete && r.report["same_direction_pairs"] == 3 &&
                  r.report["vertex_label_mismatches"].get<std::size_t>() > 0,
              "native size-two classes pair even identical directions, with explicit label "
              "inconsistency diagnostics");
        verify(r);
    }
    {
        auto s = triangle();
        s.mesh.indices.indices[point_channel] = {1, 1, 2, 3, 1, 0};
        auto r = build_native_polyface_connectivity(s, true, b);
        check(r.complete && r.half_edges.size() == 3 && r.report["filtered_corners"] == 2 &&
                  r.read_to_half_edge[1] == SIZE_MAX,
              "filtered duplicate identities do not receive invented half edges");
        const auto &e = r.half_edges[r.read_to_half_edge[0]];
        check(e.successor_read_index == 2,
              "packed edge retains original nonconsecutive corner positions");
        verify(r);
        s.mesh.indices.indices[point_channel] = {-1, 0};
        r = build_native_polyface_connectivity(s, false, b);
        check(r.complete && r.nodes.size() == 2 && r.nodes[0].face_successor == 0 &&
                  r.nodes[1].face_successor == 1 && r.nodes[0].vertex_successor == 1 &&
                  r.nodes[1].mask == 0x1203,
              "one-corner native face forms a polar self loop without geometric repair");
        verify(r);
        s.mesh.num_per_face = 4;
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 1, 3, 4, 0};
        r = build_native_polyface_connectivity(s, false, b);
        check(r.complete && r.half_edges.size() == 6,
              "fixed padding does not create spurious zero-vertex edges");
        verify(r);
        s.mesh.num_per_face = 0;
        s.mesh.indices.indices[point_channel] = {0, 1, 2, 3, 0, 0, 1, 3, 4};
        r = build_native_polyface_connectivity(s, false, b);
        check(r.complete && r.read_to_half_edge.size() == 9 && r.read_to_half_edge[0] == SIZE_MAX &&
                  r.read_to_half_edge[4] == SIZE_MAX,
              "variable leading zeros and unterminated last face preserve read positions");
        verify(r);
        s = triangle();
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 1, 99, 3, 0};
        r = build_native_polyface_connectivity(s, false, b);
        check(r.native_succeeded && !r.complete && r.half_edges.size() == 3 &&
                  r.report["invalid_point_facets"] == 1 &&
                  r.report["unvisited_nonzero_indices"] == 3,
              "invalid later face leaves partial original connectivity with diagnostics");
        s.mesh.indices.indices[point_channel].clear();
        r = build_native_polyface_connectivity(s, false, b);
        check(!r.native_succeeded && !r.complete && r.nodes.empty() &&
                  r.points == s.mesh.coordinates.points,
              "empty half-edge graph returns native false but retains all source coordinates");
    }
    for (std::size_t copies : {11, 14, 39}) {
        auto s = triangle();
        s.mesh.indices.indices[point_channel].clear();
        for (std::size_t i = 0; i < copies; ++i)
            s.mesh.indices.indices[point_channel].insert(
                s.mesh.indices.indices[point_channel].end(), {1, 2, 3, 0});
        auto r = build_native_polyface_connectivity(s, false, b);
        check(r.complete && r.report["boundary_edges"] == 3 * copies &&
                  r.nodes.size() == 6 * copies,
              "large repeated endpoint classes survive native sort thresholds as independent "
              "boundary faces");
        for (std::size_t i = 1; i < r.half_edges.size(); ++i) {
            auto key = [](const auto &e) {
                return std::pair{std::min(e.vertex0, e.vertex1), std::max(e.vertex0, e.vertex1)};
            };
            check(key(r.half_edges[i - 1]) <= key(r.half_edges[i]),
                  "native sorted half-edge keys are monotonic");
        }
        verify(r);
    }
    {
        TubeBudget measured;
        build_native_polyface_connectivity(source, false, measured);
        for (auto limit : {std::size_t(0), std::size_t(20), measured.work / 2, measured.work - 1}) {
            TubeBudget limited;
            limited.max_work = limit;
            rejects([&] { build_native_polyface_connectivity(source, false, limited); });
        }
        for (auto limit : {std::size_t(0), std::size_t(8), std::size_t(20)}) {
            TubeBudget limited;
            limited.max_control_points = limit;
            rejects([&] { build_native_polyface_connectivity(source, false, limited); });
        }
        auto s = source;
        s.mesh.indices.indices[normal_channel] = {1};
        s.mesh.coordinates.normals = {{0, 0, 1}};
        rejects([&] { build_native_polyface_connectivity(s, false, b); });
        check(source.mesh.indices.indices[point_channel] == std::vector<std::int32_t>{1, -2, 3, 0},
              "source remains immutable on success and failure");
        auto run = [&] {
            TubeBudget local;
            return build_native_polyface_connectivity(source, false, local);
        };
        auto f1 = std::async(std::launch::async, run), f2 = std::async(std::launch::async, run);
        auto a = f1.get(), z = f2.get();
        check(a.report == base.report && z.report == base.report &&
                  a.nodes[0].face_successor == z.nodes[0].face_successor,
              "independent native connectivity computations are deterministic and concurrent");
    }
    return checks;
}
