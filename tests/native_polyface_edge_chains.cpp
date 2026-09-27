#include "native_polyface_edge_chains.hpp"
#include "native_polyface_smooth_normals.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_polyface_edge_chains_tests() {
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
        check(caught, "edge-chain operation rejects unsafe reads and exhausted budgets");
    };
    auto square = [] {
        NativePolyfaceFaceDataState s;
        s.mesh.coordinates.points = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 1, 3, 4, 0};
        return s;
    };
    auto run = [](const NativePolyfaceFaceDataState &s, std::size_t draw = 0) {
        TubeBudget b;
        return build_native_polyface_edge_chains(s, draw, b);
    };
    auto expect = [&](const NativeBuilderEdgeChain &e, std::uint32_t type,
                      std::vector<std::uint32_t> ids, std::vector<std::int32_t> points) {
        check(e.topology_type == type && e.topology_ids == ids && e.point_indices == points,
              "native edge chain preserves topology type, source IDs, direction and one-based "
              "vertices");
    };
    const auto source = square();
    const auto base = run(source);
    check(base.native_succeeded && base.native_status == 0 && base.complete &&
              base.output.mesh.edge_chains.size() == 5 && base.report["shared_edge_records"] == 1,
          "two visible adjacent triangles produce five individual edge records");
    const auto &edges = base.output.mesh.edge_chains;
    expect(edges[0], 24, {0, 1}, {1, 2});
    expect(edges[1], 24, {1, 2}, {2, 3});
    expect(edges[2], 4, {1, 3}, {3, 1});
    expect(edges[3], 24, {2, 3}, {3, 4});
    expect(edges[4], 24, {3, 0}, {4, 1});
    check(base.output.mesh.coordinates.points == source.mesh.coordinates.points &&
              base.output.mesh.indices.indices == source.mesh.indices.indices,
          "edge generation preserves point pool and every source index channel");
    {
        auto s = source;
        s.mesh.indices.indices[point_channel] = {1, -2, 3, 0};
        auto r = run(s);
        check(r.complete && r.output.mesh.edge_chains.size() == 2,
              "negative source edge is omitted in registration");
        expect(r.output.mesh.edge_chains[0], 24, {0, 1}, {1, 2});
        expect(r.output.mesh.edge_chains[1], 24, {2, 0}, {3, 1});
        s.mesh.indices.indices[point_channel] = {-1, -2, -3, 0};
        r = run(s);
        check(r.complete && r.native_succeeded && r.output.mesh.edge_chains.empty(),
              "all-hidden mesh has a successful empty edge list");
        s = source;
        s.mesh.indices.indices[point_channel][2] = -3;
        r = run(s);
        check(r.complete && r.output.mesh.edge_chains.size() == 5 &&
                  r.report["shared_edge_records"] == 0 &&
                  r.report["emitted"][2]["emitted_from_hidden_half_edge"] == true,
              "second pass can emit from a hidden edge when its other occurrence registered it");
        expect(r.output.mesh.edge_chains[2], 24, {2, 0}, {3, 1});
        check(r.report["emitted"][2]["first_visible_facet"] == 3,
              "topology vertex orientation comes from emission, not the registering half-edge");
        s.mesh.indices.indices[point_channel][4] = -1;
        r = run(s);
        check(r.complete && r.output.mesh.edge_chains.size() == 4,
              "shared edge with both occurrences hidden is never emitted");
    }
    {
        auto s = source;
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 2, 1, 4, 0, 1, 2, 4, 0};
        auto r = run(s);
        check(r.complete && r.report["edges_with_over_two_visible_occurrences"] == 1,
              "three visible incident facets preserve native endpoint-pair selection");
        expect(r.output.mesh.edge_chains[0], 4, {1, 5}, {1, 2});
        check(r.report["emitted"][0]["visible_occurrences"] == 3,
              "middle incident face is observable in the occurrence count but does not replace "
              "native IDs");
        s.mesh.indices.indices[point_channel] = {1, 2, 0, 1, 0, 1, 2, 3, 0, 1, 3, 4, 0};
        r = run(s);
        check(r.complete && r.report["passes"][0]["skipped_short_facets"] == 2 &&
                  r.output.mesh.edge_chains.size() == 5,
              "one- and two-corner facets are skipped without advancing native qualified-face "
              "numbers");
        expect(r.output.mesh.edge_chains[2], 4, {1, 3}, {3, 1});
        s.mesh.indices.indices[point_channel] = {1, 2, 1, 0};
        r = run(s);
        check(r.complete && r.output.mesh.edge_chains.size() == 2,
              "repeated and collapsed edges retain identity");
        expect(r.output.mesh.edge_chains[0], 24, {0, 1}, {1, 2});
        expect(r.output.mesh.edge_chains[1], 24, {0, 0}, {1, 1});
        check(r.report["emitted"][0]["visible_occurrences"] == 2 &&
                  r.report["shared_edge_records"] == 0,
              "two occurrences in one facet do not create a shared-face topology ID");
        s = source;
        s.mesh.coordinates.points.insert(s.mesh.coordinates.points.end(),
                                         source.mesh.coordinates.points.begin(),
                                         source.mesh.coordinates.points.end());
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 5, 6, 7, 0};
        r = run(s);
        check(r.complete && r.output.mesh.edge_chains.size() == 6 &&
                  r.report["shared_edge_records"] == 0,
              "equal coordinates with distinct source vertex IDs are never joined");
    }
    {
        auto s = source;
        s.mesh.num_per_face = 4;
        auto r = run(s);
        check(r.complete && r.report["emitted"] == base.report["emitted"],
              "fixed padded faces retain source positions and output order");
        s = source;
        s.mesh.num_per_face = 3;
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 1, 3, 4};
        r = run(s);
        check(r.complete && r.output.mesh.edge_chains.size() == 5,
              "fixed unpadded faces wrap within each block");
        expect(r.output.mesh.edge_chains[2], 4, {1, 3}, {3, 1});
        s = source;
        s.mesh.indices.indices[point_channel] = {0, 1, 2, 3, 0, 0, 1, 3, 4};
        r = run(s);
        check(r.complete && r.output.mesh.edge_chains.size() == 5 &&
                  r.report["emitted"][0]["read_index"] == 1,
              "variable leading zeros and unterminated final face preserve source read positions");
        s = source;
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 1, 2, 99, 0, 1, 3, 4, 0};
        r = run(s);
        check(r.native_succeeded && !r.complete && r.output.mesh.edge_chains.size() == 3 &&
                  r.report["passes"][0]["unvisited_nonzero_indices"] == 6 &&
                  r.report["passes"][1]["invalid_point_facets"] == 1,
              "native success after invalid-facet traversal retains partial output with incomplete "
              "diagnostics");
        s = source;
        s.mesh.num_per_face = 4;
        s.mesh.indices.indices[point_channel] = {0, 0, 0, 0, 1, 2, 3, 0};
        r = run(s);
        check(r.native_succeeded && !r.complete && r.output.mesh.edge_chains.empty(),
              "empty first fixed block stops both passes and does not conceal unread faces");
        s = source;
        s.mesh.indices.indices[point_channel].clear();
        r = run(s);
        check(r.complete && r.native_succeeded && r.output.mesh.edge_chains.empty(),
              "empty indexed mesh has no edges and returns native success");
        s = source;
        s.mesh.coordinates.normals = {{0, 0, 1}};
        s.mesh.indices.indices[normal_channel] = {1, 1, 99, 0, 1, 1, 1, 0};
        r = run(s);
        check(r.native_succeeded && !r.complete && r.output.mesh.edge_chains.size() == 5 &&
                  r.report["passes"][0]["truncated_attribute_visits"] == 1,
              "all-data visitor truncation is reported even though edge keys use only point "
              "identities");
        s.mesh.indices.indices[normal_channel] = {1};
        rejects([&] { run(s); });
    }
    {
        auto s = base.output;
        s.mesh.indices.indices[point_channel] = {999};
        const auto r = run(s);
        check(!r.native_succeeded && r.native_status == 1 && !r.complete &&
                  r.report["reason"] == "edge_chains_already_exist" &&
                  r.output.mesh.edge_chains.size() == 5 &&
                  r.output.mesh.indices.indices[point_channel] ==
                      s.mesh.indices.indices[point_channel],
              "existing chains reject generation before traversing even invalid point references");
        expect(r.output.mesh.edge_chains[2], 4, {1, 3}, {3, 1});
        auto arbitrary = run(source, std::numeric_limits<std::size_t>::max());
        check(arbitrary.complete && arbitrary.report["emitted"] == base.report["emitted"] &&
                  arbitrary.report["draw_method_used"] == false,
              "native draw-method argument does not affect topology identity or output order");
        s = source;
        s.mesh.face_data = {native_null_face_data()};
        s.mesh.face_data[0].source_index = 1234;
        s.mesh.indices.indices[face_channel] = {9, 9, 9, 0, 9, 9, 9, 0};
        s.mesh.indices.indices[color_channel] = {5, 5, 5, 0, 5, 5, 5, 0};
        auto unchanged = run(s);
        check(
            unchanged.complete && unchanged.output.mesh.face_data[0].source_index == 1234 &&
                unchanged.output.mesh.indices.indices == s.mesh.indices.indices,
            "face records and unrelated channels are preserved, not used for generated facet IDs");
        expect(unchanged.output.mesh.edge_chains[2], 4, {1, 3}, {3, 1});
    }
    {
        TubeBudget b;
        const auto smooth = build_native_polyface_approximate_normals(source, 1, 1, true, b);
        const auto r = run(smooth.output);
        check(r.complete && r.output.mesh.edge_chains.size() == 4 &&
                  r.report["shared_edge_records"] == 0,
              "edge generation consumes actual visibility produced by smooth-normal preprocessing");
        const auto uv = build_native_polyface_parameters(r.output, 2, b);
        const auto faces = build_native_polyface_face_data(uv.output, b);
        const auto assembled = assemble_native_builder_polyfaces({faces.output.mesh}, {}, b);
        check(uv.complete && faces.complete && assembled.complete &&
                  assembled.edge_chains.size() == 4,
              "generated edges survive normal, UV, face-data and matched-builder integration");
        for (std::size_t i = 0; i < 4; ++i)
            expect(assembled.edge_chains[i], r.output.mesh.edge_chains[i].topology_type,
                   r.output.mesh.edge_chains[i].topology_ids,
                   r.output.mesh.edge_chains[i].point_indices);
    }
    {
        TubeBudget measured;
        build_native_polyface_edge_chains(source, 0, measured);
        for (auto limit : {std::size_t(0), measured.work / 2, measured.work - 1}) {
            TubeBudget b;
            b.max_work = limit;
            rejects([&] { build_native_polyface_edge_chains(source, 0, b); });
        }
        for (auto limit : {std::size_t(0), std::size_t(13), std::size_t(30)}) {
            TubeBudget b;
            b.max_control_points = limit;
            rejects([&] { build_native_polyface_edge_chains(source, 0, b); });
        }
        auto task = [&] { return run(source); };
        auto a = std::async(std::launch::async, task), b = std::async(std::launch::async, task);
        auto x = a.get(), y = b.get();
        check(x.complete && y.complete && x.report == y.report &&
                  x.output.mesh.edge_chains.size() == 5,
              "concurrent edge generation has no shared mutable visitor or registration state");
        check(source.mesh.edge_chains.empty() &&
                  source.mesh.indices.indices == square().mesh.indices.indices,
              "all success and failure paths leave the original mesh unchanged");
    }
    return checks;
}
