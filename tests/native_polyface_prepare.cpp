#include "native_polyface_prepare.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_polyface_prepare_tests() {
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
        check(caught, "outer preparation rejects unsupported queries and unsafe resource use");
    };
    auto base = [] {
        auto s = make_native_polyface_mesh();
        s.data.coordinates.points = {{0, 0, 0}, {4, 0, 0}, {4, 2, 0}, {0, 2, 0}};
        s.data.indices.indices[point_channel] = {1, 2, 3, 4, 0};
        return s;
    };
    auto prepare = [](const NativePolyfaceMesh &s, const NativePolyfacePreparationOptions &o) {
        TubeBudget b;
        return prepare_native_polyface_for_builder(s, o, b);
    };
    auto assemble = [](const std::vector<NativePolyfaceMesh> &s,
                       const NativePolyfacePreparationOptions &o) {
        TubeBudget b;
        return assemble_native_prepared_polyfaces(s, o, {}, b);
    };
    auto steps = [](const NativePolyfacePreparation &r) {
        std::vector<std::string> a;
        for (const auto &s : r.report["steps"])
            a.push_back(s["step"]);
        return a;
    };
    NativePolyfacePreparationOptions options;
    const auto source = base();
    const auto r = prepare(source, options);
    check(r.copied && r.complete &&
              steps(r) == std::vector<std::string>{"query_copy", "approximate_normals",
                                                   "parameters", "face_data", "triangulation"},
          "outer preparation executes missing attributes, face grouping and triangulation in "
          "original order");
    check(r.output.data.indices.indices[point_channel].size() == 8 &&
              r.output.data.coordinates.normals.size() == 4 &&
              r.output.data.coordinates.parameters.size() == 4 &&
              r.output.data.face_data.size() == 1,
          "quad prepares attributes before triangle output without duplicating pools");
    check(r.report["steps"][1]["result"]["max_single_edge_angle"] == 30.0 * 0x1.1df46a2529d39p-6 &&
              r.report["steps"][1]["result"]["max_accumulated_angle"] ==
                  90.0 * 0x1.1df46a2529d39p-6,
          "outer smooth-normal defaults use original degree multiplier and argument order");
    const auto built = assemble({source}, options);
    check(built.native_succeeded && built.complete && built.sources.size() == 1 &&
              built.assembled.indices.indices[point_channel].size() == 8 &&
              built.assembled.coordinates.points.size() == 4 &&
              built.assembled.coordinates.normals.size() == 1 &&
              built.assembled.coordinates.parameters.size() == 4 &&
              built.assembled.face_data.size() == 1,
          "complete indexed outer path reaches original matched builder and coordinate maps");
    check(source.data.coordinates.normals.empty() && source.data.coordinates.parameters.empty() &&
              source.data.indices.indices[point_channel].size() == 5,
          "preparation and assembly leave original source unchanged");
    {
        auto o = options;
        o.normals_required = false;
        o.parameters_required = false;
        o.max_edges_per_face = 8;
        auto s = source;
        s.data.coordinates.points = {{0, 0, 0}, {4, 0, 0}, {1, 1, 0}, {0, 2, 0}};
        o.convex_facets_required = true;
        const auto direct = prepare(s, o);
        check(!direct.copied && direct.complete &&
                  direct.report["source_convexity_tested"] == false && steps(direct).empty() &&
                  direct.output.data.indices.indices == s.data.indices.indices,
              "convexity option alone does not trigger original outer preparation");
        o.edge_chains_required = true;
        const auto convex = prepare(s, o);
        check(
            convex.copied && convex.complete &&
                convex.report["source_has_convex_facets"] == false &&
                steps(convex) ==
                    std::vector<std::string>{"query_copy", "triangulation", "edge_chains"} &&
                convex.report["steps"][1]["reason"] == "source_nonconvex" &&
                convex.output.data.indices.indices[point_channel].size() == 8,
            "another preparation need permits original source-nonconvex conditional triangulation");
        const auto retained = prepare(source, o);
        check(
            retained.complete && retained.report["source_has_convex_facets"] == true &&
                steps(retained) == std::vector<std::string>{"query_copy", "edge_chains"} &&
                retained.output.data.indices.indices[point_channel].size() == 5 &&
                retained.output.data.edge_chains.size() == 4 && !retained.output.edge_chain_active,
            "convex source below limit is retained; generated edge records do not activate vector");
        o.convex_facets_required = false;
        const auto concave = prepare(s, o);
        check(concave.complete && concave.output.data.indices.indices[point_channel].size() == 5,
              "source nonconvexity is observed but does not itself mandate splitting");
    }
    {
        auto o = options;
        o.normals_required = false;
        o.parameters_required = false;
        for (auto limit : {0u, 1u, 2u, 3u, 4u, 5u, UINT32_MAX, UINT32_MAX - 1}) {
            o.max_edges_per_face = limit;
            auto q = prepare(source, o);
            const bool expected = limit < 4 || limit > INT32_MAX;
            check(q.copied == expected,
                  "outer edge limit uses original signed 32-bit comparison including zero");
            if (expected) {
                const auto sign =
                    limit <= INT32_MAX ? std::int64_t(limit) : std::int64_t(limit) - 0x100000000LL;
                check(q.report["steps"][1]["max_edges"] == static_cast<std::size_t>(sign),
                      "negative edge settings are sign-extended into triangulator size_t");
            }
            const auto expected_size =
                limit == 4 || limit == 5 || limit == UINT32_MAX - 1 ? 5u : 8u;
            check(q.complete &&
                      q.output.data.indices.indices[point_channel].size() == expected_size,
                  "edge-limit consumer preserves original clamp and unsigned limit-plus-one wrap");
        }
        auto s = source;
        s.data.indices.indices[point_channel] = {1, 2, 3, 0};
        s.data.num_per_face = 4;
        s.index_rows.fill(4);
        o.max_edges_per_face = 3;
        const auto fixed = prepare(s, o);
        check(!fixed.copied && fixed.report["max_facet_size"] == 3,
              "outer max query counts actual corners rather than fixed stride");
    }
    {
        auto o = options;
        o.normals_required = false;
        o.max_edges_per_face = 4;
        for (auto mode : {-1, 0, 1, 2, 99}) {
            o.parameter_mode = mode;
            const auto q = prepare(source, o);
            const auto selector = mode == 0 ? 2 : mode == 1 ? 3 : 1;
            check(q.complete && q.report["steps"][1]["step"] == "parameters" &&
                      q.report["steps"][1]["result"]["coordinate_selector"] == selector,
                  "outer texture parameter mode maps to original projection selectors");
            Point2 low{1e9, 1e9}, high{-1e9, -1e9};
            for (auto p : q.output.data.coordinates.parameters)
                for (unsigned i = 0; i < 2; ++i) {
                    low[i] = std::min(low[i], p[i]);
                    high[i] = std::max(high[i], p[i]);
                }
            const auto small = std::min(high[0] - low[0], high[1] - low[1]);
            const auto large = std::max(high[0] - low[0], high[1] - low[1]);
            const auto wanted_small = mode == 0 ? 1.0 : mode == 1 ? .5 : 2.0;
            const auto wanted_large = mode == 0 || mode == 1 ? 1.0 : 4.0;
            check(std::abs(small - wanted_small) < 1e-14 && std::abs(large - wanted_large) < 1e-14,
                  "rectangle UV extents demonstrate independent-axis, uniform and unscaled modes");
            check(!q.output.pool_active[native_face_data_pool] &&
                      q.output.data.indices.active[face_channel],
                  "face preparation without later activation preserves native pool/index flag "
                  "distinction");
        }
        auto existing = source;
        existing.data.coordinates.normals = {{0, 0, 9}};
        existing.data.coordinates.parameters = {{.9, .8}};
        existing.data.face_data.resize(1);
        existing.data.edge_chains.push_back({24, {7, 8}, {1, 2}});
        o = options;
        o.max_edges_per_face = 8;
        o.edge_chains_required = true;
        o.hide_smooth_edges = true;
        const auto q = prepare(existing, o);
        check(!q.copied && q.output.data.coordinates.normals == existing.data.coordinates.normals &&
                  q.output.data.coordinates.parameters == existing.data.coordinates.parameters,
              "nonempty original pools suppress synthesis even without corresponding index arrays");
        existing.data.coordinates.parameters.clear();
        o.parameters_required = false;
        check(!prepare(existing, o).copied,
              "parameters-required setting also controls face-data preparation");
        existing.data.face_data.clear();
        check(!prepare(existing, o).copied,
              "missing face records alone do not trigger when UV not requested");
    }
    {
        auto o = options;
        o.parameters_required = false;
        o.max_edges_per_face = 3;
        o.hide_smooth_edges = true;
        auto s = source;
        s.data.indices.indices[point_channel] = {1, 2, 3, 0, 1, 3, 4, 0};
        const auto hidden = prepare(s, o);
        check(hidden.complete && hidden.output.data.indices.indices[point_channel][2] < 0 &&
                  hidden.output.data.indices.indices[point_channel][4] < 0,
              "smooth normal stage hides coplanar shared edges before any later processing");
        s.data.coordinates.parameter_scope = 8;
        s.data.coordinates.normalize_normals = false;
        s.data.coordinates.reverse_normals = true;
        const auto q = prepare(s, o);
        check(q.output.data.coordinates.parameter_scope == 8 &&
                  !q.output.data.coordinates.normalize_normals &&
                  q.output.data.coordinates.reverse_normals,
              "query-copy does not discard caller-owned builder coordinate controls");
    }
    {
        auto o = options;
        o.edge_chains_required = true;
        const auto empty = prepare(make_native_polyface_mesh(), o);
        check(empty.copied && !empty.complete &&
                  steps(empty) == std::vector<std::string>{"query_copy", "approximate_normals",
                                                           "parameters", "face_data",
                                                           "edge_chains"} &&
                  empty.report["steps"][1]["result"]["native_succeeded"] == false &&
                  empty.report["steps"][2]["result"]["native_succeeded"] == false,
              "failed early native preparation does not suppress subsequent original steps");
        const auto assembled = assemble({make_native_polyface_mesh()}, o);
        check(assembled.native_succeeded && !assembled.complete,
              "final matched native success does not hide failed earlier preparation");
        auto s = source;
        s.face_material_ids = {0xfedcba9876543210ULL};
        s.face_smooth_groups = {42};
        const auto metadata = assemble({s}, options);
        check(metadata.native_succeeded && !metadata.complete &&
                  metadata.report["untransferred_metadata_sources"] == Json::array({0}) &&
                  metadata.sources[0].output.face_material_ids == s.face_material_ids,
              "unmapped extensions remain available with source preparation and prevent full "
              "assembly completeness");
        s = source;
        s.texture_id = 77;
        const auto texture = assemble({s}, options);
        check(!texture.complete && !texture.sources[0].complete &&
                  texture.sources[0].report["steps"][0]["result"]["source_texture_id_differs"] ==
                      true,
              "native omitted texture identity remains an explicit incomplete-stage diagnostic");
    }
    {
        auto o = options;
        o.normals_required = false;
        o.parameters_required = false;
        o.max_edges_per_face = 8;
        auto s = source;
        s.data.indices.indices[point_channel] = {1, 999, 3, 0};
        const auto q = prepare(s, o);
        check(!q.complete && !q.copied && q.report["max_facet_size"] == 0,
              "source point-only query failure propagates despite a direct matched route");
        auto a = source, b = source;
        a.data.num_per_face = 5;
        const auto mismatch = assemble({a, b}, o);
        check(!mismatch.native_succeeded && !mismatch.complete && mismatch.sources.size() == 2,
              "prepared multiple sources retain original matched layout rejection");
        const auto many = assemble({source, source}, options);
        check(many.complete && many.assembled.coordinates.points.size() == 4 &&
                  many.assembled.indices.indices[point_channel].size() == 16,
              "shared builder applies only its native coordinate map across prepared sources");
        s.mesh_style = 5;
        rejects([&] { prepare(s, o); });
        TubeBudget budget;
        budget.max_control_points = 4;
        rejects([&] { prepare_native_polyface_for_builder(source, o, budget); });
        budget = TubeBudget{};
        budget.max_work = 1;
        rejects([&] { prepare_native_polyface_for_builder(source, o, budget); });
        budget = TubeBudget{};
        budget.max_control_points = 15;
        rejects([&] { assemble_native_prepared_polyfaces({source, source}, o, {}, budget); });
    }
    auto task = [source, options, assemble] {
        return assemble({source}, options).assembled.indices.indices;
    };
    auto a = std::async(std::launch::async, task), b = std::async(std::launch::async, task);
    check(a.get() == b.get(),
          "outer pipeline is deterministic across independent concurrent calls");
    return checks;
}
