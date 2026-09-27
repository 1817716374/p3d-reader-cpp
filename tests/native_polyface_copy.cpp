#include "native_polyface_copy.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_polyface_copy_tests() {
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
        check(caught, "query-copy rejects native out-of-bounds reads and exhausted budgets");
    };
    auto copy = [](const NativePolyfaceMesh &s, const NativePolyfaceMesh &d) {
        TubeBudget b;
        return copy_native_polyface_query(s, d, b);
    };
    const auto fresh = make_native_polyface_mesh();
    check(fresh.data.two_sided && fresh.mesh_style == 1 && fresh.num_per_row == 0 &&
              fresh.data.num_per_face == 0 && fresh.pool_active[native_point_pool] &&
              fresh.data.indices.active[point_channel] && !fresh.edge_chain_active &&
              fresh.edge_chain_rows == 1,
          "fresh native handle gets constructor plus clear-tags defaults");
    for (auto n : {0u, 1u, 3u, UINT32_MAX})
        for (auto style : {1u, 3u, 5u, 17u}) {
            const auto m = make_native_polyface_mesh(n, style);
            check(
                m.data.num_per_face == n && m.mesh_style == style &&
                    m.data.indices.active[point_channel] == (style == 1),
                "factory retains original global count/style and style-gated point index activity");
            for (std::size_t k = 0; k < native_polyface_pool_count; ++k)
                check(m.pool_rows[k] == 1 && m.pool_active[k] == (k == native_point_pool),
                      "factory initializes independent pool row widths and activity");
            for (std::size_t k = 0; k < polyface_channel_count; ++k)
                check(m.index_rows[k] == std::max(1u, n) && m.index_tags[k].num_per_struct == 1 &&
                          m.index_tags[k].index_family == UINT32_MAX &&
                          m.index_tags[k].indexed_by == 0,
                      "factory keeps unsigned independent index tags and row widths");
        }
    check(fresh.pool_tags[native_point_pool].tag == 2 &&
              fresh.pool_tags[native_point_pool].indexed_by == 1 &&
              fresh.pool_tags[native_parameter_pool].num_per_struct == 2 &&
              fresh.pool_tags[native_normal_pool].tag == 4 &&
              fresh.pool_tags[native_float_color_pool].num_per_struct == 3 &&
              fresh.pool_tags[native_float_color_pool].tag == 9 &&
              fresh.pool_tags[native_float_color_pool].indexed_by == 7 &&
              fresh.pool_tags[native_color_table_pool].tag == 12 &&
              fresh.pool_tags[native_color_table_pool].indexed_by == 11 &&
              fresh.pool_tags[native_face_data_pool].tag == 0 &&
              fresh.index_tags[face_channel].tag == 23,
          "native factory preserves nonuniform vector tag identities");
    auto source = fresh;
    source.data.coordinates.points = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    source.data.coordinates.normals = {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}};
    source.data.coordinates.parameters = {{.1, .2}, {.3, .4}, {.5, .6}};
    source.double_colors = {{.1, .2, .3}, {.4, .5, .6}, {.7, .8, .9}};
    source.float_colors = {{.1f, .2f, .3f}, {.4f, .5f, .6f}, {.7f, .8f, .9f}};
    source.integer_colors = {0xff000000, 0xff00ff00, 0xffffffff};
    source.color_table = {91, 72, 53};
    source.pool_active[native_integer_color_pool] = true;
    source.data.indices.indices[point_channel] = {1, -2, 3, 0};
    source.data.indices.indices[normal_channel] = {3, 2, 1, 0};
    source.data.indices.indices[parameter_channel] = {2, 3, 1, 0};
    source.data.indices.indices[color_channel] = {3, 1, 2, 0};
    source.data.indices.indices[face_channel] = {1, 1, 1, 0};
    source.data.face_data.resize(1);
    source.data.face_data[0].source_index = 0xfedcba9876543210ULL;
    source.data.edge_chains.push_back({24, {0, 1}, {1, 2}});
    source.face_uv_points = {{.9, .8}, {.7, .6}};
    source.face_material_ids = {0x12345678abcdef09ULL, UINT64_MAX, 0};
    source.face_smooth_groups = {-1, INT32_MIN, INT32_MAX, 0};
    source.illumination_name = u"照明\U0001f4a1";
    source.num_per_row = 37;
    source.data.two_sided = false;
    auto target = fresh;
    target.data.coordinates.parameter_scope = 42;
    target.data.coordinates.normalize_normals = false;
    target.data.coordinates.reverse_normals = true;
    const auto r = copy(source, target);
    check(r.complete && r.output.data.coordinates.points == source.data.coordinates.points &&
              r.output.data.coordinates.normals == source.data.coordinates.normals &&
              r.output.data.coordinates.parameters == source.data.coordinates.parameters &&
              r.output.data.indices.indices == source.data.indices.indices,
          "copy preserves independent actual query arrays");
    check(r.output.double_colors == source.double_colors &&
              r.output.float_colors == source.float_colors &&
              r.output.integer_colors == source.integer_colors &&
              r.output.color_table == source.color_table,
          "copy transfers every non-null color representation using the common query count");
    check(r.output.face_uv_points == source.face_uv_points &&
              r.output.face_material_ids == source.face_material_ids &&
              r.output.face_smooth_groups == source.face_smooth_groups &&
              r.output.data.face_data[0].source_index == source.data.face_data[0].source_index &&
              r.output.data.edge_chains[0].topology_ids == source.data.edge_chains[0].topology_ids,
          "independent extension arrays, full-width material IDs and edge topology are retained");
    check(r.output.illumination_name == source.illumination_name + char16_t(0) &&
              r.output.illumination_name.size() == source.illumination_name.size() + 1,
          "native copy includes terminating zero in logical UTF-16 string length");
    check(
        !r.output.data.two_sided && r.output.num_per_row == 37 &&
            r.output.data.coordinates.parameter_scope == 42 &&
            !r.output.data.coordinates.normalize_normals &&
            r.output.data.coordinates.reverse_normals,
        "native global settings transfer while builder placement context stays destination-owned");
    for (bool a : r.output.pool_active)
        check(a, "nonempty copied pools activate even if source flag was false");
    for (bool a : r.output.data.indices.active)
        check(a, "nonempty copied indices activate even if source flag was false");
    check(r.output.edge_chain_active && !source.edge_chain_active &&
              target.data.coordinates.points.empty(),
          "inputs remain unchanged and copied edge chain activates destination");
    {
        auto empty = copy(fresh, r.output);
        check(empty.complete && empty.output.data.coordinates.points.empty() &&
                  empty.output.data.face_data.empty() && empty.output.float_colors.empty() &&
                  empty.output.face_uv_points.empty() && empty.output.face_material_ids.empty() &&
                  empty.output.face_smooth_groups.empty() && empty.output.illumination_name.empty(),
              "empty source clears all copied destination buffers");
        for (bool a : empty.output.pool_active)
            check(a, "empty copy leaves old pool activity intact");
        for (bool a : empty.output.data.indices.active)
            check(a, "empty copy leaves old index activity intact");
        check(empty.output.edge_chain_active, "empty copy leaves old edge activity intact");
        auto s = source;
        s.texture_id = 0x123456789abcdef0ULL;
        auto d = fresh;
        d.texture_id = 42;
        const auto q = copy(s, d);
        check(!q.complete && q.output.texture_id == 42 &&
                  q.report["source_texture_id_differs"] == true,
              "query-copy never assigns source texture ID and reports the difference");
        s.illumination_name = std::u16string{u'a', 0, u'b', u'c'};
        auto text = copy(s, d);
        check(text.output.illumination_name == std::u16string{u'a', 0} &&
                  text.report["discarded_name_code_units"] == 2,
              "embedded string zero stops native copy and diagnoses discarded tail");
        s = fresh;
        s.illumination_name = std::u16string{0};
        text = copy(s, fresh);
        check(text.complete && text.output.illumination_name == s.illumination_name,
              "nonempty string containing zero stays distinct from null query pointer");
    }
    {
        auto d = fresh;
        d.pool_rows[native_point_pool] = 17;
        d.index_rows[parameter_channel] = 21;
        d.pool_tags[native_normal_pool].tag = 999;
        d.edge_chain_tags.indexed_by = 777;
        d.edge_chain_rows = 33;
        auto q = copy(source, d);
        check(!q.complete && q.output.pool_rows[native_point_pool] == 17 &&
                  q.output.index_rows[parameter_channel] == 21 &&
                  q.output.pool_tags[native_normal_pool].tag == 999 &&
                  q.output.edge_chain_tags.indexed_by == 777 && q.output.edge_chain_rows == 33 &&
                  q.report["different_source_row_widths"] == 3 &&
                  q.report["different_source_vector_tags"] == 2,
              "copy preserves destination row metadata and tags and diagnoses source mismatch");
        auto s = source;
        s.data.indices.indices[face_channel].insert(s.data.indices.indices[face_channel].end(),
                                                    {123, 456});
        q = copy(s, fresh);
        check(!q.complete &&
                  q.output.data.indices.indices[face_channel] ==
                      source.data.indices.indices[face_channel] &&
                  q.report["omitted_source_values"] == 2,
              "face indices also use point index count and discard longer tails");
        s = source;
        s.data.indices.indices[face_channel] = {1};
        rejects([&] { copy(s, fresh); });
        s = source;
        s.data.indices.indices[normal_channel] = {1, 0};
        rejects([&] { copy(s, fresh); });
        s = source;
        s.data.indices.indices[normal_channel].clear();
        q = copy(s, fresh);
        check(q.complete && q.output.data.indices.indices[normal_channel].empty() &&
                  !q.output.data.indices.active[normal_channel],
              "copy does not replicate missing normal indices");
    }
    for (unsigned present = 0; present < 16; ++present)
        for (unsigned active = 0; active < 16; ++active) {
            auto s = fresh;
            if (present & 1)
                s.double_colors.resize(2);
            if (present & 2)
                s.float_colors.resize(2);
            if (present & 4)
                s.integer_colors.resize(2);
            if (present & 8)
                s.color_table.resize(2);
            for (unsigned k = 0; k < 4; ++k)
                s.pool_active[native_double_color_pool + k] = (active & (1u << k)) != 0;
            unsigned selected = 4;
            for (unsigned k : {1u, 0u, 2u, 3u})
                if (active & (1u << k)) {
                    selected = k;
                    break;
                }
            const bool query_nonzero = selected != 4 && (present & (1u << selected));
            const auto q = copy(s, fresh);
            const std::array<std::size_t, 4> actual{
                q.output.double_colors.size(), q.output.float_colors.size(),
                q.output.integer_colors.size(), q.output.color_table.size()};
            for (unsigned k = 0; k < 4; ++k) {
                const auto wanted = query_nonzero && (present & (1u << k)) ? 2u : 0u;
                check(
                    actual[k] == wanted &&
                        q.output.pool_active[native_double_color_pool + k] == (wanted != 0),
                    "common active color count controls all independently present source buffers");
            }
            check(q.complete == (present == 0 || query_nonzero),
                  "omitted color values prevent complete copy");
        }
    {
        auto s = source;
        s.float_colors.resize(1);
        s.pool_active[native_float_color_pool] = true;
        auto q = copy(s, fresh);
        check(!q.complete && q.output.double_colors.size() == 1 &&
                  q.output.color_table.size() == 1 && q.output.integer_colors.size() == 1 &&
                  q.report["omitted_source_values"] == 6,
              "short active float count truncates other color representations by native rule");
        s = source;
        s.double_colors.resize(1);
        rejects([&] { copy(s, fresh); });
        const auto self = copy(source, source);
        check(self.complete &&
                  self.output.data.coordinates.points == source.data.coordinates.points &&
                  source.illumination_name.back() != 0,
              "aliased input references use immutable snapshot semantics");
        TubeBudget b;
        b.max_control_points = 2;
        rejects([&] { copy_native_polyface_query(source, fresh, b); });
        b = TubeBudget{};
        b.max_work = 1;
        rejects([&] { copy_native_polyface_query(source, fresh, b); });
    }
    {
        TubeBudget b;
        const auto t = triangulate_native_polyface_mesh(r.output, b);
        check(t.complete && t.output.face_material_ids == source.face_material_ids &&
                  t.output.face_uv_points == source.face_uv_points &&
                  t.output.face_smooth_groups == source.face_smooth_groups &&
                  t.output.illumination_name == r.output.illumination_name &&
                  t.output.pool_tags[native_float_color_pool].tag == 9,
              "query-copy to visitor and triangulation retains independent metadata arrays");
        auto s = fresh;
        s.face_uv_points.resize(30);
        b = TubeBudget{};
        b.max_control_points = 20;
        rejects([&] { visit_native_polyface(s, b); });
    }
    auto task = [source, fresh, copy] { return copy(source, fresh).output.face_material_ids; };
    auto a = std::async(std::launch::async, task), b = std::async(std::launch::async, task);
    check(a.get() == b.get(), "native query-copy has no shared mutable state");
    return checks;
}
