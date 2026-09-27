#include "native_polyface_copy.hpp"
#include "native_bezier.hpp"

namespace p3d::swept_detail {
NativePolyfaceMesh make_native_polyface_mesh(std::uint32_t num_per_face, std::uint32_t mesh_style) {
    NativePolyfaceMesh out;
    out.data.num_per_face = num_per_face;
    out.data.two_sided = true;
    out.mesh_style = mesh_style;
    out.num_per_row = 0;
    out.pool_rows.fill(1);
    out.index_rows.fill(std::max(std::uint32_t(1), num_per_face));
    out.edge_chain_rows = 1;
    out.pool_tags[native_point_pool] = {3, 2, 0, 1};
    out.pool_tags[native_normal_pool] = {3, 4, 0, 3};
    out.pool_tags[native_parameter_pool] = {2, 6, 0, 5};
    out.pool_tags[native_double_color_pool] = {3, 8, 0, 7};
    out.pool_tags[native_float_color_pool] = {3, 9, 0, 7};
    out.pool_tags[native_integer_color_pool] = {1, 10, 0, 9};
    out.pool_tags[native_color_table_pool] = {1, 12, 0, 11};
    const std::array<std::uint32_t, polyface_channel_count> tags{1, 5, 3, 7, 23};
    for (std::size_t c = 0; c < tags.size(); ++c)
        out.index_tags[c] = {1, tags[c], UINT32_MAX, 0};
    out.pool_active[native_point_pool] = true;
    out.data.indices.active[point_channel] = mesh_style == 1;
    return out;
}
namespace {
struct CopyContext {
    TubeBudget &budget;
    curve_detail::BezierWork work;
    std::size_t storage = 0;
    explicit CopyContext(TubeBudget &b) : budget(b), work{b.work, b.max_work} {}
    void count(std::size_t n) {
        require(n <= budget.max_control_points - storage, "native query-copy storage budget");
        storage += n;
        work.charge(n);
    }
    void mesh(const NativePolyfaceMesh &m) {
        count(m.data.coordinates.points.size());
        count(m.data.coordinates.normals.size());
        count(m.data.coordinates.parameters.size());
        count(m.data.face_data.size());
        count(m.data.edge_chains.size());
        for (const auto &e : m.data.edge_chains) {
            count(e.topology_ids.size());
            count(e.point_indices.size());
        }
        for (const auto &v : m.data.indices.indices)
            count(v.size());
        count(m.double_colors.size());
        count(m.float_colors.size());
        count(m.integer_colors.size());
        count(m.color_table.size());
        count(m.face_uv_points.size());
        count(m.face_material_ids.size());
        count(m.face_smooth_groups.size());
        count(m.illumination_name.size());
    }
};
bool equal_tags(const NativePolyfaceVectorTags &a, const NativePolyfaceVectorTags &b) {
    return a.num_per_struct == b.num_per_struct && a.tag == b.tag &&
           a.index_family == b.index_family && a.indexed_by == b.indexed_by;
}
} // namespace
NativePolyfaceCopy copy_native_polyface_query(const NativePolyfaceMesh &source,
                                              const NativePolyfaceMesh &destination,
                                              TubeBudget &budget) {
    CopyContext context(budget);
    context.mesh(source);
    context.mesh(destination);
    // Snapshot semantics: neither caller-owned input is modified, including
    // when both references denote the same object.
    NativePolyfaceCopy out;
    out.output = destination;
    auto &d = out.output;
    const auto &s = source;
    d.data.num_per_face = s.data.num_per_face;
    d.data.two_sided = s.data.two_sided;
    d.mesh_style = s.mesh_style;
    d.num_per_row = s.num_per_row;
    require(s.face_material_ids.size() <= INT32_MAX && s.face_uv_points.size() <= INT32_MAX &&
                s.face_smooth_groups.size() <= INT32_MAX,
            "native query-copy extension count exceeds signed query range");
    d.face_material_ids = s.face_material_ids;
    d.face_uv_points = s.face_uv_points;
    d.face_smooth_groups = s.face_smooth_groups;
    std::size_t omitted = 0;
    Json copies = Json::array();
    auto copy = [&](auto &dest, const auto &src, std::size_t requested, bool &active,
                    const char *name) {
        const auto count = src.empty() ? 0 : requested;
        require(count <= src.size(), "native query-copy requested count exceeds source array");
        context.work.charge(count);
        dest.assign(src.begin(), src.begin() + count);
        if (count)
            active = true;
        const auto dropped = src.size() - count;
        omitted += dropped;
        copies.push_back({{"channel", name},
                          {"source_count", src.size()},
                          {"query_count", requested},
                          {"copied_count", count},
                          {"omitted_values", dropped},
                          {"active", active}});
    };
    copy(d.data.coordinates.points, s.data.coordinates.points, s.data.coordinates.points.size(),
         d.pool_active[native_point_pool], "points");
    copy(d.data.coordinates.normals, s.data.coordinates.normals, s.data.coordinates.normals.size(),
         d.pool_active[native_normal_pool], "normals");
    copy(d.data.coordinates.parameters, s.data.coordinates.parameters,
         s.data.coordinates.parameters.size(), d.pool_active[native_parameter_pool], "parameters");
    copy(d.data.face_data, s.data.face_data, s.data.face_data.size(),
         d.pool_active[native_face_data_pool], "face_data");
    std::size_t color_count = 0;
    if (s.pool_active[native_float_color_pool])
        color_count = s.float_colors.size();
    else if (s.pool_active[native_double_color_pool])
        color_count = s.double_colors.size();
    else if (s.pool_active[native_integer_color_pool])
        color_count = s.integer_colors.size();
    else if (s.pool_active[native_color_table_pool])
        color_count = s.color_table.size();
    // Query-copy requests the common ColorCount independently for all four
    // non-null representations; it does not use the visitor's pointer priority.
    copy(d.float_colors, s.float_colors, color_count, d.pool_active[native_float_color_pool],
         "float_colors");
    copy(d.double_colors, s.double_colors, color_count, d.pool_active[native_double_color_pool],
         "double_colors");
    copy(d.color_table, s.color_table, color_count, d.pool_active[native_color_table_pool],
         "color_table");
    copy(d.integer_colors, s.integer_colors, color_count, d.pool_active[native_integer_color_pool],
         "integer_colors");
    const auto index_count = s.data.indices.indices[point_channel].size();
    const std::array<const char *, polyface_channel_count> names{
        "point_indices", "parameter_indices", "normal_indices", "color_indices", "face_indices"};
    for (auto c : {point_channel, normal_channel, parameter_channel, color_channel, face_channel})
        copy(d.data.indices.indices[c], s.data.indices.indices[c], index_count,
             d.data.indices.active[c], names[c]);
    copy(d.data.edge_chains, s.data.edge_chains, s.data.edge_chains.size(), d.edge_chain_active,
         "edge_chains");
    d.illumination_name.clear();
    std::size_t discarded_name_units = 0;
    if (!s.illumination_name.empty()) {
        const auto first_zero = s.illumination_name.find(char16_t(0));
        const auto n = first_zero == std::u16string::npos ? s.illumination_name.size() : first_zero;
        context.work.charge(n);
        d.illumination_name.assign(s.illumination_name.data(), n);
        // Native appends the terminating code unit before testing it.
        context.count(1);
        d.illumination_name.push_back(0);
        if (first_zero != std::u16string::npos)
            discarded_name_units = s.illumination_name.size() - first_zero - 1;
    }
    std::size_t different_rows = 0, different_tags = 0;
    for (std::size_t i = 0; i < s.pool_rows.size(); ++i) {
        different_rows += s.pool_rows[i] != d.pool_rows[i];
        different_tags += !equal_tags(s.pool_tags[i], d.pool_tags[i]);
    }
    for (std::size_t i = 0; i < s.index_rows.size(); ++i) {
        different_rows += s.index_rows[i] != d.index_rows[i];
        different_tags += !equal_tags(s.index_tags[i], d.index_tags[i]);
    }
    different_rows += s.edge_chain_rows != d.edge_chain_rows;
    different_tags += !equal_tags(s.edge_chain_tags, d.edge_chain_tags);
    const bool texture_differs = s.texture_id != d.texture_id;
    // The source remains available to callers even when native preparation
    // omits information. No fabricated transfer repairs the original operation.
    out.complete =
        !omitted && !discarded_name_units && !texture_differs && !different_rows && !different_tags;
    out.report = {{"scope", "native_polyface_query_copy"},
                  {"native_return_type", "void"},
                  {"complete", out.complete},
                  {"references_checked", false},
                  {"copies", std::move(copies)},
                  {"omitted_source_values", omitted},
                  {"color_query_count", color_count},
                  {"texture_id_retained_from_destination", true},
                  {"source_texture_id_differs", texture_differs},
                  {"destination_row_widths_retained", true},
                  {"different_source_row_widths", different_rows},
                  {"destination_vector_tags_retained", true},
                  {"different_source_vector_tags", different_tags},
                  {"discarded_name_code_units", discarded_name_units}};
    return out;
}
} // namespace p3d::swept_detail
