#include "internal.hpp"

namespace p3d {
namespace {
struct GeometryBytes {
    const Bytes &entry;
    std::size_t start;
    std::size_t size;
    void range(std::size_t offset, std::size_t count) const {
        require(offset <= size && count <= size - offset, "bgfb_geometry_field_unavailable");
    }
    template <class T> T at(std::size_t offset) const {
        range(offset, sizeof(T));
        T value;
        std::memcpy(&value, entry.data() + start + offset, sizeof(T));
        return value;
    }
    std::size_t indirect(std::size_t offset) const {
        const auto relative = at<std::uint32_t>(offset);
        require(relative >= 4, "bgfb_geometry_relative_pointer_unavailable");
        range(offset, relative);
        const auto target = offset + relative;
        range(target, 4);
        return target;
    }
    std::optional<std::size_t> field(std::size_t table, unsigned index, std::size_t width) const {
        const auto vt64 = static_cast<std::int64_t>(table) - at<std::int32_t>(table);
        require(vt64 >= 0 && std::uint64_t(vt64) <= size, "bgfb_geometry_vtable_unavailable");
        const auto vt = static_cast<std::size_t>(vt64);
        const auto bytes = at<std::uint16_t>(vt), object_size = at<std::uint16_t>(vt + 2);
        require(bytes >= 4 && bytes % 2 == 0 && object_size >= 4,
                "bgfb_geometry_table_extent_unavailable");
        range(vt, bytes);
        range(table, object_size);
        const auto slot = 4u + 2u * index;
        const auto offset = slot < bytes ? at<std::uint16_t>(vt + slot) : 0;
        if (!offset)
            return std::nullopt;
        require(offset >= 4 && offset <= object_size &&
                    width <= static_cast<std::size_t>(object_size - offset),
                "bgfb_geometry_field_extent_unavailable");
        range(table + offset, width);
        return table + offset;
    }
};

// These readers copy their inputs without geometric validity checks. This is
// evidence for a non-null geometry object, not for its topology, displayability,
// material footer or eventual insertion in the owning element's graphics.
Json construction(const GeometryBytes &b, std::size_t root, unsigned tag) {
    const auto pointer = b.field(root, 1, 4);
    require(pointer.has_value(), "bgfb_geometry_union_data_unavailable");
    const auto table = b.indirect(*pointer);
    Json out = {{"allocation_assumption", "successful"},
                {"geometry_validity", "not_checked_by_reader"}};
    if (tag >= 6 && tag <= 9) {
        const std::size_t sizes[] = {120, 120, 104, 136};
        const char *names[] = {"DgnCone", "DgnSphere", "DgnTorusPipe", "DgnBox"};
        const auto size = sizes[tag - 6];
        const auto detail = b.field(table, 0, size);
        require(detail.has_value(), "native_solid_reader_requires_detail_pointer");
        out.update({{"geometry_type", names[tag - 6]},
                    {"operation", "copy_fixed_detail"},
                    {"detail_offset", *detail},
                    {"detail_bytes", size}});
        return out;
    }
    require(tag == 13, "native_geometry_construction_not_supported");
    auto scalar = [&](unsigned index) {
        const auto p = b.field(table, index, 4);
        return p ? b.at<std::int32_t>(*p) : 0;
    };
    const auto num_per_face = scalar(10), num_per_row = scalar(11), mesh_style = scalar(12);
    const auto sided = b.field(table, 13, 1);
    out.update({{"geometry_type", "Polyface"},
                {"operation", "copy_native_channels"},
                {"num_per_face", num_per_face},
                {"num_per_row", num_per_row},
                {"mesh_style", mesh_style},
                {"source_two_sided", sided ? b.at<std::uint8_t>(*sided) != 0 : false},
                // Layout initialization follows the source flag assignment.
                {"two_sided", true},
                {"index_block_size", std::max(std::uint32_t(1), std::uint32_t(num_per_face))},
                {"channels", Json::object()},
                {"unread_fields",
                 {"faceIndex", "faceData", "auxData", "expectedClosure", "taggedNumericData",
                  "edgeMateIndex"}}});
    const char *names[] = {"point",      "param",      "normal",      "doubleColor", "intColor",
                           "pointIndex", "paramIndex", "normalIndex", "colorIndex",  "colorTable"};
    const unsigned components[] = {3, 2, 3, 3, 1, 1, 1, 1, 1, 1};
    for (unsigned i = 0; i < 10; ++i) {
        const auto p = b.field(table, i, 4);
        Json channel = {{"source_present", p.has_value()},
                        {"active", p.has_value() || i == 0 || (i == 5 && mesh_style == 1)},
                        {"components", components[i]},
                        {"source_scalar_count", 0},
                        {"element_count", 0},
                        {"ignored_tail_scalars", 0}};
        if (p) {
            const auto vector = b.indirect(*p);
            const auto count = b.at<std::uint32_t>(vector);
            const auto elements = count / components[i];
            const auto copied = std::uint64_t(elements) * components[i] * (i < 4 ? 8 : 4);
            require(copied <= b.size, "bgfb_native_channel_extent_unavailable");
            b.range(vector + 4, static_cast<std::size_t>(copied));
            channel.update({{"source_scalar_count", count},
                            {"element_count", elements},
                            {"ignored_tail_scalars", count % components[i]},
                            {"data_offset", vector + 4},
                            {"copied_bytes", copied}});
        }
        out["channels"][names[i]] = std::move(channel);
    }
    return out;
}
Json native_input(const Bytes &entry, bool model_has_project) {
    const std::size_t geometry_start = model_has_project ? 36 : 32;
    Json out = {{"status", "not_evaluated"},
                {"model_has_project", model_has_project},
                {"geometry_offset", geometry_start}};
    if (entry.size() < geometry_start) {
        out.update({{"status", "rejected"}, {"reason", "truncated_entry_header"}});
        return out;
    }
    try {
        Reader r(entry, 4);
        const auto type = r.i32();
        out["entry_geometry_type"] = type;
        static const std::map<int, const char *> readers = {
            {1, "curve_primitive_bgfb"}, {2, "curve_vector_bgfb"},
            {3, "polyface_bgfb"},        {4, "bspline_curve_via_bgfb_primitive"},
            {5, "bspline_surface_bgfb"}, {6, "solid_primitive_bgfb"},
            {7, "text_entity_bytes"},    {10, "csg_tree_bytes"}};
        r.p = geometry_start - 8;
        const auto count = r.u64();
        // The native length guard precedes the type dispatch. Empty or oversized
        // ranges return the existing Entry without reading geometry OR its footer.
        // The native unsigned addition can wrap; do not classify that path here.
        require(count <= UINT64_MAX - geometry_start, "native_geometry_range_addition_overflow");
        if (!count || count > r.left()) {
            out.update(
                {{"status", "geometry_not_read"},
                 {"reader", nullptr},
                 {"geometry_pointer", "null"},
                 {"material_footer", "not_read"},
                 {"reason", count ? "geometry_byte_range_unavailable" : "empty_geometry_bytes"}});
            return out;
        }
        if (!readers.count(type)) {
            out.update({{"status", "geometry_not_read"},
                        {"reader", nullptr},
                        {"geometry_pointer", "null"}});
            return out;
        }
        out["reader"] = readers.at(type);
        // Text and the CSG archive have separate readers. A decoded BGFB packet
        // is not evidence that their native byte reader succeeds.
        if (type == 7 || type == 10)
            return out;
        if (type == 4)
            out["post_read_operation"] = "get_bspline_curve_pointer";
        auto reject = [&](const char *reason) {
            out.update({{"status", "rejected"}, {"reason", reason}, {"geometry_pointer", "null"}});
        };
        const GeometryBytes b{entry, geometry_start, static_cast<std::size_t>(count)};
        require(b.size >= 12, "bgfb_root_header_unavailable");
        require(std::memcmp(entry.data() + geometry_start, "bg0001fb", 8) == 0,
                "geometry_signature_requires_native_reader");
        const auto root64 = std::uint64_t(8) + b.at<std::uint32_t>(8);
        require(root64 <= b.size && b.size - root64 >= 4, "bgfb_root_table_unavailable");
        const auto root = static_cast<std::size_t>(root64);
        const auto vt64 = static_cast<std::int64_t>(root) - b.at<std::int32_t>(root);
        require(vt64 >= 0 && std::uint64_t(vt64) <= b.size && b.size - std::uint64_t(vt64) >= 4,
                "bgfb_root_vtable_unavailable");
        const auto vt = static_cast<std::size_t>(vt64);
        const auto size = b.at<std::uint16_t>(vt);
        require(size >= 4 && size % 2 == 0 && size <= b.size - vt,
                "bgfb_root_vtable_extent_unavailable");
        // The native bytesTo* wrapper requires BOTH root union fields to be present,
        // before it dispatches the tag. Omitted data is not a default geometry.
        if (size < 8 || !b.at<std::uint16_t>(vt + 4) || !b.at<std::uint16_t>(vt + 6)) {
            reject("missing_bgfb_union_field");
            return out;
        }
        const auto offset = b.at<std::uint16_t>(vt + 4);
        require(offset < b.size - root, "bgfb_union_tag_unavailable");
        const auto tag = b.at<std::uint8_t>(root + offset);
        out["bgfb_geometry_type"] = tag;
        bool selected = false;
        switch (type) {
        case 1:
        case 4:
            selected = tag == 1 || tag == 2 || tag == 3 || tag == 4 || tag == 16 || tag == 17 ||
                       tag == 18 || tag == 19;
            break;
        case 2:
            selected = tag == 5;
            break;
        case 3:
            selected = tag == 13;
            break;
        case 5:
            selected = tag == 14;
            break;
        case 6:
            selected = (tag >= 6 && tag <= 12) || tag == 20 || tag == 21;
            break;
        }
        out["root_dispatch"] = selected ? "selected" : "rejected";
        if (!selected) {
            reject("bgfb_type_not_accepted_by_entry_reader");
            return out;
        }
        if ((type == 6 && tag >= 6 && tag <= 9) || type == 3) {
            out["construction"] = construction(b, root, tag);
            out["status"] = "geometry_constructed";
            out["geometry_pointer"] = "non_null";
        }
        // Selection is not construction: child validity, native geometry creation,
        // color/model context and the material footer still need their own checks.
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace

Json graphics_entry_native_input(const Bytes &entry) {
    Json out = {{"scope", "entry_geometry_reader_before_material_footer"},
                {"project_context", "not_provided"},
                {"status", "not_evaluated"},
                {"with_project", native_input(entry, true)},
                {"without_project", native_input(entry, false)}};
    const auto &with = out.at("with_project").at("status");
    const auto &without = out.at("without_project").at("status");
    if (with == without && (with == "rejected" || with == "geometry_not_read"))
        out["status"] = with;
    return out;
}

Json parametric_graphics_append_rule(std::int32_t type) {
    const bool add = (type >= 1 && type <= 7) || type == 10;
    Json out = {{"scope", "parametric_graphics_append_after_successful_source_restore"},
                {"action", add ? "invoke_add" : "skip"}};
    if (!add)
        return out;
    out.update({{"output_count", "zero_or_one_depending_on_add_result"},
                {"symbology", "copy_entry"},
                {"transparency", "copy_entry"},
                {"inline_material",
                 type == 2 || type == 3 || type == 6 || type == 10 ? "copy_entry" : "not_passed"},
                {"line_style_scale", type == 1 || type == 2 ? "copy_entry" : "one"},
                {"widths", type == 1 || type == 2 ? "copy_entry" : "zero"},
                {"layer_id", std::uint32_t(0xffffffff)},
                {"view_flag", "destination_graphics"},
                {"geometry_operation", "type_specific_copy_with_destination_transform"},
                {"source_container_transform", "not_read"},
                {"source_container_material", "not_read"}});
    return out;
}
} // namespace p3d
