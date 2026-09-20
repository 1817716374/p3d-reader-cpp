#include "internal.hpp"

namespace p3d {
namespace {
struct GeometryBytes {
    const Bytes &entry;
    std::size_t start;
    std::size_t size;
    template <class T> T at(std::size_t offset) const {
        require(offset <= size && sizeof(T) <= size - offset, "bgfb_geometry_field_unavailable");
        T value;
        std::memcpy(&value, entry.data() + start + offset, sizeof(T));
        return value;
    }
};
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
        if (!selected)
            reject("bgfb_type_not_accepted_by_entry_reader");
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
