#include "graphics_native.hpp"

namespace p3d {
namespace {
// Track the byte reader's return value separately from the material's validity
// flag and the subsequent current-project material lookup. Neither determines
// whether the enclosing Entry is returned.
Json material_input(const Bytes &bytes, std::size_t start, std::size_t size) {
    Json out = {{"status", "not_evaluated"}, {"material_pointer", "unknown"}};
    try {
        require(start <= bytes.size() && size <= bytes.size() - start && size > 0,
                "native_material_input_unavailable");
        require(size <= INT32_MAX, "native_material_signed_cursor_limit");
        std::size_t p = 0;
        auto advance = [&](std::uint64_t n) {
            require(n <= UINT64_MAX - p, "native_material_range_addition_overflow");
            if (n > size - p) {
                out.update({{"status", "rejected"},
                            {"material_pointer", "null"},
                            {"reason", "native_material_length_guard"},
                            {"guard_offset", p}});
                return false;
            }
            p += static_cast<std::size_t>(n);
            return true;
        };
        auto string = [&]() {
            const auto at = p;
            if (!advance(8))
                return false;
            // Native strings consume all bytes, but copy floor(n / 2) UTF-16
            // characters. Odd lengths are not an input rejection here.
            return advance(Reader(bytes, start + at).u64());
        };
        advance(1); // The validity byte is copied, not validated.
        if (!string())
            return out;
        for (auto n : {1u, 24u, 1u, 8u, 1u, 4u, 4u})
            if (!advance(n))
                return out;
        if (!string() || !advance(16) || !advance(16) || !advance(8) || !string() || !advance(8))
            return out;
        for (auto components : {3u, 1u, 3u, 1u, 1u, 1u, 1u, 1u, 1u})
            if (!advance(1) || !advance(8 * components))
                return out;
        out["mandatory_bytes"] = p;
        // The second optional-marker cursor starts at zero. It is advanced to
        // the display-name end only when the first marker is actually consumed.
        std::size_t extension = 0;
        if (size - p > 4 && Reader(bytes, start + p).u32() == 0xabcd) {
            p += 4;
            if (!string())
                return out;
            extension = p;
            out["display_name_block"] = "read";
        } else
            out["display_name_block"] = "not_read";
        out["extended_data_probe_offset"] = extension;
        if (size - extension > 4 && Reader(bytes, start + extension).u32() == 0xabce) {
            p = extension + 4;
            const auto at = p;
            if (!advance(8))
                return out;
            const auto count = Reader(bytes, start + at).u64();
            if (!advance(count))
                return out;
            // Allocation rounds down to an even byte count, but memcpy uses
            // the original count. Do not emulate that unsafe native operation.
            require(count >= 2 && count % 2 == 0, "native_material_extension_unsafe_string_width");
            out["extended_data_block"] = "read";
        } else
            out["extended_data_block"] = "not_read";
        out.update({{"status", "material_constructed"},
                    {"material_pointer", "non_null"},
                    {"allocation_assumption", "successful"},
                    {"service_return_assumption", "normal"},
                    {"current_project_material_resolution", "not_evaluated"}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json entry_restore(const Bytes &bytes, const Json &geometry) {
    Json out = {{"status", "not_evaluated"},
                {"scope", "entry_reader_return_before_container_finish"},
                {"model_context_assumption", "valid"},
                {"allocation_assumption", "successful"},
                {"service_return_assumption", "normal"}};
    const auto status = geometry.at("status");
    if (status == "rejected") {
        out.update({{"status", "rejected"}, {"entry_pointer", "null"}});
        return out;
    }
    if (status != "geometry_constructed" && status != "geometry_not_read")
        return out;
    if (geometry.value("material_footer", std::string()) == "not_read") {
        out["footer"] = {{"status", "not_read"}};
        out.update({{"status", "retained"}, {"entry_pointer", "non_null"}});
        return out;
    }
    const auto start = geometry.at("geometry_offset").get<std::size_t>();
    const auto count = Reader(bytes, start - 8).u64();
    // The geometry reader already established a non-wrapping, bounded range.
    auto footer = graphics_native_footer(bytes, start + static_cast<std::size_t>(count), true);
    if (footer.at("status") == "read" || footer.at("status") == "not_present")
        out.update({{"status", "retained"}, {"entry_pointer", "non_null"}});
    out["footer"] = std::move(footer);
    return out;
}

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
Json leaf_construction(const GeometryBytes &b, std::size_t root, unsigned tag) {
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

struct GeometryConstruction {
    const GeometryBytes &b;
    std::size_t remaining = 1000000;
    void visit(unsigned depth) {
        require(depth <= 80, "native_geometry_construction_depth_limit");
        require(remaining > 0, "native_geometry_construction_work_limit");
        --remaining;
    }
    Json object(const char *type, const char *kind, bool present = true) const {
        return {{"geometry_type", type},
                {"geometry_kind", kind},
                {"geometry_pointer", present ? "non_null" : "null"},
                {"allocation_assumption", "successful"},
                {"geometry_validity", "not_checked_by_reader"}};
    }
    std::optional<std::size_t> child(std::size_t table, unsigned field) const {
        const auto p = b.field(table, field, 4);
        return p ? std::optional<std::size_t>(b.indirect(*p)) : std::nullopt;
    }
    bool flag(std::size_t table, unsigned field) const {
        const auto p = b.field(table, field, 1);
        return p && b.at<std::uint8_t>(*p) != 0;
    }
    Json curve_vector(std::optional<std::size_t> table, unsigned depth) {
        visit(depth);
        auto out = object("CurveVector", "curve_vector", table.has_value());
        if (!table)
            return out;
        const auto type = b.field(*table, 0, 4);
        out["boundary_type"] = type ? b.at<std::int32_t>(*type) : 0;
        const auto vector = child(*table, 1);
        require(vector.has_value(), "native_curve_vector_requires_member_vector");
        const auto count = b.at<std::uint32_t>(*vector);
        require(count <= b.size / 4, "native_curve_vector_member_extent_unavailable");
        b.range(*vector + 4, std::size_t(count) * 4);
        out["source_member_count"] = count;
        out["members"] = Json::array();
        std::size_t appended = 0;
        for (std::uint32_t i = 0; i < count; ++i) {
            auto member = variant(b.indirect(*vector + 4 + std::size_t(i) * 4), depth + 1);
            member["source_member_index"] = i;
            if (member.at("geometry_pointer") == "null")
                member["action"] = "skip_null";
            else if (member.at("geometry_kind") == "curve" ||
                     member.at("geometry_kind") == "curve_vector") {
                member["action"] = member.at("geometry_kind") == "curve"
                                       ? "append_curve"
                                       : "wrap_nested_curve_vector";
                member["output_member_index"] = appended++;
            } else
                member["action"] = "skip_non_curve";
            out["members"].push_back(std::move(member));
        }
        out["output_member_count"] = appended;
        return out;
    }
    Json variant(std::size_t root, unsigned depth = 0) {
        visit(depth);
        const auto tag_field = b.field(root, 0, 1);
        const auto tag = tag_field ? b.at<std::uint8_t>(*tag_field) : 0;
        // The generic member reader does not inspect the union data for these tags.
        if (!tag || tag == 15 || tag > 21)
            return object("unselected_union_tag", "none", false);
        if ((tag >= 6 && tag <= 9) || tag == 13) {
            auto out = leaf_construction(b, root, tag);
            out["geometry_kind"] = tag == 13 ? "polyface" : "solid";
            out["geometry_pointer"] = "non_null";
            return out;
        }
        require(tag == 1 || tag == 2 || tag == 4 || tag == 5 || tag == 10 || tag == 12 ||
                    tag == 18 || tag == 21,
                "native_geometry_construction_not_supported");
        const auto table = child(root, 1);
        if (tag == 5)
            return curve_vector(table, depth + 1);
        require(table.has_value(), "native_geometry_reader_requires_union_data");
        if (tag == 1 || tag == 2) {
            const auto bytes = tag == 1 ? 48u : 88u;
            const auto detail = b.field(*table, 0, bytes);
            require(detail.has_value(), "native_curve_reader_requires_detail_pointer");
            auto out = object(tag == 1 ? "LineSegment" : "EllipticArc", "curve");
            out.update({{"operation", "copy_fixed_detail"},
                        {"detail_offset", *detail},
                        {"detail_bytes", bytes}});
            return out;
        }
        if (tag == 4 || tag == 18) {
            const auto vector = child(*table, 0);
            require(vector.has_value(), "native_point_curve_requires_vector");
            const auto count = b.at<std::uint32_t>(*vector);
            const auto copied = std::uint64_t(count / 3) * 24;
            require(copied <= b.size, "native_point_curve_extent_unavailable");
            b.range(*vector + 4, static_cast<std::size_t>(copied));
            auto out = object(tag == 4 ? "LineString" : "PointString", "curve");
            out.update({{"operation", "copy_point_tuples"},
                        {"source_scalar_count", count},
                        {"point_count", count / 3},
                        {"ignored_tail_scalars", count % 3},
                        {"data_offset", *vector + 4},
                        {"copied_bytes", copied}});
            return out;
        }
        if (tag == 10) {
            auto out = object("DgnExtrusion", "solid");
            out["base_curve"] = curve_vector(child(*table, 0), depth + 1);
            const auto vector = b.field(*table, 1, 24);
            require(vector.has_value(), "native_extrusion_requires_vector_detail");
            out.update({{"operation", "retain_base_curve_and_copy_extrusion"},
                        {"extrusion_vector_offset", *vector},
                        {"capped", flag(*table, 2)}});
            return out;
        }
        if (tag == 21) {
            auto out = object("P3DSectionLoft", "solid");
            out["section0"] = curve_vector(child(*table, 0), depth + 1);
            out["section1"] = curve_vector(child(*table, 1), depth + 1);
            const auto vector = child(*table, 2);
            require(vector.has_value(), "native_section_loft_requires_guide_groups");
            const auto count = b.at<std::int32_t>(*vector);
            // The outer vector is resized before the signed loop guard. A negative
            // size cannot be interpreted as the empty-loop behavior of inner groups.
            require(count >= 0, "native_section_loft_negative_group_allocation");
            require(std::size_t(count) <= b.size / 4,
                    "native_section_loft_group_extent_unavailable");
            b.range(*vector + 4, std::size_t(count) * 4);
            out.update({{"operation", "retain_sections_and_nested_guide_groups"},
                        {"capped", flag(*table, 3)},
                        {"group_count", count},
                        {"guide_groups", Json::array()}});
            for (std::int32_t i = 0; i < count; ++i) {
                visit(depth + 1);
                const auto group = b.indirect(*vector + 4 + std::size_t(i) * 4);
                const auto source_count = b.at<std::int32_t>(group);
                const auto read_count = static_cast<std::uint32_t>(std::max(0, source_count));
                require(read_count <= b.size / 4, "native_section_loft_guide_extent_unavailable");
                b.range(group + 4, std::size_t(read_count) * 4);
                Json item = {{"source_group_index", i},
                             {"source_count_signed", source_count},
                             {"guide_count", read_count},
                             {"guides", Json::array()}};
                for (std::uint32_t j = 0; j < read_count; ++j) {
                    auto guide =
                        curve_vector(b.indirect(group + 4 + std::size_t(j) * 4), depth + 2);
                    guide["source_guide_index"] = j;
                    item["guides"].push_back(std::move(guide));
                }
                out["guide_groups"].push_back(std::move(item));
            }
            return out;
        }
        auto out = object("DgnRuledSweep", "solid");
        const auto vector = child(*table, 0);
        require(vector.has_value(), "native_ruled_sweep_requires_section_vector");
        // This reader, unlike CurveVector, interprets the vector count as signed.
        const auto count = b.at<std::int32_t>(*vector);
        const auto read_count = static_cast<std::uint32_t>(std::max(0, count));
        require(read_count <= b.size / 4, "native_ruled_sweep_section_extent_unavailable");
        b.range(*vector + 4, std::size_t(read_count) * 4);
        out.update({{"operation", "retain_ordered_sections"},
                    {"source_count_signed", count},
                    {"section_count", read_count},
                    {"capped", flag(*table, 1)},
                    {"sections", Json::array()}});
        for (std::uint32_t i = 0; i < read_count; ++i) {
            auto section = curve_vector(b.indirect(*vector + 4 + std::size_t(i) * 4), depth + 1);
            section["source_section_index"] = i;
            out["sections"].push_back(std::move(section));
        }
        return out;
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
        if (!selected) {
            reject("bgfb_type_not_accepted_by_entry_reader");
            return out;
        }
        if (type != 4) {
            out["construction"] = GeometryConstruction{b}.variant(root);
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

Json graphics_native_footer(const Bytes &bytes, std::size_t offset, bool entry) {
    Json out = {{"status", "not_evaluated"},
                {"source_offset", offset},
                {"material_pointer", "null"},
                // The container factory zeroes its object; only Entry's
                // constructor explicitly initializes its line scale to one.
                {"line_style_scale", entry ? 1.0 : 0.0}};
    if (entry)
        out.update({{"start_width", 0.0},
                    {"end_width", 0.0},
                    {"layer_id", UINT32_MAX},
                    {"view_flag", UINT32_MAX}});
    try {
        require(offset <= bytes.size(), "native_footer_offset_unavailable");
        require(bytes.size() <= INT32_MAX, "native_footer_signed_cursor_limit");
        Reader r(bytes, offset);
        if (!r.left()) {
            out["status"] = "not_present";
            return out;
        }
        const auto marker = bytes[offset];
        out["marker"] = marker;
        // The native branch uses a signed-byte comparison, with no version switch.
        const bool prefixed = marker > 1 && marker < 128;
        out["format"] = prefixed ? "length_prefixed_material" : "legacy_inline_material";
        std::size_t material_size = r.left();
        if (prefixed) {
            r.u8();
            require(r.left() >= 4, "native_footer_length_unavailable");
            const auto size = r.i32();
            out["material_size_signed"] = size;
            material_size = static_cast<std::size_t>(std::max(0, size));
            require(material_size <= r.left(), "native_footer_material_range_unavailable");
        }
        if (material_size) {
            auto material = material_input(bytes, r.p, material_size);
            out["material_pointer"] = material.at("material_pointer");
            out["material_input"] = std::move(material);
            if (out.at("material_input").at("status") == "not_evaluated")
                return out;
            r.p += material_size;
        } else
            out["material_input"] = {{"status", "not_called"}};
        const auto extension = entry ? 28u : 8u;
        out["style_extension"] = "not_read";
        if (prefixed && r.left() >= extension) {
            const auto begin = r.p;
            out["line_style_scale"] = r.f64();
            if (entry) {
                out["start_width"] = r.f64();
                out["end_width"] = r.f64();
                out["layer_id"] = r.u32();
            }
            out["style_extension"] = "read";
            out["style_extension_hex"] = hex(slice(bytes, begin, extension));
        }
        out["unread_suffix_bytes"] = r.left();
        out["status"] = "read";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json graphics_entry_native_input(const Bytes &entry) {
    Json out = {{"scope", "entry_geometry_reader_before_material_footer"},
                {"project_context", "not_provided"},
                {"status", "not_evaluated"},
                {"with_project", native_input(entry, true)},
                {"without_project", native_input(entry, false)}};
    for (const auto *context : {"with_project", "without_project"})
        out[context]["entry_restore"] = entry_restore(entry, out.at(context));
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
