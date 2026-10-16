#include "line_style_source.hpp"

namespace p3d {
namespace {
constexpr std::size_t payload = 208, header_size = 176, child_size = 56;
std::uint32_t local_kind(std::uint32_t resource) {
    switch (resource) {
    case 0:
        return 6;
    case 2001:
        return 2;
    case 2002:
        return 3;
    case 2003:
        return 4;
    case 2004:
    case 2005:
        return 1;
    default:
        return resource;
    }
}
Json source(const Bytes &b, std::size_t p, std::size_t n) {
    return {{"source_offset", p}, {"data", rawbytes(slice(b, p, n))}};
}
Json child(const Bytes &b, std::size_t p) {
    Reader r(b);
    const auto offset = r.at<double>(p + 24);
    return {{"source_offset", p},
            {"resource_type", r.at<std::uint32_t>(p)},
            {"local_resource_id", r.at<std::uint32_t>(p + 4)},
            {"offset",
             {{"source_offset", p + 24},
              {"value", std::isfinite(offset) ? Json(offset) : Json()},
              {"finite", std::isfinite(offset)},
              {"source", rawbytes(slice(b, p + 24, 8))}}},
            {"unassigned_ranges", Json::array({source(b, p + 8, 16), source(b, p + 32, 24)})}};
}
// Fields after the native buffer-copy reader's reference fixups. This does not
// perform its separate, host-encoding-dependent description conversion.
Json copy_fields(const Bytes &b, std::uint32_t count, std::size_t references) {
    Json out = {{"status", "unresolved"},
                {"scope", "native_compound_copy_reader_field_projection"},
                {"fixups", Json::array()},
                {"final_entries", nullptr},
                {"description_conversion", "not_evaluated"}};
    try {
        Reader r(b);
        const auto used = r.at<std::uint16_t>(200);
        out["reference_count"] = used;
        out["reference_source_offset"] = references;
        // The native expression is signed after 32-bit multiply/add. Refuse
        // wrapped allocation sizes instead of manufacturing a smaller buffer.
        require(count <= (INT32_MAX - header_size) / child_size,
                "compound_native_copy_size_overflow");
        const auto size = count ? header_size + std::size_t(count) * child_size : 232;
        out["native_copy_bytes"] = size;
        require(size <= b.size() - payload, "compound_native_copy_source_truncated");
        auto copy = slice(b, payload, size);
        std::vector<bool> converted(count, false);
        for (unsigned i = 0; i < used; ++i) {
            const auto p = references + std::size_t(i) * 8;
            const auto id = r.at<std::uint64_t>(p);
            Json step = {{"reference_index", i}, {"source_offset", p}, {"source_id", id}};
            if (id == 0) {
                // A zero reference skips both slot reads and writes, including
                // references beyond the logical/allocated component count.
                step["action"] = "preserved_zero_reference";
            } else {
                const auto slot = header_size + std::size_t(i) * child_size;
                if (slot > copy.size() || copy.size() - slot < 8) {
                    step["action"] = "rejected_out_of_bounds_fixup";
                    out["fixups"].push_back(std::move(step));
                    throw std::runtime_error("compound_native_copy_fixup_out_of_bounds");
                }
                const auto saved = Reader(copy).at<std::uint32_t>(slot);
                const auto kind = local_kind(saved), low_id = std::uint32_t(id);
                std::memcpy(copy.data() + slot, &kind, 4);
                std::memcpy(copy.data() + slot + 4, &low_id, 4);
                if (i < count)
                    converted[i] = true;
                step.update({{"action", "converted_reference"},
                             {"saved_resource_type", saved},
                             {"kind", kind},
                             {"local_resource_id", low_id},
                             {"logical_child", i < count}});
            }
            out["fixups"].push_back(std::move(step));
        }
        Json entries = Json::array();
        for (std::uint32_t i = 0; i < count; ++i) {
            const auto p = header_size + std::size_t(i) * child_size;
            entries.push_back({{"child_index", i},
                               {"type_value", Reader(copy).at<std::uint32_t>(p)},
                               {"local_resource_id", Reader(copy).at<std::uint32_t>(p + 4)},
                               {"selection", converted[i] ? "reference_fixup" : "saved_fields"}});
        }
        // Empty compound buffers still copy one physical slot. Fixups may
        // modify it even though it never becomes a logical component.
        if (count == 0)
            out["unused_slot_storage"] = rawbytes(slice(copy, header_size, child_size));
        out.update({{"status", "resolved"}, {"final_entries", std::move(entries)}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
Json object_children(const Bytes &b, const Json &children, std::size_t references) {
    Json out = {{"status", "unresolved"},
                {"scope", "fresh_compound_before_child_resource_loading"},
                {"entries", Json::array()},
                {"child_resource_loading", "not_evaluated"},
                {"reference_count", children.size()},
                {"reference_source_offset", references}};
    try {
        Reader r(b);
        for (std::size_t i = 0; i < children.size(); ++i) {
            const auto p = references + i * 8;
            const auto reference = r.at<std::uint64_t>(p);
            const auto &c = children.at(i);
            out["entries"].push_back(
                {{"child_index", i},
                 {"reference_source_offset", p},
                 {"saved_reference_id", reference},
                 {"kind", reference ? local_kind(c.at("resource_type").get<std::uint32_t>()) : 6},
                 {"resource_id",
                  reference ? reference : c.at("local_resource_id").get<std::uint64_t>()},
                 {"selection",
                  reference ? "nonzero_saved_reference" : "zero_reference_internal_component"},
                 {"target_resolution", "not_evaluated"}});
        }
        out["status"] = "resolved";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace

Json decode_native_compound_line_style(const Bytes &b) {
    Json out = {{"status", "unresolved"},
                {"source_status", "unresolved"},
                {"source_offset", payload},
                {"children", Json::array()},
                {"resource_loading", "not_evaluated"}};
    try {
        Reader r(b);
        require(b.size() >= payload + header_size, "compound_line_style_header_truncated");
        const auto count = r.at<std::uint32_t>(payload + 172);
        out["child_count"] = count;
        out["child_count_source_offset"] = payload + 172;
        out["source_payload"] = source(b, payload, b.size() - payload);
        require(count <= (b.size() - payload - header_size) / child_size,
                "compound_line_style_children_truncated");
        out["description_source"] = source(b, payload, 128);
        out["unassigned_header"] = source(b, payload + 128, 44);
        for (std::uint32_t i = 0; i < count; ++i)
            out["children"].push_back(
                child(b, payload + header_size + std::size_t(i) * child_size));
        out["source_status"] = "decoded";
        const auto references = payload + std::size_t(r.at<std::uint16_t>(204));
        out["reference_source_offset"] = references;
        // Independent native routes: the copy reader follows saved reference
        // count; the object reader follows the compound's component count.
        out["copy_reader_fields"] = copy_fields(b, count, references);
        out["initial_object_children"] = object_children(b, out["children"], references);
        out["status"] = out["copy_reader_fields"].at("status") == "resolved" &&
                                out["initial_object_children"].at("status") == "resolved"
                            ? "decoded"
                            : "partial";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
