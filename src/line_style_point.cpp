#include "line_style_source.hpp"
#include "line_style_resource_fields.hpp"

namespace p3d {
namespace {
constexpr std::size_t payload = 208, header_size = 168, symbol_size = 88;
Json source(const Bytes &b, std::size_t p, std::size_t n) {
    return {{"source_offset", p}, {"data", rawbytes(slice(b, p, n))}};
}
Json real(const Bytes &b, std::size_t p) {
    const auto v = Reader(b).at<double>(p);
    return {{"source_offset", p},
            {"value", std::isfinite(v) ? Json(v) : Json()},
            {"finite", std::isfinite(v)},
            {"source", rawbytes(slice(b, p, 8))}};
}
Json symbol(const Bytes &b, std::size_t p, std::size_t index) {
    Reader r(b);
    const auto flags = r.at<std::uint16_t>(p + 10);
    const unsigned position = flags & 3;
    const unsigned role = position ? 0 : flags & 4 ? 1 : flags & 8 ? 2 : flags & 16 ? 3 : 4;
    const auto angle = r.at<double>(p + 56) / 57.29577951308232;
    static constexpr const char *placements[] = {"stroke", "element_origin", "element_end",
                                                 "each_vertex", "not_retained"};
    Json placement = {{"native_placement_kind", role},
                      {"placement", placements[role]},
                      {"stroke_position", position ? position : 3},
                      {"native_partial_stroke_mode", flags & 0x400 ? 2 : (flags >> 11) & 1},
                      {"native_rotation_mode", flags & 0x20 ? 2 : (flags >> 6) & 1},
                      {"scale_by_line_style", !(flags & 0x100)},
                      {"clip_at_element_end", !(flags & 0x200)},
                      {"use_symbol_color", bool(flags & 0x4000)},
                      {"use_symbol_weight", bool(flags & 0x8000)},
                      {"rotation_radians", std::isfinite(angle) ? Json(angle) : Json()},
                      {"rotation_finite", std::isfinite(angle)},
                      {"retained", role != 4}};
    return {{"source_symbol_index", index},
            {"source_offset", p},
            {"symbol_type", r.at<std::uint32_t>(p)},
            {"local_resource_id", r.at<std::uint32_t>(p + 4)},
            {"stroke_number", r.at<std::uint16_t>(p + 8)},
            {"modifiers", flags},
            {"source_options",
             {{"stroke_position", position},
              {"at_element_origin", bool(flags & 4)},
              {"at_element_end", bool(flags & 8)},
              {"at_each_vertex", bool(flags & 16)},
              {"mirror_on_reversed_line", bool(flags & 0x20)},
              {"absolute_rotation", bool(flags & 0x40)},
              {"do_not_scale", bool(flags & 0x100)},
              {"do_not_clip", bool(flags & 0x200)},
              {"no_partial_strokes", bool(flags & 0x400)},
              {"partial_origin_beyond_end", bool(flags & 0x800)},
              {"use_symbol_color", bool(flags & 0x4000)},
              {"use_symbol_weight", bool(flags & 0x8000)},
              {"unassigned_modifier_bits", flags & 0x3080}}},
            {"x_offset", real(b, p + 16)},
            {"y_offset", real(b, p + 24)},
            {"saved_rotation_degrees", real(b, p + 56)},
            {"initial_placement", std::move(placement)},
            {"unassigned_ranges",
             Json::array({source(b, p + 12, 4), source(b, p + 32, 24), source(b, p + 64, 24)})}};
}
Json copy_fields(const Bytes &b, std::uint32_t count, std::size_t references) {
    Json out = {
        {"status", "unresolved"},   {"scope", "native_line_point_copy_reader_field_projection"},
        {"fixups", Json::array()},  {"final_base_pattern", nullptr},
        {"final_symbols", nullptr}, {"description_conversion", "not_evaluated"}};
    try {
        Reader r(b);
        const auto used = r.at<std::uint16_t>(200);
        out["reference_count"] = used;
        out["reference_source_offset"] = references;
        require(count <= (INT32_MAX - header_size) / symbol_size,
                "line_point_native_copy_size_overflow");
        const auto size = count ? header_size + std::size_t(count) * symbol_size : 256;
        out["native_copy_bytes"] = size;
        require(size <= b.size() - payload, "line_point_native_copy_source_truncated");
        auto copy = slice(b, payload, size);
        // Native order is type read, type conversion/write, reference read, ID
        // write. A zero reference still writes zero; it is not a skip gate.
        auto fix = [&](std::size_t slot, unsigned reference_index, bool base) {
            Json step = {{"reference_index", reference_index},
                         {"source_offset", references + std::size_t(reference_index) * 8},
                         {"target", base ? "base_pattern" : "symbol"}};
            if (!base)
                step["symbol_index"] = reference_index - 1;
            try {
                require(slot <= copy.size() && copy.size() - slot >= 8,
                        "line_point_native_copy_slot_out_of_bounds");
                const auto saved = Reader(copy).at<std::uint32_t>(slot);
                const auto kind = native_line_style_local_kind(saved);
                step["saved_type"] = saved;
                if (base && saved == 0) {
                    step["action"] = "preserved_zero_base_type";
                    out["fixups"].push_back(std::move(step));
                    return;
                }
                std::memcpy(copy.data() + slot, &kind, 4);
                step.update({{"type_value", kind}, {"action", "type_converted_reference_unread"}});
                const auto id = r.at<std::uint64_t>(references + std::size_t(reference_index) * 8);
                const auto low = std::uint32_t(id);
                std::memcpy(copy.data() + slot + 4, &low, 4);
                step.update({{"source_id", id},
                             {"local_resource_id", low},
                             {"action", "converted_reference"}});
                out["fixups"].push_back(std::move(step));
            } catch (const std::exception &e) {
                step["reason"] = e.what();
                if (!step.contains("action"))
                    step["action"] = "rejected_slot_read";
                out["fixups"].push_back(std::move(step));
                throw;
            }
        };
        const bool base_fixed = Reader(copy).at<std::uint32_t>(152) != 0;
        fix(152, 0, true); // Independent of saved reference count.
        for (unsigned i = 1; i < used; ++i)
            fix(header_size + std::size_t(i - 1) * symbol_size, i, false);
        Json symbols = Json::array();
        for (std::uint32_t i = 0; i < count; ++i) {
            const auto p = header_size + std::size_t(i) * symbol_size;
            symbols.push_back({{"source_symbol_index", i},
                               {"type_value", Reader(copy).at<std::uint32_t>(p)},
                               {"local_resource_id", Reader(copy).at<std::uint32_t>(p + 4)},
                               {"selection", i + 1 < used ? "reference_fixup" : "saved_fields"}});
        }
        if (count == 0)
            out["unused_slot_storage"] = rawbytes(slice(copy, header_size, symbol_size));
        out.update({{"status", "resolved"},
                    {"final_base_pattern",
                     {{"type_value", Reader(copy).at<std::uint32_t>(152)},
                      {"local_resource_id", Reader(copy).at<std::uint32_t>(156)},
                      {"selection", base_fixed ? "reference_fixup" : "saved_fields"}}},
                    {"final_symbols", std::move(symbols)}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
Json object_state(const Bytes &b, const Json &symbols, std::size_t references) {
    Json out = {{"status", "unresolved"},
                {"scope", "fresh_line_point_before_child_resource_loading"},
                {"base_pattern", nullptr},
                {"retained_symbol_indices", Json::array()},
                {"assignments", Json::array()},
                {"final_placements", nullptr},
                {"child_resource_loading", "not_evaluated"},
                {"symbol_type_selection", "not_consumed_by_fresh_symbol_reader"}};
    try {
        Reader r(b);
        Json placements = Json::array();
        for (const auto &s : symbols)
            if (s.at("initial_placement").at("retained").get<bool>()) {
                out["retained_symbol_indices"].push_back(s.at("source_symbol_index"));
                placements.push_back({{"source_symbol_index", s.at("source_symbol_index")},
                                      {"placement", s.at("initial_placement")},
                                      {"stroke_number", s.at("stroke_number")},
                                      {"x_offset", s.at("x_offset")},
                                      {"y_offset", s.at("y_offset")}});
            }
        const auto id = r.at<std::uint64_t>(references);
        out["base_pattern"] = {{"kind", native_line_style_local_kind(r.at<std::uint32_t>(360))},
                               {"resource_id", id ? id : std::uint64_t(r.at<std::uint32_t>(364))},
                               {"saved_reference_id", id},
                               {"reference_source_offset", references},
                               {"selection", id ? "nonzero_saved_reference" : "saved_local_id"},
                               {"target_resolution", "not_evaluated"}};
        for (std::size_t i = 0; i < symbols.size(); ++i) {
            Json step = {{"placement_index", i}, {"reference_index", i + 1}};
            // Filtering compacts the vector, but the later native loop still
            // uses the original count. Do not repair this correspondence.
            if (i >= placements.size()) {
                step["action"] = "rejected_compacted_vector_access";
                out["assignments"].push_back(std::move(step));
                throw std::runtime_error("line_point_compacted_symbol_vector_out_of_bounds");
            }
            step["source_symbol_index"] = placements[i].at("source_symbol_index");
            step["reference_source_offset"] = references + (i + 1) * 8;
            try {
                const auto reference = r.at<std::uint64_t>(references + (i + 1) * 8);
                placements[i]["resource_id"] = reference; // Zero has no local-ID fallback here.
                placements[i]["target_resolution"] = "not_evaluated";
                step.update({{"action", "assigned_reference"}, {"resource_id", reference}});
                out["assignments"].push_back(std::move(step));
            } catch (const std::exception &e) {
                step.update({{"action", "reference_unread"}, {"reason", e.what()}});
                out["assignments"].push_back(std::move(step));
                throw;
            }
        }
        out.update({{"status", "resolved"}, {"final_placements", std::move(placements)}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace

Json decode_native_line_point_style(const Bytes &b) {
    Json out = {{"status", "unresolved"},
                {"source_status", "unresolved"},
                {"source_offset", payload},
                {"symbols", Json::array()},
                {"resource_loading", "not_evaluated"}};
    try {
        Reader r(b);
        require(b.size() >= payload + header_size, "line_point_header_truncated");
        const auto count = r.at<std::uint32_t>(372);
        out["symbol_count"] = count;
        out["symbol_count_source_offset"] = 372;
        out["source_payload"] = source(b, payload, b.size() - payload);
        require(count <= (b.size() - payload - header_size) / symbol_size,
                "line_point_symbols_truncated");
        out["base_pattern"] = {{"resource_type", r.at<std::uint32_t>(360)},
                               {"local_resource_id", r.at<std::uint32_t>(364)},
                               {"source_offset", 360}};
        out["description_source"] = source(b, payload, 128);
        out["unassigned_header"] = Json::array({source(b, 336, 24), source(b, 368, 4)});
        for (std::uint32_t i = 0; i < count; ++i)
            out["symbols"].push_back(
                symbol(b, payload + header_size + std::size_t(i) * symbol_size, i));
        out["source_status"] = "decoded";
        const auto references = payload + std::size_t(r.at<std::uint16_t>(204));
        out["reference_source_offset"] = references;
        out["copy_reader_fields"] = copy_fields(b, count, references);
        out["initial_object_state"] = object_state(b, out["symbols"], references);
        out["status"] = out["copy_reader_fields"].at("status") == "resolved" &&
                                out["initial_object_state"].at("status") == "resolved"
                            ? "decoded"
                            : "partial";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
