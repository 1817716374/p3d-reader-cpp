#include "internal.hpp"
namespace p3d {
Json native_record_input_filter(const Json &record, const Bytes &base) {
    const auto prefix = Reader(base).u32();
    const auto words = Reader(base, 8).u32();
    const bool skip_flag = (record.at("element_flags").get<unsigned>() & 8) != 0;
    const bool zero_type = record.at("element_type") == 0;
    // The physical reader checks the upper word limit before inspecting the
    // prefix. Only prefix-zero records increment its descendant counter; the
    // element reader applies its remaining filters after that increment.
    const bool read_error = words > 65535;
    const bool skipped = prefix != 0 || skip_flag || zero_type || words < 16;
    return {{"reader_profile", "bimbase_2025_native_record_input"},
            {"status", read_error ? "read_error"
                       : skipped  ? "skipped"
                                  : "accepted"},
            {"reason", read_error    ? Json("record_word_count_exceeds_limit")
                       : prefix != 0 ? Json("nonzero_prefix")
                       : skip_flag   ? Json("flag_0008")
                       : zero_type   ? Json("zero_element_type")
                       : words < 16  ? Json("record_word_count_below_minimum")
                                     : Json()},
            {"source_prefix_word", prefix},
            {"descendant_counter_effect", read_error ? "not_reached"
                                          : prefix   ? "not_incremented"
                                                     : "incremented"},
            {"native_error_code", read_error ? Json(0x12005) : Json()},
            {"record_word_count", words}};
}
static Json native_record_input_child(const Json &n, const Bytes &base) {
    const auto type = n.at("element_type").get<unsigned>();
    const auto flags = n.at("element_flags").get<unsigned>();
    const bool compound =
        (type >= 10 && type <= 32) || ((type < 33 || type > 63) && (flags & 0x40));
    Json result = {{"compound", compound}, {"descendant_count", 0}};
    auto count_offset = (flags & 0x20) ? 108u : 36u;
    if (type == 24) {
        const auto words = Reader(base, 8).u32(), base_words = Reader(base, 12).u32();
        const bool extended = (flags & 0x20) != 0;
        if (extended)
            require(base_words >= 52 && words >= base_words && base.size() >= 108,
                    "type 24 input extension exceeds declared header");
        result["conversion"] = {{"kind", "type_24_extended_header_removal"},
                                {"applied", extended},
                                {"output_element_flags", flags & ~0x20u},
                                {"output_record_word_count", words - (extended ? 36u : 0u)},
                                {"output_base_word_count", base_words - (extended ? 36u : 0u)},
                                {"descendant_count_source_offset", count_offset},
                                {"descendant_count_source_bytes", 4}};
    } else if (type == 13) {
        // All addresses include the four-byte physical record prefix. The
        // legacy discriminator is tested in this order before upgrading fields.
        bool legacy = Reader(base, 160).u32() == 0;
        if (legacy)
            legacy = Reader(base, 342).u16() == 0;
        if (legacy)
            legacy = Reader(base, 344).u16() == 0;
        std::uint16_t entries = 0;
        if (legacy) {
            entries = Reader(base, 346).u16();
            legacy = entries <= 2500;
        }
        const auto base_words = Reader(base, 12).u32();
        if (legacy)
            legacy = base_words == 172u + 8u * entries;
        Json conversion = {
            {"kind", "type_13_legacy_layout_upgrade"},
            {"applied", legacy},
            {"output_element_flags", flags},
            {"output_base_word_count", base_words + (legacy ? 12u : 0u)},
            {"output_record_word_count", Reader(base, 8).u32() + (legacy ? 12u : 0u)},
            {"payload_reconstruction", "not_evaluated"}};
        if (legacy) {
            require(base.size() >= 348u + 16u * entries, "truncated type 13 legacy entries");
            conversion["legacy_entry_count"] = entries;
        }
        // The upgrade preserves native +0x20. In an extended-header record,
        // however, native +0x68 is one repeated u16 from old native +0x3c,
        // followed by two zero bytes, rather than the old extended count.
        if (legacy && (flags & 0x20)) {
            count_offset = 64;
            result["descendant_count"] = Reader(base, count_offset).u16();
            conversion["descendant_count_source_bytes"] = 2;
        } else {
            result["descendant_count"] = Reader(base, count_offset).u32();
            conversion["descendant_count_source_bytes"] = 4;
        }
        conversion["descendant_count_source_offset"] = count_offset;
        result["conversion"] = std::move(conversion);
        return result;
    } else if (type == 62) {
        // The native branch only updates the 3x3 matrix at native +0xa0.
        // It cannot change the type/flags or add descendants. Matrix repair
        // itself is a separate geometry operation, not needed to resolve a leaf.
        require(base.size() >= 236, "truncated type 62 input matrix");
        result["conversion"] = {{"kind", "type_62_matrix_repair"},
                                {"matrix_source_offset", 164},
                                {"matrix_value_count", 9},
                                {"payload_reconstruction", "not_evaluated"}};
    }
    if (compound)
        result["descendant_count"] = Reader(base, count_offset).u32();
    return result;
}
Json native_record_input_subtree(std::size_t ni, const Json &records, std::uint32_t counter) {
    Json out = {
        {"scope", "record_input_if_reached"},       {"status", "invalid"},
        {"member_record_indices", Json::array()},   {"nodes", Json::array()},
        {"consumed_record_indices", Json::array()}, {"skipped_record_indices", Json::array()},
        {"record_conversions", Json::array()}};
    const auto &table = records[ni];
    const auto stream = table.value("stream", Json());
    try {
        const auto base = bytesof(table.at("data"));
        const auto filter = native_record_input_filter(table, base);
        if (filter["status"] != "accepted") {
            out["status"] = filter["status"];
            out["reason"] = filter["reason"];
            out["native_error_code"] = filter["native_error_code"];
            return out;
        }
        const auto root = native_record_input_child(table, base);
        out["root_topology"] = root;
        out["start_counter"] = counter;
        const auto target = std::uint32_t(counter + root["descendant_count"].get<std::uint32_t>());
        out["target_counter"] = target;
        std::vector<std::pair<std::uint32_t, std::size_t>> ends;
        if (counter < target)
            ends.emplace_back(target, ni);
        std::size_t j = ni + 1;
        auto next_offset =
            table.at("offset").get<std::uint64_t>() + table.at("length").get<std::uint64_t>();
        Json nodes = Json::array(), indices = Json::array();
        while (!ends.empty()) {
            require(j < records.size(), "native input ends before requested child");
            const auto &n = records[j];
            require(n.value("stream", Json()) == stream && n.at("offset") == next_offset,
                    "native input child crosses stream or record boundary");
            const auto b = bytesof(n.at("data"));
            const auto input = native_record_input_filter(n, b);
            out["consumed_record_indices"].push_back(j);
            if (input["status"] == "read_error") {
                out["status"] = "read_error";
                out["error_record_index"] = j;
                out["native_error_code"] = input["native_error_code"];
                return out;
            }
            if (input["descendant_counter_effect"] == "incremented")
                ++counter;
            next_offset += n.at("length").get<std::uint64_t>();
            const auto current = j++;
            if (input["status"] == "skipped") {
                out["skipped_record_indices"].push_back(current);
                // This skip is inside read-next-child. Its parent does not test
                // the counter again until an accepted complete child returns.
                continue;
            }
            const auto child = native_record_input_child(n, b);
            if (child.contains("conversion")) {
                auto conversion = child["conversion"];
                conversion["native_record_index"] = current;
                conversion["compound"] = child["compound"];
                conversion["descendant_count"] = child["descendant_count"];
                out["record_conversions"].push_back(std::move(conversion));
            }
            nodes.push_back({{"native_record_index", current},
                             {"parent_record_index", ends.back().second},
                             {"compound", child["compound"]}});
            if (ends.size() == 1)
                indices.push_back(current);
            if (child["compound"] == true) {
                const auto child_end =
                    std::uint32_t(counter + child["descendant_count"].get<std::uint32_t>());
                if (counter < child_end)
                    ends.emplace_back(child_end, current);
            }
            while (!ends.empty() && counter >= ends.back().first)
                ends.pop_back();
        }
        out["member_record_indices"] = std::move(indices);
        out["nodes"] = std::move(nodes);
        out["end_counter"] = counter;
        out["status"] = "resolved";
    } catch (const std::exception &e) {
        out["error"] = e.what();
    }
    return out;
}
} // namespace p3d
