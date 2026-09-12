#include "internal.hpp"

namespace p3d {
static unsigned minimum_list_base_bytes(unsigned type, std::uint32_t subtype, bool extended,
                                        bool dimension_bit) {
    const auto d = unsigned(dimension_bit);
    switch (type) {
    case 10:
        return subtype == 1 || subtype == 5 ? 0x40 : subtype == 6 || subtype == 9 ? 0x30 : 0x28;
    case 11:
        return 0x114;
    case 13:
        return 0x170;
    case 14:
        return 0x3c;
    case 15:
        return 0x80;
    case 17:
    case 33:
        return 0x34;
    case 18:
    case 19:
    case 22:
        return 0x70;
    case 20:
        return 0xa8 + 32 * d;
    case 21:
        return 64 * (3 + d);
    case 23:
    case 56:
        return 0x38;
    case 24:
    case 26:
    case 27:
    case 35:
    case 36:
        return 0x78;
    case 25:
        return 0xa4;
    case 28:
    case 29:
        return 0x72;
    case 30:
        return 0x90;
    case 32:
    case 62:
        return 0x100;
    case 37:
        return 0x88 + 16 * d;
    case 38:
        return 0x90 + 32 * d;
    case 40:
    case 41:
    case 44:
    case 53:
    case 57:
        return 0x80 + 8 * d;
    case 48:
    case 55:
        return 0x28;
    case 49: {
        static constexpr unsigned sizes[] = {0,     0x2e, 0xea, 0,    0x310,
                                             0x140, 0x60, 0xcc, 0x28, 0x68};
        return subtype >= 1 && subtype <= 10 ? sizes[subtype - 1] : 0x10;
    }
    case 50:
        return 0x74;
    case 51:
        return 0xd0;
    case 52:
        return 32 * (5 + d);
    case 54:
        return 0xaa + 32 * d;
    case 58:
        return 0x20;
    case 59:
        return 0x13c;
    case 60:
        return 0x160;
    case 63:
        return 0x120;
    default:
        return extended ? 0x34 : 0x10;
    }
}

Json native_list_record_header(const Json &record, const Json &conversion, bool child,
                               bool compound, std::uint32_t descendants, bool system) {
    Json out = {{"scope", "file_load_list_header_preparation"}, {"status", "unresolved"}};
    try {
        const auto data = bytesof(record.at("data"));
        const auto type = record.at("element_type").get<unsigned>();
        auto flags = record.at("element_flags").get<unsigned>();
        auto words = Reader(data, 8).u32(), base_words = Reader(data, 12).u32();
        if (conversion.is_object()) {
            flags = conversion.value("output_element_flags", flags);
            words = conversion.value("output_record_word_count", words);
            base_words = conversion.value("output_base_word_count", base_words);
        }
        out["input_element_flags"] = flags;
        out["input_record_word_count"] = words;
        out["input_base_word_count"] = base_words;
        Json validation = {{"return_code", 0}, {"return_code_ignored_by_file_loader", true}};
        if (type == 0) {
            validation["return_code"] = 0x11002;
        } else if (words > 65535) {
            validation["return_code"] = 0x11015;
        } else {
            const bool extended = (flags & 0x20) != 0;
            const bool dimension = extended && (Reader(data, 36).u16() & 0x1000) != 0;
            const auto minimum =
                minimum_list_base_bytes(type, Reader(data, 16).u32(), extended, dimension);
            validation["minimum_base_bytes"] = minimum;
            validation["extended_dimension_bit"] = extended ? Json(dimension) : Json();
            // The native validator updates base length before returning either
            // length error. File input ignores that return, but keeps the update.
            if (std::uint32_t(base_words * 2u) < minimum)
                base_words = minimum / 2;
            if (words * 2u < minimum)
                validation["return_code"] = 0x11016;
            else if (words < base_words)
                validation["return_code"] = 0x11017;
            else if (extended && !system && type != 54) {
                validation["return_code"] = nullptr;
                validation["dimension_match"] = "requires_model_context";
            }
        }
        // The file-input call sets the bypass flag: neither these error codes
        // nor the separate whole-chain dimensional test reject this subtree.
        flags &= ~8u;
        if (type >= 10 && type <= 32)
            flags |= 0x40;
        else if (type >= 33 && type <= 63)
            flags &= ~0x40u;
        if (child)
            flags |= 0x80;
        Json count;
        if (compound) {
            const unsigned offset = (flags & 0x20) ? 104 : 32; // native header, without prefix
            const auto bounded_bytes = std::min(std::uint32_t(words * 2u), 0x1fffeu);
            const bool fits = offset + 4 <= bounded_bytes;
            if (!fits)
                flags &= ~0x40u;
            count = {{"header_offset", offset},
                     {"written", fits},
                     {"accepted_descendant_count", descendants},
                     {"output_value",
                      fits ? Json(descendants <= 0x7fffffffu ? descendants : 0u) : Json()}};
        }
        if (!child)
            flags &= ~0x80u;
        out.update({{"status", "resolved"},
                    {"validation", validation},
                    {"output_element_flags", flags},
                    {"output_record_word_count", words},
                    {"output_base_word_count", base_words},
                    {"descendant_count_update", count}});
    } catch (const std::exception &e) {
        out["error"] = e.what();
    }
    return out;
}

Json native_list_input_preparation(const Json &container, const Json &records) {
    const bool system = container.at("kind") == "P3D-SSYS";
    bool initialized = !system;
    Json out = {{"scope", "empty_list_before_runtime_registration"},
                {"status", "resolved"},
                {"system_bootstrap_required", system},
                {"bootstrap_root_record_index", nullptr},
                {"roots", Json::array()},
                {"skipped_roots", Json::array()},
                {"runtime_registration", "not_evaluated"}};
    for (const auto &order : container.at("input_order")) {
        const auto &block = container.at("blocks").at(order.at("block_index").get<std::size_t>());
        if (!block.contains("record_input"))
            break; // Invalid header: the loader never calls the record reader.
        const auto &input = block.at("record_input");
        for (const auto &root : input.at("roots")) {
            const auto ni = root.at("native_record_index").get<std::size_t>();
            const auto &n = records.at(ni);
            if (!initialized) {
                const auto bytes = bytesof(n.at("data"));
                if (n.at("element_type") != 46 || Reader(bytes, 16).u32() != 8 ||
                    (n.at("element_flags").get<unsigned>() & 8)) {
                    out["skipped_roots"].push_back(
                        {{"native_record_index", ni},
                         {"block_number", order.at("block_number")},
                         {"reason", "system_list_requires_initial_type_46_subtype_8"}});
                    continue;
                }
                out["bootstrap_root_record_index"] = ni;
                initialized = true;
            }
            const auto &tree = root.at("input_tree");
            std::map<std::size_t, std::uint32_t> counts;
            std::map<std::size_t, Json> conversions;
            if (tree.at("root_topology").contains("conversion"))
                conversions[ni] = tree["root_topology"]["conversion"];
            for (const auto &c : tree.at("record_conversions"))
                conversions[c.at("native_record_index").get<std::size_t>()] = c;
            for (auto it = tree.at("nodes").rbegin(); it != tree.at("nodes").rend(); ++it) {
                const auto child = it->at("native_record_index").get<std::size_t>();
                const auto parent = it->at("parent_record_index").get<std::size_t>();
                counts[parent] += counts[child] + 1u;
            }
            Json headers = Json::array();
            auto append = [&](std::size_t index, Json parent, bool compound) {
                auto h =
                    native_list_record_header(records.at(index), conversions[index],
                                              !parent.is_null(), compound, counts[index], system);
                h["native_record_index"] = index;
                h["parent_record_index"] = parent;
                if (h["status"] != "resolved")
                    out["status"] = "partial";
                headers.push_back(std::move(h));
            };
            append(ni, Json(), tree["root_topology"]["compound"].get<bool>());
            for (const auto &node : tree.at("nodes"))
                append(node["native_record_index"].get<std::size_t>(), node["parent_record_index"],
                       node["compound"].get<bool>());
            out["roots"].push_back({{"native_record_index", ni},
                                    {"block_number", order.at("block_number")},
                                    {"headers", std::move(headers)}});
        }
        if (input.at("status") == "unresolved") {
            out["status"] = "partial";
            out["stop_reason"] = "unresolved_record_input";
            break; // Unknown earlier roots could change empty-list bootstrap state.
        }
    }
    out["system_bootstrap_found"] = system ? Json(initialized) : Json();
    return out;
}
} // namespace p3d
