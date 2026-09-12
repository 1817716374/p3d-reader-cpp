#include "internal.hpp"

namespace p3d {
Json native_reference_path(const Bytes &base, const Json &links) {
    Json out = {{"encoding", "native_owner_reference_path"},
                {"reader_profile", "bimbase_2025_reference_path_input"},
                {"scope", "path_program_if_record_is_resolved"},
                {"status", "invalid"},
                {"runtime_resolution", "not_evaluated"},
                {"matching_links", Json::array()},
                {"steps", Json::array()}};
    try {
        require(base.size() >= 38 && Reader(base, 4).u16() == 47 && Reader(base, 16).u32() == 20 &&
                    Reader(base, 36).u16() == 0x56e6,
                "reference path record requires type 47/subtype 20/signature 0x56e6");
        out["signature_source_offset"] = 36;
        std::optional<std::size_t> selected;
        bool uncertain_key = false;
        for (std::size_t i = 0; i < links.size(); ++i) {
            const auto &link = links[i];
            if (link.at("app") != 0x56d0 || !(link.at("header").get<unsigned>() & 0x1000))
                continue;
            const auto p = bytesof(link.at("payload"));
            if (p.size() >= 2 && Reader(p).u16() != 10000)
                continue;
            if (p.size() < 4) {
                // A short key before the first match does not justify choosing a
                // later link; the native callback would read beyond this payload.
                if (!selected)
                    uncertain_key = true;
                out["matching_links"].push_back(
                    {{"linkage_index", i}, {"selection", "incomplete_key"}});
                continue;
            }
            if (Reader(p, 2).u16() != 4)
                continue;
            const bool first = !selected && !uncertain_key;
            out["matching_links"].push_back(
                {{"linkage_index", i},
                 {"selection", first                        ? "first_match"
                               : uncertain_key && !selected ? "undetermined"
                                                            : "shadowed"}});
            if (first)
                selected = i;
        }
        require(!uncertain_key, "incomplete dependency key before path selection");
        require(selected.has_value(), "reference path linkage not found");
        const auto &link = links[*selected];
        out["selected_linkage_index"] = *selected;
        out["selected_linkage_offset"] = link.value("offset", Json());
        const auto p = bytesof(link.at("payload"));
        require(p.size() >= 8, "truncated selected reference path header");
        const auto flags = Reader(p, 4).u16(), count = Reader(p, 6).u16();
        const unsigned format = (flags >> 10) & 15;
        out.update({{"flags", flags},
                    {"entry_count", count},
                    {"reference_format", format},
                    {"dependency_disable_bit_applied", false},
                    {"failure_rule", "stop_on_first_unresolved_step"}});
        Json steps = Json::array();
        auto step = [&](std::size_t i, std::size_t offset, bool terminal) {
            steps.push_back({{"source_id_index", i},
                             {"payload_offset", offset},
                             {"element_id", Reader(p, offset).u64()},
                             {"operation", terminal ? "resolve_and_append_target"
                                                    : "resolve_and_expand_reference"},
                             {"lookup_profile", "owner_system"},
                             {"runtime_reject_mask", 8}});
            if (terminal)
                steps.back()["append_rule"] = "first_ancestor_without_child_flag_then_target";
            else
                steps.back()["owner_update"] = "use_expanded_reference_owner";
        };
        if (format == 0) {
            require(count <= (p.size() - 8) / 8, "truncated selected reference ID list");
            for (std::size_t i = count; i > 0; --i)
                step(i - 1, 8 + (i - 1) * 8, false);
            if (count == 0)
                out["empty_path_owner"] = "keep_existing_or_use_current";
        } else if (format == 6) {
            require(count == 1, "variable reference path requires one entry");
            require(p.size() >= 24, "truncated variable reference path header");
            const auto n = Reader(p, 8).u32();
            out["path_id_count"] = n;
            require(n >= 1 && n <= INT32_MAX && n <= (p.size() - 24) / 8,
                    "invalid variable reference path ID count");
            for (std::size_t i = n; i > 1; --i)
                step(i - 1, 24 + (i - 1) * 8, false);
            step(0, 24, true);
            out["terminal_owner_update"] = "none";
        } else {
            throw std::runtime_error("unsupported reference path format");
        }
        out["steps"] = std::move(steps);
        out["status"] = "resolved";
    } catch (const std::exception &e) {
        out["error"] = e.what();
    }
    return out;
}
} // namespace p3d
