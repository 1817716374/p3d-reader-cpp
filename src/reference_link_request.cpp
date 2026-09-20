#include "internal.hpp"
#include <p3d/reference_search.hpp>

namespace p3d {
namespace {
Json request_result(std::uint64_t link_id) {
    return {{"status", "unresolved"},
            {"scope", "ordinary_single_reference_request"},
            {"input_link_id", link_id},
            {"input_and_callbacks", "not_executed"},
            {"registry_and_lists", "not_mutated"},
            {"examined_active_entries", 0},
            {"examined_secondary_entries", 0}};
}
} // namespace

ReferenceLinkRegistryIndex::ReferenceLinkRegistryIndex(const Json &registry) {
    require(registry.at("status") == "resolved", "complete_current_link_registry_required");
    const auto &entries = registry.at("registry");
    require(entries.is_array(), "link_registry_array_required");
    indices_.reserve(entries.size());
    for (std::size_t i = 0; i < entries.size(); ++i) {
        const auto &value = entries[i].at("id");
        require(value.is_number_unsigned() ||
                    (value.is_number_integer() && value.get<std::int64_t>() >= 0),
                "link_registry_unsigned_id_required");
        require(indices_.emplace(value.get<std::uint64_t>(), i).second,
                "duplicate_link_registry_id");
    }
}

std::optional<std::size_t> ReferenceLinkRegistryIndex::entry_index(std::uint64_t id) const {
    const auto it = indices_.find(id);
    return it == indices_.end() ? std::nullopt : std::optional<std::size_t>(it->second);
}

Json reference_link_request(const ReferenceSearchContext &c, std::size_t host_index,
                            std::uint64_t link_id, const Json &registry) {
    try {
        return reference_link_request(c, host_index, link_id, ReferenceLinkRegistryIndex(registry));
    } catch (const std::exception &e) {
        auto out = request_result(link_id);
        out["reason"] = e.what();
        return out;
    }
}

Json reference_link_request(const ReferenceSearchContext &c, std::size_t host_index,
                            std::uint64_t link_id, const ReferenceLinkRegistryIndex &registry) {
    auto out = request_result(link_id);
    try {
        require(host_index < c.models.size(), "request_host_index_out_of_range");
        const auto registered = registry.entry_index(link_id);
        auto load = [&](bool insert) {
            out.update({{"status", "resolved"},
                        {"action", "input_reference"},
                        {"requires_native_input", true},
                        {"insert_registry_entry", insert},
                        {"registry_source_action", insert ? "capture_input_source" : "preserve"}});
            if (insert)
                out["registry_state_before_input"] = 0;
            return out;
        };
        // The native map miss inserts state zero and the new source pointer,
        // then inputs directly. Existing lists are not examined on this branch.
        if (!registered)
            return load(true);
        out["registry_entry_index"] = *registered;
        const auto &host = c.models[host_index];
        auto find = [&](const std::vector<std::optional<std::size_t>> &links, bool absent,
                        bool complete, bool active) {
            const char *count = active ? "examined_active_entries" : "examined_secondary_entries";
            if (absent) {
                require(links.empty(), active ? "conflicting_active_list_state"
                                              : "conflicting_secondary_list_state");
                return false;
            }
            for (std::size_t i = 0; i < links.size(); ++i) {
                out[count] = i + 1;
                require(links[i].has_value(), "null_reference_request_list_entry");
                const auto index = *links[i];
                require(index < c.models.size(), "request_list_object_index_out_of_range");
                const auto &node = c.models[index];
                // Native accesses +260 directly: no kind, validity, connected
                // root, deleted flag, file-name or geometry comparisons here.
                require(node.reference_link_id.has_value(), "loaded_reference_link_id_required");
                if (*node.reference_link_id != link_id)
                    continue;
                out.update({{"status", "resolved"},
                            {"action", "reuse_reference"},
                            {"requires_native_input", false},
                            {"insert_registry_entry", false},
                            {"registry_source_action", "preserve"},
                            {"native_return", 0},
                            {"source_list", active ? "active" : "secondary"},
                            {"entry_index", i},
                            {"object_index", index}});
                return true;
            }
            require(complete, active ? "remaining_active_request_links_required"
                                     : "remaining_secondary_request_links_required");
            return false;
        };
        if (host.valid && link_id != 0 &&
            find(host.active_links, host.active_list_known_absent, host.active_links_complete,
                 true))
            return out;
        if (find(host.secondary_links, host.secondary_list_known_absent,
                 host.secondary_links_complete, false))
            return out;
        return load(false);
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
