#include "internal.hpp"
namespace p3d {
Json material_map_bindings(const Json &maps) {
    enum State { Pending, Resolved, Dangling, Invalid, Cycle };
    struct Node {
        std::int64_t type = 0, link = 0, unresolved = 0;
        std::size_t terminal = 0;
        State state = Pending;
        bool cleared = false;
    };
    std::vector<Node> nodes(maps.size());
    std::vector<std::vector<std::size_t>> incoming(maps.size());
    // Native keys have signed int ordering, even though XML Type is uint32.
    std::map<std::int64_t, std::size_t> active;
    Json entries = Json::array();
    bool incomplete = false;
    for (std::size_t i = 0; i < maps.size(); ++i) {
        const auto &map = maps[i];
        const auto &type = map["semantics"]["type"];
        Json entry = {{"map_index", i}, {"selection", "active"}};
        if (!map.value("native_table_member", false))
            entry["selection"] = "outside_native_table";
        else if (type["status"] != "decoded") {
            const bool failed_read =
                type["status"] == "missing" ||
                type.value("conversion", Json::object()).value("status", Json()) ==
                    "no_integer_assignment";
            entry["selection"] = failed_read ? "skipped_type_read_failure" : "invalid_type";
            incomplete |= !failed_read;
        } else if (type["value"] == 0)
            entry["selection"] = "ignored_zero_type";
        else {
            auto value = type["value"].get<std::uint32_t>();
            auto key =
                value <= INT32_MAX ? std::int64_t(value) : std::int64_t(value) - 0x100000000LL;
            nodes[i].type = key;
            entry["native_type_key"] = key;
            auto old = active.find(key);
            if (old != active.end()) {
                entries[old->second]["selection"] = "superseded";
                entries[old->second]["replaced_by_map_index"] = i;
            }
            active[key] = i;
        }
        entries.push_back(std::move(entry));
    }
    std::vector<std::size_t> seeds;
    for (const auto &item : active) {
        auto i = item.second;
        auto &n = nodes[i];
        const auto &link = maps[i]["semantics"]["map_link"];
        entries[i]["source_link"] = link;
        const bool failed_read = link.value("conversion", Json::object()).value("status", Json()) ==
                                 "no_integer_assignment";
        if (link["status"] != "decoded" && link["status"] != "missing" && !failed_read) {
            n.state = Invalid;
            incomplete = true;
        } else {
            n.link =
                link["status"] == "missing" || failed_read ? 0 : link["value"].get<std::int64_t>();
            entries[i]["link_normalization"] = link["status"] == "missing" ? "missing_uses_zero"
                                               : failed_read               ? "failed_read_uses_zero"
                                               : n.link == n.type          ? "self_link_uses_zero"
                                                                           : "source_value";
            if (n.link == n.type)
                n.link = 0;
            entries[i]["normalized_link_type"] = n.link;
            if (!n.link) {
                n.state = Resolved;
                n.terminal = i;
            } else {
                auto target = active.find(n.link);
                if (target == active.end()) {
                    n.state = Dangling;
                    n.unresolved = n.link;
                } else
                    incoming[target->second].push_back(i);
            }
        }
        if (n.state != Pending)
            seeds.push_back(i);
    }
    // Classify every acyclic component once. Unvisited components lead to a
    // cycle. Reverse propagation also avoids quadratic rescans of long chains.
    for (std::size_t p = 0; p < seeds.size(); ++p) {
        auto i = seeds[p];
        for (auto source : incoming[i])
            if (nodes[source].state == Pending) {
                nodes[source].state = nodes[i].state;
                nodes[source].terminal = nodes[i].terminal;
                nodes[source].unresolved = nodes[i].unresolved;
                seeds.push_back(source);
            }
    }
    for (const auto &item : active)
        if (nodes[item.second].state == Pending)
            nodes[item.second].state = Cycle;
    // The native XML reader clears dangling links in signed type order.
    // Clearing one entry can make its unprocessed predecessors resolve to it.
    for (const auto &item : active) {
        auto i = item.second;
        if (nodes[i].state != Dangling)
            continue;
        nodes[i].cleared = true;
        nodes[i].link = 0;
        seeds.assign(1, i);
        nodes[i].state = Resolved;
        nodes[i].terminal = i;
        for (std::size_t p = 0; p < seeds.size(); ++p)
            for (auto source : incoming[seeds[p]])
                if (nodes[source].state == Dangling) {
                    nodes[source].state = Resolved;
                    nodes[source].terminal = i;
                    seeds.push_back(source);
                }
    }
    for (const auto &item : active) {
        auto i = item.second;
        const auto &n = nodes[i];
        auto &entry = entries[i];
        if (n.state != Resolved) {
            entry["status"] = n.state == Cycle ? "cyclic_link_chain" : "invalid_link_chain";
            incomplete = true;
            continue;
        }
        entry["status"] = n.cleared ? "dangling_link_cleared" : n.link ? "linked" : "local";
        entry["effective_link_type"] = n.link;
        if (n.cleared)
            entry["unresolved_target_type"] = n.unresolved;
        entry["direct_target_map_index"] = n.link ? Json(active.at(n.link)) : Json();
        entry["terminal_map_index"] = n.terminal;
        const bool local_layers = n.type == 30 || n.terminal == i;
        entry["texture_layers_source_map_index"] = local_layers ? i : n.terminal;
        entry["texture_mapping_source_map_index"] = n.terminal;
        entry["projection_frame_source_map_index"] = n.terminal;
        entry["texture_layers_mode"] = n.terminal == i ? "local"
                                       : n.type == 30  ? "local_with_linked_mapping"
                                                       : "native_shared";
        // The map-level activation, weight and explicit projection matrix are
        // separate from the linked layer container and projection frame.
        entry["map_settings_source_map_index"] = i;
    }
    return {{"status", incomplete ? "incomplete" : "resolved"},
            {"entries", entries},
            {"scope", "direct_material_children"},
            {"duplicate_type_policy", "last_occurrence_replaces_previous"},
            {"dangling_link_policy", "clear_in_signed_type_order"},
            {"cycle_policy", "report_without_native_unbounded_traversal"},
            {"texture_evaluation", "not_evaluated"}};
}
} // namespace p3d
