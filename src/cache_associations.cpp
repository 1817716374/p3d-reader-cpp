#include "internal.hpp"
#include <p3d/proxy_cache.hpp>

namespace p3d {
namespace {
template <class T> const T &known(const std::optional<T> &value, const char *reason) {
    require(value.has_value(), reason);
    return *value;
}

std::uint32_t index(const Json &value) {
    require(value.is_number_integer() &&
                (value.is_number_unsigned() || value.get<std::int64_t>() >= 0) &&
                value.get<std::uint64_t>() <= UINT32_MAX,
            "invalid_association_source_index");
    return value.get<std::uint32_t>();
}

struct GroupKey {
    std::uint64_t reference;
    std::vector<std::uint64_t> entities;
    bool operator<(const GroupKey &other) const {
        if (reference != other.reference)
            return reference < other.reference;
        if (entities.size() != other.entities.size())
            return entities.size() < other.entities.size();
        return entities < other.entities;
    }
};
} // namespace

Json compare_native_edge_cache_entity_state(const Bytes &saved,
                                            const EdgeCacheEntityStateContext &context) {
    Json out = {{"status", "not_evaluated"},
                {"scope", "edge_cache_entity_state_gate"},
                {"display_test_status", "not_evaluated"}};
    auto finish = [&](bool match, const char *reason) {
        out.update({{"status", "resolved"}, {"state_matches", match}, {"reason", reason}});
        return out;
    };
    try {
        if (!known(context.entity_found, "entity_lookup_result_required"))
            return finish(false, "entity_not_found");
        if (known(context.runtime_flags, "entity_runtime_flags_required") & 0x20008u)
            return finish(false, "entity_runtime_flags_exclude_cache");
        require(saved.size() == 8, "saved_modification_time_requires_8_bytes");
        const auto current =
            known(context.last_modified_milliseconds, "current_modification_time_required");
        const auto prior = Reader(saved).f64();
        // UCOMISD followed by JP/JNE: NaNs never match, signed zeros do, and
        // equal infinities do. Do not truncate to the SDK's uint64 return type.
        return finish(prior == current,
                      prior == current ? "modification_time_matches" : "modification_time_differs");
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json select_native_edge_cache_associations(const Json &decoded,
                                           const std::vector<EdgeCacheAssociationTarget> &targets,
                                           ProxyCacheLimits limits) {
    Json out = {{"status", "not_evaluated"},
                {"scope", "native_edge_cache_association_set_selection"},
                {"cache_validity_status", "not_evaluated"}};
    try {
        require(decoded.at("status") == "decoded", "decoded_associations_required");
        const auto &source = decoded.at("associations");
        require(source.is_array() && source.size() == targets.size(),
                "association_target_count_mismatch");
        const auto &directory = decoded.at("reference_directory");
        require(directory.is_array(), "reference_directory_required");
        std::size_t budget = 0;
        auto consume = [&](std::size_t count) {
            require(count <= limits.max_entries - budget, "association_selection_entry_limit");
            budget += count;
        };
        consume(source.size());
        std::vector<bool> created(source.size()), retained(source.size());
        std::vector<std::set<std::uint32_t>> links(source.size());
        std::vector<std::optional<GroupKey>> keys(source.size());
        Json rows = Json::array();
        // Resolve creation outcomes before linking: forward edges may refer to
        // any created object, including one whose own link set later fails.
        for (std::size_t i = 0; i < source.size(); ++i) {
            require(index(source[i].at("association_index")) == i,
                    "association_source_order_mismatch");
            const auto &entities = source[i].at("entities"), &edges = source[i].at("links");
            require(entities.is_array() && edges.is_array(), "association_arrays_required");
            consume(entities.size());
            consume(edges.size());
            created[i] = known(targets[i].created, "association_creation_result_required");
            if (created[i]) {
                require(index(source[i].at("reference_directory_index")) < directory.size(),
                        "created_association_requires_reference_directory_entry");
                require(targets[i].entity_identities_complete &&
                            targets[i].entity_identities.size() == entities.size(),
                        "complete_resolved_entity_identities_required");
                keys[i] = GroupKey{
                    known(targets[i].reference_identity, "resolved_reference_identity_required"),
                    targets[i].entity_identities};
            }
            rows.push_back({{"association_index", i},
                            {"created", created[i]},
                            {"selected", false},
                            {"retained", false}});
        }
        std::map<GroupKey, std::size_t> selected;
        Json selected_indices = Json::array();
        for (std::size_t i = 0; i < source.size(); ++i) {
            auto &row = rows[i];
            if (!created[i]) {
                row["reason"] = "association_not_created";
                continue;
            }
            const auto &edges = source[i].at("links");
            std::size_t examined = 0;
            for (const auto &edge : edges) {
                ++examined;
                const auto target = index(edge.at("association_index"));
                if (target >= source.size() || !created[target])
                    break;
                if (target != i)
                    links[i].insert(target);
            }
            row["examined_links"] = examined;
            row["unique_nonself_targets"] = links[i].size();
            if (links[i].size() != edges.size()) {
                links[i].clear();
                row["reason"] = "link_count_mismatch";
                continue;
            }
            const auto inserted = selected.emplace(*keys[i], i);
            row["selected_source_index"] = inserted.first->second;
            row["selected"] = inserted.second;
            row["reason"] = inserted.second ? "selected" : "equivalent_group_already_selected";
            if (inserted.second)
                selected_indices.push_back(i);
            // The native ownership set receives even an equivalent candidate
            // and its links after insertion, regardless of insertion success.
            retained[i] = true;
            for (const auto target : links[i])
                retained[target] = true;
        }
        Json retained_indices = Json::array();
        for (std::size_t i = 0; i < source.size(); ++i) {
            rows[i]["retained"] = retained[i];
            if (retained[i])
                retained_indices.push_back(i);
            rows[i]["effective_link_indices"] = links[i];
        }
        out.update({{"status", "resolved"},
                    {"entries", std::move(rows)},
                    {"selected_source_indices", std::move(selected_indices)},
                    {"retained_source_indices", std::move(retained_indices)},
                    {"output_order", "source_order"}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
