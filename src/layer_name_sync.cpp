#include "layer_name_key.hpp"

namespace p3d {
namespace {
struct IndexedName {
    std::uint32_t layer_id;
    std::u16string name;
};
using Bucket = std::vector<IndexedName>;
std::optional<std::size_t> find_name(const Bucket &bucket, const std::u16string &name,
                                     const NativeLayerNameContext &context) {
    for (std::size_t i = 0; i < bucket.size(); ++i)
        if (detail::layer_name_equal(bucket[i].name, name, context))
            return i;
    return {};
}
Json units(const std::u16string &name) {
    return std::vector<std::uint16_t>(name.begin(), name.end());
}
} // namespace
Json evaluate_native_layer_name_sync(const std::vector<NativeLayerNameCacheEntry> &input,
                                     const std::vector<NativeLayerNameSyncRequest> &requests,
                                     bool initial_name_index_populated,
                                     const NativeLayerNameContext &context) {
    Json out = {{"status", "not_evaluated"},
                {"source_profile", "prepared_group_name_cache_after_membership_alignment"},
                {"operations", Json::array()},
                {"layers", nullptr},
                {"initial_name_index_populated", initial_name_index_populated},
                {"resource_registration", "logical_name_keys_only"},
                {"runtime_notifications", "not_evaluated"}};
    std::map<std::uint32_t, NativeLayerNameCacheEntry> layers;
    std::map<detail::LayerNameKey, Bucket> index;
    bool indexed = false;
    bool index_populated = initial_name_index_populated;
    std::size_t position = 0;
    try {
        for (auto layer : input) {
            require(layer.layer_id != 0 && layer.layer_id != UINT32_MAX,
                    "prepared_layer_id_invalid");
            layer.name = detail::layer_name_prefix(std::move(layer.name));
            require(!layer.name_registered || !layer.name.empty(), "registered_layer_name_empty");
            require(layers.emplace(layer.layer_id, std::move(layer)).second,
                    "prepared_layer_id_ambiguous");
        }
        require(!initial_name_index_populated || !layers.empty(),
                "prepared_name_index_population_mismatch");
        for (; position < requests.size(); ++position) {
            const auto &request = requests[position];
            const auto found = layers.find(request.layer_id);
            require(found != layers.end(), "name_sync_target_not_in_prepared_cache");
            auto &layer = found->second;
            const auto name = detail::layer_name_prefix(request.name);
            Json operation = {{"request_index", position},
                              {"layer_id", request.layer_id},
                              {"previous_name_utf16", units(layer.name)},
                              {"requested_name_utf16", units(name)},
                              {"name_index_updated", false},
                              {"changed", false}};
            auto finish = [&](unsigned code, const char *selection) {
                operation.update({{"status", "decoded"},
                                  {"native_return_code", code},
                                  {"selection", selection}});
                out["operations"].push_back(std::move(operation));
            };
            // Native equality precedes validation, registration and index work.
            if (name == layer.name) {
                finish(0, "unchanged");
                continue;
            }
            if (name.empty() || name.size() > 512 ||
                name.find_first_of(u"<>\\/\"?*|,=&\n\t") != std::u16string::npos) {
                finish(8, "invalid_name");
                continue;
            }
            const auto new_key = detail::layer_name_key(name, context);
            std::optional<detail::LayerNameKey> old_key;
            if (layer.name_registered)
                old_key = detail::layer_name_key(layer.name, context);
            const bool same_registered_name = old_key && *old_key == new_key &&
                                              detail::layer_name_equal(layer.name, name, context);
            // The conflict query builds an empty index. Equal registered name
            // keys bypass that query, so a case-only change need not build it.
            if (!same_registered_name)
                index_populated = true;
            if (!index_populated) {
                layer.name = name;
                operation["changed"] = true;
                finish(0, "renamed");
                continue;
            }
            if (!indexed) {
                for (const auto &[id, entry] : layers) {
                    if (!entry.name_registered)
                        continue;
                    auto &bucket = index[detail::layer_name_key(entry.name, context)];
                    const auto match = find_name(bucket, entry.name, context);
                    if (match)
                        bucket[*match] = {id, entry.name};
                    else
                        bucket.push_back({id, entry.name});
                }
                indexed = true;
            }
            // Inspect all potentially unknown comparisons before mutating state.
            auto &new_bucket = index[new_key];
            auto new_match = find_name(new_bucket, name, context);
            if (!same_registered_name && new_match &&
                new_bucket[*new_match].layer_id != request.layer_id) {
                operation["conflicting_layer_id"] = new_bucket[*new_match].layer_id;
                finish(9, "name_conflict");
                continue;
            }
            std::optional<std::size_t> old_match;
            if (old_key)
                old_match = find_name(index[*old_key], layer.name, context);
            if (old_match) {
                auto &old_bucket = index[*old_key];
                old_bucket.erase(old_bucket.begin() + *old_match);
                if (*old_key == new_key && new_match) {
                    if (*old_match == *new_match)
                        new_match.reset();
                    else if (*old_match < *new_match)
                        --*new_match;
                }
            }
            if (new_match)
                new_bucket[*new_match] = {request.layer_id, name};
            else
                new_bucket.push_back({request.layer_id, name});
            layer.name = name;
            layer.name_registered = true;
            operation["changed"] = true;
            operation["name_index_updated"] = true;
            finish(0, "renamed");
        }
        out["layers"] = Json::array();
        for (const auto &[id, layer] : layers)
            out["layers"].push_back({{"layer_id", id},
                                     {"name_utf16", units(layer.name)},
                                     {"name_registered", layer.name_registered}});
        out["status"] = "evaluated";
        out["name_index_populated"] = index_populated;
    } catch (const std::exception &e) {
        out["status"] = out["operations"].empty() ? "not_evaluated" : "partially_evaluated";
        out["failed_request_index"] = position;
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
