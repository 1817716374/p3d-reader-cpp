#include "layer_name_key.hpp"
#include "native_layer_source.hpp"

namespace p3d {
namespace {
Json units(const std::u16string &s) {
    return std::vector<std::uint16_t>(s.begin(), s.end());
}
struct NameRegistry {
    NativeLayerNameContext text;
    std::vector<std::u16string> names;
    std::map<detail::LayerNameKey, std::vector<std::uint32_t>> buckets;
    std::optional<std::uint32_t> find_key(const std::u16string &name,
                                          detail::LayerNameKey key) const {
        const auto bucket = buckets.find(key);
        if (bucket == buckets.end())
            return {};
        for (auto id : bucket->second)
            if (detail::layer_name_equal(names.at(id), name, text))
                return id;
        return {};
    }
    std::optional<std::uint32_t> find(const std::u16string &name) const {
        if (name.empty() || names.empty())
            return {};
        return find_key(name, detail::layer_name_key(name, text));
    }
    std::optional<std::uint32_t> intern(const std::u16string &name) {
        if (name.empty())
            return {};
        const auto key = detail::layer_name_key(name, text);
        if (const auto existing = find_key(name, key))
            return existing;
        require(names.size() < UINT32_MAX, "layer_name_registry_capacity_unknown");
        const auto id = std::uint32_t(names.size());
        names.push_back(name);
        buckets[key].push_back(id);
        return id;
    }
};
struct LoadedName {
    std::size_t source;
    std::u16string name;
    std::uint32_t registry_key;
    std::string selection;
};
using Nodes = std::map<std::uint32_t, LoadedName>;
using NameIndex = std::map<std::uint32_t, std::uint32_t>;
NameIndex rebuild(const Nodes &nodes) {
    NameIndex out;
    for (const auto &[id, node] : nodes)
        out[node.registry_key] = id;
    return out;
}
std::optional<std::uint32_t> lookup_loading(const std::u16string &name,
                                            const NameRegistry &registry, const Nodes &nodes,
                                            NameIndex &index) {
    const auto key = registry.find(name);
    if (!key)
        return {};
    if (index.empty())
        index = rebuild(nodes);
    const auto found = index.find(*key);
    if (found == index.end() || !nodes.count(found->second))
        return {};
    return found->second;
}
} // namespace
struct NativeLayerNameCache::Impl {
    Json data;
    NameRegistry registry;
    Nodes nodes;
    NameIndex captured, query_index;
};
NativeLayerNameCache::NativeLayerNameCache(const Json &records, std::size_t table_index,
                                           const NativeLayerNameLoadContext &context) {
    auto state = std::make_shared<Impl>();
    state->data = {{"status", "unresolved"},
                   {"source_profile", "fresh_writable_saved_name_loader_without_id_repair"},
                   {"table_record_index", table_index},
                   {"loaded_layers", nullptr},
                   {"name_index_entries", nullptr},
                   {"registered_names_after_load", nullptr},
                   {"load_steps", Json::array()},
                   {"later_cache_invalidation", "not_evaluated"},
                   {"group_synchronization", "not_evaluated"}};
    state->registry.text = context.text;
    try {
        bool registry_initialized = false;
        auto initialize_registry = [&] {
            if (registry_initialized)
                return;
            if (context.registered_names_before_load)
                for (const auto &name : *context.registered_names_before_load)
                    state->registry.intern(detail::layer_name_prefix(name));
            registry_initialized = true;
        };
        detail::for_each_saved_layer_input(
            records, table_index, true, [&](detail::SavedLayerRow row) {
                initialize_registry();
                const auto id = row.layer.at("layer_id").get<std::uint32_t>();
                Json step = {{"native_record_index", row.source}, {"layer_id", id}};
                // Every saved name linkage registers, even an overwritten name or
                // a record whose layer ID will subsequently be rejected.
                for (const auto &text : row.layer.at("strings"))
                    if (text.value("role", "") == "name")
                        state->registry.intern(
                            detail::layer_name_wide(text.at("text").get<std::string>()));
                if (id == 0 || id == UINT32_MAX) {
                    step.update({{"status", "rejected"}, {"native_return_code", 0x5c}});
                    state->data["load_steps"].push_back(std::move(step));
                    return;
                }
                std::u16string name;
                std::string selection;
                const auto &initial = row.layer.at("initial_name");
                if (initial.at("status") == "decoded") {
                    name = detail::layer_name_wide(initial.at("value").get<std::string>());
                    selection = initial.at("selection").get<std::string>();
                } else {
                    require(initial.value("selection", "") == "generated_name_required",
                            "layer_initial_name_unknown");
                    require(context.registered_names_before_load.has_value(),
                            "layer_name_generation_initial_registry_unknown");
                    const auto signed_id =
                        id <= INT32_MAX ? std::int64_t(id) : std::int64_t(id) - 0x100000000ll;
                    const auto decimal = std::to_string(signed_id);
                    std::u16string candidate(decimal.begin(), decimal.end());
                    const auto count = state->nodes.size();
                    require(count < SIZE_MAX, "layer_name_generation_query_limit_overflow");
                    std::size_t queries = 0;
                    Json collisions = Json::array();
                    for (; queries <= count; ++queries) {
                        const auto occupied = lookup_loading(candidate, state->registry,
                                                             state->nodes, state->captured);
                        if (!occupied) {
                            name = std::move(candidate);
                            ++queries;
                            break;
                        }
                        collisions.push_back(*occupied);
                        candidate.push_back(u'-');
                    }
                    step["candidate_query_count"] = queries;
                    step["collision_layer_ids"] = std::move(collisions);
                    selection = "generated_name";
                }
                const auto key = state->registry.intern(name);
                if (!key) {
                    step.update({{"status", "rejected"}, {"native_return_code", 8}});
                } else {
                    state->nodes[id] = {row.source, name, *key, selection};
                    step.update({{"status", "loaded"},
                                 {"native_return_code", 0},
                                 {"name_utf16", units(name)},
                                 {"selection", selection}});
                }
                state->data["load_steps"].push_back(std::move(step));
            });
        initialize_registry();
        // Later const queries rebuild only if the captured map was empty.
        // A nonempty map can intentionally retain aliases to earlier payloads.
        state->query_index = state->captured.empty() ? rebuild(state->nodes) : state->captured;
        Json loaded = Json::array(), captured = Json::array(), registered = Json::array();
        for (const auto &[id, node] : state->nodes)
            loaded.push_back({{"layer_id", id},
                              {"native_record_index", node.source},
                              {"name_utf16", units(node.name)},
                              {"name_registered", true},
                              {"selection", node.selection}});
        for (const auto &[key, id] : state->captured)
            captured.push_back({{"key_name_utf16", units(state->registry.names.at(key))},
                                {"layer_id", id},
                                {"native_record_index", state->nodes.at(id).source}});
        if (context.registered_names_before_load)
            for (const auto &name : state->registry.names)
                registered.push_back(units(name));
        state->data.update(
            {{"status", "resolved"},
             {"loaded_layers", std::move(loaded)},
             {"name_index_entries", std::move(captured)},
             {"name_index_populated_during_load", !state->captured.empty()},
             {"lookup_index_source",
              state->captured.empty() ? "rebuilt_after_load" : "captured_during_load"},
             {"registered_names_after_load",
              context.registered_names_before_load ? std::move(registered) : Json()},
             {"initial_registry_complete", context.registered_names_before_load.has_value()}});
    } catch (const std::exception &e) {
        state->data["reason"] = e.what();
    }
    impl_ = std::move(state);
}
const Json &NativeLayerNameCache::metadata() const noexcept {
    return impl_->data;
}
Json NativeLayerNameCache::lookup(const std::u16string &input) const {
    Json out = {{"status", "unresolved"},
                {"table_record_index", impl_->data.at("table_record_index")}};
    try {
        const auto name = detail::layer_name_prefix(input);
        if (name.empty()) {
            out["status"] = "not_found";
            return out;
        }
        if (impl_->data.at("status") != "resolved") {
            out["reason"] = impl_->data.at("reason");
            return out;
        }
        const auto key = impl_->registry.find(name);
        const auto found = key ? impl_->query_index.find(*key) : impl_->query_index.end();
        if (found == impl_->query_index.end()) {
            out["status"] = "not_found";
            return out;
        }
        const auto &node = impl_->nodes.at(found->second);
        out.update({{"status", "matched"},
                    {"layer_id", found->second},
                    {"native_record_index", node.source},
                    {"name_utf16", units(node.name)},
                    {"lookup_index_source", impl_->data.at("lookup_index_source")}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
