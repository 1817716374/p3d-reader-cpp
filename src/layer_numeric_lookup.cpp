#include "native_layer_source.hpp"
#include <p3d/layer_lookup.hpp>

namespace p3d {
NativeLayerNumericIndex::NativeLayerNumericIndex(const Json &records, std::size_t table_index)
    : data_({{"status", "unresolved"},
             {"source_profile", "fresh_saved_local_layer_table"},
             {"table_record_index", table_index},
             {"entries", Json::array()},
             {"rejected_input_records", Json::array()},
             {"parent_selection", "not_evaluated"},
             {"material_loading", "not_evaluated"}}) {
    try {
        auto source = detail::read_saved_local_layers(records, table_index);
        std::map<std::uint32_t, Json> entries;
        std::map<std::uint32_t, std::pair<std::uint32_t, std::size_t>> ids;
        for (const auto &[id, row] : source.by_id) {
            const auto key = row.layer.at("numeric_key").at("value").get<std::uint32_t>();
            ids[id] = {key, row.source};
            auto &entry = entries[key];
            if (entry.is_null())
                entry = {{"numeric_key", key}, {"candidate_record_indices", Json::array()}};
            entry["candidate_record_indices"].push_back(row.source);
            entry["layer_id"] = id;
            entry["native_record_index"] = row.source;
        }
        Json pool = Json::array();
        std::map<std::uint32_t, std::size_t> indices;
        for (auto &[key, entry] : entries) {
            indices[key] = pool.size();
            pool.push_back(std::move(entry));
        }
        keys_ = std::move(indices);
        ids_ = std::move(ids);
        data_.update({{"status", "resolved"},
                      {"selection", "last_unsigned_layer_id_for_numeric_key"},
                      {"entries", std::move(pool)},
                      {"rejected_input_records", std::move(source.rejected)}});
    } catch (const std::exception &e) {
        data_["reason"] = e.what();
    }
}
Json NativeLayerNumericIndex::lookup(std::uint32_t key) const {
    Json out = {{"status", "unresolved"},
                {"numeric_key", key},
                {"table_record_index", data_.at("table_record_index")}};
    if (data_.at("status") != "resolved") {
        out["reason"] = data_.at("reason");
        return out;
    }
    const auto found = keys_.find(key);
    if (found == keys_.end()) {
        out["status"] = "not_found";
        return out;
    }
    out.update(data_.at("entries").at(found->second));
    out["entry_index"] = found->second;
    out["status"] = "matched";
    return out;
}
Json NativeLayerNumericIndex::remap_offset(std::uint32_t id, std::uint32_t offset,
                                           std::optional<std::uint32_t> parent_layer_id) const {
    Json out = {{"status", "unresolved"},
                {"scope", "native_parent_layer_offset_branch"},
                {"input_layer_id", id},
                {"offset", offset},
                {"table_record_index", data_.at("table_record_index")},
                {"parent_selection", "not_evaluated"},
                {"material_loading", "not_evaluated"}};
    if (id == 0) {
        out.update({{"status", "resolved"}, {"selection", "zero_layer_bypass"}, {"layer_id", 0}});
        return out;
    }
    if (data_.at("status") != "resolved") {
        out["reason"] = data_.at("reason");
        return out;
    }
    const auto source = ids_.find(id);
    if (source == ids_.end()) {
        out.update({{"status", "resolved"},
                    {"selection", "source_layer_missing_fallback"},
                    {"layer_id", 64}});
        return out;
    }
    const auto key = std::uint32_t(source->second.first + offset);
    const auto match = lookup(key);
    out["source_native_record_index"] = source->second.second;
    out["source_numeric_key"] = source->second.first;
    out["target_numeric_key"] = key;
    out["numeric_lookup"] = match;
    if (match.at("status") == "matched" && match.at("layer_id") == UINT32_MAX - 1) {
        out["selection"] = "mapped_layer_requires_parent_value";
        if (!parent_layer_id) {
            out["reason"] = "parent_layer_id_unknown";
            return out;
        }
        out.update({{"status", "resolved"},
                    {"selection", "mapped_layer_uses_parent_value"},
                    {"layer_id", *parent_layer_id}});
        return out;
    }
    out.update({{"status", "resolved"},
                {"selection", match.at("status") == "matched" ? "numeric_key_match"
                                                              : "target_key_missing_fallback"},
                {"layer_id", match.at("status") == "matched" ? match.at("layer_id") : Json(64)}});
    return out;
}
} // namespace p3d
