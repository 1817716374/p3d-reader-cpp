#include "internal.hpp"
#include "native_layer_source.hpp"
#include "layer_name_key.hpp"
#include <p3d/layer_lookup.hpp>
#include <codecvt>
#include <locale>

namespace p3d {
std::u16string detail::layer_name_wide(const std::string &s) {
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> codec;
    auto text = codec.from_bytes(s);
    require(codec.converted() == s.size(), "layer_name_invalid_utf8");
    return detail::layer_name_prefix(std::move(text));
}
namespace {
using detail::layer_name_equal;
using detail::layer_name_key;
using detail::layer_name_prefix;
using Key = detail::LayerNameKey;
struct Layer {
    std::size_t source;
    std::uint32_t id;
    std::u16string name;
    Key name_key;
};
} // namespace
Json lookup_native_layer_name(const Json &records, std::size_t table_index,
                              const std::u16string &input, const NativeLayerNameContext &context) {
    Json out = {{"status", "unresolved"},
                {"source_profile", "fresh_saved_local_layer_table"},
                {"table_record_index", table_index},
                {"material_loading", "not_evaluated"},
                {"parent_selection", "not_evaluated"},
                {"name_index", Json::array()}};
    try {
        const auto query = layer_name_prefix(input);
        if (query.empty()) {
            out["status"] = "not_found";
            return out;
        }
        auto source = detail::read_saved_local_layers(records, table_index);
        out["rejected_input_records"] = source.rejected;
        std::map<std::uint32_t, Layer> by_id;
        for (const auto &[id, row] : source.by_id) {
            const auto &initial = row.layer.at("initial_name");
            require(initial.at("status") == "decoded", "layer_initial_name_requires_context");
            by_id[id] = {row.source,
                         id,
                         detail::layer_name_wide(initial.at("value").get<std::string>()),
                         {}};
        }
        std::map<Key, std::vector<Layer>> names;
        for (auto &[id, layer] : by_id) {
            if (id == UINT32_MAX || layer.name.empty())
                continue;
            layer.name_key = layer_name_key(layer.name, context);
            auto &bucket = names[layer.name_key];
            auto found = std::find_if(bucket.begin(), bucket.end(), [&](const Layer &other) {
                return layer_name_equal(other.name, layer.name, context);
            });
            if (found == bucket.end())
                bucket.push_back(layer);
            else
                *found = layer;
        }
        for (const auto &[name_key, bucket] : names)
            for (const auto &layer : bucket)
                out["name_index"].push_back({{"native_record_index", layer.source},
                                             {"layer_id", layer.id},
                                             {"hash", layer.name_key.first},
                                             {"utf16_length", layer.name_key.second}});
        if (!names.empty()) {
            const auto query_key = layer_name_key(query, context);
            out["query_key"] = {{"hash", query_key.first}, {"utf16_length", query_key.second}};
            const auto bucket = names.find(query_key);
            if (bucket != names.end())
                for (const auto &layer : bucket->second)
                    if (layer_name_equal(layer.name, query, context)) {
                        out.update({{"status", "matched"},
                                    {"native_record_index", layer.source},
                                    {"layer_id", layer.id}});
                        return out;
                    }
        }
        out["status"] = "not_found";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
