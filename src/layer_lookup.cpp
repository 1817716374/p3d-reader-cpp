#include "internal.hpp"
#include "native_layer_source.hpp"
#include <p3d/layer_lookup.hpp>
#include <codecvt>
#include <locale>

namespace p3d {
namespace {
std::u16string prefix(std::u16string s) {
    s.resize(std::min(s.size(), s.find(u'\0')));
    return s;
}
std::u16string wide(const std::string &s) {
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> codec;
    auto text = codec.from_bytes(s);
    require(codec.converted() == s.size(), "layer_name_invalid_utf8");
    return prefix(std::move(text));
}
using Key = std::pair<std::uint32_t, std::uint32_t>;
Key key(const std::u16string &s, const NativeLayerNameContext &context) {
    require(bool(context.lowercase), "layer_name_source_lowercase_required");
    std::uint32_t hash = 0x1505;
    for (auto c : s) {
        const auto lower = context.lowercase(c);
        require(lower.has_value(), "layer_name_source_lowercase_unknown");
        hash = hash * 33u + std::uint16_t(*lower);
    }
    require(s.size() <= UINT32_MAX, "layer_name_length_out_of_range");
    return {hash, std::uint32_t(s.size())};
}
bool equal(const std::u16string &a, const std::u16string &b,
           const NativeLayerNameContext &context) {
    if (a == b)
        return true;
    require(bool(context.compare), "layer_name_source_comparison_required");
    auto result = context.compare(a, b);
    require(result.has_value(), "layer_name_source_comparison_unknown");
    return *result == 0;
}
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
        const auto query = prefix(input);
        if (query.empty()) {
            out["status"] = "not_found";
            return out;
        }
        auto source = detail::read_saved_local_layers(records, table_index);
        out["rejected_input_records"] = source.rejected;
        std::map<std::uint32_t, Layer> by_id;
        for (const auto &[id, row] : source.by_id)
            by_id[id] = {row.source, id, wide(row.layer.value("name", std::string())), {}};
        std::map<Key, std::vector<Layer>> names;
        for (auto &[id, layer] : by_id) {
            if (id == UINT32_MAX || layer.name.empty())
                continue;
            layer.name_key = key(layer.name, context);
            auto &bucket = names[layer.name_key];
            auto found = std::find_if(bucket.begin(), bucket.end(), [&](const Layer &other) {
                return equal(other.name, layer.name, context);
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
            const auto query_key = key(query, context);
            out["query_key"] = {{"hash", query_key.first}, {"utf16_length", query_key.second}};
            const auto bucket = names.find(query_key);
            if (bucket != names.end())
                for (const auto &layer : bucket->second)
                    if (equal(layer.name, query, context)) {
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
