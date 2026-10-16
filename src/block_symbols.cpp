#include "line_style_source.hpp"
#include <p3d/block_symbols.hpp>
#include <p3d/line_style.hpp>

namespace p3d {
namespace {
constexpr const char *scope = "native_parent_color_weight_line_style_components";
Json identity(const Json &record) {
    Json out = Json::object();
    for (const auto key : {"stream", "offset", "id"})
        if (record.contains(key))
            out[key] = record.at(key);
    return out;
}
void status(Json &out) {
    bool known = out.at("line_style_parameters").at("status") == "decoded";
    for (const auto &field : out.at("values"))
        known = known && field.at("status") == "decoded";
    out["status"] = known ? "decoded" : "partial";
}
} // namespace
Json prepare_native_block_parent_symbols(const Json &instance, const Json &records,
                                         const NativeLayerNumericIndex &index) {
    Json out = {{"status", "unresolved"},
                {"scope", scope},
                {"instance_source", identity(instance)},
                {"valid_model_required", true},
                {"layer_component", "separate_block_layers_api"},
                {"line_style_resource_loading", "not_evaluated"},
                {"final_draw_style", "not_evaluated"},
                {"ancestor_merge", "not_evaluated"}};
    try {
        const auto body = bytesof(instance.at("data"));
        Reader r(body);
        require(r.at<std::uint16_t>(4) == 62, "block_symbols_require_type_62");
        const auto flags = r.at<std::uint16_t>(112);
        out["symbol_override_mask"] = flags & 0x38u;
        const auto layer_id = r.at<std::uint32_t>(16);
        auto lookup = index.lookup_layer(layer_id);
        out["instance_layer_lookup"] = lookup;
        Json layer;
        std::string layer_error;
        if (lookup.at("status") == "matched") {
            try {
                const auto source = lookup.at("native_record_index").get<std::size_t>();
                const auto &record = records.at(source);
                const auto raw = bytesof(record.at("data"));
                require(Reader(raw).at<std::uint16_t>(4) == 49 &&
                            Reader(raw).at<std::uint32_t>(36) == layer_id,
                        "selected_layer_source_mismatch");
                layer = decode_native_layer(raw, record.at("links"));
                require(layer.at("status") == "decoded" && !layer.contains("decode_error"),
                        "selected_layer_source_unknown");
            } catch (const std::exception &e) {
                layer_error = e.what();
                layer = nullptr;
            }
        }
        Json values = Json::object(), saved = Json::object();
        for (const auto &item : {std::make_pair("color", 56u), std::make_pair("line_weight", 52u),
                                 std::make_pair("line_style", 48u)}) {
            const std::string name = item.first;
            const auto value = r.at<std::uint32_t>(item.second);
            saved[name] = value;
            Json field = {{"status", "decoded"},
                          {"value", value},
                          {"source", "instance_header"},
                          {"source_offset", item.second},
                          {"source_record", identity(instance)}};
            const bool by_layer = value == (name == "line_style" ? 0x7fffffffu : UINT32_MAX);
            if (by_layer && lookup.at("status") == "not_found")
                field["selection"] = "missing_layer_preserves_source";
            else if (by_layer && !layer.is_null()) {
                const auto &symbols = layer.at("by_layer_symbology");
                field["value"] =
                    name == "line_style"
                        ? std::uint32_t(symbols.at(name).get<std::int32_t>())
                        : symbols.at(name == "color" ? "color_index" : name).get<std::uint32_t>();
                field["source"] = "instance_layer_by_layer_symbology";
                field["native_record_index"] = lookup.at("native_record_index");
                field.erase("source_record");
                field.erase("source_offset");
            } else if (by_layer) {
                field.erase("value");
                field["status"] = "unresolved";
                field["reason"] =
                    layer_error.empty() ? "selected_layer_cache_unknown" : layer_error;
            }
            values[name] = std::move(field);
        }
        out["saved_values"] = std::move(saved);
        out["values"] = std::move(values);
        const auto loaded = read_native_graphic_line_style_parameters(instance);
        Json parameters = {{"status", "unresolved"}, {"input_reader", loaded}};
        if (loaded.at("status") != "not_evaluated" && loaded.contains("reader_return_code")) {
            const bool success = loaded.at("reader_return_code") == 0;
            parameters.update({{"status", "decoded"},
                               {"selection", success ? "graphic_reader"
                                                     : "defaults_after_graphic_reader_failure"},
                               {"state", success ? loaded.at("state")
                                                 : native_default_line_style_parameter_state()}});
        } else
            parameters["reason"] = loaded.value("reason", "block_line_style_parameters_unknown");
        out["line_style_parameters"] = std::move(parameters);
        status(out);
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
Json merge_native_parent_symbols(const Json &current, const Json &previous) {
    Json out = {{"status", "unresolved"}, {"scope", scope}};
    try {
        require(current.at("scope") == scope && previous.at("scope") == scope,
                "native_parent_symbol_inputs_required");
        const auto mask = previous.at("symbol_override_mask").get<std::uint32_t>();
        const auto current_mask = current.at("symbol_override_mask").get<std::uint32_t>();
        require(((mask | current_mask) & ~0x38u) == 0, "invalid_parent_symbol_override_mask");
        out = current;
        out["ancestor_merge"] = "applied";
        out["inherited_fields"] = Json::array();
        out["overridden_fields"] = Json::array();
        for (const auto name : {"color", "line_weight", "line_style"}) {
            auto &field = out.at("values").at(name);
            const auto sentinel = std::string(name) == "line_style" ? 0x7ffffffeu : UINT32_MAX - 1;
            if (field.at("status") == "decoded" && field.at("value") == sentinel) {
                field = previous.at("values").at(name);
                out["inherited_fields"].push_back(name);
            }
        }
        for (const auto &item : {std::make_pair("color", 8u), std::make_pair("line_weight", 16u),
                                 std::make_pair("line_style", 32u)})
            if (mask & item.second) {
                out.at("values").at(item.first) = previous.at("values").at(item.first);
                out["overridden_fields"].push_back(item.first);
            }
        if (mask & 32u)
            out["line_style_parameters"] = previous.at("line_style_parameters");
        out["symbol_override_mask"] = current_mask | mask;
        status(out);
    } catch (const std::exception &e) {
        out.update({{"status", "unresolved"}, {"reason", e.what()}});
    }
    return out;
}
} // namespace p3d
