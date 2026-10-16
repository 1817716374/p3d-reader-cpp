#include "block_layer_inputs.hpp"
#include <p3d/block_layers.hpp>

namespace p3d {
Json native_block_layer_inputs(const Bytes &bytes) {
    Json out = {{"status", "unresolved"},
                {"scope", "shared_block_base_before_parent_symbology_preparation"},
                {"source_offsets_include_stream_prefix", true},
                {"parent_preparation", "not_evaluated"}};
    try {
        Reader r(bytes);
        require(r.at<std::uint16_t>(4) == 62, "block_layer_input_requires_type_62");
        const auto flags = r.at<std::uint16_t>(112);
        out.update({{"status", "decoded"},
                    {"layer_id", r.at<std::uint32_t>(16)},
                    {"layer_source_offset", 16},
                    {"override_flags", flags},
                    {"override_flags_source_offset", 112},
                    {"force_layer", bool(flags & 1)},
                    {"use_numeric_offset", bool(flags & 2)},
                    {"other_override_bits", flags & ~3u},
                    {"source_symbology",
                     {{"line_style", r.at<std::uint32_t>(48)},
                      {"line_style_source_offset", 48},
                      {"line_weight", r.at<std::uint32_t>(52)},
                      {"line_weight_source_offset", 52},
                      {"color", r.at<std::uint32_t>(56)},
                      {"color_source_offset", 56}}}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
namespace {
Json identity(const Json &record) {
    Json out = Json::object();
    for (const auto key : {"stream", "offset", "id"})
        if (record.contains(key))
            out[key] = record.at(key);
    return out;
}
} // namespace
NativeBlockParentLayerInput
prepare_native_block_parent_layer(const Json &instance, const Json &definition,
                                  const NativeLayerNumericIndex &index) {
    NativeBlockParentLayerInput out;
    out.data = {{"status", "unresolved"},
                {"scope", "initial_shared_block_parent_layer_component"},
                {"instance_source", identity(instance)},
                {"definition_source", identity(definition)},
                {"table_record_index", index.metadata().at("table_record_index")},
                {"runtime_validity", "not_evaluated"},
                {"valid_model_required", true},
                {"other_symbology", "not_evaluated"}};
    try {
        const auto input = native_block_layer_inputs(bytesof(instance.at("data")));
        require(input.at("status") == "decoded",
                input.value("reason", "block_layer_input_unknown"));
        const auto bytes = bytesof(definition.at("data"));
        require(Reader(bytes).at<std::uint16_t>(4) == 32, "block_definition_requires_type_32");
        const auto definition_layer = Reader(bytes).at<std::uint32_t>(16);
        NativeParentLayerRule rule;
        rule.layer_id = input.at("layer_id").get<std::uint32_t>();
        rule.force_layer = input.at("force_layer").get<bool>();
        rule.use_numeric_offset = input.at("use_numeric_offset").get<bool>();
        const auto instance_lookup = index.lookup_layer(rule.layer_id);
        const auto definition_lookup = index.lookup_layer(definition_layer);
        out.data.update({{"input", input},
                         {"definition_layer_id", definition_layer},
                         {"definition_layer_source_offset", 16},
                         {"instance_layer_lookup", instance_lookup},
                         {"definition_layer_lookup", definition_lookup}});
        if (instance_lookup.at("status") != "unresolved" &&
            definition_lookup.at("status") != "unresolved") {
            const auto a = instance_lookup.value("numeric_key", std::uint32_t(0));
            const auto b = definition_lookup.value("numeric_key", std::uint32_t(0));
            rule.numeric_offset = std::uint32_t(a - b);
            out.data.update({{"status", "decoded"},
                             {"instance_numeric_key", a},
                             {"definition_numeric_key", b},
                             {"numeric_offset", *rule.numeric_offset}});
        } else {
            out.data.update({{"status", "partial"},
                             {"numeric_offset", nullptr},
                             {"reason", "selected_layer_cache_unknown"}});
        }
        out.rule = rule;
    } catch (const std::exception &e) {
        out.data["reason"] = e.what();
    }
    return out;
}
NativeParentLayerRule merge_native_parent_layer_rules(const NativeParentLayerRule &current,
                                                      const NativeParentLayerRule &previous) {
    auto out = current;
    if (out.layer_id == UINT32_MAX - 1)
        out.layer_id = previous.layer_id;
    if (previous.use_numeric_offset) {
        out.use_numeric_offset = true;
        out.numeric_offset = previous.numeric_offset;
    }
    if (previous.force_layer) {
        out.force_layer = true;
        out.layer_id = previous.layer_id;
    }
    return out;
}
Json apply_native_parent_layer_rule(std::uint32_t id, bool protected_layer,
                                    const NativeParentLayerRule &rule,
                                    const NativeLayerNumericIndex &index) {
    Json out = {{"status", "resolved"},
                {"scope", "native_parent_layer_component"},
                {"input_layer_id", id},
                {"input_protected_layer", protected_layer},
                {"protected_layer",
                 protected_layer || (!rule.force_layer && rule.use_numeric_offset && id != 0)},
                {"parent_layer_id", rule.layer_id},
                {"material_loading", "not_evaluated"}};
    if (protected_layer) {
        out.update({{"selection", "protected_layer_bypass"}, {"layer_id", id}});
    } else if (rule.force_layer) {
        out.update({{"selection", "forced_parent_layer"}, {"layer_id", rule.layer_id}});
    } else if (rule.use_numeric_offset && id != 0) {
        if (!rule.numeric_offset) {
            out.update({{"status", "unresolved"}, {"reason", "parent_numeric_offset_unknown"}});
        } else {
            out["offset_query"] = index.remap_offset(id, *rule.numeric_offset, rule.layer_id);
            out["status"] = out.at("offset_query").at("status");
            if (out.at("status") == "resolved")
                out["layer_id"] = out.at("offset_query").at("layer_id");
            else
                out["reason"] = out.at("offset_query").at("reason");
            out["selection"] = "numeric_offset";
        }
    } else {
        out.update({{"selection", id == UINT32_MAX - 1 ? "inherited_parent_value" : "unchanged"},
                    {"layer_id", id == UINT32_MAX - 1 ? rule.layer_id : id}});
    }
    return out;
}
} // namespace p3d
