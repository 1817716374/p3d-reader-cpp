#include "internal.hpp"
#include <p3d/entity_material.hpp>
#include <codecvt>
#include <locale>

namespace p3d {
namespace {
unsigned link_word(const Json &link, const char *key) {
    const auto &value = link.at(key);
    require(value.is_number_integer() &&
                (value.is_number_unsigned() || value.get<std::int64_t>() >= 0) &&
                value.get<std::uint64_t>() <= UINT16_MAX,
            std::string("invalid_linkage_") + key);
    return value.get<unsigned>();
}
const char *result_status(const GraphicsMaterialResult &value) {
    return !value.known ? "unresolved" : value.material_index ? "matched" : "not_found";
}
} // namespace

NativeEntityMaterialSelection
resolve_native_entity_material(const Json &native_record,
                               const NativeEntityMaterialContext &context) {
    NativeEntityMaterialSelection out;
    auto &report = out.report;
    report = {{"status", "unresolved"},
              {"scope", "native_entity_with_selected_owning_project"},
              {"id_reference", {{"status", "not_inspected"}}},
              {"name_reference", {{"status", "not_inspected"}}},
              {"lookups", Json::array()}};
    report["entity_model_resource_available"] = context.entity_model_resource_available
                                                    ? Json(*context.entity_model_resource_available)
                                                    : Json(nullptr);
    const char *phase = "source_record";
    try {
        const auto &links = native_record.at("links");
        require(links.is_array(), "complete_native_linkage_array_required");
        for (const auto *key : {"id", "offset", "stream"})
            if (native_record.contains(key))
                report["source_record"][key] = native_record.at(key);

        // Selection is lazy: the name path may succeed without ever using an ID.
        bool id_inspected = false;
        std::optional<std::uint64_t> id;
        auto read_id = [&] {
            if (id_inspected)
                return;
            phase = "id_reference";
            auto &reference = report["id_reference"];
            reference["status"] = "unresolved";
            for (std::size_t i = 0; i < links.size(); ++i) {
                const auto &link = links[i];
                if (!(link_word(link, "header") & 0x1000) || link_word(link, "app") != 0x41)
                    continue;
                const auto payload = bytesof(link.at("payload"));
                require(payload.size() >= 4, "material_reference_key_truncated");
                if (Reader(payload).u32() != 0x1000e)
                    continue;
                reference["linkage_index"] = i;
                if (link.contains("offset"))
                    reference["linkage_offset"] = link.at("offset");
                reference["source_payload"] = link.at("payload");
                require(payload.size() >= 12, "material_reference_id_truncated");
                const auto value = Reader(payload, 4).u64();
                reference["material_id"] = value;
                reference["status"] = value == UINT64_MAX ? "sentinel" : "selected";
                if (value != UINT64_MAX)
                    id = value;
                id_inspected = true;
                return;
            }
            reference["status"] = "absent";
            id_inspected = true;
        };
        auto read_name = [&]() -> std::optional<std::u16string> {
            phase = "name_reference";
            auto &reference = report["name_reference"];
            reference["status"] = "unresolved";
            for (std::size_t i = 0; i < links.size(); ++i) {
                const auto &link = links[i];
                if (!(link_word(link, "header") & 0x1000) || link_word(link, "app") != 0x4f5a)
                    continue;
                reference["linkage_index"] = i;
                if (link.contains("offset"))
                    reference["linkage_offset"] = link.at("offset");
                reference["decoded"] =
                    decode_native_material_name(bytesof(link.at("payload")), context.ansi_decoder);
                const auto &decoded = reference["decoded"];
                require(decoded.at("status") == "decoded", "material_reference_name_unresolved");
                const auto text = decoded.at("material_name").get<std::string>();
                const auto prefix = text.substr(0, text.find('\0'));
                std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> codec;
                const auto wide = codec.from_bytes(prefix);
                require(codec.converted() == prefix.size(),
                        "material_name_utf8_conversion_incomplete");
                reference["status"] = "selected";
                reference["lookup_name_utf16_code_units"] =
                    std::vector<std::uint16_t>(wide.begin(), wide.end());
                return wide;
            }
            reference["status"] = "absent";
            return std::nullopt;
        };
        auto perform = [&](Json step, const std::function<GraphicsMaterialResult()> &load) {
            GraphicsMaterialResult value;
            step["status"] = "unresolved";
            try {
                value = load();
                require(value.known || !value.material_index, "unknown_material_has_identity");
                step["status"] = result_status(value);
                if (value.material_index)
                    step["material_index"] = *value.material_index;
                if (!value.known)
                    step["reason"] = "native_material_load_unknown";
            } catch (const std::exception &e) {
                value = {};
                step["reason"] = e.what();
            }
            report["lookups"].push_back(std::move(step));
            return value;
        };
        auto load_id = [&](NativeEntityMaterialIdQuery query) {
            const bool primary = query == NativeEntityMaterialIdQuery::primary_resource_update_true;
            phase = primary ? "primary_id_loading" : "fallback_id_loading";
            return perform({{"query", primary ? "primary_resource_id" : "fallback_resource_id"},
                            {"update", primary},
                            {"material_id", *id},
                            {"linkage_index", report["id_reference"].at("linkage_index")}},
                           [&] {
                               require(bool(context.lookup_id),
                                       "native_material_id_loader_required");
                               return context.lookup_id(*id, query);
                           });
        };
        auto load_name = [&] {
            const auto name = read_name();
            if (!name)
                return GraphicsMaterialResult{true, {}};
            phase = "primary_name_loading";
            // Even empty names pass through project preparation in the outer
            // native search. Do not replace that whole operation with a miss.
            return perform({{"query", "primary_entity_name"},
                            {"update", true},
                            {"lookup_name_utf16_code_units",
                             report["name_reference"].at("lookup_name_utf16_code_units")},
                            {"linkage_index", report["name_reference"].at("linkage_index")}},
                           [&] {
                               require(bool(context.lookup_name),
                                       "native_material_name_loader_required");
                               return context.lookup_name(*name);
                           });
        };

        if (context.entity_model_resource_available == false) {
            out.material = load_name();
        } else {
            read_id();
            if (id) {
                phase = "primary_route_selection";
                require(context.entity_model_resource_available.has_value(),
                        "entity_model_resource_availability_unknown");
                out.material = load_id(NativeEntityMaterialIdQuery::primary_resource_update_true);
            } else {
                out.material = load_name();
            }
        }
        if (out.material.known && !out.material.material_index) {
            // A failed primary ID operation does NOT try the source name.
            // The outer wrapper goes straight to its second ID operation.
            read_id();
            if (id)
                out.material = load_id(NativeEntityMaterialIdQuery::fallback_resource_update_false);
        }
        report["status"] = result_status(out.material);
        if (out.material.material_index)
            report["material_index"] = *out.material.material_index;
        if (!out.material.known)
            report["unresolved_phase"] = phase;
    } catch (const std::exception &e) {
        out.material = {};
        report["reason"] = e.what();
        report["unresolved_phase"] = phase;
    }
    return out;
}
} // namespace p3d
