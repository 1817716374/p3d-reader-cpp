#include "internal.hpp"

namespace p3d {
namespace {
bool known(const std::optional<bool> &value, const char *reason) {
    require(value.has_value(), reason);
    return *value;
}
Json font_value(const NativeFontResource &font, const Json &request) {
    auto flag = [](const std::optional<bool> &value) { return value ? Json(*value) : Json(); };
    return {{"identity", font.identity},
            {"type", font.type},
            {"is_valid", flag(font.is_valid)},
            {"is_big_font", flag(font.is_big_font)},
            {"primary_substitution_required", flag(font.primary_substitution_required)},
            {"request", request}};
}
Json default_request(const char *family) {
    return {{"status", "default_font_request"}, {"requested_font_family", family}};
}
} // namespace

Json select_native_text_fonts(const Json &catalog, std::uint32_t primary_lookup_id,
                              std::uint32_t big_lookup_id, const NativeFontContext &context) {
    Json out = {{"status", "unresolved"},
                {"scope", "native_font_selection_with_supplied_resource_context"},
                {"primary_lookup_id", primary_lookup_id},
                {"big_lookup_id", big_lookup_id},
                {"glyph_status", "not_evaluated"},
                {"character_mapping_status", "not_evaluated"},
                {"steps", Json::array()}};
    try {
        auto load = [&](const Json &request, const char *step, bool allow_absent = false) {
            require(bool(context.resolve), "font_resource_provider_required");
            auto font = context.resolve(request);
            require(font.has_value(), "font_resource_state_unknown");
            require(font->present || allow_absent, "required_font_object_absent");
            if (font->present) {
                require(!font->identity.is_null(), "font_resource_identity_required");
                require(font->type == 2 || font->type == 3, "unsupported_font_resource_type");
                require(request.at("requested_font_family") ==
                            (font->type == 2 ? "TrueType" : "Shx"),
                        "font_resource_family_mismatch");
                if (font->type == 2) {
                    require(!font->is_big_font.value_or(false), "truetype_cannot_be_big_font");
                    font->is_big_font = false;
                }
            }
            out["steps"].push_back(
                {{"action", step},
                 {"request", request},
                 {"resource", font->present ? font_value(*font, request) : Json()}});
            return *font;
        };
        Json main_request = native_primary_font_request(catalog, primary_lookup_id);
        require(main_request.at("status") != "unresolved", "primary_font_request_unresolved");
        auto main = load(main_request, "primary_request",
                         main_request.at("status") == "declaration_request");
        if (!main.present) {
            main_request = default_request(primary_lookup_id < 1024 ? "Shx" : "TrueType");
            main = load(main_request, "null_primary_entry_fallback");
        }
        const bool substitute = main.type == 3
                                    ? !known(main.is_valid, "primary_shx_validity_unknown")
                                    : known(main.primary_substitution_required,
                                            "primary_truetype_substitution_unknown");
        if (substitute) {
            main_request = default_request(main.type == 3 ? "Shx" : "TrueType");
            main = load(main_request, "primary_substitution");
            // The native substitution getter is called once. Its returned
            // default is not recursively substituted or checked for validity.
        }
        std::optional<NativeFontResource> big;
        Json big_request;
        if (main.type == 3) {
            if (big_lookup_id != 0) {
                bool fallback = true;
                if (big_lookup_id > 255) {
                    auto request = native_primary_font_request(catalog, big_lookup_id);
                    require(request.at("status") != "unresolved", "big_font_registry_unresolved");
                    if (request.at("status") == "declaration_request") {
                        auto candidate = load(request, "explicit_big_font_request", true);
                        // Unlike the primary path, this candidate is tested
                        // directly; its substitution getter is never called.
                        if (candidate.present &&
                            known(candidate.is_valid, "explicit_big_font_validity_unknown") &&
                            known(candidate.is_big_font, "explicit_big_font_kind_unknown")) {
                            big = std::move(candidate);
                            big_request = std::move(request);
                            fallback = false;
                        }
                    }
                }
                if (fallback) {
                    Json request = {{"status", "default_big_font_request"},
                                    {"requested_font_family", "Shx"}};
                    auto candidate = load(request, "default_big_font_request", true);
                    // Native fallback tests the big-font flag, not is_valid.
                    if (candidate.present &&
                        known(candidate.is_big_font, "default_big_font_kind_unknown")) {
                        big = std::move(candidate);
                        big_request = std::move(request);
                    }
                }
            }
            if (known(main.is_big_font, "primary_shx_kind_unknown")) {
                big = main;
                big_request = main_request;
                out["steps"].push_back({{"action", "primary_becomes_big_font"},
                                        {"resource", font_value(main, main_request)}});
                main_request = default_request("Shx");
                main = load(main_request, "main_replaced_by_default_shx");
            }
        }
        out["main_font"] = font_value(main, main_request);
        out["big_font"] = big ? font_value(*big, big_request) : Json();
        out["character_mapping_font"] = big ? out["big_font"] : out["main_font"];
        out["status"] = "resolved";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
