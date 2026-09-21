#include "internal.hpp"

unsigned native_font_selection_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool value, const char *message) {
        ++checks;
        require(value, message);
    };
    Json catalog = {{"scope", "fresh_system_font_declarations_before_runtime_callbacks"},
                    {"status", "resolved"},
                    {"entries", Json::array()}};
    for (unsigned id : {512u, 513u, 514u, 1024u, 0xffffffffu})
        catalog["entries"].push_back({{"font_id", id},
                                      {"name", std::to_string(id)},
                                      {"requested_font_family", id < 1024 ? "Shx" : "TrueType"},
                                      {"source", {{"source_id", id + 10}}}});
    auto resource = [](const char *name, unsigned type, bool valid, bool big, bool substitute) {
        NativeFontResource f;
        f.identity = name;
        f.type = type;
        f.is_valid = valid;
        f.is_big_font = big;
        f.primary_substitution_required = substitute;
        return f;
    };
    auto shx = resource("main", 3, true, false, false);
    auto big = resource("big", 3, true, true, false);
    auto invalid = resource("invalid", 3, false, false, false);
    auto ttf = resource("ttf", 2, true, false, false);
    std::map<unsigned, NativeFontResource> declared = {
        {512, shx}, {513, big}, {514, invalid}, {1024, ttf}, {0xffffffffu, ttf}};
    auto default_shx = resource("default-shx", 3, true, false, false);
    auto default_big = resource("default-big", 3, true, true, false);
    auto default_ttf = resource("default-ttf", 2, true, false, false);
    std::vector<Json> requests;
    NativeFontContext context;
    context.resolve = [&](const Json &request) -> std::optional<NativeFontResource> {
        requests.push_back(request);
        if (request.at("status") == "declaration_request")
            return declared.at(request.at("declaration").at("font_id").get<unsigned>());
        if (request.at("status") == "default_big_font_request")
            return default_big;
        return request.at("requested_font_family") == "Shx" ? default_shx : default_ttf;
    };
    auto select = [&](unsigned primary, unsigned secondary) {
        requests.clear();
        return select_native_text_fonts(catalog, primary, secondary, context);
    };
    auto result = select(1024, 513);
    check(result.at("status") == "resolved" && result.at("big_font").is_null() &&
              result.at("character_mapping_font").at("identity") == "ttf" && requests.size() == 1,
          "TrueType ignores big-font references entirely");
    result = select(512, 0);
    check(result.at("status") == "resolved" && result.at("big_font").is_null() &&
              result.at("main_font").at("identity") == "main" && requests.size() == 1,
          "zero big-font key does not request the default big font");
    result = select(512, 513);
    check(result.at("big_font").at("identity") == "big" &&
              result.at("main_font").at("identity") == "main" &&
              result.at("character_mapping_font").at("identity") == "big" && requests.size() == 2,
          "valid big declaration controls native type54 character mapping");
    for (unsigned id : {1u, 255u, 256u, 511u, 512u, 514u, 1023u, 1024u, 65536u, 0xffffffffu}) {
        result = select(512, id);
        check(result.at("status") == "resolved" &&
                  result.at("big_font").at("identity") == "default-big" &&
                  requests.back().at("status") == "default_big_font_request",
              "nonzero missing invalid or non-big declarations request default big font");
    }
    declared[513].is_valid = false;
    declared[513].is_big_font.reset();
    declared[513].primary_substitution_required.reset();
    result = select(512, 513);
    check(result.at("big_font").at("identity") == "default-big" && requests.size() == 3,
          "invalid explicit big object bypasses big-kind and substitution queries");
    declared[513] = big;
    declared[513].primary_substitution_required = true;
    result = select(512, 513);
    check(result.at("big_font").at("identity") == "big" && requests.size() == 2,
          "explicit big candidate is never passed through primary substitution");
    declared[513] = big;
    default_big.is_valid = false;
    result = select(512, 999);
    check(result.at("big_font").at("identity") == "default-big" &&
              result.at("big_font").at("is_valid") == false,
          "fallback big-font selection tests kind but does not test validity");
    default_big.is_big_font = false;
    result = select(512, 999);
    check(result.at("big_font").is_null() &&
              result.at("character_mapping_font").at("identity") == "main",
          "non-big fallback is discarded");
    default_big.present = false;
    default_big.is_big_font.reset();
    result = select(512, 999);
    check(result.at("status") == "resolved" && result.at("big_font").is_null(),
          "known absent default big font is distinct from unknown resource context");
    default_big = resource("default-big", 3, true, true, false);
    result = select(513, 512);
    check(result.at("main_font").at("identity") == "default-shx" &&
              result.at("big_font").at("identity") == "big" && requests.size() == 4,
          "primary big font overwrites secondary fallback after ordered secondary checks");
    result = select(513, 0);
    check(result.at("big_font").at("identity") == "big" && requests.size() == 2,
          "primary big font moves to big slot even when secondary key is zero");
    result = select(514, 513);
    check(result.at("main_font").at("identity") == "default-shx" &&
              result.at("big_font").at("identity") == "big",
          "invalid primary SHX is substituted before secondary selection");
    default_shx.is_valid = false;
    default_shx.primary_substitution_required = true;
    result = select(514, 0);
    check(result.at("status") == "resolved" && requests.size() == 2 &&
              result.at("main_font").at("is_valid") == false,
          "returned primary default is not recursively validated or substituted");
    default_shx = resource("default-shx", 3, true, false, false);
    declared[1024].is_valid = false;
    result = select(1024, 513);
    check(result.at("main_font").at("identity") == "ttf" && requests.size() == 1,
          "TrueType primary substitution is separate from its validity flag");
    declared[1024].is_valid = true;
    declared[1024].primary_substitution_required = true;
    result = select(1024, 513);
    check(result.at("main_font").at("identity") == "default-ttf" && requests.size() == 2,
          "TrueType substitution can occur even when the separate validity flag is true");
    result = select(0xffffffffu, 0);
    check(result.at("main_font").at("request").at("declaration").at("font_id") == 0xffffffffu,
          "explicit lookup keys retain full unsigned width");
    declared[1024] = ttf;
    declared[1024].primary_substitution_required.reset();
    check(select(1024, 0).at("reason") == "primary_truetype_substitution_unknown",
          "unknown TrueType substitution is not silently treated as usable");
    declared[1024] = ttf;
    declared[512].is_valid.reset();
    check(select(512, 0).at("reason") == "primary_shx_validity_unknown",
          "unknown SHX validity is not treated as a missing font");
    declared[512] = shx;
    declared[513].is_big_font.reset();
    check(select(512, 513).at("reason") == "explicit_big_font_kind_unknown",
          "unknown explicit big-font kind cannot silently fall back");
    declared[513] = big;
    default_big.is_big_font.reset();
    check(select(512, 999).at("reason") == "default_big_font_kind_unknown",
          "unknown default big-font kind cannot silently be discarded");
    NativeFontContext unknown;
    unknown.resolve = [](const Json &) { return std::optional<NativeFontResource>{}; };
    check(select_native_text_fonts(catalog, 512, 0, unknown).at("reason") ==
              "font_resource_state_unknown",
          "unknown provider result is not an absent default");
    check(select_native_text_fonts(catalog, 512, 0, {}).at("reason") ==
              "font_resource_provider_required",
          "selection requires explicit external resource metadata");
    default_ttf.primary_substitution_required = false;
    result = select_native_text_fonts(Json(), 255, 513, context);
    check(result.at("status") == "resolved" && result.at("big_font").is_null(),
          "low primary TrueType branch never requires a font registry");
    check(select_native_text_fonts(Json(), 256, 0, context).at("status") == "unresolved",
          "high primary key requires resolved source declarations");
    declared[512].identity = nullptr;
    check(select(512, 0).at("reason") == "font_resource_identity_required",
          "selected resources retain caller-provided identity");
    declared[512] = ttf;
    check(select(512, 0).at("reason") == "font_resource_family_mismatch",
          "a provider cannot change the declaration's native factory family");
    declared[512] = shx;
    declared[1024].is_big_font = true;
    check(select(1024, 0).at("reason") == "truetype_cannot_be_big_font",
          "resource metadata cannot contradict the native TrueType big-font predicate");
    declared[1024] = ttf;
    declared[512].present = false;
    result = select(512, 0);
    check(result.at("status") == "resolved" &&
              result.at("main_font").at("identity") == "default-shx" && requests.size() == 2,
          "known null primary registry entry follows ID-based default request");
    declared[512] = shx;
    declared[513].present = false;
    default_big = big;
    result = select(512, 513);
    check(result.at("status") == "resolved" &&
              result.at("big_font").at("request").at("status") == "default_big_font_request",
          "known null explicit big entry follows default-big fallback");
    default_shx.present = false;
    check(select(256, 0).at("reason") == "required_font_object_absent",
          "default primary must be a non-null object before its native virtual call");
    unknown.resolve = [](const Json &) -> std::optional<NativeFontResource> {
        throw std::runtime_error("provider failure");
    };
    result = select_native_text_fonts(catalog, 512, 513, unknown);
    check(result.at("status") == "unresolved" && result.at("reason") == "provider failure" &&
              !result.contains("main_font") && !result.contains("character_mapping_font"),
          "failed resource provider publishes no completed selection");
    return checks;
}
