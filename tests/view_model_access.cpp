#include "internal.hpp"
#include <p3d/view_model.hpp>
#include <p3d/view_model_access.hpp>
#include "view_model_access_oracle.hpp"
using namespace p3d;
unsigned view_model_access_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *message) { ++checks; require(value, message); };
    auto object = [](const Json &value) -> std::optional<std::size_t> {
        return value.is_null() ? std::nullopt : std::optional<std::size_t>(value.get<std::size_t>());
    };
    const auto oracle = Json::parse(view_model_access_oracle);
    for (const auto &row : oracle.at("cases")) {
        NativeViewModelAccessContext c;
        c.cached_model = {true, object(row.at("cached_before"))};
        c.owning_file_present = row.at("file_present").get<bool>();
        c.stored_model_id = row.at("stored_model_id").get<std::int32_t>();
        c.file_state_680 = row.at("file_state_680").get<std::int32_t>();
        c.resident_models_complete = true; c.system_context_object = 0;
        for (const auto &entry : row.at("registry_entries"))
            c.resident_models[entry[0].get<std::int32_t>()] = object(entry[1]);
        for (const auto &state : row.at("states"))
            c.object_states[state.at("object_index").get<std::size_t>()] = {
                true, state.at("status_record_byte14").get<std::uint8_t>(), state.at("model_byte154").get<std::uint8_t>()};
        if (row.contains("helper_object_index")) c.forced_load_result = {true, object(row.at("helper_object_index"))};
        const auto result = row.at("use_preferred").get<bool>() ?
            resolve_native_preferred_view_model(c, {true, object(row.at("preferred"))}) :
            resolve_native_view_model(c, row.at("force_load").get<bool>());
        check(result.resolved && result.reason.empty() && result.object_index == object(row.at("returned_object_index")),
              "returned model identity matches original cache, signed lookup, state gates and bounded load paths");
        check(result.cache_write && (*result.cache_write ? result.object_index : c.cached_model.object_index) == object(row.at("cached_after")),
              "cache transition matches original and preferred hits do not replace the regular cache");
        if (row.contains("cached_requery")) {
            NativeViewModelAccessContext changed;
            changed.cached_model = {true, object(row.at("cached_after"))};
            for (unsigned i = 0; i < 2; ++i)
                check(resolve_native_view_model(changed, i != 0).object_index == object(row.at("cached_requery")[i]),
                      "cached object identity survives detached file and changed model state");
        }
        if (row.value("helper_return_code", 0u) == 1u)
            check(result.object_index && result.cache_write == true,
                  "helper loading error does not suppress the pointer returned and cached by the getter");
    }
    NativeViewModelAccessContext c;
    check(!resolve_native_view_model(c).resolved, "unknown current cache cannot become a known cache miss");
    c.cached_model = {true, 0};
    check(resolve_native_view_model(c).resolved && resolve_native_view_model(c).object_index == 0,
          "graph object zero is a non-null identity and short circuits all file state");
    check(!resolve_native_preferred_view_model(c, {}).resolved,
          "unknown preferred cache takes precedence over a known regular cached object");
    check(resolve_native_preferred_view_model({}, {true, 3}).object_index == 3,
          "known preferred hit does not require regular cache or file state");
    c.cached_model = {true, std::nullopt};
    check(!resolve_native_view_model(c).resolved, "known cache miss still requires file presence");
    c.owning_file_present = false;
    check(resolve_native_view_model(c, true).resolved && !resolve_native_view_model(c, true).forced_load_required,
          "absent file bypasses unknown ID and forced loading");
    c.owning_file_present = true; c.stored_model_id = -2;
    check(resolve_native_view_model(c, true).cache_write == false,
          "model sentinel returns null without calling the loader or writing the cache");
    c.stored_model_id = -3;
    check(!resolve_native_view_model(c).resolved, "negative IDs other than -2 still require resident lookup");
    c.resident_models[-3] = std::nullopt;
    check(resolve_native_view_model(c).resolved && !resolve_native_view_model(c).object_index,
          "known null registry entry proves a miss without requiring a complete registry");
    c.resident_models[-3] = 9;
    check(resolve_native_view_model(c).reason == "resident_model_state_not_established",
          "non-null lookup does not certify model readiness");
    c.object_states[9] = {false, std::nullopt, std::nullopt};
    check(!resolve_native_view_model(c).resolved && !resolve_native_view_model(c).cache_write,
          "null metadata pointer is not a normal unavailable-model result");
    c.object_states[9] = {true, 0, std::nullopt};
    check(!resolve_native_view_model(c).resolved, "both state bytes are read before the combined readiness check");
    c.object_states[9].model_byte154 = 255;
    check(resolve_native_view_model(c).resolved && !resolve_native_view_model(c).object_index,
          "zero status byte prevents caching even when the second byte is nonzero");
    c.object_states[9].status_record_byte14 = 128;
    check(resolve_native_view_model(c).object_index == 9 && resolve_native_view_model(c).cache_write == true,
          "any nonzero byte values satisfy the native gates, not only boolean one");
    check(!resolve_native_view_model(c, true).resolved && resolve_native_view_model(c, true).forced_load_required,
          "force loading cannot substitute the ready resident pointer for the complete helper result");
    c.file_state_680 = 0;
    check(resolve_native_view_model(c, true).resolved && resolve_native_view_model(c, true).cache_write == true &&
          !resolve_native_view_model(c, true).object_index, "known helper guard failure writes a null result to cache");
    c.file_state_680 = 1; c.forced_load_result = {true, 42};
    check(resolve_native_view_model(c, true).object_index == 42,
          "complete helper return identity need not match the old resident entry");
    c.stored_model_id = -1; c.system_context_object.reset();
    check(!resolve_native_view_model(c).resolved, "embedded system context still needs its graph identity");
    c.system_context_object = 9;
    check(resolve_native_view_model(c).object_index == 9, "system context uses the same readiness gates");

    NativeViewModelContext constructor;
    constructor.resident_registry_complete = true;
    constructor.host_result = NativeViewModelHostResult::Reject;
    constructor.file_default_model_id = 99;
    c.stored_model_id = project_native_view_model_id(7, constructor).model_id;
    c.resident_models[99] = 9;
    check(resolve_native_view_model(c).object_index == 9,
          "acquisition follows the constructor's default-model decision rather than the source ID");
    constructor.host_result = NativeViewModelHostResult::Accept;
    c.stored_model_id = project_native_view_model_id(7, constructor).model_id;
    c.resident_models_complete = true;
    check(resolve_native_view_model(c).resolved && !resolve_native_view_model(c).object_index,
          "host acceptance during construction does not imply an available resident model object");
    return checks;
}
