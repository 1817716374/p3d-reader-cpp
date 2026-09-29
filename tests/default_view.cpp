#include "internal.hpp"
#include <p3d/default_view.hpp>
#include "default_view_oracle.hpp"
using namespace p3d;
unsigned default_view_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *message) { ++checks; require(value, message); };
    auto near = [](double a, double b) { return std::abs(a-b) <= 2e-12*(1+std::abs(a)+std::abs(b)); };
    const auto oracle = Json::parse(default_view_oracle);
    for (const auto &row : oracle.at("cases")) {
        NativeDefaultViewInput input; input.model_present = true;
        input.directory.owning_file_present = !row.at("entries").is_null();
        input.directory.model_id = row.at("model_id"); input.directory.entries_complete = true;
        if (!row.at("entries").is_null()) for (const auto &e : row.at("entries")) {
            NativeViewDirectoryEntry entry;
            entry.model_id = e.at("model_id"); entry.value_50 = e.at("value_50");
            entry.state_54 = e.at("state_54"); entry.excluded_55 = e.at("excluded_55"); entry.state_59 = e.at("state_59");
            input.directory.entries.push_back(entry);
        }
        input.model_query_68 = row.at("model_query_68"); input.model_record_value_1c = row.at("model_record_value_1c");
        input.range = row.at("range").get<std::array<double,6>>();
        if (!row.at("viewport").is_null()) input.viewport = row.at("viewport").get<std::array<std::int32_t,4>>();
        input.slot = row.at("slot"); input.orientation_preset = row.at("preset");
        input.selected = row.at("selected"); input.state_188 = row.at("state_188");
        const auto result = project_native_default_view(input);
        check(result.resolved && result.reason.empty(), "complete bounded default constructor input resolves");
        check(result.flags == row.at("flags").get<std::array<std::uint32_t,3>>() &&
              result.values_a8_to_c0 == row.at("values_a8_to_c0").get<std::array<double,4>>() &&
              result.state_188 == row.at("result_state_188"),
              "default flags, numeric fields and state byte match full allocated native constructor");
        bool frame_equal = result.frame.resolved && result.frame.slot_index == row.at("slot_index") &&
                           near(result.frame.half_depth,row.at("half_depth").get<double>());
        for (unsigned i=0;i<3;++i)
            frame_equal &= near(result.frame.origin[i],row.at("origin")[i].get<double>()) &&
                           near(result.frame.delta[i],row.at("delta")[i].get<double>());
        for (unsigned i=0;i<9;++i) frame_equal &= near(result.frame.orientation[i],row.at("orientation")[i].get<double>());
        check(frame_equal, "preset reset, directory matrix decision and frame match full original constructor");
        check(result.directory.preserve_spatial_orientation == ((row.at("flags")[0].get<std::uint32_t>() & 0x50000u) != 0),
              "ordered runtime directory predicate agrees with original default flags branch");
    }
    NativeViewDirectoryContext directory;
    check(!evaluate_native_view_directory(directory).preserve_spatial_orientation,
          "unknown owning file is not absent");
    directory.owning_file_present = false;
    check(evaluate_native_view_directory(directory).preserve_spatial_orientation == false,
          "absent file needs no directory or model ID");
    directory.owning_file_present = true;
    check(!evaluate_native_view_directory(directory).preserve_spatial_orientation,
          "present file requires bound model ID");
    directory.model_id = 7;
    check(!evaluate_native_view_directory(directory).preserve_spatial_orientation,
          "incomplete directory cannot prove a miss");
    directory.entries = {{7,1,1,0,0},{7,0,1,0,1}};
    auto predicate = evaluate_native_view_directory(directory);
    check(predicate.preserve_spatial_orientation == false && predicate.entry_index == 0,
          "first matching false entry prevents later true duplicate and needs no suffix completeness");
    directory.entries[0].excluded_55 = 255;
    predicate = evaluate_native_view_directory(directory);
    check(predicate.preserve_spatial_orientation == true && predicate.entry_index == 1,
          "excluded entry is skipped before matching model ID");
    directory.entries.clear(); directory.entries_complete = true;
    predicate = evaluate_native_view_directory(directory);
    check(predicate.preserve_spatial_orientation == false && !predicate.entry_index,
          "complete missing directory entry uses ordinary branch");
    NativeDefaultViewInput input;
    check(!project_native_default_view(input).resolved, "default constructor needs known model presence");
    input.model_present = false;
    check(project_native_default_view(input).reason == "default_view_requires_nonnull_model",
          "null model must not enter native default flag dereference");
    input.model_present = true; input.directory = directory;
    check(project_native_default_view(input).reason == "default_view_model_query_68_unknown",
          "ordinary directory needs model virtual predicate");
    input.model_query_68 = true;
    check(project_native_default_view(input).resolved, "true virtual predicate skips record value");
    input.model_query_68 = false;
    check(project_native_default_view(input).reason == "default_view_model_record_value_1c_unknown",
          "false virtual predicate requires record value");
    input.directory.entries = {{7,0,128,0,0}}; input.model_query_68.reset();
    const auto special = project_native_default_view(input);
    check(special.resolved && special.values_a8_to_c0[0] == 0 && !special.frame.orientation_reduced,
          "special directory skips model virtual predicates and preserves cleared matrix");
    input.range[0] = std::numeric_limits<double>::infinity();
    check(!project_native_default_view(input).resolved, "default projection propagates nonfinite range diagnosis");
    return checks;
}
