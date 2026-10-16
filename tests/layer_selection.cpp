#include "internal.hpp"
using namespace p3d;
unsigned layer_selection_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *message) {
        require(value, message);
        ++checks;
    };
    const Json index = {{"P3D-SM", "MODELS"},
                        {"P3D~MMH", "MMH"},
                        {"P3DSysD", "SYSD"},
                        {"P3D-SSYS", "SYS"},
                        {"P3D-SSYSA", "SYSA"}};
    auto table = [](const std::string &file, std::uint64_t id, const char *kind = "local") {
        return Json{{"id", id},
                    {"stream", {file, "SYSD", "SYS", "records"}},
                    {"offset", 0},
                    {"length", 68},
                    {"layer_table", {{"kind", kind}, {"declared_child_count", 0}}}};
    };
    auto model = [](const std::string &file, const Json &id) {
        return Json{{"7",
                     {{"layer_group_references",
                       {{{"table_id", id}, {"stream", {file, "MODELS", "M7", "MMH"}}}}}}}};
    };
    auto query = [&](const Json &native, const Json &models) {
        return build_layer_tables(index, native, Json::array(), models).at("model_references")[0];
    };
    auto native = Json::array({table("A", 100), table("B", 100), table("A", 77, "layer_group"),
                               table("B", 77, "layer_group")});
    const auto original = native;
    auto result = query(native, model("A", 0));
    check(result["status"] == "none" && result["file_scope"] == Json::array({"A"}) &&
              result["initial_layer_cache"]["status"] == "resolved" &&
              result["initial_layer_cache"]["table_index"] == 0 &&
              result["initial_layer_cache"]["native_record_index"] == 0 && native == original,
          "zero saved group selects only the current file's local table without changing records");
    result = query(native, model("B", 0));
    check(result["initial_layer_cache"]["table_index"] == 1,
          "same local table ID in another file does not merge initial caches");
    result = query(native, model("A", 77));
    check(result["status"] == "resolved" && result["table_indices"] == Json::array({2}) &&
              result["initial_layer_cache"]["status"] == "unresolved" &&
              result["initial_layer_cache"]["reason"] == "referenced_group_cache_not_evaluated",
          "group reference is file scoped and does not masquerade as evaluated runtime selection");
    result = query(native, model("B", 77));
    check(result["table_indices"] == Json::array({3}),
          "equal group IDs resolve independently in each file scope");
    result = query(native, model("C", 77));
    check(result["status"] == "missing" && result["table_indices"].empty(),
          "foreign group ID is not a match for an otherwise absent local group");
    auto unknown = model("A", 77);
    unknown["7"]["layer_group_references"][0].erase("stream");
    result = query(native, unknown);
    check(result["status"] == "unresolved_scope" && result["table_indices"].empty() &&
              result["initial_layer_cache"]["status"] == "unresolved",
          "missing header scope never guesses a group across files");
    unknown = model("A", 0);
    unknown["7"]["layer_group_references"][0].erase("table_id");
    result = query(native, unknown);
    check(result["status"] == "unsupported_header" &&
              result["initial_layer_cache"]["status"] == "unresolved",
          "unknown saved group reference is not defaulted to zero");
    result = query(Json::array({table("B", 100)}), model("A", 0));
    check(result["initial_layer_cache"]["status"] == "unresolved" &&
              result["initial_layer_cache"]["local_table_indices"].empty(),
          "missing local table is not supplied from a different file");
    native.push_back(table("A", 101));
    result = query(native, model("A", 0));
    check(result["initial_layer_cache"]["status"] == "unresolved" &&
              result["initial_layer_cache"]["local_table_indices"] == Json::array({0, 4}),
          "multiple local tables remain explicit candidates instead of choosing first");
    native = original;
    native.push_back(table("A", 77, "layer_group"));
    result = query(native, model("A", 77));
    check(result["status"] == "ambiguous" && result["table_indices"] == Json::array({2, 4}),
          "same-scope duplicate groups remain ambiguous");
    native = Json::array({table("A", 100)});
    native[0]["layer_table"]["declared_child_count"] = 1;
    result = query(native, model("A", 0));
    check(result["initial_layer_cache"]["status"] == "unresolved" &&
              result["initial_layer_cache"]["reason"] == "local_layer_table_membership_invalid",
          "broken local table membership cannot produce resolved initial selection");
    native = Json::array({table("A", 100)});
    native[0]["stream"] = {"A", "SYSD", "SYS", "extra", "records"};
    result = query(native, model("A", 0));
    check(result["initial_layer_cache"]["status"] == "unresolved",
          "system container marker at an unrelated depth does not establish file scope");
    unknown = model("A", 77);
    unknown["7"]["layer_group_references"][0]["stream"] = {"A", "MODELS", "M7", "OTHER"};
    check(query(original, unknown)["status"] == "unresolved_scope",
          "model header marker must identify the expected physical header stream");
    auto incomplete_index = index;
    incomplete_index.erase("P3DSysD");
    result = build_layer_tables(incomplete_index, original, Json::array(), model("A", 77))
                 .at("model_references")[0];
    check(result["status"] == "unresolved_scope" && result["table_indices"].empty(),
          "missing system root mapping cannot turn an unknown group catalog into a known miss");
    return checks;
}
