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
    auto saved_table = [&](const std::string &file, std::uint64_t id, std::uint64_t selector = 0) {
        Bytes bytes(68, 0);
        auto put = [&](std::size_t at, auto value) {
            std::memcpy(bytes.data() + at, &value, sizeof(value));
        };
        put(4, std::uint16_t(10));
        put(6, std::uint16_t(0x80));
        put(8, std::uint32_t(32));
        put(12, std::uint32_t(32));
        put(16, std::uint32_t(1));
        put(20, id);
        put(44, selector);
        auto record = parse_native(bytes).at(0);
        record["stream"] = {file, "SYSD", "SYS", "records"};
        return record;
    };
    auto local = saved_table("A", 100);
    auto group = saved_table("A", 77, UINT64_MAX - 3);
    result = query(Json::array({local}), model("A", 88));
    check(result["status"] == "missing" && result["initial_layer_cache"]["status"] == "resolved" &&
              result["initial_layer_cache"]["selection"] == "missing_group_file_local_fallback" &&
              result["initial_layer_cache"]["table_index"] == 0 &&
              result["initial_layer_cache"]["group_lookup"]["status"] == "not_found" &&
              result["initial_layer_cache"]["group_lookup"]["source_group_count"] == 0,
          "complete saved input with no matching group selects the actual local fallback");
    result = query(Json::array({local, group}), model("A", 88));
    check(result["initial_layer_cache"]["status"] == "resolved" &&
              result["initial_layer_cache"]["group_lookup"]["source_group_count"] == 1,
          "other known group keys do not prevent a confirmed missing-key fallback");
    result = query(Json::array({local, group}), model("A", 77));
    check(result["initial_layer_cache"]["status"] == "unresolved" &&
              result["initial_layer_cache"]["reason"] == "referenced_group_cache_not_evaluated",
          "present group is not replaced by a local table before its contents are evaluated");
    auto foreign = saved_table("B", 88, UINT64_MAX - 3);
    result = query(Json::array({local, foreign}), model("A", 88));
    check(result["initial_layer_cache"]["selection"] == "missing_group_file_local_fallback" &&
              result["initial_layer_cache"]["group_lookup"]["source_group_count"] == 0,
          "equal group key in another file is excluded from the current owner's inventory");
    auto high = saved_table("A", 0xfedcba9800000058ull, UINT64_MAX - 3);
    result = query(Json::array({local, high}), model("A", 88));
    check(result["initial_layer_cache"]["status"] == "resolved" &&
              result["initial_layer_cache"]["group_lookup"]["source_group_count"] == 1,
          "group inventory compares complete unsigned 64-bit identifiers");
    auto broken = group;
    broken["data"] = rawbytes(Bytes(40, 0));
    result = query(Json::array({local, broken}), model("A", 88));
    check(result["initial_layer_cache"]["status"] == "unresolved" &&
              result["initial_layer_cache"]["group_lookup"]["status"] == "unresolved",
          "unreadable saved table cannot prove registry absence");
    broken = group;
    broken["id"] = 123;
    result = query(Json::array({local, broken}), model("A", 77));
    check(result["initial_layer_cache"]["status"] == "unresolved" &&
              result["initial_layer_cache"]["group_lookup"]["status"] == "unresolved",
          "mismatched raw key and annotation cannot manufacture a missing group");
    broken = group;
    broken["stream"] = {"A", "SYSD", "SYS", "extra", "records"};
    result = query(Json::array({local, broken}), model("A", 88));
    check(result["initial_layer_cache"]["status"] == "unresolved" &&
              result["initial_layer_cache"]["group_lookup"]["reason"] ==
                  "group_registry_table_scope_unknown",
          "unclassified table scope blocks a negative group lookup");
    broken = group;
    broken["layer_table"]["declared_child_count"] = 1;
    result = query(Json::array({local, broken}), model("A", 88));
    check(result["initial_layer_cache"]["status"] == "unresolved",
          "invalid input membership does not allow a partial registry to appear complete");
    result = query(Json::array({local, saved_table("A", 101)}), model("A", 88));
    check(result["initial_layer_cache"]["status"] == "unresolved" &&
              result["initial_layer_cache"]["reason"] == "local_layer_table_missing_or_ambiguous",
          "confirmed group miss does not choose among ambiguous local fallbacks");
    auto hidden = group;
    hidden["layer_table"]["kind"] = "local";
    result = query(Json::array({local, hidden}), model("A", 77));
    check(result["initial_layer_cache"]["status"] == "unresolved" &&
              result["initial_layer_cache"]["group_lookup"]["status"] == "source_present" &&
              result["initial_layer_cache"]["group_lookup"]["matching_table_indices"] == Json({1}),
          "raw selector and key remain authoritative when catalog annotations hide a source");
    auto with_member = bytesof(local.at("data"));
    const std::uint32_t count = 1;
    std::memcpy(with_member.data() + 36, &count, sizeof(count));
    Bytes member(108, 0);
    auto put_member = [&](std::size_t at, auto value) {
        std::memcpy(member.data() + at, &value, sizeof(value));
    };
    put_member(4, std::uint16_t(49));
    put_member(6, std::uint16_t(0x80));
    put_member(8, std::uint32_t(52));
    put_member(12, std::uint32_t(52));
    put_member(16, std::uint32_t(1));
    put_member(36, std::uint32_t(64));
    put_member(44, std::uint16_t(7));
    with_member.insert(with_member.end(), member.begin(), member.end());
    auto member_records = parse_native(with_member);
    for (auto &record : member_records)
        record["stream"] = {"A", "SYSD", "SYS", "records"};
    check(query(member_records, model("A", 88))["initial_layer_cache"]["status"] == "resolved",
          "complete valid saved member records permit the missing-group local fallback");
    member[4] = 37;
    member_records[1]["data"] = rawbytes(member);
    result = query(member_records, model("A", 88));
    check(result["initial_layer_cache"]["status"] == "unresolved" &&
              result["initial_layer_cache"]["group_lookup"]["reason"] ==
                  "group_registry_source_layer_type_unknown",
          "a forged layer annotation cannot turn the wrong raw member type into complete input");
    return checks;
}
