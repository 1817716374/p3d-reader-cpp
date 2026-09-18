#include "internal.hpp"

unsigned native_id_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto put = [](Bytes &b, std::size_t at, std::uint64_t value, unsigned length) {
        for (unsigned i = 0; i < length; ++i)
            b.at(at + i) = std::uint8_t(value >> (8 * i));
    };
    auto control = [&](unsigned flags) {
        Bytes raw(32, 0);
        put(raw, 12, flags, 2);
        for (unsigned i = 12; i < 32; ++i)
            raw[i] ^= i == 12 ? 0x26 : raw[i - 1];
        return raw;
    };
    Bytes payload(0x610, 0);
    const std::uint64_t high = 0xfedcba9876543210ull;
    put(payload, 0x128, high, 8);
    put(payload, 0x310, 0x40000, 4);
    auto h = native_file_header(control(1), payload);
    check(h["status"] == "resolved" && h["source_id_counter"] == high &&
              h["initial_probe"]["id_counter"] == high &&
              h["initial_probe"]["header_flags"] == 0x40001 &&
              bytesof(h["header_payload"]) == payload,
          "file header preserves a full-width persisted ID counter and merges control flags");
    auto reset = native_file_header(control(3), payload);
    check(reset["source_id_counter"] == high && reset["initial_probe"]["id_counter"] == 0 &&
              reset["initial_probe"]["header_flags"] == 2 &&
              reset["initial_probe"]["action"] == "initialize_blank_header",
          "initial blank-header branch is distinct from the preserved serialized payload");
    check(native_file_header(Bytes(31), payload)["native_error_code"] == 0x14007,
          "truncated index control header does not claim an initial counter");
    check(native_file_header(control(1), Bytes(0x130))["status"] == "unresolved",
          "a readable counter alone is not a complete ordinary file header");
    auto extra = payload;
    extra.push_back(0xab);
    check(bytesof(native_file_header(control(1), extra)["header_payload_suffix"]) == Bytes{0xab},
          "extra serialized header data is retained without silently extending the reader layout");
    put(payload, 0x128, 10, 8);
    h = native_file_header(control(1), payload);
    Json list = {
        {"status", "resolved"}, {"system_bootstrap_required", true}, {"roots", Json::array()}};
    Json records = Json::array();
    auto add = [&](std::initializer_list<std::uint64_t> ids) {
        const auto root = records.size();
        Json headers = Json::array();
        for (auto id : ids) {
            const auto ni = records.size();
            records.push_back({{"id", id}});
            headers.push_back({{"status", "resolved"},
                               {"native_record_index", ni},
                               {"parent_record_index", ni == root ? Json() : Json(root)}});
        }
        list["roots"].push_back(
            {{"native_record_index", root}, {"block_number", 1}, {"headers", headers}});
    };
    add({5});
    add({5, 100});
    auto result = native_system_id_assignments(list, records, h);
    check(result["roots"][1]["counter_after_subtree_preparation"] == 100 &&
              result["roots"][1]["records"][0]["assigned_id"] == 101 &&
              result["roots"][1]["records"][1]["assigned_id"] == 100 &&
              result["final_id_counter"] == 101,
          "all subtree IDs raise the counter before a duplicate root is registered");
    check(result["roots"][1]["records"][0]["collisions"] ==
                  Json::array({{{"id", 5},
                                {"existing_native_record_index", 0},
                                {"existing_input_occurrence_index", 0}}}) &&
              records[1]["id"] == 5,
          "duplicate assignment reports the existing source and preserves original IDs");
    list["roots"] = Json::array();
    records = Json::array();
    add({0, 11, 0});
    result = native_system_id_assignments(list, records, h);
    const auto &assigned = result["roots"][0]["records"];
    check(assigned[0]["prepared_id"] == 11 && assigned[1]["prepared_id"] == 11 &&
              assigned[2]["prepared_id"] == 12 && assigned[1]["assigned_id"] == 13 &&
              assigned[2]["assigned_id"] == 12 && result["registry"].size() == 3,
          "zero allocation and intra-subtree collisions remain separate ordered operations");
    put(payload, 0x128, std::numeric_limits<std::uint64_t>::max(), 8);
    h = native_file_header(control(1), payload);
    list["roots"] = Json::array();
    records = Json::array();
    add({0, 5});
    result = native_system_id_assignments(list, records, h);
    check(result["roots"][0]["records"][0]["assigned_id"] == 0 &&
              result["roots"][0]["records"][0]["registration_status"] == "unindexed_zero_id" &&
              result["roots"][0]["records"][0]["native_registration_return_code"] == 1 &&
              result["registry"].size() == 1 && result["final_id_counter"] == 5,
          "uint64 allocation overflow leaves zero unindexed even if a later descendant raises the "
          "counter");
    list["roots"] = Json::array();
    records = Json::array();
    add({1});
    add({1});
    result = native_system_id_assignments(list, records, h);
    check(result["roots"][1]["records"][0]["assigned_id"] == 0 && result["final_id_counter"] == 0 &&
              result["registry"] == Json::array({{{"id", 1},
                                                  {"native_record_index", 0},
                                                  {"input_occurrence_index", 0},
                                                  {"block_number", 1}}}),
          "duplicate overflow does not replace an earlier occupied ID or invent a nonzero ID");
    check(native_system_id_assignments(list, records, reset)["reason"] ==
              "ordinary_initial_file_header_required",
          "blank-header probing is not treated as ordinary system loading");
    list["system_bootstrap_required"] = false;
    check(native_system_id_assignments(list, records, h)["reason"] == "system_container_required",
          "model ID state is not guessed from a fresh file counter");
    list["system_bootstrap_required"] = true;
    list["status"] = "partial";
    check(native_system_id_assignments(list, records, h)["status"] == "partial",
          "incomplete list preparation cannot produce a complete ID result");
    list["status"] = "resolved";
    list["roots"] = Json::array();
    records = Json::array();
    add({5});
    auto repeat = list["roots"][0];
    repeat["block_number"] = 2;
    list["roots"].push_back(repeat);
    put(payload, 0x128, 10, 8);
    h = native_file_header(control(1), payload);
    result = native_system_id_assignments(list, records, h);
    check(result["registry"].size() == 2 && result["registry"][0]["native_record_index"] == 0 &&
              result["registry"][1]["native_record_index"] == 0 &&
              result["registry"][1]["input_occurrence_index"] == 1 &&
              result["registry"][1]["block_number"] == 2 && result["registry"][1]["id"] == 11,
          "repeated source-stream input creates separate occurrences instead of invented source "
          "reuse");
    Json model_records = Json::array({{{"id", 5}}, {{"id", 5}}, {{"id", 100}}, {{"id", 0}}});
    auto model_root = [](std::size_t root, std::vector<std::size_t> nodes, unsigned block) {
        Json headers = Json::array();
        for (auto ni : nodes)
            headers.push_back({{"status", "resolved"},
                               {"native_record_index", ni},
                               {"parent_record_index", ni == root ? Json() : Json(root)}});
        return Json{{"native_record_index", root}, {"block_number", block}, {"headers", headers}};
    };
    auto model_container = [&](const char *kind, const char *physical, Json roots) {
        return Json{
            {"kind", kind},
            {"container", StreamPath{"models", "A", physical}},
            {"list_preparation",
             {{"status", "resolved"}, {"system_bootstrap_required", false}, {"roots", roots}}}};
    };
    const auto control_list = model_container("P3D-SMC", "C", Json::array({model_root(0, {0}, 1)}));
    const auto graphics_list = model_container(
        "P3D-SMG", "G", Json::array({model_root(1, {1, 2}, 1), model_root(3, {3}, 2)}));
    Json model_index = {{"P3D-SMC", "C"}, {"P3D-SMG", "G"}};
    const StreamPath model_path{"models", "A"};
    auto model_result = native_model_id_assignments(Json::array({graphics_list, control_list}),
                                                    model_records, model_index, model_path, 10);
    check(model_result["status"] == "resolved" &&
              model_result["roots"][1]["records"][0]["assigned_id"] == 101 &&
              model_result["roots"][2]["records"][0]["assigned_id"] == 102 &&
              model_result["registry"].size() == 4,
          "model control and graphics share registration while physical container order is "
          "irrelevant");
    check(model_result["roots"][1]["counter_before"] == 10 &&
              model_result["roots"][1]["counter_after_subtree_preparation"] == 100 &&
              model_result["roots"][1]["records"][0]["collisions"][0]
                          ["existing_input_occurrence_index"] == 0,
          "model cross-list collision occurs only after preparing the complete graphics subtree");
    check(model_result["roots"][2]["records"][0]["input_occurrence_index"] == 3 &&
              model_result["registry"][2]["input_list_index"] == 1 &&
              model_result["registry"][2]["container"] == StreamPath{"models", "A", "G"},
          "registered identities retain shared occurrence sequence and exact container origin");
    auto aliased = model_index;
    aliased["P3D-SMG"] = "C";
    model_result = native_model_id_assignments(Json::array({control_list}), model_records, aliased,
                                               model_path, 10);
    check(model_result["registry"].size() == 2 && model_result["registry"][1]["id"] == 11 &&
              model_result["registry"][1]["input_list_index"] == 1 &&
              model_result["registry"][1]["kind"] == "P3D-SMG",
          "aliased model containers are processed twice without invented source reuse");
    auto incomplete = control_list;
    incomplete["list_preparation"]["status"] = "partial";
    model_result = native_model_id_assignments(Json::array({incomplete, graphics_list}),
                                               model_records, model_index, model_path, 10);
    check(model_result["status"] == "partial" && model_result["inputs"].size() == 1 &&
              model_result["registry"].size() == 1,
          "uncertain earlier model list prevents calculating later IDs with an assumed counter");
    model_result = native_model_id_assignments(Json::array({graphics_list}), model_records,
                                               model_index, model_path, 10);
    check(model_result["status"] == "resolved" &&
              model_result["inputs"][0]["status"] == "not_loaded" &&
              model_result["roots"][0]["records"][0]["assigned_id"] == 5,
          "missing control stream contributes no IDs before available graphics input");
    auto foreign = control_list;
    foreign["container"] = StreamPath{"models", "B", "C"};
    model_result = native_model_id_assignments(Json::array({foreign, graphics_list}), model_records,
                                               model_index, model_path, 10);
    check(model_result["registry"].size() == 3 && model_result["registry"][0]["id"] == 5,
          "same numeric ID from another model never enters this model registry");
    auto missing_alias = model_index;
    missing_alias.erase("P3D-SMC");
    check(native_model_id_assignments(Json::array({control_list}), model_records, missing_alias,
                                      model_path, 10)["status"] == "unresolved",
          "missing alias is not replaced with a guessed physical name");
    check(native_model_id_assignments(Json::array({control_list, control_list}), model_records,
                                      model_index, model_path, 10)["status"] == "unresolved",
          "ambiguous model input containers are not arbitrarily selected");
    check(native_model_id_assignments(Json::array({control_list}), model_records, model_index, {},
                                      10)["status"] == "unresolved",
          "model registration requires explicit storage identity");
    const auto max_counter = std::numeric_limits<std::uint64_t>::max();
    model_result = native_model_id_assignments(Json::array({control_list}), model_records, aliased,
                                               model_path, max_counter);
    check(model_result["roots"][1]["records"][0]["assigned_id"] == 0 &&
              model_result["registry"].size() == 1,
          "cross-list duplicate counter overflow preserves the earlier registration");
    return checks;
}
