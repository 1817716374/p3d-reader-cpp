#include "internal.hpp"
#include "view_selection_oracle.hpp"
using namespace p3d;
namespace {
void put(Bytes &bytes, std::size_t offset, std::uint64_t value, unsigned count = 8) {
    for (unsigned i = 0; i < count; ++i) bytes.at(offset+i) = (value >> (i*8)) & 255;
}
Json source(std::size_t index, std::uint64_t id) {
    return {{"native_record_index", index}, {"input_occurrence_index", index+10},
            {"source_id", 999}, {"assigned_id", id}};
}
}
unsigned view_table_selection_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) { ++checks; require(ok, message); };
    Json list = {{"status", "resolved"}, {"scope", "empty_list_before_runtime_registration"},
                 {"system_bootstrap_required", true},
                 {"roots", Json::array({{{"native_record_index", 0}, {"block_number", 1}}})}};
    auto header_source = source(0, 101);
    header_source["parent_record_index"] = nullptr;
    Json ids = {{"status", "resolved"}, {"scope", "first_system_input_with_empty_id_registry"},
                {"roots", Json::array({{{"native_record_index", 0}, {"block_number", 1},
                                        {"records", Json::array({header_source})}}})}};
    Bytes data(0x254); put(data, 16, 8, 4);
    Json records = Json::array({{{"element_type", 46}, {"data", rawbytes(data)}}});
    Json collection = {{"status", "conditional"},
        {"scope", "R1.18_initial_common_then_lite_view_collection"}, {"entries", Json::array()}};
    const auto oracle = Json::parse(view_selection_oracle);
    for (const auto &row : oracle.at("cases")) {
        collection["entries"] = Json::array();
        for (const auto &index : row.at("collection_input_indices")) {
            const auto i = index.get<std::size_t>();
            collection["entries"].push_back({{"collection_index", collection["entries"].size()},
                {"table_input_index", i}, {"source", source(i+1, row.at("assigned_ids")[i].get<std::uint64_t>())}});
        }
        for (const auto &query : row.at("queries")) {
            put(data, 0x24c, query.at("requested_id").get<std::uint64_t>());
            records[0]["data"] = rawbytes(data);
            const auto result = project_file_view_table_selection(list, records, ids, collection);
            if (query.at("selected_input_index").is_null()) {
                check(result.at("status") == "unresolved" && result.at("lookup") == "no_assigned_id_match" &&
                          result.at("fallback") == "required" && !result.contains("selected_entry"),
                      "a bounded native miss does not establish general host/default-model fallback");
            } else {
                check(query.at("native_return_code") == 0 && result.at("status") == "conditional" &&
                          result.at("selected_entry").at("table_input_index") == query.at("selected_input_index") &&
                          result.at("selected_entry").at("source").at("assigned_id") == query.at("requested_id"),
                      "full original file getter matches first post-registration 64-bit ID in merged collection");
            }
            check(result.at("query_source").at("assigned_id") == 101 &&
                      result.at("requested_id_data_offset") == 0x24c &&
                      result.at("runtime_application") == "not_evaluated" && result.at("model_selection") == "not_evaluated",
                  "query source and exact field location are separate from model/display application");
        }
    }
    put(data, 0x24c, 999); records[0]["data"] = rawbytes(data);
    check(project_file_view_table_selection(list, records, ids, collection).at("lookup") == "no_assigned_id_match",
          "source IDs are not substituted for assigned IDs");
    auto bad_records = records; bad_records[0]["element_type"] = 14; bad_records.push_back(records[0]);
    check(project_file_view_table_selection(list, bad_records, ids, collection).at("status") == "unresolved",
          "first system root mismatch cannot be repaired by searching for a later header");
    bad_records = records; data.resize(0x253); bad_records[0]["data"] = rawbytes(data);
    check(!project_file_view_table_selection(list, bad_records, ids, collection).contains("requested_id"),
          "truncated query ID does not become zero or partial bytes");
    auto bad_ids = ids; bad_ids["roots"][0]["block_number"] = 2;
    check(!project_file_view_table_selection(list, records, bad_ids, collection).contains("requested_id"),
          "mismatched root assignment cannot establish query source identity");
    collection["status"] = "unresolved";
    check(!project_file_view_table_selection(list, records, ids, collection).contains("selected_entry"),
          "an incomplete collection cannot prove a match or a miss");
    list["roots"] = Json::array();
    check(!project_file_view_table_selection(list, records, ids, collection).contains("requested_id"),
          "missing native header does not emulate the uninitialized fallback stack value");
    return checks;
}
