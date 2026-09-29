#include "internal.hpp"
#include "view_fallback_oracle.hpp"
using namespace p3d;
namespace {
void put(Bytes &b, std::size_t offset, std::uint64_t value) {
    for (unsigned i = 0; i < 8; ++i) b.at(offset+i) = (value >> (8*i)) & 255;
}
Json input_record(const Json &input, std::uint64_t bits) {
    Json links = Json::array();
    for (const auto &link : input.at("links")) {
        const auto text = link.at("payload_hex").get<std::string>();
        Bytes bytes;
        for (std::size_t i = 0; i < text.size(); i += 2)
            bytes.push_back(static_cast<std::uint8_t>(std::stoul(text.substr(i, 2), nullptr, 16)));
        links.push_back({{"app", link.at("app")}, {"header", link.at("header")}, {"payload", rawbytes(bytes)}});
    }
    Bytes data(0x40); put(data, 16, input.at("lite").get<bool>() ? 1000 : 1); put(data, 0x1c, bits);
    return {{"element_type", 14}, {"data", rawbytes(data)}, {"links", links}};
}
}
unsigned view_table_fallback_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) { ++checks; require(ok, message); };
    const auto oracle = Json::parse(view_fallback_oracle);
    Json header = {{"status", "resolved"}, {"initial_probe", {{"action", "read_header_payload"},
                                                               {"default_model_id", 0xfffffffeu}}}};
    Json records, collection;
    for (const auto &row : oracle.at("cases")) {
        Bytes query_data(0x254); put(query_data, 16, 8); put(query_data, 0x24c, 0xfedcba9876543210ull);
        records = Json::array({{{"element_type", 46}, {"data", rawbytes(query_data)}}});
        for (std::size_t i = 0; i < row.at("inputs").size(); ++i)
            records.push_back(input_record(row.at("inputs")[i], row.at("comparison_bits")[i].get<std::uint64_t>()));
        collection = {{"status", "conditional"},
            {"scope", "R1.18_initial_common_then_lite_view_collection"}, {"entries", Json::array()}};
        for (const auto &index : row.at("collection_input_indices")) {
            const auto i = index.get<std::size_t>();
            collection["entries"].push_back({{"collection_index", collection["entries"].size()},
                {"table_input_index", i}, {"source", {{"native_record_index", i+1},
                    {"input_occurrence_index", i+20}, {"source_id", 99}, {"assigned_id", row.at("assigned_ids")[i]}}}});
        }
        const auto result = project_file_view_table_fallback(records, collection, header);
        const auto &native = row.at("queries")[0];
        check(result.at("status") == "conditional" && result.at("native_return_code") == native.at("native_return_code"),
              "resident fallback result/error agrees with the full original file getter");
        check(native.at("selected_input_index").is_null()
                  ? result.at("decision") == "no_table" && !result.contains("selected_entry")
                  : result.at("decision") == "existing_table" &&
                        result.at("selected_entry").at("table_input_index") == native.at("selected_input_index"),
              "native exclusion, ties, unordered binary64 and all-ones preference are preserved");
        check(row.at("default_input_index").is_null()
                  ? result.at("default_name_lookup") == "absent"
                  : result.at("excluded_default_entry").at("table_input_index") == row.at("default_input_index"),
              "first case-insensitive default name matches original including NUL and whitespace behavior");
        Json list = {{"status", "resolved"}, {"scope", "empty_list_before_runtime_registration"},
            {"system_bootstrap_required", true},
            {"roots", Json::array({{{"native_record_index", 0}, {"block_number", 1}}})}};
        Json ids = {{"status", "resolved"}, {"scope", "first_system_input_with_empty_id_registry"},
            {"roots", Json::array({{{"native_record_index", 0}, {"block_number", 1},
                {"records", Json::array({{{"native_record_index", 0}, {"parent_record_index", nullptr},
                    {"input_occurrence_index", 0}, {"source_id", 10}, {"assigned_id", 10}}})}}})}};
        const auto query = project_file_view_table_selection(list, records, ids, collection, header);
        check(query.at("status") == "conditional" && query.at("fallback") == "evaluated" &&
                  query.at("fallback_projection") == result &&
                  query.at("native_return_code") == native.at("native_return_code"),
              "file table selection follows the same verified fallback only after an ID miss");
    }
    // Last fixture has an all-ones candidate before a later entry. Unknown
    // later comparison bytes must not defeat native early return.
    records[3]["data"] = rawbytes(Bytes{});
    check(project_file_view_table_fallback(records, collection, header).at("status") == "conditional",
          "preferred ID returns before reading later comparison fields");
    collection["entries"][1]["source"]["assigned_id"] = 20;
    check(project_file_view_table_fallback(records, collection, header).at("status") == "unresolved",
          "truncated reached comparison input cannot be silently dropped");
    records[1].erase("links");
    check(project_file_view_table_fallback(records, collection, header).at("status") == "unresolved",
          "unknown default name cannot establish exclusion identity");
    collection["entries"] = Json::array();
    check(project_file_view_table_fallback(records, collection, Json::object()).at("status") == "unresolved",
          "missing file model state does not suppress native table creation");
    header["initial_probe"]["default_model_id"] = 0;
    check(project_file_view_table_fallback(records, collection, header).at("status") == "unresolved",
          "a real default model needs the unmodeled creation path");
    header["initial_probe"]["default_model_id"] = 0xfffffffeu;
    header["initial_probe"]["action"] = "initialize_blank_header";
    check(project_file_view_table_fallback(records, collection, header).at("status") == "unresolved",
          "blank-header initialization is not evidence for the ordinary file model field");
    return checks;
}
