#include "internal.hpp"
#include "view_tables_oracle.hpp"
using namespace p3d;
namespace {
void put(Bytes &bytes, std::size_t at, std::uint32_t value) {
    for (unsigned i = 0; i < 4; ++i) bytes.at(at+i) = static_cast<std::uint8_t>(value >> (8*i));
}
Json record(unsigned type, unsigned subtype, int slot, unsigned marker) {
    Bytes data(0x124);
    put(data, 16, subtype);
    put(data, 0xfc, 0xffffffff);
    put(data, 0x100, 0x31415926);
    put(data, 0x10c, static_cast<std::uint16_t>(slot));
    data[0x114] = static_cast<std::uint8_t>(marker);
    data[0x115] = 17;
    data[0x116] = 43;
    for (unsigned i = 0; i < 32; ++i) data[0x74+i] = static_cast<std::uint8_t>(marker);
    return {{"element_type", type}, {"id", 8}, {"data", rawbytes(data)}, {"links", Json::array()}};
}
Json header(std::size_t ni, Json parent) {
    return {{"native_record_index", ni}, {"parent_record_index", parent}, {"status", "resolved"},
            {"output_element_flags", 0}, {"output_record_word_count", 144}};
}
Json list(const Json &headers) {
    return {{"scope", "empty_list_before_runtime_registration"}, {"status", "resolved"},
            {"system_bootstrap_required", true}, {"system_bootstrap_found", true},
            {"roots", Json::array({{{"native_record_index", 0}, {"block_number", 1}, {"headers", headers}}})}};
}
Json assignments(const Json &l, const Json &records) {
    return native_system_id_assignments(l, records,
        {{"status", "resolved"}, {"initial_probe", {{"action", "read_header_payload"}, {"id_counter", 100}}}});
}
}

unsigned view_table_inputs_tests() {
    unsigned checks = 0;
    auto check = [&](bool condition, const char *message) { ++checks; require(condition, message); };
    const auto oracle = Json::parse(view_tables_oracle);
    for (const auto &row : oracle.at("cases")) {
        Json records = Json::array({record(14, row.at("lite").get<bool>() ? 1000 : 1, 0, 0)});
        Json headers = Json::array({header(0, nullptr)});
        unsigned marker = 0;
        for (const auto &child : row.at("children")) {
            const auto ci = records.size();
            records.push_back(record(child.at("type"), child.at("subtype"), child.at("slot"), ++marker));
            headers.push_back(header(ci, 0));
            if (child.at("nested") == true) {
                headers.push_back(header(records.size(), ci));
                records.push_back(record(11, 1, 7, 250));
            }
        }
        const auto l = list(headers), ids = assignments(l, records);
        const auto result = initial_native_view_table_inputs(l, records, ids);
        check(result.at("status") == "resolved" && result.at("collection_selection") == "not_evaluated",
              "view inputs are resolved independently of final collection and active-view selection");
        check(result.at("collection_projection").at("status") == "conditional" &&
                  result.at("collection_projection").at("entries").at(0).at("table_input_index") == 0,
              "initial view input API attaches the independently scoped collection projection");
        const auto &table = result.at("tables").at(0);
        check(table.at("status") == "conditional" && table.at("slot_projection_status") == "conditional" &&
                  table.at("child_scan").size() == row.at("children").size(),
              "built-in table child scan preserves immediate sibling scope");
        for (unsigned slot = 0; slot < 8; ++slot) {
            const auto &actual = table.at("slots").at(slot), &expected = row.at("slots").at(slot);
            check(actual.at("slot_index") == expected.at("slot") &&
                      actual.at("constructor_rgb") == expected.at("rgb") &&
                      actual.at("constructor_words") == expected.at("words") &&
                      bytesof(actual.at("auxiliary_bytes")) == expected.at("aux_bytes").get<Bytes>(),
                  "slot RGB, flag changes and auxiliary copies match the full original ordinary/Lite constructor");
            const auto &source = actual.at("source");
            check(source.at("source_id") == 8 && source.at("assigned_id") != 8 &&
                      source.at("input_occurrence_index").get<std::size_t>() > 0,
                  "direct and copied views retain their specific duplicate-ID input occurrence");
        }
    }
    Json records = Json::array({record(14, 1, 0, 0)});
    auto l = list(Json::array({header(0, nullptr)}));
    auto ids = assignments(l, records);
    auto out = initial_native_view_table_inputs(l, records, ids);
    const auto &empty = out.at("tables").at(0);
    check(empty.at("slot_projection_status") == "unresolved" && empty.at("slots").size() == 8 &&
              !empty.at("slots").at(0).contains("constructor_rgb"),
          "no valid child does not invent native default views or black backgrounds");
    records[0] = record(14, 2, 0, 0);
    out = initial_native_view_table_inputs(l, records, ids);
    check(out.at("tables").at(0).at("status") == "not_a_view_table_input",
          "other type14 subtypes do not inherit either table constructor layout");
    records[0] = record(14, 1, 0, 0);
    records.push_back(record(11, 1, 2, 1));
    l = list(Json::array({header(0, nullptr), header(1, 0)}));
    ids = assignments(l, records);
    Bytes truncated(32); put(truncated, 16, 1); records[1]["data"] = rawbytes(truncated);
    out = initial_native_view_table_inputs(l, records, ids);
    check(out.at("status") == "partial" && !out.at("tables").at(0).contains("slots"),
          "truncated eligible child cannot be skipped to claim a complete slot projection");
    ids["roots"][0]["records"][1]["parent_record_index"] = 99;
    out = initial_native_view_table_inputs(l, records, ids);
    check(out.at("status") == "unresolved" || out.at("status") == "partial",
          "assignment mismatch cannot hide a direct child by changing its parent");
    l["status"] = "partial";
    check(initial_native_view_table_inputs(l, records, ids).at("status") == "unresolved",
          "incomplete prepared input cannot establish which table children survived loading");
    return checks;
}
