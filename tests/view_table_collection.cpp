#include "internal.hpp"
#include "view_collection_oracle.hpp"
using namespace p3d;
namespace {
Json record(const Json &input) {
    Json links = Json::array();
    for (const auto &link : input.at("links")) {
        const auto text = link.at("payload_hex").get<std::string>();
        Bytes payload;
        for (std::size_t i = 0; i < text.size(); i += 2)
            payload.push_back(static_cast<std::uint8_t>(std::stoul(text.substr(i, 2), nullptr, 16)));
        links.push_back({{"header", link.at("header")}, {"app", link.at("app")}, {"payload", rawbytes(payload)}});
    }
    return {{"links", links}};
}
Json table(const Json &input, unsigned index) {
    return {{"status", "conditional"}, {"table_kind", input.at("lite").get<bool>() ? "lite" : "ordinary"},
            {"source", {{"input_occurrence_index", index}}}, {"text_fields", project_view_table_text_fields(record(input))}};
}
}
unsigned view_table_collection_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *message) { ++checks; require(value, message); };
    const auto oracle = Json::parse(view_collection_oracle);
    for (const auto &row : oracle.at("cases")) {
        Json tables = Json::array();
        for (std::size_t i = 0; i < row.at("inputs").size(); ++i) {
            auto t = table(row.at("inputs")[i], static_cast<unsigned>(i));
            check(t.at("text_fields").at("status") == "resolved",
                  "bounded original view-table strings have a determined constructor result");
            for (unsigned field = 0; field < 2; ++field)
                check(t.at("text_fields").at("fields")[field].at("constructor_utf16_units") ==
                          row.at("constructor_text_units")[i][field],
                      "constructor text units match original including duplicate, NUL, truncation and invalid-marker behavior");
            tables.push_back(std::move(t));
        }
        const auto plan = project_view_table_collection(tables);
        Json selected = Json::array();
        for (const auto &entry : plan.at("entries")) selected.push_back(entry.at("table_input_index"));
        check(plan.at("status") == "conditional" && selected == row.at("selected_input_indices") &&
                  plan.at("active_table_selection") == "not_evaluated",
              "ordinary append then first exact pair replacement matches the complete original collection constructor");
    }
    Json empty = {{"lite", false}, {"links", Json::array()}};
    auto a = table(empty, 4), b = table(empty, 7);
    b["table_kind"] = "lite";
    auto plan = project_view_table_collection(Json::array({a, b}));
    check(plan.at("entries")[0].at("source").at("input_occurrence_index") == 7 &&
              plan.at("operations")[1].at("replaced_table_input_index") == 0,
          "replacement preserves the selected source occurrence and the displaced input index");
    auto malformed = record(empty);
    malformed["links"].push_back({{"app", 0x56d2}, {"header", 0x1007}, {"payload", rawbytes(Bytes{1, 0})}});
    a["text_fields"] = project_view_table_text_fields(malformed);
    plan = project_view_table_collection(Json::array({a, b}));
    check(a.at("text_fields").at("status") == "unresolved" && !plan.contains("entries"),
          "unknown earlier text cannot masquerade as empty and change later replacement positions");
    check(project_view_table_text_fields(Json::object()).at("status") == "unresolved",
          "missing linkage inventory is different from an explicitly empty linkage list");
    malformed["links"][0]["payload"] = rawbytes(Bytes{1});
    check(project_view_table_text_fields(malformed).at("status") == "unresolved",
          "truncated user string key cannot silently be skipped");
    a = table(empty, 4); a["table_kind"] = "unknown";
    check(project_view_table_collection(Json::array({a})).at("status") == "unresolved",
          "unknown constructor kinds cannot be silently removed from a collection plan");
    return checks;
}
