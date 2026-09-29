#include "internal.hpp"
#include "display_style_xml.hpp"
using namespace p3d;
namespace {
void put(Bytes &b, unsigned at, std::uint32_t x) {
    for (unsigned i = 0; i < 4; ++i)
        b.at(at + i) = std::uint8_t(x >> (8 * i));
}
Json record(unsigned type, std::uint64_t id, std::int32_t maximum = 3) {
    Bytes b(44);
    put(b, 40, std::uint32_t(maximum));
    return {{"element_type", type}, {"id", id}, {"data", rawbytes(b)}};
}
Json header(unsigned ni, unsigned flags = 0, Json parent = nullptr) {
    return {{"status", "resolved"}, {"native_record_index", ni},
            {"output_element_flags", flags}, {"output_record_word_count", 20},
            {"parent_record_index", parent}};
}
Json list(const Json &roots) {
    Json out = {{"scope", "empty_list_before_runtime_registration"}, {"status", "resolved"},
                {"system_bootstrap_required", true}, {"system_bootstrap_found", true},
                {"roots", Json::array()}};
    for (const auto &headers : roots)
        out["roots"].push_back({{"native_record_index", headers[0].at("native_record_index")},
                                {"block_number", 1}, {"headers", headers}});
    return out;
}
Json attribute(unsigned key, unsigned index, const Bytes &b) {
    return {{"group", 0}, {"key", key}, {"index", index}, {"offset", 100 + index},
            {"payload", rawbytes(b)}, {"decoded", Json::object()}};
}
Json marker(std::uint32_t key, unsigned size = 8) {
    Bytes b(std::max(size, 4u));
    put(b, 0, key);
    b.resize(size);
    return attribute(22900, 1, b);
}
Json xml(unsigned index, const char *name) {
    auto a = attribute(22903, index, {});
    a["decoded"]["tree"] = {{"tag", "ShowStyle"}, {"attributes", {{"Name", name}}},
                             {"children", Json::array({
                                 {{"tag", "Flags"}, {"attributes", Json::object()}},
                                 {{"tag", "Overrides"}, {"attributes", {{"DisplayMode", "3"}}}}
                             })}};
    return a;
}
Json input(const Json &attributes) {
    Json out = {{"status", "resolved"}, {"attachments", Json::array()}};
    for (std::size_t i = 0; i < attributes.size(); ++i)
        if (!attributes[i].is_null())
            out["attachments"].push_back({{"target", {{"input_occurrence_index", i}}},
                {"stream", StreamPath{"FILE", "ATTR", "B1"}},
                {"attributes", attributes[i]}, {"lookup", native_attribute_lookup(attributes[i])}});
    return out;
}
Json run(const Json &l, const Json &records, const Json &in,
         DisplayStyleHandlerProfile profile = DisplayStyleHandlerProfile::UnspecifiedHost) {
    const auto ids = native_system_id_assignments(l, records,
        {{"status", "resolved"}, {"initial_probe", {{"action", "read_header_payload"}, {"id_counter", 100}}}});
    return initial_native_display_style_tables(l, records, ids, in, profile);
}
} // namespace

unsigned display_style_tables_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *message) { ++checks; require(value, message); };
    auto l = list(Json::array({Json::array({header(0)}), Json::array({header(1)}),
                              Json::array({header(2)})}));
    auto records = Json::array({record(92, 8), record(92, 8), record(92, 3)});
    auto attrs = Json::array({Json::array({marker(0x006f0000), xml(1, "first"), xml(3, "last")}),
                              Json::array({marker(0x006f0000), xml(1, "duplicate table")}),
                              Json::array({marker(0x597e0000), xml(0, "Lite")})});
    auto result = run(l, records, input(attrs));
    const auto &common = result.at("common");
    check(common["selected_table"]["native_record_index"] == 0 &&
              result["lite"]["selected_table"]["native_record_index"] == 2,
          "first matching handler wins independently for common and Lite");
    check(common["scan"].size() == 1 && common["native_scan_return_code"] == 11 &&
              result["runtime_resolution"] == "not_evaluated",
          "successful callback stops scanning and does not assert rendering");
    check(common["table_load"]["slot_count"] == 4 && common["table_load"]["entries"].size() == 2 &&
              common["table_load"]["missing_attribute_ranges"] ==
                  Json::array({{{"first", 0}, {"last", 0}}, {{"first", 2}, {"last", 2}}}),
          "missing attributes append null slots without compacting later indices");
    check(common["table_load"]["entries"][1]["slot_index"] == 3 &&
              common["table_load"]["entries"][0]["native_xml_import"]["fields"]["Name"]["value"] == "first",
          "slot input keeps the selected source and typed XML fields");
    attrs[0] = nullptr;
    result = run(l, records, input(attrs));
    check(result["common"]["selected_table"]["native_record_index"] == 1 &&
              result["common"]["selected_table"]["source_id"] == 8 &&
              result["common"]["selected_table"]["assigned_id"] == 101,
          "initial attachments use occurrence identity after duplicate source IDs are reassigned");
    attrs[0] = Json::array({marker(0x006f0000), marker(0x597e0000), xml(0, "A")});
    result = run(l, records, input(attrs));
    check(result["lite"]["selected_table"]["source_ordinal"] == 1 &&
              result["common"]["selected_table"]["native_record_index"] == 1,
          "small attribute collections select the last equal marker key");
    for (unsigned i = 0; i < 3; ++i)
        attrs[0].push_back(attribute(100 + i, 0, {}));
    result = run(l, records, input(attrs));
    check(result["common"]["selected_table"]["native_record_index"] == 0 &&
              result["common"]["selected_table"]["source_ordinal"] == 0,
          "large attribute collections use native sorted lower_bound instead of reverse scan");
    attrs[0] = Json::array({marker(0x006f0000), xml(0, "old"), xml(0, "new"), xml(8, "outside")});
    result = run(l, records, input(attrs));
    check(result["common"]["table_load"]["entries"].size() == 1 &&
              result["common"]["table_load"]["entries"][0]["source_ordinal"] == 2 &&
              result["common"]["table_load"]["entries"][0]["matching_source_ordinals"].size() == 2,
          "duplicate slot keys select one source and out-of-range XML does not create slots");
    records[0] = record(92, 8, -1);
    result = run(l, records, input(attrs));
    check(result["common"]["table_load"]["slot_count"] == 0 &&
              result["common"]["selected_table"]["native_record_index"] == 0,
          "an empty first table does not fall through to another table");
    records[0] = record(92, 8, 2147483646);
    result = run(l, records, input(attrs));
    check(result["common"]["table_load"]["slot_count"] == 2147483647 &&
              result["common"]["table_load"]["missing_attribute_ranges"].back()["last"] == 2147483646,
          "large finite slot layouts remain sparse");
    records[0] = record(92, 8, 2147483647);
    result = run(l, records, input(attrs));
    check(result["common"]["table_load"]["reason"] == "native_int32_slot_loop_wrap" &&
              !result["common"]["table_load"].contains("slot_count"),
          "signed native loop wrap is unresolved instead of publishing a finite list");
    records[0] = record(92, 8);
    attrs[0] = Json::array({marker(0x006f0000, 4)});
    result = run(l, records, input(attrs));
    check(result["common"]["selected_table"]["native_record_index"] == 1,
          "short marker uses the non-style built-in type-92 fallback");
    attrs[0] = Json::array({marker(0x12340000)});
    result = run(l, records, input(attrs));
    check(result["common"]["status"] == "unresolved" && result["common"]["selected_table"].is_null(),
          "unmodeled registration cannot be skipped to claim a later table");
    const auto default_service = DisplayStyleHandlerProfile::BuiltinDefaultService;
    check(run(l, records, input(attrs), default_service)["common"]["status"] == "unresolved",
          "default-service profile does not guess an unprobed registry key");
    for (auto registration : {0x58740000u, 0x58740001u}) {
        attrs[0] = Json::array({marker(registration)});
        check(run(l, records, input(attrs))["common"]["status"] == "unresolved",
              "unspecified host must not borrow default-service observations");
        result = run(l, records, input(attrs), default_service);
        check(result["host_service"] == "original_default_service" &&
                  result["common"]["selected_table"]["native_record_index"] == 1 &&
                  result["lite"]["selected_table"]["native_record_index"] == 2,
              "confirmed unregistered keys fall back before scanning later common and Lite tables");
        check(result["common"]["scan"][0]["handler_vtable_rva"] == 0x54f230 &&
                  result["common"]["scan"][0]["registration_lookup"] == "absent_after_default_service",
              "fallback retains the actual missing key and native handler identity");
    }
    auto bad_input = input(attrs);
    bad_input["status"] = "partial";
    check(run(l, records, bad_input)["common"]["status"] == "unresolved",
          "incomplete attribute input does not establish table selection");
    attrs[0] = Json::array({marker(0x006f0000)});
    auto bad_list = l;
    bad_list["roots"][0]["headers"][0]["output_record_word_count"] = 15;
    result = run(bad_list, records, input(attrs));
    check(result["common"]["status"] == "absent" && result["common"]["native_scan_return_code"] == 0x44,
          "invalid header terminates before attempting a later style table");
    bad_list = l;
    bad_list["roots"][1]["headers"][0]["output_record_word_count"] = 70000;
    check(run(bad_list, records, input(attrs))["common"]["status"] == "selected",
          "invalid later header cannot undo an already stopped matching scan");
    auto tree = list(Json::array({Json::array({header(0, 0x40), header(1, 0x80, 0)}),
                                 Json::array({header(2)})}));
    records[0] = record(10, 8);
    attrs[0] = nullptr;
    attrs[2] = Json::array({marker(0x006f0000)});
    result = run(tree, records, input(attrs));
    check(result["common"]["selected_table"]["native_record_index"] == 2 &&
              result["common"]["scan"][1]["action"] == "skip_rejected_subtree",
          "style-marked descendants of a rejected compound root are not visited");
    records[0] = record(92, 8);
    records[1] = record(49, 8);
    result = run(tree, records, input(attrs));
    check(result["common"]["selected_table"]["native_record_index"] == 1,
          "accepted type-92 compound descendants bypass the type mask");
    attrs[1] = nullptr;
    check(run(tree, records, input(attrs))["common"]["status"] == "unresolved",
          "unknown descendant fallback handler remains unresolved");
    records[1] = record(92, 8);
    check(run(tree, records, input(attrs))["common"]["selected_table"]["native_record_index"] == 2,
          "missing type-92 markers are proven non-style defaults");
    auto ids = native_system_id_assignments(l, records,
        {{"status", "resolved"}, {"initial_probe", {{"action", "read_header_payload"},
                                                       {"id_counter", UINT64_MAX}}}});
    // Use an actual zero-ID wrap from the allocator rather than a fake selected ID.
    records[0] = record(92, 0);
    attrs[0] = Json::array({marker(0x006f0000)});
    ids = native_system_id_assignments(l, records,
        {{"status", "resolved"}, {"initial_probe", {{"action", "read_header_payload"},
                                                       {"id_counter", UINT64_MAX}}}});
    // A normal attribute loader cannot attach to zero. This injected context
    // covers the selector's re-lookup guard independently of the loader.
    result = initial_native_display_style_tables(l, records, ids, input(attrs));
    check(result["common"]["status"] == "selected" &&
              result["common"]["table_load"]["reason"] == "selected_id_not_indexed",
          "selected zero ID empties the load instead of selecting another table");
    const Json absent = {{"status", "absent"}};
    auto table = [](std::uint64_t size, const Json &entries = Json::array()) {
        return Json{{"status", "selected"}, {"table_load", {{"status", "resolved"},
                     {"slot_count", size}, {"entries", entries}}}};
    };
    auto style_entry = [](unsigned index, const char *name, const char *mode) {
        Json tree = {{"tag", "ShowStyle"}, {"attributes", {{"Name", name}}},
                     {"children", Json::array({
                         {{"tag", "Flags"}, {"attributes", {{"Material", "true"}, {"DisplayShadows", "true"}}}},
                         {{"tag", "Overrides"}, {"attributes", {{"DisplayMode", mode}, {"Material", "99"},
                                                                  {"VisibleEdgeColor", "17"}, {"Transparency", "0.5"}}}}
                     })}};
        return Json{{"slot_index", index}, {"attribute_index", index}, {"source_ordinal", index + 10},
                    {"attribute_offset", index + 100}, {"xml_tree", tree},
                    {"native_xml_import", decode_display_style_xml(tree)}};
    };
    auto plan = plan_initial_lite_style_list({{"status", "unresolved"}}, table(2));
    check(plan["status"] == "layout_resolved" && plan["decision"] == "retain_nonempty_lite" &&
              plan["slot_count"] == 2 && plan["operations"].empty(),
          "nonempty Lite layout blocks common copy even when every object is null");
    plan = plan_initial_lite_style_list(absent, absent);
    check(plan["decision"] == "empty_lists" && plan["slot_count"] == 0,
          "two absent tables produce an empty list without fabricated default styles");
    check(plan_initial_lite_style_list(table(1), {{"status", "unresolved"}})["status"] == "unresolved",
          "unknown Lite selection cannot justify common copying");
    plan = plan_initial_lite_style_list(table(2147483647), absent);
    check(plan["slot_count"] == 2147483647 && plan["entries"].empty() && plan["null_slot_ranges"].size() == 1 &&
              plan["null_slot_ranges"][0]["last"] == 2147483646,
          "large null lists are copied as sparse ranges without native-sized allocations");
    Json entries = Json::array({style_entry(1, "Alpha", "2"), style_entry(2, "alpha", "4"),
                               style_entry(3, "ALPHA", "5"), style_entry(4, "Other", "3")});
    const auto original_entries = entries;
    plan = plan_initial_lite_style_list(table(6, entries), absent);
    check(plan["status"] == "conditional" && plan["slot_count"] == 6 && plan["entries"].size() == 4 &&
              entries == original_entries && plan["file_writeback"] == "not_evaluated",
          "copy plan preserves source entries and does not claim runtime file registration");
    check(plan["entries"][0]["slot_index"] == 1 && plan["entries"][0]["stored_index"] == 1 &&
              plan["entries"][0]["source_common_slot"] == 3 && plan["entries"][0]["name"] == "ALPHA" &&
              plan["entries"][1]["source_common_slot"] == 2 && plan["entries"][2]["source_common_slot"] == 3,
          "case-insensitive duplicate copy replaces the first slot while retaining later appended identities");
    check(plan["null_slot_ranges"] == Json::array({{{"first", 0}, {"last", 0}}, {{"first", 5}, {"last", 5}}}),
          "null holes before and after copied objects keep their original indices");
    const auto &projected = plan["entries"][0]["projected_xml_import"];
    check(projected["packed_flags_at_48"] == (2048 + 6) &&
              projected["fields"]["Overrides.Material"]["value"] == 99 &&
              projected["fields"]["Flags.Material"]["value"] == false &&
              projected["fields"]["Overrides.Transparency"]["value"] == 0 &&
              projected["fields"]["Overrides.VisibleEdgeColor"]["value"] == 0,
          "Lite copy changes native masks and edge values but retains disabled material identity");
    plan = plan_initial_lite_style_list(table(2, Json::array({style_entry(0, u8"Ä", "1"),
                                                           style_entry(1, u8"ä", "2")})), absent);
    check(plan["entries"][0]["source_common_slot"] == 0 && plan["entries"][1]["source_common_slot"] == 1,
          "observed C-locale comparison does not fold non-ASCII names");
    auto rejected = style_entry(1, "Rejected", "6");
    rejected["xml_tree"]["attributes"].erase("Name");
    rejected["native_xml_import"] = decode_display_style_xml(rejected["xml_tree"]);
    plan = plan_initial_lite_style_list(table(3, Json::array({rejected})), absent);
    check(plan["entries"].empty() && plan["null_slot_ranges"][0]["last"] == 2 &&
              plan["operations"][1]["reason"] == "rejected_xml_import",
          "failed object import remains a null slot instead of compressing the list");
    entries[1]["native_xml_import"]["fields"]["Overrides.DisplayHandler"]["value"] = 123;
    plan = plan_initial_lite_style_list(table(6, entries), absent);
    check(plan["status"] == "unresolved" && !plan.contains("slot_count") && plan["entries"].empty() &&
              plan["reason"] == "custom_display_handler_import_unmodeled",
          "a later unresolved callback cannot publish a completed list or a guessed replacement");
    entries[1] = original_entries[1];
    entries[1]["attribute_index"] = 5;
    check(plan_initial_lite_style_list(table(6, entries), absent)["reason"] == "style_slot_attribute_index_mismatch",
          "initial input index invariant is checked before native stored-index replacement is modeled");
    // Match the original 3fe9f0 query probe: selection precedes the
    // reader's length check, including duplicates on both lookup paths.
    auto reference = [](std::int32_t value, unsigned index = 0) {
        Bytes b(4); put(b, 0, std::uint32_t(value));
        return attribute(20080, index, b);
    };
    l = list(Json::array({Json::array({header(0)}), Json::array({header(1)}),
                         Json::array({header(2)})}));
    records = Json::array({record(92, 8), record(11, 8), record(11, 10)});
    attrs = Json::array({Json::array({marker(0x597e0000), xml(1, "Lite one")}),
                        Json::array({reference(1)}), Json::array({reference(3)})});
    result = run(l, records, input(attrs));
    auto refs = result["reference_inputs"]["references"];
    check(result["reference_inputs"]["status"] == "resolved" && refs.size() == 2 &&
              result["reference_inputs"]["native_reader_dispatch"] == "not_evaluated",
          "attached reference selection does not assert record constructor dispatch");
    check(refs[0]["source_id"] == 8 && refs[0]["assigned_id"] == 101 &&
              refs[0]["input_occurrence_index"] == 1 && refs[0]["source_ordinal"] == 0 &&
              refs[0]["lite_slot_query"]["entry"]["native_xml_import"]["fields"]["Name"]["value"] == "Lite one",
          "reference input joins the assigned occurrence and selected Lite input without source-ID aliasing");
    check(refs[0]["reader_projection"]["status"] == "conditional" &&
              refs[0]["lite_slot_query"]["outcome"] == "selected_style_input" &&
              refs[0]["lite_slot_query"]["object_construction"] == "not_evaluated" &&
              refs[1]["lite_slot_query"]["outcome"] == "null_slot",
          "selected input and missing Lite slot remain distinct from native object construction");
    attrs[1] = Json::array({reference(1), attribute(20080, 0, Bytes{1, 2, 3})});
    result = run(l, records, input(attrs));
    auto ref = result["reference_inputs"]["references"][0];
    check(ref["source_ordinal"] == 1 && ref["payload_size"] == 3 &&
              ref["reader_projection"]["action"] == "keep_default_invalid_length" &&
              ref["reader_projection"]["display_style_index"] == -1 &&
              ref["lite_slot_query"]["outcome"] == "negative_index",
          "malformed last duplicate keeps default instead of falling back to valid earlier payload");
    attrs[1] = Json::array({attribute(20080, 0, Bytes{1, 2, 3}), reference(1)});
    for (unsigned i = 0; i < 4; ++i) attrs[1].push_back(attribute(20082, i, Bytes{1}));
    ref = run(l, records, input(attrs))["reference_inputs"]["references"][0];
    check(ref["source_ordinal"] == 0 && ref["matching_source_ordinals"].size() == 2 &&
              ref["reader_projection"]["action"] == "keep_default_invalid_length",
          "six-attribute lower_bound can select malformed first duplicate without fallback");
    attrs[1] = Json::array({reference(1, 1)});
    ref = run(l, records, input(attrs))["reference_inputs"]["references"][0];
    check(!ref.contains("source_ordinal") && ref["reader_projection"]["display_style_index"] == -1 &&
              ref["reader_projection"]["action"] == "keep_default_missing_attribute",
          "nonzero attribute index cannot satisfy exact index-zero read");
    attrs[1] = Json::array({reference(1), reference(2, 1)});
    ref = run(l, records, input(attrs))["reference_inputs"]["references"][0];
    check(ref["source_ordinal"] == 0 && ref["reader_projection"]["display_style_index"] == 1,
          "later same-key different-index attribute cannot shadow index zero");
    attrs[1] = Json::array({reference(INT32_MIN)});
    attrs[2] = Json::array({reference(4)});
    refs = run(l, records, input(attrs))["reference_inputs"]["references"];
    check(refs[0]["reader_projection"]["display_style_index"] == INT32_MIN &&
              refs[0]["lite_slot_query"]["outcome"] == "negative_index" &&
              refs[1]["lite_slot_query"]["outcome"] == "out_of_range",
          "signed negative and exact count boundary cannot select a slot");
    attrs[0][1]["decoded"]["tree"]["attributes"].erase("Name");
    attrs[1] = Json::array({reference(1)});
    ref = run(l, records, input(attrs))["reference_inputs"]["references"][0];
    check(ref["lite_slot_query"]["outcome"] == "null_slot" &&
              ref["lite_slot_query"]["status"] == "conditional" &&
              ref["lite_slot_query"]["entry"]["native_xml_import"]["status"] == "rejected",
          "rejected retained XML gives a conditional null while preserving selected input evidence");
    attrs[0] = Json::array({marker(0x006f0000), xml(1, "Alpha"), xml(2, "alpha")});
    ref = run(l, records, input(attrs))["reference_inputs"]["references"][0];
    check(ref["lite_slot_query"]["status"] == "conditional" &&
              ref["lite_slot_query"]["outcome"] == "projected_style_input" &&
              ref["lite_slot_query"]["entry"]["slot_index"] == 1 &&
              ref["lite_slot_query"]["entry"]["source_common_slot"] == 2 &&
              ref["lite_slot_query"]["conditions"].size() == 4,
          "reference follows the completed copy plan including earlier-name replacement and its conditions");
    attrs[0] = Json::array({marker(0x12340000)});
    ref = run(l, records, input(attrs))["reference_inputs"]["references"][0];
    check(ref["status"] == "resolved" && ref["reader_projection"]["display_style_index"] == 1 &&
              ref["lite_slot_query"]["reason"] == "lite_initialization_unresolved",
          "valid reference input remains readable when table resolution is blocked");
    auto incomplete = input(attrs);
    incomplete["attachments"][1]["lookup"]["status"] = "unresolved";
    result = run(l, records, incomplete);
    check(result["reference_inputs"]["status"] == "partial" &&
              !result["reference_inputs"]["references"][0].contains("reader_projection"),
          "unknown attribute selection cannot publish a guessed default reference");
    incomplete["status"] = "partial";
    check(run(l, records, incomplete)["reference_inputs"]["status"] == "unresolved",
          "partial initial attachment input blocks a complete reference inventory");
    return checks;
}
