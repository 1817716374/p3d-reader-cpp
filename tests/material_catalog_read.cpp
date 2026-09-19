#include "material_catalog_read.hpp"
#include <future>

unsigned material_catalog_read_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto put = [](Bytes &bytes, std::uint32_t value) {
        for (unsigned i = 0; i < 4; ++i)
            bytes.push_back(static_cast<std::uint8_t>(value >> (8 * i)));
    };
    auto link = [&](unsigned key, const std::string &value) {
        Bytes payload;
        put(payload, key);
        put(payload, static_cast<std::uint32_t>(value.size() + 1));
        payload.insert(payload.end(), value.begin(), value.end());
        payload.push_back(0);
        return Json{
            {"app", 0x56d2}, {"header", 0x1000}, {"offset", 36}, {"payload", rawbytes(payload)}};
    };
    auto record = [&](const std::string &name) {
        Bytes body(36);
        body[8] = 16;
        return Json{{"id", 10},
                    {"offset", 36},
                    {"element_type", 37},
                    {"element_flags", 0},
                    {"data", rawbytes(body)},
                    {"stream", Json::array({"root", "system", "native"})},
                    {"links", Json::array({link(1, name)})}};
    };
    auto attribute = [&](const std::u16string &xml, bool terminated = true, Bytes suffix = {}) {
        Bytes bytes;
        for (auto unit : xml) {
            bytes.push_back(unit & 255);
            bytes.push_back(unit >> 8);
        }
        if (terminated) {
            bytes.push_back(0);
            bytes.push_back(0);
        }
        bytes.insert(bytes.end(), suffix.begin(), suffix.end());
        Bytes payload;
        put(payload, 1);
        put(payload, static_cast<std::uint32_t>(bytes.size()));
        payload.insert(payload.end(), bytes.begin(), bytes.end());
        return Json{{"group", 0},
                    {"key", 20014},
                    {"index", 0},
                    {"payload", rawbytes(payload)},
                    {"decoded", decode_attribute(0, 20014, payload)}};
    };
    const Json target = {{"id", 777},
                         {"native_record_index", 1},
                         {"input_occurrence_index", 21},
                         {"block_number", 1}};
    auto attachment = [&](Json attributes, Json t = Json()) {
        if (t.is_null())
            t = target;
        return Json{{"target", t},
                    {"stream", Json::array({"root", "attributes"})},
                    {"offset", 88},
                    {"attributes", attributes},
                    {"lookup", native_attribute_lookup(attributes)}};
    };
    Json records = Json::array({record("Wrong source"), record("Object name")});
    Json catalog = {
        {"scope", "single_system_table_with_empty_material_catalog"},
        {"status", "resolved"},
        {"entries",
         Json::array({{{"id", 777},
                       {"name", "Catalog name"},
                       {"resource", {{"primary_context", {{"kind", "current_resource_context"}}}}},
                       {"member", {{"native_record_index", 0}}},
                       {"source_attribute", {{"wrong", true}}}}})}};
    const auto first = attribute(u"<Material color.r='0.2'/>");
    const auto second = attribute(u"<DifferentRoot color.r='0.8'/>");
    Json container = {
        {"system_id_assignments", {{"status", "resolved"}, {"registry", Json::array({target})}}},
        {"initial_attribute_input",
         {{"status", "resolved"},
          {"attachments", Json::array({attachment(Json::array({first, second}))})}}}};
    MaterialCatalogOptions options;
    auto run = [&] { return native_catalog_material_read(catalog, container, records, options); };
    auto reset_attributes = [&](Json values) {
        container["initial_attribute_input"]["attachments"] = Json::array({attachment(values)});
    };
    auto result = run();
    const auto &read = result.at("entries")[0];
    check(result.at("status") == "resolved" && read.at("status") == "decoded",
          "registered source material is decoded");
    check(read.at("target") == target && read.at("source_record").at("source_id") == 10 &&
              read.at("source_record").at("native_record_index") == 1,
          "assigned ID registry chooses the object, not same source ID or member hint");
    check(read.at("material_attribute").at("source_ordinal") == 1 &&
              read.at("material").at("tree").at("tag") == "DifferentRoot",
          "native collection selects the later duplicate for a short collection and accepts "
          "arbitrary root name");
    check(read.at("material").at("name") == "Catalog name" &&
              read.at("material").at("object_name") == "Object name" &&
              read.at("material").at("id") == 777,
          "catalog name and assigned ID overwrite object identity without discarding it");
    check(read.at("material").at("settings").at("source_parameters").at("color.r") == "0.8" &&
              read.at("source_attribute").at("payload") == second.at("payload"),
          "selected material parameters and complete source payload agree");
    check(read.at("object_resource_reference").at("interpretation").at("reader_fallback") ==
              "missing_resource_reference",
          "missing resource string uses native current-context fallback");
    auto with_tail = attribute(u"<M/>", true, Bytes{0, 0xdc, 7});
    reset_attributes(Json::array({with_tail}));
    result = run();
    check(result.at("entries")[0].at("status") == "decoded" &&
              result.at("entries")[0].at("material").at("ignored_suffix").at("bytes") == 3,
          "material reader ignores even invalid UTF16 after first NUL word and retains suffix");
    for (const auto &xml : {std::u16string(u"<M>"), std::u16string(u"<A/><B/>"),
                            std::u16string(u""), std::u16string(u"<!DOCTYPE M><M/>")}) {
        reset_attributes(Json::array({attribute(xml)}));
        check(run().at("entries")[0].at("status") == "unresolved",
              "invalid or unsupported XML never produces confirmed material data");
    }
    reset_attributes(Json::array({attribute(u"<M/>", false)}));
    check(run().at("entries")[0].at("reason") == "native_material_xml_terminator_outside_payload",
          "missing terminator does not read outside attribute");
    auto opaque = first;
    opaque["decoded"] = {{"encoding", "opaque"}};
    reset_attributes(Json::array({opaque}));
    check(run().at("entries")[0].at("reason") == "material_attribute_codec_unavailable",
          "unknown envelope remains unresolved");
    reset_attributes(Json::array({first}));
    catalog["entries"][0]["id"] = 900;
    check(run().at("entries")[0].at("status") == "not_found",
          "absent assigned ID does not fall back to source record ID");
    catalog["entries"][0]["id"] = 777;
    catalog["entries"][0]["resource"]["primary_context"]["kind"] = "explicit_context_keys";
    check(run().at("entries")[0].at("reason") == "current_material_resource_context_keys_required",
          "unknown current context is not assumed equal to an explicit resource");
    auto &resource_context = catalog["entries"][0]["resource"]["primary_context"];
    resource_context["index_key"] = "same-index";
    resource_context["comparison_key"] = "FOREIGN";
    options.current_context = MaterialCatalogContext{"same-index", "current"};
    check(run().at("entries")[0].at("reason") == "external_material_resource_context_required",
          "same ID-index key cannot override unequal resource comparison keys");
    resource_context["comparison_key"] = "CURRENT";
    resource_context["index_key"] = "different-index";
    check(run().at("entries")[0].at("status") == "decoded",
          "provider selects current context by comparison key, independently of ID-index key");
    options.case_equal = [](const auto &a, const auto &b) { return a == b; };
    check(run().at("entries")[0].at("reason") == "external_material_resource_context_required",
          "caller context comparison is honored for provider context selection");
    std::pair<std::string, std::string> compared;
    options.case_equal = [&](const auto &a, const auto &b) {
        compared = {a, b};
        return true;
    };
    check(run().at("entries")[0].at("status") == "decoded" &&
              compared == std::make_pair(std::string("current"), std::string("CURRENT")),
          "provider compares current context to catalog resource context in native argument order");
    options = {};
    catalog["entries"][0]["resource"]["primary_context"]["kind"] = "current_resource_context";
    records[1]["links"] = Json::array();
    check(run().at("entries")[0].at("reason") == "material_object_name_reader_failed",
          "missing object name cannot be patched by catalog name");
    records[1] = record("Object name");
    records[1]["links"].push_back(link(3, ""));
    check(run().at("entries")[0].at("reason") == "material_object_resource_unresolved",
          "present empty resource differs from missing resource fallback");
    records[1] = record("Object name");
    auto wrong_group = first;
    wrong_group["group"] = 1;
    reset_attributes(Json::array({wrong_group}));
    check(run().at("entries")[0].at("reason") == "material_attribute_unavailable",
          "exact group zero material attribute is required");
    reset_attributes(Json::array({first}));
    Json many = Json::array({first, second});
    for (unsigned i = 0; i < 4; ++i) {
        auto extra = first;
        extra["key"] = 100 + i;
        many.push_back(extra);
    }
    reset_attributes(many);
    check(run().at("entries")[0].at("material_attribute").at("source_ordinal") == 0,
          "larger collections use native lower-bound duplicate choice instead of last-item "
          "selection");
    Bytes auxiliary_payload;
    for (auto value : {1u, 8u, 8u, 1u})
        put(auxiliary_payload, value);
    auto extra = Json{{"group", 0},
                      {"key", 22906},
                      {"index", 0},
                      {"payload", rawbytes(auxiliary_payload)},
                      {"decoded", decode_attribute(0, 22906, auxiliary_payload)}};
    reset_attributes(Json::array({first, extra}));
    const auto auxiliary = run().at("entries")[0].at("material").at("supplementary_attributes");
    check(auxiliary[0].at("status") == "absent" && auxiliary[1].at("status") == "selected" &&
              auxiliary[1].at("source_ordinal") == 1 &&
              auxiliary[1].at("source_attribute").at("payload") == extra.at("payload"),
          "supplementary material records come from the same resolved attribute collection");
    reset_attributes(Json::array({first}));
    container["initial_attribute_input"]["status"] = "partial";
    check(run().at("status") == "unresolved",
          "incomplete attribute input cannot confirm final source selection");
    container["initial_attribute_input"]["status"] = "resolved";
    container["system_id_assignments"]["registry"].push_back(target);
    check(run().at("reason") == "ambiguous_system_registry",
          "ambiguous registry fails instead of arbitrarily selecting a source");
    container["system_id_assignments"]["registry"] = Json::array({target});
    catalog["status"] = "partial";
    check(run().at("status") == "partial" && run().at("decoded_entry_indices") == Json::array({0}),
          "accepted catalog prefix is readable without claiming complete catalog");
    catalog["status"] = "resolved";
    auto second_target = target;
    second_target["id"] = 778;
    second_target["input_occurrence_index"] = 22;
    container["system_id_assignments"]["registry"].push_back(second_target);
    container["initial_attribute_input"]["attachments"].push_back(
        attachment(Json::array({second}), second_target));
    auto second_entry = catalog["entries"][0];
    second_entry["id"] = 778;
    second_entry["name"] = "Second load";
    catalog["entries"].push_back(second_entry);
    result = run();
    check(result.at("entries")[0]
                      .at("material")
                      .at("settings")
                      .at("source_parameters")
                      .at("color.r") == "0.2" &&
              result.at("entries")[1]
                      .at("material")
                      .at("settings")
                      .at("source_parameters")
                      .at("color.r") == "0.8",
          "repeated physical record uses each input occurrence's own attribute collection");
    check(result.at("entries")[1].at("material").at("name") == "Second load",
          "source parsing cache does not merge catalog identities");
    const auto original = Json::array({catalog, container, records}).dump();
    auto expected = run();
    std::vector<std::future<Json>> tasks;
    for (unsigned i = 0; i < 4; ++i)
        tasks.push_back(std::async(std::launch::async, run));
    for (auto &task : tasks)
        check(task.get() == expected, "parallel material reading has no mutable shared state");
    check(Json::array({catalog, container, records}).dump() == original,
          "material source reading preserves original records and attributes");
    return checks;
}
