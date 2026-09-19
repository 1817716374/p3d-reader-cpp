#include "material_catalog_read.hpp"
#include <future>
#include <lz4/lz4.h>

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
    auto texture_body = [&](std::u16string name, Bytes data) {
        Bytes body(256, 0);
        require(name.size() < 128, "test texture name capacity");
        for (std::size_t i = 0; i < name.size(); ++i) {
            body[2 * i] = name[i] & 255;
            body[2 * i + 1] = name[i] >> 8;
        }
        put(body, static_cast<std::uint32_t>(data.size()));
        put(body, 999); // This stored field does not replace the lookup group.
        body.insert(body.end(), data.begin(), data.end());
        return body;
    };
    auto texture = [&](unsigned group, unsigned index, Bytes body, unsigned version = 1) {
        Bytes payload;
        put(payload, version);
        put(payload, static_cast<std::uint32_t>(body.size()));
        if (version == 3) {
            Bytes compressed(LZ4_compressBound(static_cast<int>(body.size())));
            auto size = LZ4_compress_default(reinterpret_cast<const char *>(body.data()),
                                             reinterpret_cast<char *>(compressed.data()),
                                             static_cast<int>(body.size()),
                                             static_cast<int>(compressed.size()));
            require(size > 0, "texture test compression");
            payload.insert(payload.end(), compressed.begin(), compressed.begin() + size);
        } else
            payload.insert(payload.end(), body.begin(), body.end());
        return Json{{"group", group},
                    {"key", 22913},
                    {"index", index},
                    {"payload", rawbytes(payload)},
                    {"decoded", {{"encoding", "opaque"}}}};
    };
    auto files = [&] {
        return run().at("entries")[0].at("material").at("embedded_texture_inputs");
    };
    const Bytes image = {0x89, 'P', 'N', 'G', 0, 0xff};
    const auto image_body = texture_body(u"..\\纹理.png", image);
    const auto image_attribute = texture(1, 0, image_body);
    reset_attributes(Json::array({first}));
    check(files().at("status") == "absent", "no embedded attributes stays absent");
    for (auto version : {1u, 2u, 3u}) {
        reset_attributes(Json::array({first, texture(1, 0, image_body, version)}));
        const auto entry = files().at("entries")[0];
        check(
            entry.at("status") == "decoded" && bytesof(entry.at("file").at("file_data")) == image &&
                entry.at("file").at("filename") == "..\\纹理.png" &&
                entry.at("file").at("native_map_type") == 999 && entry.at("group") == 1,
            "source envelope gives exact image bytes and unmodified name, not a path or map guess");
    }
    reset_attributes(Json::array({first, texture(39, 0, image_body), texture(40, 0, image_body),
                                  texture(0, 1, image_body), texture(0, 0, image_body)}));
    result = files();
    check(result.at("entries").size() == 2 && result.at("entries")[0].at("group") == 0 &&
              result.at("entries")[1].at("group") == 39,
          "native embedded queries use ascending groups 0 through 39, exactly index zero");
    check(result.at("entries")[0].at("source_attribute").at("stream") ==
                  Json::array({"root", "attributes"}) &&
              result.at("entries")[0].at("source_attribute").at("record_offset") == 88 &&
              result.at("entries")[0].at("source_ordinal") == 4,
          "embedded image links to the selected attachment and source ordinal");
    auto newer = texture(1, 0, texture_body(u"new.png", Bytes{1}));
    reset_attributes(Json::array({first, image_attribute, newer}));
    check(files().at("entries")[0].at("source_ordinal") == 2,
          "short native attribute collection selects later duplicate image");
    reset_attributes(Json::array({first, image_attribute, newer, texture(2, 1, image_body),
                                  texture(3, 1, image_body), texture(4, 1, image_body)}));
    check(files().at("entries")[0].at("source_ordinal") == 1,
          "long native attribute collection selects first duplicate image");
    auto broken = image_attribute;
    broken["payload"] = rawbytes(Bytes{1});
    reset_attributes(Json::array({first, image_attribute, broken, texture(2, 0, image_body)}));
    result = files();
    check(result.at("status") == "partial" &&
              result.at("entries")[0].at("status") == "unresolved" &&
              result.at("entries")[1].at("status") == "decoded",
          "selected broken image does not fall back to an older duplicate or block later groups");
    check(run().at("entries")[0].at("status") == "decoded",
          "embedded image failure does not discard decoded material source content");
    auto tail = image_body;
    tail.insert(tail.end(), {7, 8, 9});
    reset_attributes(Json::array({first, texture(1, 0, tail)}));
    result = files().at("entries")[0].at("file");
    check(bytesof(result.at("file_data")) == image &&
              bytesof(result.at("ignored_suffix")) == Bytes({7, 8, 9}),
          "native file length bounds exported bytes and preserves ignored trailing bytes");
    for (auto size : {0u, 0x1400000u, 0x80000000u, 0xffffffffu}) {
        Bytes body(256, 0xff);
        put(body, size);
        reset_attributes(Json::array({first, texture(1, 0, body)}));
        const auto skipped = files().at("entries")[0];
        check(skipped.at("status") == "skipped" &&
                  skipped.at("file").at("reason") == "native_file_size_guard" &&
                  bytesof(skipped.at("file").at("data")) == body,
              "signed positive less-than-20MiB guard precedes filename and content reads");
    }
    auto truncated = image_body;
    truncated.pop_back();
    reset_attributes(Json::array({first, texture(1, 0, truncated)}));
    check(files().at("entries")[0].at("status") == "unresolved",
          "eligible declared file cannot read outside decoded payload");
    auto spilled_name = image_body;
    std::fill(spilled_name.begin(), spilled_name.begin() + 256, 1);
    reset_attributes(Json::array({first, texture(1, 0, spilled_name)}));
    result = files().at("entries")[0].at("file");
    check(result.at("filename_within_field") == false &&
              result.at("filename_terminator_offset") == 258,
          "native zero-terminated name may cross nominal field, bounded by actual payload");
    auto invalid_name = image_body;
    invalid_name[0] = 0;
    invalid_name[1] = 0xdc;
    reset_attributes(Json::array({first, texture(1, 0, invalid_name)}));
    check(files().at("entries")[0].at("status") == "unresolved",
          "invalid UTF16 is preserved in source instead of inventing a filename");
    check(files().at("filesystem_processing") == "not_performed" &&
              files().at("texture_reference_binding") == "not_performed",
          "source image availability does not claim filesystem or rendering binding");
    reset_attributes(Json::array({first, image_attribute}));
    auto second_target = target;
    second_target["id"] = 778;
    second_target["input_occurrence_index"] = 22;
    container["system_id_assignments"]["registry"].push_back(second_target);
    container["initial_attribute_input"]["attachments"].push_back(
        attachment(Json::array({second, newer}), second_target));
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
    check(result.at("entries")[0]
                      .at("material")
                      .at("embedded_texture_inputs")
                      .at("entries")[0]
                      .at("file")
                      .at("filename") == "..\\纹理.png" &&
              result.at("entries")[1]
                      .at("material")
                      .at("embedded_texture_inputs")
                      .at("entries")[0]
                      .at("file")
                      .at("filename") == "new.png",
          "same physical material record can have different embedded images per native input");
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
