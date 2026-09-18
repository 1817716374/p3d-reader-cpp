#include "internal.hpp"

unsigned reference_file_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool condition, const char *message) {
        ++checks;
        require(condition, message);
    };
    auto put = [](Bytes &bytes, std::size_t at, std::uint64_t value, unsigned size) {
        for (unsigned i = 0; i < size; ++i)
            bytes.at(at + i) = std::uint8_t(value >> (8 * i));
    };
    auto text_link = [&](unsigned key, const std::string &text) {
        Bytes bytes(8 + text.size() + (text.size() & 1));
        put(bytes, 0, key, 2);
        put(bytes, 4, text.size(), 4);
        std::copy(text.begin(), text.end(), bytes.begin() + 8);
        return Json{{"header", 0x1000 | unsigned((bytes.size() + 4) / 2 - 1)},
                    {"app", 0x56d2},
                    {"offset", 372},
                    {"payload", rawbytes(bytes)}};
    };
    auto make_record = [&](unsigned primary, unsigned secondary, Json links) {
        Json input = {
            {"status", "decoded"},
            {"origin_inputs", {{"primary_flags", primary}, {"secondary_flags", secondary}}}};
        return Json{{"element_type", 13},
                    {"reference_input", input},
                    {"reference_target", native_reference_target(input, links)}};
    };
    const auto empty = make_record(0, 0, Json::array());
    ReferenceFileQueryContext context;
    context.complete_host_chain = true;
    context.inherited_search_context = u"";
    context.host_file = NativeFileReference{u"E:/models/source.p3d", u"server:real/source.p3d"};
    context.current_file = *context.host_file;
    auto out = initial_reference_file_query(empty, context);
    check(out["status"] == "reuse_current_file" &&
              out["stored_reference"]["text"] == "source.p3d" &&
              out["lookup_reference"]["text"] == "server:real/source.p3d" &&
              out["reference_strings_source"] == "host_specification_clone" &&
              out["target_resolution"] == "not_evaluated",
          "host fallback strips only saved path and reuses matching current file");
    auto own =
        make_record(0, 0, Json::array({text_link(3, "saved.p3d"), text_link(31, "lookup.p3d")}));
    context.current_file = NativeFileReference{u"other.p3d", u"lookup.p3d"};
    context.host_file.reset();
    out = initial_reference_file_query(own, context);
    check(out["status"] == "reuse_current_file" && out["stored_reference"]["text"] == "saved.p3d" &&
              out["lookup_source"] == "initial_specification_lookup_reference",
          "own file specification precedes host fallback and preserves lookup override");
    context.current_file = NativeFileReference{u"lookup.p3d", u""};
    check(initial_reference_file_query(own, context)["status"] == "reuse_current_file",
          "current file wrapper uses saved reference only for empty lookup reference");
    context.current_file = NativeFileReference{u"lookup.p3d", u"other.p3d"};
    check(initial_reference_file_query(own, context)["status"] == "not_evaluated",
          "different file references need explicit native case comparison context");
    unsigned comparisons = 0;
    context.equal = [&](const std::u16string &a, const std::u16string &b) {
        ++comparisons;
        return a == u"LOOKUP.P3D" && b == u"lookup.p3d";
    };
    check(initial_reference_file_query(own, context)["status"] == "external_search_required" &&
              comparisons == 1,
          "known unequal references continue to external search");
    context.current_file->lookup_reference = u"LOOKUP.P3D";
    check(initial_reference_file_query(own, context)["status"] == "reuse_current_file" &&
              comparisons == 2,
          "caller native wide comparison can identify current-file reuse");
    context.equal = {};
    context.current_file.reset();
    check(initial_reference_file_query(own, context)["reason"] ==
              "current_file_availability_required",
          "missing current context is not a known absent file");
    context.current_file_known_absent = true;
    check(initial_reference_file_query(own, context)["status"] == "external_search_required",
          "known absent current file enters external search");
    context.current_file = NativeFileReference{};
    check(initial_reference_file_query(own, context)["reason"] ==
              "contradictory_current_file_context",
          "contradictory current context remains unresolved");
    context.current_file_known_absent = false;
    context.current_file.reset();
    context.inherited_search_context.reset();
    check(initial_reference_file_query(own, context)["reason"] ==
              "prepared_host_search_context_required",
          "empty key48 does not prove the prepared host search context empty");
    context.inherited_search_context = u"search-token";
    out = initial_reference_file_query(own, context);
    check(out["status"] == "external_search_required" &&
              out["search_context"]["text"] == "search-token" &&
              !out.contains("current_file_reference"),
          "nonempty prepared context bypasses current-file comparison");
    auto with_search =
        make_record(0, 0, Json::array({text_link(3, "lookup.p3d"), text_link(48, "saved-search")}));
    out = initial_reference_file_query(with_search, context);
    check(out["status"] == "external_search_required" &&
              out["search_context"]["text"] == "saved-search" &&
              out["search_context_source"] == "persisted_key_48",
          "nonempty source search text is not replaced by host inheritance");
    context.inherited_search_context = u"";
    context.current_file = NativeFileReference{u"", u"lookup.p3d"};
    const auto alternate = make_record(
        0, 0x20000,
        Json::array({text_link(3, "lookup.p3d"), text_link(35, "not-an-alternate-file.p3d")}));
    check(initial_reference_file_query(alternate, context)["status"] == "reuse_current_file",
          "key35 is not the initial runtime alternate-file string");

    context.host_file = NativeFileReference{u"", u"host.p3d"};
    const auto host_only = make_record(0x200, 0, Json::array());
    out = initial_reference_file_query(host_only, context);
    check(out["status"] == "reuse_host_file" && out["native_error_code"] == 0 &&
              !out.contains("search_context"),
          "host branch with empty saved reference precedes current lookup and search context");
    context.host_file->stored_reference = u"E:/host.p3d";
    out = initial_reference_file_query(host_only, context);
    check(out["status"] == "rejected" && out["native_error_code"] == 0x13011,
          "nonempty inherited filename is rejected in the host-only branch");
    auto compound_host = make_record(0x200, 0, Json::array({text_link(3, "<1>model")}));
    context.host_file.reset();
    out = initial_reference_file_query(compound_host, context);
    check(out["reason"] == "host_file_availability_required",
          "empty compound saved reference still needs host file availability");
    context.host_file_known_absent = true;
    out = initial_reference_file_query(compound_host, context);
    check(out["status"] == "no_file" && out["native_error_code"] == 0,
          "known missing host returns no file without inventing an error code");
    check(initial_reference_file_query(empty, context)["reason"] ==
              "host_file_unavailable_for_specification",
          "missing host cannot supply fallback specification");
    context.host_file = NativeFileReference{};
    check(initial_reference_file_query(empty, context)["reason"] ==
              "contradictory_host_file_context",
          "contradictory host specification context remains unresolved");
    context.host_file_known_absent = false;
    context.host_file.reset();
    check(initial_reference_file_query(empty, context)["reason"] ==
              "host_file_specification_required",
          "unknown host fallback is not inferred from current file");
    context.complete_host_chain = false;
    check(initial_reference_file_query(own, context)["reason"] == "reference_query_gate_unresolved",
          "incomplete host chain prevents file query decisions");
    Bytes block_bytes(16);
    put(block_bytes, 0, 19, 2);
    put(block_bytes, 4, 1, 4);
    put(block_bytes, 8, 13, 8);
    const auto blocked = make_record(
        0, 0,
        Json::array(
            {Json{{"header", 0x1009}, {"app", 0x56d5}, {"payload", rawbytes(block_bytes)}}}));
    out = initial_reference_file_query(blocked, context);
    check(out["status"] == "blocked" && out["native_error_code"] == 0 &&
              !out.contains("stored_reference"),
          "current gate blocks query without unrelated missing context");
    context.host_reference_targets = {blocked.at("reference_target")};
    check(initial_reference_file_query(own, context)["status"] == "blocked",
          "host gate propagates into integrated file query");
    context.host_reference_targets.clear();
    context.complete_host_chain = true;

    struct PathCase {
        std::u16string source, basename;
    };
    for (const auto &c :
         {PathCase{u"E:\\models\\part.p3d", u"part.p3d"},
          PathCase{u"project:folder/part.P3D", u"part.P3D"}, PathCase{u"folder\\name.", u"name."},
          PathCase{u"folder/.p3d", u".p3d"}, PathCase{u"folder/", u""}, PathCase{u".", u"."},
          PathCase{u"..", u".."}, PathCase{u"C:name", u"name"}, PathCase{u"logical:name", u"name"},
          PathCase{u"logical:a:b.p3d", u"logical:a:b.p3d"},
          PathCase{u"file:///E:/folder/name.p3d", u"name.p3d"},
          PathCase{u"E:/模型/构件.p3d", u"构件.p3d"},
          PathCase{std::u16string(u"E:/a.p3d\0discard", 16), u"a.p3d"}}) {
        context.host_file = NativeFileReference{c.source, u"lookup.p3d"};
        out = initial_reference_file_query(empty, context);
        const auto &units = out.at("stored_reference").at("utf16_code_units");
        check(out["status"] == "reuse_current_file" &&
                  units == Json(std::vector<std::uint16_t>(c.basename.begin(), c.basename.end())) &&
                  out["lookup_reference"]["text"] == "lookup.p3d",
              "host clone preserves lexical filename including extension and leaves lookup "
              "unchanged");
    }
    for (const auto size : {259u, 260u}) {
        context.host_file = NativeFileReference{std::u16string(size, u'a'), u"lookup.p3d"};
        out = initial_reference_file_query(empty, context);
        check(out["stored_reference"]["utf16_code_units"].size() == (size < 260 ? size : 0) &&
                  out["host_path_buffer_limit"] == (size >= 260),
              "host basename observes native 260-code-unit part buffer");
    }
    context.host_file =
        NativeFileReference{std::u16string(259, u'd') + u"/short.p3d", u"lookup.p3d"};
    out = initial_reference_file_query(empty, context);
    check(out["host_path_buffer_limit"] == true && out["stored_reference"]["text"] == "",
          "directory overflow clears filename even when only basename is requested");
    context.host_file = NativeFileReference{std::u16string(1, char16_t(0xd800)), u"lookup.p3d"};
    out = initial_reference_file_query(empty, context);
    check(out["stored_reference"]["utf16_code_units"] == Json::array({0xd800}) &&
              out["stored_reference"]["text_status"] == "invalid_utf16" &&
              Json::parse(out.dump()) == out,
          "host clone preserves invalid wide units without invalid JSON text");
    context.host_file = NativeFileReference{u"", u"lookup.p3d"};
    context.current_file->lookup_reference = std::u16string(u"lookup.p3d\0extra", 16);
    check(initial_reference_file_query(empty, context)["status"] == "reuse_current_file",
          "native file comparison uses NUL-terminated wide text");
    auto malformed = own;
    malformed["reference_target"]["file_specification"]["status"] = "not_evaluated";
    check(initial_reference_file_query(malformed, context)["status"] == "not_evaluated",
          "unresolved own specification cannot fall back to host");
    malformed = own;
    malformed["element_type"] = 62;
    check(initial_reference_file_query(malformed, context)["status"] == "not_evaluated",
          "file query does not treat ordinary blocks as model attachments");
    return checks;
}
