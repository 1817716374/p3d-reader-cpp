#include "internal.hpp"
#include <limits>

unsigned native_material_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto append = [](Bytes &b, std::uint64_t value, unsigned count) {
        for (unsigned i = 0; i < count; ++i)
            b.push_back(static_cast<std::uint8_t>(value >> (i * 8)));
    };
    auto put = [](Bytes &b, std::size_t offset, std::uint64_t value, unsigned count) {
        for (unsigned i = 0; i < count; ++i)
            b.at(offset + i) = static_cast<std::uint8_t>(value >> (i * 8));
    };
    auto payload = [&](std::uint64_t id) {
        Bytes b;
        append(b, 0x1000e, 4);
        append(b, id, 8);
        return b;
    };
    auto linkage = [&](const Bytes &p, unsigned app = 0x41, unsigned header = 0) {
        Bytes b;
        auto total = p.size() + 4;
        if (!header && total > 512) {
            unsigned shift = 0;
            auto words = total / 2;
            auto mantissa = words;
            while (mantissa > 255) {
                ++shift;
                mantissa = (words + (std::size_t(1) << shift) - 1) >> shift;
            }
            header = 0x5000 | (shift << 8) | unsigned(mantissa);
            total = (mantissa << shift) * 2;
        }
        append(b, header ? header : (0x1000 | (total / 2 - 1)), 2);
        append(b, app, 2);
        b.insert(b.end(), p.begin(), p.end());
        b.resize(total);
        return b;
    };
    auto record = [&](const std::vector<Bytes> &links, std::uint64_t id = 19) {
        Bytes b(36);
        put(b, 4, 1, 2);
        put(b, 12, 16, 4);
        put(b, 20, id, 8);
        for (const auto &l : links)
            b.insert(b.end(), l.begin(), l.end());
        put(b, 8, (b.size() - 4) / 2, 4);
        return b;
    };
    const auto maximum = std::numeric_limits<std::uint64_t>::max();
    for (auto id : {std::uint64_t(0), std::uint64_t(1), std::uint64_t(0xfedcba9876543210),
                    maximum - 1, maximum}) {
        const auto p = payload(id);
        auto rows = parse_native(record({linkage(p)}));
        const auto &l = rows[0]["links"][0];
        const auto &d = l["decoded"];
        check(d["status"] == "decoded" && d["material_id"].is_number_unsigned() &&
                  d["material_id"].get<std::uint64_t>() == id && d["material_id_offset"] == 8 &&
                  d["key"] == 0x1000e && d["reader_selection"] == "first_match" &&
                  d["unassigned_suffix_hex"] == "" && bytesof(l["payload"]) == p,
              "native material IDs retain every bit including sentinel values and raw bytes");
        auto refs = native_material_references(rows);
        check(refs.size() == 1 && refs[0]["reference"] == d &&
                  refs[0]["native_record_index"] == 0 && refs[0]["record_id"] == 19 &&
                  refs[0]["record_offset"] == 0 && refs[0]["linkage_index"] == 0 &&
                  refs[0]["linkage_offset"] == 36 && refs[0]["stream"].is_null(),
              "native material aggregate identifies the exact source linkage");
    }
    auto duplicate =
        parse_native(record({linkage(payload(7)), linkage(payload(8)), linkage(payload(7))}));
    auto refs = native_material_references(duplicate);
    check(refs.size() == 3 && refs[0]["reference"]["reader_selection"] == "first_match" &&
              refs[1]["reference"]["reader_selection"] == "shadowed" &&
              refs[2]["reference"]["reader_selection"] == "shadowed" &&
              refs[1]["reference"]["material_id"] == 8 &&
              refs[2]["reference"]["material_id"] == 7 && refs[2]["linkage_index"] == 2 &&
              refs[2]["linkage_offset"] == 68,
          "native getter first-match order does not discard duplicate or shadowed references");
    for (std::size_t size : {4, 6, 8, 10}) {
        auto short_payload = payload(maximum);
        short_payload.resize(size);
        auto rows = parse_native(record({linkage(short_payload), linkage(payload(42))}));
        refs = native_material_references(rows);
        check(rows.size() == 1 && refs.size() == 2 && refs[0]["reference"]["status"] == "invalid" &&
                  refs[0]["reference"]["material_id"].is_null() &&
                  refs[0]["reference"].contains("decode_error") &&
                  refs[0]["reference"]["reader_selection"] == "first_match" &&
                  refs[1]["reference"]["status"] == "decoded" &&
                  refs[1]["reference"]["reader_selection"] == "shadowed" &&
                  bytesof(rows[0]["links"][0]["payload"]) == short_payload,
              "a truncated first reference is local damage and does not select a later ID");
    }
    for (std::size_t extra : {2, 4}) {
        auto p = payload(73);
        p.resize(p.size() + extra, 0xab);
        auto rows = parse_native(record({linkage(p)}));
        const auto &d = rows[0]["links"][0]["decoded"];
        check(d["status"] == "partial" && d["material_id"] == 73 &&
                  d["unassigned_suffix_hex"] == hex(Bytes(extra, 0xab)) &&
                  bytesof(rows[0]["links"][0]["payload"]) == p,
              "unknown material linkage suffixes remain visible instead of counted as decoded");
    }
    auto extended_payload = payload(99);
    extended_payload.resize(1020, 0x5c);
    auto extended = parse_native(record({linkage(extended_payload, 0x41, 0x5200 | 128)}));
    check(extended[0]["links"][0]["decoded"]["material_id"] == 99 &&
              extended[0]["links"][0]["decoded"]["status"] == "partial" &&
              extended[0]["links"][0]["decoded"]["unassigned_suffix_hex"] == hex(Bytes(1008, 0x5c)),
          "extended linkage word counts bound the material prefix and preserve the full suffix");
    auto wrong_key = payload(5);
    put(wrong_key, 0, 0x1000f, 4);
    auto key_only = payload(5);
    key_only.resize(4);
    auto rows = parse_native(
        record({linkage(payload(5), 0x42), linkage(wrong_key), linkage(key_only, 0x41, 7),
                linkage(Bytes{0x0e, 0x00}), linkage(Bytes{}), linkage(payload(6))}));
    for (std::size_t i = 0; i < 5; ++i)
        check(!rows[0]["links"][i].contains("decoded"),
              "non-user, other-app, other-key and incomplete-key linkages are not material IDs");
    refs = native_material_references(rows);
    check(refs.size() == 1 && refs[0]["linkage_index"] == 5 &&
              refs[0]["reference"]["reader_selection"] == "first_match",
          "unrelated linkages do not consume first material selection");
    auto first = record({linkage(payload(100))}, 21);
    auto second = record({linkage(payload(101))}, 21);
    const auto second_offset = first.size();
    first.insert(first.end(), second.begin(), second.end());
    rows = parse_native(first);
    rows[0]["stream"] = Json::array({"root", "model-a", "native"});
    rows[1]["stream"] = rows[0]["stream"];
    auto other_stream = rows[0];
    other_stream["stream"] = Json::array({"root", "model-b", "native"});
    rows.push_back(other_stream);
    refs = native_material_references(rows);
    check(refs.size() == 3 && refs[1]["native_record_index"] == 1 &&
              refs[1]["record_offset"] == second_offset &&
              refs[1]["linkage_offset"] == second_offset + 36 &&
              refs[1]["reference"]["reader_selection"] == "first_match" &&
              refs[2]["native_record_index"] == 2 && refs[2]["stream"] != refs[0]["stream"] &&
              refs[2]["record_id"] == refs[0]["record_id"],
          "duplicate record IDs and streams retain distinct source identity without fabricated "
          "reuse");
    check(native_material_references(parse_native(record({}))).empty() &&
              native_material_references(Json::array()).empty(),
          "absent native material linkage creates no default reference");
    auto catalog_record = [&](const std::vector<Bytes> &links) {
        auto b = record(links);
        put(b, 4, 49, 2);
        put(b, 16, 18, 4);
        return b;
    };
    auto string_payload = [&](unsigned key, const std::string &text, unsigned extra_word = 0) {
        Bytes p;
        append(p, key, 2);
        append(p, extra_word, 2);
        append(p, text.size() + 4, 4);
        append(p, 0x0001feff, 4); // Native Latin-1 marker, not a guessed UTF-8 string.
        p.insert(p.end(), text.begin(), text.end());
        if (p.size() % 2)
            p.push_back(0);
        return p;
    };
    auto catalog = [&](const std::vector<Bytes> &links) {
        return native_material_catalog_records(parse_native(catalog_record(links)));
    };
    auto c = catalog({linkage(string_payload(1, "stone", 0x1234), 0x56d2),
                      linkage(string_payload(3, "$(_P3DLIB)\\stone.pal"), 0x56d2),
                      linkage(string_payload(2, "other"), 0x56d2)});
    check(c.size() == 1 && c[0]["catalog_name"]["value"] == "stone" &&
              c[0]["catalog_name"]["status"] == "decoded" && c[0]["strings"][0]["key"] == 1 &&
              c[0]["strings"][0]["unassigned_word"] == 0x1234 &&
              c[0]["strings"][0]["role"] == "catalog_name" &&
              c[0]["strings"][0]["suffix_hex"] == "00",
          "catalog names match the low 16-bit key and preserve the unnamed high word and padding");
    check(c[0]["resource_reference"]["value"] == "$(_P3DLIB)\\stone.pal" &&
              c[0]["resource_reference"]["entry_index"] == 1 &&
              c[0]["strings"][1]["role"] == "resource_reference" &&
              c[0]["resource_lookup_status"] == "not_performed" &&
              c[0]["strings"][2]["role"] == "unassigned" && c[0]["strings"][2]["text"] == "other",
          "resource references retain source tokens without resolving paths or inventing unknown "
          "roles");
    auto missing = catalog({});
    auto empty =
        catalog({linkage(string_payload(1, ""), 0x56d2), linkage(string_payload(3, ""), 0x56d2)});
    check(missing[0]["catalog_name"]["status"] == "missing" &&
              missing[0]["resource_reference"]["status"] == "missing" &&
              missing[0]["resource_reference"]["value"].is_null() &&
              empty[0]["catalog_name"]["status"] == "decoded" &&
              empty[0]["resource_reference"]["value"] == "",
          "absent and explicitly empty catalog resources remain distinct");
    c = catalog(
        {linkage(string_payload(1, "first"), 0x56d2), linkage(string_payload(1, "second"), 0x56d2),
         linkage(string_payload(3, ""), 0x56d2), linkage(string_payload(3, "later"), 0x56d2)});
    check(c[0]["catalog_name"]["value"] == "first" && c[0]["resource_reference"]["value"] == "" &&
              c[0]["strings"].size() == 4 && c[0]["strings"][1]["reader_selection"] == "shadowed" &&
              c[0]["strings"][3]["reader_selection"] == "shadowed",
          "catalog name and resource use independent first occurrences without dropping later "
          "values");
    for (unsigned key : {1, 3}) {
        auto damaged = string_payload(key, "bad");
        put(damaged, 4, 2000, 4);
        c = catalog({linkage(damaged, 0x56d2), linkage(string_payload(key, "later"), 0x56d2)});
        const auto field = key == 1 ? "catalog_name" : "resource_reference";
        check(c[0][field]["status"] == "invalid" && c[0][field]["value"].is_null() &&
                  c[0][field]["entry_index"] == 0 && c[0]["strings"][0].contains("decode_error") &&
                  c[0]["strings"][1]["reader_selection"] == "shadowed" &&
                  bytesof(c[0]["strings"][0]["payload"]) == damaged,
              "invalid first catalog strings cannot be silently replaced with subsequent strings");
    }
    auto short_header = Bytes{1, 0};
    c = catalog({linkage(short_header, 0x56d2)});
    check(c[0]["catalog_name"]["status"] == "invalid" && c[0]["strings"][0]["key"] == 1 &&
              c[0]["strings"][0]["unassigned_word"].is_null(),
          "short catalog string header still identifies the selected key safely");
    Bytes invalid_unicode;
    append(invalid_unicode, 1, 4);
    append(invalid_unicode, 4, 4);
    append(invalid_unicode, 0xfeff, 2);
    append(invalid_unicode, 0xdc00, 2);
    c = catalog({linkage(invalid_unicode, 0x56d2)});
    check(c[0]["catalog_name"]["status"] == "invalid" &&
              bytesof(c[0]["strings"][0]["payload"]) == invalid_unicode,
          "invalid native Unicode is retained without fabricating a catalog name");
    auto nonuser = Bytes{1, 0, 0, 0};
    c = catalog({linkage(nonuser, 0x56d2, 7), linkage(string_payload(1, "valid"), 0x56d2)});
    check(c[0]["catalog_name"]["value"] == "valid" && c[0]["catalog_name"]["entry_index"] == 1 &&
              c[0]["strings"][0]["reader_selection"] == "not_selected" &&
              c[0]["strings"][0]["status"] == "invalid",
          "non-user linkage cannot shadow a real catalog string");
    auto native = parse_native(catalog_record({linkage(string_payload(1, "same"), 0x56d2)}));
    native[0]["stream"] = Json::array({"root", "model-a", "native"});
    native.push_back(native[0]);
    native[1]["stream"] = Json::array({"root", "model-b", "native"});
    native.push_back(native[0]);
    native[2]["offset"] = 800;
    c = native_material_catalog_records(native);
    check(
        c.size() == 3 && c[0]["record_id"] == c[1]["record_id"] &&
            c[0]["stream"] != c[1]["stream"] && c[2]["native_record_index"] == 2 &&
            c[2]["record_offset"] == 800 && c[0]["strings"][0]["linkage_offset"] == 36,
        "catalog records with equal IDs and names never collapse across source records or streams");
    auto wrong_type = catalog_record({});
    put(wrong_type, 4, 10, 2);
    auto wrong_subtype = catalog_record({});
    put(wrong_subtype, 16, 17, 4);
    check(native_material_catalog_records(parse_native(wrong_type)).empty() &&
              native_material_catalog_records(parse_native(wrong_subtype)).empty(),
          "only native material definition records enter the material catalog");
    auto interpret = [&](const std::string &text) {
        return catalog(
            {linkage(string_payload(3, text), 0x56d2)})[0]["resource_reference"]["interpretation"];
    };
    auto project = interpret("$(_P3DPROJECT)\\stone-Palette.pal");
    check(project["status"] == "decoded" && project["kind"] == "project_resource" &&
              project["project_member_name"] == "stone-Palette" && project["extension"] == "pal" &&
              project["primary_context"] == "current_resource_context" &&
              project["secondary_context_rule"] == "palette_lookup_or_primary_context" &&
              project["lookup_request"]["reference"] == "stone-Palette.pal" &&
              project["lookup_request"]["search_path_setting"] == "P3d_Material" &&
              project["lookup_request"]["failure_context"] == "primary_context" &&
              project["lookup_status"] == "not_performed",
          "project palette retains primary context and a separate conditional secondary resource "
          "request");
    project = interpret("$(_p3dproject)\\folder\\other.dir/Stone.PaL");
    check(
        project["status"] == "decoded" && project["project_member_name"] == "Stone" &&
            project["extension"] == "PaL" && project["lookup_request"]["reference"] == "Stone.PaL",
        "project comparison ignores ASCII case while lookup uses only the leaf with original case");
    for (const auto &tail : {"model.p3d", "name", "picture.jpg", "folder\\", ""}) {
        project = interpret(std::string("$(_P3DPROJECT)\\") + tail);
        check(project["status"] == "decoded" && project["lookup_request"].is_null() &&
                  project["secondary_context_rule"] == "primary_context",
              "non-palette project resources reuse the primary context without a fabricated "
              "palette request");
    }
    project = interpret("$(_P3DPROJECT)\\.pal");
    check(project["project_member_name"] == "" && project["lookup_request"]["reference"] == ".pal",
          "a leading extension does not become a nonempty project member name");
    project = interpret("$(_P3DPROJECT)\\name.");
    check(project["project_member_name"] == "name" && project["extension"] == "" &&
              project["lookup_request"].is_null(),
          "trailing dot is excluded from the project member name without inventing an extension");
    for (const auto &value : {"$(_P3DPROJECT)/name.pal", "name.pal"})
        check(interpret(value)["kind"] == "palette_resource" &&
                  interpret(value)["lookup_request"]["reference"] == value,
              "ordinary palette paths do not inherit project-token context rules");
    check(interpret("")["status"] == "rejected_empty_reference",
          "successfully read empty resources take the native rejection branch");
    const auto &missing_resource = missing[0]["resource_reference"];
    const auto &fallback_resource = missing_resource["interpretation"];
    check(missing_resource["status"] == "missing" && missing_resource["value"].is_null() &&
              fallback_resource["status"] == "decoded" &&
              fallback_resource["reader_fallback"] == "missing_resource_reference" &&
              fallback_resource["member_name"] == "" &&
              fallback_resource["primary_context"] == "current_resource_context" &&
              fallback_resource["secondary_context_initial_state"] == "unset" &&
              fallback_resource["secondary_context_rule"] == "default_resource_service" &&
              fallback_resource["lookup_request"].is_null(),
          "missing resource key initializes the current-context descriptor without inventing "
          "source text");
    check(fallback_resource["descriptor_mode"] == 0,
          "failed resource getter initializes mode zero independently of string contents");
    auto raw_catalog_text = [&](const Bytes &bytes, unsigned key = 3) {
        Bytes p;
        append(p, key, 2);
        append(p, 0, 2);
        append(p, bytes.size(), 4);
        p.insert(p.end(), bytes.begin(), bytes.end());
        if (p.size() % 2)
            p.push_back(0);
        return catalog({linkage(p, 0x56d2)})[0][key == 1 ? "catalog_name" : "resource_reference"];
    };
    auto native_wide = [&](const std::u16string &text, unsigned marker = 0xfeff) {
        Bytes bytes;
        append(bytes, marker, 2);
        for (auto unit : text)
            append(bytes, unit, 2);
        return bytes;
    };
    auto buffer_text = raw_catalog_text(Bytes{'a', 0xe9}, 1);
    check(buffer_text["status"] == "invalid" && buffer_text["reader"]["status"] == "success" &&
              buffer_text["reader"]["value"] == u8"aé",
          "catalog reader widens unmarked bytes independently of generic source-string decoding");
    buffer_text = raw_catalog_text(native_wide(u"\u8140", 0xfdff), 1);
    check(buffer_text["reader"]["value"] == u8"\u8140" &&
              buffer_text["reader"]["character_conversion"] == "copy_utf16le_units",
          "catalog's fixed 1200 reader copies the FDFF form as UTF16 rather than generic packed "
          "DBCS");
    buffer_text = raw_catalog_text(Bytes{0, 'x', 'y'});
    check(buffer_text["reader"]["status"] == "success" && buffer_text["reader"]["value"] == "" &&
              buffer_text["interpretation"]["status"] == "rejected_empty_reference",
          "a zero first byte succeeds with an empty string before marker inspection");
    buffer_text = raw_catalog_text(Bytes{0xfd, 0xff, 'a', 'b'});
    check(buffer_text["reader"]["conversion_return_code"] == 2 &&
              buffer_text["interpretation"]["reader_fallback"] == "resource_string_read_failed",
          "unsupported native byte-order marker causes resource-reader fallback");
    for (std::size_t n : {510u, 511u, 512u}) {
        buffer_text = raw_catalog_text(native_wide(std::u16string(n, u'q')), 1);
        check(buffer_text["reader"]["native_getter_return_code"] == (n == 510 ? 0 : 1),
              "UTF16 catalog buffer reserves its native termination boundary at 511 units");
    }
    auto terminated = std::u16string(512, u'q');
    terminated.back() = 0;
    buffer_text = raw_catalog_text(native_wide(terminated), 1);
    check(buffer_text["reader"]["status"] == "success" &&
              buffer_text["reader"]["value"].get<std::string>().size() == 511,
          "a terminator at the last copied UTF16 unit makes a full buffer succeed");
    terminated[0] = u'Q';
    terminated[1] = 0;
    terminated.push_back(u'z');
    buffer_text = raw_catalog_text(native_wide(terminated));
    check(buffer_text["reader"]["status"] == "success" && buffer_text["reader"]["value"] == "Q" &&
              buffer_text["interpretation"]["member_name"] == "Q",
          "UTF16 conversion can succeed despite an ignored tail beyond its zero-ended buffer");
    Bytes byte_tail{0xff, 0xfe, 1, 0};
    for (auto unit : terminated)
        byte_tail.push_back(static_cast<std::uint8_t>(unit));
    buffer_text = raw_catalog_text(byte_tail);
    check(buffer_text["reader"]["conversion_return_code"] == 1 &&
              buffer_text["interpretation"]["reader_fallback"] == "resource_string_read_failed",
          "byte-widening branch detects an unread tail even when the copied buffer ends in zero");
    buffer_text = raw_catalog_text(native_wide(std::u16string{u'Q', 0, char16_t(0xdc00)}));
    check(buffer_text["reader"]["value"] == "Q" &&
              buffer_text["interpretation"]["member_name"] == "Q",
          "invalid UTF16 after the first NUL cannot invalidate the native reader's used prefix");
    buffer_text = raw_catalog_text(Bytes{0xff, 0xfe, 0x41});
    check(buffer_text["reader"]["status"] == "unresolved" &&
              buffer_text["interpretation"]["status"] == "unavailable_text",
          "a zero-unit native copy boundary is not silently converted into reader fallback");
    buffer_text = raw_catalog_text({});
    check(buffer_text["reader"]["native_getter_return_code"] == 0 &&
              buffer_text["interpretation"]["status"] == "rejected_empty_reference",
          "a present zero-byte string succeeds empty instead of taking missing-key fallback");
    auto short_marker_payload = string_payload(3, "");
    put(short_marker_payload, 4, 2, 4);
    auto short_marker = catalog({linkage(short_marker_payload, 0x56d2)})[0]["resource_reference"];
    check(short_marker["reader"]["reason"] == "marker_exceeds_declared_string" &&
              short_marker["interpretation"]["status"] == "unavailable_text",
          "marker probing sees linkage suffix bytes but rejects declared-size underflow");
    put(short_marker_payload, 4, 0xffff, 4);
    short_marker =
        catalog({linkage(short_marker_payload, 0x56d2),
                 linkage(string_payload(3, "later.p3d"), 0x56d2)})[0]["resource_reference"];
    check(short_marker["reader"]["reason"] == "declared_string_exceeds_linkage" &&
              short_marker["reader"]["value"].is_null() &&
              short_marker["interpretation"]["status"] == "unavailable_text",
          "unsafe first resource read neither falls through to later links nor invents a failure "
          "result");
    project = interpret("$(_P3DPROJECT)\\C:stone.pal");
    check(project["status"] == "unresolved_path_syntax" && !project.contains("project_member_name"),
          "drive and scheme syntax requires its own native path rules");
    project = interpret(std::string("$(_P3DPROJECT)\\") + std::string(260, 'x') + ".pal");
    check(project["status"] == "unresolved_path_component_limit",
          "native path buffer limits prevent a fabricated member name");
    project = interpret(std::string("$(_P3DPROJECT)\\") + std::string(512, 'x'));
    check(project["status"] == "decoded" &&
              project["reader_fallback"] == "resource_string_read_failed" &&
              project["member_name"] == "" && project["lookup_request"].is_null(),
          "native buffer failure uses the default resource descriptor without using truncated path "
          "text");
    std::string with_nul = "$(_P3DPROJECT)\\first.pal";
    with_nul.push_back('\0');
    with_nul += "second.p3d";
    c = catalog({linkage(string_payload(3, with_nul), 0x56d2)});
    project = c[0]["resource_reference"]["interpretation"];
    check(project["lookup_request"]["reference"] == "first.pal" &&
              project["ignored_text_suffix"] == std::string("\0second.p3d", 11) &&
              c[0]["resource_reference"]["value"] == with_nul,
          "native resource input stops at the first NUL while the full decoded source survives");
    auto lib = interpret("$(_P3DLIB)|shared.p3d|folder/Stone-Palette.pal");
    check(lib["status"] == "decoded" && lib["kind"] == "library_resource" &&
              lib["library_reference"] == "shared.p3d" &&
              lib["member_source"] == "folder/Stone-Palette.pal" &&
              lib["member_name"] == "Stone-Palette" &&
              lib["primary_context"] == "current_resource_context" &&
              lib["secondary_context_rule"] == "library_resource_service" &&
              lib["lookup_request"]["reference"] == "shared.p3d" &&
              lib["lookup_request"]["search_path_setting"] == "P3d_Material" &&
              !lib["lookup_request"].contains("failure_context") &&
              lib["lookup_status"] == "not_performed",
          "library resource separates the library lookup from the member name without a guessed "
          "failure fallback");
    lib = interpret("$(_P3DLIB)|outer.p3d|$(_P3DLIB)|inner.p3d|dir/name.pal");
    check(lib["status"] == "decoded" && lib["library_reference"] == "outer.p3d" &&
              lib["discarded_nested_library_reference"] == "inner.p3d" &&
              lib["member_path"] == "dir/name.pal" && lib["member_name"] == "name" &&
              lib["lookup_request"]["reference"] == "outer.p3d",
          "one nested library wrapper is removed without substituting its library for the outer "
          "lookup");
    lib = interpret("$(_P3DLIB)|outer|$(_P3DLIB)|middle|$(_P3DLIB)|inner|name.pal");
    check(lib["member_path"] == "$(_P3DLIB)|inner|name.pal" &&
              lib["member_name"] == "$(_P3DLIB)|inner|name" &&
              lib["discarded_nested_library_reference"] == "middle",
          "native nested library removal is one level rather than recursive expansion");
    lib = interpret("$(_P3DLIB)");
    check(lib["status"] == "decoded" && lib["library_reference"] == "" &&
              lib["member_name"] == "" && lib["lookup_request"]["reference"] == "",
          "the bare library marker has explicit empty outputs and is distinct from an empty source "
          "reference");
    lib = interpret("$(_p3dlIb)|Some.P3D|Name.PAL");
    check(lib["status"] == "decoded" && lib["member_name"] == "Name" &&
              lib["library_reference"] == "Some.P3D",
          "unambiguous ASCII case variants retain the library spelling");
    check(interpret("$(_p3dlib)|lib|name")["status"] == "unresolved_prefix_locale" &&
              interpret("$(_P3DLIB)|lib|$(_p3dlib)|inner|name")["status"] ==
                  "unresolved_prefix_locale",
          "native locale-dependent I comparison is not silently replaced by C-locale lowercase "
          "matching");
    lib = interpret("$(_P3DLIB)extra|lib|name.pal");
    check(lib["prefix_suffix"] == "extra" && lib["member_name"] == "name" &&
              lib["library_reference"] == "lib",
          "native prefix checking does not discard or reject text before the first pipe");
    lib = interpret("$(_P3DLIB)||lib|name.pal");
    check(lib["library_reference"] == "|lib" && lib["member_name"] == "name",
          "the second native separator search skips the immediately adjacent pipe");
    lib = interpret("$(_P3DLIB)|lib|name|tail.pal");
    check(lib["member_path"] == "name|tail.pal" && lib["member_name"] == "name|tail",
          "later pipes remain part of the member source instead of arbitrary token splitting");
    lib = interpret("$(_P3DLIB)|lib|");
    check(lib["status"] == "decoded" && lib["member_name"] == "",
          "empty library member is preserved");
    for (const auto &value : {"$(_P3DLIB)\\name.pal", "$(_P3DLIB)|only", "$(_P3DLIB)||name",
                              "$(_P3DLIB)|lib|$(_P3DLIB)"})
        check(interpret(value)["status"] == "unresolved_library_syntax",
              "incomplete library separators are diagnosed without emulating native unsigned-index "
              "wraparound");
    check(interpret("$(_P3DLIB)|C:\\library.p3d|name.pal")["lookup_request"]["reference"] ==
                  "C:\\library.p3d" &&
              interpret("$(_P3DLIB)|lib|C:\\name.pal")["status"] == "unresolved_path_syntax",
          "opaque library resource paths are preserved while unconfirmed member drive syntax stays "
          "unresolved");
    std::u16string chinese = u"$(_P3DLIB)|共享库.p3d|目录\\石材.pal";
    Bytes chinese_payload;
    append(chinese_payload, 3, 4);
    append(chinese_payload, 2 + chinese.size() * 2, 4);
    append(chinese_payload, 0xfeff, 2);
    for (auto unit : chinese)
        append(chinese_payload, unit, 2);
    c = catalog({linkage(chinese_payload, 0x56d2)});
    lib = c[0]["resource_reference"]["interpretation"];
    check(lib["status"] == "decoded" && lib["library_reference"] == "共享库.p3d" &&
              lib["member_name"] == "石材" &&
              bytesof(c[0]["strings"][0]["payload"]) == chinese_payload,
          "library and member Unicode text are decoded from native UTF16 without losing source "
          "bytes");
    auto ordinary = interpret("../folder.with.dot/Stone.PaL");
    check(ordinary["descriptor_mode"].is_null() &&
              ordinary["descriptor_mode_rule"] ==
                  "one_if_palette_loaded_without_matching_resource_member",
          "ordinary palette descriptor mode depends on both resource lookup and member existence");
    check(ordinary["status"] == "decoded" && ordinary["kind"] == "palette_resource" &&
              ordinary["member_name"] == "Stone" && ordinary["extension"] == "PaL" &&
              ordinary["lookup_request"]["reference"] == "../folder.with.dot/Stone.PaL" &&
              ordinary["lookup_request"]["search_path_setting"] == "P3d_Material" &&
              ordinary["primary_context"].is_null() && ordinary["lookup_status"] == "not_performed",
          "ordinary palettes preserve the full resource path and leave conditional primary context "
          "unevaluated");
    check(ordinary["matching_resource_member"]["status"] == "not_evaluated" &&
              ordinary["matching_resource_member"]["source_string_key"] == 3 &&
              ordinary["matching_resource_member"]["member_name"] == "Stone" &&
              ordinary["matching_resource_member"]["comparison"] ==
                  "native_case_insensitive_locale_dependent",
          "palette membership compares resource member names rather than display names or catalog "
          "name key one");
    check(ordinary["context_cases"] ==
              Json::array({{{"lookup", "failure"},
                            {"descriptor_mode", 0},
                            {"matching_resource_member", nullptr},
                            {"primary_context", "current_resource_context"},
                            {"secondary_context", "current_resource_context"}},
                           {{"lookup", "success"},
                            {"descriptor_mode", 0},
                            {"matching_resource_member", true},
                            {"primary_context", "current_resource_context"},
                            {"secondary_context", "resolved_palette_resource"}},
                           {{"lookup", "success"},
                            {"descriptor_mode", 1},
                            {"matching_resource_member", false},
                            {"primary_context", "resolved_palette_resource"},
                            {"secondary_context", "resolved_palette_resource"}}}),
          "ordinary palette context choice preserves all native lookup and membership branches");
    check(interpret("$(_P3DPROJECT)\\Stone.pal")["descriptor_mode"] == 0 &&
              interpret("$(_P3DLIB)|library|Stone.pal")["descriptor_mode"] == 0 &&
              interpret("Stone.p3d")["descriptor_mode"] == 0 &&
              interpret("Stone")["descriptor_mode"] == 0,
          "project library document and extensionless readers all explicitly use descriptor mode "
          "zero");
    check(!interpret("").contains("descriptor_mode") &&
              !interpret("image.jpg").contains("descriptor_mode"),
          "rejected resource references do not gain a usable descriptor mode");
    for (const auto &value : {"scene.p3d", "folder/Scene.P3D", "name"}) {
        ordinary = interpret(value);
        check(ordinary["status"] == "decoded" && ordinary["kind"] == "current_context_resource" &&
                  ordinary["primary_context"] == "current_resource_context" &&
                  ordinary["secondary_context_rule"] == "primary_context" &&
                  ordinary["lookup_request"].is_null(),
              "ordinary p3d and extensionless references use current contexts without fabricating "
              "an external file "
              "request");
    }
    for (const auto &value : {".jpg", "picture.jpg", "name.pal.extra"}) {
        ordinary = interpret(value);
        check(ordinary["status"] == "rejected_extension" && !ordinary.contains("lookup_request"),
              "ordinary nonempty extensions require the native pal or p3d branch");
    }
    for (const auto &value : {".", "folder/"}) {
        ordinary = interpret(value);
        check(ordinary["status"] == "decoded" && ordinary["member_name"] == "" &&
                  ordinary["secondary_context_rule"] == "primary_context",
              "an empty extension takes the native current-context branch before extension "
              "rejection");
    }
    ordinary = interpret(".pal");
    check(ordinary["kind"] == "palette_resource" && ordinary["primary_context"].is_null() &&
              ordinary["lookup_request"]["reference"] == ".pal" &&
              ordinary["matching_resource_member"]["member_name"] == "",
          "ordinary pal with an empty member still uses palette membership selection because its "
          "extension is nonempty");
    auto table_record = [&](std::uint32_t count, unsigned flags = 0x40, unsigned subtype = 18) {
        const std::size_t count_at = (flags & 0x20) ? 108 : 36;
        Bytes b(count_at + 8, 0);
        put(b, 4, 10, 2);
        put(b, 6, flags, 2);
        put(b, 8, (b.size() - 4) / 2, 4);
        put(b, 12, (b.size() - 4) / 2, 4);
        put(b, 16, subtype, 4);
        put(b, 20, 77, 8);
        put(b, count_at, count, 4);
        put(b, count_at + 4, 0x12345678, 4);
        return b;
    };
    auto child_record = [&]() {
        auto b = catalog_record({linkage(string_payload(1, "same"), 0x56d2)});
        put(b, 6, 0x80, 2);
        return b;
    };
    auto native_rows = [&](const std::vector<Bytes> &source) {
        Bytes joined;
        for (const auto &b : source)
            joined.insert(joined.end(), b.begin(), b.end());
        auto result = parse_native(joined);
        for (auto &n : result)
            n["stream"] = Json::array({"root", "system", "native"});
        return result;
    };
    auto inspect_tables = [&](const Json &source) {
        auto catalog = native_material_catalog_records(source);
        auto tables = native_material_catalog_tables(source, catalog);
        return Json{{"catalog", catalog}, {"tables", tables}};
    };
    auto source =
        native_rows({child_record(), table_record(4), child_record(), table_record(1, 0xc0),
                     child_record(), child_record(), table_record(1), child_record()});
    auto original_source = source;
    auto membership = inspect_tables(source);
    const auto &tables = membership["tables"];
    const auto &entries = membership["catalog"];
    check(
        tables.size() == 3 && tables[0]["membership_status"] == "resolved" &&
            tables[0]["declared_descendant_count"] == 4 &&
            tables[0]["member_record_indices"] == Json::array({2, 3, 5}) &&
            tables[1]["member_record_indices"] == Json::array({4}) &&
            tables[2]["member_record_indices"] == Json::array({7}),
        "material table counts include descendants while membership includes direct children only");
    check(tables[0]["members"][0]["catalog_record_index"] == 1 &&
              tables[0]["members"][1]["catalog_record_index"].is_null() &&
              tables[0]["members"][2]["catalog_record_index"] == 3 &&
              entries[0]["table_memberships"].empty() &&
              entries[2]["table_memberships"] ==
                  Json::array({{{"table_index", 1}, {"member_index", 0}}}) &&
              entries[4]["table_memberships"] ==
                  Json::array({{{"table_index", 2}, {"member_index", 0}}}),
          "table membership links exact records without merging duplicate IDs or adopting orphans");
    check(source == original_source && tables[0]["native_record_index"] == 1 &&
              tables[0]["record_offset"] == source[1]["offset"] &&
              tables[0]["stream"] == source[1]["stream"] &&
              tables[0]["runtime_selection"] == "not_evaluated" &&
              bytesof(tables[0]["unassigned_ranges"][0]["data"]) == Bytes({0x78, 0x56, 0x34, 0x12}),
          "table source locations and unnamed suffix survive without claiming runtime table "
          "selection");
    auto extended_table = table_record(1, 0x60);
    put(extended_table, 36, 0xffffffff, 4);
    auto leaf = child_record();
    put(leaf, 6, 0xc0, 2);
    membership = inspect_tables(native_rows({extended_table, leaf}));
    check(membership["tables"][0]["membership_status"] == "resolved" &&
              membership["tables"][0]["descendant_count_source_offset"] == 108 &&
              membership["tables"][0]["declared_descendant_count"] == 1 &&
              bytesof(membership["tables"][0]["unassigned_ranges"][0]["data"]).size() == 72 &&
              membership["tables"][0]["member_record_indices"] == Json::array({1}),
          "extended headers move the count and type 49 remains a leaf even with compound flag set");
    membership = inspect_tables(native_rows({table_record(0), child_record(), table_record(0)}));
    check(membership["tables"].size() == 2 &&
              membership["tables"][0]["membership_status"] == "resolved" &&
              membership["tables"][0]["members"].empty() &&
              membership["catalog"][0]["table_memberships"].empty(),
          "empty material tables do not adopt adjacent records");
    auto other = record({});
    put(other, 6, 0x80, 2);
    membership = inspect_tables(native_rows({table_record(1), other}));
    check(membership["tables"][0]["membership_status"] == "resolved" &&
              membership["tables"][0]["member_record_indices"] == Json::array({1}) &&
              membership["tables"][0]["members"][0]["catalog_record_index"].is_null(),
          "noncatalog source children remain in their original table order");
    for (unsigned type : {10u, 32u, 64u}) {
        auto nested = table_record(1, 0xc0, 19);
        put(nested, 4, type, 2);
        membership =
            inspect_tables(native_rows({table_record(3), nested, child_record(), child_record()}));
        check(membership["tables"].size() == 1 &&
                  membership["tables"][0]["member_record_indices"] == Json::array({1, 3}) &&
                  membership["catalog"][0]["table_memberships"].empty(),
              "native compound classification excludes nested descendants from outer catalog "
              "membership");
    }
    auto valid = native_rows({table_record(2), child_record(), child_record()});
    auto unflagged_child = child_record();
    put(unflagged_child, 6, 0, 2);
    auto unflagged_rows = native_rows({table_record(2), unflagged_child, unflagged_child});
    membership = inspect_tables(unflagged_rows);
    check(membership["tables"][0]["membership_status"] == "resolved" &&
              membership["tables"][0]["member_record_indices"] == Json::array({1, 2}) &&
              membership["tables"][0]["unflagged_descendant_record_indices"] ==
                  Json::array({1, 2}) &&
              unflagged_rows[1]["element_flags"] == 0 &&
              membership["catalog"][1]["table_memberships"][0]["member_index"] == 1,
          "native input reconstructs child flags from counts without modifying source flags or "
          "losing membership");
    for (unsigned damage : {0u, 1u, 3u, 4u, 5u, 6u}) {
        auto bad = valid;
        if (damage == 0)
            bad[2]["stream"] = Json::array({"other"});
        if (damage == 1)
            bad[2]["offset"] = bad[2]["offset"].get<std::size_t>() + 2;
        if (damage == 3)
            bad = native_rows({table_record(UINT32_MAX), child_record()});
        if (damage == 4) {
            auto b = table_record(0);
            b.resize(36);
            put(b, 8, 16, 4);
            put(b, 12, 16, 4);
            bad = native_rows({b, child_record()});
        }
        if (damage == 5)
            bad = native_rows({table_record(2), table_record(2, 0xc0, 19), child_record()});
        if (damage == 6) {
            auto b = other;
            put(b, 4, 10, 2);
            put(b, 16, 19, 4);
            bad = native_rows({table_record(1), b});
        }
        membership = inspect_tables(bad);
        bool unowned = true;
        for (const auto &entry : membership["catalog"])
            unowned = unowned && entry["table_memberships"].empty();
        check(membership["tables"].size() == 1 &&
                  membership["tables"][0]["membership_status"] == "invalid" &&
                  membership["tables"][0].contains("membership_error") &&
                  membership["tables"][0]["members"].empty() && unowned,
              "invalid table spans publish no partial membership and retain independent catalog "
              "records");
    }
    auto skipped_child = child_record();
    put(skipped_child, 6, 0x88, 2);
    membership = inspect_tables(native_rows({table_record(1, 0x48), skipped_child}));
    check(membership["tables"][0]["initial_load_filter"]["status"] == "skipped" &&
              membership["tables"][0]["initial_load_filter"]["reason"] == "flag_0008" &&
              membership["catalog"][0]["initial_load_filter"]["status"] == "skipped" &&
              membership["catalog"][0]["catalog_name"]["value"] == "same" &&
              membership["catalog"][0]["table_memberships"].size() == 1,
          "input-skipped catalog records and tables still retain all source content and stored "
          "relationships");
    for (std::uint32_t words : {16u, 65535u, 65536u}) {
        auto b = catalog_record({});
        b.resize(4 + std::size_t(words) * 2);
        put(b, 8, words, 4);
        auto result = native_material_catalog_records(parse_native(b));
        const auto &filter = result[0]["initial_load_filter"];
        check(result.size() == 1 && filter["record_word_count"] == words &&
                  filter["status"] == (words <= 65535 ? "accepted" : "read_error") &&
                  (words <= 65535 ? filter["reason"].is_null()
                                  : filter["reason"] == "record_word_count_exceeds_limit") &&
                  (words <= 65535 ? filter["native_error_code"].is_null()
                                  : filter["native_error_code"] == 0x12005),
              "native input accepts 16 through 65535 words but retained oversized records are not "
              "load candidates");
    }
    membership = inspect_tables(native_rows({table_record(0)}));
    check(membership["tables"][0]["initial_load_filter"]["status"] == "accepted" &&
              membership["tables"][0]["initial_load_filter"]["record_word_count"] == 20 &&
              membership["tables"][0]["runtime_selection"] == "not_evaluated",
          "passing the input filter is distinct from runtime table selection");
    auto input_members = [&](const std::vector<Bytes> &records) {
        return inspect_tables(native_rows(records))["tables"][0]["initial_load_members"];
    };
    auto input = input_members(
        {table_record(4), child_record(), table_record(1, 0xc0), child_record(), child_record()});
    check(input["status"] == "resolved" && input["start_counter"] == 1 &&
              input["target_counter"] == 5 && input["end_counter"] == 5 &&
              input["member_record_indices"] == Json::array({1, 2, 4}) &&
              input["members"][0]["catalog_record_index"] == 0 &&
              input["members"][1]["catalog_record_index"].is_null(),
          "initial table input keeps direct children and waits for complete nested children");
    input = input_members({table_record(1), skipped_child, child_record(), child_record()});
    check(input["status"] == "resolved" && input["target_counter"] == 2 &&
              input["end_counter"] == 3 && input["member_record_indices"] == Json::array({2}) &&
              input["consumed_record_indices"] == Json::array({1, 2}) &&
              input["skipped_record_indices"] == Json::array({1}),
          "a skipped final child increments the counter but read-next-child still obtains the "
          "following accepted record before its parent tests completion");
    input = input_members({table_record(2), skipped_child, child_record()});
    check(input["status"] == "resolved" && input["end_counter"] == 3 &&
              input["member_record_indices"] == Json::array({2}),
          "a counted skip can consume part of the declared span without creating a member");
    auto prefixed = child_record();
    put(prefixed, 0, 9, 4);
    membership = inspect_tables(native_rows({table_record(1), prefixed, child_record()}));
    input = membership["tables"][0]["initial_load_members"];
    check(input["status"] == "resolved" && input["end_counter"] == 2 &&
              input["member_record_indices"] == Json::array({2}) &&
              input["skipped_record_indices"] == Json::array({1}) &&
              membership["catalog"][0]["initial_load_filter"]["source_prefix_word"] == 9 &&
              membership["catalog"][0]["initial_load_filter"]["reason"] == "nonzero_prefix" &&
              membership["catalog"][0]["initial_load_filter"]["descendant_counter_effect"] ==
                  "not_incremented" &&
              membership["tables"][0]["membership_status"] == "invalid" &&
              membership["catalog"][0]["table_memberships"].empty(),
          "nonzero source prefixes skip physical records without counting or inventing raw "
          "span ownership, while conditional input members remain reconstructible");
    input = input_members({prefixed, skipped_child, table_record(1), child_record()});
    check(input["status"] == "resolved" && input["start_counter"] == 2 &&
              input["end_counter"] == 3 && input["member_record_indices"] == Json::array({3}),
          "the per-block counter includes previous element-reader skips but excludes prefix skips");
    auto zero_type = other;
    put(zero_type, 4, 0, 2);
    input = input_members({table_record(1), zero_type, child_record()});
    check(input["status"] == "resolved" && input["end_counter"] == 3 &&
              input["skipped_record_indices"] == Json::array({1}) &&
              input["member_record_indices"] == Json::array({2}),
          "zero-type records are counted element-reader skips");
    auto skipped_compound = table_record(100, 0x48, 19);
    input = input_members({table_record(2), skipped_compound, child_record()});
    check(input["status"] == "resolved" && input["member_record_indices"] == Json::array({2}),
          "a skipped compound header does not consume its declared descendants recursively");
    input = input_members({table_record(2), table_record(1, 0xc0, 19), skipped_child,
                           child_record(), child_record()});
    check(input["status"] == "resolved" && input["end_counter"] == 4 &&
              input["member_record_indices"] == Json::array({1}) &&
              input["consumed_record_indices"] == Json::array({1, 2, 3}),
          "nested read-next-child may finish beyond both stored counts before the outer table "
          "returns, without promoting the nested accepted record");
    input = input_members({table_record(2), child_record(), skipped_child});
    check(input["status"] == "invalid" && input["members"].empty() &&
              input["member_record_indices"].empty() && input.contains("error") &&
              input["consumed_record_indices"] == Json::array({1, 2}),
          "missing accepted child after a skip invalidates the whole input table without partial "
          "published members");
    auto oversized = catalog_record({});
    oversized.resize(4 + 65536 * 2);
    put(oversized, 8, 65536, 4);
    put(oversized, 0, 1, 4);
    put(oversized, 6, 8, 2);
    membership = inspect_tables(native_rows({table_record(2), child_record(), oversized}));
    input = membership["tables"][0]["initial_load_members"];
    check(input["status"] == "read_error" && input["native_error_code"] == 0x12005 &&
              input["error_record_index"] == 2 && input["members"].empty() &&
              membership["catalog"][1]["initial_load_filter"]["status"] == "read_error" &&
              membership["catalog"][1]["initial_load_filter"]["descendant_counter_effect"] ==
                  "not_reached",
          "oversize read errors take precedence over both prefix and flag skips and discard the "
          "partial table result");
    for (unsigned type : {13u, 24u, 62u}) {
        auto conversion = other;
        put(conversion, 4, type, 2);
        input = input_members({table_record(2), child_record(), conversion});
        check(input["status"] == "invalid" && input.contains("error") && input["members"].empty(),
              "truncated conversion inputs fail without publishing a partial table");
    }
    auto typed_compound = [&](unsigned type, unsigned flags, std::uint32_t count) {
        auto b = table_record(count, flags, 19);
        put(b, 4, type, 2);
        return b;
    };
    for (unsigned flags : {0x80u, 0xa0u}) {
        auto converted = typed_compound(24, flags, 1);
        input = input_members({table_record(3), converted, child_record(), child_record()});
        const auto &conversion = input["record_conversions"][0];
        check(input["status"] == "resolved" &&
                  input["member_record_indices"] == Json::array({1, 3}) &&
                  conversion["native_record_index"] == 1 && conversion["compound"] == true &&
                  conversion["descendant_count"] == 1 &&
                  conversion["applied"] == bool(flags & 0x20) &&
                  conversion["output_element_flags"] == 0x80 &&
                  conversion["output_base_word_count"] == 20 &&
                  conversion["output_record_word_count"] == 20 &&
                  conversion["descendant_count_source_offset"] == ((flags & 0x20) ? 108 : 36),
              "type 24 input removes its optional extension and reads the same nested count "
              "from the relocated header");
    }
    auto legacy_13 = [&](unsigned flags, unsigned entries) {
        Bytes b(348 + entries * 16, 0);
        put(b, 4, 13, 2);
        put(b, 6, flags, 2);
        put(b, 8, (b.size() - 4) / 2, 4);
        put(b, 12, (b.size() - 4) / 2, 4);
        put(b, 36, 1, 4);
        put(b, 346, entries, 2);
        return b;
    };
    for (unsigned entries : {0u, 1u, 2500u}) {
        auto converted = legacy_13(0x80, entries);
        input = input_members({table_record(3), converted, child_record(), child_record()});
        const auto &conversion = input["record_conversions"][0];
        check(input["status"] == "resolved" &&
                  input["member_record_indices"] == Json::array({1, 3}) &&
                  conversion["kind"] == "type_13_legacy_layout_upgrade" &&
                  conversion["applied"] == true && conversion["legacy_entry_count"] == entries &&
                  conversion["output_base_word_count"] == 184 + 8 * entries &&
                  conversion["output_record_word_count"] == 184 + 8 * entries &&
                  conversion["descendant_count_source_offset"] == 36 &&
                  conversion["descendant_count_source_bytes"] == 4 &&
                  conversion["payload_reconstruction"] == "base_reconstructed" &&
                  conversion["upgraded_base"]["bytes"] == 372 + 16 * entries,
              "type 13 legacy upgrade preserves the ordinary descendant count and records "
              "the twelve-word header growth with a reconstructed base payload");
    }
    auto extended_13 = legacy_13(0xa0, 0);
    put(extended_13, 64, 1, 2);
    put(extended_13, 66, 0xffff, 2);
    put(extended_13, 108, 100, 4);
    auto original_conversion_rows =
        native_rows({table_record(3), extended_13, child_record(), child_record()});
    auto unchanged_conversion_rows = original_conversion_rows;
    membership = inspect_tables(original_conversion_rows);
    input = membership["tables"][0]["initial_load_members"];
    check(input["status"] == "resolved" && input["member_record_indices"] == Json::array({1, 3}) &&
              input["record_conversions"][0]["descendant_count"] == 1 &&
              input["record_conversions"][0]["descendant_count_source_offset"] == 64 &&
              input["record_conversions"][0]["descendant_count_source_bytes"] == 2 &&
              original_conversion_rows == unchanged_conversion_rows,
          "extended type 13 legacy upgrade uses the repeated old u16 and zero padding for "
          "the new count rather than old count or neighboring bytes, preserving source data");
    for (unsigned discriminator : {160u, 342u, 344u, 346u, 12u}) {
        auto modern = legacy_13(0x80, 0);
        if (discriminator == 160)
            put(modern, 160, 1, 4);
        else if (discriminator == 346)
            put(modern, 346, 2501, 2);
        else if (discriminator == 12) {
            modern.resize(modern.size() + 2);
            put(modern, 8, (modern.size() - 4) / 2, 4);
            put(modern, 12, (modern.size() - 4) / 2, 4);
        } else
            put(modern, discriminator, 1, 2);
        input = input_members({table_record(2), modern, child_record()});
        check(input["status"] == "resolved" && input["member_record_indices"] == Json::array({1}) &&
                  input["record_conversions"][0]["applied"] == false &&
                  input["record_conversions"][0]["descendant_count"] == 1,
              "each nonlegacy type 13 discriminator preserves the original layout and count");
    }
    auto reference_62 = other;
    reference_62.resize(260, 0);
    put(reference_62, 4, 62, 2);
    put(reference_62, 6, 0xe0, 2);
    put(reference_62, 8, 128, 4);
    put(reference_62, 12, 128, 4);
    input = input_members({table_record(2), reference_62, child_record()});
    check(input["status"] == "resolved" && input["member_record_indices"] == Json::array({1, 2}) &&
              input["record_conversions"][0]["kind"] == "type_62_matrix_repair" &&
              input["record_conversions"][0]["compound"] == false &&
              input["record_conversions"][0]["descendant_count"] == 0 &&
              input["record_conversions"][0]["matrix_source_offset"] == 164 &&
              input["record_conversions"][0]["payload_reconstruction"] == "not_evaluated",
          "type 62 matrix conversion leaves the input record a leaf even with compound and "
          "extended-header flags and does not claim repaired geometry");
    input = input_members({table_record(UINT32_MAX), child_record()});
    check(input["status"] == "resolved" && input["start_counter"] == 1 &&
              input["target_counter"] == 0 && input["end_counter"] == 1 &&
              input["members"].empty() && input["consumed_record_indices"].empty(),
          "the native unsigned target addition wraps before the initial comparison");
    input = input_members({table_record(1, 0x48), child_record()});
    check(input["status"] == "skipped" && input["reason"] == "flag_0008" &&
              input["members"].empty() && input["consumed_record_indices"].empty(),
          "a skipped table does not enter its child reader");
    auto separated = native_rows({child_record(), table_record(1), child_record()});
    separated[0]["stream"] = Json::array({"another", "block"});
    input = inspect_tables(separated)["tables"][0]["initial_load_members"];
    check(input["status"] == "resolved" && input["start_counter"] == 1 && input["end_counter"] == 2,
          "other input blocks do not contribute to the table counter");
    for (bool different_stream : {false, true}) {
        auto bad = native_rows({table_record(1), child_record()});
        if (different_stream)
            bad[1]["stream"] = Json::array({"another", "block"});
        else
            bad[1]["offset"] = bad[1]["offset"].get<std::size_t>() + 4;
        input = inspect_tables(bad)["tables"][0]["initial_load_members"];
        check(input["status"] == "invalid" && input["members"].empty(),
              "conditional input refuses gaps and does not fetch children from another stream");
    }
    auto name_payload = [&](const Bytes &name, std::uint16_t declared) {
        Bytes p;
        append(p, declared, 2);
        p.insert(p.end(), name.begin(), name.end());
        p.push_back(0);
        if (p.size() % 2)
            p.push_back(0);
        return p;
    };
    const Bytes ascii{'S', 't', 'e', 'e', 'l'};
    auto named = decode_native_material_name(name_payload(ascii, 5));
    check(named["status"] == "decoded" && named["material_name"] == "Steel" &&
              named["name_offset"] == 6 && named["terminator_offset"] == 11 &&
              named["declared_length_matches"] == true,
          "legacy name linkage decodes byte count and ANSI C string");
    for (auto length : {0u, 2u, 65535u}) {
        auto value = decode_native_material_name(name_payload(ascii, std::uint16_t(length)));
        check(value["material_name"] == "Steel" && value["declared_length_matches"] == false &&
                  value["actual_byte_length"] == 5,
              "native name getter ignores an inconsistent declared byte count");
    }
    auto tail = name_payload(ascii, 5);
    tail.insert(tail.end(), {0x81, 0xfe, 1, 2});
    auto with_tail = decode_native_material_name(tail);
    check(with_tail["material_name"] == "Steel" &&
              bytesof(with_tail["ignored_suffix"]) == Bytes({0x81, 0xfe, 1, 2}),
          "bytes after first NUL remain an ignored suffix, not part of the name");
    for (const auto &bytes : {Bytes{}, Bytes{0}, Bytes{0, 0}, Bytes{2, 0, 'A', 'B'}})
        check(decode_native_material_name(bytes)["status"] == "invalid",
              "short or unterminated material name cannot read beyond its linkage");
    auto empty_name = decode_native_material_name(name_payload({}, 0));
    check(empty_name["status"] == "decoded" && empty_name["material_name"] == "" &&
              empty_name["lookup_request"]["empty_name_matches"] == false,
          "empty name is a stored value but matches no catalog entry");
    const auto ansi = name_payload({0xe9}, 1);
    auto needs_decoder = decode_native_material_name(ansi);
    check(needs_decoder["status"] == "requires_ansi_decoder" &&
              needs_decoder["material_name"].is_null() &&
              bytesof(needs_decoder["name_bytes"]) == Bytes({0xe9}),
          "high ANSI bytes are not guessed as UTF8 or GBK");
    unsigned decoder_calls = 0;
    auto decoded = decode_native_material_name(ansi, [&](const Bytes &bytes) {
        ++decoder_calls;
        check(bytes == Bytes({0xe9}), "caller receives only the NUL-excluded name bytes");
        return std::string(u8"é");
    });
    check(decoder_calls == 1 && decoded["status"] == "decoded" && decoded["material_name"] == u8"é",
          "caller-selected ANSI decoding yields usable Unicode without a global locale");
    auto failing = decode_native_material_name(ansi, [](const Bytes &) -> std::string {
        throw std::runtime_error("code page unavailable");
    });
    check(failing["status"] == "invalid" && failing["material_name"].is_null() &&
              failing["decode_error"] == "code page unavailable",
          "decoder failure retains source bytes and no fabricated name");
    check(decode_native_material_name(
              ansi, [](const Bytes &) { return std::string(1, char(0xff)); })["status"] ==
              "invalid",
          "caller decoder must return valid UTF8");
    Bytes long_name(70, 'Q');
    check(decode_native_material_name(name_payload(long_name, 70))["actual_byte_length"] == 70,
          "writer's 29-byte truncation is not invented as a reader format limit");
    auto name_link = linkage(name_payload(ascii, 5), 0x4f5a);
    auto names =
        parse_native(record({linkage(Bytes{0, 0}, 0x4f5a), name_link, linkage(payload(27))}));
    check(names[0]["links"][0]["decoded"]["status"] == "invalid" &&
              names[0]["links"][0]["decoded"]["reader_selection"] == "first_match" &&
              names[0]["links"][1]["decoded"]["reader_selection"] == "shadowed" &&
              names[0]["links"][2]["decoded"]["reader_selection"] == "first_match",
          "name and numeric references have independent first-match selection before validation");
    auto name_nonuser = parse_native(record({linkage(Bytes{0, 0, 0, 0}, 0x4f5a, 1), name_link}));
    check(!name_nonuser[0]["links"][0].contains("decoded") &&
              name_nonuser[0]["links"][1]["decoded"]["reader_selection"] == "first_match",
          "non-user linkage app bytes do not consume the native name selection");
    // Exercise selection through the actual block, tree, list and ID readers.
    // Physical stream order intentionally disagrees with logical block order.
    auto system_block = [&](const char *name, const std::vector<Bytes> &parts,
                            unsigned capacity = 1, unsigned flags = 0) {
        Bytes decoded, raw(16), control(16);
        for (const auto &part : parts)
            decoded.insert(decoded.end(), part.begin(), part.end());
        put(control, 0, capacity, 4);
        put(control, 4, flags, 4);
        for (std::size_t i = 0; i < 16; ++i)
            raw[i] = control[i] ^ (i ? raw[i - 1] : 0x26);
        raw.push_back(0);
        return Stream{{"root", "system", name},
                      std::make_shared<Bytes>(raw),
                      std::make_shared<Bytes>(decoded),
                      24};
    };
    auto bootstrap = table_record(0, 0, 8);
    put(bootstrap, 4, 46, 2);
    put(bootstrap, 20, 99, 8);
    auto deleted_member = child_record();
    put(deleted_member, 6, 0x88, 2);
    auto deleted_table = table_record(0, 0x48);
    std::vector<Stream> blocks{
        system_block("b",
                     {table_record(4), child_record(), table_record(1, 0xc0), child_record(),
                      deleted_member, table_record(0)},
                     1, 1),
        system_block("a", {table_record(0), bootstrap, table_record(1, 0x40, 19), table_record(0),
                           deleted_table})};
    Json aliases = {{"P3D-SSYS", "system"}, {"$1", "a"}, {"$2", "b"}};
    Json initial_header = {
        {"status", "resolved"},
        {"initial_probe", {{"action", "read_header_payload"}, {"id_counter", 100}}}};
    auto select = [&]() {
        Json rows = Json::array();
        for (const auto &block : blocks)
            for (auto n : parse_native(*block.decoded)) {
                n["stream"] = block.path;
                rows.push_back(std::move(n));
            }
        const auto container = native_input_containers(blocks, aliases, rows).at(0);
        const auto &list = container.at("list_preparation");
        const auto ids = native_system_id_assignments(list, rows, initial_header);
        return Json{{"selection", native_system_material_table(list, rows, ids)},
                    {"container", container},
                    {"rows", rows},
                    {"ids", ids}};
    };
    auto selected = select();
    auto chosen = selected["selection"]["table"];
    check(selected["selection"]["status"] == "selected" && chosen["native_record_index"] == 0 &&
              chosen["root_input_index"] == 2 && chosen["block_number"] == 2,
          "initial table is the first accepted system root, not prebootstrap, nested or deleted "
          "tables");
    check(chosen["source_id"] == 77 && chosen["assigned_id"] != 77 &&
              chosen["assigned_id"] == selected["ids"]["roots"][2]["records"][0]["assigned_id"],
          "selected table identity follows duplicate-ID registration instead of source-ID merging");
    check(chosen["members"].size() == 3 && chosen["members"][0]["native_record_index"] == 1 &&
              chosen["members"][1]["native_record_index"] == 2 &&
              chosen["members"][2]["native_record_index"] == 5,
          "native read-next skips the flagged terminal child and accepts the following record "
          "before its parent checks the counter; nested descendants stay nested");
    check(chosen["members"][0]["input_occurrence_index"] ==
              selected["ids"]["roots"][2]["records"][1]["input_occurrence_index"],
          "table member identity retains the exact input occurrence");
    auto before = selected;
    auto repeated = select();
    check(repeated == before, "table selection is deterministic and does not mutate input records");
    aliases.erase("$2");
    check(select()["selection"]["status"] == "absent",
          "a missing logical block alias cannot select a physically present later table");
    aliases["$2"] = "b";
    blocks[0] = system_block("b", {table_record(0)}, 0, 1);
    check(select()["selection"]["status"] == "absent",
          "zero-capacity block does not create a table");
    blocks[0] = system_block("b", {table_record(0), child_record()}, 1, 1);
    chosen = select()["selection"]["table"];
    check(chosen["members"].empty(),
          "an empty selected table does not adopt adjacent catalog records");
    blocks[0] = system_block("b", {table_record(1, 0x60), child_record()}, 1, 1);
    check(select()["selection"]["table"]["members"].size() == 1,
          "extended selected tables use prepared child ownership");
    aliases["$2"] = "a";
    aliases["$3"] = "b";
    chosen = select()["selection"]["table"];
    check(chosen["block_number"] == 2 && chosen["native_record_index"] == 2 &&
              chosen["root_input_index"] == 2,
          "repeated physical block aliases preserve occurrences and existing bootstrap state");
    aliases["$2"] = "b";
    aliases.erase("$3");
    blocks[1].compression_offset = 8;
    check(select()["selection"]["status"] == "unresolved",
          "unknown earlier block layout cannot be treated as absence or skipped to choose a later "
          "table");
    blocks[1].compression_offset = 24;
    initial_header["initial_probe"]["action"] = "initialize_blank_header";
    check(select()["selection"]["status"] == "unresolved",
          "blank-header probing does not imply the ordinary fresh-file material table");
    initial_header["initial_probe"]["action"] = "read_header_payload";
    auto partial = selected["container"]["list_preparation"];
    partial["status"] = "partial";
    check(native_system_material_table(partial, selected["rows"], selected["ids"])["status"] ==
              "unresolved",
          "partial list preparation cannot publish a complete initial table");
    partial = selected["container"]["list_preparation"];
    partial["system_bootstrap_required"] = false;
    check(native_system_material_table(partial, selected["rows"], selected["ids"])["reason"] ==
              "system_container_required",
          "ordinary model lists are not system material tables");
    auto wrong_ids = selected["ids"];
    wrong_ids["roots"][2]["records"][0]["native_record_index"] = 1;
    auto rejected = native_system_material_table(selected["container"]["list_preparation"],
                                                 selected["rows"], wrong_ids);
    check(rejected["status"] == "unresolved" && rejected["table"].is_null(),
          "inconsistent assignment provenance does not publish a partial selected table");
    auto zero_id_table = table_record(0);
    put(zero_id_table, 20, 0, 8);
    blocks = {system_block("a", {bootstrap, zero_id_table}, 1, 1)};
    initial_header["initial_probe"]["id_counter"] = std::numeric_limits<std::uint64_t>::max();
    chosen = select()["selection"]["table"];
    check(chosen["native_record_index"] == 1 && chosen["assigned_id"] == 0,
          "an unindexed zero ID after counter wrap still belongs to the root list and can be "
          "selected");
    return checks;
}
