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
        check(interpret(value)["status"] == "unresolved_resource_form",
              "other resource syntaxes do not inherit project-token context rules");
    check(interpret("")["status"] == "rejected_empty_reference" &&
              missing[0]["resource_reference"]["interpretation"]["status"] == "unavailable_text",
          "explicitly empty resources are rejected and missing source text is not reinterpreted");
    project = interpret("$(_P3DPROJECT)\\C:stone.pal");
    check(project["status"] == "unresolved_path_syntax" && !project.contains("project_member_name"),
          "drive and scheme syntax requires its own native path rules");
    project = interpret(std::string("$(_P3DPROJECT)\\") + std::string(260, 'x') + ".pal");
    check(project["status"] == "unresolved_path_component_limit",
          "native path buffer limits prevent a fabricated member name");
    project = interpret(std::string("$(_P3DPROJECT)\\") + std::string(512, 'x'));
    check(project["status"] == "unresolved_conversion_limit",
          "long catalog text does not assume the native fixed-size conversion buffer succeeded");
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
    return checks;
}
