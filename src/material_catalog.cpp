#include "internal.hpp"
#include <codecvt>
#include <locale>

namespace p3d {
namespace {
Json catalog_string_reader(const Json &field, const Json &strings) {
    Json out = {{"reader_profile", "bimbase_2025_catalog_string_buffer"},
                {"buffer_utf16_units", 512},
                {"status", "unresolved"},
                {"value", nullptr}};
    if (field.at("entry_index").is_null()) {
        out.update({{"status", "missing"}, {"native_getter_return_code", 1}});
        return out;
    }
    const auto payload =
        bytesof(strings.at(field.at("entry_index").get<std::size_t>()).at("payload"));
    if (payload.size() < 8) {
        out["reason"] = "truncated_linkage_string_header";
        return out;
    }
    const auto count = Reader(payload, 4).u32();
    if (count > payload.size() - 8) {
        out["reason"] = "declared_string_exceeds_linkage";
        return out;
    }
    const auto source = slice(payload, 8, count);
    unsigned code = 0;
    std::u16string units;
    if (!source.empty() && source.front() != 0) {
        // Marker probing reads the copied linkage, including its suffix. The
        // native copy has an extra zero word beyond the linkage boundary.
        auto probe = [&](std::size_t i) { return 8 + i < payload.size() ? payload[8 + i] : 0; };
        const unsigned first = probe(0) | (probe(1) << 8);
        const unsigned second = probe(2) | (probe(3) << 8);
        bool wide = false;
        std::size_t prefix = 0;
        if (first == 0xfdff) {
            prefix = 2;
            wide = true;
        } else if (first == 0xfeff) {
            prefix = second == 1 ? 4 : 2;
            wide = second != 1;
        } else if (first == 0xfffd || first == 0xfffe) {
            code = 2;
        }
        out["prefix_bytes"] = prefix;
        out["character_conversion"] = code   ? "unsupported_marker"
                                      : wide ? "copy_utf16le_units"
                                             : "widen_unsigned_bytes";
        if (!code) {
            if (prefix > source.size()) {
                out["reason"] = "marker_exceeds_declared_string";
                return out;
            }
            const auto available = (source.size() - prefix) / (wide ? 2 : 1);
            if (wide && source.size() > prefix && available == 0) {
                out["reason"] = "native_zero_unit_copy_boundary";
                return out;
            }
            const auto copied = std::min<std::size_t>(512, available);
            for (std::size_t i = 0; i < copied; ++i) {
                const auto at = prefix + i * (wide ? 2 : 1);
                units.push_back(char16_t(source[at] | (wide ? unsigned(source[at + 1]) << 8 : 0)));
            }
            out["copied_utf16_units"] = copied;
            if (!units.empty() && units.back() != 0 && copied >= 511) {
                units.back() = 0;
                code = 1;
            } else if (!wide && copied < available) {
                code = 1;
            }
            // The UTF16 branch compares the capacity against the *copied*
            // count. A zero at its last copied unit can therefore mask a tail
            // beyond the fixed buffer; preserve that native distinction.
        }
    }
    out["conversion_return_code"] = code;
    out["native_getter_return_code"] = code ? 1 : 0;
    out["status"] = code ? "failure" : "success";
    out["output_utf16_units"] = Json::array();
    for (auto unit : units)
        out["output_utf16_units"].push_back(unsigned(unit));
    try {
        std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> codec;
        const auto nul = units.find(u'\0');
        if (nul != std::u16string::npos) {
            try {
                const auto suffix = units.substr(nul);
                const auto text = codec.to_bytes(suffix);
                require(codec.converted() == suffix.size(), "incomplete suffix UTF16 conversion");
                out["ignored_text_suffix"] = text;
            } catch (const std::exception &e) {
                out["ignored_suffix_text_error"] = e.what();
            }
            units.resize(nul);
        }
        out["value"] = codec.to_bytes(units);
        require(codec.converted() == units.size(), "incomplete UTF16 conversion");
    } catch (const std::exception &e) {
        out["value"] = nullptr;
        out["text_error"] = e.what();
    }
    return out;
}
Json native_catalog_input_members(std::size_t ni, const Json &records,
                                  const std::map<std::size_t, std::size_t> &catalog_indices,
                                  std::uint32_t counter) {
    auto out = native_record_input_subtree(ni, records, counter);
    out["scope"] = "table_input_if_reached";
    out["members"] = Json::array();
    if (out["status"] == "resolved")
        for (const auto &index : out["member_record_indices"]) {
            const auto ci = catalog_indices.find(index.get<std::size_t>());
            out["members"].push_back({{"native_record_index", index},
                                      {"catalog_record_index",
                                       ci == catalog_indices.end() ? Json() : Json(ci->second)}});
        }
    return out;
}
Json catalog_resource_interpretation(const Json &field) {
    Json out = {{"reader_profile", "bimbase_2025_material_catalog_resource"},
                {"scope", "source_text_if_reader_succeeds"},
                {"status", "unavailable_text"},
                {"lookup_status", "not_performed"}};
    const auto &reader = field.at("reader");
    if (reader.at("status") == "missing" || reader.at("status") == "failure") {
        // A failed getter takes a different branch from a successfully read,
        // explicitly empty resource string. Source text remains unchanged.
        out.update(
            {{"scope", "resource_reader_failure_fallback"},
             {"status", "decoded"},
             {"kind", "current_context_resource"},
             {"reader_fallback", reader.at("status") == "missing" ? "missing_resource_reference"
                                                                  : "resource_string_read_failed"},
             {"member_name", ""},
             {"descriptor_mode", 0},
             {"primary_context", "current_resource_context"},
             {"secondary_context_initial_state", "unset"},
             {"secondary_context_rule", "default_resource_service"},
             {"lookup_request", nullptr}});
        return out;
    }
    if (reader.at("status") != "success" || !reader.at("value").is_string())
        return out;
    if (reader.contains("ignored_text_suffix"))
        out["ignored_text_suffix"] = reader.at("ignored_text_suffix");
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> codec;
    const auto source = reader.at("value").get<std::string>();
    std::u16string text;
    try {
        text = codec.from_bytes(source);
        if (codec.converted() != source.size())
            return out;
    } catch (const std::range_error &) {
        return out;
    }
    const auto nul = text.find(u'\0');
    if (nul != std::u16string::npos) {
        out["ignored_text_suffix"] = codec.to_bytes(text.substr(nul));
        text.resize(nul);
    }
    if (text.empty()) {
        out.update({{"status", "rejected_empty_reference"}, {"kind", "empty"}});
        return out;
    }
    auto ascii_lower = [](char16_t c) { return c >= u'A' && c <= u'Z' ? char16_t(c + 32) : c; };
    auto starts = [&](const std::u16string &s, const std::u16string &prefix) {
        if (s.size() < prefix.size())
            return false;
        for (std::size_t i = 0; i < prefix.size(); ++i)
            if (ascii_lower(s[i]) != ascii_lower(prefix[i]))
                return false;
        return true;
    };
    const std::u16string library_prefix = u"$(_P3DLIB)";
    if (starts(text, library_prefix)) {
        out["kind"] = "library_resource";
        // Unlike PROJECT, LIB contains I: lowercase i versus uppercase I
        // depends on the native CRT locale. Do not silently assume the C locale.
        if (text[7] != u'I') {
            out["status"] = "unresolved_prefix_locale";
            return out;
        }
        std::u16string library, member;
        if (text.size() != library_prefix.size()) {
            const auto first = text.find(u'|');
            // The native search starts two units after the first separator,
            // so adjacent separators are not ordinary empty-field splitting.
            const auto second = first == std::u16string::npos ? first : text.find(u'|', first + 2);
            if (first == std::u16string::npos || second == std::u16string::npos) {
                out["status"] = "unresolved_library_syntax";
                return out;
            }
            out["prefix_suffix"] =
                codec.to_bytes(text.substr(library_prefix.size(), first - library_prefix.size()));
            library = text.substr(first + 1, second - first - 1);
            member = text.substr(second + 1);
        }
        out["library_reference"] = codec.to_bytes(library);
        out["member_source"] = codec.to_bytes(member);
        if (starts(member, library_prefix)) {
            if (member[7] != u'I') {
                out["status"] = "unresolved_prefix_locale";
                return out;
            }
            const auto first = member.find(u'|');
            const auto second =
                first == std::u16string::npos ? first : member.find(u'|', first + 2);
            if (first == std::u16string::npos || second == std::u16string::npos) {
                out["status"] = "unresolved_library_syntax";
                return out;
            }
            out["discarded_nested_library_reference"] =
                codec.to_bytes(member.substr(first + 1, second - first - 1));
            member = member.substr(second + 1);
        }
        out["member_path"] = codec.to_bytes(member);
        if (member.find(u':') != std::u16string::npos) {
            out["status"] = "unresolved_path_syntax";
            return out;
        }
        const auto separator = member.find_last_of(u"/\\");
        const auto leaf = separator == std::u16string::npos ? member : member.substr(separator + 1);
        const auto dot = leaf.find_last_of(u'.');
        const auto stem = dot == std::u16string::npos ? leaf : leaf.substr(0, dot);
        const auto ext_size = dot == std::u16string::npos ? 0 : leaf.size() - dot;
        if ((separator != std::u16string::npos && separator + 1 >= 260) || stem.size() >= 260 ||
            ext_size >= 260) {
            out["status"] = "unresolved_path_component_limit";
            return out;
        }
        out.update(
            {{"status", "decoded"},
             {"member_name", codec.to_bytes(stem)},
             {"descriptor_mode", 0},
             {"primary_context", "current_resource_context"},
             {"secondary_context_rule", "library_resource_service"},
             {"lookup_request",
              {{"reference", codec.to_bytes(library)}, {"search_path_setting", "P3d_Material"}}}});
        return out;
    }
    const std::u16string prefix = u"$(_P3DPROJECT)\\";
    bool project = text.size() >= prefix.size();
    for (std::size_t i = 0; project && i < prefix.size(); ++i)
        project = ascii_lower(text[i]) == ascii_lower(prefix[i]);
    out["kind"] = project ? "project_resource" : "path_resource";
    // The native path splitter also handles schemes and drive syntax. Keep
    // their still-unconfirmed rules separate from ordinary path components.
    if (text.find(u':') != std::u16string::npos) {
        out["status"] = "unresolved_path_syntax";
        return out;
    }
    const auto separator = text.find_last_of(u"/\\");
    const auto leaf = separator == std::u16string::npos ? text : text.substr(separator + 1);
    const auto dot = leaf.find_last_of(u'.');
    const auto stem = dot == std::u16string::npos ? leaf : leaf.substr(0, dot);
    const auto ext_with_dot = dot == std::u16string::npos ? std::u16string() : leaf.substr(dot);
    // P3DDC's splitter supplies 260-unit buffers for each path component.
    // Out-of-range cases have no portable definite split in this view.
    if ((separator != std::u16string::npos && separator + 1 >= 260) || stem.size() >= 260 ||
        ext_with_dot.size() >= 260) {
        out["status"] = "unresolved_path_component_limit";
        return out;
    }
    const auto extension = ext_with_dot.empty() ? std::u16string() : ext_with_dot.substr(1);
    const bool palette = extension.size() == 3 && ascii_lower(extension[0]) == u'p' &&
                         ascii_lower(extension[1]) == u'a' && ascii_lower(extension[2]) == u'l';
    if (!project && !extension.empty()) {
        out["member_name"] = codec.to_bytes(stem);
        out["extension"] = codec.to_bytes(extension);
        if (palette) {
            out.update(
                {{"status", "decoded"},
                 {"kind", "palette_resource"},
                 {"primary_context", nullptr},
                 {"descriptor_mode", nullptr},
                 {"descriptor_mode_rule", "one_if_palette_loaded_without_matching_resource_member"},
                 {"primary_context_rule", "palette_if_loaded_and_no_matching_resource_member"},
                 {"secondary_context_rule", "palette_lookup_or_current_context"},
                 {"matching_resource_member",
                  {{"status", "not_evaluated"},
                   {"source_string_key", 3},
                   {"member_name", codec.to_bytes(stem)},
                   {"comparison", "native_case_insensitive_locale_dependent"}}},
                 {"lookup_request",
                  {{"reference", codec.to_bytes(text)}, {"search_path_setting", "P3d_Material"}}},
                 {"context_cases",
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
                                {"secondary_context", "resolved_palette_resource"}}})}});
            return out;
        }
        const bool document = extension.size() == 3 && ascii_lower(extension[0]) == u'p' &&
                              extension[1] == u'3' && ascii_lower(extension[2]) == u'd';
        if (!document) {
            out["status"] = "rejected_extension";
            return out;
        }
    }
    if (!project)
        out["kind"] = "current_context_resource";
    out[project ? "project_member_name" : "member_name"] = codec.to_bytes(stem);
    out.update({{"status", "decoded"},
                {"descriptor_mode", 0},
                {"extension", codec.to_bytes(extension)},
                {"primary_context", "current_resource_context"},
                {"secondary_context_rule",
                 palette ? "palette_lookup_or_primary_context" : "primary_context"},
                {"lookup_request", nullptr}});
    if (palette)
        out["lookup_request"] = {{"reference", codec.to_bytes(leaf)},
                                 {"search_path_setting", "P3d_Material"},
                                 {"success_context", "resolved_palette_resource"},
                                 {"failure_context", "primary_context"}};
    return out;
}
} // namespace
Json native_material_catalog_records(const Json &native_records,
                                    const std::vector<std::size_t> *selected_members) {
    Json out = Json::array();
    const auto count = selected_members ? selected_members->size() : native_records.size();
    for (std::size_t i = 0; i < count; ++i) {
        const auto ni = selected_members ? selected_members->at(i) : i;
        const auto &n = native_records[ni];
        if (!selected_members && n.at("element_type") != 49)
            continue;
        const auto base = bytesof(n.at("data"));
        if (!selected_members && (base.size() < 20 || Reader(base, 16).u32() != 18))
            continue;
        Json strings = Json::array();
        Json name = {{"status", "missing"}, {"entry_index", nullptr}, {"value", nullptr}};
        Json resource = name;
        const auto &links = n.at("links");
        for (std::size_t li = 0; li < links.size(); ++li) {
            const auto &l = links[li];
            if (l.at("app") != 0x56d2)
                continue;
            const auto b = bytesof(l.at("payload"));
            const bool user = (l.at("header").get<unsigned>() & 0x1000) != 0;
            Json item = {{"linkage_index", li},
                         {"linkage_offset", l.at("offset")},
                         {"payload", l.at("payload")},
                         {"key", nullptr},
                         {"unassigned_word", nullptr},
                         {"status", "invalid"},
                         {"text", nullptr},
                         {"role", "unassigned"},
                         {"reader_selection", "not_selected"}};
            Json *field = nullptr;
            if (user && b.size() >= 2) {
                const auto key = Reader(b).u16();
                item["key"] = key;
                if (key == 1 || key == 3) {
                    item["role"] = key == 1 ? "catalog_name" : "resource_reference";
                    field = key == 1 ? &name : &resource;
                    if ((*field)["entry_index"].is_null()) {
                        (*field)["entry_index"] = strings.size();
                        item["reader_selection"] = "first_match";
                    } else {
                        field = nullptr;
                        item["reader_selection"] = "shadowed";
                    }
                }
            }
            try {
                require(user, "native catalog string requires user linkage");
                require(b.size() >= 8, "native catalog string header is truncated");
                Reader r(b, 2);
                item["unassigned_word"] = r.u16();
                const auto count = r.u32();
                item["declared_byte_count"] = count;
                require(count <= r.left(), "native catalog string exceeds linkage payload");
                const auto text = r.take(count);
                item["text"] = native_string(text);
                item["status"] = "decoded";
                // String length excludes the linkage's alignment padding. Preserve
                // the suffix without making it part of the source string.
                item["suffix_hex"] = hex(r.take(r.left()));
            } catch (const std::exception &e) {
                item["decode_error"] = e.what();
            }
            if (field) {
                (*field)["status"] = item["status"];
                (*field)["value"] = item["text"];
            }
            strings.push_back(std::move(item));
        }
        name["reader"] = catalog_string_reader(name, strings);
        resource["reader"] = catalog_string_reader(resource, strings);
        resource["interpretation"] = catalog_resource_interpretation(resource);
        out.push_back({{"stream", n.value("stream", Json())},
                       {"native_record_index", ni},
                       {"record_id", n.at("id")},
                       {"record_offset", n.at("offset")},
                       {"initial_load_filter", native_record_input_filter(n, base)},
                       {"strings", std::move(strings)},
                       {"catalog_name", std::move(name)},
                       {"resource_reference", std::move(resource)},
                       {"resource_lookup_status", "not_performed"}});
    }
    return out;
}
Json native_material_catalog_tables(const Json &native_records, Json &catalog_records) {
    Json tables = Json::array();
    std::map<std::size_t, std::size_t> catalog_indices;
    for (std::size_t ci = 0; ci < catalog_records.size(); ++ci) {
        catalog_indices.emplace(catalog_records[ci].at("native_record_index").get<std::size_t>(),
                                ci);
        catalog_records[ci]["table_memberships"] = Json::array();
    }
    auto compound = [](const Json &record) {
        const auto type = record.at("element_type").get<unsigned>();
        return (type >= 10 && type <= 32) ||
               ((type < 33 || type > 63) && (record.at("element_flags").get<unsigned>() & 0x40));
    };
    auto count_offset = [](const Json &record) -> std::size_t {
        return (record.at("element_flags").get<unsigned>() & 0x20) ? 108 : 36;
    };
    // Count each relevant block once, not its entire prefix for every table.
    // A fresh native block reader starts with an unsigned zero counter.
    std::map<Json, std::uint32_t> stream_counters;
    for (const auto &n : native_records)
        if (n.at("element_type") == 10) {
            const auto base = bytesof(n.at("data"));
            if (base.size() >= 20 && Reader(base, 16).u32() == 18)
                stream_counters.emplace(n.value("stream", Json()), 0);
        }
    for (std::size_t ni = 0; ni < native_records.size(); ++ni) {
        const auto &n = native_records[ni];
        const auto stream_counter = stream_counters.find(n.value("stream", Json()));
        if (stream_counter == stream_counters.end())
            continue;
        const auto base = bytesof(n.at("data"));
        if (Reader(base).u32() == 0)
            ++stream_counter->second;
        if (n.at("element_type") != 10)
            continue;
        if (base.size() < 20 || Reader(base, 16).u32() != 18)
            continue;
        Json table = {{"native_record_index", ni},
                      {"stream", n.value("stream", Json())},
                      {"record_id", n.at("id")},
                      {"record_offset", n.at("offset")},
                      {"initial_load_filter", native_record_input_filter(n, base)},
                      {"initial_load_members",
                       native_catalog_input_members(ni, native_records, catalog_indices,
                                                    stream_counter->second)},
                      {"membership_status", "invalid"},
                      {"runtime_selection", "not_evaluated"},
                      {"member_record_indices", Json::array()},
                      {"unflagged_descendant_record_indices", Json::array()},
                      {"members", Json::array()}};
        try {
            const auto offset = count_offset(n);
            const auto count = Reader(base, offset).u32();
            Json unassigned = Json::array();
            if (offset > 36)
                unassigned.push_back(
                    {{"source_offset", 36}, {"data", rawbytes(slice(base, 36, offset - 36))}});
            if (base.size() > offset + 4)
                unassigned.push_back(
                    {{"source_offset", offset + 4},
                     {"data", rawbytes(slice(base, offset + 4, base.size() - offset - 4))}});
            table.update({{"declared_descendant_count", count},
                          {"descendant_count_source_offset", offset},
                          {"unassigned_ranges", std::move(unassigned)}});
            require(count <= native_records.size() - ni - 1, "material table descendant count");
            const auto end = ni + 1 + count;
            // The count covers all descendants. A nested compound record consumes
            // its own span; its children are not direct members of this table.
            std::vector<std::size_t> parent_ends{end};
            Json members = Json::array(), indices = Json::array(), unflagged = Json::array();
            auto next_offset =
                n.at("offset").get<std::uint64_t>() + n.at("length").get<std::uint64_t>();
            for (std::size_t j = ni + 1; j < end; ++j) {
                const auto &child = native_records[j];
                require(child.value("stream", Json()) == table["stream"] &&
                            child.at("offset") == next_offset,
                        "material table descendants cross stream or record boundary");
                require(Reader(bytesof(child.at("data"))).u32() == 0,
                        "nonzero-prefix record cannot define a counted source member span");
                // The input reader reconstructs the parent chain from counts and
                // sets 0x80 itself. Its absence in stored bytes is not a boundary.
                if (!(child.at("element_flags").get<unsigned>() & 0x80))
                    unflagged.push_back(j);
                while (!parent_ends.empty() && parent_ends.back() == j)
                    parent_ends.pop_back();
                require(!parent_ends.empty(), "material table parent span");
                if (parent_ends.size() == 1) {
                    auto found = catalog_indices.find(j);
                    members.push_back(
                        {{"native_record_index", j},
                         {"catalog_record_index",
                          found == catalog_indices.end() ? Json() : Json(found->second)}});
                    indices.push_back(j);
                }
                if (compound(child)) {
                    const auto data = bytesof(child.at("data"));
                    const auto descendants = Reader(data, count_offset(child)).u32();
                    require(descendants <= parent_ends.back() - j - 1,
                            "nested material table member exceeds parent span");
                    if (descendants)
                        parent_ends.push_back(j + 1 + descendants);
                }
                next_offset += child.at("length").get<std::uint64_t>();
            }
            table["member_record_indices"] = std::move(indices);
            table["members"] = std::move(members);
            table["unflagged_descendant_record_indices"] = std::move(unflagged);
            table["membership_status"] = "resolved";
            // Publish associations only after the entire source span is valid.
            for (std::size_t mi = 0; mi < table["members"].size(); ++mi) {
                const auto &ci = table["members"][mi]["catalog_record_index"];
                if (!ci.is_null())
                    catalog_records[ci.get<std::size_t>()]["table_memberships"].push_back(
                        {{"table_index", tables.size()}, {"member_index", mi}});
            }
        } catch (const std::exception &e) {
            table["membership_error"] = e.what();
        }
        tables.push_back(std::move(table));
    }
    return tables;
}
} // namespace p3d
