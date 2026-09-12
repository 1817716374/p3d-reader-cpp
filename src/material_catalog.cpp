#include "internal.hpp"
#include <codecvt>
#include <locale>

namespace p3d {
namespace {
Json catalog_resource_interpretation(const Json &field) {
    Json out = {{"reader_profile", "bimbase_2025_material_catalog_resource"},
                {"scope", "source_text_if_reader_succeeds"},
                {"status", "unavailable_text"},
                {"lookup_status", "not_performed"}};
    if (field.at("status") != "decoded")
        return out;
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> codec;
    const auto source = field.at("value").get<std::string>();
    std::u16string text;
    try {
        text = codec.from_bytes(source);
        if (codec.converted() != source.size())
            return out;
    } catch (const std::range_error &) {
        return out;
    }
    // The native catalog getter has a 512-unit output buffer. Do not
    // invent conversion/truncation behavior for values beyond that buffer.
    if (text.size() >= 512) {
        out["status"] = "unresolved_conversion_limit";
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
    const std::u16string prefix = u"$(_P3DPROJECT)\\";
    bool project = text.size() >= prefix.size();
    for (std::size_t i = 0; project && i < prefix.size(); ++i)
        project = ascii_lower(text[i]) == ascii_lower(prefix[i]);
    if (!project) {
        out["status"] = "unresolved_resource_form";
        return out;
    }
    out["kind"] = "project_resource";
    // The native path splitter also handles schemes and drive syntax. These
    // require separate rules; a project token does not justify guessing them.
    if (text.find(u':') != std::u16string::npos) {
        out["status"] = "unresolved_path_syntax";
        return out;
    }
    const auto separator = text.find_last_of(u"/\\");
    const auto leaf = text.substr(separator + 1);
    const auto dot = leaf.find_last_of(u'.');
    const auto stem = dot == std::u16string::npos ? leaf : leaf.substr(0, dot);
    const auto ext_with_dot = dot == std::u16string::npos ? std::u16string() : leaf.substr(dot);
    // P3DDC's splitter supplies 260-unit buffers for each path component.
    // Out-of-range cases have no portable definite split in this view.
    if (separator + 1 >= 260 || stem.size() >= 260 || ext_with_dot.size() >= 260) {
        out["status"] = "unresolved_path_component_limit";
        return out;
    }
    const auto extension = ext_with_dot.empty() ? std::u16string() : ext_with_dot.substr(1);
    const bool palette = extension.size() == 3 && ascii_lower(extension[0]) == u'p' &&
                         ascii_lower(extension[1]) == u'a' && ascii_lower(extension[2]) == u'l';
    out.update({{"status", "decoded"},
                {"project_member_name", codec.to_bytes(stem)},
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
Json native_material_catalog_records(const Json &native_records) {
    Json out = Json::array();
    for (std::size_t ni = 0; ni < native_records.size(); ++ni) {
        const auto &n = native_records[ni];
        if (n.at("element_type") != 49)
            continue;
        const auto base = bytesof(n.at("data"));
        if (base.size() < 20 || Reader(base, 16).u32() != 18)
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
        resource["interpretation"] = catalog_resource_interpretation(resource);
        out.push_back({{"stream", n.value("stream", Json())},
                       {"native_record_index", ni},
                       {"record_id", n.at("id")},
                       {"record_offset", n.at("offset")},
                       {"strings", std::move(strings)},
                       {"catalog_name", std::move(name)},
                       {"resource_reference", std::move(resource)},
                       {"resource_lookup_status", "not_performed"}});
    }
    return out;
}
} // namespace p3d
