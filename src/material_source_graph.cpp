#include "internal.hpp"
#include "material_source_graph.hpp"
#include "material_json_name.hpp"
#include <charconv>
#include <tuple>

namespace p3d {
namespace {
using SourceKey = std::tuple<std::string, std::uint64_t, std::uint64_t, std::uint64_t>;
std::uint64_t integer(const Json &v) {
    require(v.is_number_integer() && (v.is_number_unsigned() || v.get<std::int64_t>() >= 0),
            "material_source_nonnegative_integer_required");
    return v.get<std::uint64_t>();
}
SourceKey source_key(const Json &v, bool definition) {
    const auto header = integer(v.at(definition ? "attribute_header_offset" : "offset"));
    require(header <= UINT64_MAX - 16, "material_source_attribute_header_offset_overflow");
    if (definition)
        require(integer(v.at("attribute_offset")) == header + 16,
                "material_source_attribute_header_payload_mismatch");
    return {v.at("stream").dump(), integer(v.at("record_offset")), header,
            integer(v.at(definition ? "attribute_ordinal" : "source_ordinal"))};
}
std::string prefix(std::string s) {
    s.resize(std::min(s.size(), s.find('\0')));
    return s;
}
std::u16string wide(const Bytes &b) {
    std::u16string result;
    for (std::size_t i = 0; i < b.size(); i += 2) {
        const auto c = unsigned(b[i]) | (i + 1 < b.size() ? unsigned(b[i + 1]) << 8 : 0);
        if (!c)
            break;
        result.push_back(char16_t(c));
    }
    return result;
}
std::string text(const std::u16string &s) {
    Bytes bytes;
    bytes.reserve(s.size() * 2);
    for (const auto c : s) {
        bytes.push_back(std::uint8_t(c));
        bytes.push_back(std::uint8_t(c >> 8));
    }
    return utf16(bytes);
}
bool c_equal(const std::string &a, const std::string &b) {
    if (a.size() != b.size())
        return false;
    auto lower = [](unsigned char c) { return c >= 'A' && c <= 'Z' ? c + 32 : c; };
    for (std::size_t i = 0; i < a.size(); ++i)
        if (lower(a[i]) != lower(b[i]))
            return false;
    return true;
}
Json source_record(const Json &r) {
    Json out = Json::object();
    for (const auto *k : {"stream", "id", "offset"})
        if (r.contains(k))
            out[k] = r.at(k);
    return out;
}
} // namespace
namespace detail {
Json material_source_graph(const Json &materials, const Json &catalogs, const Json &graphics,
                           const Json &native, const StoredMaterialSourceOptions &options) {
    Json out = {
        {"status", "unresolved"},
        {"scope", "file_local_stored_material_reference_graph"},
        {"definition_index_basis", "document_materials_definitions"},
        {"catalog_context_selection", "not_performed"},
        {"runtime_material_loading", "not_performed"},
        {"final_draw_binding", "not_evaluated"},
        {"geometry_part_mapping", "not_established"},
        {"comparison", options.catalog.case_equal ? "caller_supplied" : "native_crt_c_locale"},
        {"definitions", Json::array()},
        {"catalogs", Json::array()},
        {"attribute_assignments", Json::array()},
        {"native_link_references", Json::array()},
        {"errors", Json::array()}};
    bool complete = true;
    auto error = [&](const char *phase, std::size_t index, const std::exception &e) {
        complete = false;
        out["errors"].push_back({{"phase", phase}, {"source_index", index}, {"reason", e.what()}});
    };
    try {
        const auto &definitions = materials.at("definitions");
        require(definitions.is_array() && catalogs.is_array() && graphics.is_array() &&
                    native.is_array(),
                "complete_material_source_arrays_required");
        std::map<SourceKey, std::vector<std::size_t>> definition_sources;
        for (std::size_t i = 0; i < definitions.size(); ++i) {
            const auto &d = definitions[i];
            Json node = {{"definition_index", i}};
            for (const auto *k :
                 {"id", "scope", "stream", "record_offset", "attribute_index", "attribute_ordinal",
                  "attribute_offset", "attribute_header_offset"})
                if (d.contains(k))
                    node[k] = d.at(k);
            node["source_view_status"] = d.contains("decode_error") ? "not_decoded" : "decoded";
            try {
                definition_sources[source_key(d, true)].push_back(i);
                node["source_identity_status"] = "known";
            } catch (const std::exception &e) {
                node["source_identity_status"] = "unresolved";
                error("definition", i, e);
            }
            out["definitions"].push_back(std::move(node));
        }
        auto source_edge = [&](const Json *a) {
            Json edge = {{"status", "not_evaluated"}, {"definition_indices", Json::array()}};
            if (!a)
                return edge;
            const auto k = source_key(*a, false);
            edge["source_identity"] = {{"stream", a->at("stream")},
                                       {"record_offset", a->at("record_offset")},
                                       {"attribute_header_offset", a->at("offset")},
                                       {"attribute_payload_offset", integer(a->at("offset")) + 16},
                                       {"attribute_ordinal", a->at("source_ordinal")}};
            auto found = definition_sources.find(k);
            if (found == definition_sources.end()) {
                edge["status"] = "not_found";
                edge["reason"] = "selected_material_source_definition_not_indexed";
                complete = false;
            } else {
                edge["definition_indices"] = found->second;
                edge["status"] = found->second.size() == 1 ? "matched" : "ambiguous";
            }
            return edge;
        };
        for (std::size_t ci = 0; ci < catalogs.size(); ++ci) {
            const auto &c = catalogs[ci];
            Json node = {{"catalog_context_index", ci},
                         {"status", c.at("status")},
                         {"entries", Json::array()}};
            for (const auto *k : {"container", "current_context", "unvisited_member_count"})
                if (c.contains(k))
                    node[k] = c.at(k);
            node["registration_inputs"] = Json::array();
            if (c.contains("input"))
                for (const auto &input : c.at("input")) {
                    Json event = Json::object();
                    for (const auto *k :
                         {"member_index", "member", "action", "entry_index", "existing_entry_index",
                          "native_registration_return_code", "reason"})
                        if (input.contains(k))
                            event[k] = input.at(k);
                    node["registration_inputs"].push_back(std::move(event));
                }
            if (c.at("status") != "resolved" && c.at("status") != "absent")
                complete = false;
            std::map<std::size_t, const Json *> reads;
            if (c.contains("initial_source_materials"))
                for (const auto &r : c.at("initial_source_materials").at("entries"))
                    require(reads.emplace(integer(r.at("catalog_entry_index")), &r).second,
                            "duplicate_catalog_read_entry");
            for (std::size_t ei = 0; ei < c.at("entries").size(); ++ei) {
                const auto &entry = c.at("entries")[ei];
                Json item = {{"catalog_entry_index", ei}};
                for (const auto *k : {"id", "name", "resource", "member_index", "member",
                                      "source_resource_reference"})
                    if (entry.contains(k))
                        item[k] = entry.at(k);
                try {
                    item["declared_source"] = source_edge(entry.contains("source_attribute")
                                                              ? &entry.at("source_attribute")
                                                              : nullptr);
                    const auto r = reads.find(ei);
                    item["initial_read_status"] =
                        r == reads.end() ? Json("not_evaluated") : r->second->at("status");
                    const Json *a = r != reads.end() && r->second->contains("source_attribute")
                                        ? &r->second->at("source_attribute")
                                        : nullptr;
                    item["initial_read_source"] = source_edge(a);
                } catch (const std::exception &e) {
                    item["source_error"] = e.what();
                    error("catalog_entry", ei, e);
                }
                node["entries"].push_back(std::move(item));
            }
            out["catalogs"].push_back(std::move(node));
        }
        const auto equal = options.catalog.case_equal ? options.catalog.case_equal : c_equal;
        auto query = [&](const std::optional<std::string> &name, std::optional<std::uint64_t> id) {
            Json result = {{"status", "unresolved"},
                           {"candidates", Json::array()},
                           {"candidate_set_complete", true},
                           {"selection", "no_runtime_winner_selected"}};
            bool known = true;
            if (name)
                result["name"] = prefix(*name);
            else {
                result["id"] = *id;
                result["resource_context_filter"] = "not_performed";
                result["id_index_basis"] = "initial_registered_catalog_id";
            }
            if ((name && prefix(*name).empty()) || (!name && *id == UINT64_MAX)) {
                result["status"] = "not_requested";
                result["reason"] = name ? "native_empty_name" : "native_id_sentinel";
                return result;
            }
            try {
                for (const auto &c : out.at("catalogs")) {
                    if (c.at("status") != "resolved" && c.at("status") != "absent")
                        known = false;
                    for (const auto &e : c.at("entries")) {
                        const bool matches =
                            name ? equal(prefix(e.at("name").get<std::string>()), prefix(*name))
                                 : integer(e.at("id")) == *id;
                        if (matches)
                            result["candidates"].push_back(
                                {{"catalog_context_index", c.at("catalog_context_index")},
                                 {"catalog_entry_index", e.at("catalog_entry_index")}});
                    }
                }
            } catch (const std::exception &e) {
                known = false;
                result["reason"] = e.what();
            }
            result["candidate_set_complete"] = known;
            result["status"] =
                known ? (result["candidates"].empty() ? "no_candidates" : "candidates")
                      : (result["candidates"].empty() ? "unresolved" : "partial_candidates");
            if (!known)
                complete = false;
            return result;
        };
        std::size_t assignment = 0;
        for (std::size_t gi = 0; gi < graphics.size(); ++gi) {
            const auto &g = graphics[gi];
            const auto &attributes = g.at("attributes");
            auto lookup = native_attribute_lookup(attributes);
            for (std::size_t ai = 0; ai < attributes.size(); ++ai) {
                const auto &a = attributes[ai];
                if ((a.at("group") != 2 && a.at("group") != 4) || a.at("key") != 10001)
                    continue;
                Json row = {{"assignment_index", assignment++}, {"graphics_record_index", gi},
                            {"attribute_ordinal", ai},          {"source_record", source_record(g)},
                            {"group", a.at("group")},           {"attribute_index", a.at("index")},
                            {"queries", Json::array()},         {"status", "unresolved"}};
                if (a.contains("offset"))
                    row["attribute_offset"] = a.at("offset");
                try {
                    require(lookup.at("status") == "resolved",
                            "material_source_attribute_selection_unknown");
                    bool selected = false;
                    for (const auto &k : lookup.at("keys"))
                        if (k.at("group") == a.at("group") && k.at("key") == 10001 &&
                            k.at("index") == a.at("index"))
                            selected = k.at("selected_source_ordinal") == ai;
                    row["attribute_lookup_selection"] = selected ? "selected" : "shadowed";
                    {
                        const auto group = integer(a.at("group")), index = integer(a.at("index"));
                        const auto b = bytesof(a.at("payload"));
                        if (group == 2 || index == 0) {
                            const auto w = wide(b);
                            row["query_utf16_code_units"] =
                                std::vector<std::uint16_t>(w.begin(), w.end());
                            row["role"] = group == 2 ? "legacy_part_name" : "advanced_entity_name";
                            if (group == 2)
                                row["part_index"] = index;
                            row["queries"].push_back(query(text(w), {}));
                            row["status"] = "read";
                        } else if (index == 1) {
                            row["role"] = "advanced_part_names";
                            const auto end = std::find(b.begin(), b.end(), std::uint8_t(0));
                            const auto names = Json::parse(b.begin(), end, nullptr, true, true);
                            require(names.is_object() || names.is_null(),
                                    "part_material_json_not_object_or_null");
                            bool names_complete = true;
                            if (names.is_object())
                                for (auto n = names.begin(); n != names.end(); ++n) {
                                    Json q = {{"source_key", n.key()}, {"status", "not_requested"}};
                                    std::int32_t part = 0;
                                    const auto p = std::from_chars(
                                        n.key().data(), n.key().data() + n.key().size(), part);
                                    const bool eligible =
                                        p.ec == std::errc() &&
                                        p.ptr == n.key().data() + n.key().size() &&
                                        std::to_string(part) == n.key() && part >= 0;
                                    if (eligible) {
                                        q["part_index"] = part;
                                        try {
                                            q["query"] = query(material_json_name(n.value()), {});
                                            q["status"] = "read";
                                        } catch (const std::exception &e) {
                                            q["status"] = "unresolved";
                                            q["reason"] = e.what();
                                            complete = false;
                                            names_complete = false;
                                        }
                                    } else
                                        q["reason"] = "not_an_ordinary_rebuilt_part_key";
                                    row["queries"].push_back(std::move(q));
                                }
                            row["status"] = names_complete ? "read" : "partial";
                        } else if (index == 2) {
                            row["role"] = "reverse_part_index";
                            const auto end = std::find(b.begin(), b.end(), std::uint8_t(0));
                            const auto names = Json::parse(b.begin(), end, nullptr, true, true);
                            require(names.is_object() || names.is_null(),
                                    "reverse_material_json_not_object_or_null");
                            bool groups_complete = true;
                            if (names.is_object())
                                for (auto group = names.begin(); group != names.end(); ++group) {
                                    Json mention = {{"source_material_key", group.key()},
                                                    {"direction", "reverse_index_only"},
                                                    {"source_part_keys", Json::array()},
                                                    {"status", "unresolved"}};
                                    if (group.value().is_object()) {
                                        for (auto p = group.value().begin();
                                             p != group.value().end(); ++p)
                                            mention["source_part_keys"].push_back(p.key());
                                        mention["query"] = query(group.key(), {});
                                        mention["status"] = "read";
                                    } else {
                                        mention["reason"] = "reverse_material_group_not_object";
                                        groups_complete = false;
                                        complete = false;
                                    }
                                    row["queries"].push_back(std::move(mention));
                                }
                            row["status"] = groups_complete ? "read" : "partial";
                        } else {
                            row["status"] = "unassigned_index";
                            complete = false;
                        }
                    }
                } catch (const std::exception &e) {
                    row["reason"] = e.what();
                    complete = false;
                }
                out["attribute_assignments"].push_back(std::move(row));
            }
        }
        for (std::size_t ni = 0; ni < native.size(); ++ni) {
            const auto &n = native[ni];
            bool id_seen = false, name_seen = false, id_key_unknown = false;
            for (std::size_t li = 0; li < n.at("links").size(); ++li) {
                const auto &l = n.at("links")[li];
                const auto app = integer(l.at("app")), header = integer(l.at("header"));
                if (!(header & 0x1000) || (app != 0x41 && app != 0x4f5a))
                    continue;
                const auto b = bytesof(l.at("payload"));
                const bool primary_layer = app == 0x41 && b.size() >= 4 &&
                                           Reader(b).u32() == 0x1000c &&
                                           n.contains("layer_definition");
                if (app == 0x41 && b.size() >= 4 && Reader(b).u32() != 0x1000e && !primary_layer)
                    continue;
                Json row = {{"native_record_index", ni},
                            {"linkage_index", li},
                            {"source_record", source_record(n)},
                            {"role", app == 0x41 ? (primary_layer ? "layer_primary_material_id"
                                                                  : "native_material_id_link")
                                                 : "native_material_name_link"},
                            {"status", "unresolved"}};
                if (l.contains("offset"))
                    row["linkage_offset"] = l.at("offset");
                try {
                    if (app == 0x41) {
                        row["entity_getter_selection"] = primary_layer    ? "not_applicable"
                                                         : id_seen        ? "later_match"
                                                         : id_key_unknown ? "earlier_key_unresolved"
                                                                          : "first_match";
                        require(b.size() >= 4, "possible_material_link_key_truncated");
                        row["material_link_key"] = Reader(b).u32();
                        if (!primary_layer)
                            id_seen = true;
                        require(b.size() >= 12, "material_reference_id_truncated");
                        row["query"] = query({}, Reader(b).at<std::uint64_t>(4));
                        row["status"] = "read";
                    } else {
                        row["entity_getter_selection"] = name_seen ? "later_match" : "first_match";
                        name_seen = true;
                        const auto decoded = decode_native_material_name(b, options.ansi_decoder);
                        row["name_read"] = decoded;
                        require(decoded.at("status") == "decoded",
                                "material_reference_name_unknown");
                        row["query"] = query(decoded.at("material_name").get<std::string>(), {});
                        row["status"] = "read";
                    }
                } catch (const std::exception &e) {
                    row["reason"] = e.what();
                    complete = false;
                    if (app == 0x41 && b.size() < 4 && !id_seen)
                        id_key_unknown = true;
                }
                out["native_link_references"].push_back(std::move(row));
            }
        }
        out["status"] = complete ? "resolved" : "partial";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace detail
Json read_stored_material_graph(const Document &doc, const StoredMaterialSourceOptions &options) {
    return detail::material_source_graph(doc.materials(),
                                         doc.native_material_catalog(options.catalog),
                                         doc.graphics_records(), doc.native_records(), options);
}
} // namespace p3d
