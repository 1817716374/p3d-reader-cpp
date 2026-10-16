#include "internal.hpp"
#include "line_style_source.hpp"
#include "layer_sync_properties.hpp"
#include <p3d/layer_lookup.hpp>
namespace p3d {
static Json layer_material_reference(const Json &links) {
    Json out = {{"encoding", "native_layer_material_reference"},
                {"status", "decoded"},
                {"material_id", UINT64_MAX},
                {"source", "constructor_default"},
                {"source_link_index", nullptr},
                {"eligible_for_layer_fallback", false},
                {"material_loading", "not_evaluated"},
                {"entries", Json::array()}};
    bool bounded = true;
    for (std::size_t i = 0; i < links.size(); ++i) {
        const auto &link = links[i];
        if (link.at("app") != 0x41 || !(link.at("header").get<unsigned>() & 0x1000))
            continue;
        Json entry = {{"source_link_index", i}, {"source_offset", link.at("offset")}};
        try {
            const auto bytes = bytesof(link.at("payload"));
            Reader r(bytes);
            const auto key = r.u32();
            if (key != 0x1000d)
                continue;
            entry["key"] = key;
            const auto id = r.u64();
            entry.update({{"status", "decoded"},
                          {"material_id", id},
                          {"payload_byte_offset", 4},
                          {"trailing_storage", rawbytes(r.take(r.left()))}});
            // This reader visits every linkage and assigns on every match.
            // It is distinct from the entity material getter's first match.
            out.update({{"material_id", id},
                        {"source", "saved_linkage"},
                        {"source_link_index", i},
                        {"eligible_for_layer_fallback", id != 0 && id != UINT64_MAX}});
        } catch (const std::exception &e) {
            bounded = false;
            entry.update({{"status", "unresolved"}, {"reason", e.what()}});
        }
        out["entries"].push_back(std::move(entry));
    }
    if (!bounded) {
        // Do not read into the following linkage to emulate an unchecked load.
        out.update({{"status", "unresolved"},
                    {"material_id", nullptr},
                    {"source", "unresolved_linkage"},
                    {"source_link_index", nullptr},
                    {"eligible_for_layer_fallback", nullptr},
                    {"reason", "native_layer_material_read_outside_linkage"}});
    }
    return out;
}
static std::string layer_string(const Bytes &b) {
    // This linkage reader supplies native code page 1200: unmarked bytes are
    // widened individually, and both FF FE and FF FD wide forms are UTF-16LE.
    // Other native string users can supply a different code page.
    if (b.empty() || b[0] == 0)
        return "";
    std::string text;
    if (b.size() >= 2 && b[0] == 255 && (b[1] == 254 || b[1] == 253)) {
        bool narrow = b[1] == 254 && b.size() >= 4 && b[2] == 1 && b[3] == 0;
        auto body = slice(b, narrow ? 4 : 2, b.size() - (narrow ? 4 : 2));
        require(narrow || body.size() % 2 == 0, "layer UTF16 byte length");
        text = narrow ? latin1(body) : utf16(body);
    } else {
        require(b.size() < 2 || b[1] != 255 || (b[0] != 253 && b[0] != 254),
                "unsupported layer string byte order");
        text = latin1(b);
    }
    auto nul = text.find('\0');
    if (nul != std::string::npos)
        text.resize(nul);
    return text;
}
// Offsets below include the four-byte native stream prefix. A layer ID is
// scoped to its containing layer table, not to the whole document.
Json decode_native_layer(const Bytes &b, const Json &links) {
    Json out = {{"encoding", "layer_definition"}, {"view_visibility", "not_evaluated"}};
    Json strings = Json::array();
    for (const auto &link : links) {
        if (link["app"] != 0x56d2 || !(link["header"].get<unsigned>() & 0x1000))
            continue;
        Json text = {{"source_offset", link["offset"]}};
        try {
            auto raw = bytesof(link["payload"]);
            Reader r(raw);
            auto key = r.u16(), flags = r.u16();
            auto length = r.u32();
            text.update({{"key", key}, {"flags", flags}, {"byte_count", length}});
            auto value = layer_string(r.take(length));
            text["text"] = value;
            text["trailing_storage"] = rawbytes(r.take(r.left()));
            if (key == 1 || key == 2) {
                auto role = key == 1 ? "name" : "description";
                text["role"] = role;
                out[role] = value;
            }
        } catch (const std::exception &e) {
            text["decode_error"] = e.what();
        }
        strings.push_back(std::move(text));
    }
    out["strings"] = std::move(strings);
    try {
        Reader r(b, 36);
        auto id = r.u32();
        auto field28 = r.u32();
        auto version = r.u16();
        out.update({{"layer_id", id}, {"field_at_28", field28}, {"version", version}});
        // Version 0 has a different layout. Later versions are not presumed
        // compatible merely because the original reader has a default branch.
        if (version < 1 || version > 7) {
            out["status"] = "unsupported_version";
            return out;
        }
        r.need(62); // common fields through the 64-bit value at offset 100
        auto state = r.u16();
        auto extra = r.at<std::uint32_t>(92);
        unsigned access = (state >> 12) & 3;
        if (state & 0x10)
            access |= 1; // native reader restores this legacy access bit
        auto legacy = r.at<std::uint32_t>(76), extended = r.at<std::uint32_t>(80);
        auto color = legacy;
        if (extended && (extended <= 0xfffffffdu ? (extended & 255) : extended) == legacy)
            color = extended;
        const auto primary_legacy = r.at<std::uint32_t>(60);
        const auto primary_extended = r.at<std::uint32_t>(64);
        auto primary_color = primary_legacy;
        if (primary_extended &&
            (primary_extended <= 0xfffffffdu ? (primary_extended & 255) : primary_extended) ==
                primary_legacy)
            primary_color = primary_extended;
        float transparency = r.at<float>(84);
        // Version 1 predates the saved numeric-key field. Its default layer
        // uses zero; other layers use their ID. Later versions copy the word.
        const auto saved_key = r.at<std::uint32_t>(96);
        const auto numeric_key = version == 1 ? (id == 64 ? 0u : id) : saved_key;
        out.update({{"status", "decoded"},
                    {"numeric_key",
                     {{"value", numeric_key},
                      {"source", version == 1 ? "legacy_layer_id" : "saved_field"},
                      {"source_offset", version == 1 ? 36 : 96},
                      {"saved_value", saved_key}}},
                    {"state_flags", state},
                    {"display", bool(state & 0x20)},
                    {"print", bool(state & 0x40)},
                    {"frozen", bool(state & 0x4000)},
                    {"access_mode", access},
                    {"locked", access == 1},
                    {"unassigned_state_bits", state & ~0x7070u},
                    {"by_layer_symbology",
                     {{"color_index", color},
                      {"legacy_color_index", legacy},
                      {"extended_color_index", extended},
                      {"line_style", r.at<std::int32_t>(68)},
                      {"line_weight", r.at<std::uint32_t>(72)}}},
                    {"primary_symbology",
                     {{"color_index", primary_color},
                      {"legacy_color_index", primary_legacy},
                      {"extended_color_index", primary_extended},
                      {"line_style", r.at<std::int32_t>(52)},
                      {"line_weight", r.at<std::uint32_t>(56)}}},
                    {"transparency", std::isfinite(transparency) ? Json(transparency) : Json()},
                    {"transparency_source_bits", r.at<std::uint32_t>(84)},
                    {"display_priority", r.at<std::int32_t>(88)},
                    {"extended_flags", extra},
                    {"business_code", version >= 4 ? Json((extra >> 3) & 0xffff) : Json()},
                    {"unassigned_extended_bits", extra & ~(version >= 4 ? 0x7fff8u : 0u)},
                    {"unassigned_ranges",
                     Json::array({{{"source_offset", 48}, {"data", rawbytes(slice(b, 48, 4))}},
                                  {{"source_offset", 100},
                                   {"data", rawbytes(slice(b, 100, b.size() - 100))}}})}});
        if (!std::isfinite(transparency))
            out["transparency_error"] = "nonfinite source value";
        out["material_reference"] = layer_material_reference(links);
        out["line_style_parameter_inputs"] = native_layer_line_style_input(links);
    } catch (const std::exception &e) {
        out["decode_error"] = e.what();
    }
    return out;
}
Json decode_native_layer_table(const Bytes &b, const Json &links) {
    Json out = {{"encoding", "layer_table"}, {"inheritance_status", "not_evaluated"}};
    try {
        Reader r(b, 36);
        r.need(32);
        auto count = r.u32();
        auto field = r.u16(), flags = r.u16();
        auto selector = r.u64();
        const char *kind = selector == 0                ? "local"
                           : selector == UINT64_MAX - 3 ? "layer_group"
                           : selector == UINT64_MAX - 1 ? "nested_model_link"
                           : selector == UINT64_MAX || selector == UINT64_MAX - 2 ? "unassigned"
                                                                                  : "model_link";
        out.update({{"declared_child_count", count},
                    {"field_at_28", field},
                    {"table_flags", flags},
                    {"selector", selector},
                    {"kind", kind},
                    {"unassigned_tail", rawbytes(r.take(r.left()))}});
    } catch (const std::exception &e) {
        out["decode_error"] = e.what();
    }
    Json names = Json::array(), paths = Json::array();
    bool first_name = true, first_path = true;
    for (const auto &link : links) {
        if (!(link["header"].get<unsigned>() & 0x1000))
            continue;
        if (link["app"] == 0x56d2) {
            Json item = {{"source_offset", link["offset"]}};
            bool group_name = false;
            try {
                auto b = bytesof(link["payload"]);
                Reader r(b);
                auto key = r.u16(), flags = r.u16();
                group_name = key == 10;
                auto size = r.u32();
                item.update({{"key", key}, {"flags", flags}, {"byte_count", size}});
                item["text"] = layer_string(r.take(size));
                item["trailing_storage"] = rawbytes(r.take(r.left()));
                if (group_name && first_name && out.value("kind", "") == "layer_group")
                    out["name"] = item["text"];
            } catch (const std::exception &e) {
                item["decode_error"] = e.what();
            }
            if (group_name)
                first_name = false; // the native table getter selects occurrence zero
            names.push_back(std::move(item));
        } else if (link["app"] == 0x56f1) {
            Json item = {{"source_offset", link["offset"]}};
            bool reference_path = false;
            try {
                auto b = bytesof(link["payload"]);
                Reader r(b);
                auto field = r.u32();
                auto key = r.u16(), flags = r.u16();
                reference_path = key == 1;
                auto count = r.u32();
                item.update({{"key", key},
                             {"flags", flags},
                             {"field_at_0", field},
                             {"entry_count", count}});
                require(count <= r.left() / 8, "layer table link path count");
                Json ids = Json::array();
                for (std::uint32_t i = 0; i < count; ++i)
                    ids.push_back(r.u64());
                item["entry_ids"] = std::move(ids);
                item["trailing_storage"] = rawbytes(r.take(r.left()));
            } catch (const std::exception &e) {
                item["decode_error"] = e.what();
            }
            if (reference_path && first_path && out.value("kind", "") == "nested_model_link") {
                out["path_linkage_index"] = paths.size();
                out["path_status"] = item.contains("decode_error") ? "decode_error" : "decoded";
            }
            if (reference_path)
                first_path = false;
            paths.push_back(std::move(item));
        }
    }
    if (out.value("kind", "") == "nested_model_link" && !out.contains("path_status"))
        out["path_status"] = "missing";
    out["strings"] = std::move(names);
    out["linkage_paths"] = std::move(paths);
    return out;
}

static Json layer_group_states(const Json &index, const Json &native, const Json &graphics,
                               const Json &tables) {
    Json result = Json::array();
    const auto system = index.value("P3D-SSYS", std::string());
    auto table_scope = [&](const Json &table, const std::string &container) -> Json {
        auto path = table["stream"].get<StreamPath>();
        if (container.empty() || path.size() < 2 || path[path.size() - 2] != container)
            return nullptr;
        path.resize(path.size() - 2);
        return path;
    };
    // IDs identify layers within a file. Names do not establish this pairing.
    std::map<std::string, std::vector<std::size_t>> file_tables;
    for (std::size_t i = 0; i < tables.size(); ++i) {
        const auto &n = native[tables[i]["native_record_index"].get<std::size_t>()];
        auto scope = table_scope(tables[i], system);
        if (!scope.is_null() && n["layer_table"].value("kind", "") == "local")
            file_tables[scope.dump()].push_back(i);
    }
    for (std::size_t gi = 0; gi < tables.size(); ++gi) {
        const auto &table = tables[gi];
        const auto &header = native[table["native_record_index"].get<std::size_t>()]["layer_table"];
        if (header.value("kind", "") != "layer_group")
            continue;
        Json out = {{"group_table_index", gi},
                    {"layers", Json::array()},
                    {"excluded_group_record_indices", Json::array()},
                    {"view_visibility", "not_evaluated"},
                    {"name_synchronization", "not_evaluated"},
                    {"other_properties", "not_evaluated"}};
        try {
            auto scope = table_scope(table, system);
            require(!scope.is_null(), "unrecognized layer group scope");
            auto found = file_tables.find(scope.dump());
            auto candidates =
                found == file_tables.end() ? std::vector<std::size_t>() : found->second;
            out["file_table_indices"] = candidates;
            require(candidates.size() == 1, "missing or ambiguous file layer table");
            const auto &base = tables[candidates[0]];
            require(table["membership_status"] == "resolved" &&
                        base["membership_status"] == "resolved",
                    "invalid group or file table membership");
            require(table["attribute_status"] == "resolved" ||
                        table["attribute_status"] == "missing",
                    "ambiguous group attribute records");
            unsigned mode = 0;
            std::size_t sync_count = 0, override_count = 0;
            std::map<std::uint32_t, std::vector<Json>> overrides;
            for (const auto &record_index : table["attribute_record_indices"]) {
                const auto &attrs = graphics[record_index.get<std::size_t>()]["attributes"];
                for (std::size_t ai = 0; ai < attrs.size(); ++ai) {
                    const auto &a = attrs[ai];
                    if (a["key"] != 4 || a["index"] != 0 || (a["group"] != 0 && a["group"] != 1))
                        continue;
                    const auto &d = a["decoded"];
                    require(!d.contains("decode_error"), "invalid group synchronization attribute");
                    if (a["group"] == 1) {
                        require(++sync_count == 1 &&
                                    d.value("encoding", "") == "layer_group_sync_state",
                                "ambiguous or unsupported group sync mode");
                        mode = d.at("sync_state").get<unsigned>();
                    } else {
                        require(++override_count == 1 &&
                                    d.value("encoding", "") == "layer_group_overrides",
                                "ambiguous or unsupported group overrides");
                        for (std::size_t ei = 0; ei < d.at("entries").size(); ++ei) {
                            const auto &entry = d["entries"][ei];
                            overrides[entry.at("layer_id").get<std::uint32_t>()].push_back(
                                {{"graphics_record_index", record_index},
                                 {"attribute_index", ai},
                                 {"entry_index", ei}});
                        }
                    }
                }
            }
            out["sync_state"] = mode;
            out["sync_state_source"] = sync_count ? "attribute" : "native_default";
            require(mode <= 2, "unsupported group sync mode");
            std::map<std::uint32_t, std::vector<std::size_t>> group_ids, base_ids;
            auto ids = [&](const Json &members, auto &map) {
                for (const auto &member : members) {
                    auto i = member.get<std::size_t>();
                    const auto &layer = native[i]["layer_definition"];
                    require(layer.contains("layer_id"), "missing layer identity");
                    map[layer["layer_id"].get<std::uint32_t>()].push_back(i);
                }
            };
            ids(table["member_record_indices"], group_ids);
            ids(base["member_record_indices"], base_ids);
            // Native synchronization first adds missing file layers and removes
            // group-only layers, even in AlwaysUnsync mode. Keep source records.
            for (const auto &entry : group_ids)
                if (!base_ids.count(entry.first))
                    for (auto i : entry.second)
                        out["excluded_group_record_indices"].push_back(i);
            bool partial = false;
            for (const auto &entry : base_ids) {
                auto id = entry.first;
                const auto group = group_ids.find(id);
                auto group_members =
                    group == group_ids.end() ? std::vector<std::size_t>() : group->second;
                Json layer = {{"layer_id", id},
                              {"file_record_indices", entry.second},
                              {"group_record_indices", group_members},
                              {"properties", Json::object()}};
                try {
                    require(entry.second.size() == 1 && group_members.size() <= 1,
                            "ambiguous layer identity");
                    auto source = entry.second[0];
                    auto target = group_members.empty() ? source : group_members[0];
                    const auto &file_layer = native[source]["layer_definition"];
                    const auto &group_layer = native[target]["layer_definition"];
                    require(file_layer.value("status", "") == "decoded" &&
                                group_layer.value("status", "") == "decoded",
                            "unsupported source layer definition");
                    auto matches = overrides.find(id);
                    const Json *mask = nullptr;
                    if (mode == 0 && !group_members.empty() && matches != overrides.end()) {
                        require(matches->second.size() == 1, "ambiguous layer override entries");
                        const auto &ref = matches->second[0];
                        layer["override_source"] = ref;
                        mask =
                            &graphics[ref["graphics_record_index"].get<std::size_t>()]["attributes"]
                                     [ref["attribute_index"].get<std::size_t>()]["decoded"]
                                     ["entries"][ref["entry_index"].get<std::size_t>()]
                                     ["set_property_bits"];
                        require(mask->is_array(), "invalid layer override bitmap");
                    }
                    for (const auto &property : detail::layer_sync_properties) {
                        const bool override = mask && std::find(mask->begin(), mask->end(),
                                                                property.bit) != mask->end();
                        const bool from_file =
                            group_members.empty() || mode == 2 || (mode == 0 && !override);
                        const auto &value = from_file ? file_layer : group_layer;
                        const auto &values =
                            property.source_container ? value.at(property.source_container) : value;
                        layer["properties"][property.result_name] = {
                            {"property_bit", property.bit},
                            {"value", values.at(property.source_member)},
                            {"native_record_index", from_file ? source : target},
                            {"selection", group_members.empty() ? "new_file_layer"
                                          : from_file           ? "file_sync"
                                                                : "group_value"}};
                        if (property.bit == 35 && value.contains("transparency_error")) {
                            layer["properties"][property.result_name]["error"] =
                                value["transparency_error"];
                            partial = true;
                        }
                    }
                    layer["status"] = "evaluated_supported_properties";
                } catch (const std::exception &e) {
                    layer["status"] = "not_evaluated";
                    layer["error"] = e.what();
                    layer["properties"] = Json::object();
                    partial = true;
                }
                out["layers"].push_back(std::move(layer));
            }
            out["status"] = partial ? "partially_evaluated" : "evaluated_supported_properties";
        } catch (const std::exception &e) {
            out["status"] = "not_evaluated";
            out["error"] = e.what();
        }
        result.push_back(std::move(out));
    }
    return result;
}

Json build_layer_tables(const Json &index, const Json &native, const Json &graphics,
                        const Json &models) {
    Json tables = Json::array(), orphans = Json::array();
    std::set<std::size_t> owned;
    std::map<std::string, std::vector<std::size_t>> attributes;
    auto scope = [&](const Json &record, bool is_native) -> Json {
        auto path = record.at("stream").get<StreamPath>();
        if (path.empty())
            return nullptr;
        path.pop_back();
        for (auto pair : {std::pair<const char *, const char *>{"P3D-SSYS", "P3D-SSYSA"},
                          {"P3D-SMG", "P3D-SMGA"},
                          {"P3D-SMC", "P3D-SMCA"}}) {
            auto from = index.value(is_native ? pair.first : pair.second, std::string());
            auto to = index.value(pair.second, std::string());
            if (from.empty() || to.empty())
                continue;
            auto at = std::find(path.begin(), path.end(), from);
            if (at == path.end())
                continue;
            *at = to;
            return path;
        }
        return nullptr;
    };
    auto key = [](const Json &scope, const Json &id) { return scope.dump() + ":" + id.dump(); };
    for (std::size_t i = 0; i < graphics.size(); ++i) {
        auto s = scope(graphics[i], false);
        if (!s.is_null())
            attributes[key(s, graphics[i]["id"])].push_back(i);
    }
    for (std::size_t i = 0; i < native.size(); ++i) {
        const auto &record = native[i];
        if (!record.contains("layer_table"))
            continue;
        const auto &header = record["layer_table"];
        Json table = {{"native_record_index", i},
                      {"id", record["id"]},
                      {"stream", record["stream"]},
                      {"offset", record["offset"]},
                      {"member_record_indices", Json::array()},
                      {"inheritance_status", "not_evaluated"}};
        std::map<std::uint32_t, std::vector<std::size_t>> member_ids;
        Json members = Json::array();
        try {
            require(!header.contains("decode_error"), "invalid layer table header");
            auto count = header.at("declared_child_count").get<std::uint32_t>();
            require(count <= native.size() - i - 1, "layer table child count");
            auto next_offset =
                record["offset"].get<std::uint64_t>() + record["length"].get<std::uint64_t>();
            for (std::size_t j = i + 1; j <= i + count; ++j) {
                const auto &child = native[j];
                require(child["stream"] == record["stream"] && child["offset"] == next_offset,
                        "layer table children cross stream or record boundary");
                require(child.contains("layer_definition") &&
                            (child["element_flags"].get<unsigned>() & 0x80),
                        "unexpected layer table child");
                members.push_back(j);
                next_offset += child["length"].get<std::uint64_t>();
            }
            for (const auto &j : members) {
                auto k = j.get<std::size_t>();
                owned.insert(k);
                const auto &layer = native[k]["layer_definition"];
                if (layer.contains("layer_id"))
                    member_ids[layer["layer_id"].get<std::uint32_t>()].push_back(k);
            }
            table["member_record_indices"] = std::move(members);
            table["membership_status"] = "resolved";
        } catch (const std::exception &e) {
            table["membership_status"] = "invalid";
            table["membership_error"] = e.what();
        }
        auto s = scope(record, true);
        const auto found = attributes.find(key(s, record["id"]));
        std::vector<std::size_t> candidates;
        if (!s.is_null() && found != attributes.end())
            candidates = found->second;
        table["attribute_record_indices"] = candidates;
        table["attribute_status"] = s.is_null()              ? "unrecognized_scope"
                                    : candidates.empty()     ? "missing"
                                    : candidates.size() == 1 ? "resolved"
                                                             : "ambiguous";
        Json bindings = Json::array(), sync = Json::array();
        for (auto gi : candidates) {
            const auto &attrs = graphics[gi]["attributes"];
            for (std::size_t ai = 0; ai < attrs.size(); ++ai) {
                const auto &a = attrs[ai];
                if (a["key"] != 4 || a["index"] != 0)
                    continue;
                if (a["group"] == 1) {
                    sync.push_back({{"graphics_record_index", gi}, {"attribute_index", ai}});
                    continue;
                }
                if (a["group"] != 0)
                    continue;
                const auto &decoded = a["decoded"];
                if (decoded.value("encoding", "") != "layer_group_overrides" ||
                    !decoded.contains("entries"))
                    continue;
                for (std::size_t ei = 0; ei < decoded["entries"].size(); ++ei) {
                    auto id = decoded["entries"][ei]["layer_id"].get<std::uint32_t>();
                    const auto it = member_ids.find(id);
                    auto rows = it == member_ids.end() ? std::vector<std::size_t>() : it->second;
                    bindings.push_back(
                        {{"graphics_record_index", gi},
                         {"attribute_index", ai},
                         {"entry_index", ei},
                         {"layer_id", id},
                         {"member_record_indices", rows},
                         {"status", table["membership_status"] != "resolved" ? "invalid_table"
                                    : candidates.size() != 1 ? "ambiguous_attributes"
                                    : rows.empty()           ? "missing_layer"
                                    : rows.size() == 1       ? "resolved"
                                                             : "ambiguous_layer"}});
                }
            }
        }
        table["override_bindings"] = std::move(bindings);
        table["sync_attributes"] = std::move(sync);
        // Preserve every source member above; this separate initial cache
        // index follows native loading and does not replace saved membership.
        if (header.value("kind", "") == "local")
            table["initial_numeric_index"] = NativeLayerNumericIndex(native, i).metadata();
        tables.push_back(std::move(table));
    }
    for (std::size_t i = 0; i < native.size(); ++i)
        if (native[i].contains("layer_definition") && !owned.count(i))
            orphans.push_back(i);
    Json model_references = Json::array();
    auto system = index.value("P3D-SSYS", std::string());
    const auto model_container = index.value("P3D-SM", std::string());
    const auto model_header = index.value("P3D~MMH", std::string());
    const auto system_container = index.value("P3DSysD", std::string());
    auto file_scope = [&](StreamPath path, bool model) -> Json {
        if (model) {
            if (model_container.empty() || model_header.empty() || system_container.empty() ||
                system.empty() || path.size() < 3 || path[path.size() - 3] != model_container ||
                path.back() != model_header)
                return nullptr;
            path.resize(path.size() - 3);
        } else {
            if (system.empty() || system_container.empty() || path.size() < 3 ||
                path[path.size() - 2] != system || path[path.size() - 3] != system_container)
                return nullptr;
            path.resize(path.size() - 3);
        }
        return path;
    };
    std::map<std::string, std::vector<std::size_t>> group_ids, local_tables;
    for (std::size_t ti = 0; ti < tables.size(); ++ti) {
        const auto &record = native[tables[ti]["native_record_index"].get<std::size_t>()];
        const auto scope = file_scope(record.at("stream").get<StreamPath>(), false);
        if (scope.is_null())
            continue;
        const auto kind = record["layer_table"].value("kind", "");
        if (kind == "layer_group")
            group_ids[key(scope, record.at("id"))].push_back(ti);
        else if (kind == "local")
            local_tables[scope.dump()].push_back(ti);
    }
    // A missing saved group can use the native null-result fallback only after
    // the complete owner's source keys are known. Do not use a partial or
    // annotation-only catalog as evidence that the registry has no such key.
    std::map<std::string, Json> group_inventories;
    auto group_inventory = [&](const Json &wanted_scope) -> const Json & {
        const auto scope_key = wanted_scope.dump();
        auto found = group_inventories.find(scope_key);
        if (found != group_inventories.end())
            return found->second;
        Json inventory = {{"status", "decoded"}, {"groups", Json::array()}};
        try {
            for (std::size_t ti = 0; ti < tables.size(); ++ti) {
                const auto &table = tables.at(ti);
                const auto &record = native.at(table.at("native_record_index").get<std::size_t>());
                const auto scope = file_scope(record.at("stream").get<StreamPath>(), false);
                require(!scope.is_null(), "group_registry_table_scope_unknown");
                if (scope != wanted_scope)
                    continue;
                const auto body = bytesof(record.at("data"));
                Reader raw(body);
                require(raw.at<std::uint16_t>(4) == 10 && raw.at<std::uint32_t>(16) == 1,
                        "group_registry_source_type_mismatch");
                require(raw.at<std::uint32_t>(8) >= raw.at<std::uint32_t>(12) &&
                            body.size() == 4 + std::uint64_t(raw.at<std::uint32_t>(12)) * 2 &&
                            record.at("length").get<std::uint64_t>() ==
                                4 + std::uint64_t(raw.at<std::uint32_t>(8)) * 2,
                        "group_registry_source_extent_unknown");
                const auto header = decode_native_layer_table(body, record.at("links"));
                require(!header.contains("decode_error") &&
                            raw.at<std::uint64_t>(20) == record.at("id").get<std::uint64_t>(),
                        "group_registry_source_header_unknown");
                require(table.at("membership_status") == "resolved" &&
                            table.at("member_record_indices").size() ==
                                header.at("declared_child_count").get<std::uint32_t>(),
                        "group_registry_source_membership_unknown");
                for (const auto &member : table.at("member_record_indices")) {
                    const auto &row = native.at(member.get<std::size_t>());
                    const auto raw_layer = bytesof(row.at("data"));
                    require(Reader(raw_layer).at<std::uint16_t>(4) == 49 &&
                                Reader(raw_layer).at<std::uint32_t>(16) == 1,
                            "group_registry_source_layer_type_unknown");
                    const auto layer = decode_native_layer(raw_layer, row.at("links"));
                    require(layer.at("status") == "decoded" && !layer.contains("decode_error"),
                            "group_registry_source_layer_unknown");
                }
                if (header.at("selector") == UINT64_MAX - 3)
                    inventory["groups"].push_back(
                        {{"table_index", ti}, {"id", raw.at<std::uint64_t>(20)}});
            }
        } catch (const std::exception &e) {
            inventory["status"] = "unresolved";
            inventory["reason"] = e.what();
        }
        return group_inventories.emplace(scope_key, std::move(inventory)).first->second;
    };
    for (auto model = models.begin(); model != models.end(); ++model) {
        if (!model.value().contains("layer_group_references"))
            continue;
        const auto &references = model.value()["layer_group_references"];
        for (std::size_t ri = 0; ri < references.size(); ++ri) {
            const auto &ref = references[ri];
            Json row = {{"model_id", model.key()},
                        {"reference_index", ri},
                        {"table_indices", Json::array()}};
            const auto scope = ref.contains("stream")
                                   ? file_scope(ref.at("stream").get<StreamPath>(), true)
                                   : Json();
            row["file_scope"] = scope;
            row["scope_status"] = scope.is_null() ? "unresolved" : "resolved";
            if (!ref.contains("table_id")) {
                row["status"] = "unsupported_header";
            } else {
                auto id = ref["table_id"].get<std::uint64_t>();
                const auto found = group_ids.find(key(scope, id));
                auto candidates = id && !scope.is_null() && found != group_ids.end()
                                      ? found->second
                                      : std::vector<std::size_t>();
                row["table_id"] = id;
                row["table_indices"] = candidates;
                row["status"] = !id                     ? "none"
                                : scope.is_null()       ? "unresolved_scope"
                                : candidates.empty()    ? "missing"
                                : candidates.size() > 1 ? "ambiguous"
                                : tables[candidates[0]]["membership_status"] == "resolved"
                                    ? "resolved"
                                    : "invalid_table";
            }
            Json selection = {{"status", "unresolved"},
                              {"source_profile", "fresh_physical_model_without_layer_group"},
                              {"local_table_indices", Json::array()},
                              {"runtime_changes", "not_evaluated"},
                              {"material_loading", "not_evaluated"}};
            if (scope.is_null())
                selection["reason"] = "model_file_scope_unknown";
            else if (!ref.contains("table_id"))
                selection["reason"] = "model_layer_group_reference_unknown";
            else {
                const auto local = local_tables.find(scope.dump());
                const auto candidates =
                    local == local_tables.end() ? std::vector<std::size_t>() : local->second;
                selection["local_table_indices"] = candidates;
                bool use_local = ref.at("table_id") == 0;
                if (!use_local) {
                    selection["source_profile"] = "fresh_physical_model_group_lookup";
                    selection["reason"] = "referenced_group_cache_not_evaluated";
                    if (row.at("status") == "missing") {
                        const auto &inventory = group_inventory(scope);
                        Json lookup = {{"status", "unresolved"},
                                       {"source_profile", "fresh_saved_system_group_registry"},
                                       {"runtime_changes", "not_evaluated"}};
                        if (inventory.at("status") == "decoded") {
                            Json matches = Json::array();
                            for (const auto &entry : inventory.at("groups"))
                                if (entry.at("id") == ref.at("table_id"))
                                    matches.push_back(entry.at("table_index"));
                            lookup["source_group_count"] = inventory.at("groups").size();
                            lookup["matching_table_indices"] = matches;
                            lookup["status"] = matches.empty() ? "not_found" : "source_present";
                            use_local = matches.empty();
                        } else
                            lookup["reason"] = inventory.at("reason");
                        selection["group_lookup"] = std::move(lookup);
                    }
                }
                if (use_local && candidates.size() != 1)
                    selection["reason"] = "local_layer_table_missing_or_ambiguous";
                else if (use_local && tables[candidates[0]]["membership_status"] != "resolved")
                    selection["reason"] = "local_layer_table_membership_invalid";
                else if (use_local) {
                    selection.erase("reason");
                    selection.update(
                        {{"status", "resolved"},
                         {"table_index", candidates[0]},
                         {"native_record_index", tables[candidates[0]]["native_record_index"]},
                         {"selection", ref.at("table_id") == 0
                                           ? "file_local_table"
                                           : "missing_group_file_local_fallback"}});
                }
            }
            row["initial_layer_cache"] = std::move(selection);
            model_references.push_back(std::move(row));
        }
    }
    auto group_states = layer_group_states(index, native, graphics, tables);
    return {{"tables", std::move(tables)},
            {"group_states", std::move(group_states)},
            {"unassigned_layer_record_indices", std::move(orphans)},
            {"model_references", std::move(model_references)}};
}
} // namespace p3d
