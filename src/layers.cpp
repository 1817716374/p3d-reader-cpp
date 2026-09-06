#include "internal.hpp"
namespace p3d {
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
        float transparency = r.at<float>(84);
        out.update({{"status", "decoded"},
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
                    {"transparency", std::isfinite(transparency) ? Json(transparency) : Json()},
                    {"transparency_source_bits", r.at<std::uint32_t>(84)},
                    {"extended_flags", extra},
                    {"business_code", version >= 4 ? Json((extra >> 3) & 0xffff) : Json()},
                    {"unassigned_extended_bits", extra & ~(version >= 4 ? 0x7fff8u : 0u)},
                    {"unassigned_ranges",
                     Json::array({{{"source_offset", 48}, {"data", rawbytes(slice(b, 48, 20))}},
                                  {{"source_offset", 88}, {"data", rawbytes(slice(b, 88, 4))}},
                                  {{"source_offset", 96},
                                   {"data", rawbytes(slice(b, 96, b.size() - 96))}}})}});
        if (!std::isfinite(transparency))
            out["transparency_error"] = "nonfinite source value";
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
        tables.push_back(std::move(table));
    }
    for (std::size_t i = 0; i < native.size(); ++i)
        if (native[i].contains("layer_definition") && !owned.count(i))
            orphans.push_back(i);
    Json model_references = Json::array();
    auto system = index.value("P3D-SSYS", std::string());
    std::map<std::uint64_t, std::vector<std::size_t>> group_ids;
    for (std::size_t ti = 0; ti < tables.size(); ++ti) {
        const auto &record = native[tables[ti]["native_record_index"].get<std::size_t>()];
        auto path = record["stream"].get<StreamPath>();
        if (!system.empty() && std::find(path.begin(), path.end(), system) != path.end() &&
            record["layer_table"].value("kind", "") == "layer_group")
            group_ids[record["id"].get<std::uint64_t>()].push_back(ti);
    }
    for (auto model = models.begin(); model != models.end(); ++model) {
        if (!model.value().contains("layer_group_references"))
            continue;
        const auto &references = model.value()["layer_group_references"];
        for (std::size_t ri = 0; ri < references.size(); ++ri) {
            const auto &ref = references[ri];
            Json row = {{"model_id", model.key()},
                        {"reference_index", ri},
                        {"table_indices", Json::array()}};
            if (!ref.contains("table_id")) {
                row["status"] = "unsupported_header";
            } else {
                auto id = ref["table_id"].get<std::uint64_t>();
                const auto found = group_ids.find(id);
                auto candidates =
                    id && found != group_ids.end() ? found->second : std::vector<std::size_t>();
                row["table_id"] = id;
                row["table_indices"] = candidates;
                row["status"] = !id                     ? "none"
                                : candidates.empty()    ? "missing"
                                : candidates.size() > 1 ? "ambiguous"
                                : tables[candidates[0]]["membership_status"] == "resolved"
                                    ? "resolved"
                                    : "invalid_table";
            }
            model_references.push_back(std::move(row));
        }
    }
    return {{"tables", std::move(tables)},
            {"unassigned_layer_record_indices", std::move(orphans)},
            {"model_references", std::move(model_references)}};
}
} // namespace p3d
