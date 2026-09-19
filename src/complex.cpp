#include "blob_internal.hpp"
namespace p3d {
static Json component_text(const Bytes &b, bool ansi = true);
static Json list(Reader &r, std::function<Json()> f, std::size_t min = 1, char fmt = 'Q') {
    auto n = r.count(fmt, min);
    Json out = Json::array();
    for (std::uint64_t i = 0; i < n; ++i)
        out.push_back(f());
    return out;
}
static Json ref(Reader &r) {
    auto v = r.expect("I", 1);
    auto guid = r.string();
    return {
        {"version", v}, {"document_guid", guid}, {"model_id", r.u32()}, {"element_id", r.u64()}};
}
static Json tokens(const Bytes &b) {
    Reader r(b);
    Json ts = Json::array();
    while (r.left()) {
        auto off = r.p;
        auto kind = r.u32();
        Json item = {{"offset", off}, {"type_code", kind}};
        std::map<unsigned, std::pair<std::string, std::string>> fmt = {
            {0, {"B", "unassigned_byte"}},
            {1, {"i", "int32"}},
            {2, {"d", "double"}},
            {3, {"q", "int64"}},
            {6, {"3d", "point3d"}}};
        if (fmt.count(kind)) {
            item["value"] = r.number(fmt[kind].first);
            item["kind"] = fmt[kind].second;
        } else if (kind == 4 || kind == 5) {
            auto data = r.take(r.u32());
            if (kind == 5)
                item.update(
                    {{"kind", "binary"}, {"binary_base64", base64(data)}, {"bytes", data.size()}});
            else {
                std::string text, enc = "utf8";
                try {
                    text = utf8(data);
                } catch (...) {
                    text = gb18030(data);
                    enc = "gb18030";
                }
                item.update({{"kind", "string"}, {"value", text}, {"encoding", enc}});
            }
        } else
            throw std::runtime_error("property token type");
        ts.push_back(item);
    }
    require(ts.size() >= 2 && ts[0]["kind"] == "string" && ts[1]["kind"] == "string",
            "property block namespace/class");
    return {{"namespace", ts[0]["value"]},
            {"class_name", ts[1]["value"]},
            {"tokens", ts},
            {"note", "Ordered typed values, including property names and declared types. Object "
                     "scopes and type-0 byte semantics remain unassigned."}};
}
static Json bfa_driven(const Bytes &payload) {
    Reader r(payload);
    auto property_reference = [&](unsigned kind) {
        const auto offset = r.p + 19; // Relative to the complete BFA node body.
        require(hex(r.take(3)) == "7c2340", "BFA driven property reference marker");
        Json out = {{"body_offset", offset},
                    {"storage_kind", kind},
                    {"reference_scope", "component_project"},
                    {"resolution_status", "requires_component_context"}};
        if (kind == 3) {
            out["kind"] = "object_property";
            out["object_id"] = r.u64();
            out["property_id"] = r.i64();
        } else {
            // The native component maps these object references to property IDs.
            // They must not be interpreted directly as BPPropertyID values.
            out["kind"] = "component_property_reference";
            out["property_reference_id"] = r.u64();
        }
        return out;
    };
    const auto kind = r.u32();
    require(kind >= 1 && kind <= 3, "BFA driven target storage kind");
    Json out = {{"target", property_reference(kind)}, {"inputs", Json::array()}};
    for (unsigned group = 1; group <= 3; ++group) {
        const std::uint8_t end = group == 1 ? '[' : group == 2 ? ']' : '#';
        while (true) {
            require(r.left() != 0, "BFA driven input list terminator");
            if (payload[r.p] == end) {
                r.u8();
                break;
            }
            out["inputs"].push_back(property_reference(group));
        }
    }
    const auto formula_size = r.u32();
    const auto formula_offset = r.p + 19;
    const auto encoded = r.take(formula_size);
    Json formula = {{"body_offset", formula_offset},
                    {"encoded_bytes", rawbytes(encoded)},
                    {"evaluation_status", "not_performed"},
                    {"expression_status", "requires_native_expression_conversion"}};
    try {
        Reader fr(encoded);
        Json parts = Json::array();
        Bytes expanded;
        std::size_t literal = 0;
        auto flush = [&](std::size_t end) {
            if (end == literal)
                return;
            const auto bytes = slice(encoded, literal, end - literal);
            parts.push_back({{"kind", "literal"},
                             {"encoded_offset", literal},
                             {"value", component_text(bytes, false)}});
            expanded.insert(expanded.end(), bytes.begin(), bytes.end());
        };
        while (fr.left()) {
            const auto at = fr.p;
            if (fr.left() >= 3 && encoded[at] == '>' && encoded[at + 1] == '<' &&
                encoded[at + 2] == '@') {
                flush(at);
                fr.take(3);
                const auto id = fr.u64();
                parts.push_back({{"kind", "packed_reference"},
                                 {"encoded_offset", at},
                                 {"reference_id_bits", id}});
                // Expand only explicitly packed references. This view is not
                // the SDK's converted formula string or its evaluation result.
                const auto text = std::string("><@") + std::to_string(id);
                expanded.insert(expanded.end(), text.begin(), text.end());
                literal = fr.p;
            } else
                fr.u8();
        }
        flush(encoded.size());
        formula["parts"] = std::move(parts);
        formula["reference_expanded_form"] = component_text(expanded, false);
        formula["token_status"] = "decoded";
    } catch (const std::exception &e) {
        // Formula corruption cannot consume the independently bounded flags.
        formula["token_status"] = "malformed";
        formula["decode_error"] = e.what();
    }
    auto flag = [&](Json &dst, const char *name) {
        const auto value = r.u8();
        dst[std::string(name) + "_byte"] = value;
        dst[name] = value <= 1 ? Json(value != 0) : Json(nullptr);
        if (value > 1)
            dst[std::string(name) + "_status"] = "invalid_boolean";
    };
    flag(formula, "valid");
    out["formula"] = std::move(formula);
    flag(out, "bidirectional");
    // The native reader only reads this field for version >= 2.0. BfaTree
    // does not carry that context; retain its physical absence, not a default.
    out["driven_type"] = nullptr;
    out["driven_type_code"] = nullptr;
    out["driven_type_status"] = "not_stored";
    if (r.left()) {
        const auto type = r.i32();
        out["driven_type_code"] = type;
        out["driven_type_status"] = type >= 0 && type <= 2 ? "known_value" : "unknown_value";
        if (type >= 0 && type <= 2)
            out["driven_type"] = type == 0 ? "default" : type == 1 ? "geometry" : "location";
    }
    out["consumed_body_bytes"] = r.p + 19;
    if (r.left())
        out["unassigned_suffix_hex"] = hex(r.take(r.left()));
    return out;
}
static Json bfa_property_visibility(const Bytes &unit) {
    // The native property loader assigns a NUL-terminated string. Its display
    // predicate searches that string as bytes, without decoding a code page.
    const auto end = std::find(unit.begin(), unit.end(), std::uint8_t(0));
    const std::string text(unit.begin(), end);
    Json out = {
        {"source_field", "unit"}, {"scope", "stored_property_definition"}, {"status", "evaluated"}};
    if (text.size() > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())) {
        out["status"] = "native_string_index_out_of_range";
        return out;
    }
    const auto marker = text.find("isShow:");
    out["marker_found"] = marker != std::string::npos;
    if (marker == std::string::npos) {
        out["is_show"] = true;
        out["selection_rule"] = "marker_absent_default_true";
        return out;
    }
    out["marker_byte_offset"] = marker;
    const auto hash = text.find('#', marker);
    // Preserve the two distinct native substr branches. With '#', the
    // comparison retains the marker itself; it does not strip "isShow:".
    const auto start = hash == std::string::npos ? marker + 7 : marker;
    const auto length = (hash == std::string::npos ? text.size() : hash) - start;
    out["comparison_byte_offset"] = start;
    out["comparison_byte_length"] = length;
    out["selection_rule"] = hash == std::string::npos ? "suffix_after_colon" : "prefix_before_hash";
    out["comparison"] = "case_sensitive_equal_true";
    out["is_show"] = length == 4 && text.compare(start, length, "true") == 0;
    return out;
}
static Json bfa_property(const Bytes &payload, Json &record) {
    Reader r(payload);
    auto text = [&]() {
        const auto bytes = r.take(r.u32());
        auto out = component_text(bytes, false);
        const auto end = std::find(bytes.begin(), bytes.end(), std::uint8_t(0));
        out["native_value"] = component_text(Bytes(bytes.begin(), end), false);
        return std::make_pair(bytes, out);
    };
    auto flag = [&](Json &dst, const char *name) {
        const auto value = r.u8();
        dst[std::string(name) + "_byte"] = value;
        dst[name] = value <= 1 ? Json(value != 0) : Json(nullptr);
        if (value > 1)
            dst[std::string(name) + "_status"] = "invalid_boolean";
    };
    Json out;
    flag(out, "variable");
    record["unassigned_prefix_byte"] = out["variable_byte"]; // Legacy field.
    Json names = Json::array(), keys = Json::object();
    for (const auto language : {"chinese", "english"}) {
        const auto name = text();
        keys[language] = name.second;
        try {
            names.push_back(gb18030(name.first));
        } catch (const std::exception &e) {
            names.push_back(nullptr);
            record["names_decode_error"] = e.what();
        }
    }
    record["names"] = std::move(names);
    record["names_encoding"] = "legacy_gb18030_display";
    record["suffix_hex"] = hex(slice(payload, r.p, r.left()));
    out["keys"] = std::move(keys);
    const auto type = r.i32();
    out["declared_type_code"] = type;
    static const char *types[] = {"none",        "int64",          "double",
                                  "string",      "bool",           "binary",
                                  "enumeration", "component_type", "subcomponent_type"};
    out["declared_type"] = type >= 0 && type <= 8 ? Json(types[type]) : Json(nullptr);
    auto value = [&](bool map_layout) {
        const auto offset = r.p;
        const auto code = r.u32();
        // The base value and the two derived maps use different wire enums.
        const auto kind = map_layout ? (code == 0   ? 2u
                                        : code == 1 ? 0u
                                        : code == 2 ? 1u
                                                    : code)
                                     : code;
        Json v = {{"body_offset", offset + 19}, {"type_code", code}};
        if (kind == 0) {
            v["kind"] = "double";
            v["value"] = r.f64();
        } else if (kind == 1) {
            v["kind"] = "int64";
            v["value"] = r.i64();
        } else if (kind == 2) {
            v["kind"] = "bool";
            flag(v, "value");
        } else if (kind == 3) {
            v["kind"] = "string";
            v["value"] = text().second;
        } else if (map_layout && kind == 4) {
            v["kind"] = "binary";
            v["value"] = rawbytes(r.take(r.u32()));
        } else if ((!map_layout && kind == 4) || (map_layout && kind == 5)) {
            v["kind"] = "none";
            v["value"] = nullptr;
        } else
            throw std::runtime_error("BFA property value wire type");
        v["source_bytes"] = rawbytes(slice(payload, offset, r.p - offset));
        return v;
    };
    out["base_value"] = value(false);
    flag(out, "readonly");
    const auto unit = text();
    out["unit"] = unit.second;
    out["property_visibility"] = bfa_property_visibility(unit.first);
    out["group"] = text().second;
    out["description"] = text().second;
    require(out["variable_byte"].get<unsigned>() <= 1, "BFA property variable discriminator");
    const bool variable = out["variable"].get<bool>();
    out["combobox"] = {{"status", "not_stored"}};
    // Older bodies omit the combo block. In derived bodies the next explicit
    // delimiter is '{'; no version or missing default is inferred from it.
    if (r.left() && (variable || payload[r.p] != '{')) {
        Json combo;
        flag(combo, "enabled");
        const auto count = r.count('I', 4);
        combo["options"] = Json::array();
        for (std::uint64_t i = 0; i < count; ++i)
            combo["options"].push_back(text().second);
        combo["status"] = "stored";
        out["combobox"] = std::move(combo);
    }
    if (!variable) {
        Json maps = Json::array();
        for (const auto delimiters : {std::make_pair('{', '}'), std::make_pair('}', '-')}) {
            require(r.u8() == delimiters.first, "BFA property value-map opening marker");
            Json entries = Json::array();
            while (true) {
                require(r.left() != 0, "BFA property value-map closing marker");
                if (payload[r.p] == delimiters.second)
                    break;
                const auto offset = r.p + 19;
                require(hex(r.take(3)) == "7c2340", "BFA property map reference marker");
                const auto id = r.u64();
                entries.push_back(
                    {{"body_offset", offset}, {"reference_id", id}, {"value", value(true)}});
            }
            maps.push_back(std::move(entries));
        }
        r.u8(); // '-'
        out["unassigned_value_maps"] = std::move(maps);
        out["value_map_semantics"] = Json::array();
        for (unsigned map_index = 0; map_index < 2; ++map_index) {
            const auto &entries = out["unassigned_value_maps"][map_index];
            // Native map insertion retains the first value for a duplicate
            // key. Keep every source entry but index the loaded map separately.
            std::map<std::uint64_t, std::size_t> first;
            for (std::size_t i = 0; i < entries.size(); ++i)
                first.emplace(entries[i]["reference_id"].get<std::uint64_t>(), i);
            Json selected = Json::array();
            for (const auto &entry : first)
                selected.push_back(entry.second);
            Json semantics = {
                {"kind", map_index == 0 ? "placed_instance_values" : "component_type_values"},
                {"reference_scope", "component_project"},
                {"key_kind", map_index == 0 ? "bfa_placed_handle" : "imported_bfa_type"},
                {"duplicate_key_rule", "first_entry_wins"},
                {"selected_entry_indices", std::move(selected)},
                {"selected_order", "unsigned_reference_id"}};
            if (map_index == 1)
                semantics["missing_key_value"] = "none";
            out["value_map_semantics"].push_back(std::move(semantics));
        }
        const auto width = r.left();
        require(width == 2 || width == 7 || width == 8 || width >= 13,
                "BFA property control-field layout");
        Json controls;
        flag(controls, "inner_property");
        flag(controls, "type_property");
        if (width >= 7) {
            flag(controls, "can_delete");
            flag(controls, "name_editable");
            flag(controls, "value_editable");
            flag(controls, "description_editable");
            flag(controls, "category_inner_property");
            controls["unassigned_byte"] = controls["category_inner_property_byte"]; // Legacy alias.
        }
        if (width >= 8)
            flag(controls, "value_type_editable");
        if (width >= 13) {
            flag(controls, "driven_readonly");
            const auto source = r.i32();
            controls["source_type_code"] = source;
            controls["source_type"] = source == 0   ? Json("default")
                                      : source == 1 ? Json("code")
                                      : source == 2 ? Json("user")
                                                    : Json(nullptr);
        }
        out["controls"] = std::move(controls);
    }
    out["consumed_body_bytes"] = r.p + 19;
    if (r.left())
        out["unassigned_suffix_hex"] = hex(r.take(r.left()));
    out["resolution_status"] = "requires_component_context";
    return out;
}
static std::int32_t bfa_enum_code(std::uint64_t wire) {
    const auto low = std::uint32_t(wire);
    return std::int32_t(low < 0x80000000u ? std::int64_t(low) : std::int64_t(low) - 0x100000000LL);
}
static Json bfa_component_extension(const Bytes &payload, std::size_t start, bool extended) {
    Json candidates = Json::array();
    for (bool points_stored : {false, true}) {
        if (points_stored && !extended)
            continue; // Driven points were introduced after the extra context field.
        try {
            Reader r(payload);
            r.p = start;
            auto text = [&](bool ansi) {
                const auto offset = r.p + 19;
                const auto bytes = r.take(r.u32());
                auto value = component_text(bytes, ansi);
                const auto end = std::find(bytes.begin(), bytes.end(), std::uint8_t(0));
                value["native_value"] = component_text(Bytes(bytes.begin(), end), ansi);
                value["body_offset"] = offset;
                return value;
            };
            Json names = Json::array();
            std::map<std::uint64_t, std::size_t> name_indices;
            while (r.left() && (!points_stored || payload[r.p] != '+')) {
                const auto offset = r.p + 19;
                const auto key = r.u64();
                auto value = text(false);
                name_indices[key] = names.size();
                names.push_back({{"body_offset", offset},
                                 {"offset_key", key},
                                 {"display_name", std::move(value)}});
            }
            if (points_stored)
                require(r.u8() == '+', "BFA display-name map terminator");
            Json selected_names = Json::array();
            for (const auto &entry : name_indices)
                selected_names.push_back(entry.second);
            Json candidate = {{"has_driven_point_section", points_stored},
                              {"offset_to_display_name_map",
                               {{"entries", std::move(names)},
                                {"selected_entry_indices", std::move(selected_names)},
                                {"duplicate_key_rule", "last_entry_wins"}}}};
            if (points_stored) {
                Json points = Json::array();
                while (r.left()) {
                    const auto offset = r.p + 19;
                    auto name = text(true);
                    auto description = text(true);
                    const auto snap_wire = r.u64();
                    const auto snap_code = bfa_enum_code(snap_wire);
                    Json snap_flags = Json::array();
                    static const std::map<std::uint32_t, std::string> snap_names = {
                        {1u, "Nearest"},
                        {1u << 2, "MidPoint"},
                        {1u << 3, "Center"},
                        {1u << 4, "EndPoint"},
                        {1u << 6, "Intersection"},
                        {1u << 7, "Tangency"},
                        {1u << 8, "TangentPoint"},
                        {1u << 9, "Perpendicular"},
                        {1u << 10, "PerpendicularPoint"},
                        {1u << 11, "Parallel"},
                        {1u << 12, "Multi3"},
                        {1u << 14, "Multi1"},
                        {1u << 15, "Multi2"},
                        {1u << 16, "GeometricCenter"},
                        {1u << 17, "Quadrant"},
                        {1u << 18, "Extension"},
                        {1u << 19, "ApparentIntersection"},
                        {1u << 20, "Insertion"},
                        {1u << 21, "Node"},
                        {1u << 22, "enGetBaseCurve"},
                        {1u << 23, "enDivide"}};
                    std::uint32_t known_snap_bits = 0;
                    for (const auto &flag : snap_names) {
                        known_snap_bits |= flag.first;
                        if (snap_code != -1 && (std::uint32_t(snap_code) & flag.first))
                            snap_flags.push_back(flag.second);
                    }
                    Json formulas = Json::array();
                    std::map<std::int32_t, std::size_t> formula_indices;
                    while (r.left() && payload[r.p] != '@') {
                        const auto formula_offset = r.p + 19;
                        const auto wire = r.u64();
                        const auto code = bfa_enum_code(wire);
                        auto formula = text(true);
                        static const std::map<std::int32_t, std::string> coordinates = {
                            {0, "invalid"}, {1, "x"}, {2, "y"}, {3, "z"}, {4, "all"}};
                        formula_indices[code] = formulas.size();
                        formulas.push_back(
                            {{"body_offset", formula_offset},
                             {"coordinate_wire_uint64", wire},
                             {"coordinate_type_code", code},
                             {"coordinate_type",
                              coordinates.count(code) ? Json(coordinates.at(code)) : Json(nullptr)},
                             {"formula", std::move(formula)}});
                    }
                    const bool terminated = r.left() != 0;
                    if (terminated)
                        r.u8();
                    Json selected = Json::array();
                    for (const auto &entry : formula_indices)
                        selected.push_back(entry.second);
                    points.push_back(
                        {{"body_offset", offset},
                         {"name", std::move(name)},
                         {"description", std::move(description)},
                         {"snap_mode_wire_uint64", snap_wire},
                         {"snap_mode_code", snap_code},
                         {"snap_mode_flags", std::move(snap_flags)},
                         {"snap_mode_status", snap_code == -1  ? "invalid"
                                              : snap_code == 0 ? "none"
                                                               : "flags"},
                         {"unknown_snap_mode_bits",
                          snap_code == -1 ? Json(nullptr)
                                          : Json(std::uint32_t(snap_code) & ~known_snap_bits)},
                         {"formulas", std::move(formulas)},
                         {"selected_formula_indices", std::move(selected)},
                         {"duplicate_coordinate_rule", "last_entry_wins"},
                         {"native_is_valid", formula_indices.size() == 3 && snap_code != -1},
                         {"terminator_stored", terminated},
                         {"formula_status", "not_evaluated"}});
                }
                candidate["driven_points"] = std::move(points);
            }
            candidate["consumed_body_bytes"] = r.p + 19;
            candidates.push_back(std::move(candidate));
        } catch (const std::exception &) {
            // A malformed candidate retains its complete raw extension in the caller.
        }
    }
    return {{"layout_status", candidates.empty()       ? "malformed_or_unsupported"
                              : candidates.size() == 1 ? "unique_candidate"
                                                       : "requires_version_context"},
            {"layout_candidates", std::move(candidates)}};
}
static Json bfa_component(const Bytes &payload, Json &record) {
    Reader r(payload);
    Json out, names = Json::array(), texts = Json::array();
    for (unsigned i = 0; i < 2; ++i) {
        const auto bytes = r.take(r.u32());
        auto text = component_text(bytes, false);
        const auto end = std::find(bytes.begin(), bytes.end(), std::uint8_t(0));
        text["native_value"] = component_text(Bytes(bytes.begin(), end), false);
        texts.push_back(std::move(text));
        try {
            names.push_back(gb18030(bytes));
        } catch (const std::exception &e) {
            names.push_back(nullptr);
            record["names_decode_error"] = e.what();
        }
    }
    record["names"] = std::move(names);
    record["names_encoding"] = "legacy_gb18030_display";
    record["suffix_hex"] = hex(Bytes(payload.begin() + r.p, payload.end()));
    out["name_fields"] = std::move(texts);
    out["unassigned_binary_fields"] = Json::array();
    for (unsigned i = 0; i < 2; ++i) {
        const auto offset = r.p + 19;
        const auto bytes = r.take(r.u32());
        out["unassigned_binary_fields"].push_back(
            {{"body_offset", offset}, {"bytes", bytes.size()}, {"base64", base64(bytes)}});
    }
    out["unassigned_int32"] = r.i32();
    out["next_property_id"] = out["unassigned_int32"]; // Compatibility alias retains the old field.
    Json candidates = Json::array();
    // The native reader gets its version from the containing component context.
    // Without it, keep both compatible layouts instead of guessing from ID bits.
    for (bool extended : {false, true}) {
        try {
            Reader c(payload);
            c.p = r.p;
            Json candidate = {{"has_context_uint64", extended},
                              {"property_id_maps", Json::array()}};
            if (extended) {
                const auto tree_id = c.i64();
                candidate["unassigned_context_uint64"] = std::uint64_t(tree_id);
                candidate["related_tree_reference"] = {{"tree_id", tree_id},
                                                       {"kind", "BPTree"},
                                                       {"scope", "active_project"},
                                                       {"resolution_status", "not_performed"}};
            }
            for (unsigned map_index = 0; map_index < 2; ++map_index) {
                Json entries = Json::array();
                std::map<std::uint64_t, std::size_t> selected;
                while (c.left() && payload[c.p] != '+') {
                    const auto offset = c.p + 19;
                    require(hex(c.take(3)) == "7c2340", "BFA property ID map reference");
                    const auto id = c.u64();
                    const auto property_id = c.i64();
                    selected[id] = entries.size();
                    entries.push_back({{"body_offset", offset},
                                       {"property_definition_id", id},
                                       {"property_id", property_id},
                                       {"property_id_bits", std::uint64_t(property_id)}});
                }
                const bool terminated = c.left() != 0;
                require(terminated || (map_index == 1 && !extended),
                        "BFA property ID map terminator");
                if (terminated)
                    c.u8();
                Json indices = Json::array();
                std::map<std::int64_t, std::size_t> inverse;
                for (const auto &entry : selected) {
                    indices.push_back(entry.second);
                    // Native inverse lookup walks the effective unsigned node-key map
                    // to its end, replacing the result for every equal SDK property ID.
                    inverse[entries[entry.second]["property_id"].get<std::int64_t>()] =
                        entry.second;
                }
                Json inverse_indices = Json::array();
                for (const auto &entry : inverse)
                    inverse_indices.push_back(entry.second);
                candidate["property_id_maps"].push_back(
                    {{"driven_storage_kind", map_index + 1},
                     {"entries", std::move(entries)},
                     {"selected_entry_indices", std::move(indices)},
                     {"duplicate_key_rule", "last_entry_wins"},
                     {"inverse_lookup",
                      {{"key", "property_id"},
                       {"selected_entry_indices", std::move(inverse_indices)},
                       {"index_order", "signed_property_id"},
                       {"duplicate_value_rule", "greatest_unsigned_definition_id_wins"},
                       {"missing_key_value", std::uint64_t(0)}}},
                     {"terminator_stored", terminated}});
            }
            candidate["consumed_body_bytes"] = c.p + 19;
            if (c.left()) {
                auto extension = bfa_component_extension(payload, c.p, extended);
                extension["source_bytes"] = rawbytes(Bytes(payload.begin() + c.p, payload.end()));
                if (extension["layout_status"] == "unique_candidate")
                    candidate["consumed_body_bytes"] = payload.size() + 19;
                else
                    candidate["unassigned_suffix_hex"] =
                        hex(Bytes(payload.begin() + c.p, payload.end()));
                candidate["extension"] = std::move(extension);
            }
            candidates.push_back(std::move(candidate));
        } catch (const std::exception &) {
            // A rejected layout never consumes bytes from another candidate or node.
        }
    }
    out["mapping_layout_status"] = candidates.empty()       ? "malformed_or_unsupported"
                                   : candidates.size() == 1 ? "unique_candidate"
                                                            : "requires_version_context";
    out["mapping_layout_candidates"] = std::move(candidates);
    out["reference_scope"] = "component_project";
    return out;
}
static Json bfa(const Bytes &b) {
    static const std::map<std::string, std::string> kinds = {{"~$^", "component_definition"},
                                                             {"@#$", "component_type"},
                                                             {"!_#", "property_definition"},
                                                             {"&@`", "primitive"},
                                                             {"`%!", "driven_object"}};
    Reader r(b);
    auto reference = [&](Reader &r) {
        require(hex(r.take(3)) == "7c2340", "BFA reference");
        return r.u64();
    };
    Json records = Json::array();
    while (r.left()) {
        auto off = r.p;
        auto tag = utf8(r.take(3));
        require(kinds.count(tag) != 0, "BFA type");
        auto oid = reference(r);
        auto body = r.take(r.u64());
        auto children = list(r, [&]() { return Json(reference(r)); }, 11, 'I');
        Reader br(body);
        require(body.size() >= 19 && reference(br) == oid, "BFA body identity");
        Json item = {{"offset", off},
                     {"type_tag", tag},
                     {"node_kind", kinds.at(tag)},
                     {"id", oid},
                     {"children", children},
                     {"unassigned_header_uint64", br.u64()},
                     {"body_base64", base64(body)}};
        if (tag == "`%!") {
            const auto payload = br.take(br.left());
            item["payload_hex"] = hex(payload);
            try {
                item["driven"] = bfa_driven(payload);
                item["payload_status"] = "structure_decoded";
            } catch (const std::exception &e) {
                item["unassigned_suffix_hex"] = hex(payload);
                item["payload_status"] = "malformed_or_unsupported";
                item["decode_error"] = e.what();
            }
        } else if (tag == "!_#") {
            const auto payload = br.take(br.left());
            item["payload_hex"] = hex(payload);
            try {
                item["property_definition"] = bfa_property(payload, item);
                item["payload_status"] = "structure_decoded";
            } catch (const std::exception &e) {
                item["payload_status"] = "malformed_or_unsupported";
                item["decode_error"] = e.what();
            }
        } else if (tag == "~$^") {
            const auto payload = br.take(br.left());
            item["payload_hex"] = hex(payload);
            try {
                item["component_definition"] = bfa_component(payload, item);
                item["payload_status"] = "structure_decoded";
            } catch (const std::exception &e) {
                item["payload_status"] = "malformed_or_unsupported";
                item["decode_error"] = e.what();
            }
        } else if (tag == "&@`") {
            if (br.left())
                try {
                    item["property_block"] = tokens(br.take(br.left()));
                } catch (const std::exception &e) {
                    item["decode_error"] = e.what();
                }
            else
                item["empty_property_block"] = true;
        } else {
            Json names = Json::array();
            for (int i = 0; i < (tag == "@#$" ? 1 : 2); ++i) {
                const auto bytes = br.take(br.u32());
                if (tag == "@#$") {
                    // Keep the old display field, but never require its assumed code page
                    // to parse the native reference list following the bounded name.
                    item["names_encoding"] = "legacy_gb18030_display";
                    try {
                        names.push_back(gb18030(bytes));
                    } catch (const std::exception &e) {
                        names.push_back(nullptr);
                        item["names_decode_error"] = e.what();
                    }
                    auto name = component_text(bytes, false);
                    const auto end = std::find(bytes.begin(), bytes.end(), std::uint8_t(0));
                    name["native_value"] = component_text(Bytes(bytes.begin(), end), false);
                    item["type_name"] = std::move(name);
                } else
                    names.push_back(gb18030(bytes));
            }
            item["names"] = names;
            const auto suffix = br.take(br.left());
            if (tag == "@#$") {
                item["suffix_hex"] = hex(suffix);
                item["placed_instance_reference_scope"] = "component_project";
                item["placed_instance_resolution_status"] = "not_performed";
                try {
                    Reader refs(suffix);
                    require(refs.left() % 11 == 0, "BFA placed-instance reference width");
                    Json ids = Json::array();
                    while (refs.left())
                        ids.push_back(reference(refs));
                    item["placed_instance_ids"] = std::move(ids);
                } catch (const std::exception &e) {
                    item["placed_instance_decode_error"] = e.what();
                    item["unassigned_suffix_hex"] = hex(suffix);
                }
            } else
                item["unassigned_suffix_hex"] = hex(suffix);
        }
        records.push_back(item);
    }
    std::set<std::uint64_t> ids, external;
    for (auto &v : records)
        ids.insert(v["id"].get<std::uint64_t>());
    for (auto &v : records)
        for (auto &c : v["children"])
            if (!ids.count(c))
                external.insert(c.get<std::uint64_t>());
    std::map<std::uint64_t, std::vector<std::size_t>> local_nodes;
    for (std::size_t i = 0; i < records.size(); ++i)
        local_nodes[records[i]["id"].get<std::uint64_t>()].push_back(i);
    // The native loader builds parent pointers from serialized child references;
    // the common body's unassigned u64 is not used as a parent ID.
    std::map<std::uint64_t, Json> parent_sources;
    for (std::size_t i = 0; i < records.size(); ++i)
        for (std::size_t c = 0; c < records[i]["children"].size(); ++c) {
            auto &sources = parent_sources[records[i]["children"][c].get<std::uint64_t>()];
            if (sources.is_null())
                sources = Json::array();
            sources.push_back({{"parent_record_index", i}, {"child_index", c}});
        }
    Json parents = Json::array();
    for (std::size_t i = 0; i < records.size(); ++i) {
        const auto id = records[i]["id"].get<std::uint64_t>();
        const auto source = parent_sources.find(id);
        Json binding = {
            {"record_index", i},
            {"scope", "this_bfa_tree"},
            {"parent_sources", source == parent_sources.end() ? Json::array() : source->second}};
        if (local_nodes.at(id).size() != 1)
            binding["status"] = "ambiguous_node_id";
        else if (source == parent_sources.end())
            binding["status"] = "no_parent_in_this_tree";
        else if (source->second.size() != 1)
            binding["status"] = "ambiguous_parent";
        else {
            const auto parent_index = source->second[0]["parent_record_index"].get<std::size_t>();
            const auto parent_id = records[parent_index]["id"].get<std::uint64_t>();
            if (local_nodes.at(parent_id).size() != 1)
                binding["status"] = "ambiguous_parent_id";
            else if (parent_index == i)
                binding["status"] = "self_parent";
            else {
                binding["status"] = "matched_parent_record";
                binding["parent_record_index"] = parent_index;
                binding["parent_id"] = parent_id;
                binding["parent_node_kind"] = records[parent_index]["node_kind"];
                // Native wrapper construction narrows IDs to signed int32.
                // Keep raw u64 identities here and flag cases needing loader conversion.
                binding["native_identity_status"] =
                    std::uint64_t(bfa_enum_code(id)) == id &&
                            std::uint64_t(bfa_enum_code(parent_id)) == parent_id
                        ? "preserved"
                        : "requires_32bit_conversion";
            }
        }
        parents.push_back(std::move(binding));
    }
    Json bindings = Json::array();
    for (std::size_t i = 0; i < records.size(); ++i) {
        const auto &node = records[i];
        if (!node.contains("property_definition"))
            continue;
        const auto &property = node["property_definition"];
        if (!property.contains("value_map_semantics"))
            continue;
        const auto &entries = property["unassigned_value_maps"][1];
        for (const auto &selected : property["value_map_semantics"][1]["selected_entry_indices"]) {
            const auto entry_index = selected.get<std::size_t>();
            const auto type_id = entries[entry_index]["reference_id"].get<std::uint64_t>();
            Json binding = {{"property_definition_id", node["id"]},
                            {"property_record_index", i},
                            {"type_definition_id", type_id},
                            {"value_source", {{"map_index", 1}, {"entry_index", entry_index}}},
                            {"scope", "this_bfa_tree"}};
            const auto found = local_nodes.find(type_id);
            if (found == local_nodes.end())
                binding["target_status"] = "not_in_this_tree";
            else if (found->second.size() != 1)
                binding["target_status"] = "ambiguous_id";
            else {
                const auto index = found->second.front();
                if (records[index]["node_kind"] != "component_type")
                    binding["target_status"] = "unexpected_node_kind";
                else {
                    binding["target_status"] = "matched_type_record";
                    binding["type_record_index"] = index;
                }
            }
            bindings.push_back(std::move(binding));
        }
    }
    Json property_ids = Json::array();
    for (std::size_t i = 0; i < records.size(); ++i) {
        const auto &node = records[i];
        if (!node.contains("component_definition"))
            continue;
        const auto &definition = node["component_definition"];
        if (definition["mapping_layout_status"] != "unique_candidate")
            continue;
        const auto &maps = definition["mapping_layout_candidates"][0]["property_id_maps"];
        for (std::size_t m = 0; m < maps.size(); ++m)
            for (const auto &selected : maps[m]["selected_entry_indices"]) {
                const auto index = selected.get<std::size_t>();
                const auto &entry = maps[m]["entries"][index];
                const auto id = entry["property_definition_id"].get<std::uint64_t>();
                Json binding = {{"component_record_index", i},
                                {"component_definition_id", node["id"]},
                                {"mapping_source",
                                 {{"layout_index", 0}, {"map_index", m}, {"entry_index", index}}},
                                {"property_definition_id", id},
                                {"property_id", entry["property_id"]},
                                {"scope", "this_bfa_tree"}};
                const auto found = local_nodes.find(id);
                if (found == local_nodes.end())
                    binding["target_status"] = "not_in_this_tree";
                else if (found->second.size() != 1)
                    binding["target_status"] = "ambiguous_id";
                else if (records[found->second.front()]["node_kind"] != "property_definition")
                    binding["target_status"] = "unexpected_node_kind";
                else {
                    binding["target_status"] = "matched_property_record";
                    binding["property_record_index"] = found->second.front();
                    binding["parent_binding_index"] = found->second.front();
                }
                property_ids.push_back(std::move(binding));
            }
    }
    std::map<std::pair<std::uint64_t, unsigned>, std::vector<std::size_t>> property_mapping_sources;
    for (std::size_t i = 0; i < property_ids.size(); ++i) {
        const auto &binding = property_ids[i];
        if (binding["target_status"] == "matched_property_record")
            property_mapping_sources[{binding["property_definition_id"].get<std::uint64_t>(),
                                      binding["mapping_source"]["map_index"].get<unsigned>() + 1}]
                .push_back(i);
    }
    Json driven_bindings = Json::array();
    for (std::size_t i = 0; i < records.size(); ++i) {
        const auto &node = records[i];
        if (!node.contains("driven"))
            continue;
        auto bind = [&](const Json &reference, const char *role, std::size_t input_index) {
            const auto kind = reference["storage_kind"].get<unsigned>();
            const bool property = kind != 3;
            const auto id =
                reference[property ? "property_reference_id" : "object_id"].get<std::uint64_t>();
            Json binding = {{"driven_record_index", i},
                            {"driven_object_id", node["id"]},
                            {"reference_source", {{"role", role}}},
                            {"storage_kind", kind},
                            {"referenced_id", id},
                            {"scope", "this_bfa_tree"},
                            {"runtime_resolution_status", "requires_component_context"}};
            if (std::string(role) == "input")
                binding["reference_source"]["input_index"] = input_index;
            if (!property)
                binding["property_id"] = reference["property_id"];
            const auto found = local_nodes.find(id);
            // Native target conversion ignores a zero property reference/object ID.
            // Input lists, in contrast, retain every serialized entry, including zero.
            if (std::string(role) == "target" && id == 0)
                binding["target_status"] = "inactive_zero_target";
            else if (found == local_nodes.end())
                binding["target_status"] = "not_in_this_tree";
            else if (found->second.size() != 1)
                binding["target_status"] = "ambiguous_id";
            else {
                const auto index = found->second.front();
                if (records[index]["node_kind"] != (property ? "property_definition" : "primitive"))
                    binding["target_status"] = "unexpected_node_kind";
                else {
                    binding["target_status"] =
                        property ? "matched_property_record" : "matched_object_record";
                    binding[property ? "property_record_index" : "object_record_index"] = index;
                    if (property) {
                        const auto sources = property_mapping_sources.find({id, kind});
                        binding["component_mapping_binding_indices"] =
                            sources == property_mapping_sources.end() ? Json::array()
                                                                      : Json(sources->second);
                        binding["parent_binding_index"] = index;
                        binding["parent_component_mapping_binding_indices"] = Json::array();
                        const auto &parent = parents[index];
                        if (parent["status"] == "matched_parent_record" &&
                            parent["parent_node_kind"] == "component_definition" &&
                            parent["native_identity_status"] == "preserved" &&
                            sources != property_mapping_sources.end())
                            for (const auto source_index : sources->second)
                                if (property_ids[source_index]["component_record_index"] ==
                                    parent["parent_record_index"])
                                    binding["parent_component_mapping_binding_indices"].push_back(
                                        source_index);
                        binding["environment_selection_status"] = "not_performed";
                    } else
                        binding["property_resolution_status"] = "requires_object_property_schema";
                }
            }
            driven_bindings.push_back(std::move(binding));
        };
        bind(node["driven"]["target"], "target", 0);
        const auto &inputs = node["driven"]["inputs"];
        for (std::size_t n = 0; n < inputs.size(); ++n)
            bind(inputs[n], "input", n);
    }
    return {{"records", records},
            {"external_child_ids", external},
            {"node_parent_bindings", std::move(parents)},
            {"type_property_bindings", std::move(bindings)},
            {"component_property_bindings", std::move(property_ids)},
            {"driven_reference_bindings", std::move(driven_bindings)},
            {"note", "Node kinds and component-type placed-instance references are identified. "
                     "The placed-instance IDs are not definition-tree child IDs. Other node "
                     "suffixes and some header semantics remain unassigned. Driven references "
                     "require component context; formula conversion and evaluation are separate."}};
}
static Json cached(const Bytes &b) {
    Reader r(b);
    require(b.size() >= 80, "cached geometry length");
    auto version = r.f64();
    auto count = r.u32(), kind = r.u32();
    require(version == 3.1 && count == 1, "cached geometry version/count");
    Json out = {{"version", version},
                {"count", count},
                {"kind_code", kind},
                {"display_header_hex", hex(slice(b, 16, 56))},
                {"display_rgb", r.uints(3)}};
    r.p = 72;
    if (kind == 3) {
        auto header = r.number("3IB");
        auto points = list(r, [&]() { return r.doubles(3); }, 24, 'I');
        require(r.take(24) == Bytes(24), "cached auxiliary channels");
        auto indices = list(r, [&]() { return Json(r.i32()); }, 4, 'I');
        require(r.left() == 68, "cached mesh footer");
        require(r.take(12) == Bytes(12), "cached additional indices");
        Json polygons = Json::array(), face = Json::array();
        for (auto &index : indices) {
            auto i = index.get<std::int64_t>();
            require(std::llabs(i) <= static_cast<long long>(points.size()), "cached point index");
            if (i)
                face.push_back(i);
            else if (!face.empty()) {
                polygons.push_back(face);
                face = Json::array();
            }
        }
        require(face.empty(), "cached polygon termination");
        out.update(
            {{"kind", "indexed_mesh"},
             {"points", points},
             {"indices", indices},
             {"polygons", polygons},
             {"mesh_header_fields", header},
             {"empty_auxiliary_channels", 6},
             {"empty_additional_indices", 3},
             {"footer_hex", hex(r.take(r.left()))},
             {"unassigned_regions", {"display_header_hex", "mesh_header_fields", "footer_hex"}}});
    } else if (kind == 2) {
        auto boundary = r.u32();
        auto curves = list(
            r,
            [&]() {
                auto op = r.u32();
                if (op == 2)
                    return Json{{"kind", "polyline"},
                                {"points", list(r, [&]() { return r.doubles(3); }, 24, 'I')}};
                require(op == 3, "cached curve opcode");
                return Json{{"kind", "elliptical_arc"}, {"origin", r.doubles(3)},
                            {"vector0", r.doubles(3)},  {"vector90", r.doubles(3)},
                            {"start_angle", r.f64()},   {"sweep_angle", r.f64()}};
            },
            4, 'I');
        require(r.left() == 40, "cached curve footer");
        auto footer = hex(slice(b, r.p, r.left()));
        out.update({{"kind", "curve_collection"},
                    {"boundary_code", boundary},
                    {"curves", curves},
                    {"footer_rgb", r.uints(3)},
                    {"footer_hex", footer},
                    {"unassigned_regions", {"display_header_hex", "boundary_code", "footer_hex"}}});
        r.skip(r.left());
    } else
        throw std::runtime_error("cached geometry kind");
    return out;
}
Json decode_inline_material(const Bytes &b) {
    Reader r(b);
    auto string = [&]() {
        auto n = r.u64();
        require(n % 2 == 0, "inline material string width");
        return utf16(r.take(n));
    };
    auto boolean = [&]() {
        auto flag = r.u8();
        require(flag <= 1, "inline material boolean");
        return flag;
    };
    auto marked = [&](const std::string &label, unsigned n) {
        auto flag = boolean();
        return Json{{"kind", label},
                    {"flag", flag},
                    {"enabled", flag != 0},
                    {"value", n == 1 ? Json(r.f64()) : r.doubles(n)}};
    };
    auto valid = boolean();
    auto name = string();
    Json parameters = Json::array({marked("color", 3), marked("transparency", 1)});
    auto flag = boolean();
    auto unit = r.i32(), mode = r.i32();
    auto filename = string();
    auto scale = r.doubles(2), offset = r.doubles(2);
    auto rotation = r.f64();
    auto bump_filename = string();
    auto bump_factor = r.f64();
    for (auto k :
         {"specular_color", "specular_factor", "glow_color", "glow_factor", "ambient_factor",
          "diffuse_factor", "roughness_factor", "reflect_factor", "refract_factor"})
        parameters.push_back(marked(k, std::string(k).find("color") != std::string::npos ? 3 : 1));
    auto footer_start = r.p;
    Json out = {
        {"encoding", "inline_material_v1"},
        {"wire_format", "BPMaterial_unversioned"},
        {"is_valid", valid != 0},
        {"name", name},
        {"parameters", parameters},
        {"has_map", flag != 0},
        {"map_unit", unit},
        {"map_mode", mode},
        {"uv_scale", scale},
        {"uv_offset", offset},
        {"rotation_degrees", rotation},
        {"bump_factor", bump_factor},
        {"texture_references",
         Json::array(
             {{{"role", "pattern"}, {"filename", filename}, {"flag", flag}, {"enabled", flag != 0}},
              {{"role", "bump"}, {"filename", bump_filename}}})},
        {"display_name", name},
        {"display_name_source", "name_fallback"}};
    static const std::map<int, std::string> units = {{0, "relative"}, {3, "absolute"}};
    static const std::map<int, std::string> modes = {{0, "parametric"}, {1, "elevation_drape"},
                                                     {2, "planar"},     {4, "cubic"},
                                                     {5, "spherical"},  {6, "cylindrical"}};
    out["map_unit_status"] = units.count(unit) ? "identified" : "unknown_value";
    out["map_mode_status"] = modes.count(mode) ? "identified" : "unknown_value";
    if (units.count(unit))
        out["map_unit_name"] = units.at(unit);
    if (modes.count(mode))
        out["map_mode_name"] = modes.at(mode);
    if (r.left()) {
        r.expect("I", 0xabcd);
        out["display_name"] = string();
        out["display_name_source"] = "serialized";
    }
    if (r.left() >= 4 && Reader(b, r.p).u32() == 0xabce) {
        auto start = r.p;
        r.u32();
        auto n = r.u64();
        require(n >= 2 && n % 2 == 0, "inline material extension string width");
        auto raw = r.take(n);
        require(raw[raw.size() - 2] == 0 && raw.back() == 0,
                "inline material extension string terminator");
        auto text = utf16(slice(raw, 0, raw.size() - 2));
        Json extension = {{"offset", start}, {"text", text}, {"source_bytes", rawbytes(raw)}};
        if (text.empty())
            extension["json_status"] = "empty";
        else {
            auto value = Json::parse(text, nullptr, false);
            if (value.is_discarded())
                extension["json_status"] = "invalid_json";
            else {
                extension["json_status"] = "parsed";
                extension["value"] = std::move(value);
            }
        }
        out["extended_data"] = std::move(extension);
    }
    // Preserve future extensions beyond the known display-name and JSON blocks.
    if (r.left())
        out["unassigned_suffix_hex"] = hex(r.take(r.left()));
    out["footer_hex"] = hex(slice(b, footer_start, b.size() - footer_start));
    out["note"] = "Source material flags and parameters; texture paths are references. "
                  "The encoding label is a library identifier, not a serialized version byte.";
    return out;
}
static std::uint32_t be32(Reader &r) {
    auto b = r.take(4);
    return (unsigned(b[0]) << 24) | (unsigned(b[1]) << 16) | (unsigned(b[2]) << 8) | b[3];
}
Json boolean_record(const Bytes &b) {
    Reader r(b);
    auto payload = r.take(be32(r));
    Reader p(payload);
    Json operands = Json::array();
    while (p.left())
        operands.push_back(decode_graphics_bytes(p.take(be32(p))));
    std::function<Json(const Bytes &, unsigned)> tree = [&](const Bytes &b, unsigned depth) {
        require(depth <= 80, "boolean tree depth");
        Reader t(b);
        auto header = t.take(be32(t));
        Reader h(header);
        auto op = be32(h), leaf = be32(h);
        require(leaf <= 1 && header.size() == 8 + 4 * leaf, "boolean node header");
        Json node = {{"operation_code", op}, {"is_leaf", bool(leaf)}, {"children", Json::array()}};
        if (leaf) {
            auto ix = be32(h);
            require(ix < operands.size(), "boolean operand index");
            node["operand_index"] = ix;
        }
        while (t.left())
            node["children"].push_back(tree(t.take(be32(t)), depth + 1));
        require(leaf ? node["children"].empty() : node["children"].size() == 2,
                "boolean child arity");
        return node;
    };
    auto expression = tree(r.take(be32(r)), 0);
    r.finish();
    return {{"operands", operands},
            {"expression", expression},
            {"note", "Operand geometry and expression topology retained. Operation codes and "
                     "inactive-transform/display flags need renderer confirmation; no boolean "
                     "re-evaluation is substituted for stored scene geometry."}};
}
static Json feature_setting(Reader &r) {
    auto mapping = [&](std::function<Json()> fn) {
        return list(r, [&]() { return Json{{"key", r.string()}, {"value", fn()}}; }, 8);
    };
    auto strmap = [&]() { return mapping([&]() { return Json(r.string()); }); };
    auto feature = [&]() {
        return Json{{"header", r.expect("II", {2, 1})},
                    {"parent_path", r.string()},
                    {"unassigned_string", r.string()},
                    {"name", r.string()},
                    {"alias", r.string()},
                    {"template_path", r.string()},
                    {"component_paths", strmap()},
                    {"related_feature_paths", strmap()}};
    };
    auto style = [&]() {
        return Json{{"header", r.expect("II", {1, 1})},    {"parent_path", r.string()},
                    {"unassigned_string", r.string()},     {"name", r.string()},
                    {"display_name", r.string()},          {"unassigned_uint64", r.u64()},
                    {"unassigned_uint32", r.number("III")}};
    };
    return {{"version", r.expect("I", 3)},    {"path", r.string()},
            {"feature", feature()},           {"style", style()},
            {"named_styles", mapping(style)}, {"named_features", mapping(feature)}};
}
static Json vector_mesh(Reader &r) {
    auto v = r.get<float>();
    auto nf = r.u32(), nv = r.u32();
    require(v == 1.f && nv <= r.b.size() / 52 && nf <= r.b.size() / 32, "vector mesh header");
    std::set<std::uint32_t> ids;
    Json vertices = Json::array(), faces = Json::array();
    for (unsigned i = 0; i < nv; ++i) {
        auto p = r.doubles(3), u = r.doubles(3);
        auto id = r.u32();
        require(ids.insert(id).second, "vector duplicate ID");
        vertices.push_back({{"id", id}, {"position", p}, {"unassigned_vector", u}});
    }
    for (unsigned i = 0; i < nf; ++i) {
        auto u = r.doubles(3);
        auto word = r.u32();
        Json edges = Json::array(), verts = Json::array();
        while (true) {
            auto id = r.u32();
            if (!id)
                break;
            auto a = r.u32(), b = r.u32();
            require(ids.count(a) && ids.count(b), "vector vertex missing");
            edges.push_back({{"face_id", id}, {"start", a}, {"end", b}});
            verts.push_back(a);
        }
        require(!edges.empty(), "vector empty face");
        for (std::size_t j = 0; j < edges.size(); ++j)
            require(edges[j]["face_id"] == edges[0]["face_id"] &&
                        edges[j]["end"] == edges[(j + 1) % edges.size()]["start"],
                    "vector face identity/closure");
        faces.push_back({{"id", edges[0]["face_id"]},
                         {"unassigned_vector", u},
                         {"unassigned_uint32", word},
                         {"edges", edges},
                         {"vertex_ids", verts}});
    }
    return {{"version", v}, {"vertices", vertices}, {"faces", faces}};
}
// This unversioned suffix follows the complete component-instance list.
// Material and property text use Windows ANSI; remarks have no confirmed code page.
static Json component_text(const Bytes &b, bool ansi) {
    Json out = {{"encoding", ansi ? "windows_ansi" : "not_established"},
                {"source_bytes", rawbytes(b)},
                {"text", nullptr},
                {"status", ansi ? "requires_ansi_decoder" : "requires_text_decoder"}};
    if (std::all_of(b.begin(), b.end(), [](auto c) { return c < 128; })) {
        out["text"] = std::string(b.begin(), b.end());
        out["status"] = "ascii_subset";
    }
    return out;
}
static Json component_footer(Reader &r) {
    const auto start = r.p;
    auto material_bytes = r.take(r.u32());
    Json material = component_text(material_bytes);
    const auto end = std::find(material_bytes.begin(), material_bytes.end(), std::uint8_t(0));
    const Bytes lookup_bytes(material_bytes.begin(), end);
    const auto lookup = component_text(lookup_bytes);
    material["lookup_name"] = lookup["text"];
    material["lookup_name_bytes"] = rawbytes(lookup_bytes);
    material["lookup_status"] = "not_performed";
    material["comparison"] = "case_sensitive_utf16_code_units";
    material["catalog_context"] = "active_project_material_list";
    material["applies_to"] = "all_rebuilt_graphics_entries";
    material["on_lookup_miss"] = "preserve_entry_materials";
    auto remark_bytes = r.take(r.u32());
    auto remark = component_text(remark_bytes, false);
    const auto remark_end = std::find(remark_bytes.begin(), remark_bytes.end(), std::uint8_t(0));
    const Bytes remark_prefix(remark_bytes.begin(), remark_end);
    remark["native_value"] = component_text(remark_prefix, false);
    const auto binary = r.take(r.u32());
    const auto flag = r.u8();
    const auto count = r.count('I', 13);
    Json values = Json::array();
    Json selected_values = Json::array();
    std::set<std::int64_t> selected_ids;
    for (std::uint64_t i = 0; i < count; ++i) {
        const auto offset = r.p;
        const auto property_id = r.i64();
        const auto type = r.u8();
        const auto data = r.take(r.u32());
        if (selected_ids.insert(property_id).second)
            selected_values.push_back(i);
        Reader v(data);
        Json item = {
            {"source_offset", offset},        {"key", static_cast<std::uint64_t>(property_id)},
            {"property_id", property_id},     {"type_code", type},
            {"source_bytes", rawbytes(data)}, {"kind", "unassigned"}};
        try {
            switch (type) {
            case 1:
                item["kind"] = "int64";
                item["value"] = v.i64();
                break;
            case 2:
                item["kind"] = "double";
                item["value"] = v.f64();
                break;
            case 3: {
                item["kind"] = "boolean";
                auto b = v.u8();
                item["boolean_value"] = b;
                item["value"] = b != 0;
                break;
            }
            case 4: {
                item["kind"] = "string";
                item["value"] = component_text(v.take(v.left()));
                const auto end = std::find(data.begin(), data.end(), std::uint8_t(0));
                item["value"]["native_value"] = component_text(Bytes(data.begin(), end));
                break;
            }
            case 5:
                item["kind"] = "binary";
                v.skip(v.left());
                break;
            case 6:
                item["kind"] = "null";
                item["value"] = nullptr;
                break;
            default:
                v.skip(v.left());
                break;
            }
            v.finish();
        } catch (const std::exception &e) {
            item["decode_error"] = e.what();
        }
        // The native getter stops at the first key match, including a null,
        // unknown or malformed value. It does not search for a later valid one.
        item["native_read_action"] = type < 1 || type > 5 ? "leave_destination_unchanged"
                                     : item.contains("decode_error")
                                         ? "not_evaluated_malformed_payload"
                                         : "set_value";
        values.push_back(std::move(item));
    }
    auto ids = [&]() {
        const auto n = r.count('I', 8);
        Json result = Json::array();
        for (std::uint64_t i = 0; i < n; ++i)
            result.push_back(r.u64());
        return result;
    };
    auto first = ids(), second = ids();
    Json out = {{"source_offset", start},
                {"material_name_reference", std::move(material)},
                {"remark", std::move(remark)},
                {"attached_data_block", rawbytes(binary)},
                {"hollow_byte", flag},
                {"hollow", flag <= 1 ? Json(flag != 0) : Json(nullptr)},
                {"hollow_status", flag <= 1 ? "decoded" : "invalid_boolean"},
                {"values", std::move(values)},
                {"stored_value_selection",
                 {{"scope", "component_data_footer"},
                  {"duplicate_key_rule", "first_entry_wins"},
                  {"index_order", "source_order"},
                  {"selected_entry_indices", std::move(selected_values)},
                  {"missing_key_action", "leave_destination_unchanged"},
                  {"fallback_to_type_values", false}}},
                {"unassigned_id_sets", Json::array({std::move(first), std::move(second)})},
                {"value_key_semantics", "component_property_id"},
                {"material_application_status", "not_evaluated"}};
    if (r.left())
        out["unassigned_suffix_hex"] = hex(r.take(r.left()));
    return out;
}
Json complex_blob(const std::string &name, const Bytes &b) {
    Reader r(b);
    Json out;
    if (name == "GraphicsGeom")
        return cached(b);
    if (name == "DataBlock")
        return tokens(b);
    if (name == "BfaTree")
        return bfa(b);
    if (name == "FeatureSetting")
        out = feature_setting(r);
    else if (name == "GraphicsVecExtend")
        out = vector_mesh(r);
    else if (name == "RuleRelation") {
        auto ver = r.expect("I", 2);
        auto first = list(
            r, [&]() { return Json{{"reference", ref(r)}, {"unassigned_uint32", r.u32()}}; }, 24);
        auto second = list(r, [&]() { return ref(r); }, 24);
        auto third = r.expect("Q", 0);
        out = {{"version", ver},
               {"reference_map", first},
               {"references", second},
               {"unassigned_empty_collection_count", third}};
    } else if (name == "NestRelation") {
        auto ver = r.expect("I", 1);
        auto children = list(r, [&]() { return ref(r); }, 24);
        auto parent = ref(r);
        out = {{"version", ver}, {"children", children}, {"parent", parent}};
    } else if (name == "Definition") {
        r.expect("4I", {1, 302, 1, 1});
        auto points = list(
            r,
            [&]() {
                r.expect("I", 1);
                return r.doubles(3);
            },
            28);
        out = {{"version", 1},
               {"kind_code", 302},
               {"nested_versions", {1, 1}},
               {"point_version", 1},
               {"points", points},
               {"note", "Ordered exact source points; kind-code 302 is preserved. No additional "
                        "interpolation rule is inferred."}};
    } else if (name == "ParaCmptInstance") {
        auto reference = [&]() -> Json {
            auto data = r.take(11);
            if (data == Bytes(11))
                return nullptr;
            Reader dr(data);
            require(hex(dr.take(3)) == "7c2340", "component reference");
            return dr.u64();
        };
        auto parent = reference();
        auto instances = list(
            r,
            [&]() {
                auto oid = reference();
                auto body = decode_graphics_bytes(r.take(r.u32()));
                body["object_id"] = oid;
                return body;
            },
            15, 'I');
        const auto footer_start = r.p;
        auto footer = component_footer(r);
        out = {{"parent_id", parent},
               {"type_definition_id", parent},
               {"type_definition_reference",
                {{"kind", "imported_bfa_type"},
                 {"scope", "component_project"},
                 {"resolution_status", parent.is_null() ? "absent" : "not_performed"}}},
               {"instances", instances},
               {"footer", std::move(footer)},
               {"footer_hex", hex(slice(b, footer_start, b.size() - footer_start))},
               {"note", "Reference list and serialized graphics entries retain their source order. "
                        "Cached payloads are not added again to the visible scene."}};
    } else
        throw std::runtime_error("unsupported complex field");
    r.finish();
    return out;
}
} // namespace p3d
