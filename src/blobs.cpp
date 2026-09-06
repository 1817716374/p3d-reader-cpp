#include "blob_internal.hpp"
namespace p3d {
Json text_json(const Bytes &b) {
    auto first = std::find_if(
        b.begin(), b.end(), [](auto c) { return c != ' ' && c != '\t' && c != '\r' && c != '\n'; });
    if (first == b.end() || (*first != '{' && *first != '['))
        return nullptr;
    try {
        return {{"decoded", Json::parse(utf8(b))}, {"encoding", "utf8_json"}};
    } catch (...) {
    }
    try {
        return {{"decoded", Json::parse(gb18030(b))}, {"encoding", "gb18030_json"}};
    } catch (...) {
    }
    Json tree;
    try {
        tree = Json::parse(latin1(b));
    } catch (...) {
        return nullptr;
    }
    Json enc = Json::object(), unresolved = Json::array();
    auto text = [&](const std::string &s, const std::string &path) {
        Bytes bytes;
        bool ascii = true;
        for (std::size_t i = 0; i < s.size();) {
            auto c = (unsigned char)s[i++];
            if (c < 128)
                bytes.push_back(c);
            else {
                ascii = false;
                if (c != 0xc2 && c != 0xc3)
                    return s;
                if (i == s.size())
                    return s;
                auto d = (unsigned char)s[i++];
                if ((d & 192) != 128)
                    return s;
                bytes.push_back(((c & 31) << 6) | (d & 63));
            }
        }
        if (ascii)
            return s;
        try {
            auto v = utf8(bytes);
            enc["utf8"] = enc.value("utf8", 0) + 1;
            return v;
        } catch (...) {
        }
        try {
            auto v = gb18030(bytes);
            enc["gb18030"] = enc.value("gb18030", 0) + 1;
            return v;
        } catch (...) {
        }
        unresolved.push_back({{"path", path}, {"raw_base64", base64(bytes)}});
        return s;
    };
    std::function<Json(const Json &, const std::string &)> visit =
        [&](const Json &v, const std::string &path) -> Json {
        if (v.is_string())
            return text(v.get<std::string>(), path);
        if (v.is_array()) {
            Json a = Json::array();
            for (std::size_t i = 0; i < v.size(); ++i)
                a.push_back(visit(v[i], path + "[" + std::to_string(i) + "]"));
            return a;
        }
        if (v.is_object()) {
            Json o = Json::object();
            for (auto i = v.begin(); i != v.end(); ++i) {
                auto key = text(i.key(), path + ".<key>");
                require(!o.contains(key), "mixed JSON key collision");
                o[key] = visit(i.value(), path + "." + key);
            }
            return o;
        }
        return v;
    };
    try {
        return {{"decoded", visit(tree, "$")},
                {"encoding", "mixed_utf8_gb18030_json"},
                {"string_encodings", enc},
                {"unresolved_strings", unresolved}};
    } catch (...) {
        return nullptr;
    }
}
static Json extension(const Bytes &b) {
    Reader r(b);
    auto n = r.u64();
    require(n <= 10000, "extension count");
    Json out = Json::array();
    for (std::uint64_t i = 0; i < n; ++i) {
        auto key = r.string('I');
        auto typ = r.u32();
        if (typ == 1 || typ == 2) {
            auto storage = r.take(17), trailer = r.take(4);
            Reader s(storage, typ == 1 ? 8 : 0);
            out.push_back({{"key", key},
                           {"value", typ == 1 ? Json(s.i32()) : Json(s.f64())},
                           {"variant_type", typ},
                           {"value_kind", typ == 1 ? "int32" : "float64"},
                           {"variant_storage_hex", hex(storage)},
                           {"trailer_hex", hex(trailer)}});
        } else if (typ == 5) {
            auto header = r.take(17);
            auto size = r.u32();
            Json marker = nullptr;
            if (key == "Dict.Bimbase.Template.Key" && size > r.left()) {
                marker = size;
                size = r.u32();
            }
            auto data = r.take(size);
            Json val = {{"binary_base64", base64(data)}, {"bytes", size}};
            auto decoded = text_json(data);
            if (!decoded.is_null())
                val.update(decoded);
            if (key == "BooleanRecord")
                try {
                    val.update({{"decoded", boolean_record(data)},
                                {"encoding", "boolean_operand_tree_and_bgfb"}});
                } catch (...) {
                }
            out.push_back({{"key", key},
                           {"value", val},
                           {"variant_type", typ},
                           {"opaque_variant_header_hex", hex(header)},
                           {"binary_marker", marker}});
        } else {
            require(typ == 4, "extension variant");
            auto header = r.take(13);
            auto val = r.string('I');
            auto trailer = r.take(4);
            out.push_back({{"key", key},
                           {"value", val},
                           {"variant_type", typ},
                           {"opaque_variant_header_hex", hex(header)},
                           {"trailer_hex", hex(trailer)}});
        }
    }
    r.finish();
    return out;
}
Json decode_binary_field(const std::string &name, const Bytes &b, const std::string &cl,
                         const Json &roots) {
    Json out = {{"binary_base64", base64(b)}, {"bytes", b.size()}};
    auto apply = [&](const std::string &encoding, std::function<Json()> fn) {
        try {
            auto val = fn();
            out["decoded"] = std::move(val);
            out["encoding"] = encoding;
        } catch (const std::exception &) {
        }
    };
    static const std::map<std::string, std::string> apps = {
        {"Parameter", "grouped_design_parameters"}, {"GroupInfoBinary", "named_group_tree"},
        {"ExtendPro", "appearance_extension_json"}, {"ModelIdMap", "model_id_map"},
        {"DataIdMap", "parameter_handle_data_ids"}, {"DataUnit", "postfix_typed_associations"}};
    if (apps.count(name))
        apply(apps.at(name), [&]() { return application_blob(name, b, cl); });
    static const std::set<std::string> enums = {"type",
                                                "ConeStyle",
                                                "BeamType",
                                                "DesignSpeed",
                                                "ClcMode",
                                                "RebarType",
                                                "ConcreteGrade",
                                                "MLevel",
                                                "DLevel",
                                                "YLevel",
                                                "Hlevel",
                                                "LLevel",
                                                "Tlevel",
                                                "DrillingHoleConstructType",
                                                "PileMechanicalType",
                                                "PileConstructionType",
                                                "PileArrangementStyle",
                                                "PumpingWaterType",
                                                "SolidType",
                                                "ProtectionType",
                                                "DeepLevel"};
    if (enums.count(name) && b.size() == 4)
        apply("application_enum_int32", [&]() {
            return Json{{"value", Reader(b).i32()},
                        {"enum_label", nullptr},
                        {"semantic_status", "numeric_value_only"}};
        });
    if (name == "NodeReservedData" && b == Bytes(26))
        apply("observed_zero_reserved_block", [&]() {
            return Json{
                {"bytes", 26}, {"all_zero", true}, {"semantic_status", "reserved_layout_unknown"}};
        });
    if (name == "TemplatedProperty" && hex(b) == "010000000000000000000000")
        apply("empty_templated_property_v1",
              [&]() { return Json{{"version", 1}, {"count", 0}, {"entries", Json::array()}}; });
    if (name == "BinaryData" && cl == "PBStandardSectionProfile") {
        apply("partial_standard_section", [&]() { return application_blob(name, b, cl); });
        if (roots.value("Type", Json()) == 0x40002)
            apply("pilecap_section_cereal", [&]() { return application_blob("Pilecap", b, cl); });
    }
    if (name == "DesignPara" && cl == "StructStandardFloorModel")
        apply("native_standard_floor_design", [&]() { return application_blob(name, b, cl); });
    if (name == "CerealDatas") {
        apply("drawing_settings_cereal", [&]() { return application_blob(name, b, cl); });
        apply("assembly_settings_cereal",
              [&]() { return application_blob("AssemblyCereal", b, cl); });
    }
    if (name == "DictDataValue" && cl == "PBPrDictionary") {
        if (roots.value("DictName", Json()) == "CLIP_DICT_NAME" &&
            roots.value("DictDataName", Json()) == "CLIP_DICTDATA_NAME" && b.size() == 1)
            apply("struct_model_clip_flag", [&]() {
                return Json{{"allow_trim_storage_uint8", b[0]},
                            {"semantic_status", "decoded"},
                            {"unresolved_spans", Json::array()}};
            });
        if (roots.value("DictName", Json()) == "PBBIM_PC_PLANMODULEPROPERTY_DICT_V2" &&
            roots.value("DictDataName", Json()) == "PBBIM_PC_PLANMODULEPROPERTY_DATA" &&
            b == Bytes(8))
            apply("empty_plan_module_property_map", [&]() {
                return Json{{"count", 0},
                            {"entries", Json::array()},
                            {"semantic_status", "decoded_empty_collection"},
                            {"unresolved_spans", Json::array()}};
            });
    }
    static const std::set<std::string> profiles = {"Profile",
                                                   "SectionVector",
                                                   "SectionProfile",
                                                   "FusionBodySection1CurveArray",
                                                   "FusionBodySection2CurveArray",
                                                   "BaseLine"};
    if (profiles.count(name))
        apply("analytic_profile", [&]() { return profile(b); });
    auto txt = text_json(b);
    if (!txt.is_null())
        out.update(txt);
    if (name == "ExtendProperty")
        try {
            out["decoded_entries"] = extension(b);
            out["encoding"] = "extension_variant_map";
        } catch (...) {
        }
    static const std::map<std::string, std::string> complex = {
        {"NestRelation", "nesting_references"},
        {"GraphicsGeom", "cached_geometry_3_1"},
        {"FeatureSetting", "feature_setting_v3"},
        {"GraphicsVecExtend", "directed_edge_polygon_mesh"},
        {"RuleRelation", "rule_reference_collections"},
        {"DataBlock", "typed_property_tokens"},
        {"BfaTree", "bfa_graph_and_property_tokens"},
        {"ParaCmptInstance", "component_instances_and_bgfb"},
        {"Definition", "linear_element_point_sequence"}};
    if (name == "CustomExtensionProperty" || name == "SecondaryDevelopmentProperty")
        apply(name == "CustomExtensionProperty" ? "road_corridor_stations"
                                                : "secondary_application_properties",
              [&]() { return application_blob(name, b, cl); });
    if (complex.count(name))
        apply(complex.at(name), [&]() { return complex_blob(name, b); });
    if (name == "Definition" && (cl == "ComponentElement" || cl == "FeatureStyleDataElement"))
        apply("application_component_definition", [&]() { return application_blob(name, b, cl); });
    if (!b.empty() && b[0] == 0x40)
        apply("mc_nbfx_binary_xml", [&]() { return binary_xml(b); });
    if (!b.empty() && b[0] <= 3) {
        auto w = b[0] == 1 || b[0] == 3 ? 4 : 2;
        if (b.size() >= 1 + 2u * w) {
            Reader r(b, 1 + w);
            auto n = w == 4 ? r.u32() : r.u16();
            if (n == b.size() - 1)
                apply("mysql_compatible_binary_json", [&]() { return binary_json(b); });
        }
    }
    return out;
}
void enrich_tree(Json &node, const std::string &root_class, const Json &root_fields) {
    if (root_class.empty()) {
        Json fields = Json::object();
        if (node.contains("children"))
            for (auto &child : node["children"]) {
                auto name = child["name"].get<std::string>();
                if (name == "Type" || name == "DictName" || name == "DictDataName")
                    fields[name] = child.value("value", Json());
            }
        auto name = node["name"].get<std::string>();
        enrich_tree(node, name, fields);
        return;
    }
    if (node.contains("value") && node["value"].is_object() &&
        node["value"].contains("binary_base64"))
        node["value"] = decode_binary_field(node["name"], unbase64(node["value"]["binary_base64"]),
                                            root_class, root_fields);
    if (node.contains("children"))
        for (auto &child : node["children"])
            enrich_tree(child, root_class, root_fields);
}
} // namespace p3d
