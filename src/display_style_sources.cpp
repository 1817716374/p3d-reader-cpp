#include "internal.hpp"
#include "display_style_xml.hpp"

namespace p3d {
namespace {
Json identity(const Json &r, std::size_t record, std::size_t attribute) {
    const auto &a = r.at("attributes").at(attribute);
    return {{"graphics_record_index", record}, {"stream", r.at("stream")},
            {"record_id", r.at("id")}, {"record_offset", r.at("offset")},
            {"attribute_ordinal", attribute}, {"attribute_offset", a.at("offset")},
            {"attribute_index", a.at("index")}};
}

Json lite_copy_projection(const Json &tree) {
    Json out = {{"status", "unresolved"},
                {"condition", "imported_common_style_in_selected_table_with_empty_lite_list"},
                {"scope", "confirmed_native_copy_writes"},
                {"runtime_application", "not_evaluated"}};
    try {
        const auto tag = tree.at("tag").get<std::string>();
        require(tag == "ShowStyle" || tag == "DisplayStyle", "unsupported style XML root");
        auto derived = tree;
        Json *flags = nullptr, *overrides = nullptr;
        for (auto &child : derived.at("children")) {
            const auto name = child.at("tag").get<std::string>();
            if (name == "Flags") {
                require(!flags, "ambiguous Flags nodes");
                flags = &child.at("attributes");
            } else if (name == "Overrides") {
                require(!overrides, "ambiguous Overrides nodes");
                overrides = &child.at("attributes");
            }
        }
        require(flags && overrides && flags->is_object() && overrides->is_object(),
                "style copy projection requires Flags and Overrides objects");
        // P3DKJ 0xdbb2e..0xdbb46: flags48=(flags48 & ~0x79)|6,
        // flags50 &= ~0x47, zero DWORDs54/58 and DOUBLE60. Material's
        // enable bit is cleared, but its stored 64-bit identifier is retained.
        (*overrides)["DisplayMode"] = "6";
        for (const auto *name : {"DisplayVisibleEdges", "VisibleEdgeColor", "VisibleEdgeWeight",
                                 "Transparency", "Material"})
            (*flags)[name] = "false";
        for (const auto *name : {"VisibleEdgeColor", "VisibleEdgeWeight", "Transparency"})
            (*overrides)[name] = "0";
        out["status"] = "conditional";
        out["xml_tree"] = std::move(derived);
        out["unmodeled_steps"] = Json::array({"source_xml_runtime_import",
                                               "table_registration_and_selection",
                                               "resource_remapping"});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace

Json build_display_style_sources(const Json &index, const Json &native, const Json &graphics) {
    Json out = {{"status", "partial"}, {"scope", "file_display_style_source_candidates"},
                {"runtime_resolution", "not_evaluated"},
                {"common_to_lite_conversion", "not_evaluated"},
                {"tables", Json::array()}, {"references", Json::array()}};
    const auto system = index.value("P3D-SSYS", std::string());
    const auto attributes = index.value("P3D-SSYSA", std::string());
    for (std::size_t gi = 0; gi < graphics.size(); ++gi) {
        const auto &g = graphics[gi];
        const auto &attrs = g.at("attributes");
        for (std::size_t ai = 0; ai < attrs.size(); ++ai) {
            const auto &a = attrs[ai];
            const auto &d = a.at("decoded");
            if (d.value("encoding", "") != "native_handler_reference")
                continue;
            const auto registration = d.at("registration_key").get<std::uint32_t>();
            if (registration != 0x006f0000u && registration != 0x597e0000u)
                continue;
            Json table = {{"source", identity(g, gi, ai)}, {"registration_key", registration},
                          {"kind", registration == 0x597e0000u ? "lite" : "common"},
                          {"native_record_candidates", Json::array()},
                          {"header_status", "missing"}, {"entries", Json::array()}};
            auto path = g.at("stream").get<StreamPath>();
            // Only the file system table stream establishes this namespace.
            // Identical IDs in model streams must not participate in the join.
            const bool system_scope = !system.empty() && !attributes.empty() &&
                                      path.size() == 3 && path[1] == attributes;
            table["file_system_scope"] = system_scope;
            if (system_scope) {
                path[1] = system;
                for (std::size_t ni = 0; ni < native.size(); ++ni)
                    if (native[ni].at("stream") == Json(path) && native[ni].at("id") == g.at("id"))
                        table["native_record_candidates"].push_back(ni);
            }
            const auto &candidates = table["native_record_candidates"];
            if (candidates.size() > 1) {
                table["header_status"] = "ambiguous";
            } else if (candidates.size() == 1) {
                try {
                    const auto &n = native.at(candidates[0].get<std::size_t>());
                    const auto base = bytesof(n.at("data"));
                    require(n.at("element_type") == 92 && base.size() >= 44,
                            "display style table base header requires type 92 and 44 bytes");
                    // Native header +0x24, i.e. retained source base +40.
                    table["maximum_entry_index"] = Reader(base, 40).i32();
                    table["header_status"] = "decoded";
                } catch (const std::exception &e) {
                    table["header_status"] = "invalid";
                    table["header_error"] = e.what();
                }
            }
            for (std::size_t ei = 0; ei < attrs.size(); ++ei) {
                const auto &entry = attrs[ei];
                if (entry.at("group") != 0 || entry.at("key") != 22903)
                    continue;
                const auto &decoded = entry.at("decoded");
                Json row = {{"source", identity(g, gi, ei)}, {"index", entry.at("index")},
                            {"status", "uninterpreted"}, {"within_declared_range", nullptr}};
                if (table.contains("maximum_entry_index")) {
                    const auto maximum = table["maximum_entry_index"].get<std::int32_t>();
                    row["within_declared_range"] = maximum >= 0 &&
                        entry.at("index").get<std::uint32_t>() <= std::uint32_t(maximum);
                }
                if (decoded.contains("tree")) {
                    const auto &tree = decoded.at("tree");
                    const auto tag = tree.value("tag", "");
                    row["xml_tree"] = tree;
                    row["status"] = tag == "DisplayStyle" || tag == "ShowStyle"
                                        ? "source_xml" : "unrecognized_xml_root";
                    if (tree.contains("attributes") && tree["attributes"].contains("Name"))
                        row["name"] = tree["attributes"]["Name"];
                    row["native_xml_import"] = decode_display_style_xml(tree);
                    if (registration == 0x006f0000u)
                        row["lite_copy_projection"] = lite_copy_projection(tree);
                }
                table["entries"].push_back(std::move(row));
            }
            out["tables"].push_back(std::move(table));
        }
    }
    for (std::size_t gi = 0; gi < graphics.size(); ++gi) {
        const auto &g = graphics[gi];
        for (std::size_t ai = 0; ai < g.at("attributes").size(); ++ai) {
            const auto &d = g["attributes"][ai].at("decoded");
            if (d.value("encoding", "") != "native_display_style_reference")
                continue;
            const auto id = d.at("display_style_index").get<std::int32_t>();
            Json ref = {{"source", identity(g, gi, ai)}, {"display_style_index", id},
                        {"status", id < 0 ? "negative_index" : "no_source_candidate"},
                        {"candidates", Json::array()}, {"runtime_resolution", "not_evaluated"}};
            if (id >= 0) {
                for (std::size_t ti = 0; ti < out["tables"].size(); ++ti) {
                    const auto &table = out["tables"][ti];
                    if (!table.at("file_system_scope").get<bool>())
                        continue;
                    for (std::size_t ei = 0; ei < table["entries"].size(); ++ei) {
                        const auto &entry = table["entries"][ei];
                        if (entry.at("index") != id || entry.at("within_declared_range") == false)
                            continue;
                        Json candidate = {{"table_index", ti}, {"entry_index", ei},
                                          {"table_kind", table.at("kind")},
                                          {"header_status", table.at("header_status")},
                                          {"entry_status", entry.at("status")}};
                        if (entry.contains("name"))
                            candidate["name"] = entry.at("name");
                        ref["candidates"].push_back(std::move(candidate));
                    }
                }
                if (!ref["candidates"].empty())
                    ref["status"] = "source_candidates";
            }
            out["references"].push_back(std::move(ref));
        }
    }
    return out;
}

Json Document::display_style_sources() const {
    return build_display_style_sources(index(), native_records(), graphics_records());
}
} // namespace p3d
