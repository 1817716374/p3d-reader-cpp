#include "geometry.hpp"
#include "line_style_source.hpp"
#include <p3d/line_style.hpp>
#include <p3d/native_input.hpp>

namespace p3d {
namespace {
std::uint64_t reference_id(const Json &record) {
    const auto b = bytesof(record.at("data"));
    Reader r(b);
    require(b.size() >= 532, "point_symbol_fixed_parameters_truncated");
    require(r.at<std::uint16_t>(4) == 49 && r.at<std::uint32_t>(16) == 8 &&
                r.at<std::uint16_t>(70) == 1,
            "point_symbol_component_required");
    // Independent native OO reference read: do not parse or reconstruct the
    // possibly large embedded buffer merely to locate this separate target.
    return r.at<std::uint64_t>(208 + std::size_t(r.at<std::uint16_t>(204)));
}
} // namespace

Json native_line_style_symbol_geometry_source(const Json &symbol, const Json &owner_ids,
                                              const Json &system_ids, const Json &records,
                                              std::size_t max_nodes) {
    Json out = {{"scope", "initial_symbol_geometry_source_registry_binding"},
                {"status", "not_evaluated"},
                {"source_tree_status", "not_evaluated"},
                {"nodes", Json::array()},
                {"runtime_filter", "not_applied"},
                {"descriptor_conversion", "not_evaluated"},
                {"placement", "not_evaluated"},
                {"appearance", "not_evaluated"}};
    try {
        const auto id = reference_id(symbol);
        out["requested_id"] = id;
        if (!id) {
            out["status"] = "not_requested";
            out["source_tree_status"] = "not_requested";
            return out; // The native reader does not perform an ID-zero lookup.
        }
        auto lookup = native_owner_object_lookup(owner_ids, system_ids, id);
        out["lookup"] = lookup;
        if (lookup.at("status") != "selected") {
            out["status"] = lookup.at("status");
            if (lookup.contains("reason"))
                out["reason"] = lookup.at("reason");
            return out;
        }
        const auto scope = lookup.at("selected_scope").get<std::string>();
        out["selected_scope"] = scope;
        out["target"] = lookup.at("target");
        const auto &ids = scope == "owner" ? owner_ids : system_ids;
        std::map<std::size_t, const Json *> objects;
        std::map<std::size_t, std::vector<std::size_t>> children;
        for (const auto &root : ids.at("roots"))
            for (const auto &item : root.at("records")) {
                const auto occurrence = item.at("input_occurrence_index").get<std::size_t>();
                require(objects.emplace(occurrence, &item).second,
                        "ambiguous_symbol_geometry_input_occurrence");
                const auto &parent = item.at("parent_input_occurrence_index");
                if (!parent.is_null())
                    children[parent.get<std::size_t>()].push_back(occurrence);
            }
        const auto first = lookup.at("target").at("input_occurrence_index").get<std::size_t>();
        const auto &target = *objects.at(first);
        require(target.at("assigned_id") == id && target.at("native_record_index") ==
                                                      lookup.at("target").at("native_record_index"),
                "symbol_geometry_registry_identity_mismatch");
        struct Pending {
            std::size_t occurrence;
            Json parent;
        };
        std::vector<Pending> pending{{first, nullptr}};
        std::set<std::size_t> visited;
        while (!pending.empty()) {
            require(out["nodes"].size() < max_nodes, "symbol_geometry_source_node_limit");
            auto next = std::move(pending.back());
            pending.pop_back();
            require(visited.insert(next.occurrence).second, "cyclic_symbol_geometry_source_tree");
            const auto &item = *objects.at(next.occurrence);
            const auto index = item.at("native_record_index").get<std::size_t>();
            const auto &record = records.at(index);
            // Do not rewrite the record's original ID/flags to imitate a
            // runtime descriptor. The prepared occurrence is independent.
            Json node = item;
            const auto n = out["nodes"].size();
            node["node_index"] = n;
            node["parent_node_index"] = next.parent;
            node["children"] = Json::array();
            node["element_type"] = record.at("element_type");
            node["source_element_id"] = record.at("id");
            if (!next.parent.is_null())
                out["nodes"][next.parent.get<std::size_t>()]["children"].push_back(n);
            out["nodes"].push_back(std::move(node));
            const auto found = children.find(next.occurrence);
            if (found != children.end())
                for (auto it = found->second.rbegin(); it != found->second.rend(); ++it)
                    pending.push_back({*it, n});
        }
        out["source_tree_status"] = "resolved";
        out["status"] = "selected";
    } catch (const std::exception &e) {
        out["status"] = "not_evaluated";
        out["reason"] = e.what();
    }
    return out;
}

Json read_native_line_style_symbol_geometry_source(const Document &document,
                                                   const StreamPath &model_storage,
                                                   std::uint64_t initial_id_counter,
                                                   const Json &record, std::size_t max_nodes) {
    Json result;
    try {
        const auto id = reference_id(record);
        if (!id) {
            result = native_line_style_symbol_geometry_source(record, {}, {}, {}, max_nodes);
        } else {
            const auto containers = p3d::native_input_containers(
                document.streams(), document.index(), document.native_records());
            const auto owner = p3d::native_model_id_assignments(
                containers, document.native_records(), document.index(), model_storage,
                initial_id_counter);
            bool known_model = false;
            for (const auto &input : owner.at("inputs"))
                known_model = known_model || input.at("status") != "not_loaded";
            require(known_model, "model_record_container_unavailable");
            Json system;
            const auto lookup = native_owner_object_lookup(owner, {}, id);
            if (lookup.value("reason", Json()) == "complete_system_registry_required") {
                const Json *selected = nullptr;
                bool ambiguous = false;
                for (const auto &container : containers)
                    if (container.at("kind") == "P3D-SSYS") {
                        ambiguous = ambiguous || selected != nullptr;
                        selected = &container;
                    }
                if (selected && !ambiguous)
                    system = native_system_id_assignments(selected->at("list_preparation"),
                                                          document.native_records(),
                                                          document.file_header());
            }
            result = native_line_style_symbol_geometry_source(record, owner, system,
                                                              document.native_records(), max_nodes);
        }
    } catch (const std::exception &e) {
        result = {{"scope", "initial_symbol_geometry_source_registry_binding"},
                  {"status", "not_evaluated"},
                  {"source_tree_status", "not_evaluated"},
                  {"nodes", Json::array()},
                  {"reason", e.what()}};
    }
    result["model_storage"] = model_storage;
    result["initial_id_counter"] = initial_id_counter;
    return result;
}

Json prepare_native_line_style_symbol_geometry_source(const Document &document,
                                                      const StreamPath &model_storage,
                                                      std::uint64_t initial_id_counter,
                                                      const Json &record, std::size_t max_nodes) {
    auto out = read_native_line_style_symbol_geometry_source(document, model_storage,
                                                             initial_id_counter, record, max_nodes);
    out["scope"] = "initial_symbol_geometry_source_record_preparation";
    out["preparation_status"] = "not_evaluated";
    out["prepared_nodes"] = Json::array();
    if (out.at("status") == "not_requested") {
        out["preparation_status"] = "not_requested";
        return out;
    }
    if (out.at("status") != "selected")
        return out;
    try {
        const auto &nodes = out.at("nodes");
        std::vector<std::uint64_t> descendants(nodes.size(), 0);
        for (std::size_t n = nodes.size(); n-- > 0;) {
            const auto &parent = nodes[n].at("parent_node_index");
            if (!parent.is_null()) {
                const auto p = parent.get<std::size_t>();
                require(p < n, "symbol_source_parent_order_invalid");
                descendants[p] += descendants[n] + 1;
            }
        }
        std::map<StreamPath, const Bytes *> streams;
        for (const auto &stream : document.streams()) {
            require(streams.emplace(stream.path, stream.decoded.get()).second,
                    "ambiguous_native_record_stream");
        }
        bool all_decoded = true;
        for (std::size_t n = 0; n < nodes.size(); ++n) {
            const auto &node = nodes[n];
            const auto &saved =
                document.native_records().at(node.at("native_record_index").get<std::size_t>());
            const auto found = streams.find(saved.at("stream").get<StreamPath>());
            require(found != streams.end() && found->second, "native_source_stream_unavailable");
            const auto full = slice(*found->second, saved.at("offset").get<std::size_t>(),
                                    saved.at("length").get<std::size_t>());
            require(descendants[n] <= UINT32_MAX, "symbol_source_descendant_count_overflow");
            auto prepared = prepare_native_record_input(
                full, node.at("assigned_id").get<std::uint64_t>(), std::uint32_t(descendants[n]),
                !node.at("parent_input_occurrence_index").is_null(),
                out.at("selected_scope") == "system");
            require(prepared.at("status") == "prepared",
                    prepared.value("reason", std::string("symbol_source_record_not_prepared")));
            require(prepared.at("header_preparation").at("output_element_flags") ==
                        node.at("prepared_element_flags"),
                    "symbol_prepared_header_flags_mismatch");
            all_decoded = all_decoded && prepared.at("decoded_status") == "decoded";
            out["prepared_nodes"].push_back(
                {{"source_node_index", n},
                 {"native_record_index", node.at("native_record_index")},
                 {"input_occurrence_index", node.at("input_occurrence_index")},
                 {"preparation", std::move(prepared)}});
        }
        out["preparation_status"] = "prepared";
        out["all_prepared_records_decoded"] = all_decoded;
    } catch (const std::exception &e) {
        out["preparation_reason"] = e.what();
    }
    return out;
}

Geometry reconstruct_native_line_style_symbol_geometry_source(
    const Document &document, const StreamPath &model_storage, std::uint64_t initial_id_counter,
    const Json &record, const Tessellation &policy, std::size_t max_nodes) {
    Geometry geometry;
    try {
        const auto source = read_native_line_style_symbol_geometry_source(
            document, model_storage, initial_id_counter, record, max_nodes);
        if (source.at("status") == "not_requested")
            return geometry;
        require(source.at("status") == "selected",
                source.value("reason", std::string("symbol_geometry_source_not_selected")));
        for (const auto &node : source.at("nodes")) {
            const auto &saved =
                document.native_records().at(node.at("native_record_index").get<std::size_t>());
            auto part = reconstruct_native_line_style_symbol_record(saved, policy);
            for (auto *items : {&part.primitive_ranges, &part.unknown})
                for (auto &item : *items) {
                    item["symbol_source_node_index"] = node.at("node_index");
                    item["native_record_index"] = node.at("native_record_index");
                    item["input_occurrence_index"] = node.at("input_occurrence_index");
                    item["assigned_element_id"] = node.at("assigned_id");
                    item["lookup_scope"] = source.at("selected_scope");
                }
            merge_geometry(geometry, part, identity());
        }
        geometry.notes.push_back(
            "Saved source subtree coordinates; runtime descriptor conversion, "
            "symbol scale/offset, placement and appearance are not evaluated.");
    } catch (const std::exception &e) {
        geometry.unknown.push_back({{"reason", e.what()}});
    }
    return geometry;
}
} // namespace p3d
