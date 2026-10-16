#include "line_style_source.hpp"
#include <p3d/line_style.hpp>

namespace p3d {
namespace {
bool compound(std::uint16_t type, std::uint16_t flags) {
    return (type >= 10 && type <= 32) || ((type < 33 || type > 63) && (flags & 0x40));
}
} // namespace

Json native_line_style_symbol_initial_descriptor(Json source, std::size_t max_bytes) {
    Json out = {{"scope", "ordinary_initial_symbol_descriptor_and_owned_copy"},
                {"status", "not_evaluated"},
                {"runtime_state", "ordinary_fresh_input_before_host_callbacks"},
                {"source_preparation", std::move(source)},
                {"nodes", Json::array()},
                {"root_descriptor_index", nullptr},
                {"symbol_copy_status", "not_evaluated"},
                {"placement", "not_evaluated"},
                {"appearance", "not_evaluated"},
                {"unvisited_source_node_indices", Json::array()}};
    std::size_t total = 0;
    try {
        const auto &prepared = out.at("source_preparation");
        if (prepared.at("preparation_status") == "not_requested") {
            out["status"] = "not_requested";
            out["symbol_copy_status"] = "not_requested";
            out["copied_native_bytes"] = 0;
            return out;
        }
        require(prepared.at("status") == "selected" &&
                    prepared.at("preparation_status") == "prepared",
                prepared.value(
                    "preparation_reason",
                    prepared.value("reason",
                                   std::string("complete_symbol_source_preparation_required"))));
        const auto &nodes = prepared.at("nodes"), &buffers = prepared.at("prepared_nodes");
        require(nodes.is_array() && !nodes.empty() && buffers.size() == nodes.size(),
                "complete_symbol_prepared_node_mapping_required");
        auto bytes = [&](std::size_t i) {
            const auto &entry = buffers.at(i);
            require(entry.at("source_node_index") == i &&
                        entry.at("native_record_index") == nodes.at(i).at("native_record_index") &&
                        entry.at("input_occurrence_index") ==
                            nodes.at(i).at("input_occurrence_index"),
                    "symbol_descriptor_source_identity_mismatch");
            const auto &input = entry.at("preparation");
            require(input.at("status") == "prepared", "symbol_descriptor_record_not_prepared");
            auto b = bytesof(input.at("prepared_record"));
            Reader r(b);
            r.need(36);
            const auto words = r.at<std::uint32_t>(8);
            require(words >= 16 && words <= 65535,
                    "ordinary_initial_native_header_copy_not_loaded");
            require(b.size() == 4 + std::size_t(words) * 2,
                    "symbol_descriptor_native_record_truncated");
            require(r.at<std::uint64_t>(20) == nodes.at(i).at("assigned_id") &&
                        r.at<std::uint16_t>(4) == nodes.at(i).at("element_type"),
                    "symbol_descriptor_prepared_header_identity_mismatch");
            return b;
        };
        struct Pending {
            std::size_t source;
            Json parent;
        };
        std::vector<Pending> pending{{0, nullptr}};
        std::set<std::size_t> visited;
        while (!pending.empty()) {
            auto next = std::move(pending.back());
            pending.pop_back();
            out["pending_source_node_index"] = next.source;
            require(visited.insert(next.source).second, "cyclic_symbol_descriptor_source_tree");
            const auto &saved_node = nodes.at(next.source);
            const auto &parent_source = saved_node.at("parent_node_index");
            if (!next.parent.is_null()) {
                const auto parent = next.parent.get<std::size_t>();
                require(parent < out["nodes"].size() && !parent_source.is_null() &&
                            parent_source == out["nodes"][parent].at("source_node_index"),
                        "symbol_descriptor_source_parent_mismatch");
            }
            auto b = bytes(next.source);
            Reader r(b);
            const auto type = r.at<std::uint16_t>(4), flags = r.at<std::uint16_t>(6);
            const bool grouped = compound(type, flags);
            // A root ignores its saved count. A compound child with a
            // nonpositive count is copied without recursing into its children.
            bool recurse = grouped;
            if (recurse && !next.parent.is_null())
                recurse = r.at<std::int32_t>((flags & 0x20) ? 108 : 36) > 0;
            const auto count = b.size() - 4;
            require(total <= max_bytes && count <= max_bytes - total,
                    "symbol_descriptor_byte_limit");
            auto native = slice(b, 4, count);
            if (!next.parent.is_null()) {
                const auto child_flags = std::uint16_t(flags | 0x80);
                std::memcpy(native.data() + 2, &child_flags, 2);
            }
            const auto index = out["nodes"].size();
            Json back, group;
            if (next.parent.is_null()) {
                if (grouped)
                    group = index;
            } else {
                auto &children = out["nodes"][next.parent.get<std::size_t>()]["children"];
                back = children.empty() ? next.parent : children.back();
                group = next.parent;
                if (!children.empty())
                    out["nodes"][children.back().get<std::size_t>()]["next_sibling_index"] = index;
                children.push_back(index);
            }
            out["nodes"].push_back(
                {{"descriptor_index", index},
                 {"source_node_index", next.source},
                 {"native_record_index", saved_node.at("native_record_index")},
                 {"input_occurrence_index", saved_node.at("input_occurrence_index")},
                 {"assigned_element_id", saved_node.at("assigned_id")},
                 {"lookup_scope", prepared.at("selected_scope")},
                 {"parent_descriptor_index", next.parent},
                 {"next_sibling_index", nullptr},
                 {"back_link_index", back},
                 {"native_group_index", group},
                 {"native_flag_30", grouped ? 1 : 0},
                 {"construction_native_flag_34", next.parent.is_null() || recurse ? 1 : 0},
                 {"native_flag_34", 0},
                 {"native_header", rawbytes(native)},
                 {"copied_bytes", count},
                 {"children", Json::array()},
                 {"children_walk", recurse ? "recursive" : "single_record"},
                 {"prepared_stream_decode_status",
                  buffers.at(next.source).at("preparation").at("decoded_status")}});
            total += count;
            if (recurse) {
                const auto &children = saved_node.at("children");
                require(children.is_array(), "symbol_descriptor_children_array_required");
                for (auto it = children.rbegin(); it != children.rend(); ++it)
                    pending.push_back({it->get<std::size_t>(), index});
            }
        }
        // 187990 clones the constructed chain into symbol-owned storage.
        // Fresh descriptors have no auxiliary allocation entries. Their clone
        // retains native headers/provenance and resets field +0x34 to zero.
        for (std::size_t i = 0; i < nodes.size(); ++i)
            if (!visited.count(i))
                out["unvisited_source_node_indices"].push_back(i);
        out["root_descriptor_index"] = 0;
        out["symbol_copy_status"] = "copied";
        out["status"] = "built";
        out.erase("pending_source_node_index");
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    out["copied_native_bytes"] = total;
    return out;
}

Json build_native_line_style_symbol_initial_descriptor(const Document &document,
                                                       const StreamPath &model_storage,
                                                       std::uint64_t initial_id_counter,
                                                       const Json &record, std::size_t max_nodes,
                                                       std::size_t max_bytes) {
    return native_line_style_symbol_initial_descriptor(
        prepare_native_line_style_symbol_geometry_source(document, model_storage,
                                                         initial_id_counter, record, max_nodes),
        max_bytes);
}
} // namespace p3d
