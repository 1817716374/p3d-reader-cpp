#include "native_group_layers.hpp"
namespace p3d {
namespace {
std::string source_key(const Json &record) {
    return record.at("stream").dump() + "@" + record.at("offset").dump();
}
} // namespace
NativeGroupLayerQueries::NativeGroupLayerQueries(const Json &records, const Json &containers,
                                                 const std::vector<std::size_t> &targets)
    : data({{"scope", "initial_record_children_before_runtime_callbacks"},
            {"valid_loaded_root_required", true},
            {"runtime_validity", "not_evaluated"},
            {"queries", Json::array()},
            {"occurrences", Json::array()},
            {"containers", Json::array()},
            {"input_roots", Json::array()}}) {
    std::map<std::size_t, std::size_t> target_indices;
    for (const auto ni : targets) {
        const auto &record = records.at(ni);
        require(record.at("element_type") == 21, "group_layer_query_requires_type_21");
        const auto qi = data["queries"].size();
        require(target_indices.emplace(ni, qi).second, "duplicate_group_layer_query_source");
        require(by_source.emplace(source_key(record), qi).second,
                "ambiguous_group_layer_query_source_location");
        data["queries"].push_back({{"query_index", qi},
                                   {"native_record_index", ni},
                                   {"status", "not_evaluated"},
                                   {"selection", "recursive_first_child"},
                                   {"occurrence_indices", Json::array()},
                                   {"input_errors", Json::array()}});
    }
    for (std::size_t ci = 0; ci < containers.size(); ++ci) {
        data["containers"].push_back({{"container", containers.at(ci).at("container")},
                                      {"kind", containers.at(ci).at("kind")}});
        const auto &roots = containers.at(ci).at("list_preparation").at("roots");
        for (std::size_t ri = 0; ri < roots.size(); ++ri) {
            const auto &root = roots.at(ri);
            const auto &headers = root.at("headers");
            std::set<std::size_t> affected;
            for (const auto &h : headers) {
                const auto found =
                    target_indices.find(h.at("native_record_index").get<std::size_t>());
                if (found != target_indices.end())
                    affected.insert(found->second);
            }
            if (affected.empty())
                continue;
            try {
                const auto first = data["occurrences"].size();
                Json nodes = Json::array();
                std::map<std::size_t, std::size_t> indices;
                for (std::size_t hi = 0; hi < headers.size(); ++hi) {
                    const auto &h = headers.at(hi);
                    require(h.at("status") == "resolved", "group_input_header_preparation_unknown");
                    const auto ni = h.at("native_record_index").get<std::size_t>();
                    require(ni < records.size(), "group_input_source_index_invalid");
                    require(indices.emplace(ni, hi).second,
                            "group_input_source_repeated_within_root");
                    Json parent;
                    if (hi == 0) {
                        require(h.at("parent_record_index").is_null() &&
                                    root.at("native_record_index") == ni,
                                "group_input_root_identity_invalid");
                    } else {
                        require(!h.at("parent_record_index").is_null(),
                                "group_input_parent_missing");
                        const auto found =
                            indices.find(h.at("parent_record_index").get<std::size_t>());
                        require(found != indices.end() && found->second < hi,
                                "group_input_parent_must_precede_child");
                        parent = first + found->second;
                        nodes.at(found->second)["children"].push_back(first + hi);
                    }
                    const auto &record = records.at(ni);
                    require(record.at("stream") ==
                                records.at(root.at("native_record_index").get<std::size_t>())
                                    .at("stream"),
                            "group_input_child_crosses_source_stream");
                    nodes.push_back({{"native_record_index", ni},
                                     {"source_key", source_key(record)},
                                     {"source_id", record.at("id")},
                                     {"stream", record.at("stream")},
                                     {"offset", record.at("offset")},
                                     {"container_index", ci},
                                     {"root_input_index", ri},
                                     {"header_index", hi},
                                     {"parent", parent},
                                     {"children", Json::array()},
                                     {"native_type", record.at("element_type")}});
                }
                for (std::size_t hi = nodes.size(); hi-- > 0;) {
                    auto &node = nodes.at(hi);
                    Json result;
                    if (node.at("native_type") != 21) {
                        const auto ni = node.at("native_record_index").get<std::size_t>();
                        const auto bytes = bytesof(records.at(ni).at("data"));
                        // This getter reads the common native header, even if
                        // a non-graphic child uses the word as another field.
                        result = {{"status", "decoded"},
                                  {"layer_id", Reader(bytes, 16).u32()},
                                  {"terminal_selection", "stored_header"},
                                  {"terminal_occurrence_index", first + hi}};
                    } else if (node.at("children").empty()) {
                        result = {{"status", "decoded"},
                                  {"layer_id", UINT32_MAX},
                                  {"terminal_selection", "empty_group"},
                                  {"terminal_occurrence_index", first + hi}};
                    } else {
                        const auto child = node.at("children")[0].get<std::size_t>();
                        require(child >= first && child - first < nodes.size(),
                                "group_input_first_child_index_invalid");
                        result = nodes.at(child - first).at("layer_query");
                    }
                    node["layer_query"] = std::move(result);
                }
                data["input_roots"].push_back(
                    {{"container_index", ci},
                     {"root_input_index", ri},
                     {"native_record_index", root.at("native_record_index")},
                     {"block_number", root.at("block_number")},
                     {"first_occurrence_index", first},
                     {"occurrence_count", nodes.size()}});
                for (std::size_t hi = 0; hi < nodes.size(); ++hi) {
                    const auto ni = nodes.at(hi).at("native_record_index").get<std::size_t>();
                    const auto target = target_indices.find(ni);
                    if (target != target_indices.end())
                        data["queries"].at(target->second)["occurrence_indices"].push_back(first +
                                                                                           hi);
                    data["occurrences"].push_back(std::move(nodes.at(hi)));
                }
            } catch (const std::exception &e) {
                for (auto qi : affected)
                    data["queries"].at(qi)["input_errors"].push_back(
                        {{"container_index", ci}, {"root_input_index", ri}, {"reason", e.what()}});
            }
        }
    }
    for (auto &query : data["queries"]) {
        if (!query.at("input_errors").empty()) {
            query["reason"] = "group_input_occurrence_unresolved";
            continue;
        }
        if (query.at("occurrence_indices").empty()) {
            query["reason"] = "group_not_accepted_in_initial_input";
            continue;
        }
        std::optional<std::uint32_t> layer;
        bool empty = true, consistent = true;
        for (const auto &oi : query.at("occurrence_indices")) {
            const auto &result = data.at("occurrences").at(oi.get<std::size_t>()).at("layer_query");
            const auto id = result.at("layer_id").get<std::uint32_t>();
            if (layer && *layer != id)
                consistent = false;
            layer = id;
            empty = empty && result.at("terminal_selection") == "empty_group";
        }
        if (!consistent) {
            query["reason"] = "group_input_occurrences_disagree";
            continue;
        }
        query.update({{"status", "decoded"}, {"layer_id", *layer}, {"empty_group_result", empty}});
    }
}
} // namespace p3d
