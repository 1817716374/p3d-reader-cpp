#include "internal.hpp"
#include <p3d/view_sequence.hpp>
#include <numeric>
#include <set>

namespace p3d {
Json collect_view_link_candidates(const ViewCandidateContext &context) {
    Json out = {{"status", "unresolved"}, {"scope", "explicit_native_model_graph"}};
    try {
        auto node = [&](std::size_t index) -> const ViewCandidateNode & {
            require(index < context.nodes.size(), "candidate_model_index_out_of_range");
            return context.nodes[index];
        };
        const auto &current = node(context.current_model);
        if (!current.valid || (context.root_model_available && !*context.root_model_available)) {
            out.update(
                {{"status", "resolved"},
                 {"native_return_code", 1},
                 {"output_replaced", false},
                 {"reason", !current.valid ? "invalid_current_model" : "root_model_absent"}});
            return out;
        }
        require(context.root_model_available.has_value(), "root_model_availability_required");

        // Cache complete ancestry decisions for this call only. Shared parents
        // and repeated candidates refer to the same node, not equal file IDs.
        std::map<std::size_t, bool> ancestry;
        auto belongs = [&](std::size_t index) {
            std::vector<std::size_t> path;
            std::set<std::size_t> visiting;
            bool matched = false;
            for (;;) {
                const auto known = ancestry.find(index);
                if (known != ancestry.end()) {
                    matched = known->second;
                    break;
                }
                require(visiting.insert(index).second, "cycle_in_candidate_parent_chain");
                const auto &item = node(index);
                path.push_back(index);
                if (!item.valid)
                    break;
                require(item.kind == ViewCandidateNodeKind::model ||
                            item.kind == ViewCandidateNodeKind::reference,
                        "candidate_ancestry_kind_required");
                if (item.kind == ViewCandidateNodeKind::model || index == context.current_model) {
                    matched = index == context.current_model;
                    break;
                }
                require(item.parent_known, "candidate_parent_required");
                if (!item.parent)
                    break;
                index = *item.parent;
            }
            for (auto seen : path)
                ancestry.emplace(seen, matched);
            return matched;
        };
        std::vector<std::optional<std::size_t>> candidates;
        Json sources = Json::array(), rejected = Json::array();
        auto append = [&](std::optional<std::size_t> index, const char *source,
                          std::size_t offset) {
            if (index)
                (void)node(*index);
            candidates.push_back(index);
            sources.push_back({{"source", source}, {"source_index", offset}});
        };
        if (context.provided_candidates) {
            const auto &provided = *context.provided_candidates;
            for (std::size_t i = 0; i < provided.size(); ++i) {
                const auto index = provided[i];
                const char *reason = nullptr;
                if (!index)
                    reason = "null_candidate";
                else if (!node(*index).valid)
                    reason = "invalid_candidate";
                else if (!belongs(*index))
                    reason = "different_model_ancestry";
                if (reason)
                    rejected.push_back({{"source_index", i}, {"reason", reason}});
                else
                    append(index, "provided_candidate", i);
            }
        } else {
            if (context.include_current_model)
                append(context.current_model, "current_model", 0);
            if (context.include_links) {
                require(context.links_complete, "complete_native_link_list_required");
                for (std::size_t i = 0; i < context.links.size(); ++i)
                    append(context.links[i], "model_link", i);
            }
        }

        std::vector<std::size_t> order(candidates.size());
        std::iota(order.begin(), order.end(), 0);
        Json matches = Json::array();
        const bool sort = candidates.size() > 1 && context.apply_sequence && context.sequence &&
                          !context.sequence->empty();
        if (sort) {
            std::vector<ViewSequenceCandidate> known;
            bool all_ids_known = true;
            for (const auto &index : candidates) {
                if (!index || !node(*index).link_id) {
                    all_ids_known = false;
                    break;
                }
                known.push_back({*node(*index).link_id, *index == context.current_model});
            }
            if (all_ids_known) {
                const auto sorted = order_view_link_candidates(*context.sequence, known);
                order = sorted.at("candidate_indices").get<std::vector<std::size_t>>();
                matches = sorted.at("matches");
            } else {
                // Follow native short-circuiting when some IDs are unavailable:
                // zero compares identity only, and earlier matches avoid later
                // dereferences. Unknown unused fields must not reject a result.
                std::size_t next = 0;
                for (std::size_t i = 0; i < context.sequence->size(); ++i) {
                    const auto id = context.sequence->at(i);
                    for (std::size_t pos = next; pos < order.size(); ++pos) {
                        const auto index = candidates[order[pos]];
                        bool match;
                        if (id == 0)
                            match = index && *index == context.current_model;
                        else {
                            require(index.has_value(), "null_candidate_in_native_id_lookup");
                            const auto &item = node(*index);
                            require(item.link_id.has_value(), "candidate_link_id_required");
                            match = *item.link_id == id;
                        }
                        if (!match)
                            continue;
                        matches.push_back({{"sequence_index", i},
                                           {"candidate_index", order[pos]},
                                           {"from_position", pos},
                                           {"to_position", next}});
                        std::swap(order[pos], order[next]);
                        ++next;
                        break;
                    }
                }
            }
        }
        Json result = Json::array();
        for (auto index : order) {
            auto item = sources[index];
            item["collected_index"] = index;
            item["model_index"] = candidates[index] ? Json(*candidates[index]) : Json();
            result.push_back(std::move(item));
        }
        out.update({{"status", "resolved"},
                    {"native_return_code", candidates.empty() ? 1 : 0},
                    {"output_replaced", true},
                    {"sequence_applied", sort},
                    {"candidates", std::move(result)},
                    {"rejected_candidates", std::move(rejected)},
                    {"matches", std::move(matches)}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
