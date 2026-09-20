#include "internal.hpp"
#include <p3d/reference_search.hpp>
#include "reference_graph_internal.hpp"
#include <unordered_set>

namespace p3d {
namespace {
template <class T> const T &known(const std::optional<T> &value, const char *reason) {
    require(value.has_value(), reason);
    return *value;
}
std::u16string prefix(const std::u16string &s) {
    return s.substr(0, s.find(u'\0'));
}
} // namespace

Json reference_descendant_search_gate(const ReferenceDescendantGateContext &c) {
    Json out = {{"status", "unresolved"},
                {"scope", "ordinary_reference_descendant_search_gate"},
                {"examined_candidate_hosts", 0}};
    auto finish = [&](bool required, const char *reason) {
        out.update({{"status", "resolved"}, {"search_required", required}, {"reason", reason}});
        return out;
    };
    try {
        if (!known(c.host_valid, "host_validity_required"))
            return finish(false, "invalid_host");
        if (!known(c.host_parent_present, "host_parent_presence_required"))
            return finish(false, "host_has_no_parent");
        if (!known(c.candidate_valid, "candidate_validity_required"))
            return finish(false, "invalid_candidate");
        if (!known(c.candidate_reference_present, "candidate_reference_presence_required"))
            return finish(false, "candidate_has_no_reference");
        if (known(c.candidate_primary_flags, "candidate_primary_flags_required") & 0x10000000u)
            return finish(true, "candidate_primary_bit28");
        for (std::size_t i = 0; i < c.candidate_hosts.size(); ++i) {
            out["examined_candidate_hosts"] = i + 1;
            const auto &host = c.candidate_hosts[i];
            if (!host.valid)
                return finish(false, "invalid_candidate_host");
            require(!(host.reference_known_absent && host.reference_primary_flags),
                    "conflicting_candidate_host_reference_state");
            if (host.reference_known_absent)
                return finish(false, "candidate_host_has_no_reference");
            if (known(host.reference_primary_flags, "candidate_host_reference_flags_required") &
                0x10000000u) {
                out["matching_candidate_host_index"] = i;
                return finish(true, "candidate_host_primary_bit28");
            }
        }
        require(c.complete_candidate_host_chain, "remaining_candidate_host_chain_required");
        return finish(false, "complete_candidate_host_chain_without_bit28");
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json match_reference_model(const ReferenceSearchQuery &q, const ReferenceSearchModel &m) {
    Json out = {{"status", "unresolved"}, {"scope", "loaded_reference_model_search_match"}};
    auto finish = [&](bool matched, const char *reason) {
        out.update({{"status", "resolved"}, {"matched", matched}, {"reason", reason}});
        return out;
    };
    try {
        require(q.reference_present, "reference_object_required_for_model_match");
        bool root = false;
        if (m.valid) {
            if (m.object_dispatch == ReferenceObjectDispatch::model) {
                require(!m.root_present || *m.root_present, "ordinary_model_has_no_self_root");
                root = true;
            } else
                root = known(m.root_present, "search_model_root_presence_required");
        }
        std::u16string file;
        bool file_present = false;
        if (root) {
            require(!(m.file_known_absent && m.file), "conflicting_search_model_file_state");
            if (!m.file_known_absent) {
                const auto &f = known(m.file, "search_model_file_required");
                file = prefix(f.lookup_reference);
                if (file.empty())
                    file = prefix(f.stored_reference);
                file_present = true;
            }
        }
        auto equal = [&](const std::u16string &a, const std::u16string &b) {
            if (a == b)
                return true;
            require(bool(q.equal), "native_wide_case_comparison_required");
            return q.equal(a, b);
        };
        if (equal(file,
                  prefix(known(q.stored_file_reference, "search_stored_file_reference_required"))))
            out["file_match_source"] = "stored_reference";
        else if (equal(file, prefix(known(q.lookup_file_reference,
                                          "search_lookup_file_reference_required"))))
            out["file_match_source"] = "lookup_reference";
        else
            return finish(false, "file_references_differ");
        if (known(q.runtime_flags, "search_reference_runtime_flags_required") & 0x1000u) {
            const auto id = !m.valid ? UINT32_C(0xfffffffe)
                            : !root  ? UINT32_MAX
                                     : known(m.model_id, "search_model_id_required");
            out["model_match_kind"] = "model_id";
            out["compared_model_id"] = id;
            return finish(id == known(q.model_id, "search_reference_model_id_required"),
                          "model_id_comparison");
        }
        const auto name =
            prefix(known(q.model_name, "search_reference_primary_model_name_required"));
        if (root && !name.empty()) {
            out["model_match_kind"] = "primary_model_name";
            return finish(equal(name, prefix(known(m.model_name, "search_model_name_required"))),
                          "primary_model_name_comparison");
        }
        out["model_match_kind"] = "file_default_model";
        return finish(root && file_present &&
                          known(m.is_default_model, "search_default_model_state_required"),
                      "default_model_comparison");
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json detail::search_reference_descendants_impl(const ReferenceSearchQuery &query,
                                               const std::vector<ReferenceSearchModel> &models,
                                               bool start_known, std::optional<std::size_t> start) {
    Json out = {{"status", "unresolved"},
                {"scope", "explicit_active_reference_descendant_search"},
                {"visited_model_indices", Json::array()},
                {"skipped_links", Json::array()}};
    auto finish = [&](bool matched) {
        out.update({{"status", "resolved"}, {"matched", matched}});
        return out;
    };
    struct Frame {
        std::optional<std::size_t> index;
        std::size_t next = 0;
        bool entered = false;
    };
    try {
        if (!query.reference_present)
            return finish(false);
        require(start_known, "reference_search_start_required");
        ReferenceSearchModel null_model;
        null_model.valid = false;
        std::vector<Frame> stack = {{start}};
        std::unordered_set<std::size_t> active;
        if (start)
            active.insert(*start);
        while (!stack.empty()) {
            auto &frame = stack.back();
            require(!frame.index || *frame.index < models.size(),
                    "search_model_index_out_of_range");
            const auto &model = frame.index ? models[*frame.index] : null_model;
            if (!frame.entered) {
                frame.entered = true;
                out["visited_model_indices"].push_back(frame.index ? Json(*frame.index) : Json());
                auto matched = match_reference_model(query, model);
                if (matched.at("status") != "resolved") {
                    out["reason"] = matched.at("reason");
                    out["unresolved_model_match"] = std::move(matched);
                    return out;
                }
                if (matched.at("matched").get<bool>()) {
                    out["matching_model_index"] = frame.index ? Json(*frame.index) : Json();
                    out["matching_path"] = Json::array();
                    for (const auto &f : stack)
                        out["matching_path"].push_back(f.index ? Json(*f.index) : Json());
                    out["model_match"] = std::move(matched);
                    return finish(true);
                }
            }
            if (!model.valid || model.active_list_known_absent) {
                if (model.valid)
                    require(model.active_links.empty(), "conflicting_active_list_state");
                if (frame.index)
                    active.erase(*frame.index);
                stack.pop_back();
                continue;
            }
            if (frame.next == model.active_links.size()) {
                require(model.active_links_complete, "remaining_active_reference_links_required");
                if (frame.index)
                    active.erase(*frame.index);
                stack.pop_back();
                continue;
            }
            const auto slot = frame.next++;
            const auto child = model.active_links[slot];
            auto skip = [&](const char *reason) {
                out["skipped_links"].push_back({{"parent_model_index", *frame.index},
                                                {"link_index", slot},
                                                {"reason", reason}});
            };
            if (!child) {
                skip("null_link");
                continue;
            }
            require(*child < models.size(), "active_reference_index_out_of_range");
            const auto &node = models[*child];
            if (detail::reference_object_kind(node) != 2) {
                skip("native_kind_not_2");
                continue;
            }
            require(node.valid, "invalid_kind2_reference_in_active_list");
            require(detail::reference_object_present(node), "kind2_object_has_no_reference");
            if (known(node.reference_runtime_flags, "active_reference_runtime_flags_required") &
                0x20u) {
                skip("reference_runtime_bit5");
                continue;
            }
            require(active.insert(*child).second, "cycle_in_active_reference_search");
            stack.push_back({child});
        }
        return finish(false);
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
Json search_reference_descendants(const ReferenceSearchQuery &query,
                                  const ReferenceSearchContext &context) {
    return detail::search_reference_descendants_impl(query, context.models, context.start_known,
                                                     context.start);
}
} // namespace p3d
