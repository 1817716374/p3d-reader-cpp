#include "internal.hpp"
#include "reference_graph_internal.hpp"
#include <unordered_set>

namespace p3d {
namespace {
template <class T> const T &known(const std::optional<T> &v, const char *reason) {
    require(v.has_value(), reason);
    return *v;
}
const ReferenceSearchModel &object(const ReferenceSearchContext &c, std::size_t i) {
    require(i < c.models.size(), "reference_graph_index_out_of_range");
    return c.models[i];
}
std::optional<std::size_t> parent(const ReferenceSearchModel &m) {
    if (m.object_dispatch == ReferenceObjectDispatch::model) {
        require(!m.parent_known || !m.parent, "ordinary_model_has_nonnull_parent");
        return {};
    }
    require(m.parent_known, "reference_parent_relation_required");
    return m.parent;
}
} // namespace
std::int32_t detail::reference_object_kind(const ReferenceSearchModel &m) {
    if (m.object_dispatch == ReferenceObjectDispatch::unknown)
        return known(m.native_kind, "active_reference_kind_required");
    const auto kind = m.object_dispatch == ReferenceObjectDispatch::model ? 1 : 2;
    require(m.object_dispatch == ReferenceObjectDispatch::model ||
                m.object_dispatch == ReferenceObjectDispatch::reference,
            "unknown_reference_object_dispatch");
    require(!m.native_kind || *m.native_kind == kind, "conflicting_reference_object_kind");
    return kind;
}
bool detail::reference_object_present(const ReferenceSearchModel &m) {
    if (m.object_dispatch == ReferenceObjectDispatch::unknown)
        return known(m.reference_present, "active_reference_presence_required");
    const bool present = m.object_dispatch == ReferenceObjectDispatch::reference;
    require(present || m.object_dispatch == ReferenceObjectDispatch::model,
            "unknown_reference_object_dispatch");
    require(!m.reference_present || *m.reference_present == present,
            "conflicting_reference_object_presence");
    return present;
}

Json reference_parent_root(const ReferenceSearchContext &c, std::optional<std::size_t> index) {
    Json out = {{"status", "unresolved"},
                {"scope", "native_parent_root_query"},
                {"parent_path", Json::array()}};
    try {
        std::unordered_set<std::size_t> visited;
        while (index) {
            require(visited.insert(*index).second, "cycle_in_reference_parent_root_query");
            const auto &m = object(c, *index);
            out["parent_path"].push_back(*index);
            if (!m.valid) {
                index.reset();
                break;
            }
            if (m.object_dispatch == ReferenceObjectDispatch::model)
                break;
            require(m.object_dispatch == ReferenceObjectDispatch::reference,
                    "reference_parent_dispatch_required");
            index = parent(m);
        }
        out.update({{"status", "resolved"}, {"model_index", index ? Json(*index) : Json()}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

std::optional<std::size_t>
detail::reference_connected_root_impl(const std::vector<ReferenceSearchModel> &models,
                                      std::optional<std::size_t> index) {
    if (!index)
        return {};
    require(*index < models.size(), "reference_graph_index_out_of_range");
    const auto &node = models[*index];
    if (!node.valid)
        return {};
    std::optional<std::size_t> root;
    if (node.object_dispatch == ReferenceObjectDispatch::model) {
        require(!node.root_present || *node.root_present, "ordinary_model_has_no_self_root");
        root = index;
        require(!node.connected_root_known || node.connected_root == root,
                "ordinary_model_has_nonself_connected_root");
    } else if (node.connected_root_known) {
        root = node.connected_root;
    } else {
        require(node.root_present && !*node.root_present, "connected_root_identity_required");
    }
    require(!node.root_present || *node.root_present == root.has_value(),
            "conflicting_connected_root_presence");
    require(!root || *root < models.size(), "connected_root_index_out_of_range");
    // The native getter returns this model pointer directly. It does not walk
    // the target's parent/root chain or apply another object-validity filter.
    return root;
}

Json reference_connected_root(const ReferenceSearchContext &c, std::optional<std::size_t> index) {
    Json out = {{"status", "unresolved"}, {"scope", "native_connected_root_query"}};
    try {
        const auto root = detail::reference_connected_root_impl(c.models, index);
        out.update({{"status", "resolved"}, {"model_index", root ? Json(*root) : Json()}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json reference_nesting_depth(const ReferenceSearchContext &c, std::optional<std::size_t> index,
                             std::int32_t cap) {
    Json out = {{"status", "unresolved"},
                {"scope", "native_reference_ancestor_nesting_depth"},
                {"cap", cap},
                {"reference_path", Json::array()},
                {"local_limits", Json::array()}};
    try {
        std::unordered_set<std::size_t> visited;
        std::vector<std::int32_t> limits;
        while (index) {
            const auto &m = object(c, *index);
            if (!m.valid || !detail::reference_object_present(m))
                break;
            require(visited.insert(*index).second, "cycle_in_reference_nesting_depth");
            const auto source = known(m.reference_nest_depth, "reference_nest_depth_required");
            const auto local = cap > 0 ? std::min<std::int32_t>(source, cap) : source;
            limits.push_back(local);
            out["reference_path"].push_back(*index);
            out["local_limits"].push_back(local);
            index = parent(m);
        }
        // Native only recurses when a parent has a reference. A parent model
        // without one terminates the chain; its null-reference result 1 is NOT
        // subtracted into the preceding reference's own limit.
        std::int32_t depth = limits.empty() ? 1 : limits.back();
        for (std::size_t i = limits.size(); i > 1; --i) {
            const auto bits = static_cast<std::uint32_t>(depth) - 1u;
            const auto previous = bits <= INT32_MAX
                                      ? static_cast<std::int32_t>(bits)
                                      : static_cast<std::int32_t>(static_cast<std::int64_t>(bits) -
                                                                  INT64_C(0x100000000));
            depth = std::min(limits[i - 2], previous);
        }
        out.update({{"status", "resolved"}, {"remaining_depth", depth}, {"exhausted", depth <= 0}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json reference_link_loading_policy(const ReferenceSearchContext &c, std::optional<std::size_t> host,
                                   bool force_input, std::int32_t cap) {
    Json out = {{"status", "unresolved"},
                {"scope", "ordinary_reference_collection_depth_gate"},
                {"force_input", force_input}};
    bool restricted = false;
    if (!force_input) {
        out["nesting"] = reference_nesting_depth(c, host, cap);
        if (out["nesting"].at("status") != "resolved") {
            out["reason"] = out["nesting"].at("reason");
            return out;
        }
        restricted = out["nesting"].at("exhausted").get<bool>();
    }
    out.update({{"status", "resolved"},
                {"require_source_bit14", restricted},
                {"process_view_sequence", !restricted}});
    return out;
}

Json evaluate_reference_descendant_filter(const ReferenceSearchQuery &query,
                                          const ReferenceSearchContext &c,
                                          std::optional<std::size_t> host,
                                          std::optional<std::size_t> candidate) {
    Json out = {{"status", "unresolved"},
                {"scope", "graph_derived_reference_descendant_filter"},
                {"candidate_host_indices", Json::array()},
                {"search_performed", false}};
    ReferenceDescendantGateContext gate;
    std::optional<std::size_t> host_parent;
    auto evaluate = [&]() {
        out["gate"] = reference_descendant_search_gate(gate);
        if (out["gate"].at("status") != "resolved") {
            out["reason"] = out["gate"].at("reason");
            return out;
        }
        if (!out["gate"].at("search_required").get<bool>()) {
            out.update({{"status", "resolved"}, {"excluded", false}});
            return out;
        }
        out["search_root"] = reference_parent_root(c, host_parent);
        if (out["search_root"].at("status") != "resolved") {
            out["reason"] = out["search_root"].at("reason");
            return out;
        }
        require(query.reference_present, "conflicting_search_candidate_presence");
        auto selected_query = query;
        const auto &runtime = object(c, *candidate).reference_runtime_flags;
        require(!runtime || !query.runtime_flags || *runtime == *query.runtime_flags,
                "conflicting_search_candidate_runtime_flags");
        if (runtime)
            selected_query.runtime_flags = runtime;
        const auto &root = out["search_root"].at("model_index");
        std::optional<std::size_t> start;
        if (!root.is_null())
            start = root.get<std::size_t>();
        out["search_performed"] = true;
        out["search"] =
            detail::search_reference_descendants_impl(selected_query, c.models, true, start);
        if (out["search"].at("status") != "resolved") {
            out["reason"] = out["search"].at("reason");
            return out;
        }
        out.update({{"status", "resolved"}, {"excluded", out["search"].at("matched")}});
        return out;
    };
    try {
        gate.host_valid = host && object(c, *host).valid;
        if (!*gate.host_valid)
            return evaluate();
        host_parent = parent(object(c, *host));
        gate.host_parent_present = host_parent.has_value();
        if (!*gate.host_parent_present)
            return evaluate();
        gate.candidate_valid = candidate && object(c, *candidate).valid;
        if (!*gate.candidate_valid)
            return evaluate();
        const auto &node = object(c, *candidate);
        gate.candidate_reference_present = detail::reference_object_present(node);
        if (!*gate.candidate_reference_present)
            return evaluate();
        gate.candidate_primary_flags = node.reference_primary_flags;
        if (!gate.candidate_primary_flags || (*gate.candidate_primary_flags & 0x10000000u))
            return evaluate();
        auto ancestor = parent(node);
        std::unordered_set<std::size_t> visited;
        while (ancestor) {
            require(visited.insert(*ancestor).second, "cycle_in_candidate_host_flag_query");
            const auto &m = object(c, *ancestor);
            out["candidate_host_indices"].push_back(*ancestor);
            ReferenceAncestorState state;
            state.valid = m.valid;
            if (m.valid) {
                state.reference_known_absent = !detail::reference_object_present(m);
                if (!state.reference_known_absent)
                    state.reference_primary_flags = m.reference_primary_flags;
            }
            gate.candidate_hosts.push_back(state);
            if (!state.valid || state.reference_known_absent || !state.reference_primary_flags ||
                (*state.reference_primary_flags & 0x10000000u))
                return evaluate();
            ancestor = parent(m);
        }
        gate.complete_candidate_host_chain = true;
        return evaluate();
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
