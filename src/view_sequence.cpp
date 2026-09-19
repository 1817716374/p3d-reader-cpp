#include "internal.hpp"
#include <p3d/view_sequence.hpp>
#include <numeric>
#include <set>

namespace p3d {
namespace {
Json entry(std::uint64_t id, const char *source, std::size_t index) {
    return {{"id", id}, {"source", source}, {"source_index", index}};
}
std::uint64_t source_id(const Json &value) {
    require(value.is_number_unsigned() ||
                (value.is_number_integer() && value.get<std::int64_t>() >= 0),
            "view_sequence_id_requires_unsigned_integer");
    return value.get<std::uint64_t>();
}
} // namespace

Json resolve_view_link_sequence(const Json &saved, const ViewSequenceContext &context) {
    Json out = {{"status", "unresolved"},
                {"scope", "explicit_model_link_state"},
                {"removed_entries", Json::array()},
                {"added_link_indices", Json::array()}};
    try {
        std::vector<Json> entries;
        bool allocated = context.previous_sequence.has_value();
        if (allocated)
            for (std::size_t i = 0; i < context.previous_sequence->size(); ++i)
                entries.push_back(entry((*context.previous_sequence)[i], "previous_sequence", i));
        out["reconciliation"] = "not_requested";
        if (!saved.is_null()) {
            require(saved.at("encoding") == "view_link_sequence" && !saved.contains("decode_error"),
                    "decoded_view_link_sequence_required");
            const auto flag = source_id(saved.at("sequence_flag"));
            require(flag <= 65535, "view_sequence_flag_out_of_range");
            out["sequence_flag_nonzero"] = flag != 0;
            const auto &ids = saved.at("entry_ids");
            require(ids.is_array() && source_id(saved.at("entry_count")) == ids.size(),
                    "view_sequence_count_mismatch");
            if (ids.empty()) {
                // The loader changes the saved flag but does not allocate,
                // clear or reconcile a sequence when the record count is zero.
                out["reconciliation"] = "skipped_empty_record";
            } else {
                allocated = true;
                std::vector<Json> loaded;
                for (std::size_t i = 0; i < ids.size(); ++i)
                    loaded.push_back(entry(source_id(ids[i]), "saved_sequence", i));
                loaded.insert(loaded.end(), entries.begin(), entries.end());
                entries = std::move(loaded); // Native range-insert at begin(), not assign().
                require(context.links_complete, "complete_native_link_list_required");
                out["reconciliation"] = "skipped_empty_link_list";
                if (!context.links.empty()) {
                    std::map<std::uint64_t, std::size_t> first;
                    std::vector<bool> retained(entries.size(), false);
                    for (std::size_t i = 0; i < entries.size(); ++i) {
                        const auto id = source_id(entries[i].at("id"));
                        first.emplace(id, i);
                        retained[i] = id == 0;
                    }
                    std::vector<Json> added;
                    for (std::size_t i = 0; i < context.links.size(); ++i) {
                        const auto &link = context.links[i];
                        if (!link.present || link.id == 0 || link.excluded_from_reconciliation)
                            continue;
                        const auto found = first.find(link.id);
                        if (found != first.end())
                            retained[found->second] = true;
                        else {
                            // Search only the original sequence: duplicate new
                            // links are added repeatedly in original list order.
                            added.push_back(entry(link.id, "model_link", i));
                            out["added_link_indices"].push_back(i);
                        }
                    }
                    bool prepend = false;
                    if (!added.empty()) {
                        if (entries.size() > 1)
                            prepend = entries.back().at("id") == 0;
                        else {
                            require(context.current_model_last.has_value(),
                                    "current_model_position_required");
                            prepend = *context.current_model_last;
                        }
                    }
                    // The threshold includes null and excluded native list slots.
                    const bool remove = entries.size() + added.size() > context.links.size() + 1;
                    out["deletion_threshold_exceeded"] = remove;
                    if (remove) {
                        for (std::size_t i = entries.size(); i-- > 0;)
                            if (!retained[i])
                                out["removed_entries"].push_back(entries[i]);
                        std::vector<Json> kept;
                        for (std::size_t i = 0; i < entries.size(); ++i)
                            if (retained[i])
                                kept.push_back(std::move(entries[i]));
                        entries = std::move(kept);
                    }
                    entries.insert(prepend ? entries.begin() : entries.end(), added.begin(),
                                   added.end());
                    out["reconciliation"] = "performed";
                }
            }
        }
        if (!allocated && context.initialize_default) {
            require(context.links_complete, "complete_native_link_list_required");
            if (!context.links.empty())
                require(context.current_model_last.has_value(), "current_model_position_required");
            const bool last = context.current_model_last.value_or(false);
            if (!last)
                entries.push_back(entry(0, "current_model", 0));
            for (std::size_t i = 0; i < context.links.size(); ++i) {
                require(context.links[i].present, "null_link_in_native_default_sequence");
                entries.push_back(entry(context.links[i].id, "model_link", i));
            }
            if (last)
                entries.push_back(entry(0, "current_model", 0));
            out["default_initialized"] = true;
            allocated = true;
        } else
            out["default_initialized"] = false;
        Json ids = Json::array();
        for (const auto &value : entries)
            ids.push_back(value.at("id"));
        out["sequence_allocated"] = allocated;
        out["entry_ids"] = allocated ? std::move(ids) : Json();
        out["entries"] = std::move(entries);
        out["status"] = "resolved";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json order_view_link_candidates(const std::vector<std::uint64_t> &ids,
                                const std::vector<ViewSequenceCandidate> &candidates) {
    std::vector<std::size_t> order(candidates.size());
    std::iota(order.begin(), order.end(), 0);
    std::map<std::uint64_t, std::set<std::size_t>> by_id;
    std::set<std::size_t> current;
    auto insert = [&](std::size_t pos) {
        const auto &c = candidates[order[pos]];
        by_id[c.link_id].insert(pos);
        if (c.is_current_model)
            current.insert(pos);
    };
    auto erase = [&](std::size_t pos) {
        const auto &c = candidates[order[pos]];
        by_id.at(c.link_id).erase(pos);
        if (c.is_current_model)
            current.erase(pos);
    };
    for (std::size_t i = 0; i < order.size(); ++i)
        insert(i);
    Json matches = Json::array();
    std::size_t next = 0;
    if (order.size() > 1)
        for (std::size_t i = 0; i < ids.size(); ++i) {
            const auto found = by_id.find(ids[i]);
            if (ids[i] != 0 && found == by_id.end())
                continue;
            const auto &positions = ids[i] == 0 ? current : found->second;
            const auto at = positions.lower_bound(next);
            if (at == positions.end())
                continue;
            const auto pos = *at;
            matches.push_back({{"sequence_index", i},
                               {"candidate_index", order[pos]},
                               {"from_position", pos},
                               {"to_position", next}});
            if (pos != next) {
                erase(pos);
                erase(next);
                std::swap(order[pos], order[next]);
                insert(pos);
                insert(next);
            }
            ++next;
        }
    return {{"status", "ordered"},
            {"scope", "explicit_collected_model_candidates"},
            {"candidate_indices", order},
            {"matches", matches}};
}
} // namespace p3d
