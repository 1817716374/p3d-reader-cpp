#include "internal.hpp"
#include <p3d/view_sequence.hpp>
#include <future>
#include <numeric>
#include <random>

unsigned view_sequence_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto saved = [](const std::vector<std::uint64_t> &ids, unsigned flag = 0) {
        return Json{{"encoding", "view_link_sequence"},
                    {"sequence_flag", flag},
                    {"entry_count", ids.size()},
                    {"entry_ids", ids}};
    };
    ViewSequenceContext context;
    auto run = [&](std::vector<std::uint64_t> ids) {
        return resolve_view_link_sequence(saved(ids), context);
    };
    check(resolve_view_link_sequence(nullptr, context).at("entry_ids").is_null(),
          "absent sequence is not silently allocated");
    context.previous_sequence = std::vector<std::uint64_t>{};
    check(resolve_view_link_sequence(nullptr, context).at("entry_ids") == Json::array(),
          "allocated empty sequence differs from absent pointer");
    context.previous_sequence = std::vector<std::uint64_t>{9};
    check(run({}).at("entry_ids") == Json::array({9}),
          "empty record preserves an existing sequence without requiring link enumeration");
    context.previous_sequence.reset();
    context.links_complete = true;
    check(run({7, 0}).at("entry_ids") == Json::array({7, 0}),
          "empty native link list does not delete saved stale links");
    context.links = {{1}, {2}};
    context.current_model_last = false;
    auto result = run({0, 99});
    check(result.at("entry_ids") == Json::array({0, 1, 2}) &&
              result.at("removed_entries")[0].at("id") == 99,
          "excess sequence removes unmatched nonzero IDs and appends new links");
    check(result.at("entries")[1].at("source") == "model_link" &&
              result.at("entries")[1].at("source_index") == 0,
          "added sequence entries retain native list provenance");
    check(run({99, 0}).at("entry_ids") == Json::array({1, 2, 0}),
          "trailing current-model marker causes new links to be prepended");
    context.links = {{1}};
    check(run({1, 99}).at("entry_ids") == Json::array({1, 99}),
          "unmatched IDs remain when size threshold is not exceeded");
    check(run({1, 1, 0}).at("entry_ids") == Json::array({1, 0}),
          "native matching marks only the first stored duplicate before excess deletion");
    context.links = {{1}, {0, false}, {5, true, true}};
    check(run({1, 99, 0}).at("entry_ids") == Json::array({1, 99, 0}),
          "null and excluded slots still contribute to native deletion threshold");
    context.links = {{1}, {1}};
    context.current_model_last = true;
    check(run({0}).at("entry_ids") == Json::array({1, 1, 0}),
          "new duplicate links are not deduplicated against other additions");
    context.current_model_last.reset();
    check(run({0}).at("status") == "unresolved" && !run({0}).contains("entry_ids"),
          "unknown insertion side does not produce a guessed final sequence");
    check(run({1}).at("status") == "resolved",
          "unused position flag is not required when no links are added");
    context.links_complete = false;
    check(run({1}).at("status") == "unresolved",
          "partial native list cannot prove sequence reconciliation");
    context.links_complete = true;
    context.links.clear();
    context.previous_sequence = std::vector<std::uint64_t>{8, 9};
    check(run({7}).at("entry_ids") == Json::array({7, 8, 9}),
          "native record loader inserts at beginning rather than replacing existing contents");
    context.previous_sequence.reset();
    context.initialize_default = true;
    context.current_model_last = false;
    context.links = {{8}, {0}, {8, true, true}};
    check(resolve_view_link_sequence(nullptr, context).at("entry_ids") == Json::array({0, 8, 0, 8}),
          "default builder keeps zeros, duplicates and excluded link IDs in supplied order");
    context.current_model_last = true;
    check(run({}).at("entry_ids") == Json::array({8, 0, 8, 0}),
          "zero-count record can be followed by explicitly requested default initialization");
    context.links[1].present = false;
    check(resolve_view_link_sequence(nullptr, context).at("status") == "unresolved",
          "default builder refuses native null dereference instead of filtering a slot");
    context.links.clear();
    auto bad = saved({1});
    bad["entry_count"] = 2;
    check(resolve_view_link_sequence(bad, context).at("status") == "unresolved",
          "saved sequence count must match decoded source entries");
    bad = saved({1});
    bad["entry_ids"][0] = -1;
    check(resolve_view_link_sequence(bad, context).at("status") == "unresolved",
          "negative JSON source ID does not wrap into a valid unsigned ID");
    bad = saved({1});
    bad["decode_error"] = "truncated";
    check(resolve_view_link_sequence(bad, context).at("status") == "unresolved",
          "failed source decoder is not treated as a complete saved sequence");
    result = resolve_view_link_sequence(saved({UINT64_MAX, 0}, 65535), context);
    check(result.at("entry_ids") == Json::array({UINT64_MAX, std::uint64_t(0)}) &&
              result.at("sequence_flag_nonzero") == true,
          "full unsigned IDs and raw nonzero sequence flag survive loading");

    std::vector<ViewSequenceCandidate> candidates = {{1, false}, {2, false}, {3, false}};
    check(order_view_link_candidates({3}, candidates).at("candidate_indices") ==
              Json::array({2, 1, 0}),
          "native ordering swaps positions instead of stable moving the matched candidate");
    candidates = {{0, false}, {5, true}, {5, false}};
    check(order_view_link_candidates({0, 5}, candidates).at("candidate_indices") ==
              Json::array({1, 2, 0}),
          "zero sequence ID uses current-model identity, nonzero IDs use candidate link ID");
    candidates = {{5, false}, {5, false}, {0, true}, {8, false}};
    result = order_view_link_candidates({5, 99, 5, 0, 5}, candidates);
    check(result.at("candidate_indices") == Json::array({0, 1, 2, 3}) &&
              result.at("matches").size() == 3,
          "duplicates consume distinct suffix candidates and absent IDs do not advance cursor");
    check(order_view_link_candidates({0}, {{0, true}}).at("matches").empty() &&
              order_view_link_candidates({1}, {}).at("candidate_indices").empty(),
          "native zero-or-one candidate case performs no selection loop");

    // Independent literal vector/linear-search oracle for the optimized indexes.
    std::mt19937_64 random(0x14447133);
    for (unsigned trial = 0; trial < 800; ++trial) {
        ViewSequenceContext c;
        c.links_complete = true;
        c.current_model_last = bool(random() & 1);
        c.initialize_default = true;
        for (std::size_t i = 0, n = random() % 9; i < n; ++i)
            c.links.push_back({random() % 6, bool(random() % 5), bool(random() % 4 == 0)});
        std::vector<std::uint64_t> input;
        for (std::size_t i = 0, n = 1 + random() % 9; i < n; ++i)
            input.push_back(random() % 7);
        c.previous_sequence = std::vector<std::uint64_t>{};
        for (std::size_t i = 0, n = random() % 4; i < n; ++i)
            c.previous_sequence->push_back(random() % 7);
        auto expected = *c.previous_sequence;
        expected.insert(expected.begin(), input.begin(), input.end());
        if (!c.links.empty()) {
            auto unmatched = expected;
            std::vector<std::uint64_t> added;
            for (const auto &link : c.links) {
                if (!link.present || !link.id || link.excluded_from_reconciliation)
                    continue;
                auto found = std::find(expected.begin(), expected.end(), link.id);
                if (found == expected.end())
                    added.push_back(link.id);
                else
                    unmatched[std::size_t(found - expected.begin())] = 0;
            }
            const bool before = expected.size() > 1 ? expected.back() == 0 : *c.current_model_last;
            if (expected.size() + added.size() > c.links.size() + 1)
                for (std::size_t i = unmatched.size(); i-- > 0;)
                    if (unmatched[i])
                        expected.erase(expected.begin() + i);
            expected.insert(before ? expected.begin() : expected.end(), added.begin(), added.end());
        }
        const auto source = saved(input);
        const auto unchanged = source.dump();
        result = resolve_view_link_sequence(source, c);
        check(result.at("status") == "resolved" && result.at("entry_ids") == expected &&
                  source.dump() == unchanged,
              "indexed reconciliation agrees with literal native vector operations");
        candidates.clear();
        for (std::size_t i = 0, n = random() % 15; i < n; ++i)
            candidates.push_back({random() % 7, bool(random() % 4 == 0)});
        std::vector<std::size_t> ordered(candidates.size());
        std::iota(ordered.begin(), ordered.end(), 0);
        if (ordered.size() > 1) {
            std::size_t next = 0;
            for (auto id : input) {
                for (std::size_t i = next; i < ordered.size(); ++i) {
                    const auto &candidate = candidates[ordered[i]];
                    if (id == 0 ? candidate.is_current_model : candidate.link_id == id) {
                        std::swap(ordered[next++], ordered[i]);
                        break;
                    }
                }
            }
        }
        check(order_view_link_candidates(input, candidates).at("candidate_indices") == ordered,
              "position indexes preserve native suffix search and swap semantics");
    }
    context.links = {{1}, {2}};
    context.current_model_last = false;
    const auto source = saved({0, 1});
    const auto expected = resolve_view_link_sequence(source, context);
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(std::async(std::launch::async,
                                  [&] { return resolve_view_link_sequence(source, context); }));
    for (auto &job : jobs)
        check(job.get() == expected, "view sequence resolution is call-local and parallel-safe");
    return checks;
}
