#include "internal.hpp"
#include <p3d/proxy_cache.hpp>

unsigned cache_association_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto bytes = [](double value) {
        Bytes b(8);
        std::memcpy(b.data(), &value, 8);
        return b;
    };
    EdgeCacheEntityStateContext state;
    check(compare_native_edge_cache_entity_state({}, state).at("status") == "not_evaluated",
          "unknown lookup is not a missing entity");
    state.entity_found = false;
    check(compare_native_edge_cache_entity_state({}, state).at("state_matches") == false,
          "missing entity short circuits flags, saved time and current time");
    state.entity_found = true;
    check(compare_native_edge_cache_entity_state(bytes(1.), state).at("reason") ==
              "entity_runtime_flags_required",
          "found entity needs actual runtime flags");
    for (unsigned bit = 0; bit < 32; ++bit) {
        state.runtime_flags = std::uint32_t(1) << bit;
        state.last_modified_milliseconds.reset();
        const auto value = compare_native_edge_cache_entity_state({}, state);
        check((bit == 3 || bit == 17)
                  ? value.at("status") == "resolved" && value.at("state_matches") == false
                  : value.at("status") == "not_evaluated",
              "only native exclusion flag bits short circuit the time gate");
    }
    state.runtime_flags = 0;
    state.last_modified_milliseconds = 1234.25;
    for (unsigned n = 0; n <= 12; ++n)
        if (n != 8)
            check(compare_native_edge_cache_entity_state(Bytes(n), state).at("status") ==
                      "not_evaluated",
                  "saved time must have the exact native width");
    const auto matched = compare_native_edge_cache_entity_state(bytes(1234.25), state);
    check(matched.at("state_matches") == true &&
              matched.at("display_test_status") == "not_evaluated",
          "matching state does not claim display eligibility");
    check(compare_native_edge_cache_entity_state(bytes(1234.75), state).at("state_matches") ==
              false,
          "fractional milliseconds are not truncated to the SDK integer return");
    state.last_modified_milliseconds = -0.;
    check(compare_native_edge_cache_entity_state(bytes(0.), state).at("state_matches") == true,
          "signed zero comparison is numeric, not bitwise");
    const auto inf = std::numeric_limits<double>::infinity();
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    for (const auto pair : std::vector<std::pair<double, double>>{
             {inf, inf}, {-inf, -inf}, {inf, -inf}, {nan, nan}, {nan, 0.}, {0., nan}}) {
        state.last_modified_milliseconds = pair.second;
        check(
            compare_native_edge_cache_entity_state(bytes(pair.first), state).at("state_matches") ==
                (!std::isnan(pair.first) && !std::isnan(pair.second) && pair.first == pair.second),
            "native ordered double equality handles NaN and infinity");
    }

    auto input = [](const std::vector<std::vector<std::uint32_t>> &links) {
        Json source = {{"status", "decoded"},
                       {"reference_directory", Json::array({Json::object()})},
                       {"associations", Json::array()}};
        for (std::size_t i = 0; i < links.size(); ++i) {
            Json row = {{"association_index", i},
                        {"reference_directory_index", 0},
                        {"entities", Json::array({{{"entity_id", 7}}})},
                        {"links", Json::array()}};
            for (const auto target : links[i])
                row["links"].push_back({{"association_index", target}});
            source["associations"].push_back(std::move(row));
        }
        return source;
    };
    auto contexts = [](std::size_t n) {
        std::vector<EdgeCacheAssociationTarget> result(n);
        for (std::size_t i = 0; i < n; ++i) {
            result[i].created = true;
            result[i].reference_identity = 1;
            result[i].entity_identities = {100 + i};
            result[i].entity_identities_complete = true;
        }
        return result;
    };
    const auto chain = input({{1}, {2}, {1}});
    auto target = contexts(3);
    const auto original = chain;
    auto result = select_native_edge_cache_associations(chain, target);
    check(result.at("status") == "resolved" &&
              result.at("selected_source_indices") == Json({0, 1, 2}) &&
              result.at("retained_source_indices") == Json({0, 1, 2}) &&
              result.at("cache_validity_status") == "not_evaluated" && chain == original,
          "forward references and cycles are allowed without mutating source or claiming validity");
    result = select_native_edge_cache_associations(input({{1}, {1}}), contexts(2));
    check(result.at("selected_source_indices") == Json({0}) &&
              result.at("retained_source_indices") == Json({0, 1}) &&
              result.at("entries")[1].at("reason") == "link_count_mismatch" &&
              result.at("entries")[1].at("effective_link_indices").empty(),
          "failed group can remain retained as another group's target with its own links cleared");
    result = select_native_edge_cache_associations(input({{1, 1}, {}}), contexts(2));
    check(result.at("selected_source_indices") == Json({1}) &&
              result.at("entries")[0].at("unique_nonself_targets") == 1 &&
              result.at("entries")[0].at("effective_link_indices").empty(),
          "duplicate target cardinality rejects the whole source group");
    result =
        select_native_edge_cache_associations(input({{1, UINT32_MAX, 2}, {}, {}}), contexts(3));
    check(result.at("entries")[0].at("examined_links") == 2 &&
              result.at("entries")[0].at("effective_link_indices").empty(),
          "missing target stops lookup and clears the partially constructed set");
    target = contexts(3);
    target[1].created = false;
    target[1].reference_identity.reset();
    target[1].entity_identities_complete = false;
    result = select_native_edge_cache_associations(chain, target);
    check(result.at("selected_source_indices").empty() &&
              result.at("retained_source_indices").empty(),
          "uncreated association needs no identity and rejects incoming links");
    target = contexts(3);
    target[1] = target[0];
    result = select_native_edge_cache_associations(input({{}, {2}, {}}), target);
    check(result.at("selected_source_indices") == Json({0, 2}) &&
              result.at("retained_source_indices") == Json({0, 1, 2}) &&
              result.at("entries")[1].at("selected_source_index") == 0 &&
              result.at("entries")[1].at("effective_link_indices") == Json({2}) &&
              result.at("entries")[0].at("effective_link_indices").empty(),
          "equivalent group keeps the first representative without merging different link sets");
    result = select_native_edge_cache_associations(input({{1}, {}, {}}), target);
    check(result.at("selected_source_indices") == Json({0, 2}) &&
              result.at("entries")[0].at("effective_link_indices") == Json({1}),
          "distinct created objects with equivalent group keys are not treated as self references");
    target[1].reference_identity = 2;
    check(select_native_edge_cache_associations(input({{}, {}, {}}), target)
                  .at("selected_source_indices") == Json({0, 1, 2}),
          "same entity under different actual references forms a different group");
    auto ordered = input({{}, {}, {}});
    for (auto &row : ordered["associations"])
        row["entities"].push_back({{"entity_id", 7}});
    target = contexts(3);
    target[0].entity_identities = {2, 3};
    target[1].entity_identities = {3, 2};
    target[2].entity_identities = {2, 2};
    check(select_native_edge_cache_associations(ordered, target).at("selected_source_indices") ==
              Json({0, 1, 2}),
          "entity order and repeated entity occurrences participate in group equivalence");
    target[2] = target[0];
    check(select_native_edge_cache_associations(ordered, target).at("selected_source_indices") ==
              Json({0, 1}),
          "identical ordered entity sequences reuse only the native group representative");
    target = contexts(3);
    target[1].created.reset();
    check(select_native_edge_cache_associations(chain, target).at("reason") ==
              "association_creation_result_required",
          "unknown creation never silently becomes false");
    target = contexts(3);
    target[1].reference_identity.reset();
    check(select_native_edge_cache_associations(chain, target).at("status") == "not_evaluated",
          "known creation requires actual reference identity");
    target = contexts(3);
    target[1].entity_identities.clear();
    check(select_native_edge_cache_associations(chain, target).at("status") == "not_evaluated",
          "missing entity identities are not replaced by source IDs");
    check(select_native_edge_cache_associations(chain, {}).at("status") == "not_evaluated",
          "all source positions need creation outcomes");
    auto invalid = chain;
    invalid["associations"][0]["association_index"] = 1;
    check(select_native_edge_cache_associations(invalid, contexts(3)).at("status") ==
              "not_evaluated",
          "reordered source index is rejected");
    invalid = chain;
    invalid["associations"][0]["links"][0]["association_index"] = -1;
    check(select_native_edge_cache_associations(invalid, contexts(3)).at("status") ==
              "not_evaluated",
          "negative source index is not wrapped to uint32");
    invalid = chain;
    invalid["associations"][0]["reference_directory_index"] = 10;
    check(select_native_edge_cache_associations(invalid, contexts(3)).at("status") ==
              "not_evaluated",
          "caller cannot claim a created group from an absent directory entry");
    ProxyCacheLimits limits;
    limits.max_entries = 8;
    check(select_native_edge_cache_associations(chain, contexts(3), limits).at("status") ==
              "not_evaluated",
          "aggregate group, entity and link budget rejects oversized queries");
    limits.max_entries = 9;
    check(select_native_edge_cache_associations(chain, contexts(3), limits).at("status") ==
              "resolved",
          "exact query budget succeeds");
    check(
        select_native_edge_cache_associations(input({}), {}).at("selected_source_indices").empty(),
        "empty source produces an empty selection");

    // Exhaust every directed three-node topology and all creation masks. For
    // distinct group identities the oracle is simply absence of self edges or
    // missing targets; retained objects are the union of accepted rows/targets.
    for (unsigned graph = 0; graph < 512; ++graph) {
        std::vector<std::vector<std::uint32_t>> adjacency(3);
        for (unsigned from = 0; from < 3; ++from)
            for (unsigned to = 0; to < 3; ++to)
                if (graph & (1u << (from * 3 + to)))
                    adjacency[from].push_back(to);
        const auto data = input(adjacency);
        for (unsigned mask = 0; mask < 8; ++mask) {
            auto runtime = contexts(3);
            Json expected_selected = Json::array(), expected_retained = Json::array();
            unsigned keep = 0;
            for (unsigned from = 0; from < 3; ++from) {
                runtime[from].created = (mask & (1u << from)) != 0;
                const auto edges = (graph >> (from * 3)) & 7;
                if ((mask & (1u << from)) && !(edges & (1u << from)) && !(edges & ~mask)) {
                    expected_selected.push_back(from);
                    keep |= edges | (1u << from);
                }
            }
            for (unsigned i = 0; i < 3; ++i)
                if (keep & (1u << i))
                    expected_retained.push_back(i);
            const auto answer = select_native_edge_cache_associations(data, runtime);
            check(answer.at("status") == "resolved" &&
                      answer.at("selected_source_indices") == expected_selected &&
                      answer.at("retained_source_indices") == expected_retained,
                  "all small directed graphs preserve native selection and ownership semantics");
        }
    }
    return checks;
}
