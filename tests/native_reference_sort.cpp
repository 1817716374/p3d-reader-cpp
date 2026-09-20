#include "internal.hpp"
#include "native_reference_sort.hpp"
#include <future>
#include <numeric>
#include <random>

unsigned native_reference_sort_tests() {
    using namespace p3d;
    using Entry = NativeViewLinkSortEntry;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto indices = [](const Json &r) {
        return r.at("link_indices").get<std::vector<std::size_t>>();
    };
    check(indices(order_native_view_links({})).empty(), "empty sort needs no comparison fields");
    check(indices(order_native_view_links({{false, {}, {}}})) == std::vector<std::size_t>{0},
          "single null slot is preserved because the native sorter never compares it");
    auto r = order_native_view_links({{true, 2, {}}, {true, 2, 3}});
    check(r.at("status") == "unresolved" && !r.contains("link_indices"),
          "needed unknown key does not publish a partial permutation");
    check(order_native_view_links({{true, {}, 0}, {true, 1, 0}}).at("status") == "unresolved",
          "unknown kind cannot be inferred from key");
    check(order_native_view_links({{false, 2, 0}, {true, 2, 1}}).at("reason") ==
              "null_link_in_native_sort_comparison",
          "visited null comparator input is explicit");
    std::vector<Entry> mixed = {
        {true, INT32_MAX, {}}, {true, 2, UINT32_MAX},  {true, -1, {}}, {true, 2, 0},
        {true, INT32_MIN, {}}, {true, 2, 0x80000000u}, {true, 1, {}}};
    check(indices(order_native_view_links(mixed)) ==
              std::vector<std::size_t>({4, 2, 6, 3, 5, 1, 0}),
          "kind comparison is signed and same-kind2 key comparison is unsigned");
    check(indices(order_native_view_links({{true, 1, {}}, {true, 1, {}}, {true, 1, {}}})) ==
              std::vector<std::size_t>({0, 1, 2}),
          "equal nonreference kinds need no secondary keys");
    check(indices(order_native_view_links({{true, 2, {}}, {true, 1, {}}})) ==
              std::vector<std::size_t>({1, 0}),
          "different kinds do not read an unused kind2 secondary key");
    for (std::size_t size : {0u, 1u, 31u, 32u, 33u, 40u, 41u, 42u, 127u, 512u}) {
        std::vector<Entry> same(size, {true, 2, 4});
        detail::NativeReferenceSortStats stats;
        auto order = detail::native_reference_sort_indices(same, &stats);
        std::vector<std::size_t> identity(size);
        std::iota(identity.begin(), identity.end(), 0);
        check(order == identity,
              "all-equal input retains native equal-band order at every size boundary");
        check((stats.partitions > 0) == (size > 32) && (stats.ninthers > 0) == (size > 41),
              "insertion and nine-sample thresholds use exact native inclusive-last distance");
    }
    // Independently traced one-partition permutations for alternating keys.
    // Both exhausted-side rotations distinguish native order from stable sort.
    std::vector<Entry> alternating;
    for (unsigned i = 0; i < 33; ++i)
        alternating.push_back({true, 2, 1 - i % 2});
    const std::vector<std::size_t> upward = {15, 1,  9,  3,  13, 5,  11, 7,  17, 19, 21,
                                             23, 25, 27, 29, 31, 16, 0,  18, 2,  20, 4,
                                             22, 6,  24, 8,  26, 10, 28, 12, 30, 14, 32};
    detail::NativeReferenceSortStats stats;
    check(detail::native_reference_sort_indices(alternating, &stats) == upward &&
              stats.rotate_up == 8,
          "right-side zeros rotate through the equal band in the exact native permutation");
    for (auto &e : alternating)
        e.secondary_key = 1 - *e.secondary_key;
    const std::vector<std::size_t> downward = {0,  18, 2,  20, 4,  22, 6,  24, 8,  26, 10,
                                               28, 12, 30, 14, 32, 16, 1,  3,  5,  7,  9,
                                               11, 13, 15, 25, 21, 27, 19, 29, 23, 31, 17};
    check(detail::native_reference_sort_indices(alternating, &stats) == downward &&
              stats.rotate_down == 8,
          "left-side ones rotate through the equal band with the native tie order");
    std::mt19937 random(152);
    for (unsigned trial = 0; trial < 120; ++trial) {
        const unsigned size = trial < 10 ? trial : 33 + random() % 2500;
        std::vector<std::uint32_t> keys(size);
        std::iota(keys.begin(), keys.end(), 0);
        std::shuffle(keys.begin(), keys.end(), random);
        std::vector<Entry> input;
        for (auto key : keys)
            input.push_back({true, 2, key});
        std::vector<std::size_t> expected(size);
        std::iota(expected.begin(), expected.end(), 0);
        std::sort(expected.begin(), expected.end(),
                  [&](auto a, auto b) { return keys[a] < keys[b]; });
        check(detail::native_reference_sort_indices(input) == expected,
              "native permutation agrees with independently determined unique-key rank");
        auto heap_order = detail::native_reference_sort_indices(input, &stats, 0);
        check(heap_order == expected && stats.heap_ranges == (size > 32 ? 1u : 0u),
              "heap fallback and small-range precedence retain unique-key rank");
    }
    auto saved = [](std::initializer_list<std::uint64_t> ids) {
        return Json{{"encoding", "view_link_sequence"},
                    {"sequence_flag", 2},
                    {"entry_count", ids.size()},
                    {"entry_ids", std::vector<std::uint64_t>(ids)}};
    };
    ViewSequenceContext context;
    context.links_complete = true;
    context.current_model_last = false;
    context.links = {{20}, {10}, {30}};
    const std::vector<Entry> input = {{true, 2, 2}, {true, 2, 1}, {true, 2, 3}};
    r = prepare_native_view_sequence(saved({0}), context, input);
    check(r.at("sequence").at("entry_ids") == Json::array({0, 20, 10, 30}) &&
              r.at("link_sort").at("link_indices") == Json::array({1, 0, 2}),
          "saved sequence reconciliation consumes original link order before sorting");
    r = prepare_native_view_sequence(nullptr, context, input);
    check(r.at("sequence").at("entry_ids") == Json::array({0, 10, 20, 30}) &&
              r.at("sequence").at("entries")[1].at("source_index") == 1,
          "default sequence uses sorted links but preserves original source indices");
    context.current_model_last = true;
    check(prepare_native_view_sequence(nullptr, context, input).at("sequence").at("entry_ids") ==
              Json::array({10, 20, 30, 0}),
          "current-model-last flag applies after native link sorting");
    context.current_model_last.reset();
    context.previous_sequence = std::vector<std::uint64_t>{};
    check(prepare_native_view_sequence(nullptr, context, input)
              .at("sequence")
              .at("entry_ids")
              .empty(),
          "allocated empty sequence prevents default construction and avoids position flag");
    context.previous_sequence.reset();
    context.current_model_last = false;
    r = prepare_native_view_sequence(saved({}), context, input);
    check(r.at("sequence").at("default_initialized") == true &&
              r.at("sequence").at("sequence_flag_nonzero") == true,
          "empty saved record permits default initialization and retains its flag");
    context.links.clear();
    context.initialize_default = true;
    context.current_model_last.reset();
    check(prepare_native_view_sequence(nullptr, context, {})
                  .at("sequence")
                  .at("sequence_allocated") == false,
          "native outer call does not construct a current-model-only default for an empty list");
    context.links = {{1, false}};
    r = prepare_native_view_sequence(nullptr, context, {{false, {}, {}}});
    check(r.at("status") == "unresolved",
          "single null slot passes sorting but cannot be used for default construction");
    context.previous_sequence = std::vector<std::uint64_t>{9};
    check(prepare_native_view_sequence(nullptr, context, {{false, {}, {}}})
                  .at("sequence")
                  .at("entry_ids") == Json::array({9}),
          "single null slot is harmless when existing sequence avoids default builder");
    check(prepare_native_view_sequence(nullptr, context, {}).at("reason") ==
              "native_link_sort_input_count_mismatch",
          "sort descriptors must correspond to the same native list");
    check(prepare_native_view_sequence(nullptr, context, {{true, 2, 0}}).at("reason") ==
              "native_link_sort_presence_mismatch",
          "presence disagreement is not a second independent list");
    context.links_complete = false;
    check(prepare_native_view_sequence(nullptr, context, {{false, {}, {}}}).at("status") ==
              "unresolved",
          "composition cannot order an unknown list suffix");
    const auto expected = order_native_view_links(alternating);
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(std::async(std::launch::async,
                                  [alternating] { return order_native_view_links(alternating); }));
    for (auto &job : jobs)
        check(job.get() == expected, "native sorting has no shared mutable state");
    return checks;
}
