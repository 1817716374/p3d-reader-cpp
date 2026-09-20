#include "internal.hpp"
#include <p3d/reference_search.hpp>
#include <p3d/reference_loading.hpp>
#include <future>
#include <random>

unsigned reference_nesting_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto put = [](Bytes &b, std::size_t at, std::uint64_t value, unsigned n) {
        for (unsigned i = 0; i < n; ++i)
            b.at(at + i) = std::uint8_t(value >> (8 * i));
    };
    auto record = [&](std::uint16_t depth, bool legacy, unsigned flags = 0) {
        Bytes bytes(legacy ? 348 : 372);
        put(bytes, 4, 13, 2);
        put(bytes, 8, (bytes.size() - 4) / 2, 4);
        put(bytes, 12, (bytes.size() - 4) / 2, 4);
        if (!legacy)
            put(bytes, 160, 1, 4);
        const auto matrix = legacy ? 212 : 220;
        for (auto at : {matrix, matrix + 32, matrix + 64, legacy ? 284 : 292})
            put(bytes, at, UINT64_C(0x3ff0000000000000), 8);
        put(bytes, legacy ? 340 : 364, depth, 2);
        put(bytes, legacy ? 52 : 60, flags, 4);
        return parse_native(bytes).at(0);
    };
    ReferenceSearchContext c;
    auto root = ReferenceSearchModel{};
    root.object_dispatch = ReferenceObjectDispatch::model;
    c.models = {root};
    check(reference_nesting_depth(c, 0).at("remaining_depth") == 1 &&
              reference_nesting_depth(c, std::nullopt, 1).at("remaining_depth") == 1,
          "ordinary model or null reference gives native depth one");
    for (bool legacy : {false, true})
        for (unsigned depth : {0u, 1u, 31u, 65535u}) {
            const auto r = record(depth, legacy, 0x10000080);
            const auto &source = r.at("reference_input").at("nesting_inputs");
            check(
                source.at("nest_depth") == depth &&
                    source.at("source_offset") == (legacy ? 340 : 364),
                "current and upgraded legacy nest depth preserve uint16 value and original offset");
            const auto node = initial_reference_graph_node(r, 0);
            check(node.object_dispatch == ReferenceObjectDispatch::reference && node.parent_known &&
                      node.parent == 0 && node.root_present == false &&
                      node.reference_present == true && node.native_kind == 2 &&
                      node.active_list_known_absent && node.active_links.empty() &&
                      node.reference_runtime_flags == 0 &&
                      node.reference_primary_flags == 0x10000080 &&
                      node.reference_nest_depth == depth,
                  "record builds ordinary initial reference node without inventing its connected "
                  "target");
            c.models = {root, node};
            check(reference_nesting_depth(c, 1).at("remaining_depth") == depth,
                  "parent ordinary model terminates without consuming another reference level");
        }
    c.models = {root, initial_reference_graph_node(record(5, false), 0),
                initial_reference_graph_node(record(10, false), 1),
                initial_reference_graph_node(record(8, false), 2)};
    auto r = reference_nesting_depth(c, 3);
    check(r.at("remaining_depth") == 3 && r.at("reference_path") == Json::array({3, 2, 1}),
          "each reference ancestor reduces its inherited allowance by one");
    check(reference_nesting_depth(c, 3, 3).at("remaining_depth") == 1,
          "positive cap applies separately at each level before recursive decrement");
    check(reference_nesting_depth(c, 3, 1).at("remaining_depth") == -1,
          "native signed remaining depth is preserved below zero");
    check(reference_nesting_depth(c, 3, 0).at("remaining_depth") == 3 &&
              reference_nesting_depth(c, 3, INT32_MIN).at("remaining_depth") == 3,
          "zero and negative cap do not impose an extra bound");
    c.models[2].reference_nest_depth = 0;
    check(reference_nesting_depth(c, 3).at("remaining_depth") == -1,
          "an intermediate zero depth also restricts its descendants");
    c.models[1].valid = false;
    check(reference_nesting_depth(c, 2).at("remaining_depth") == 0,
          "invalid parent terminates before its depth and reference presence are read");
    c.models[1] = {};
    c.models[1].reference_present = false;
    check(reference_nesting_depth(c, 2).at("remaining_depth") == 0,
          "known parent without reference terminates even with unknown virtual kind");
    c.models[1].reference_present.reset();
    check(reference_nesting_depth(c, 2).at("status") == "unresolved",
          "unknown ancestor reference cannot be assumed absent even at local zero");
    c.models[1] = initial_reference_graph_node(record(5, false), 0);
    c.models[2].parent_known = false;
    check(reference_nesting_depth(c, 2).at("status") == "unresolved",
          "unknown parent relationship cannot be inferred from nesting allowance");
    c.models[2].parent_known = true;
    c.models[1].parent = 3;
    check(reference_nesting_depth(c, 3).at("reason") == "cycle_in_reference_nesting_depth",
          "cycle remains unresolved instead of producing a truncated chain depth");
    check(reference_nesting_depth(c, 99).at("status") == "unresolved",
          "out-of-range object identity is not a null reference");
    auto p = reference_link_loading_policy(c, 99, true);
    check(p.at("status") == "resolved" && p.at("require_source_bit14") == false &&
              p.at("process_view_sequence") == true && !p.contains("nesting"),
          "force-input policy does not inspect unused graph or nesting context");
    check(reference_link_loading_policy(c, 3).at("status") == "unresolved",
          "unknown effective depth does not silently enable ordinary collection");
    c.models = {root, initial_reference_graph_node(record(0, false), 0)};
    p = reference_link_loading_policy(c, 1);
    check(p.at("require_source_bit14") == true && p.at("process_view_sequence") == false,
          "exhausted depth requires source bit14 and skips that collection pass's view sequence");
    ReferenceLoadingDecisionContext loading;
    loading.require_source_bit14 = p.at("require_source_bit14").get<bool>();
    loading.native_input_status = 0;
    loading.host_filter_excluded = false;
    loading.source_runtime_deleted = false;
    loading.descendant_filter_required = false;
    loading.ancestor_repeated = false;
    check(initial_reference_loading_decision(record(7, false), loading).at("loading_state") == 7 &&
              initial_reference_loading_decision(record(7, false, 0x4000), loading)
                      .at("loading_state") == 1,
          "nesting policy composes with record-driven loading without rejecting bit14 exceptions");
    auto unknown = c.models[1];
    unknown.reference_nest_depth.reset();
    c.models[1] = unknown;
    check(reference_nesting_depth(c, 1).at("status") == "unresolved",
          "missing current nesting field is not a zero allowance");
    auto bad = record(1, false);
    bad["element_type"] = 47;
    bool threw = false;
    try {
        (void)initial_reference_graph_node(bad, 0);
    } catch (const std::exception &) {
        threw = true;
    }
    check(threw, "initial reference node refuses other native record types");
    bad = record(1, false);
    bad["reference_target"]["initial_runtime_state"]["status"] = "not_evaluated";
    bad["reference_target"]["initial_loaded_flags"]["primary"]["status"] = "not_evaluated";
    const auto partial = initial_reference_graph_node(bad, 0);
    check(!partial.reference_runtime_flags && !partial.reference_primary_flags &&
              partial.reference_nest_depth == 1,
          "unknown flag words stay optional without losing independent nesting depth");

    std::mt19937 random(0x1552025);
    for (unsigned trial = 0; trial < 80; ++trial) {
        const unsigned count = 1 + random() % 18;
        std::vector<std::uint16_t> depths(count);
        c.models.assign(count + 1, {});
        c.models.back() = root;
        for (unsigned i = 0; i < count; ++i) {
            depths[i] = random() % 12;
            c.models[i].object_dispatch = ReferenceObjectDispatch::reference;
            c.models[i].parent_known = true;
            c.models[i].parent = i + 1;
            c.models[i].reference_nest_depth = depths[i];
        }
        for (std::int32_t cap : {INT32_MIN, -1, 0, 1, 3, 65535, INT32_MAX}) {
            std::function<int(unsigned)> literal = [&](unsigned i) {
                int n = cap > 0 ? std::min<int>(depths[i], cap) : depths[i];
                return i + 1 == count ? n : std::min(n, literal(i + 1) - 1);
            };
            check(reference_nesting_depth(c, 0, cap).at("remaining_depth") == literal(0),
                  "iterative graph evaluation matches independent literal native recursion");
        }
    }
    c.models.assign(5001, {});
    c.models.back() = root;
    for (unsigned i = 0; i < 5000; ++i) {
        c.models[i].object_dispatch = ReferenceObjectDispatch::reference;
        c.models[i].reference_nest_depth = 65535;
        c.models[i].parent_known = true;
        c.models[i].parent = i + 1;
    }
    r = reference_nesting_depth(c, 0);
    check(r.at("remaining_depth") == 60536 && r.at("reference_path").size() == 5000,
          "deep native parent chain uses an iterative traversal without C++ call-stack exhaustion");
    const auto baseline = reference_link_loading_policy(c, 0, false, 2);
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(std::async(std::launch::async,
                                  [&] { return reference_link_loading_policy(c, 0, false, 2); }));
    for (auto &job : jobs)
        check(job.get() == baseline, "nesting evaluation has no shared mutable graph state");
    return checks;
}
