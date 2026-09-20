#include "internal.hpp"
#include <p3d/reference_search.hpp>
#include <p3d/reference_loading.hpp>
#include <future>

unsigned reference_nesting_tests();
unsigned reference_link_request_tests();
unsigned reference_connection_tests();

unsigned reference_graph_tests() {
    using namespace p3d;
    using Dispatch = ReferenceObjectDispatch;
    unsigned checks =
        reference_nesting_tests() + reference_link_request_tests() + reference_connection_tests();
    auto check = [&](bool ok, const char *s) {
        ++checks;
        require(ok, s);
    };
    ReferenceSearchContext c;
    c.models.resize(5);
    auto &root = c.models[0];
    root.object_dispatch = Dispatch::model;
    root.file = NativeFileReference{u"F", u"F"};
    root.model_id = 7;
    root.active_list_known_absent = true;
    c.models[1].object_dispatch = Dispatch::reference;
    c.models[1].parent_known = true;
    c.models[1].parent = 0;
    c.models[1].reference_primary_flags = 0;
    // The host's connected target root is absent; parent-root lookup must still
    // find model 0. These are independent native virtual queries.
    c.models[1].root_present = false;
    c.models[2].object_dispatch = Dispatch::reference;
    c.models[2].parent_known = true;
    c.models[2].parent = 1;
    c.models[2].reference_primary_flags = 0x10000000;
    c.models[2].reference_runtime_flags = 0x1000;
    c.models[3].object_dispatch = Dispatch::reference;
    c.models[3].parent_known = true;
    c.models[3].parent = 0;
    c.models[4].object_dispatch = Dispatch::model;
    auto r = reference_parent_root(c, 2);
    check(r.at("model_index") == 0 && r.at("parent_path") == Json::array({2, 1, 0}),
          "reference parent-root query follows parent identity, not connected root availability");
    check(reference_parent_root(c, 0).at("model_index") == 0,
          "ordinary model parent-root query returns itself without file or parent data");
    check(reference_parent_root(c, {}).at("model_index").is_null(),
          "known null query has a null parent root");
    c.models[1].valid = false;
    check(reference_parent_root(c, 2).at("model_index").is_null(),
          "invalid parent terminates before unknown root state");
    c.models[1].valid = true;
    c.models[1].parent_known = false;
    check(reference_parent_root(c, 2).at("reason") == "reference_parent_relation_required",
          "missing relation is not a known null parent");
    c.models[1].parent_known = true;
    c.models[1].parent = 2;
    check(reference_parent_root(c, 2).at("reason") == "cycle_in_reference_parent_root_query",
          "parent-root cycle is diagnosed without recursion overflow");
    c.models[1].parent = 0;
    c.models[0].object_dispatch = Dispatch::unknown;
    c.models[0].native_kind = 1;
    check(reference_parent_root(c, 2).at("status") == "unresolved",
          "numeric kind alone does not establish virtual parent-root implementation");
    c.models[0].object_dispatch = Dispatch::model;
    c.models[0].native_kind.reset();
    c.models[1].parent = 99;
    check(reference_parent_root(c, 2).at("reason") == "reference_graph_index_out_of_range",
          "parent object index must address this graph");
    c.models[1].parent = 0;
    ReferenceSearchQuery q;
    q.stored_file_reference = u"F";
    q.model_id = 7;
    c.start = 4;
    c.start_known = false;
    r = evaluate_reference_descendant_filter(q, c, 1, 2);
    check(r.at("excluded") == true && r.at("search_root").at("model_index") == 0 &&
              r.at("search").at("matching_model_index") == 0 &&
              r.at("gate").at("search_required") == true,
          "graph adapter derives gate, host-parent root and candidate runtime flags without "
          "explicit start");
    check(!q.runtime_flags && c.start == 4 && !c.start_known,
          "derived query and search start do not mutate caller inputs");
    ReferenceLoadingDecisionContext decision;
    decision.native_input_status = 0;
    decision.loaded_primary_flags = 0;
    decision.host_filter_excluded = false;
    decision.source_runtime_deleted = false;
    decision.descendant_filter_required = r.at("gate").at("search_required").get<bool>();
    decision.descendant_match = r.at("excluded").get<bool>();
    decision.keep_descendant_match = false;
    check(reference_loading_decision(13, decision).at("loading_state") == 3,
          "graph-derived filter directly supplies the ordinary loading branch");
    r = evaluate_reference_descendant_filter({}, c, 0, 999);
    check(r.at("excluded") == false && !r.at("search_performed").get<bool>(),
          "ordinary root host has no parent and does not inspect an unused candidate index");
    check(evaluate_reference_descendant_filter({}, c, {}, 999).at("excluded") == false,
          "known null host avoids candidate and query reads");
    check(evaluate_reference_descendant_filter({}, c, 1, {}).at("excluded") == false,
          "known null candidate is a negative gate");
    check(evaluate_reference_descendant_filter({}, c, 1, 4).at("excluded") == false,
          "ordinary candidate has no reference object without needing an explicit false field");
    c.models[2].reference_primary_flags = 0;
    c.models[1].reference_primary_flags = 0x10000000;
    r = evaluate_reference_descendant_filter(q, c, 1, 2);
    check(r.at("excluded") == true && r.at("candidate_host_indices") == Json::array({1}),
          "candidate parent flag selects the search while retaining object provenance");
    c.models[2].parent = 4;
    c.models[4].reference_primary_flags = 0x10000000;
    check(evaluate_reference_descendant_filter({}, c, 1, 2).at("excluded") == false,
          "ordinary ancestor terminates the flag query without interpreting irrelevant reference "
          "flags");
    c.models[2].parent = 3;
    c.models[3].reference_primary_flags = 0;
    c.models[3].parent = 2;
    check(evaluate_reference_descendant_filter(q, c, 1, 2).at("reason") ==
              "cycle_in_candidate_host_flag_query",
          "flag-query cycle is distinct from parent-root cycle and incomplete chain");
    c.models[2].reference_primary_flags = 0x10000000;
    c.models[2].parent_known = false;
    check(evaluate_reference_descendant_filter(q, c, 1, 2).at("excluded") == true,
          "candidate flag short-circuits its unused broken parent chain");
    c.models[1].parent = 3;
    c.models[3].parent = 1;
    r = evaluate_reference_descendant_filter(q, c, 1, 2);
    check(r.at("reason") == "cycle_in_reference_parent_root_query" &&
              r.at("gate").at("search_required") == true,
          "selected gate does not conceal a cycle while selecting the actual search root");
    c.models[1].parent = 0;
    q.runtime_flags = 0;
    check(evaluate_reference_descendant_filter(q, c, 1, 2).at("reason") ==
              "conflicting_search_candidate_runtime_flags",
          "query and graph cannot describe two incompatible runtime flag values for one candidate");
    q.runtime_flags.reset();
    q.reference_present = false;
    check(evaluate_reference_descendant_filter(q, c, 1, 2).at("reason") ==
              "conflicting_search_candidate_presence",
          "selected ordinary reference cannot be a null query object");
    q.reference_present = true;
    c.models[2].reference_present = false;
    check(evaluate_reference_descendant_filter(q, c, 1, 2).at("reason") ==
              "conflicting_reference_object_presence",
          "explicit reference absence cannot contradict confirmed ordinary reference dispatch");
    c.models[2].reference_present.reset();
    c.models[0].valid = false;
    q.stored_file_reference = u"";
    q.model_id = 0xfffffffeu;
    r = evaluate_reference_descendant_filter(q, c, 1, 2);
    check(r.at("excluded") == true && r.at("search_root").at("model_index").is_null(),
          "invalid host parent passes a null search root through the native sentinel matcher");
    c.models[0].valid = true;
    q.stored_file_reference = u"F";
    q.model_id = 8;
    c.models[0].active_list_known_absent = false;
    c.models[0].active_links = {3};
    c.models[0].active_links_complete = true;
    c.models[3].root_present = true;
    c.models[3].file = NativeFileReference{u"F", u"F"};
    c.models[3].model_id = 8;
    c.models[3].reference_runtime_flags = 0;
    c.models[3].active_list_known_absent = true;
    r = evaluate_reference_descendant_filter(q, c, 1, 2);
    check(
        r.at("search").at("matching_path") == Json::array({0, 3}),
        "confirmed dispatch supplies native kind and reference presence to descendant enumeration");
    c.models[3].native_kind = 1;
    check(evaluate_reference_descendant_filter(q, c, 1, 2).at("reason") ==
              "conflicting_reference_object_kind",
          "explicit kind cannot contradict known reference implementation");
    c.models[3].native_kind.reset();
    c.models[0].root_present = false;
    check(evaluate_reference_descendant_filter(q, c, 1, 2).at("reason") ==
              "ordinary_model_has_no_self_root",
          "ordinary model must return itself for the connected-root query");
    c.models[0].root_present.reset();
    c.models[0].parent_known = true;
    c.models[0].parent = 1;
    check(evaluate_reference_descendant_filter({}, c, 0, 2).at("reason") ==
              "ordinary_model_has_nonnull_parent",
          "ordinary model parent lookup cannot accept a contradictory nonnull parent");
    c.models[0].parent_known = false;
    const auto baseline = evaluate_reference_descendant_filter(q, c, 1, 2);
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(std::async(std::launch::async, [q, c] {
            return evaluate_reference_descendant_filter(q, c, 1, 2);
        }));
    for (auto &job : jobs)
        check(job.get() == baseline,
              "graph-derived filters are independent across concurrent calls");
    ReferenceSearchContext deep;
    deep.models.resize(8000);
    for (std::size_t i = 0; i + 1 < deep.models.size(); ++i) {
        deep.models[i].object_dispatch = Dispatch::reference;
        deep.models[i].parent_known = true;
        deep.models[i].parent = i + 1;
    }
    deep.models.back().object_dispatch = Dispatch::model;
    r = reference_parent_root(deep, 0);
    check(r.at("model_index") == 7999 && r.at("parent_path").size() == 8000,
          "deep parent-root traversal is iterative and preserves the complete identity path");
    return checks;
}
