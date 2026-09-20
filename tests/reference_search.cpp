#include "internal.hpp"
#include <p3d/reference_search.hpp>
#include <p3d/reference_loading.hpp>
#include <future>

unsigned reference_graph_tests();
unsigned reference_search_input_tests();

unsigned reference_search_tests() {
    using namespace p3d;
    unsigned checks = reference_graph_tests() + reference_search_input_tests();
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    ReferenceDescendantGateContext gate;
    check(reference_descendant_search_gate(gate).at("status") == "unresolved",
          "unknown host gate is not false");
    gate.host_valid = false;
    check(reference_descendant_search_gate(gate).at("search_required") == false,
          "invalid host avoids all later gate inputs");
    gate.host_valid = true;
    gate.host_parent_present = false;
    check(reference_descendant_search_gate(gate).at("search_required") == false,
          "root host has no parent search");
    gate.host_parent_present = true;
    gate.candidate_valid = false;
    check(reference_descendant_search_gate(gate).at("search_required") == false,
          "invalid candidate suppresses search");
    gate.candidate_valid = true;
    gate.candidate_reference_present = false;
    check(reference_descendant_search_gate(gate).at("search_required") == false,
          "candidate without reference suppresses search");
    gate.candidate_reference_present = true;
    gate.candidate_primary_flags = 0x10000000;
    check(reference_descendant_search_gate(gate).at("search_required") == true,
          "candidate bit28 needs no ancestor suffix");
    gate.candidate_primary_flags = 0;
    check(reference_descendant_search_gate(gate).at("status") == "unresolved",
          "negative candidate bit does not prove complete host chain");
    ReferenceAncestorState a;
    a.reference_primary_flags = 0;
    ReferenceAncestorState b;
    b.reference_primary_flags = 0x10000000;
    gate.candidate_hosts = {a, b};
    auto r = reference_descendant_search_gate(gate);
    check(r.at("search_required") == true && r.at("matching_candidate_host_index") == 1,
          "ancestor bit28 opens search without needing files or model names");
    gate.candidate_hosts[0].reference_primary_flags.reset();
    gate.candidate_hosts[0].reference_known_absent = true;
    check(reference_descendant_search_gate(gate).at("search_required") == false,
          "host without reference terminates instead of skipping toward later flags");
    gate.candidate_hosts[0] = a;
    gate.candidate_hosts[1] = a;
    gate.complete_candidate_host_chain = true;
    check(reference_descendant_search_gate(gate).at("search_required") == false,
          "complete unflagged host chain is known negative");
    gate.candidate_hosts[1].valid = false;
    gate.complete_candidate_host_chain = false;
    check(reference_descendant_search_gate(gate).at("search_required") == false,
          "invalid ancestor terminates without a completeness declaration");

    ReferenceSearchQuery q;
    q.stored_file_reference = u"file";
    q.runtime_flags = 0x1000;
    q.model_id = 7;
    auto model = [](std::uint32_t id) {
        ReferenceSearchModel m;
        m.root_present = true;
        m.file = NativeFileReference{u"stored", u"file"};
        m.model_id = id;
        m.model_name = u"main";
        m.is_default_model = false;
        m.native_kind = 2;
        m.reference_present = true;
        m.reference_runtime_flags = 0;
        m.active_list_known_absent = true;
        return m;
    };
    auto m = model(7);
    r = match_reference_model(q, m);
    check(r.at("matched") == true && r.at("file_match_source") == "stored_reference",
          "stored file match avoids unknown lookup and name inputs");
    m.model_id = 8;
    check(match_reference_model(q, m).at("matched") == false,
          "explicit model ID mismatch does not fall back to default/name");
    q.equal = [](const auto &x, const auto &y) { return x == y; };
    q.stored_file_reference = u"other";
    q.lookup_file_reference = u"file";
    m.model_id = 7;
    check(match_reference_model(q, m).at("file_match_source") == "lookup_reference",
          "lookup file is compared only after stored reference differs");
    q.lookup_file_reference = u"other";
    q.runtime_flags.reset();
    m.model_id.reset();
    check(match_reference_model(q, m).at("matched") == false,
          "file mismatch avoids both model selection and actual ID");
    q.lookup_file_reference.reset();
    check(match_reference_model(q, m).at("status") == "unresolved",
          "unavailable lookup is not an empty string after stored mismatch");
    q.stored_file_reference = u"file";
    q.runtime_flags = 0;
    q.model_name = u"main";
    check(match_reference_model(q, m).at("matched") == true,
          "name mode does not require an explicit model ID");
    q.model_name = u"other";
    m.is_default_model = true;
    check(match_reference_model(q, m).at("matched") == false,
          "nonempty unmatched name does not fall back to default model");
    q.model_name = u"";
    m.model_name.reset();
    check(match_reference_model(q, m).at("matched") == true,
          "empty primary name uses default status without requiring model name");
    m.is_default_model = false;
    m.model_name = u"";
    check(match_reference_model(q, m).at("matched") == false,
          "unlike ancestor repetition, two empty names do not override nondefault status");
    m.file->lookup_reference = u"";
    m.file->stored_reference = u"file";
    q.model_name = u"main";
    m.model_name = u"main";
    check(match_reference_model(q, m).at("matched") == true,
          "model wrapper falls back from empty lookup to stored file reference");
    m.file.reset();
    m.file_known_absent = true;
    q.stored_file_reference = u"";
    check(
        match_reference_model(q, m).at("matched") == true,
        "missing file supplies empty comparison string but existing root name still participates");
    q.model_name = u"";
    m.is_default_model.reset();
    check(match_reference_model(q, m).at("matched") == false,
          "model without file cannot be file default even with empty name");
    m = {};
    m.root_present = false;
    q.runtime_flags = 0x1000;
    q.model_id = UINT32_MAX;
    check(match_reference_model(q, m).at("matched") == true,
          "valid model with no root uses native minus-one ID sentinel");
    m.valid = false;
    m.root_present.reset();
    q.model_id = 0xfffffffeu;
    check(match_reference_model(q, m).at("matched") == true,
          "invalid model uses native minus-two sentinel without root lookup");
    q.model_id = UINT32_MAX;
    check(match_reference_model(q, m).at("matched") == false,
          "invalid model and missing root are not the same sentinel");
    m = model(7);
    q.stored_file_reference = std::u16string(u"file\0tail", 9);
    q.model_id = 7;
    check(match_reference_model(q, m).at("matched") == true,
          "file pointer comparison ignores suffix after a wide NUL");
    q.runtime_flags = 0;
    q.model_name = std::u16string(u"main\0tail", 9);
    check(match_reference_model(q, m).at("matched") == true,
          "model name pointer comparison ignores its NUL suffix");
    q.stored_file_reference = u"FILE";
    q.model_name = u"MAIN";
    q.equal = {};
    check(match_reference_model(q, m).at("status") == "unresolved",
          "distinct strings require originating wide comparison semantics");
    std::vector<std::pair<std::u16string, std::u16string>> comparisons;
    q.equal = [&](const auto &x, const auto &y) {
        comparisons.emplace_back(x, y);
        return true;
    };
    check(match_reference_model(q, m).at("matched") == true && comparisons.size() == 2 &&
              comparisons[0] == std::make_pair(std::u16string(u"file"), std::u16string(u"FILE")) &&
              comparisons[1] == std::make_pair(std::u16string(u"MAIN"), std::u16string(u"main")),
          "file and name comparisons preserve their different native argument directions");

    q = {};
    q.stored_file_reference = u"file";
    q.runtime_flags = 0x1000;
    q.model_id = 7;
    ReferenceSearchContext c;
    c.start_known = true;
    c.start = 0;
    c.models = {model(1), model(7), model(7), model(7)};
    auto links = [&](std::size_t i, std::vector<std::optional<std::size_t>> children) {
        c.models[i].active_links = std::move(children);
        c.models[i].active_list_known_absent = false;
        c.models[i].active_links_complete = true;
    };
    links(0, {{}, 1, 2, 3});
    c.models[1].native_kind = 1;
    c.models[2].reference_runtime_flags = 0x20;
    r = search_reference_descendants(q, c);
    check(r.at("matched") == true && r.at("matching_model_index") == 3 &&
              r.at("visited_model_indices") == Json::array({0, 3}) &&
              r.at("skipped_links").size() == 3,
          "null, other-kind and runtime-bit5 entries are skipped before model matching");
    links(1, {3});
    links(2, {3});
    links(0, {1, 2});
    check(search_reference_descendants(q, c).at("matched") == false,
          "filtered entry subtrees are not traversed through the depth-zero iterator");
    c.models[1] = model(4);
    c.models[2] = model(5);
    links(1, {3});
    links(2, {3});
    links(0, {1, 2});
    c.models[3] = model(6);
    r = search_reference_descendants(q, c);
    check(r.at("matched") == false && r.at("visited_model_indices") == Json::array({0, 1, 3, 2, 3}),
          "shared native object is revisited in each occurrence without global deduplication");
    c.models[3].model_id = 7;
    r = search_reference_descendants(q, c);
    check(r.at("matching_path") == Json::array({0, 1, 3}) &&
              r.at("visited_model_indices") == Json::array({0, 1, 3}),
          "search uses depth-first native link order and returns the first match");
    c.models[0].active_links_complete = false;
    c.models[0].active_links.push_back(9999);
    check(search_reference_descendants(q, c).at("matched") == true,
          "early match does not require an unvisited list suffix");
    c.models[1].model_id.reset();
    check(search_reference_descendants(q, c).at("status") == "unresolved",
          "earlier unknown match cannot be bypassed to reach a later match");
    c.models[1].model_id = 4;
    c.models[3].model_id = 6;
    c.models[0].active_links = {1, 2};
    check(search_reference_descendants(q, c).at("reason") ==
              "remaining_active_reference_links_required",
          "known nonmatching prefix cannot prove exhaustive absence");
    c.models[0].active_links_complete = true;
    links(3, {0});
    check(search_reference_descendants(q, c).at("reason") == "cycle_in_active_reference_search",
          "active recursion cycle is explicit instead of hanging or returning false");
    c.models[0].model_id = 7;
    check(search_reference_descendants(q, c).at("matched") == true,
          "starting model match succeeds before following its cycle");
    c.models[0].model_id = 1;
    c.models[0].active_links = {9999};
    check(search_reference_descendants(q, c).at("reason") == "active_reference_index_out_of_range",
          "invalid graph identity is not treated as missing file data");
    c.models[0].active_links = {1};
    c.models[1].valid = false;
    check(search_reference_descendants(q, c).at("reason") ==
              "invalid_kind2_reference_in_active_list",
          "native invalid kind2 dereference is reported instead of silently filtering it");
    c.models[1].valid = true;
    c.models[1].reference_present = false;
    check(search_reference_descendants(q, c).at("reason") == "kind2_object_has_no_reference",
          "kind2 with no reference state is not a plain model");
    c.models[1].reference_present = true;
    c.models[1].native_kind.reset();
    check(search_reference_descendants(q, c).at("status") == "unresolved",
          "unknown virtual kind is not guessed from stored model type");
    c = {};
    q.reference_present = false;
    check(search_reference_descendants(q, c).at("matched") == false,
          "null query reference exits before selecting the search root");
    check(match_reference_model(q, m).at("status") == "unresolved",
          "standalone native matcher requires a reference object");
    q.reference_present = true;
    c.start_known = true;
    q.stored_file_reference = u"";
    q.model_id = 0xfffffffeu;
    r = search_reference_descendants(q, c);
    check(r.at("matched") == true && r.at("matching_model_index").is_null(),
          "null starting model still enters native matching with invalid-model ID sentinel");
    q.model_id = 7;
    check(search_reference_descendants(q, c).at("matched") == false,
          "nonmatching null starting model has no active descendants");

    q.stored_file_reference = u"file";
    c.start = 0;
    c.models.assign(5000, model(1));
    for (std::size_t i = 0; i + 1 < c.models.size(); ++i)
        links(i, {i + 1});
    c.models.back().model_id = 7;
    r = search_reference_descendants(q, c);
    check(r.at("matched") == true && r.at("matching_path").size() == 5000,
          "deep native reference chain uses an iterative stack without C++ recursion overflow");
    gate.candidate_primary_flags = 0x10000000;
    ReferenceLoadingDecisionContext decision;
    decision.native_input_status = 0;
    decision.loaded_primary_flags = 0;
    decision.host_filter_excluded = false;
    decision.source_runtime_deleted = false;
    decision.descendant_filter_required =
        reference_descendant_search_gate(gate).at("search_required").get<bool>();
    decision.descendant_match = r.at("matched").get<bool>();
    decision.keep_descendant_match = false;
    check(reference_loading_decision(13, decision).at("loading_state") == 3,
          "confirmed descendant gate and search feed the ordinary loading decision before ancestor "
          "repetition");
    c.models.resize(2);
    links(0, {1});
    c.models[1] = model(7);
    const auto expected = search_reference_descendants(q, c);
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(
            std::async(std::launch::async, [q, c] { return search_reference_descendants(q, c); }));
    for (auto &job : jobs)
        check(job.get() == expected,
              "independent descendant searches have no global mutable state");
    return checks;
}
