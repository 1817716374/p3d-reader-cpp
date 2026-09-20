#include "internal.hpp"
#include <p3d/view_sequence.hpp>
#include <p3d/reference_recursion.hpp>
#include <future>
#include <numeric>
#include <random>

unsigned reference_loading_tests();
unsigned native_reference_sort_tests();

namespace {
unsigned reference_repetition_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto target = [](std::u16string primary, std::u16string alternate = u"") {
        return Json{
            {"strings",
             Json::array({{{"key", 21},
                           {"status", "decoded"},
                           {"utf16_code_units",
                            std::vector<std::uint16_t>(primary.begin(), primary.end())}},
                          {{"key", 36},
                           {"status", "decoded"},
                           {"utf16_code_units",
                            std::vector<std::uint16_t>(alternate.begin(), alternate.end())}}})}};
    };
    auto text = target(u"model", u"alternate");
    ReferenceRepetitionContext c;
    check(reference_ancestor_repetition(0x200, 0, nullptr, c).at("repeated") == false,
          "reference bit 9 disables checking before gate or strings are needed");
    check(reference_ancestor_repetition(0, 0, text, c).at("status") == "unresolved",
          "unknown native gate is not assumed enabled or disabled");
    c.native_setting_enabled = false;
    c.host_file_fallback_gate = false;
    check(reference_ancestor_repetition(0, 0, nullptr, c).at("repeated") == false,
          "both disabled gate inputs avoid unused string and chain reads");
    c.native_setting_enabled.reset();
    c.host_file_fallback_gate = true;
    c.lookup_reference = u"file";
    c.complete_ancestor_chain = true;
    c.equal = [](const auto &a, const auto &b) { return a == b; };
    auto ancestor = [](std::u16string file, std::u16string name) {
        ReferenceAncestorState a;
        a.reference_known_absent = true;
        a.file = NativeFileReference{file, file};
        a.is_default_model = false;
        a.model_name = name;
        return a;
    };
    c.ancestors = {ancestor(u"file", u"model")};
    auto r = reference_ancestor_repetition(0, 0, text, c);
    check(r.at("repeated") == false && r.at("matching_ancestor_indices") == Json::array({0}),
          "first host match has depth plus previous matches zero and does not reach limit one");
    c.ancestors.insert(c.ancestors.begin(), ancestor(u"other", u"model"));
    r = reference_ancestor_repetition(0, 0, text, c);
    check(r.at("repeated") == true && r.at("triggering_ancestor_index") == 1 &&
              r.at("native_comparison_value") == 1,
          "a single match at depth one meets native threshold without two equal ancestors");
    c.limit = 2;
    check(reference_ancestor_repetition(0, 0, text, c).at("repeated") == false,
          "limit compares depth plus previous match count rather than current match count");
    c.ancestors[0] = ancestor(u"file", u"model");
    check(reference_ancestor_repetition(0, 0, text, c).at("native_comparison_value") == 2,
          "earlier matches contribute to later depth comparison");
    c.limit = -1;
    check(reference_ancestor_repetition(0, 0, text, c).at("triggering_ancestor_index") == 0,
          "negative signed limit preserves the native comparison");
    c.limit = 1;
    c.complete_ancestor_chain = false;
    check(reference_ancestor_repetition(0, 0, text, c).at("repeated") == true,
          "a decisive repetition does not require an unvisited parent suffix");
    c.ancestors.resize(1);
    check(reference_ancestor_repetition(0, 0, text, c).at("reason") ==
              "remaining_ancestor_chain_required",
          "below-threshold prefix cannot prove absence of repetition");
    c.ancestors[0].valid = false;
    check(reference_ancestor_repetition(0, 0, text, c).at("repeated") == false,
          "invalid host terminates the native chain independently of supplied completeness");
    c.ancestors = {ancestor(u"file", u"model"), ancestor(u"file", u"model")};
    c.ancestors[0].reference_known_absent = false;
    c.ancestors[0].reference_primary_flags = 0x200;
    c.ancestors[0].file.reset();
    c.ancestors[0].model_name.reset();
    r = reference_ancestor_repetition(0, 0, text, c);
    check(r.at("triggering_ancestor_index") == 1 &&
              r.at("matching_ancestor_indices") == Json::array({1}),
          "skipped reference counts toward depth but needs neither file nor model name");
    c.ancestors[0].reference_primary_flags.reset();
    check(reference_ancestor_repetition(0, 0, text, c).at("status") == "unresolved",
          "missing reference state cannot silently skip a host");
    c.ancestors = {ancestor(u"unused", u"unused"), ancestor(u"file", u"model")};
    c.ancestors[0].file.reset();
    c.ancestors[0].file_known_absent = true;
    check(reference_ancestor_repetition(0, 0, text, c).at("repeated") == true,
          "known absent host file is different from missing file context");
    c.ancestors[0] = ancestor(u"file", u"model");
    c.ancestors[1].file->lookup_reference = u"";
    check(reference_ancestor_repetition(0, 0, text, c).at("repeated") == true,
          "ancestor wrapper falls back to stored reference when its lookup reference is empty");
    c.ancestors[1].file->stored_reference = u"other/file";
    c.complete_ancestor_chain = true;
    check(reference_ancestor_repetition(0, 0, text, c).at("repeated") == false,
          "file comparison does not discard directories or compare basenames only");
    c.equal = {};
    check(reference_ancestor_repetition(0, 0, text, c).at("status") == "unresolved",
          "unequal wide strings require the actual comparison context");
    c.equal = [](const auto &a, const auto &b) { return a == b; };
    c.ancestors = {ancestor(u"alternate-file", u"alternate"),
                   ancestor(u"alternate-file", u"alternate")};
    c.alternate_file_reference = u"alternate-file";
    c.lookup_reference.reset();
    check(reference_ancestor_repetition(0, 0x20000, text, c).at("repeated") == true,
          "bit 17 selects runtime alternate file and persisted alternate model name");
    c.alternate_file_reference = u"";
    check(reference_ancestor_repetition(0, 0x20000, text, c).at("reason") ==
              "repetition_lookup_reference_required",
          "known empty alternate file takes lookup-reference fallback");
    c.alternate_file_reference.reset();
    c.lookup_reference = u"file";
    check(reference_ancestor_repetition(0, 0x20000, text, c).at("reason") ==
              "alternate_repetition_file_state_required",
          "unavailable alternate file is not treated as an empty one");
    c.ancestors = {ancestor(u"file", u"model"), ancestor(u"file", u"model")};
    check(reference_ancestor_repetition(0, 0x8000, text, c).at("repeated") == true,
          "file-default-model selection bit does not replace this check's model name");
    c.ancestors[0].is_default_model.reset();
    c.ancestors[1].is_default_model.reset();
    check(reference_ancestor_repetition(0, 0, text, c).at("repeated") == true,
          "nonempty query name makes default-model status irrelevant to matching");
    for (auto &a : c.ancestors) {
        a.is_default_model = true;
        a.model_name.reset();
    }
    check(reference_ancestor_repetition(0, 0, target(u""), c).at("repeated") == true,
          "empty query matches a known default model without reading its name");
    for (auto &a : c.ancestors) {
        a.is_default_model = false;
        a.model_name = u"";
    }
    check(reference_ancestor_repetition(0, 0, target(u""), c).at("repeated") == true,
          "non-default empty-name model can still match through the name branch");
    c.ancestors = {ancestor(u"file", u"model"), ancestor(u"file", u"model")};
    c.lookup_reference = std::u16string(u"file\0tail", 9);
    check(reference_ancestor_repetition(0, 0, target(std::u16string(u"model\0tail", 10)), c)
                  .at("repeated") == true,
          "native pointer strings compare only their NUL-terminated prefixes");

    std::mt19937 rng(148);
    for (unsigned trial = 0; trial < 500; ++trial) {
        ReferenceRepetitionContext random;
        random.native_setting_enabled = true;
        random.lookup_reference = u"file";
        random.equal = [](const auto &a, const auto &b) { return a == b; };
        random.complete_ancestor_chain = true;
        random.limit = static_cast<std::int32_t>(rng() % 18) - 2;
        bool expected = false;
        unsigned prior = 0;
        const unsigned size = rng() % 20;
        for (unsigned i = 0; i < size; ++i) {
            const bool file_match = rng() % 2, model_match = rng() % 2, skip = rng() % 5 == 0;
            auto a = ancestor(file_match ? u"file" : u"other", model_match ? u"model" : u"other");
            a.reference_known_absent = false;
            a.reference_primary_flags = skip ? 0x200u : 0u;
            random.ancestors.push_back(a);
            if (!skip && file_match && model_match) {
                if (static_cast<std::int32_t>(i + prior) >= random.limit)
                    expected = true;
                ++prior;
            }
        }
        check(reference_ancestor_repetition(0, 0, text, random).at("repeated") == expected,
              "native depth and prior-match predicate agrees with independent chain oracle");
    }
    return checks;
}

unsigned model_link_registry_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    Json records = Json::array();
    Json input = {{"scope", "fresh_model_control_then_graphics_id_registration"},
                  {"status", "resolved"},
                  {"model_storage", Json::array({"model"})},
                  {"initial_id_counter", 100},
                  {"roots", Json::array()},
                  {"inputs", Json::array({{{"kind", "P3D-SMC"},
                                           {"status", "resolved"},
                                           {"container", Json::array({"model", "control"})},
                                           {"root_count", 0}}})}};
    auto add = [&](unsigned type, std::uint64_t source_id, std::uint64_t assigned_id,
                   unsigned subtype = 0, bool control = true) {
        Bytes bytes(type == 13 ? 372 : 44, 0);
        auto put = [&](std::size_t offset, auto value) {
            std::memcpy(bytes.data() + offset, &value, sizeof value);
        };
        put(4, static_cast<std::uint16_t>(type));
        put(8, static_cast<std::uint32_t>((bytes.size() - 4) / 2));
        put(12, static_cast<std::uint32_t>((bytes.size() - 4) / 2));
        put(16, static_cast<std::uint32_t>(subtype));
        put(20, source_id);
        const auto ni = records.size();
        records.push_back(parse_native(bytes).at(0));
        const auto ri = input["roots"].size();
        input["roots"].push_back(
            {{"kind", control ? "P3D-SMC" : "P3D-SMG"},
             {"input_list_index", control ? 0 : 1},
             {"container", Json::array({"model", control ? "control" : "graphics"})},
             {"native_record_index", ni},
             {"block_number", ri + 1},
             {"records", Json::array({{{"native_record_index", ni},
                                       {"input_occurrence_index", ri},
                                       {"parent_input_occurrence_index", nullptr},
                                       {"source_id", source_id},
                                       {"assigned_id", assigned_id}}})}});
        if (control)
            input["inputs"][0]["root_count"] = input["inputs"][0]["root_count"].get<unsigned>() + 1;
        return ri;
    };
    add(13, 8, UINT64_MAX);
    add(13, 8, 2);
    add(13, 9, UINT64_C(0x8000000000000000));
    add(47, 11, 11, 33);
    auto last = add(47, 12, 12, 33);
    const auto graphics = add(13, 44, 44, 0, false);
    add(47, 45, 45, 33, false);
    // A descendant in the prepared preorder is not a control-list root.
    auto child = input["roots"][graphics]["records"][0];
    child["parent_input_occurrence_index"] = 0;
    child["input_occurrence_index"] = 99;
    input["roots"][0]["records"].push_back(child);
    const auto unchanged = records;
    auto r = initial_model_link_registry(input, records);
    check(
        r.at("status") == "resolved" && r.at("registry").size() == 3 &&
            r.at("registry")[0].at("id") == 2 &&
            r.at("registry")[1].at("id") == UINT64_C(0x8000000000000000) &&
            r.at("registry")[2].at("id") == UINT64_MAX && records == unchanged,
        "control-root registry uses assigned IDs and unsigned tree order without changing sources");
    check(r.at("saved_sequence_source").at("root_index") == last &&
              r.at("registry")[0].at("source").at("source_id") == 8,
          "last control sequence wins while original colliding file IDs remain available");
    const auto sequence_index = input["roots"][last]["native_record_index"].get<std::size_t>();
    records[sequence_index]["view_link_sequence"]["decode_error"] = "invalid sequence";
    check(
        initial_model_link_registry(input, records).at("saved_sequence_source").at("root_index") ==
            last,
        "registry selection does not fall back from a malformed final sequence");
    input["status"] = "partial"; // Later graphics input does not alter completed control IDs.
    check(initial_model_link_registry(input, records).at("status") == "resolved",
          "incomplete later graphics input does not block completed control registration");
    input["roots"][0]["records"][0]["assigned_id"] = 0;
    input["roots"][1]["records"][0]["assigned_id"] = 0;
    r = initial_model_link_registry(input, records);
    check(r.at("registry").size() == 2 && r.at("registry")[0].at("id") == 0 &&
              r.at("registry")[0].at("source").at("root_index") == 1 &&
              r.at("replaced_sources").size() == 1,
          "zero registration keys are retained and repeated keys select the last root");
    auto bad = input;
    bad["inputs"][0]["status"] = "partial";
    check(initial_model_link_registry(bad, records).at("status") == "unresolved",
          "incomplete control input cannot establish the complete link registry");
    bad = input;
    bad["roots"].erase(0);
    check(initial_model_link_registry(bad, records).at("reason") ==
              "incomplete_assigned_control_roots",
          "control root count prevents silently accepting a missing source");
    bad = input;
    bad["roots"][0]["records"][0]["assigned_id"] = -1;
    check(initial_model_link_registry(bad, records).at("status") == "unresolved",
          "negative assigned ID cannot wrap to a valid unsigned key");
    bad = input;
    bad["roots"][1]["records"][0]["input_occurrence_index"] = 0;
    check(initial_model_link_registry(bad, records).at("status") == "unresolved",
          "duplicate occurrence identity is not merged merely because source IDs match");
    bad = input;
    bad["roots"][0]["container"] = Json::array({"other", "control"});
    check(initial_model_link_registry(bad, records).at("status") == "unresolved",
          "control roots cannot be borrowed from a different model container");
    bad = input;
    bad["roots"] = Json::array();
    bad["inputs"][0]["status"] = "not_loaded";
    r = initial_model_link_registry(bad, records);
    check(r.at("registry").empty() && r.at("saved_sequence_source").is_null(),
          "missing control container has an empty conditional initial registry");
    bad["status"] = "unresolved";
    check(initial_model_link_registry(bad, records).at("status") == "unresolved",
          "failed ID preparation is not interpreted as an empty successful model");
    return checks;
}

unsigned view_candidate_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    using Kind = ViewCandidateNodeKind;
    ViewCandidateContext c;
    c.nodes = {{Kind::model, true, {}, false, 0},    {Kind::reference, true, 0, true, 10},
               {Kind::reference, true, 1, true, 20}, {Kind::model, true, {}, false, 0},
               {Kind::reference, true, 3, true, 10}, {Kind::reference, false, 0, true, 30}};
    c.root_model_available = true;
    auto ids = [](const Json &r) {
        Json result = Json::array();
        for (const auto &item : r.at("candidates"))
            result.push_back(item.at("model_index"));
        return result;
    };
    c.provided_candidates = std::vector<std::optional<std::size_t>>{2, 1, 2, 4, 3, {}, 5, 0};
    auto r = collect_view_link_candidates(c);
    check(
        r.at("status") == "resolved" && ids(r) == Json::array({2, 1, 2, 0}) &&
            r.at("rejected_candidates").size() == 4,
        "provided candidates follow ancestry, preserve duplicate identity and reject null/invalid");
    c.include_current_model = c.include_links = false;
    check(ids(collect_view_link_candidates(c)) == ids(r),
          "explicit candidates override both include flags");
    c.current_model = 1;
    check(ids(collect_view_link_candidates(c)) == Json::array({2, 1, 2}),
          "nested selected reference accepts its own identity and descendants");
    c.sequence = std::vector<std::uint64_t>{0, 20};
    r = collect_view_link_candidates(c);
    check(ids(r) == Json::array({1, 2, 2}) && r.at("matches").size() == 2,
          "collection and sequence swaps use selected object identity rather than a zero file ID");
    c.nodes[2].parent_known = false;
    r = collect_view_link_candidates(c);
    check(r.at("status") == "unresolved" && !r.contains("candidates"),
          "unknown parent does not publish a partial final candidate list");
    c.nodes[2].parent_known = true;
    c.nodes[2].parent = 2;
    check(collect_view_link_candidates(c).at("reason") == "cycle_in_candidate_parent_chain",
          "parent cycle before a match is explicit instead of looping");
    c.current_model = 2;
    c.provided_candidates = std::vector<std::optional<std::size_t>>{2};
    check(ids(collect_view_link_candidates(c)) == Json::array({2}),
          "identity succeeds without following an unused self-cycle");
    c.current_model = 0;
    c.nodes[2].parent = 1;
    c.nodes[1].valid = false;
    c.provided_candidates = std::vector<std::optional<std::size_t>>{2};
    check(ids(collect_view_link_candidates(c)).empty(),
          "invalid parent terminates ancestry before the selected root");
    c.nodes[1].valid = true;
    c.nodes[1].kind = Kind::unknown;
    check(collect_view_link_candidates(c).at("status") == "unresolved",
          "unknown native object kind is not assumed to forward to parent");
    c.nodes[1].kind = Kind::reference;
    c.nodes[3].parent = 0;
    c.nodes[3].parent_known = true;
    c.provided_candidates = std::vector<std::optional<std::size_t>>{4};
    check(ids(collect_view_link_candidates(c)).empty(),
          "ordinary model terminates traversal despite an irrelevant parent field");
    c.provided_candidates->clear();
    check(collect_view_link_candidates(c).at("native_return_code") == 1 &&
              collect_view_link_candidates(c).at("output_replaced") == true,
          "explicit empty filter replaces output with empty list");
    c.root_model_available = false;
    r = collect_view_link_candidates(c);
    check(r.at("output_replaced") == false && !r.contains("candidates"),
          "missing root returns before replacing existing native output");
    c.root_model_available.reset();
    check(collect_view_link_candidates(c).at("status") == "unresolved",
          "root availability is not inferred from a parent or sequence");
    c.nodes[0].valid = false;
    check(collect_view_link_candidates(c).at("output_replaced") == false,
          "invalid selected model returns before requiring a root");
    c.nodes[0].valid = true;
    c.root_model_available = true;
    c.provided_candidates.reset();
    c.include_current_model = c.include_links = true;
    check(collect_view_link_candidates(c).at("status") == "unresolved",
          "default collection requires the complete list when links are requested");
    c.links_complete = true;
    c.links = {4, 5, {}, 1, 1};
    c.apply_sequence = false;
    check(ids(collect_view_link_candidates(c)) == Json::array({0, 4, 5, nullptr, 1, 1}),
          "default collection copies all slots without validity or ancestry filtering");
    c.apply_sequence = true;
    c.sequence = std::vector<std::uint64_t>{0};
    check(ids(collect_view_link_candidates(c)) == Json::array({0, 4, 5, nullptr, 1, 1}),
          "zero sequence item does not dereference a null link");
    c.sequence = std::vector<std::uint64_t>{99};
    r = collect_view_link_candidates(c);
    check(r.at("reason") == "null_candidate_in_native_id_lookup" && !r.contains("candidates"),
          "nonzero sequence item reports the native null dereference boundary");
    c.nodes[4].link_id.reset();
    c.links = {4};
    c.sequence = std::vector<std::uint64_t>{0};
    check(ids(collect_view_link_candidates(c)) == Json::array({0, 4}),
          "unqueried missing ID does not prevent identity-only ordering");
    c.sequence = std::vector<std::uint64_t>{10};
    check(collect_view_link_candidates(c).at("reason") == "candidate_link_id_required",
          "missing ID matters only when visited by native ID comparison");
    c.include_current_model = false;
    check(ids(collect_view_link_candidates(c)) == Json::array({4}),
          "one candidate bypasses sequence ID lookup");
    c.links = {99};
    check(collect_view_link_candidates(c).at("status") == "unresolved",
          "graph indices cannot refer outside the supplied identity table");

    // Independent literal ancestry and swap implementation over random forests.
    std::mt19937_64 rng(146);
    for (unsigned trial = 0; trial < 400; ++trial) {
        ViewCandidateContext random;
        random.root_model_available = true;
        const std::size_t count = 2 + rng() % 50;
        for (std::size_t i = 0; i < count; ++i)
            random.nodes.push_back(
                {i == 0 || rng() % 7 == 0 ? Kind::model : Kind::reference, i == 0 || rng() % 9 != 0,
                 i ? std::optional<std::size_t>(rng() % i) : std::nullopt, true, rng() % 8});
        random.current_model = rng() % count;
        random.nodes[random.current_model].valid = true;
        random.provided_candidates.emplace();
        std::vector<std::size_t> expected;
        for (unsigned j = 0; j < 70; ++j) {
            const auto candidate = static_cast<std::size_t>(rng() % count);
            random.provided_candidates->push_back(candidate);
            auto at = candidate;
            for (;;) {
                const auto &n = random.nodes[at];
                if (!n.valid)
                    break;
                if (at == random.current_model) {
                    expected.push_back(candidate);
                    break;
                }
                if (n.kind == Kind::model || !n.parent)
                    break;
                at = *n.parent;
            }
        }
        random.sequence.emplace();
        std::size_t next = 0;
        for (unsigned j = 0; j < 12; ++j) {
            const auto id = rng() % 10;
            random.sequence->push_back(id);
            if (expected.size() <= 1)
                continue;
            for (std::size_t k = next; k < expected.size(); ++k) {
                if (id == 0 ? expected[k] != random.current_model
                            : *random.nodes[expected[k]].link_id != id)
                    continue;
                std::swap(expected[next++], expected[k]);
                break;
            }
        }
        const auto result = collect_view_link_candidates(random);
        check(result.at("status") == "resolved" && ids(result) == Json(expected),
              "cached ancestry and indexed swaps equal literal forest traversal and swaps");
    }
    ViewCandidateContext deep;
    deep.root_model_available = true;
    deep.nodes.push_back({Kind::model, true, {}, true, 0});
    for (std::size_t i = 1; i <= 10000; ++i)
        deep.nodes.push_back({Kind::reference, true, i - 1, true, i});
    deep.provided_candidates = std::vector<std::optional<std::size_t>>{10000, 9999, 10000};
    const auto deep_result = collect_view_link_candidates(deep);
    check(ids(deep_result) == Json::array({10000, 9999, 10000}),
          "deep shared ancestry is iterative and keeps repeated object occurrences");
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(
            std::async(std::launch::async, [&] { return collect_view_link_candidates(deep); }));
    for (auto &job : jobs)
        check(job.get() == deep_result, "candidate graph caches are call-local and parallel-safe");
    return checks;
}
} // namespace

unsigned view_sequence_tests() {
    using namespace p3d;
    unsigned checks = view_candidate_tests() + model_link_registry_tests() +
                      reference_repetition_tests() + reference_loading_tests() +
                      native_reference_sort_tests();
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
    // Exercise the file-to-context connection, including neighboring flag bits
    // and the legacy reference layout's distinct physical source offsets.
    auto put = [](Bytes &b, std::size_t offset, auto value) {
        std::memcpy(b.data() + offset, &value, sizeof value);
    };
    for (unsigned bit = 0; bit < 32; ++bit) {
        Bytes model(500, 0);
        put(model, 4, std::uint16_t(47));
        put(model, 8, std::uint32_t(248));
        put(model, 12, std::uint32_t(248));
        put(model, 16, std::uint32_t(32));
        put(model, 72, std::uint32_t(1) << bit);
        const auto original = model;
        const auto records = parse_native(model);
        const auto &view = records.at(0).at("model_view_state");
        check(view.at("status") == "decoded" && view.at("current_model_last") == (bit == 11) &&
                  view.at("flags_source_offset") == 72 && model == original &&
                  bytesof(records.at(0).at("data")) == model,
              "model sequence position comes from bit 11 and preserves the source");
        ViewSequenceContext loaded;
        loaded.current_model_last = view.at("current_model_last").get<bool>();
        loaded.links_complete = true;
        loaded.initialize_default = true;
        loaded.links = {{123}};
        check(resolve_view_link_sequence(nullptr, loaded).at("entry_ids") ==
                  (bit == 11 ? Json::array({123, 0}) : Json::array({0, 123})),
              "decoded model header determines default current-model position");
        for (const bool legacy : {false, true}) {
            Bytes reference(legacy ? 348 : 372, 0);
            put(reference, 4, std::uint16_t(13));
            put(reference, 8, std::uint32_t((reference.size() - 4) / 2));
            put(reference, 12, std::uint32_t((reference.size() - 4) / 2));
            put(reference, 20, UINT64_MAX);
            put(reference, legacy ? 44 : 48, std::uint32_t(0xfedcba98));
            put(reference, legacy ? 52 : 60, std::uint32_t(1) << bit);
            const auto before = reference;
            const auto parsed = parse_native(reference);
            const auto &input = parsed.at(0).at("reference_input");
            const auto &fields = input.at("view_sequence_inputs");
            check(input.at("status") == "decoded" && input.at("layout").at("upgraded") == legacy &&
                      fields.at("link_id") == UINT64_MAX &&
                      fields.at("same_kind_2_sort_value") == 0xfedcba98u &&
                      fields.at("sort_value_source_offset") == (legacy ? 44 : 48) &&
                      fields.at("primary_flags_source_offset") == (legacy ? 52 : 60) &&
                      reference == before && bytesof(parsed.at(0).at("data")) == before,
                  "modern and legacy references preserve unsigned link ordering inputs");
            loaded.links = {{fields.at("link_id").get<std::uint64_t>(), true,
                             fields.at("excluded_from_reconciliation").get<bool>()}};
            loaded.initialize_default = false;
            loaded.current_model_last = false;
            check(resolve_view_link_sequence(saved({0}), loaded).at("entry_ids") ==
                      (bit == 14 ? Json::array({0}) : Json::array({std::uint64_t(0), UINT64_MAX})),
                  "only primary bit 14 excludes a base-loaded reference from reconciliation");
        }
    }
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
