#include "internal.hpp"
#include <p3d/reference_search.hpp>
#include <p3d/reference_model.hpp>
#include <future>

unsigned reference_connection_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    ReferenceSearchContext c;
    c.models.resize(5);
    c.models[0].object_dispatch = ReferenceObjectDispatch::model;
    c.models[0].file = NativeFileReference{u"Host", u"Host"};
    c.models[0].model_id = 99;
    c.models[0].active_links = {1, 3};
    c.models[0].active_links_complete = true;
    c.start_known = true;
    c.start = 0;
    auto &a = c.models[1];
    a.object_dispatch = ReferenceObjectDispatch::reference;
    a.parent_known = true;
    a.parent = 0;
    a.connected_root_known = true;
    a.connected_root = 2;
    a.reference_runtime_flags = 0;
    a.active_list_known_absent = true;
    c.models[3] = a;
    auto &root = c.models[2];
    root.object_dispatch = ReferenceObjectDispatch::model;
    root.file = NativeFileReference{u"Target", u""};
    root.model_id = 7;
    root.model_name = u"Shared";
    root.is_default_model = true;
    root.active_links = {99}; // Root metadata does not replace the reference's own child list.
    check(reference_connected_root(c, 1).at("model_index") == 2 &&
              reference_parent_root(c, 1).at("model_index") == 0,
          "connected target and parent owner are distinct native object relations");
    check(reference_connected_root(c, 0).at("model_index") == 0 &&
              reference_connected_root(c, std::nullopt).at("model_index").is_null(),
          "ordinary model root is itself and absent object has no connected root");
    ReferenceSearchQuery q;
    q.stored_file_reference = u"Target";
    q.lookup_file_reference = u"Target";
    q.runtime_flags = 0x1000;
    q.model_id = 7;
    q.equal = [](const std::u16string &x, const std::u16string &y) { return x == y; };
    auto r = match_reference_model(q, c, 1);
    check(r.at("matched") == true && r.at("root_model_index") == 2,
          "graph matching reads connected model fields without duplicating them on the reference");
    check(match_reference_model(q, c.models[1]).at("reason") ==
              "connected_root_graph_context_required",
          "flat matcher does not reinterpret an explicit graph binding as local root metadata");
    c.models[1].file = NativeFileReference{u"stale", u"stale"};
    c.models[1].model_id = 123;
    check(match_reference_model(q, c, 1).at("matched") == true,
          "explicit root identity is authoritative over the reference's older flat projection");
    r = search_reference_descendants(q, c);
    check(r.at("matched") == true && r.at("matching_model_index") == 1 &&
              r.at("model_match").at("root_model_index") == 2 &&
              r.at("matching_path") == Json::array({0, 1}),
          "descendant search reports source reference identity while matching its target model");
    root.model_id = 8;
    check(match_reference_model(q, c, 1).at("matched") == false &&
              match_reference_model(q, c, 3).at("matched") == false,
          "two references see the same changed target metadata instead of stale copies");
    r = search_reference_descendants(q, c);
    check(r.at("status") == "resolved" && r.at("matched") == false &&
              r.at("visited_model_indices") == Json::array({0, 1, 3}),
          "active traversal stays on reference lists and preserves both shared-target paths");
    q.runtime_flags = 0;
    q.model_name = u"Shared";
    check(match_reference_model(q, c, 3).at("matched") == true,
          "primary-name comparison uses connected root name");
    q.model_name = u"";
    check(match_reference_model(q, c, 1).at("matched") == true,
          "default-model comparison uses connected root file and default state");
    root.is_default_model = false;
    check(match_reference_model(q, c, 1).at("matched") == false,
          "default flag is borrowed from the target rather than assumed for a shared model");
    root.valid = false;
    q.runtime_flags = 0x1000;
    q.model_id = 8;
    check(reference_connected_root(c, 1).at("model_index") == 2 &&
              match_reference_model(q, c, 1).at("matched") == true,
          "native root getter does not add a second object validity test on the returned pointer");
    root.valid = true;
    c.models[1].root_present = false;
    check(reference_connected_root(c, 1).at("reason") == "conflicting_connected_root_presence",
          "explicit root absence cannot coexist with a nonnull binding");
    c.models[1].root_present.reset();
    c.models[1].connected_root = 99;
    check(match_reference_model(q, c, 1).at("reason") == "connected_root_index_out_of_range",
          "unknown target identity is not an absent root");
    c.models[1].valid = false;
    check(reference_connected_root(c, 1).at("model_index").is_null(),
          "invalid source object skips unused malformed connected-root data");
    q.stored_file_reference = u"";
    q.model_id = 0xfffffffeu;
    check(match_reference_model(q, c, 1).at("matched") == true,
          "invalid source keeps native minus-two model sentinel in graph matching");
    c.models[1].valid = true;
    c.models[1].connected_root.reset();
    q.model_id = UINT32_MAX;
    check(match_reference_model(q, c, 1).at("matched") == true &&
              reference_connected_root(c, 1).at("model_index").is_null(),
          "known null root has its separate minus-one model sentinel");
    c.models[1].connected_root_known = false;
    check(reference_connected_root(c, 1).at("reason") == "connected_root_identity_required",
          "unresolved connection cannot be inferred from file/model flat fields");
    c.models[1].root_present = false;
    check(reference_connected_root(c, 1).at("model_index").is_null(),
          "known root absence is enough to answer identity query without a binding");
    c.models[0].connected_root_known = true;
    c.models[0].connected_root = 2;
    check(reference_connected_root(c, 0).at("reason") ==
              "ordinary_model_has_nonself_connected_root",
          "ordinary model self-root cannot be replaced by another graph object");
    c.models[0].connected_root = 0;
    check(reference_connected_root(c, 0).at("model_index") == 0,
          "explicit model self-root binding is accepted");
    check(reference_connected_root(c, 88).at("status") == "unresolved",
          "out-of-range source object is not a null native pointer");

    // The target resolver returns graph identity, which can be recorded as an
    // established binding. This is not an invocation of native attachment hooks.
    ReferenceModelQuery target_query;
    target_query.secondary_flags = 0;
    target_query.runtime_flags = 0x1000;
    target_query.explicit_model_id = 8;
    NativeModelLookupContext lookup;
    lookup.model_cache[8] = 2;
    const auto selected = resolve_reference_model(target_query, {}, lookup);
    c.models[1].connected_root_known = true;
    c.models[1].connected_root = selected.at("object_index").get<std::size_t>();
    c.models[1].root_present.reset();
    q.stored_file_reference = u"Target";
    q.model_id = 8;
    check(match_reference_model(q, c, 1).at("matched") == true,
          "model-resolution identity composes with graph matching without copying model payload");

    using Op = NativeModelReferenceOperation;
    using List = std::vector<std::optional<std::size_t>>;
    auto listcheck = [&](const List &before, std::optional<std::size_t> key, Op op,
                         const Json &after, bool changed, const char *why) {
        const auto value = native_model_reference_list(before, key, op);
        check(value.at("status") == "resolved" && value.at("references") == after &&
                  value.at("changed") == changed,
              why);
    };
    listcheck({0, 2}, 3, Op::add, Json::array({0, 2, 3}), true,
              "new back-reference appends in source order");
    listcheck({0, 2, 2}, 2, Op::add, Json::array({0, 2, 2}), false,
              "add checks identity without normalizing existing duplicates");
    listcheck({0, 2, 3}, 2, Op::remove, Json::array({0, 3}), true,
              "ordinary unregister preserves other pointer order");
    listcheck({0, 2, 3}, 3, Op::remove, Json::array({0, 2}), true,
              "last back-reference can be erased");
    listcheck({0, 2}, 3, Op::remove, Json::array({0, 2}), false,
              "absent unregister leaves list unchanged");
    listcheck({}, 0, Op::remove, Json::array(), false, "empty unregister is a no-op");
    listcheck({}, std::nullopt, Op::add, Json::array({nullptr}), true,
              "null pointer identity remains distinct from index zero");
    listcheck({std::nullopt, 0}, std::nullopt, Op::remove, Json::array({0}), true,
              "known null pointer can be compared and removed");
    listcheck({1, 2, 1, 3, 1}, 1, Op::remove, Json::array({2, 3, 3, 1}), true,
              "native compaction followed by one erase preserves its exact duplicate-input result");
    listcheck({2, 2, 2}, 2, Op::remove, Json::array({2, 2}), true,
              "all-equal invalid duplicate list shrinks by only one native entry");
    check(native_model_reference_list({}, 0, static_cast<Op>(99)).at("status") == "unresolved",
          "unknown list operation is diagnosed");
    const auto baseline = search_reference_descendants(q, c);
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(
            std::async(std::launch::async, [&] { return search_reference_descendants(q, c); }));
    for (auto &job : jobs)
        check(job.get() == baseline,
              "connected target metadata is borrowed for concurrent graph searches");
    return checks;
}
