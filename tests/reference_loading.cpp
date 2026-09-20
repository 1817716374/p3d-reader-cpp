#include "internal.hpp"
#include <p3d/reference_loading.hpp>
#include <p3d/reference_recursion.hpp>
#include <future>

unsigned reference_search_tests();

unsigned reference_loading_tests() {
    using namespace p3d;
    unsigned checks = reference_search_tests();
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    ReferenceHostFilterContext h;
    check(reference_host_loading_filter(0, 0, h).at("status") == "unresolved",
          "host validity is not inferred from missing context");
    h.host_valid = false;
    check(reference_host_loading_filter(0, 0x80000, h).at("excluded") == false,
          "invalid host exits before secondary bit 19");
    h.host_valid = true;
    h.host_reference_present = false;
    check(reference_host_loading_filter(0, 0x80000, h).at("excluded") == false,
          "ordinary host without reference exits before parent reads");
    h.host_reference_present = true;
    h.host_reference_valid = false;
    check(reference_host_loading_filter(0, 0x80000, h).at("excluded") == false,
          "invalid reference has no usable parent");
    h.host_reference_valid = true;
    h.host_reference_parent_present = false;
    check(reference_host_loading_filter(0, 0x80000, h).at("excluded") == false,
          "absent host-reference parent suppresses this exclusion");
    h.host_reference_parent_present = true;
    h.host_reference_secondary_flags = 0x400000;
    check(reference_host_loading_filter(0, 0x80000, h).at("excluded") == false,
          "host bit 22 prevents exclusion without a file comparison");
    h.host_reference_secondary_flags = 0;
    check(reference_host_loading_filter(0, 0x80000, h).at("reason") ==
              "host_file_identity_comparison_required",
          "unknown file identity is not equality");
    h.same_file_object = false;
    check(reference_host_loading_filter(0, 0x80000, h).at("excluded") == true &&
              reference_host_loading_filter(0, 0, h).at("excluded") == false,
          "distinct file identities select candidate bit 19 without parent model fields");
    h.same_file_object = true;
    h.parent_root_present = false;
    check(reference_host_loading_filter(0, 0x80000, h).at("excluded") == true,
          "equal file identities alone do not provide the model-kind exception");
    h.parent_root_present = true;
    check(reference_host_loading_filter(0, 0x80000, h).at("status") == "unresolved",
          "same file and existing root require the actual model kind");
    for (auto kind : {-1, 0, 1, 2, 3, INT32_MAX}) {
        h.parent_model_kind = kind;
        check(reference_host_loading_filter(0, 0x80000, h).at("excluded") == (kind != 1),
              "only native parent model kind one suppresses bit-19 exclusion");
    }
    h = {};
    h.host_valid = h.host_reference_present = h.host_reference_valid =
        h.host_reference_parent_present = true;
    h.host_models = {{false, {}}};
    check(reference_host_loading_filter(0x10000000, 0x80000, h).at("excluded") == false,
          "bit-28 branch treats a null model root as positive native query 9");
    h.host_models = {{true, 0x200u}};
    check(reference_host_loading_filter(0x10000000, 0x80000, h).at("excluded") == false,
          "first host flag 9 avoids all second-host and ordinary-branch fields");
    h.host_models = {{true, 0u}};
    check(reference_host_loading_filter(0x10000000, 0x80000, h).at("status") == "unresolved",
          "one negative host query is insufficient with unknown parent presence");
    h.complete_host_chain = true;
    check(reference_host_loading_filter(0x10000000, 0x80000, h).at("excluded") == true,
          "known chain end after one host selects candidate bit 19");
    h.complete_host_chain = false;
    h.host_models.push_back({true, 0x200u});
    check(reference_host_loading_filter(0x10000000, 0x80000, h).at("excluded") == false,
          "second host participates in model query 9");
    h.host_models[1].root_model_flags = 0;
    h.host_models.push_back({true, 0x200u});
    auto r = reference_host_loading_filter(0x10000000, 0x80000, h);
    check(r.at("excluded") == true && r.at("examined_host_models") == 2,
          "third host flag cannot alter the native two-host window");
    h.host_models[1].root_present.reset();
    check(reference_host_loading_filter(0x10000000, 0x80000, h).at("status") == "unresolved",
          "unknown root is different from null root");

    ReferenceLoadingDecisionContext d;
    auto expect = [&](std::uint16_t type, int state, int code, bool transferred, const char *list) {
        const auto value = reference_loading_decision(type, d);
        check(value.at("status") == "resolved" && value.at("loading_state") == state &&
                  value.at("native_return") == code &&
                  value.at("reference_transferred") == transferred &&
                  value.at("requested_list") == list,
              "native loading state, return code, ownership and list destination agree");
    };
    expect(47, 4, 1, false, "none");
    d.require_source_bit14 = true;
    d.source_primary_flags = 0;
    expect(13, 7, 0, false, "none");
    d.source_primary_flags.reset();
    check(reference_loading_decision(13, d).at("status") == "unresolved",
          "source flags are required when bit-14 filtering is requested");
    d.require_source_bit14 = false;
    d.native_input_status = -37;
    expect(13, 4, -37, false, "none");
    d.native_input_status = 0;
    d.loaded_primary_flags = 0x80;
    expect(13, 5, 0, false, "none");
    d.loaded_primary_flags = 0;
    d.host_filter_excluded = true;
    expect(13, 6, 0, false, "none");
    d.host_filter_excluded = false;
    d.source_runtime_deleted = true;
    expect(13, 9, 0, true, "secondary");
    d.source_runtime_deleted = false;
    d.descendant_filter_required = true;
    check(reference_loading_decision(13, d).at("reason") == "descendant_match_result_required",
          "enabled descendant search cannot assume a negative result");
    d.descendant_match = true;
    d.keep_descendant_match = false;
    expect(13, 3, 0, false, "none");
    d.keep_descendant_match = true;
    expect(13, 9, 0, true, "secondary");
    d.descendant_match = false;
    d.ancestor_repeated = true;
    d.keep_ancestor_repetition = false;
    expect(13, 8, 0, false, "none");
    d.keep_ancestor_repetition = true;
    expect(13, 9, 0, true, "secondary");
    d.ancestor_repeated = false;
    d.keep_ancestor_repetition.reset();
    expect(13, 1, 0, true, "active");
    d.descendant_filter_required = false;
    d.descendant_match.reset();
    d.keep_descendant_match.reset();
    expect(13, 1, 0, true, "active");
    // Every missing operation needed on the ordinary success path must prevent
    // publishing a definitive loading state. Irrelevant later values are not needed.
    for (unsigned field = 0; field < 6; ++field) {
        auto missing = d;
        if (field == 0)
            missing.native_input_status.reset();
        if (field == 1)
            missing.loaded_primary_flags.reset();
        if (field == 2)
            missing.host_filter_excluded.reset();
        if (field == 3)
            missing.source_runtime_deleted.reset();
        if (field == 4)
            missing.descendant_filter_required.reset();
        if (field == 5)
            missing.ancestor_repeated.reset();
        auto value = reference_loading_decision(13, missing);
        check(value.at("status") == "unresolved" && !value.contains("loading_state"),
              "unknown loading operation does not leak an apparent success state");
    }

    ReferenceLinkInsertionContext insert;
    insert.runtime_byte_4e9_nonzero = true;
    check(!reference_link_insertion(ReferenceLinkList::active, insert).at("append").get<bool>(),
          "runtime byte 4e9 short-circuits the second byte and the list");
    insert.runtime_byte_4e9_nonzero = false;
    insert.runtime_byte_4e8_nonzero = true;
    check(!reference_link_insertion(ReferenceLinkList::secondary, insert).at("append").get<bool>(),
          "runtime byte 4e8 suppresses secondary insertion");
    insert.runtime_byte_4e8_nonzero = false;
    check(reference_link_insertion(ReferenceLinkList::active, insert).at("append") == true,
          "active append needs no keys or existing list and never deduplicates");
    check(reference_link_insertion(ReferenceLinkList::secondary, insert).at("status") ==
              "unresolved",
          "unknown remaining secondary entries are not an empty list");
    insert.existing_entries_complete = true;
    check(reference_link_insertion(ReferenceLinkList::secondary, insert).at("append") == true,
          "empty secondary list appends without reading new key");
    insert.new_secondary_key = 0;
    insert.existing_entries = {{true, 0}, {false, {}}};
    insert.existing_entries_complete = false;
    r = reference_link_insertion(ReferenceLinkList::secondary, insert);
    check(r.at("append") == false && r.at("matching_entry_index") == 0 &&
              r.at("examined_entries") == 1,
          "zero key match stops before a null slot and unknown list suffix");
    insert.new_secondary_key = UINT32_MAX;
    check(reference_link_insertion(ReferenceLinkList::secondary, insert).at("reason") ==
              "null_secondary_list_entry",
          "visited null entry is not silently discarded");
    check(reference_link_insertion(ReferenceLinkList::active, insert).at("append") == true,
          "active list does not inspect even malformed existing entries");
    insert.existing_entries = {{true, 0x80000000u}, {true, UINT32_MAX}, {true, UINT32_MAX}};
    r = reference_link_insertion(ReferenceLinkList::secondary, insert);
    check(r.at("matching_entry_index") == 1 && r.at("append") == false,
          "secondary matching compares complete uint32 keys and selects the first duplicate");
    insert.new_secondary_key = 4;
    check(reference_link_insertion(ReferenceLinkList::secondary, insert).at("status") ==
              "unresolved",
          "nonmatching prefix does not prove no duplicate exists");
    insert.existing_entries_complete = true;
    check(reference_link_insertion(ReferenceLinkList::secondary, insert).at("append") == true,
          "complete nonmatching secondary list allows append");
    check(reference_link_insertion(static_cast<ReferenceLinkList>(7), insert).at("status") ==
              "unresolved",
          "invalid list discriminator does not default to an active append");

    // Integrate the actual predicate outputs with routing and insertion. A
    // retained reference can be returned even if insertion finds a duplicate.
    h = {};
    h.host_valid = false;
    d.host_filter_excluded = reference_host_loading_filter(0, 0, h).at("excluded").get<bool>();
    ReferenceRepetitionContext repetition;
    repetition.native_setting_enabled = repetition.host_file_fallback_gate = false;
    d.ancestor_repeated =
        reference_ancestor_repetition(0, 0, nullptr, repetition).at("repeated").get<bool>();
    r = reference_loading_decision(13, d);
    check(r.at("loading_state") == 1 &&
              reference_link_insertion(ReferenceLinkList::active, insert).at("append") == true,
          "resolved predicates compose into normal active-reference insertion");
    d.source_runtime_deleted = true;
    insert.new_secondary_key = UINT32_MAX;
    r = reference_loading_decision(13, d);
    check(r.at("reference_transferred") == true && r.at("requested_list") == "secondary" &&
              reference_link_insertion(ReferenceLinkList::secondary, insert).at("append") == false,
          "reference transfer survives secondary duplicate rejection");
    const auto baseline = reference_loading_decision(13, d);
    std::vector<std::future<Json>> jobs;
    for (int i = 0; i < 4; ++i)
        jobs.push_back(
            std::async(std::launch::async, [d] { return reference_loading_decision(13, d); }));
    for (auto &job : jobs)
        check(job.get() == baseline, "loading decisions have no shared mutable state");
    return checks;
}
