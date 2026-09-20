#include "internal.hpp"
#include <p3d/reference_model.hpp>
#include <future>

unsigned reference_model_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto entry = [](std::uint32_t id, std::u16string name, bool excluded = false) {
        return Json{{"model_id", id},
                    {"excluded_from_name_lookup", excluded},
                    {"name",
                     {{"status", "decoded"},
                      {"utf16_code_units", std::vector<std::uint16_t>(name.begin(), name.end())}}}};
    };
    Json directory = {
        {"status", "decoded"},
        {"native_input_status", "decoded"},
        {"entries", Json::array({entry(4, u"A", true), entry(5, u"A"), entry(6, u"A")})}};
    ReferenceModelFileContext file;
    file.default_model_id = 7;
    file.directory = &directory;
    ReferenceModelQuery q;
    q.requested_load_mask = 0xfffffffbu;
    q.secondary_flags = 0x8000;
    auto r = select_reference_model(q, file);
    check(r.at("model_id") == 7 && r.at("load_mask_after_selection") == 0 &&
              r.at("selection") == "forced_file_default",
          "forced default precedes unknown explicit-ID and name state and clears the whole load "
          "mask");
    q.secondary_flags = 0;
    q.runtime_flags = 0x1000;
    q.explicit_model_id = 0xffffffffu;
    r = select_reference_model(q, file);
    check(r.at("model_id") == UINT32_MAX && r.at("signed_model_id") == -1 &&
              r.at("load_mask_after_selection") == 0xfffffffbu,
          "explicit ID keeps bit pattern and load mask without reading name or directory");
    q.runtime_flags = 0;
    q.requested_name = u"";
    r = select_reference_model(q, file);
    check(r.at("model_id") == 7 && r.at("load_mask_after_selection") == 0xfffffffbu &&
              r.at("selection") == "empty_name_file_default",
          "empty-name default does not clear the load mask like forced default");
    q.requested_name = std::u16string({u'\0', u'A'});
    check(select_reference_model(q, file).at("model_id") == 7,
          "first NUL determines whether the requested name is empty");
    q.requested_name = std::u16string({u'A', u'\0', u'B'});
    r = select_reference_model(q, file);
    check(r.at("model_id") == 5 && r.at("directory_lookup").at("entry_index") == 1,
          "ordered name lookup skips excluded entries, stops at NUL, and keeps first duplicate");
    q.requested_name = u"a";
    check(select_reference_model(q, file).at("status") == "unresolved",
          "unknown native wide equality cannot be replaced by a guessed name comparison");
    file.equal = [](const std::u16string &a, const std::u16string &b) {
        return a == b || (a == u"A" && b == u"a");
    };
    check(select_reference_model(q, file).at("model_id") == 5,
          "supplied native comparison participates in ordered model selection");
    q.requested_name = u"missing";
    check(select_reference_model(q, file).at("status") == "missing",
          "known missing model name does not fall back to file default");
    q.requested_name = u"A";
    directory["entries"][1]["model_id"] = 0xfffffffeu;
    check(select_reference_model(q, file).at("reason") == "directory_model_id_minus_two",
          "first matching directory ID minus two rejects selection without searching later "
          "duplicates");
    directory["entries"][1]["model_id"] = 0xffffffffu;
    check(select_reference_model(q, file).at("signed_model_id") == -1,
          "directory ID minus one reaches the special slot lookup");
    directory["entries"][1]["model_id"] = 0x80000000u;
    check(select_reference_model(q, file).at("signed_model_id") == INT32_MIN,
          "other negative IDs are passed to the native lookup rejection step");
    directory["entries"][1]["model_id"] = UINT64_C(0x100000000);
    check(select_reference_model(q, file).at("status") == "unresolved",
          "external directory cannot silently truncate oversized model IDs");
    directory["entries"][1]["model_id"] = 5;
    file.directory = nullptr;
    check(select_reference_model(q, file).at("status") == "unresolved",
          "unknown current directory is not a known name miss");
    file.directory_known_absent = true;
    check(select_reference_model(q, file).at("status") == "missing",
          "known absent directory produces model selection failure");
    file.directory = &directory;
    check(select_reference_model(q, file).at("status") == "unresolved",
          "conflicting directory presence is diagnosed only on the name branch");
    file.directory_known_absent = false;

    NativeModelLookupContext models;
    for (auto id : {0x80000000u, 0xfffffffeu, 0xffffff00u}) {
        r = lookup_native_model(id, models);
        check(r.at("status") == "resolved" && r.at("found") == false &&
                  r.at("source") == "invalid_signed_model_id" && !r.contains("creation_required"),
              "signed ID below minus one returns absent before consulting any file state");
    }
    models.model_cache[-1] = 44;
    check(lookup_native_model(UINT32_MAX, models).at("reason") == "special_model_slot_required",
          "minus one must not fall back to an ordinary cache entry with the same numeric key");
    models.special_model_known = true;
    models.special_model = 0;
    r = lookup_native_model(UINT32_MAX, models);
    check(r.at("object_index") == 0 && r.at("source") == "special_model_slot",
          "known special model preserves object identity zero");
    models.model_cache[0] = 2;
    models.model_cache[5] = 8;
    models.suppressed_model_ids = {0, 5};
    check(lookup_native_model(0, models).at("object_index") == 2 &&
              lookup_native_model(5, models).at("object_index") == 8,
          "positive cache pointer wins before suppressed IDs even in an incomplete cache");
    check(lookup_native_model(6, models).at("reason") == "remaining_model_cache_required",
          "unknown cache absence cannot be treated as a provider creation request");
    models.model_cache[5].reset();
    r = lookup_native_model(5, models);
    check(r.at("source") == "suppressed_model_id" && r.at("found") == false,
          "known null cache entry falls through to the separate suppression registry");
    models.model_cache_complete = true;
    check(lookup_native_model(6, models).at("reason") == "remaining_suppressed_model_ids_required",
          "negative suppression lookup needs a complete suppression registry");
    models.suppressed_model_ids_complete = true;
    r = lookup_native_model(6, models);
    check(r.at("status") == "unresolved" && r.at("creation_required") == true &&
              r.at("reason") == "complete_model_creation_result_required",
          "missing provider result leaves identity unresolved rather than claiming a model miss");
    models.creation_result_known = true;
    models.created_model = 20;
    check(lookup_native_model(6, models).at("object_index") == 20,
          "final creation-helper result supplies target object identity");
    models.created_model.reset();
    check(lookup_native_model(6, models).at("found") == false,
          "known failed creation is distinct from unknown creation");
    models.special_model.reset();
    models.suppressed_model_ids.insert(-1);
    check(lookup_native_model(UINT32_MAX, models).at("source") == "suppressed_model_id",
          "empty special slot also checks the signed minus-one suppression entry");
    models.suppressed_model_ids.clear();
    models.created_model = 21;
    check(lookup_native_model(UINT32_MAX, models).at("object_index") == 21,
          "missing special slot can be supplied by complete creation helper result");

    q.secondary_flags = 0;
    q.runtime_flags = 0;
    q.requested_name = u"A";
    models.model_cache[5] = 12;
    r = resolve_reference_model(q, file, models);
    check(r.at("status") == "matched" && r.at("object_index") == 12 &&
              r.at("attachment_and_loading") == "not_executed" && !r.contains("native_return"),
          "composed selection returns actual cached identity without claiming attachment success");
    q.requested_name = u"missing";
    r = resolve_reference_model(q, file, {});
    check(r.at("status") == "missing" && r.at("native_return") == 0x11010 &&
              r.at("runtime_flags_set_mask") == 0x40 && r.at("primary_flags_set_mask") == 0x200 &&
              !r.contains("lookup"),
          "name failure preserves native model-error code and flag effects without reading caches");
    q.runtime_flags = 0x1000;
    q.explicit_model_id = 0x80000000u;
    check(resolve_reference_model(q, file, {}).at("status") == "missing",
          "invalid signed explicit ID reaches the same connection failure branch");
    q.explicit_model_id = 9;
    check(resolve_reference_model(q, file, {}).at("status") == "unresolved",
          "unknown lookup is not reported as the native missing-model error");

    Json record = {
        {"element_type", 13},
        {"reference_input", {{"status", "decoded"}}},
        {"reference_target",
         {{"initial_loaded_flags", {{"secondary", {{"status", "decoded"}, {"value", 0x8000}}}}}}}};
    auto initial = initial_reference_model_query(record, 7);
    check(initial.secondary_flags == 0x8000 && !initial.runtime_flags &&
              select_reference_model(initial, file).at("load_mask_after_selection") == 0,
          "initial adapter consumes loaded secondary flags and does not require unused runtime "
          "state");
    auto &target = record["reference_target"];
    target["initial_loaded_flags"]["secondary"]["value"] = 0;
    target["initial_runtime_state"] = {
        {"status", "decoded"}, {"runtime_flags", 0x1000}, {"model_id", 99}};
    initial = initial_reference_model_query(record, 3);
    check(initial.explicit_model_id == 99 && !initial.requested_name &&
              initial.requested_load_mask == 3,
          "initial explicit ID does not require unrelated model string slots");
    target["initial_runtime_state"]["runtime_flags"] = 0;
    target["strings"] =
        Json::array({{{"key", 21}, {"status", "decoded"}, {"utf16_code_units", {65, 0, 66}}},
                     {{"key", 36}, {"status", "absent"}, {"utf16_code_units", Json::array()}}});
    check(initial_reference_model_query(record).requested_name == u"A",
          "initial ordinary query uses primary name at its first NUL");
    target["initial_loaded_flags"]["secondary"]["value"] = 0x20000;
    check(initial_reference_model_query(record).requested_name == u"",
          "alternate selector preserves known empty alternate name rather than falling back to "
          "primary");
    target["strings"][1]["status"] = "not_evaluated";
    bool threw = false;
    try {
        (void)initial_reference_model_query(record);
    } catch (const std::exception &) {
        threw = true;
    }
    check(threw, "unknown selected source name must not become default-model selection");
    q.runtime_flags = 0x1000;
    models.model_cache[9] = 17;
    const auto baseline = resolve_reference_model(q, file, models);
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(std::async(std::launch::async,
                                  [&] { return resolve_reference_model(q, file, models); }));
    for (auto &job : jobs)
        check(job.get() == baseline, "selection and target lookup borrow immutable shared context");
    return checks;
}
