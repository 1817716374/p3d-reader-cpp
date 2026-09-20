#include "internal.hpp"
#include <p3d/reference_search.hpp>
#include <p3d/reference_loading.hpp>
#include <future>
#include <random>

unsigned reference_link_request_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto registry = [](std::initializer_list<std::uint64_t> ids) {
        Json entries = Json::array();
        for (auto id : ids)
            entries.push_back({{"id", id}});
        return Json{{"status", "resolved"}, {"registry", entries}};
    };
    ReferenceSearchContext c;
    c.models.resize(4);
    c.models[0].active_links = {1, 2};
    c.models[0].active_links_complete = true;
    c.models[0].secondary_links = {3};
    c.models[0].secondary_links_complete = true;
    c.models[1].reference_link_id = 0x100000001ull;
    c.models[2].reference_link_id = 1;
    c.models[3].reference_link_id = 1;
    auto registered = registry({1, 0x100000001ull});
    auto r = reference_link_request(c, 0, 1, registered);
    check(r.at("action") == "reuse_reference" && r.at("object_index") == 2 &&
              r.at("source_list") == "active" && r.at("entry_index") == 1 &&
              r.at("examined_secondary_entries") == 0 && r.at("native_return") == 0 &&
              !r.at("requires_native_input").get<bool>() && !r.contains("loading_state"),
          "single request compares full assigned uint64 ID and returns active object identity");
    r = reference_link_request(c, 0, 0x100000001ull, registered);
    check(r.at("object_index") == 1, "upper ID bits remain significant");
    auto absent = registry({});
    r = reference_link_request(c, 0, 1, absent);
    check(r.at("action") == "input_reference" && r.at("insert_registry_entry") == true &&
              r.at("registry_state_before_input") == 0 &&
              r.at("registry_source_action") == "capture_input_source" &&
              r.at("examined_active_entries") == 0 && r.at("examined_secondary_entries") == 0,
          "unregistered ID inputs directly even if an existing list contains an equal ID");
    c.models[1].reference_link_id = 1;
    c.models[1].valid = false;
    c.models[1].native_kind = 55;
    c.models[1].reference_runtime_flags = 0xffffffffu;
    c.models[1].root_present = false;
    c.models[0].active_links.push_back(std::nullopt);
    c.models[0].active_links_complete = false;
    c.models[0].secondary_links = {std::nullopt};
    r = reference_link_request(c, 0, 1, registered);
    check(r.at("object_index") == 1 && r.at("entry_index") == 0 &&
              r.at("examined_active_entries") == 1,
          "first same-ID pointer wins without inspecting kind, validity, root, flags or suffix");
    for (int state : {0, 1, 3, 4, 5, 6, 7, 8, 9}) {
        auto states = registered;
        states["registry"][0]["initial_loading_state"] = state;
        states["registry"][0]["source"] = "different_source";
        check(reference_link_request(c, 0, 1, states) == r,
              "registry state and retained source do not change existing-pointer reuse");
    }
    c.models[0].active_links = {2};
    c.models[0].active_links_complete = true;
    c.models[0].secondary_links = {3, 1};
    c.models[3].reference_link_id = 0;
    c.models[2].reference_link_id = 0;
    r = reference_link_request(c, 0, 0, registry({0}));
    check(r.at("object_index") == 3 && r.at("source_list") == "secondary" &&
              r.at("examined_active_entries") == 0,
          "registered zero skips active list but can reuse first zero-ID secondary entry");
    c.models[0].valid = false;
    c.models[0].active_links = {std::nullopt};
    c.models[3].reference_link_id = 1;
    r = reference_link_request(c, 0, 1, registered);
    check(r.at("object_index") == 3 && r.at("examined_active_entries") == 0,
          "invalid host skips active lookup but does not suppress secondary lookup");
    c.models[0].secondary_list_known_absent = true;
    c.models[0].secondary_links.clear();
    r = reference_link_request(c, 0, 1, registered);
    check(r.at("action") == "input_reference" && !r.at("insert_registry_entry").get<bool>() &&
              r.at("registry_source_action") == "preserve" &&
              !r.contains("registry_state_before_input"),
          "registered miss preserves old source and delegates state changes to ordinary input");
    c.models[0].valid = true;
    check(reference_link_request(c, 0, 1, registered).at("reason") ==
              "null_reference_request_list_entry",
          "null examined list pointer is diagnosed without inventing a skip");
    c.models[0].active_links = {99};
    check(reference_link_request(c, 0, 1, registered).at("reason") ==
              "request_list_object_index_out_of_range",
          "out-of-range object identity is diagnosed");
    c.models[0].active_links = {2};
    c.models[2].reference_link_id.reset();
    check(reference_link_request(c, 0, 1, registered).at("reason") ==
              "loaded_reference_link_id_required",
          "unknown loaded ID cannot be substituted by model ID or vector position");
    c.models[2].reference_link_id = 2;
    c.models[0].active_links_complete = false;
    c.models[0].secondary_list_known_absent = false;
    c.models[0].secondary_links = {3};
    check(reference_link_request(c, 0, 1, registered).at("reason") ==
              "remaining_active_request_links_required",
          "unknown active suffix prevents falling through to a known secondary match");
    c.models[0].active_list_known_absent = true;
    check(reference_link_request(c, 0, 1, registered).at("reason") ==
              "conflicting_active_list_state",
          "absent active list cannot also contain entries");
    c.models[0].active_links.clear();
    c.models[0].secondary_links.clear();
    c.models[0].secondary_links_complete = false;
    check(reference_link_request(c, 0, 1, registered).at("reason") ==
              "remaining_secondary_request_links_required",
          "unknown secondary suffix cannot prove new input is necessary");
    c.models[0].secondary_links = {3};
    c.models[0].secondary_list_known_absent = true;
    check(reference_link_request(c, 0, 1, registered).at("reason") ==
              "conflicting_secondary_list_state",
          "absent secondary list cannot also contain entries");
    check(reference_link_request(c, 99, 1, registered).at("status") == "unresolved",
          "request requires an actual host identity");
    auto bad = registered;
    bad["status"] = "partial";
    check(reference_link_request(c, 0, 1, bad).at("status") == "unresolved",
          "partial registry is not a complete native map");
    check(reference_link_request(c, 0, 1, registry({1, 1})).at("reason") ==
              "duplicate_link_registry_id",
          "ambiguous registry source is not resolved by inventing a duplicate rule");
    for (Json id : {Json(-1), Json(1.5), Json("1")}) {
        bad = registry({1});
        bad["registry"][0]["id"] = id;
        check(reference_link_request(c, 0, 1, bad).at("reason") ==
                  "link_registry_unsigned_id_required",
              "registry IDs require lossless unsigned integers");
    }
    c.models[0].active_links = {std::nullopt};
    c.models[0].secondary_links = {std::nullopt};
    check(reference_link_request(c, 0, UINT64_MAX, absent).at("insert_registry_entry") == true,
          "map miss never reads malformed unused lists, including maximal unsigned ID");

    // Independent literal pointer-list search with fully known inputs. Covers
    // registry miss, zero IDs, invalid hosts, duplicate IDs and both list orders.
    std::mt19937 random(0x1562025);
    for (unsigned trial = 0; trial < 300; ++trial) {
        c.models.assign(10, {});
        auto &host = c.models[0];
        host.valid = random() % 2;
        host.active_links_complete = host.secondary_links_complete = true;
        for (unsigned i = 1; i < 10; ++i)
            c.models[i].reference_link_id = random() % 5;
        for (unsigned i = 0, count = random() % 12; i < count; ++i)
            host.active_links.push_back(1 + random() % 9);
        for (unsigned i = 0, count = random() % 12; i < count; ++i)
            host.secondary_links.push_back(1 + random() % 9);
        const std::uint64_t id = random() % 5;
        const bool exists = random() % 2;
        std::optional<std::size_t> found;
        const char *list = nullptr;
        if (exists && host.valid && id)
            for (auto entry : host.active_links)
                if (c.models[*entry].reference_link_id == id) {
                    found = entry;
                    list = "active";
                    break;
                }
        if (exists && !found)
            for (auto entry : host.secondary_links)
                if (c.models[*entry].reference_link_id == id) {
                    found = entry;
                    list = "secondary";
                    break;
                }
        const auto before_active = host.active_links, before_secondary = host.secondary_links;
        r = reference_link_request(c, 0, id, exists ? registry({id}) : absent);
        check(r.at("status") == "resolved" &&
                  (found ? r.at("object_index") == *found && r.at("source_list") == list &&
                               r.at("action") == "reuse_reference"
                         : r.at("action") == "input_reference" &&
                               r.at("insert_registry_entry") == !exists) &&
                  host.active_links == before_active && host.secondary_links == before_secondary,
              "graph request matches native first-pointer rules without list mutation or "
              "deduplication");
    }
    auto snapshot_source = registry({0, UINT64_MAX, 17});
    const ReferenceLinkRegistryIndex index(snapshot_source);
    snapshot_source["registry"].clear();
    check(index.entry_index(0) == 0 && index.entry_index(UINT64_MAX) == 1 &&
              index.entry_index(17) == 2 && !index.entry_index(1),
          "prepared ID index owns its validated snapshot and preserves original entry positions");
    bool threw = false;
    try {
        (void)ReferenceLinkRegistryIndex(registry({1, 1}));
    } catch (const std::exception &) {
        threw = true;
    }
    check(threw, "prepared index rejects duplicate native map keys");
    const auto baseline = reference_link_request(c, 0, 0, registry({0, UINT64_MAX, 17}));
    check(reference_link_request(c, 0, 0, index) == baseline,
          "prepared and JSON request entry points agree");
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(
            std::async(std::launch::async, [&] { return reference_link_request(c, 0, 0, index); }));
    for (auto &job : jobs)
        check(job.get() == baseline,
              "request evaluation only borrows immutable graph and registry");
    return checks;
}
