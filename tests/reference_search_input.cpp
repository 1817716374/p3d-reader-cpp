#include "internal.hpp"
#include <p3d/reference_search.hpp>
#include <future>

unsigned reference_search_input_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto put = [](Bytes &b, std::size_t at, std::uint64_t value, unsigned n) {
        for (unsigned i = 0; i < n; ++i)
            b.at(at + i) = static_cast<std::uint8_t>(value >> (i * 8));
    };
    auto linkage = [&](unsigned app, Bytes payload, bool user = true) {
        if (payload.size() % 2)
            payload.push_back(0);
        Bytes bytes(4 + payload.size());
        put(bytes, 0, (user ? 0x1000 : 0) | (bytes.size() / 2 - 1), 2);
        put(bytes, 2, app, 2);
        std::copy(payload.begin(), payload.end(), bytes.begin() + 4);
        return bytes;
    };
    auto id_link = [&](unsigned version, unsigned id) {
        Bytes bytes(8);
        put(bytes, 0, version, 2);
        put(bytes, 4, id, 4);
        return linkage(0x5717, bytes);
    };
    auto numeric = [&](unsigned key, std::uint64_t value, unsigned count = 1) {
        Bytes bytes(16);
        put(bytes, 0, key, 2);
        put(bytes, 4, count, 4);
        put(bytes, 8, value, 8);
        return linkage(0x56d5, bytes);
    };
    auto name_link = [&](unsigned key, std::u16string name) {
        Bytes bytes(10 + 2 * name.size());
        put(bytes, 0, key, 2);
        put(bytes, 4, bytes.size() - 8, 4);
        put(bytes, 8, 0xfeff, 2);
        for (std::size_t i = 0; i < name.size(); ++i)
            put(bytes, 10 + 2 * i, name[i], 2);
        return linkage(0x56d2, bytes);
    };
    auto wire = [&](const std::vector<Bytes> &links, unsigned secondary = 0, unsigned primary = 0) {
        Bytes bytes(372);
        put(bytes, 4, 13, 2);
        put(bytes, 12, 184, 4);
        put(bytes, 60, primary, 4);
        put(bytes, 64, secondary, 4);
        // Modern discriminator; all-zero legacy-sized fields are deliberate.
        put(bytes, 160, 1, 4);
        for (auto at : {220, 252, 284, 292})
            put(bytes, at, UINT64_C(0x3ff0000000000000), 8);
        for (const auto &link : links)
            bytes.insert(bytes.end(), link.begin(), link.end());
        put(bytes, 8, (bytes.size() - 4) / 2, 4);
        return bytes;
    };
    auto record = [&](const std::vector<Bytes> &links, unsigned secondary = 0,
                      unsigned primary = 0) {
        return parse_native(wire(links, secondary, primary)).at(0);
    };
    auto runtime = [](const Json &record) -> const Json & {
        return record.at("reference_target").at("initial_runtime_state");
    };
    auto rejected = [&](const Json &r) {
        try {
            (void)initial_reference_search_query(r, {});
            return false;
        } catch (const std::exception &) {
            return true;
        }
    };
    auto r = record({}, 0xffffffff, 0xffffffff);
    check(runtime(r).at("runtime_flags") == 0 && runtime(r).at("model_id") == 0 &&
              runtime(r).at("alternate_file_reference") == "",
          "initial runtime word is cleared, not copied from either source flags");
    auto query = initial_reference_search_query(r, {u"stored", u"prepared"});
    check(query.runtime_flags == 0 && query.model_id == 0 && query.model_name == u"" &&
              query.stored_file_reference == u"stored" &&
              query.lookup_file_reference == u"prepared",
          "initial search input uses explicitly prepared file specification and empty main name");
    Bytes mask(22);
    put(mask, 2, 0xabcd, 2);
    put(mask, 4, 0xffffffff, 4);
    put(mask, 8, 0x80000000, 4);
    put(mask, 12, 37, 4);
    put(mask, 16, 123, 4);
    put(mask, 20, 0x9876, 2);
    const auto first_mask = linkage(0x56e1, mask);
    for (unsigned value = 0; value < 16; ++value) {
        r = record({id_link(3, 90), id_link(0, 0xf1234567), id_link(0, 99),
                    numeric(19, value | UINT64_C(0x87654321abcd0000)), first_mask});
        const auto &state = runtime(r);
        check(state.at("runtime_flags") == (0x1001u | (value << 18)) &&
                  state.at("model_id") == 0xf1234567u,
              "initial runtime combines first supported ID, mask and low four numeric bits");
        query = initial_reference_search_query(r, {});
        check(query.runtime_flags == (0x1001u | (value << 18)) && query.model_id == 0xf1234567u,
              "public search input preserves complete uint32 model ID and runtime flags");
    }
    const auto &selected = runtime(r).at("mask_link");
    check(selected.at("selected_linkage_index") == 4 && selected.at("control_word") == 0xabcd &&
              selected.at("word_values") == Json::array({0xffffffffu, 0x80000000u, 37u, 123u}) &&
              bytesof(selected.at("trailing_storage")) == Bytes({0x76, 0x98}),
          "reference mask retains all four uint32 words, control and trailing bytes");
    check(selected.at("source_offset") == r.at("links")[4].at("offset"),
          "selected mask source points to actual native linkage");
    r = record({numeric(19, 15), numeric(19, 2), numeric(19, 12, 0), numeric(20, 9)});
    check(runtime(r).at("runtime_flags") == (2u << 18),
          "last accepted query state wins while empty and different-key records do not override");
    auto ignored_id = id_link(0, 55);
    ignored_id[1] &= ~0x10;
    ignored_id.resize(8); // Non-user linkages occupy exactly eight bytes on disk.
    r = record({ignored_id, id_link(8, 7)});
    check(runtime(r).at("model_id") == 0 && runtime(r).at("runtime_flags") == 0,
          "non-user and unsupported ID records do not enable runtime model-ID selection");
    auto other_mask = mask;
    put(other_mask, 0, 1, 2);
    auto later_mask = mask;
    put(later_mask, 4, 12, 4);
    r = record({linkage(0x56e1, other_mask), first_mask, linkage(0x56e1, later_mask)});
    check(runtime(r).at("mask_link").at("selected_linkage_index") == 1 &&
              runtime(r).at("mask_link").at("word_values")[0] == 0xffffffffu,
          "mask selection skips other keys and stops at the first zero key");
    r = record({linkage(0x56e1, Bytes(2)), first_mask});
    check(runtime(r).at("status") == "not_evaluated" &&
              runtime(r).at("mask_link").at("selected_linkage_index") == 0 &&
              !runtime(r).contains("runtime_flags") && rejected(r),
          "truncated first selected mask prevents a falsely resolved word and later substitution");
    r = record({linkage(0x56e1, Bytes{})});
    check(runtime(r).at("status") == "not_evaluated" &&
              runtime(r).at("mask_link").at("status") == "not_evaluated" && rejected(r),
          "unknown mask key cannot be treated as absent");
    r = record({linkage(0x56e1, Bytes(4), false), first_mask});
    check(runtime(r).at("status") == "not_evaluated" &&
              runtime(r).at("mask_link").at("selected_linkage_index") == 0,
          "non-user zero-key mask is selected but cross-linkage value reads are not fabricated");
    r = record({linkage(0x56e1, Bytes{1, 0, 0, 0}, false), first_mask});
    check(runtime(r).at("mask_link").at("selected_linkage_index") == 1,
          "non-user other-key mask does not consume unrelated words");
    r = record({first_mask, linkage(0x56e1, Bytes{})});
    check(runtime(r).at("runtime_flags") == 1,
          "unvisited mask suffix does not invalidate an already selected first mask");
    r = record({linkage(0x5717, Bytes(2))});
    check(rejected(r), "truncated selected explicit model ID cannot silently become name search");
    r = record({numeric(19, 1), linkage(0x56d5, Bytes(2, 19))});
    // The two-byte payload has key 0x1313, and is never read as query-state data.
    check(runtime(r).at("runtime_flags") == (1u << 18),
          "unrelated numeric key is not required to contain a state value");
    r = record({linkage(0x56d5, Bytes{19, 0})});
    check(rejected(r), "truncated selected query-state linkage cannot yield initial flags");

    const auto primary = name_link(21, u"main"), alternate = name_link(36, u"alternate");
    r = record({primary, alternate, id_link(0, 7)}, 0x28000);
    check(r.at("reference_target").at("model_selection").at("kind") == "file_default_model",
          "ordinary target selector still honors its own default-model source bit");
    query = initial_reference_search_query(r, {u"file", u""});
    ReferenceSearchModel m;
    m.object_dispatch = ReferenceObjectDispatch::model;
    m.file = NativeFileReference{u"file", u""};
    m.model_id = 7;
    m.is_default_model = false;
    check(match_reference_model(query, m).at("matched") == true,
          "descendant matcher uses explicit runtime ID even when ordinary target requests default");
    r = record({primary, alternate, name_link(35, u"other-file")}, 0x20000);
    query = initial_reference_search_query(r, {u"file", u""});
    m.model_name = u"main";
    check(r.at("reference_target").at("model_selection").at("name") == "alternate" &&
              query.model_name == u"main" &&
              match_reference_model(query, m).at("matched") == true &&
              runtime(r).at("alternate_file_reference") == "",
          "search uses primary name; alternate name and key35 cannot replace query or file slot");
    Bytes invalid_text{21, 0, 0, 0, 100, 0, 0, 0};
    r = record({linkage(0x56d2, invalid_text), id_link(0, 7)});
    query = initial_reference_search_query(r, {u"file", u""});
    check(!query.model_name && match_reference_model(query, m).at("matched") == true,
          "unresolved primary name remains optional when explicit ID decides model match");
    r = record({linkage(0x56d2, invalid_text)});
    query = initial_reference_search_query(r, {u"file", u""});
    check(match_reference_model(query, m).at("status") == "unresolved",
          "unresolved required name never falls back to the default model");
    const std::u16string unpaired(1, char16_t(0xdc00));
    r = record({name_link(21, unpaired)});
    query = initial_reference_search_query(r, {u"file", u""});
    m.model_name = unpaired;
    check(query.model_name == unpaired && match_reference_model(query, m).at("matched") == true,
          "native name identity retains UTF16 code units even without valid JSON text");
    r = record({primary});
    unsigned comparisons = 0;
    query = initial_reference_search_query(r, {u"FILE", u""}, [&](const auto &a, const auto &b) {
        ++comparisons;
        return a == u"file" && b == u"FILE";
    });
    m.model_name = u"main";
    check(match_reference_model(query, m).at("matched") == true && comparisons == 1,
          "caller native name comparer is preserved by search-input construction");
    auto wrong = r;
    wrong["element_type"] = 47;
    check(rejected(wrong), "initial query refuses non-reference record");
    wrong = r;
    wrong["reference_input"]["status"] = "invalid";
    check(rejected(wrong), "invalid reference base cannot construct an initial search query");
    auto baseline = record({primary, numeric(19, 13), first_mask, id_link(0, 7)});
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(std::async(std::launch::async, [&] {
            return record({primary, numeric(19, 13), first_mask, id_link(0, 7)});
        }));
    for (auto &job : jobs)
        check(job.get() == baseline, "initial loaded state parsing is independent across threads");
    check(runtime(record({})).at("runtime_flags") == 0,
          "another input resets all runtime bits rather than inheriting previous record state");
    return checks;
}
