#include "internal.hpp"
#include <p3d/layer_lookup.hpp>

using namespace p3d;
namespace {
template <class T> void put(Bytes &bytes, T value) {
    const auto *first = reinterpret_cast<const std::uint8_t *>(&value);
    bytes.insert(bytes.end(), first, first + sizeof(value));
}
} // namespace
unsigned layer_lookup_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *message) {
        require(value, message);
        ++checks;
    };
    auto set = [](Bytes &b, std::size_t offset, auto value) {
        std::memcpy(b.data() + offset, &value, sizeof(value));
    };
    auto layer = [&](std::uint32_t id, std::u16string name) {
        Bytes body(108, 0), text, payload;
        set(body, 4, std::uint16_t(49));
        set(body, 6, std::uint16_t(0x80));
        set(body, 8, std::uint32_t(52));
        set(body, 16, std::uint32_t(1));
        set(body, 36, id);
        set(body, 44, std::uint16_t(6));
        put<std::uint16_t>(text, 0xfeff);
        for (auto c : name)
            put<std::uint16_t>(text, c);
        put<std::uint32_t>(payload, 1);
        put<std::uint32_t>(payload, std::uint32_t(text.size()));
        payload.insert(payload.end(), text.begin(), text.end());
        set(body, 8, std::uint32_t((108 + payload.size()) / 2));
        return Json{{"data", rawbytes(body)},
                    {"offset", 0},
                    {"stream", {"system"}},
                    {"links",
                     {{{"app", 0x56d2},
                       {"header", 0x1000 | ((4 + payload.size()) / 2 - 1)},
                       {"offset", 108},
                       {"payload", rawbytes(payload)}}}}};
    };
    auto records = [&](Json children) {
        Bytes body(68, 0);
        set(body, 4, std::uint16_t(10));
        set(body, 8, std::uint32_t(32));
        set(body, 16, std::uint32_t(1));
        set(body, 36, std::uint32_t(children.size()));
        Json out = Json::array({{{"data", rawbytes(body)},
                                 {"offset", 0},
                                 {"stream", {"system"}},
                                 {"links", Json::array()}}});
        std::uint64_t offset = 68;
        for (auto child : children) {
            child["offset"] = offset;
            offset += std::uint64_t(Reader(bytesof(child.at("data"))).at<std::uint32_t>(8)) * 2 + 4;
            out.push_back(std::move(child));
        }
        return out;
    };
    auto lower = [](char16_t c) { return c >= u'A' && c <= u'Z' ? char16_t(c + 32) : c; };
    NativeLayerNameContext text;
    text.lowercase = lower;
    unsigned compared = 0;
    text.compare = [&](const std::u16string &a, const std::u16string &b) -> std::optional<int> {
        ++compared;
        auto x = a, y = b;
        std::transform(x.begin(), x.end(), x.begin(), lower);
        std::transform(y.begin(), y.end(), y.begin(), lower);
        return x.compare(y);
    };
    auto source = records({layer(20, u"Wall"), layer(3, u"wall"), layer(9, u"Roof")});
    const auto before = source;
    auto result = lookup_native_layer_name(source, 0, u"WALL", text);
    check(result["status"] == "matched" && result["layer_id"] == 20 &&
              result["native_record_index"] == 1 && result["name_index"].size() == 2 &&
              source == before,
          "native name index overwrites in unsigned ID order rather than saved order");
    source = records({layer(9, u"Old"), layer(9, u"New")});
    check(lookup_native_layer_name(source, 0, u"Old", text)["status"] == "not_found" &&
              lookup_native_layer_name(source, 0, u"New", text)["native_record_index"] == 2,
          "same-ID layer payload loads last before names are indexed");
    source = records({layer(0x80000000u, u"same"), layer(1, u"same"), layer(UINT32_MAX, u"same")});
    result = lookup_native_layer_name(source, 0, u"same", text);
    check(result["layer_id"] == 0x80000000u && result["native_record_index"] == 1,
          "name overwrite order is unsigned and all-ones layer ID is excluded");
    source = records({layer(7, u""), layer(4, u"A")});
    check(lookup_native_layer_name(source, 0, u"", {})["status"] == "not_found" &&
              lookup_native_layer_name(Json(), 99, std::u16string({0, u'A'}), {})["status"] ==
                  "not_found",
          "native empty query returns no iterator without consulting table or locale");
    check(lookup_native_layer_name(records({}), 0, u"missing")["status"] == "not_found",
          "known empty table does not require unavailable case conversion");
    source = records({layer(4, std::u16string({u'A', 0, u'B'}))});
    result = lookup_native_layer_name(source, 0, std::u16string({u'A', 0, u'C'}), text);
    check(result["status"] == "matched" && result["query_key"]["utf16_length"] == 1 &&
              result["query_key"]["hash"] == 177670,
          "source and query use C prefixes and native lowercase DJB2 key");
    source = records({layer(8, u"Aa"), layer(2, u"B@")}); // Same hash and UTF-16 length.
    compared = 0;
    result = lookup_native_layer_name(source, 0, u"Aa", text);
    check(result["layer_id"] == 8 && result["name_index"].size() == 2 && compared > 0,
          "equal hashes and lengths still require native string comparison");
    auto unknown_compare = text;
    unknown_compare.compare = {};
    check(lookup_native_layer_name(source, 0, u"Aa", unknown_compare)["status"] == "unresolved",
          "hash collision is not resolved by inventing a source-locale comparison");
    auto unknown_lower = text;
    unknown_lower.lowercase = [](char16_t) -> std::optional<char16_t> { return {}; };
    check(lookup_native_layer_name(source, 0, u"Aa", unknown_lower)["status"] == "unresolved" &&
              lookup_native_layer_name(source, 0, u"Aa", {})["status"] == "unresolved",
          "unknown source lowercase is not replaced with ASCII or platform folding");
    source = records({layer(4, u"A")});
    source[1]["layer_definition"] = {{"name", "forged"}, {"layer_id", 300}};
    check(lookup_native_layer_name(source, 0, u"A", text)["layer_id"] == 4 &&
              lookup_native_layer_name(source, 0, u"forged", text)["status"] == "not_found",
          "layer annotations cannot replace source bodies and linkages");
    auto invalid = source;
    invalid[1]["stream"] = {"other"};
    check(lookup_native_layer_name(invalid, 0, u"A", text)["status"] == "unresolved",
          "layer members cannot cross stream boundaries");
    invalid = source;
    invalid[1]["offset"] = 70;
    check(lookup_native_layer_name(invalid, 0, u"A", text)["status"] == "unresolved",
          "noncontiguous layer records do not become a prepared table");
    invalid = source;
    auto bytes = bytesof(invalid[1]["data"]);
    set(bytes, 6, std::uint16_t(0));
    invalid[1]["data"] = rawbytes(bytes);
    check(lookup_native_layer_name(invalid, 0, u"A", text)["status"] == "unresolved",
          "table membership requires the saved child-record flag");
    invalid = source;
    bytes = bytesof(invalid[0]["data"]);
    set(bytes, 44, std::uint64_t(17));
    invalid[0]["data"] = rawbytes(bytes);
    check(lookup_native_layer_name(invalid, 0, u"A", text)["status"] == "unresolved",
          "linked layer tables do not inherit the fresh local-cache profile");
    invalid = source;
    invalid.erase(1);
    check(lookup_native_layer_name(invalid, 0, u"A", text)["status"] == "unresolved",
          "declared table member count cannot read outside the record array");
    invalid = source;
    invalid[1]["links"][0]["payload"] = rawbytes(Bytes({1, 0}));
    check(lookup_native_layer_name(invalid, 0, u"A", text)["status"] == "unresolved",
          "malformed saved names do not fall through to later candidates");
    source =
        records({layer(0, u"Rejected"), layer(UINT32_MAX, u"Rejected"), layer(64, u"Default")});
    result = lookup_native_layer_name(source, 0, u"Rejected", text);
    check(result["status"] == "not_found" && result["rejected_input_records"].size() == 2,
          "fresh source input rejects sentinel IDs before building the name index");
    for (const auto &saved : {std::u16string(), std::u16string(u"Custom"),
                              std::u16string(u"dEfAuLt"), std::u16string(u"0")}) {
        auto input = records({layer(64, saved)});
        const auto &row = input[1];
        const auto decoded = decode_native_layer(bytesof(row.at("data")), row.at("links"));
        const auto expected = saved == u"0" ? "0" : "Default";
        check(decoded["name"] == std::string(saved.begin(), saved.end()) &&
                  decoded["initial_name"]["status"] == "decoded" &&
                  decoded["initial_name"]["value"] == expected &&
                  decoded["initial_name"]["name_registered"] == true &&
                  lookup_native_layer_name(input, 0, saved == u"0" ? u"0" : u"Default",
                                           text)["layer_id"] == 64,
              "fresh default layer preserves only registered zero spelling and keeps saved names "
              "separately");
        if (!saved.empty() && saved != u"0" && saved != u"dEfAuLt")
            check(lookup_native_layer_name(input, 0, saved, text)["status"] == "not_found",
                  "a replaced default-layer name cannot remain in the initial lookup index");
    }
    for (auto id : {7u, 0x80000000u, UINT32_MAX - 1}) {
        const auto row = layer(id, u"");
        const auto decoded = decode_native_layer(bytesof(row.at("data")), row.at("links"));
        const auto expected = id <= INT32_MAX ? std::int64_t(id) : std::int64_t(id) - 0x100000000ll;
        check(
            decoded["initial_name"]["status"] == "not_evaluated" &&
                decoded["initial_name"]["value"].is_null() &&
                decoded["initial_name"]["generation_rule"]["initial_candidate"] ==
                    std::to_string(expected) &&
                decoded["initial_name"]["generation_rule"]["collision_suffix"] == "-" &&
                lookup_native_layer_name(records({row}), 0, u"anything", text)["status"] ==
                    "unresolved",
            "missing names expose signed decimal generation inputs rather than a false known miss");
    }
    using Entry = NativeLayerNameCacheEntry;
    using Request = NativeLayerNameSyncRequest;
    auto units = [](const std::u16string &s) {
        return Json(std::vector<std::uint16_t>(s.begin(), s.end()));
    };
    auto names = [&](const Json &j) {
        std::map<std::uint32_t, std::u16string> out;
        for (const auto &entry : j.at("layers")) {
            const auto raw = entry.at("name_utf16").get<std::vector<std::uint16_t>>();
            out[entry.at("layer_id").get<std::uint32_t>()] = std::u16string(raw.begin(), raw.end());
        }
        return out;
    };
    for (bool populated : {false, true}) {
        const auto sync = evaluate_native_layer_name_sync(
            {{1, u"A"}, {2, u"a"}, {3, u"B"}}, {{1, u"a"}, {2, u"B"}, {3, u"A"}}, populated, text);
        check(sync["status"] == "evaluated" && sync["operations"][0]["native_return_code"] == 0 &&
                  sync["operations"][0]["name_index_updated"] == populated &&
                  sync["operations"][1]["native_return_code"] == 9 &&
                  sync["operations"][2]["conflicting_layer_id"] == (populated ? 1 : 2) &&
                  names(sync).at(1) == u"a" && names(sync).at(2) == u"a",
              "case-only synchronization bypasses conflict and updates only a populated index");
    }
    const auto ordered = evaluate_native_layer_name_sync({{1, u"A"}, {2, u"B"}},
                                                         {{1, u"C"}, {2, u"A"}}, false, text);
    const auto reversed = evaluate_native_layer_name_sync({{1, u"A"}, {2, u"B"}},
                                                          {{2, u"A"}, {1, u"C"}}, false, text);
    check(names(ordered).at(2) == u"A" && names(reversed).at(2) == u"B" &&
              reversed["operations"][0]["native_return_code"] == 9,
          "name synchronization consumes actual operation order without sorting requests");
    check(evaluate_native_layer_name_sync({{1, u"A", false}, {2, u"a"}}, {{1, u"a"}}, false,
                                          text)["operations"][0]["native_return_code"] == 9,
          "a saved string without a registered key does not enter the case-only bypass");
    for (auto bad : std::u16string(u"<>\\/\"?*|,=&\n\t")) {
        const auto sync = evaluate_native_layer_name_sync(
            {{1, u"Old"}}, {{1, std::u16string(u"X") + bad + u"Y"}}, false);
        check(sync["status"] == "evaluated" && sync["operations"][0]["native_return_code"] == 8 &&
                  names(sync).at(1) == u"Old" && sync["name_index_populated"] == false,
              "native forbidden characters reject before locale or name-index access");
    }
    for (auto same : {std::u16string(u"<invalid>"), std::u16string(513, u'X'), std::u16string()}) {
        const auto sync =
            evaluate_native_layer_name_sync({{1, same, !same.empty()}}, {{1, same}}, false);
        check(sync["operations"][0]["selection"] == "unchanged" &&
                  sync["operations"][0]["native_return_code"] == 0,
              "exact unchanged names bypass validation including empty or oversized strings");
    }
    for (unsigned length : {512u, 513u}) {
        const auto sync = evaluate_native_layer_name_sync(
            {{1, u"Old"}}, {{1, std::u16string(length, u'X')}}, false, text);
        check(sync["operations"][0]["native_return_code"] == (length == 512 ? 0 : 8),
              "native name limit counts UTF16 units and accepts exactly 512");
    }
    std::u16string emoji;
    for (unsigned i = 0; i < 256; ++i)
        emoji += u"\U0001f600";
    check(evaluate_native_layer_name_sync({{1, u"Old"}}, {{1, emoji}}, false,
                                          text)["operations"][0]["native_return_code"] == 0,
          "surrogate pairs contribute two UTF16 units rather than one Unicode scalar");
    const auto c_prefix = evaluate_native_layer_name_sync(
        {{1, std::u16string({u'A', 0, u'X'})}}, {{1, std::u16string({u'A', 0, u'Y'})}}, false);
    check(c_prefix["operations"][0]["selection"] == "unchanged" && names(c_prefix).at(1) == u"A",
          "prepared and requested names use their source C prefixes");
    auto unknown_sync = evaluate_native_layer_name_sync({{1, u"A"}, {2, u"B"}},
                                                        {{1, u"A"}, {2, u"C"}, {1, u"D"}}, false);
    check(unknown_sync["status"] == "partially_evaluated" && unknown_sync["layers"].is_null() &&
              unknown_sync["failed_request_index"] == 1 && unknown_sync["operations"].size() == 1,
          "unknown locale stops a batch and withholds a final cache rather than publishing its "
          "prefix");
    auto collision_context = text;
    collision_context.compare = {};
    check(evaluate_native_layer_name_sync({{1, u"Aa"}, {2, u"B@"}}, {{1, u"Other"}}, false,
                                          collision_context)["status"] == "not_evaluated",
          "hash-and-length collisions require actual source comparison while building the index");
    check(evaluate_native_layer_name_sync({{1, u"A"}, {1, u"B"}}, {}, false)["status"] ==
                  "not_evaluated" &&
              evaluate_native_layer_name_sync({{0, u"A"}}, {}, false)["status"] ==
                  "not_evaluated" &&
              evaluate_native_layer_name_sync({{1, u"A"}}, {{2, u"B"}}, false, text)["layers"]
                  .is_null(),
          "invalid prepared identities and absent actual targets remain unresolved");
    // Independent oracle uses canonical resource names and an integer-ID map,
    // not the implementation's hash/length buckets. All names here are ASCII.
    const std::array<std::u16string, 8> pool{{u"A", u"a", u"B", u"C", u"Aa", u"B@", u"", u"?"}};
    for (bool populated : {false, true})
        for (const auto &x : pool)
            for (const auto &y : pool)
                for (const auto &z : pool) {
                    const std::vector<Entry> input{{1, u"A"}, {2, u"a"}, {3, u"B@", false}};
                    const std::vector<Request> requests{{1, x}, {2, y}, {3, z}};
                    auto expected_layers = std::map<unsigned, std::pair<std::u16string, bool>>{
                        {1, {u"A", true}}, {2, {u"a", true}}, {3, {u"B@", false}}};
                    std::map<std::optional<std::u16string>, unsigned> registry;
                    auto canonical = [&](std::u16string value) {
                        std::transform(value.begin(), value.end(), value.begin(), lower);
                        return value;
                    };
                    auto rebuild = [&] {
                        for (const auto &[id, entry] : expected_layers)
                            registry[entry.second
                                         ? std::optional<std::u16string>(canonical(entry.first))
                                         : std::nullopt] = id;
                    };
                    if (populated)
                        rebuild();
                    std::vector<unsigned> codes;
                    for (const auto &r : requests) {
                        auto &entry = expected_layers.at(r.layer_id);
                        unsigned code = 0;
                        if (entry.first != r.name) {
                            if (r.name.empty() || r.name == u"?")
                                code = 8;
                            else {
                                const std::optional<std::u16string> new_name = canonical(r.name);
                                const auto old_name =
                                    entry.second
                                        ? std::optional<std::u16string>(canonical(entry.first))
                                        : std::nullopt;
                                if (new_name != old_name) {
                                    if (registry.empty())
                                        rebuild();
                                    const auto match = registry.find(new_name);
                                    if (match != registry.end() && match->second != r.layer_id)
                                        code = 9;
                                }
                                if (!code) {
                                    if (!registry.empty()) {
                                        registry.erase(old_name);
                                        registry[new_name] = r.layer_id;
                                    }
                                    entry = {r.name, true};
                                }
                            }
                        }
                        codes.push_back(code);
                    }
                    const auto actual =
                        evaluate_native_layer_name_sync(input, requests, populated, text);
                    bool matches = actual["status"] == "evaluated" &&
                                   actual["name_index_populated"] == !registry.empty();
                    for (unsigned i = 0; i < 3; ++i)
                        matches =
                            matches && actual["operations"][i]["native_return_code"] == codes[i];
                    for (const auto &entry : actual["layers"]) {
                        auto id = entry["layer_id"].get<unsigned>();
                        matches = matches &&
                                  entry["name_utf16"] == units(expected_layers.at(id).first) &&
                                  entry["name_registered"] == expected_layers.at(id).second;
                    }
                    check(matches, "ordered sync matches independent canonical-resource oracle "
                                   "including lazy indexes and duplicate names");
                }
    return checks;
}
