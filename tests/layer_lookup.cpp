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
    source = records({layer(0, u"Rejected"), layer(UINT32_MAX, u"Rejected"), layer(64, u"Default")});
    result = lookup_native_layer_name(source, 0, u"Rejected", text);
    check(result["status"] == "not_found" && result["rejected_input_records"].size() == 2,
          "fresh source input rejects sentinel IDs before building the name index");
    return checks;
}
