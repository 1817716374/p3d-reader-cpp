#include "internal.hpp"
#include <p3d/layer_lookup.hpp>
#include <thread>
using namespace p3d;
unsigned layer_numeric_lookup_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        require(ok, message);
        ++checks;
    };
    auto set = [](Bytes &b, std::size_t at, auto value) {
        std::memcpy(b.data() + at, &value, sizeof(value));
    };
    auto layer = [&](std::uint32_t id, std::uint32_t key, unsigned version = 7) {
        Bytes b(108, 0);
        set(b, 4, std::uint16_t(49));
        set(b, 6, std::uint16_t(0x80));
        set(b, 8, std::uint32_t(52));
        set(b, 12, std::uint32_t(52));
        set(b, 16, std::uint32_t(1));
        set(b, 36, id);
        set(b, 44, std::uint16_t(version));
        set(b, 96, key);
        return b;
    };
    auto source = [&](const std::vector<Bytes> &layers) {
        Bytes bytes(68, 0);
        set(bytes, 4, std::uint16_t(10));
        set(bytes, 8, std::uint32_t(32));
        set(bytes, 12, std::uint32_t(32));
        set(bytes, 16, std::uint32_t(1));
        set(bytes, 36, std::uint32_t(layers.size()));
        for (const auto &b : layers)
            bytes.insert(bytes.end(), b.begin(), b.end());
        auto records = parse_native(bytes);
        for (auto &record : records)
            record["stream"] = {"system", "layers"};
        return records;
    };
    for (unsigned version = 1; version <= 7; ++version) {
        const auto d = decode_native_layer(layer(64, 0xfedcba98u, version), Json::array());
        const auto &key = d.at("numeric_key");
        check(key.at("value") == (version == 1 ? 0u : 0xfedcba98u) &&
                  key.at("saved_value") == 0xfedcba98u &&
                  key.at("source_offset") == (version == 1 ? 36 : 96),
              "numeric lookup key follows versioned reader while retaining the ignored saved word");
    }
    check(decode_native_layer(layer(123, 999, 1), Json::array())["numeric_key"]["value"] == 123,
          "legacy nondefault layer takes its unsigned ID instead of the later saved field");
    auto bytes = layer(8, 17);
    set(bytes, 100, std::uint64_t(0xfedcba9876543210ull));
    const auto decoded = decode_native_layer(bytes, Json::array());
    check(decoded["unassigned_ranges"][2]["source_offset"] == 100 &&
              bytesof(decoded["unassigned_ranges"][2]["data"]) == slice(bytes, 100, 8),
          "explained numeric key is removed from unassigned ranges without consuming its neighbor");
    auto records = source({layer(20, 5), layer(3, 5), layer(9, 7)});
    const auto before = records;
    NativeLayerNumericIndex index(records, 0);
    auto match = index.lookup(5);
    check(index.metadata()["status"] == "resolved" && match["status"] == "matched" &&
              match["layer_id"] == 20 && match["native_record_index"] == 1 &&
              match["candidate_record_indices"] == Json::array({2, 1}) && records == before,
          "equal numeric keys overwrite in unsigned layer-ID order and keep source candidates");
    check(
        index.remap_offset(3, 0)["layer_id"] == 20 && index.remap_offset(3, 2)["layer_id"] == 9,
        "offset remap uses the numeric index even for zero offset rather than returning source ID");
    check(index.remap_offset(3, 1)["selection"] == "target_key_missing_fallback" &&
              index.remap_offset(3, 1)["layer_id"] == 64 &&
              !index.remap_offset(3, 1).contains("native_record_index"),
          "confirmed absent target returns the native fallback without fabricating a definition");
    check(index.remap_offset(99, 7)["selection"] == "source_layer_missing_fallback" &&
              index.remap_offset(99, 7)["layer_id"] == 64,
          "confirmed absent source also returns fallback without searching a neighboring ID");
    records = source({layer(9, 5), layer(9, 7)});
    NativeLayerNumericIndex changed(records, 0);
    check(changed.lookup(5)["status"] == "not_found" &&
              changed.lookup(7)["native_record_index"] == 2,
          "same-ID replacement precedes numeric indexing and removes the old key");
    records = source({layer(0x80000000u, 5), layer(1, 5), layer(UINT32_MAX - 1, 7)});
    NativeLayerNumericIndex high(records, 0);
    check(high.lookup(5)["layer_id"] == 0x80000000u && high.lookup(7)["layer_id"] == UINT32_MAX - 1,
          "unsigned IDs control overwrite order and max-minus-one remains a loaded source row");
    check(high.remap_offset(1, 2)["status"] == "unresolved" &&
              !high.remap_offset(1, 2).contains("layer_id") &&
              high.remap_offset(1, 2, 27)["layer_id"] == 27 &&
              high.remap_offset(1, 2, UINT32_MAX - 1)["layer_id"] == UINT32_MAX - 1,
          "mapped inheritance sentinel reads the actual parent value once without recursive "
          "guessing");
    records = source({layer(0, 9), layer(UINT32_MAX, 9), layer(64, 0)});
    NativeLayerNumericIndex rejected(records, 0);
    check(
        rejected.lookup(9)["status"] == "not_found" && rejected.lookup(0)["layer_id"] == 64 &&
            rejected.metadata()["rejected_input_records"].size() == 2 && records.size() == 4,
        "fresh input rejects zero and all-ones IDs while zero numeric keys and raw records remain");
    records = source({layer(10, UINT32_MAX - 1), layer(20, 1), layer(30, 0)});
    NativeLayerNumericIndex wrapping(records, 0);
    check(wrapping.remap_offset(10, 3)["target_numeric_key"] == 1 &&
              wrapping.remap_offset(10, 3)["layer_id"] == 20 &&
              wrapping.remap_offset(20, UINT32_MAX)["layer_id"] == 30,
          "native offset addition wraps modulo 32 bits including negative offset bit patterns");
    NativeLayerNumericIndex unknown(Json(), 0);
    check(unknown.lookup(0)["status"] == "unresolved" &&
              unknown.remap_offset(10, 0)["status"] == "unresolved" &&
              unknown.remap_offset(0, 7)["selection"] == "zero_layer_bypass" &&
              unknown.remap_offset(0, 7)["layer_id"] == 0,
          "unknown table is not a confirmed miss and native zero bypass does not consult it");
    records[1]["layer_definition"] = {{"numeric_key", {{"value", 55}}}, {"layer_id", 999}};
    check(NativeLayerNumericIndex(records, 0).lookup(UINT32_MAX - 1)["layer_id"] == 10,
          "decoded annotations cannot replace authoritative source bytes");
    records = source({layer(20, 5), layer(3, 5)});
    auto invalid = records;
    invalid[2]["stream"] = {"different"};
    auto bad = NativeLayerNumericIndex(invalid, 0);
    check(bad.metadata()["entries"].empty() && bad.lookup(5)["status"] == "unresolved",
          "bad later boundary cannot publish the earlier usable numeric-key prefix");
    invalid = records;
    invalid[2]["offset"] = 177;
    check(NativeLayerNumericIndex(invalid, 0).lookup(5)["status"] == "unresolved",
          "noncontiguous table members cannot form a numeric index");
    invalid = source({layer(20, 5), layer(3, 7, 8)});
    check(NativeLayerNumericIndex(invalid, 0).metadata()["entries"].empty(),
          "unknown later version prevents partial index publication");
    invalid = records;
    invalid.erase(2);
    check(NativeLayerNumericIndex(invalid, 0).lookup(5)["status"] == "unresolved",
          "declared child count cannot read outside the source array");
    invalid = records;
    bytes = bytesof(invalid[0]["data"]);
    set(bytes, 44, std::uint64_t(55));
    invalid[0]["data"] = rawbytes(bytes);
    check(NativeLayerNumericIndex(invalid, 0).lookup(5)["status"] == "unresolved",
          "linked tables do not silently acquire the fresh local-table profile");
    invalid = records;
    bytes = bytesof(invalid[2]["data"]);
    set(bytes, 6, std::uint16_t(0));
    invalid[2]["data"] = rawbytes(bytes);
    check(NativeLayerNumericIndex(invalid, 0).lookup(5)["status"] == "unresolved",
          "numeric indexing preserves raw child-flag membership checks");
    check(NativeLayerNumericIndex(source({}), 0).lookup(0)["status"] == "not_found",
          "confirmed empty source table yields a known numeric-key miss");
    const auto shared = NativeLayerNumericIndex(records, 0);
    records.clear();
    Json first, second;
    std::thread a([&] { first = shared.lookup(5); });
    std::thread b([&] { second = shared.remap_offset(3, 0); });
    a.join();
    b.join();
    check(first["layer_id"] == 20 && second["layer_id"] == 20,
          "const queries share one owned index after source lifetime ends and across threads");
    records = source({layer(20, 5), layer(3, 5)});
    const auto tables = build_layer_tables(Json::object(), records, Json::array());
    check(
        tables["tables"][0]["initial_numeric_index"] ==
                NativeLayerNumericIndex(records, 0).metadata() &&
            tables["tables"][0]["member_record_indices"] == Json::array({1, 2}),
        "document layer tables expose the initial numeric index separately from saved membership");
    return checks;
}
