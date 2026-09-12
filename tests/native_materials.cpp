#include "internal.hpp"
#include <limits>

unsigned native_material_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto append = [](Bytes &b, std::uint64_t value, unsigned count) {
        for (unsigned i = 0; i < count; ++i)
            b.push_back(static_cast<std::uint8_t>(value >> (i * 8)));
    };
    auto put = [](Bytes &b, std::size_t offset, std::uint64_t value, unsigned count) {
        for (unsigned i = 0; i < count; ++i)
            b.at(offset + i) = static_cast<std::uint8_t>(value >> (i * 8));
    };
    auto payload = [&](std::uint64_t id) {
        Bytes b;
        append(b, 0x1000e, 4);
        append(b, id, 8);
        return b;
    };
    auto linkage = [&](const Bytes &p, unsigned app = 0x41, unsigned header = 0) {
        Bytes b;
        append(b, header ? header : (0x1000 | ((p.size() + 4) / 2 - 1)), 2);
        append(b, app, 2);
        b.insert(b.end(), p.begin(), p.end());
        return b;
    };
    auto record = [&](const std::vector<Bytes> &links, std::uint64_t id = 19) {
        Bytes b(36);
        put(b, 4, 1, 2);
        put(b, 12, 16, 4);
        put(b, 20, id, 8);
        for (const auto &l : links)
            b.insert(b.end(), l.begin(), l.end());
        put(b, 8, (b.size() - 4) / 2, 4);
        return b;
    };
    const auto maximum = std::numeric_limits<std::uint64_t>::max();
    for (auto id : {std::uint64_t(0), std::uint64_t(1), std::uint64_t(0xfedcba9876543210),
                    maximum - 1, maximum}) {
        const auto p = payload(id);
        auto rows = parse_native(record({linkage(p)}));
        const auto &l = rows[0]["links"][0];
        const auto &d = l["decoded"];
        check(d["status"] == "decoded" && d["material_id"].is_number_unsigned() &&
                  d["material_id"].get<std::uint64_t>() == id && d["material_id_offset"] == 8 &&
                  d["key"] == 0x1000e && d["reader_selection"] == "first_match" &&
                  d["unassigned_suffix_hex"] == "" && bytesof(l["payload"]) == p,
              "native material IDs retain every bit including sentinel values and raw bytes");
        auto refs = native_material_references(rows);
        check(refs.size() == 1 && refs[0]["reference"] == d &&
                  refs[0]["native_record_index"] == 0 && refs[0]["record_id"] == 19 &&
                  refs[0]["record_offset"] == 0 && refs[0]["linkage_index"] == 0 &&
                  refs[0]["linkage_offset"] == 36 && refs[0]["stream"].is_null(),
              "native material aggregate identifies the exact source linkage");
    }
    auto duplicate =
        parse_native(record({linkage(payload(7)), linkage(payload(8)), linkage(payload(7))}));
    auto refs = native_material_references(duplicate);
    check(refs.size() == 3 && refs[0]["reference"]["reader_selection"] == "first_match" &&
              refs[1]["reference"]["reader_selection"] == "shadowed" &&
              refs[2]["reference"]["reader_selection"] == "shadowed" &&
              refs[1]["reference"]["material_id"] == 8 &&
              refs[2]["reference"]["material_id"] == 7 && refs[2]["linkage_index"] == 2 &&
              refs[2]["linkage_offset"] == 68,
          "native getter first-match order does not discard duplicate or shadowed references");
    for (std::size_t size : {4, 6, 8, 10}) {
        auto short_payload = payload(maximum);
        short_payload.resize(size);
        auto rows = parse_native(record({linkage(short_payload), linkage(payload(42))}));
        refs = native_material_references(rows);
        check(rows.size() == 1 && refs.size() == 2 && refs[0]["reference"]["status"] == "invalid" &&
                  refs[0]["reference"]["material_id"].is_null() &&
                  refs[0]["reference"].contains("decode_error") &&
                  refs[0]["reference"]["reader_selection"] == "first_match" &&
                  refs[1]["reference"]["status"] == "decoded" &&
                  refs[1]["reference"]["reader_selection"] == "shadowed" &&
                  bytesof(rows[0]["links"][0]["payload"]) == short_payload,
              "a truncated first reference is local damage and does not select a later ID");
    }
    for (std::size_t extra : {2, 4}) {
        auto p = payload(73);
        p.resize(p.size() + extra, 0xab);
        auto rows = parse_native(record({linkage(p)}));
        const auto &d = rows[0]["links"][0]["decoded"];
        check(d["status"] == "partial" && d["material_id"] == 73 &&
                  d["unassigned_suffix_hex"] == hex(Bytes(extra, 0xab)) &&
                  bytesof(rows[0]["links"][0]["payload"]) == p,
              "unknown material linkage suffixes remain visible instead of counted as decoded");
    }
    auto extended_payload = payload(99);
    extended_payload.resize(1020, 0x5c);
    auto extended = parse_native(record({linkage(extended_payload, 0x41, 0x5200 | 128)}));
    check(extended[0]["links"][0]["decoded"]["material_id"] == 99 &&
              extended[0]["links"][0]["decoded"]["status"] == "partial" &&
              extended[0]["links"][0]["decoded"]["unassigned_suffix_hex"] == hex(Bytes(1008, 0x5c)),
          "extended linkage word counts bound the material prefix and preserve the full suffix");
    auto wrong_key = payload(5);
    put(wrong_key, 0, 0x1000f, 4);
    auto key_only = payload(5);
    key_only.resize(4);
    auto rows = parse_native(
        record({linkage(payload(5), 0x42), linkage(wrong_key), linkage(key_only, 0x41, 7),
                linkage(Bytes{0x0e, 0x00}), linkage(Bytes{}), linkage(payload(6))}));
    for (std::size_t i = 0; i < 5; ++i)
        check(!rows[0]["links"][i].contains("decoded"),
              "non-user, other-app, other-key and incomplete-key linkages are not material IDs");
    refs = native_material_references(rows);
    check(refs.size() == 1 && refs[0]["linkage_index"] == 5 &&
              refs[0]["reference"]["reader_selection"] == "first_match",
          "unrelated linkages do not consume first material selection");
    auto first = record({linkage(payload(100))}, 21);
    auto second = record({linkage(payload(101))}, 21);
    const auto second_offset = first.size();
    first.insert(first.end(), second.begin(), second.end());
    rows = parse_native(first);
    rows[0]["stream"] = Json::array({"root", "model-a", "native"});
    rows[1]["stream"] = rows[0]["stream"];
    auto other_stream = rows[0];
    other_stream["stream"] = Json::array({"root", "model-b", "native"});
    rows.push_back(other_stream);
    refs = native_material_references(rows);
    check(refs.size() == 3 && refs[1]["native_record_index"] == 1 &&
              refs[1]["record_offset"] == second_offset &&
              refs[1]["linkage_offset"] == second_offset + 36 &&
              refs[1]["reference"]["reader_selection"] == "first_match" &&
              refs[2]["native_record_index"] == 2 && refs[2]["stream"] != refs[0]["stream"] &&
              refs[2]["record_id"] == refs[0]["record_id"],
          "duplicate record IDs and streams retain distinct source identity without fabricated "
          "reuse");
    check(native_material_references(parse_native(record({}))).empty() &&
              native_material_references(Json::array()).empty(),
          "absent native material linkage creates no default reference");
    return checks;
}
