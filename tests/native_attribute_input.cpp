#include "internal.hpp"

unsigned native_attribute_input_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto put = [](Bytes &b, std::uint64_t value, unsigned n) {
        for (unsigned i = 0; i < n; ++i)
            b.push_back(static_cast<std::uint8_t>(value >> (8 * i)));
    };
    auto record = [&](std::uint64_t id, unsigned count = 1, unsigned key = 42) {
        Bytes b;
        put(b, 0xa11b, 4);
        put(b, count ? 21 : 4, 4);
        put(b, ~std::uint64_t(0), 8);
        put(b, id, 8);
        put(b, count, 4);
        if (count) {
            put(b, 9, 2);
            put(b, key, 2);
            put(b, 7, 4);
            put(b, 0x80000001u, 4);
            put(b, 123, 4);
            put(b, 99, 1);
        }
        put(b, 3, 4);
        return b;
    };
    auto block = [&](std::string name, unsigned count, const std::vector<Bytes> &parts) {
        Bytes raw;
        put(raw, count, 4);
        put(raw, 1, 4);
        raw.resize(16);
        std::uint8_t previous = 0x26;
        for (auto &v : raw) {
            v ^= previous;
            previous = v;
        }
        raw.resize(24);
        Bytes payload;
        for (const auto &part : parts)
            payload.insert(payload.end(), part.begin(), part.end());
        return Stream{{"root", "attrs", name},
                      std::make_shared<Bytes>(raw),
                      std::make_shared<Bytes>(payload),
                      24};
    };
    auto master = [&](unsigned count) {
        Bytes b;
        put(b, 5, 4);
        put(b, count, 4);
        return Stream{
            {"root", "attrs", "master"}, std::make_shared<Bytes>(b), std::make_shared<Bytes>(b)};
    };
    Json index = {{"P3D-SSYSA", "attrs"}, {"P3D~MPH", "master"}, {"$1", "b1"}, {"$2", "b2"}};
    Json ids = {{"status", "resolved"}, {"registry", Json::array()}};
    for (unsigned i = 1; i <= 3; ++i)
        ids["registry"].push_back({{"id", i},
                                   {"native_record_index", 100 + i},
                                   {"input_occurrence_index", 200 + i},
                                   {"block_number", 1}});
    std::vector<Stream> streams = {master(2), block("b1", 1, {record(1)}),
                                   block("b2", 1, {record(2)})};
    auto evaluate = [&] {
        return native_system_attribute_input(streams, index, {"root", "sys"}, ids);
    };
    auto result = evaluate();
    check(result["status"] == "resolved" && result["attachments"].size() == 2,
          "attribute master count overrides last-block control bit");
    check(result["blocks"][1]["records"][0]["native_block_mismatch"] == true &&
              result["attachments"][1]["target"]["native_record_index"] == 102,
          "assigned registry identity wins over source block equality");
    const auto &attr = result["attachments"][0]["attributes"][0];
    check(attr["source_ordinal"] == 0 && attr["group"] == 9 && attr["key"] == 42 &&
              attr["size_flags"] == 0x80000001u && attr["reserved"] == 123 &&
              bytesof(attr["payload"]) == Bytes{99},
          "attached attribute source fields survive");
    streams = {master(1), block("b1", 3, {record(1), record(1), record(2)})};
    result = evaluate();
    check(result["attachments"].size() == 1 &&
              result["blocks"][0]["records"][1]["action"] == "reject_existing_collection" &&
              result["blocks"][0]["reason"] == "invalid_record_magic",
          "duplicate collection consumes header only and next read observes body bytes");
    streams = {master(1), block("b1", 2, {record(1), record(1)})};
    result = evaluate();
    check(result["blocks"][0]["status"] == "completed" &&
              result["blocks"][0]["unconsumed_bytes"] == 25,
          "duplicate final record leaves unread body after declared count");
    streams = {master(1), block("b1", 3, {record(1, 0), record(1), record(2)})};
    result = evaluate();
    check(result["attachments"].size() == 2 &&
              result["blocks"][0]["records"][0]["action"] == "empty_without_collection",
          "zero attribute count does not occupy object collection");
    streams = {master(2), block("b1", 1, {record(1)}), block("b2", 1, {record(1)})};
    result = evaluate();
    check(result["attachments"].size() == 1 &&
              result["blocks"][1]["records"][0]["action"] == "reject_existing_collection",
          "collection presence persists between logical attribute blocks");
    streams = {master(1), block("b1", 1, {record(1), record(2)})};
    result = evaluate();
    check(result["attachments"].size() == 1 && result["blocks"][0]["unconsumed_bytes"] == 49,
          "attribute reader honors exact record count rather than all decoded records");
    streams = {master(1), block("b1", 2, {record(99), record(2)})};
    result = evaluate();
    check(result["attachments"].empty() && result["blocks"][0]["records"][0]["end_offset"] == 45 &&
              result["blocks"][0]["reason"] == "invalid_record_magic",
          "unindexed record skip uses declared length relative to 24-byte header");
    streams = {master(1), block("b1", 2, {record(99)})};
    result = evaluate();
    check(
        result["status"] == "partial" &&
            result["blocks"][0]["reason"] == "post_skip_read_exceeds_decoded_buffer",
        "native skip leaves a stale remaining-byte counter and cannot justify out-of-buffer reads");
    streams = {master(2), block("b1", 1, {Bytes(7)}), block("b2", 1, {record(2)})};
    result = evaluate();
    check(result["status"] == "resolved" && result["blocks"][0]["end_offset"] == 7 &&
              result["attachments"].size() == 1,
          "short header consumes available bytes and does not prevent later block input");
    streams = {master(2), block("b2", 1, {record(2)})};
    result = evaluate();
    check(result["status"] == "resolved" && result["attachments"].size() == 1 &&
              result["blocks"][0]["status"] == "not_loaded",
          "missing physical block does not end master-count loop");
    streams = {master(1), block("b1", 0, {record(1)})};
    check(evaluate()["attachments"].empty(), "zero block count skips body");
    auto truncated = record(1);
    truncated.resize(40);
    streams = {master(2), block("b1", 1, {truncated}), block("b2", 1, {record(2)})};
    result = evaluate();
    check(result["status"] == "partial" && result["attachments"].empty() &&
              result["blocks"].size() == 1,
          "short attribute read does not invent native uninitialized state or later attachments");
    streams = {master(1), block("b1", 1, {record(1, 65536)})};
    result = evaluate();
    check(result["attachments"].size() == 1 && result["attachments"][0]["attributes"].empty() &&
              result["blocks"][0]["records"][0]["end_offset"] == 32,
          "nonzero 32-bit count truncated to zero still creates empty collection");
    streams = {master(1), block("b1", 1, {record(1, 65537)})};
    check(evaluate()["attachments"][0]["attributes"].size() == 1,
          "attribute loop uses low sixteen count bits");
    auto saved = index;
    index.erase("$1");
    check(evaluate()["status"] == "partial", "missing alias generation is unresolved");
    index = saved;
    ids["status"] = "partial";
    check(evaluate()["reason"] == "complete_system_id_assignments_required",
          "partial ID registry cannot establish complete attachment identity");
    ids["status"] = "resolved";
    streams.clear();
    check(evaluate()["status"] == "absent", "absent attribute storage distinguished");
    return checks;
}
