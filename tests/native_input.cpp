#include "internal.hpp"

unsigned native_input_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto put = [](Bytes &b, std::size_t at, std::uint32_t value, unsigned length) {
        for (unsigned i = 0; i < length; ++i)
            b.at(at + i) = static_cast<std::uint8_t>(value >> (8 * i));
    };
    auto record = [&](unsigned type, unsigned flags, std::uint32_t count = 0) {
        const bool compound = type >= 10 && type <= 32;
        const auto offset = flags & 0x20 ? 108u : 36u;
        Bytes b(compound ? offset + 8 : 36, 0);
        put(b, 4, type, 2);
        put(b, 6, flags, 2);
        put(b, 8, (b.size() - 4) / 2, 4);
        put(b, 12, (b.size() - 4) / 2, 4);
        if (compound)
            put(b, offset, count, 4);
        return b;
    };
    auto parse = [&](const std::vector<Bytes> &parts) {
        Bytes joined;
        for (const auto &part : parts)
            joined.insert(joined.end(), part.begin(), part.end());
        auto rows = parse_native(joined);
        for (auto &n : rows)
            n["stream"] = Json::array({"system", "$1"});
        return rows;
    };
    for (unsigned flags : {0u, 0x20u, 0x40u, 0x60u}) {
        auto rows = parse({record(32, flags, 1), record(37, 0)});
        const auto copy = rows;
        auto tree = native_record_input_subtree(0, rows, 1);
        check(tree["status"] == "resolved" && tree["root_topology"]["compound"] == true &&
                  tree["member_record_indices"] == Json::array({1}) &&
                  tree["nodes"] == Json::array({{{"native_record_index", 1},
                                                 {"parent_record_index", 0},
                                                 {"compound", false}}}) &&
                  rows == copy,
              "block roots select ordinary or extended count positions and infer parentage without "
              "source child flags");
    }
    auto rows = parse({record(32, 0x20, 4), record(10, 0x80, 2), record(37, 0x80), record(37, 0x80),
                       record(37, 0x80)});
    auto tree = native_record_input_subtree(0, rows, 1);
    check(tree["status"] == "resolved" && tree["member_record_indices"] == Json::array({1, 4}) &&
              tree["nodes"].size() == 4 && tree["nodes"][0]["parent_record_index"] == 0 &&
              tree["nodes"][1]["parent_record_index"] == 1 &&
              tree["nodes"][2]["parent_record_index"] == 1 &&
              tree["nodes"][3]["parent_record_index"] == 0,
          "block input returns accepted descendants in preorder with true parents and distinct "
          "direct children");
    rows = parse({record(32, 0x20, 3), record(10, 0x80, 1), record(37, 0x88), record(37, 0x80),
                  record(37, 0x80)});
    tree = native_record_input_subtree(0, rows, 1);
    check(tree["status"] == "resolved" && tree["member_record_indices"] == Json::array({1}) &&
              tree["nodes"].size() == 2 && tree["nodes"][1]["native_record_index"] == 3 &&
              tree["nodes"][1]["parent_record_index"] == 1 &&
              tree["skipped_record_indices"] == Json::array({2}),
          "a nested skipped child can cross a stored span without losing its native parent or "
          "adopting the next sibling");
    auto prefixed = record(37, 0);
    put(prefixed, 0, 7, 4);
    rows = parse({record(32, 0, 1), prefixed, record(37, 0)});
    tree = native_record_input_subtree(0, rows, 1);
    check(tree["status"] == "resolved" && tree["end_counter"] == 2 &&
              tree["member_record_indices"] == Json::array({2}) &&
              tree["nodes"][0]["parent_record_index"] == 0,
          "uncounted prefix skips do not consume block child slots");
    for (unsigned damage = 0; damage < 3; ++damage) {
        rows = parse({record(32, 0, 2), record(37, 0), record(37, 0)});
        if (!damage)
            rows.erase(rows.begin() + 2);
        if (damage == 1)
            rows[2]["stream"] = Json::array({"system", "$2"});
        if (damage == 2)
            rows[2]["offset"] = rows[2]["offset"].get<std::uint64_t>() + 4;
        tree = native_record_input_subtree(0, rows, 1);
        check(tree["status"] == "invalid" && tree["nodes"].empty() &&
                  tree["member_record_indices"].empty(),
              "an incomplete block input publishes no partially owned geometry nodes");
    }
    rows = parse({record(32, 8, 1), record(37, 0)});
    tree = native_record_input_subtree(0, rows, 1);
    check(tree["status"] == "skipped" && tree["nodes"].empty() &&
              tree["consumed_record_indices"].empty(),
          "a skipped block root cannot expose a successful member tree");
    rows = parse({record(32, 0, 2), record(24, 0xa0, 1), record(37, 0)});
    tree = native_record_input_subtree(0, rows, 1);
    check(tree["status"] == "resolved" && tree["nodes"][1]["parent_record_index"] == 1 &&
              tree["record_conversions"][0]["kind"] == "type_24_extended_header_removal",
          "ordinary blocks share the conversion-aware subtree reader with material tables");
    rows = parse({record(37, 0), record(32, 0, 0)});
    tree = native_record_input_subtree(0, rows, 1);
    check(tree["status"] == "resolved" && tree["root_topology"]["compound"] == false &&
              tree["nodes"].empty() && tree["end_counter"] == 1,
          "a leaf subtree does not adopt an adjacent root");
    std::vector<Bytes> deep{record(32, 0, 257)};
    for (unsigned i = 0; i < 256; ++i)
        deep.push_back(record(10, 0, 256 - i));
    deep.push_back(record(37, 0));
    rows = parse(deep);
    tree = native_record_input_subtree(0, rows, 1);
    check(tree["status"] == "resolved" && tree["nodes"].size() == 257 &&
              tree["member_record_indices"] == Json::array({1}) &&
              tree["nodes"].back()["parent_record_index"] == 256,
          "deep compound input preserves each parent without recursive C++ call-stack growth");
    // The block header is a chained XOR, including bytes the reader ignores.
    auto header = [&](std::uint32_t capacity, std::uint32_t flags) {
        Bytes b(16, 0);
        put(b, 0, capacity, 4);
        put(b, 4, flags, 4);
        for (std::size_t i = 0; i < b.size(); ++i)
            b[i] ^= i ? b[i - 1] : 0x26;
        return b;
    };
    check(native_block_header(Bytes(15))["native_error_code"] == 0x14007,
          "short block headers return their own read error");
    const Bytes encoded_empty = {0x26, 0x26, 0x26, 0x26, 0x25, 0x25, 0x25, 0x25,
                                 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25};
    check(native_block_header(encoded_empty)["record_input"] == "skip" &&
              native_block_header(encoded_empty)["stop_after_block"] == true,
          "fixed encoded empty header decodes without depending on a test encoder");
    auto h = native_block_header(header(0x80000001u, 0xa5));
    check(h["record_capacity_hint"] == 0x80000001u && h["capacity_uses_list_default"] == true &&
              h["stop_after_block"] == true &&
              h["record_input"] == "read_until_record_reader_stops" &&
              h["unassigned_control_bits"] == 0xa4 && h["block_return_code"] == 0x12003,
          "signed capacity fallback preserves raw quantity and all flag bits");
    Json idx = {{"P3D-SSYS", "sys"}, {"$1", "a"}, {"$2", "b"}, {"$3", "c"}};
    auto stream = [&](std::string name, unsigned capacity, unsigned flags, const Bytes &payload) {
        auto raw = header(capacity, flags);
        raw.push_back(0); // Synthetic wrapper; the decoded buffer is supplied separately.
        return Stream{{"file", "sys", name},
                      std::make_shared<Bytes>(raw),
                      std::make_shared<Bytes>(payload),
                      24};
    };
    auto payload = record(37, 0);
    auto streams = std::vector<Stream>{stream("c", 1, 3, payload), stream("a", 1, 2, payload),
                                       stream("b", 0, 2, payload)};
    rows = Json::array();
    for (const auto &s : streams) {
        auto parsed = parse_native(*s.decoded);
        for (auto &n : parsed) {
            n["stream"] = s.path;
            rows.push_back(n);
        }
    }
    auto evaluate = [&]() { return native_input_containers(streams, idx, rows).at(0); };
    auto plan = evaluate();
    check(
        plan["input_order"] ==
                Json::array({{{"block_number", 1}, {"logical_name", "$1"}, {"block_index", 1}},
                             {{"block_number", 2}, {"logical_name", "$2"}, {"block_index", 2}},
                             {{"block_number", 3}, {"logical_name", "$3"}, {"block_index", 0}}}) &&
            plan["stop"]["reason"] == "last_block_flag",
        "block input follows numeric aliases independently of physical order");
    check(plan["blocks"][2]["record_input"]["status"] == "skipped" &&
              plan["blocks"][0]["record_input"]["roots"][0]["native_record_index"] == 0 &&
              plan["blocks"][1]["record_input"]["roots"][0]["input_tree"]["start_counter"] == 1,
          "zero capacity skips existing payload and each block restarts its record counter");
    idx.erase("$2");
    plan = evaluate();
    check(plan["input_order"].size() == 1 && plan["stop"]["reason"] == "missing_block_alias" &&
              plan["blocks"][0]["reachability"] == "not_reached",
          "missing aliases stop input without skipping a number");
    idx["$2"] = "missing";
    check(evaluate()["stop"]["reason"] == "missing_block_stream",
          "missing physical block stops input");
    idx["$2"] = "b";
    streams[2].raw = std::make_shared<Bytes>(15);
    plan = evaluate();
    check(plan["input_order"].size() == 2 && plan["stop"]["native_block_return_code"] == 0x14007 &&
              plan["blocks"][0]["reachability"] == "not_reached",
          "truncated header stops the container before later blocks");
    streams[2] = stream("b", 1, 3, payload);
    auto bad = record(32, 0, 5);
    streams[1] = stream("a", 1, 2, bad);
    rows[1] = parse_native(bad)[0];
    rows[1]["stream"] = streams[1].path;
    plan = evaluate();
    check(plan["blocks"][1]["record_input"]["status"] == "read_error" &&
              plan["blocks"][1]["record_input"]["roots"].empty() &&
              plan["blocks"][2]["reachability"] == "reached" && plan["input_order"].size() == 2,
          "failed subtree ends this block but does not override the header continuation flag");
    auto first = record(37, 0), nested = record(32, 0, 1), child = record(37, 0);
    first.insert(first.end(), nested.begin(), nested.end());
    first.insert(first.end(), child.begin(), child.end());
    streams = {stream("a", 1, 3, first)};
    rows = parse_native(first);
    for (auto &n : rows)
        n["stream"] = streams[0].path;
    plan = evaluate();
    auto input = plan["blocks"][0]["record_input"];
    check(input["status"] == "resolved" && input["end_counter"] == 3 &&
              input["roots"].size() == 2 &&
              input["roots"][1]["input_tree"]["member_record_indices"] == Json::array({2}),
          "capacity hint does not truncate records and nested children are not separate roots");
    // Keep successful earlier roots when a later root fails, without publishing its partial tree.
    put(first, 36 + 36, 2, 4);
    streams[0] = stream("a", 1, 3, first);
    rows = parse_native(first);
    for (auto &n : rows)
        n["stream"] = streams[0].path;
    input = evaluate()["blocks"][0]["record_input"];
    check(input["status"] == "read_error" && input["roots"].size() == 1 &&
              input["failed_root_record_index"] == 1,
          "a later incomplete subtree preserves earlier complete roots only");
    streams[0].compression_offset = 8;
    check(evaluate()["blocks"][0]["record_input"]["status"] == "unresolved",
          "an unconfirmed payload wrapper is not mistaken for a record-only buffer");
    rows = Json::array();
    streams.clear();
    for (unsigned i = 12; i > 0; --i) {
        const auto name = "physical" + std::to_string(i);
        idx["$" + std::to_string(i)] = name;
        auto s = stream(name, 0, i == 12 ? 3 : 2, {});
        s.raw = std::make_shared<Bytes>(header(0, i == 12 ? 3 : 2));
        s.decoded = s.raw;
        s.compression_offset = static_cast<std::size_t>(-1);
        streams.push_back(std::move(s));
    }
    plan = evaluate();
    check(plan["input_order"].size() == 12 && plan["input_order"][9]["logical_name"] == "$10" &&
              plan["input_order"][9]["block_index"] == 2,
          "two-digit block names retain numeric order rather than lexical order");
    auto other = stream("physical1", 0, 3, {});
    other.path[0] = "other-file";
    streams.push_back(other);
    auto contexts = native_input_containers(streams, idx, rows);
    check(contexts.size() == 2 && contexts[0]["input_order"].size() == 12 &&
              contexts[1]["input_order"].size() == 1,
          "identical logical and physical block names in different containers are independent");
    streams.push_back(other);
    contexts = native_input_containers(streams, idx, rows);
    check(contexts[1]["stop"]["reason"] == "ambiguous_block_stream" &&
              contexts[1]["input_order"].empty(),
          "duplicate physical streams within one container do not select an arbitrary first match");
    return checks;
}
