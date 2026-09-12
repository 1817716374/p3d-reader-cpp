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
    return checks;
}
