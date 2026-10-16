#include "line_style_source.hpp"
#include <future>

unsigned line_style_symbol_descriptor_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool value, const char *why) {
        require(value, why);
        ++checks;
    };
    auto put = [](Bytes &b, std::size_t p, auto value) {
        require(p <= b.size() && sizeof(value) <= b.size() - p, "descriptor fixture bounds");
        std::memcpy(b.data() + p, &value, sizeof(value));
    };
    auto make = [&](const std::vector<unsigned> &types, const std::vector<unsigned> &flags,
                    const std::vector<std::int32_t> &counts, const std::vector<int> &parents) {
        Json source = {{"status", "selected"},
                       {"preparation_status", "prepared"},
                       {"scope", "initial_symbol_geometry_source_record_preparation"},
                       {"selected_scope", "owner"},
                       {"nodes", Json::array()},
                       {"prepared_nodes", Json::array()}};
        for (std::size_t i = 0; i < types.size(); ++i) {
            const auto id = std::uint64_t(0xfedcba9800000000ull + i);
            Bytes b(156, 0);
            put(b, 4, std::uint16_t(types[i]));
            put(b, 6, std::uint16_t(flags[i]));
            put(b, 8, std::uint32_t(76));
            put(b, 12, std::uint32_t(76));
            put(b, 20, id);
            put(b, 36, std::int32_t(flags[i] & 0x20 ? counts[i] > 0 ? 0 : 1 : counts[i]));
            put(b, 108, std::int32_t(flags[i] & 0x20 ? counts[i] : counts[i] > 0 ? 0 : 1));
            source["nodes"].push_back(
                {{"node_index", i},
                 {"native_record_index", i + 100},
                 {"input_occurrence_index", i + 200},
                 {"assigned_id", id},
                 {"element_type", types[i]},
                 {"parent_node_index", parents[i] < 0 ? Json() : Json(parents[i])},
                 {"children", Json::array()}});
            source["prepared_nodes"].push_back({{"source_node_index", i},
                                                {"native_record_index", i + 100},
                                                {"input_occurrence_index", i + 200},
                                                {"preparation",
                                                 {{"status", "prepared"},
                                                  {"decoded_status", "decoded"},
                                                  {"prepared_record", rawbytes(b)}}}});
        }
        for (std::size_t i = 0; i < parents.size(); ++i)
            if (parents[i] >= 0)
                source["nodes"][parents[i]]["children"].push_back(i);
        return source;
    };
    // Independent category table: the middle native range remains a leaf
    // regardless of bit 0x40, while the two outer ranges use that flag.
    const std::array<std::pair<unsigned, int>, 11> categories{{{9, -1},
                                                               {10, 1},
                                                               {19, 1},
                                                               {24, 1},
                                                               {32, 1},
                                                               {33, 0},
                                                               {37, 0},
                                                               {49, 0},
                                                               {63, 0},
                                                               {64, -1},
                                                               {65535, -1}}};
    for (const auto [type, category] : categories)
        for (auto flags : {0u, 0x20u, 0x40u, 0x60u})
            for (auto count : {-1, 0, 1, INT32_MAX})
                for (const auto *scope : {"owner", "system"}) {
                    auto source =
                        make({10, type, 37, 37}, {0, flags, 0, 0}, {0, count, 0, 0}, {-1, 0, 1, 0});
                    source["selected_scope"] = scope;
                    const auto before = source;
                    const auto result = native_line_style_symbol_initial_descriptor(source, 100000);
                    const bool grouped = category == 1 || (category == -1 && (flags & 0x40));
                    const bool nested = grouped && count > 0;
                    const std::vector<unsigned> order =
                        nested ? std::vector<unsigned>{0, 1, 2, 3} : std::vector<unsigned>{0, 1, 3};
                    bool valid = result.at("status") == "built" &&
                                 result.at("symbol_copy_status") == "copied" &&
                                 result.at("root_descriptor_index") == 0 &&
                                 result.at("nodes").size() == order.size() &&
                                 result.at("copied_native_bytes") == 152 * order.size() &&
                                 source == before &&
                                 result.at("unvisited_source_node_indices") ==
                                     (nested ? Json::array() : Json::array({2}));
                    for (std::size_t n = 0; n < order.size(); ++n) {
                        const auto i = order[n];
                        const auto &node = result.at("nodes")[n];
                        auto expected = bytesof(
                            source.at("prepared_nodes")[i].at("preparation").at("prepared_record"));
                        if (i)
                            put(expected, 6,
                                std::uint16_t(Reader(expected).at<std::uint16_t>(6) | 0x80));
                        valid =
                            valid && node.at("source_node_index") == i &&
                            node.at("native_record_index") == i + 100 &&
                            node.at("input_occurrence_index") == i + 200 &&
                            node.at("lookup_scope") == scope &&
                            bytesof(node.at("native_header")) ==
                                slice(expected, 4, expected.size() - 4) &&
                            node.at("native_flag_34") == 0 &&
                            node.at("construction_native_flag_34") ==
                                (i == 0 || (i == 1 && nested) ? 1 : 0) &&
                            node.at("native_flag_30") == (i == 0 || (i == 1 && grouped) ? 1 : 0);
                    }
                    const auto &nodes = result.at("nodes");
                    const auto last = order.size() - 1;
                    valid = valid && nodes[0].at("children") == Json::array({1, last}) &&
                            nodes[0].at("parent_descriptor_index").is_null() &&
                            nodes[0].at("back_link_index").is_null() &&
                            nodes[0].at("native_group_index") == 0 &&
                            nodes[0].at("next_sibling_index").is_null() &&
                            nodes[1].at("back_link_index") == 0 &&
                            nodes[1].at("native_group_index") == 0 &&
                            nodes[1].at("next_sibling_index") == last &&
                            nodes[last].at("back_link_index") == 1 &&
                            nodes[last].at("parent_descriptor_index") == 0 &&
                            nodes[last].at("next_sibling_index").is_null();
                    if (nested)
                        valid = valid && nodes[1].at("children") == Json::array({2}) &&
                                nodes[2].at("back_link_index") == 1 &&
                                nodes[2].at("native_group_index") == 1;
                    else
                        valid = valid && nodes[1].at("children").empty();
                    check(valid, "native descriptor root/count priority group/back/sibling links "
                                 "and owned clone reset match independent category oracle");
                }
    auto source = make({37, 37}, {0xc0, 0}, {0, 0}, {-1, 0});
    auto result = native_line_style_symbol_initial_descriptor(source, 100000);
    check(result.at("status") == "built" && result.at("nodes").size() == 1 &&
              result.at("unvisited_source_node_indices") == Json::array({1}) &&
              result.at("nodes")[0].at("native_group_index").is_null() &&
              Reader(bytesof(result.at("nodes")[0].at("native_header"))).at<std::uint16_t>(2) ==
                  0xc0,
          "leaf root does not walk children and an existing source child bit is not cleared for "
          "the new root");
    source = make({10, 49, 37}, {0, 0, 0}, {0, 0, 0}, {-1, 0, 0});
    source["prepared_nodes"][1]["preparation"]["decoded_status"] = "not_decoded";
    auto b = bytesof(source["prepared_nodes"][1]["preparation"]["prepared_record"]);
    put(b, 12, std::uint32_t(392));
    source["prepared_nodes"][1]["preparation"]["prepared_record"] = rawbytes(b);
    result = native_line_style_symbol_initial_descriptor(source, 100000);
    check(result.at("status") == "built" && result.at("nodes").size() == 3 &&
              result.at("nodes")[1].at("prepared_stream_decode_status") == "not_decoded" &&
              Reader(bytesof(result.at("nodes")[1].at("native_header"))).at<std::uint32_t>(8) ==
                  392,
          "descriptor copying preserves a known prepared boundary mismatch without requiring "
          "stream linkage re-decoding");
    source = make({10, 37, 37}, {0, 0, 0}, {0, 0, 0}, {-1, 0, 0});
    result = native_line_style_symbol_initial_descriptor(source, 456);
    check(result.at("status") == "built" && result.at("nodes").size() == 3 &&
              native_line_style_symbol_initial_descriptor(source, 455).at("reason") ==
                  "symbol_descriptor_byte_limit",
          "descriptor byte budget counts exact unframed native bytes");
    result = native_line_style_symbol_initial_descriptor(source, 152);
    check(result.at("status") == "not_evaluated" && result.at("nodes").size() == 1 &&
              result.at("symbol_copy_status") == "not_evaluated" &&
              result.at("root_descriptor_index").is_null() &&
              result.at("pending_source_node_index") == 1,
          "budget failure retains only a known prefix and does not publish an owned complete "
          "descriptor");
    auto invalid = source;
    invalid["nodes"][1]["parent_node_index"] = 2;
    check(native_line_style_symbol_initial_descriptor(invalid, 100000).at("reason") ==
              "symbol_descriptor_source_parent_mismatch",
          "descriptor walk validates parent occurrence mapping");
    invalid = source;
    invalid["nodes"][0]["children"] = Json::array({0});
    check(native_line_style_symbol_initial_descriptor(invalid, 100000).at("reason") ==
              "cyclic_symbol_descriptor_source_tree",
          "cycles cannot repeatedly clone the same input occurrence");
    invalid = source;
    invalid["prepared_nodes"][1]["source_node_index"] = 2;
    check(native_line_style_symbol_initial_descriptor(invalid, 100000).at("reason") ==
              "symbol_descriptor_source_identity_mismatch",
          "prepared source identity must match the source graph");
    invalid = source;
    b = bytesof(invalid["prepared_nodes"][1]["preparation"]["prepared_record"]);
    put(b, 20, std::uint64_t(99));
    invalid["prepared_nodes"][1]["preparation"]["prepared_record"] = rawbytes(b);
    check(native_line_style_symbol_initial_descriptor(invalid, 100000).at("reason") ==
              "symbol_descriptor_prepared_header_identity_mismatch",
          "assigned full ID is checked against copied native header");
    invalid = source;
    b.resize(131076);
    put(b, 8, std::uint32_t(65536));
    invalid["prepared_nodes"][1]["preparation"]["prepared_record"] = rawbytes(b);
    check(native_line_style_symbol_initial_descriptor(invalid, 1000000).at("reason") ==
              "ordinary_initial_native_header_copy_not_loaded",
          "oversize layout cannot be mistaken for a successfully loaded ordinary native header");
    source = make({37}, {0}, {0}, {-1});
    b = bytesof(source["prepared_nodes"][0]["preparation"]["prepared_record"]);
    b.resize(131074);
    put(b, 8, std::uint32_t(65535));
    source["prepared_nodes"][0]["preparation"]["prepared_record"] = rawbytes(b);
    result = native_line_style_symbol_initial_descriptor(source, 131070);
    check(result.at("status") == "built" && result.at("copied_native_bytes") == 131070 &&
              bytesof(result.at("nodes")[0].at("native_header")).size() == 131070,
          "largest accepted native header is copied in full without dropping its tail");
    Json skipped = {{"status", "not_requested"}, {"preparation_status", "not_requested"}};
    check(native_line_style_symbol_initial_descriptor(skipped, 0).at("status") == "not_requested",
          "zero reference bypasses descriptor allocation and byte budget");
    check(native_line_style_symbol_initial_descriptor(Json::object(), 1).at("status") ==
              "not_evaluated",
          "missing source preparation is reported without throwing");
    std::vector<unsigned> types(1024, 10), flags(1024, 0);
    std::vector<std::int32_t> counts(1024, 1);
    std::vector<int> parents(1024);
    parents[0] = -1;
    for (unsigned i = 1; i < 1024; ++i)
        parents[i] = int(i) - 1;
    types.back() = 37;
    source = make(types, flags, counts, parents);
    result = native_line_style_symbol_initial_descriptor(source, 1024 * 152);
    check(result.at("status") == "built" && result.at("nodes").size() == 1024 &&
              result.at("nodes")[1023].at("back_link_index") == 1022,
          "deep descriptor trees use bounded iterative storage instead of the native recursive "
          "stack");
    source = make({10, 37, 37}, {0, 0, 0}, {0, 0, 0}, {-1, 0, 0});
    const auto before = source;
    const auto expected = native_line_style_symbol_initial_descriptor(source, 100000);
    std::array<std::future<Json>, 4> tasks;
    for (auto &task : tasks)
        task = std::async(std::launch::async, [&] {
            return native_line_style_symbol_initial_descriptor(source, 100000);
        });
    for (auto &task : tasks)
        check(task.get() == expected,
              "descriptor copies are immutable and independent across threads");
    check(source == before && expected.at("nodes").size() == 3,
          "independent source occurrences are retained without content-based deduplication");
    return checks;
}
