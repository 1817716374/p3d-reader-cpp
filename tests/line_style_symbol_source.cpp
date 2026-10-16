#include "line_style_source.hpp"
#include <p3d/line_style.hpp>
#include <future>

unsigned line_style_symbol_source_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool value, const char *why) {
        require(value, why);
        ++checks;
    };
    auto put = [](Bytes &b, std::size_t p, auto value) {
        require(p <= b.size() && sizeof(value) <= b.size() - p, "symbol source test bounds");
        std::memcpy(b.data() + p, &value, sizeof(value));
    };
    auto symbol = [&](std::uint64_t id) {
        Bytes b(544, 0);
        put(b, 4, std::uint16_t(49));
        put(b, 8, std::uint32_t(270));
        put(b, 12, std::uint32_t(270));
        put(b, 16, std::uint32_t(8));
        put(b, 70, std::uint16_t(1));
        put(b, 204, std::uint16_t(328));
        put(b, 536, id);
        return Json{{"data", rawbytes(b)}};
    };
    const StreamPath model{"models", "A"};
    const Json index = {{"P3D-SMC", "C"}, {"P3D-SMG", "G"}};
    constexpr std::uint64_t first_id = 0xfedcba9800001000ull;
    Json records = Json::array();
    for (unsigned i = 0; i < 63; ++i) {
        Bytes b(i < 31 ? 116 : 156, 0);
        put(b, 4, std::uint16_t(i < 31 ? 19 : 37));
        put(b, 8, std::uint32_t((b.size() - 4) / 2));
        put(b, 12, std::uint32_t((b.size() - 4) / 2));
        put(b, 20, first_id + i);
        const auto parsed = parse_native(b);
        records.push_back(parsed[0]);
    }
    // Heap numbering supplies an independent tree oracle. The actual input
    // headers and registered occurrences follow depth-first file input order.
    Json headers = Json::array();
    std::vector<unsigned> input_order;
    std::function<void(unsigned)> add = [&](unsigned i) {
        if (i >= 63)
            return;
        input_order.push_back(i);
        headers.push_back({{"status", "resolved"},
                           {"native_record_index", i},
                           {"parent_record_index", i ? Json((i - 1) / 2) : Json()},
                           {"output_element_flags", (i < 31 ? 0x40 : 0) | (i ? 0x80 : 0)}});
        add(2 * i + 1);
        add(2 * i + 2);
    };
    add(0);
    Json list = {
        {"status", "resolved"},
        {"system_bootstrap_required", false},
        {"roots",
         Json::array({{{"native_record_index", 0}, {"block_number", 1}, {"headers", headers}}})}};
    Json container = {{"kind", "P3D-SMC"},
                      {"container", StreamPath{"models", "A", "C"}},
                      {"list_preparation", list}};
    const auto ids =
        native_model_id_assignments(Json::array({container}), records, index, model, 0);
    check(ids.at("status") == "resolved" && ids.at("registry").size() == 63,
          "symbol source fixture uses actual initial ID registration");
    auto empty = ids;
    empty["registry"] = Json::array();
    empty["roots"] = Json::array();
    auto read = [&](const Json &s, const Json &owner, const Json &system,
                    std::size_t limit = 100000) {
        return native_line_style_symbol_geometry_source(s, owner, system, records, limit);
    };
    for (unsigned root = 0; root < 63; ++root) {
        std::vector<unsigned> expected;
        std::function<void(unsigned)> collect = [&](unsigned i) {
            if (i >= 63)
                return;
            expected.push_back(i);
            collect(2 * i + 1);
            collect(2 * i + 2);
        };
        collect(root);
        for (bool fallback : {false, true}) {
            const auto view = fallback ? read(symbol(first_id + root), empty, ids)
                                       : read(symbol(first_id + root), ids, Json());
            bool valid = view.at("status") == "selected" &&
                         view.at("source_tree_status") == "resolved" &&
                         view.at("selected_scope") == (fallback ? "system" : "owner") &&
                         view.at("nodes").size() == expected.size() &&
                         view.at("lookup").at("lookups").size() == (fallback ? 2 : 1) &&
                         view.at("runtime_filter") == "not_applied" &&
                         view.at("descriptor_conversion") == "not_evaluated";
            std::map<unsigned, std::size_t> positions;
            for (std::size_t n = 0; n < expected.size(); ++n) {
                const auto i = expected[n];
                const auto &node = view.at("nodes")[n];
                const auto p = i ? (i - 1) / 2 : 0;
                Json parent = n ? Json(positions.at(p)) : Json();
                valid = valid && node.at("native_record_index") == i &&
                        node.at("node_index") == n && node.at("parent_node_index") == parent &&
                        node.at("assigned_id") == first_id + i &&
                        node.at("source_element_id") == first_id + i &&
                        node.at("input_occurrence_index") ==
                            std::size_t(std::find(input_order.begin(), input_order.end(), i) -
                                        input_order.begin());
                positions[i] = n;
            }
            for (std::size_t n = 0; n < expected.size(); ++n) {
                Json children = Json::array();
                const auto i = expected[n];
                if (2 * i + 1 < 63) {
                    children.push_back(positions.at(2 * i + 1));
                    children.push_back(positions.at(2 * i + 2));
                }
                valid = valid && view.at("nodes")[n].at("children") == children;
            }
            check(valid, "every selected subtree preserves scoped registration identity and exact "
                         "parent/child order");
        }
    }
    // Unknown or rejected owner state must not become a system retry. The
    // system registry can contain the same ID with an unrelated occurrence.
    auto alternative = ids;
    alternative["registry"][0]["native_record_index"] = 62;
    auto j = read(symbol(first_id), ids, alternative);
    check(j.at("status") == "selected" && j.at("target").at("native_record_index") == 0 &&
              j.at("lookup").at("lookups").size() == 1,
          "same numeric system ID does not replace an owner hit");
    auto partial = ids;
    partial["status"] = "partial";
    check(read(symbol(first_id), partial, ids).at("reason") == "complete_owner_registry_required",
          "partial owner registry cannot establish a miss or a system fallback");
    check(read(symbol(first_id), empty, Json()).at("reason") == "complete_system_registry_required",
          "unknown system registry is not empty");
    check(read(symbol(first_id + 100), ids, empty).at("status") == "missing",
          "two complete misses do not trigger an unrelated source scan");
    check(read(symbol(0), Json(), Json(), 0).at("status") == "not_requested",
          "zero reference bypasses owner system and node budget independently of other source "
          "fields");
    auto duplicated = ids;
    duplicated["registry"].push_back(duplicated["registry"][0]);
    check(read(symbol(first_id), duplicated, ids).at("status") == "not_evaluated",
          "duplicate registration does not silently select a source");
    auto inconsistent = ids;
    inconsistent["registry"][0]["input_occurrence_index"] = 1;
    check(read(symbol(first_id), inconsistent, ids).at("reason") ==
              "symbol_geometry_registry_identity_mismatch",
          "registry identity must agree with its registered occurrence");
    inconsistent = ids;
    inconsistent["roots"][0]["records"][1]["input_occurrence_index"] = 0;
    check(read(symbol(first_id), inconsistent, ids).at("reason") ==
              "ambiguous_symbol_geometry_input_occurrence",
          "ambiguous occurrence metadata is not merged");
    inconsistent = ids;
    inconsistent["roots"][0]["records"][0]["parent_input_occurrence_index"] = 1;
    check(read(symbol(first_id), inconsistent, ids).at("reason") ==
              "cyclic_symbol_geometry_source_tree",
          "cycles are reported instead of duplicating source geometry indefinitely");
    j = read(symbol(first_id), ids, empty, 2);
    check(j.at("status") == "not_evaluated" && j.at("nodes").size() == 2 &&
              j.at("source_tree_status") == "not_evaluated" &&
              j.at("reason") == "symbol_geometry_source_node_limit",
          "bounded tree reports its known prefix without publishing a complete subtree");
    check(read(symbol(first_id + 62), ids, empty, 0).at("reason") ==
                  "symbol_geometry_source_node_limit" &&
              read(symbol(first_id + 62), ids, empty, 1).at("status") == "selected",
          "node budget applies to selected nodes rather than the whole owner registry");
    // A physical source container may be loaded twice. The second tree has
    // distinct assigned IDs and occurrences although all source indices agree.
    auto alias_index = index;
    alias_index["P3D-SMG"] = "C";
    const auto aliased =
        native_model_id_assignments(Json::array({container}), records, alias_index, model, 0);
    const auto alias_root_id = aliased.at("roots")[1].at("records")[0].at("assigned_id");
    const auto first = read(symbol(first_id), aliased, {});
    const auto second = read(symbol(alias_root_id.get<std::uint64_t>()), aliased, {});
    bool same_source_distinct_occurrences =
        first.at("nodes").size() == 63 && second.at("nodes").size() == 63;
    for (std::size_t n = 0; n < 63; ++n)
        same_source_distinct_occurrences =
            same_source_distinct_occurrences &&
            first.at("nodes")[n].at("native_record_index") ==
                second.at("nodes")[n].at("native_record_index") &&
            first.at("nodes")[n].at("input_occurrence_index") !=
                second.at("nodes")[n].at("input_occurrence_index") &&
            first.at("nodes")[n].at("assigned_id") != second.at("nodes")[n].at("assigned_id") &&
            first.at("nodes")[n].at("source_element_id") ==
                second.at("nodes")[n].at("source_element_id");
    check(same_source_distinct_occurrences, "native repeated input occurrences retain original "
                                            "source identity without invented deduplication");
    auto foreign = container;
    foreign["container"] = StreamPath{"models", "B", "C"};
    const auto actual_owner =
        native_model_id_assignments(Json::array({foreign}), records, index, model, 0);
    check(read(symbol(first_id), actual_owner, ids).at("selected_scope") == "system",
          "a foreign model with the same source IDs does not enter the owner registry");
    auto high_ids = ids;
    high_ids["registry"][0]["id"] = UINT64_MAX;
    high_ids["roots"][0]["records"][0]["assigned_id"] = UINT64_MAX;
    j = read(symbol(UINT64_MAX), high_ids, {});
    check(j.at("status") == "selected" && j.at("requested_id") == UINT64_MAX,
          "full unsigned maximum is an ordinary nonzero symbol geometry reference");
    auto malformed = symbol(first_id);
    auto bytes = bytesof(malformed.at("data"));
    bytes.resize(540);
    malformed["data"] = rawbytes(bytes);
    check(read(malformed, ids, ids).at("status") == "not_evaluated" &&
              !read(malformed, ids, ids).contains("lookup"),
          "missing reference bytes are rejected before registry access");
    auto broken_buffer = symbol(first_id);
    bytes = bytesof(broken_buffer.at("data"));
    put(bytes, 532, UINT32_MAX);
    broken_buffer["data"] = rawbytes(bytes);
    check(read(broken_buffer, ids, {}).at("status") == "selected",
          "independent geometry reference can bind even when the embedded buffer is incomplete");
    const auto original = ids;
    std::array<std::future<Json>, 4> tasks;
    for (auto &task : tasks)
        task = std::async(std::launch::async, [&] { return read(symbol(first_id + 3), ids, {}); });
    const auto expected = read(symbol(first_id + 3), ids, {});
    for (auto &task : tasks)
        check(task.get() == expected, "symbol source lookup is immutable and thread independent");
    check(ids == original, "source graph queries do not rewrite registered IDs flags or records");
    return checks;
}
