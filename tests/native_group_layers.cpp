#include "native_group_layers.hpp"
#include "scene_layers.hpp"
using namespace p3d;
unsigned native_group_layers_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        require(ok, message);
        ++checks;
    };
    auto make = [](unsigned type, std::uint32_t layer, std::uint32_t descendants = 0,
                   unsigned flags = 0x20, std::uint32_t prefix = 0) {
        Bytes bytes(260, 0);
        auto set = [&](std::size_t at, auto value) {
            std::memcpy(bytes.data() + at, &value, sizeof(value));
        };
        set(0, prefix);
        set(4, std::uint16_t(type));
        set(6, std::uint16_t(flags));
        set(8, std::uint32_t(128));
        set(12, std::uint32_t(128));
        set(16, layer);
        set((flags & 0x20) ? 108 : 36, descendants);
        return Json{{"element_type", type},
                    {"element_flags", flags},
                    {"id", 900},
                    {"offset", 0},
                    {"length", 260},
                    {"stream", {"models", "M", "graphics", "$1"}},
                    {"data", rawbytes(bytes)},
                    {"children", Json::array({9999})}};
    };
    auto offset_records = [](Json records) {
        for (std::size_t i = 0; i < records.size(); ++i)
            records[i]["offset"] = i * 260;
        return records;
    };
    auto prepare = [&](const Json &records, bool repeat = false) {
        const auto tree = native_record_input_subtree(0, records, 1);
        Json roots = Json::array();
        if (tree.at("status") == "resolved")
            roots.push_back({{"native_record_index", 0}, {"input_tree", tree}});
        Json container = {
            {"container", {"models", "M", "graphics"}},
            {"kind", "P3D-SMG"},
            {"input_order", {{{"block_index", 0}, {"block_number", 1}}}},
            {"blocks", {{{"record_input", {{"status", tree.at("status")}, {"roots", roots}}}}}}};
        if (repeat)
            container["input_order"].push_back({{"block_index", 0}, {"block_number", 2}});
        container["list_preparation"] = native_list_input_preparation(container, records);
        return Json::array({container});
    };
    auto records = offset_records({make(21, 0, 3), make(21, 0, 1), make(37, 64), make(37, 99)});
    const auto before = records;
    auto containers = prepare(records);
    NativeGroupLayerQueries queries(records, containers, {0, 1});
    auto query = queries.data.at("queries")[0];
    check(query.at("status") == "decoded" && query.at("layer_id") == 64 &&
              queries.data.at("queries")[1].at("layer_id") == 64 && records == before,
          "nested group getter follows first child recursively without changing saved records");
    check(queries.data.at("occurrences").size() == 4 &&
              queries.data.at("occurrences")[0]["children"] == Json::array({1, 3}) &&
              queries.data.at("occurrences")[1]["children"] == Json::array({2}) &&
              queries.data.at("occurrences")[0]["layer_query"]["terminal_occurrence_index"] == 2,
          "published source graph keeps direct-child order rather than flattening all descendants");
    check(queries.data.at("occurrences")[1]["parent"] == 0 &&
              queries.data.at("occurrences")[2]["parent"] == 1 &&
              queries.data.at("occurrences")[2]["source_id"] == 900 &&
              queries.data.at("occurrences")[2]["offset"] == 520,
          "parent links use input identities even when all saved object IDs are equal");
    NativeGroupLayerQueries repeated(records, prepare(records, true), {0});
    check(repeated.data.at("queries")[0]["occurrence_indices"] == Json::array({0, 4}) &&
              repeated.data.at("occurrences").size() == 8 &&
              repeated.data.at("occurrences")[6]["parent"] == 5 &&
              repeated.data.at("input_roots")[1]["block_number"] == 2 &&
              repeated.data.at("containers")[0]["container"] == containers[0]["container"],
          "repeated source input keeps two child graphs and original block identities");
    records = offset_records({make(21, 0, 1), make(37, 1, 0, 0x28), make(37, 77)});
    query = NativeGroupLayerQueries(records, prepare(records), {0}).data.at("queries")[0];
    check(query.at("layer_id") == 77,
          "skipped physical neighbor is not the loaded first child even after the nominal count");
    records = offset_records({make(21, 0, 1), make(37, 1, 0, 0x20, 9), make(37, 88)});
    check(
        NativeGroupLayerQueries(records, prepare(records), {0}).data.at("queries")[0]["layer_id"] ==
            88,
        "nonzero-prefix record is skipped by the existing native input reader");
    records = offset_records({make(21, 0, 2), make(19, 55, 1), make(37, 66)});
    check(
        NativeGroupLayerQueries(records, prepare(records), {0}).data.at("queries")[0]["layer_id"] ==
            55,
        "getter stops at a non-21 child even when that child is itself compound");
    records = offset_records({make(21, 0, 1), make(47, 123)});
    check(
        NativeGroupLayerQueries(records, prepare(records), {0}).data.at("queries")[0]["layer_id"] ==
            123,
        "non-21 getter reads the native header word without inventing a graphic child filter");
    records = offset_records({make(21, 0, 1), make(24, 42)});
    check(
        NativeGroupLayerQueries(records, prepare(records), {0}).data.at("queries")[0]["layer_id"] ==
            42,
        "known input header conversion preserves the word used by this getter");
    records = offset_records({make(21, 0, 2), make(21, 0), make(37, 99)});
    queries = NativeGroupLayerQueries(records, prepare(records), {0, 1});
    query = queries.data.at("queries")[0];
    check(query.at("status") == "decoded" && query.at("layer_id") == UINT32_MAX &&
              query.at("empty_group_result") == true &&
              queries.data.at("occurrences")[0]["layer_query"]["terminal_occurrence_index"] == 1,
          "empty first subgroup yields all-ones without searching a later sibling");
    auto unknown = containers;
    unknown[0]["list_preparation"]["roots"] = Json::array();
    auto root_only = offset_records({make(21, 0)});
    check(NativeGroupLayerQueries(root_only, unknown, {0}).data.at("queries")[0]["status"] ==
              "not_evaluated",
          "unreached group is not confused with a confirmed empty loaded group");
    records = offset_records({make(21, 0, 1)});
    check(NativeGroupLayerQueries(records, prepare(records), {0}).data.at("occurrences").empty(),
          "truncated physical subtree cannot publish a partial input graph");
    records = offset_records({make(21, 0, 2), make(37, 64), make(37, 65)});
    containers = prepare(records);
    unknown = containers;
    unknown[0]["list_preparation"]["roots"][0]["headers"][2]["status"] = "unresolved";
    queries = NativeGroupLayerQueries(records, unknown, {0});
    check(queries.data.at("queries")[0]["status"] == "not_evaluated" &&
              queries.data.at("occurrences").empty() && queries.data.at("input_roots").empty(),
          "unknown later preparation prevents publishing an apparently usable prefix graph");
    unknown = containers;
    unknown[0]["list_preparation"]["roots"][0]["headers"][1]["parent_record_index"] = 1;
    check(NativeGroupLayerQueries(records, unknown, {0}).data.at("occurrences").empty(),
          "self or later parent cannot create a fabricated child cycle");
    unknown = containers;
    unknown[0]["list_preparation"]["roots"][0]["headers"][2]["native_record_index"] = 1;
    check(NativeGroupLayerQueries(records, unknown, {0}).data.at("queries")[0]["status"] ==
              "not_evaluated",
          "duplicate source rows inside one input root are rejected rather than merged");
    Json deep = Json::array();
    std::vector<std::size_t> targets;
    for (unsigned i = 0; i < 512; ++i) {
        deep.push_back(make(21, 0, 512 - i));
        targets.push_back(i);
    }
    deep.push_back(make(37, 91));
    deep = offset_records(std::move(deep));
    queries = NativeGroupLayerQueries(deep, prepare(deep), targets);
    check(
        queries.data.at("queries").size() == 512 && queries.data.at("occurrences").size() == 513 &&
            queries.data.at("queries")[0]["layer_id"] == 91 &&
            queries.data.at("occurrences")[0]["layer_query"]["terminal_occurrence_index"] == 512,
        "deep group queries share one iterative input graph without copying every ancestor chain");
    const Json catalog = {
        {"tables", {{{"membership_status", "resolved"}, {"member_record_indices", {0}}}}},
        {"model_references",
         {{{"model_id", "7"},
           {"initial_layer_cache", {{"status", "resolved"}, {"table_index", 0}}}}}}};
    const Json layers = {{{"id", 100},
                          {"stream", {"system", "layers"}},
                          {"offset", 0},
                          {"layer_definition", {{"status", "decoded"}, {"layer_id", 64}}}}};
    NativeSceneLayerCatalog layer_catalog(layers, catalog);
    const Json header = {
        {"status", "decoded"}, {"layer_id", 0}, {"entity_getter_policy", "recursive_first_child"}};
    records = offset_records({make(21, 0, 1), make(37, 64)});
    query = NativeGroupLayerQueries(records, prepare(records), {0}).data.at("queries")[0];
    auto binding = layer_catalog.bind(7, header, query);
    check(
        binding.at("status") == "resolved" && binding.at("layer_id") == 64 &&
            binding.at("saved_header_layer_id") == 0 && binding.at("entity_layer_query_index") == 0,
        "scene binding uses evaluated child layer while retaining the group's saved header value");
    query = NativeGroupLayerQueries(root_only, prepare(root_only), {0}).data.at("queries")[0];
    check(layer_catalog.bind(7, header, query).at("status") == "none",
          "confirmed empty group has no stored layer association rather than an invented default");
    check(layer_catalog.bind(7, header).at("status") == "not_evaluated",
          "group header alone still cannot bypass its child-query requirement");
    return checks;
}
