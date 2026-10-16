#include "scene_layers.hpp"
using namespace p3d;
unsigned scene_layers_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        require(ok, message);
        ++checks;
    };
    auto set = [](Bytes &bytes, std::size_t offset, auto value) {
        std::memcpy(bytes.data() + offset, &value, sizeof(value));
    };
    auto record = [&](unsigned type, std::uint32_t layer) {
        Bytes bytes(40, 0);
        set(bytes, 4, std::uint16_t(type));
        set(bytes, 8, std::uint32_t(18));
        set(bytes, 12, std::uint32_t(18));
        set(bytes, 16, layer);
        return parse_native(bytes).at(0);
    };
    for (unsigned type : {19u, 20u, 21u, 37u, 40u, 44u, 52u, 54u, 62u, 97u}) {
        auto source = record(type, 0xfedcba98u);
        const auto &header = source.at("native_graphic_header");
        check(header.at("status") == "decoded" && header.at("source_offset") == 16 &&
                  header.at("layer_id") == 0xfedcba98u &&
                  header.at("entity_getter_policy") ==
                      (type == 21 ? "recursive_first_child" : "stored_header") &&
                  Reader(bytesof(source.at("data"))).at<std::uint32_t>(16) == 0xfedcba98u,
              "supported native graphic header preserves its unsigned saved layer input");
    }
    for (unsigned type : {10u, 33u, 47u, 49u})
        check(!record(type, 123).contains("native_graphic_header"),
              "nongraphic subtype fields are not reinterpreted as layer IDs");
    Json native = Json::array();
    for (std::uint64_t id : {std::uint64_t(100), UINT64_MAX - 1})
        native.push_back({{"id", id},
                          {"stream", {"system", "layers"}},
                          {"offset", std::uint64_t(0x100000000ull + native.size() * 108)},
                          {"layer_definition",
                           {{"status", "decoded"},
                            {"layer_id", 64},
                            {"name", id == 100 ? "before" : "after"},
                            {"material_reference", {{"material_id", 31}}}}}});
    Json catalog = {
        {"tables", {{{"membership_status", "resolved"}, {"member_record_indices", {0, 1}}}}},
        {"model_references",
         {{{"model_id", "7"},
           {"initial_layer_cache", {{"status", "resolved"}, {"table_index", 0}}}},
          {{"model_id", "8"},
           {"initial_layer_cache", {{"status", "resolved"}, {"table_index", 0}}}}}}};
    const auto source_before = native;
    NativeSceneLayerCatalog layers(native, catalog);
    const auto header = record(37, 64).at("native_graphic_header");
    auto ref = layers.bind(7, header);
    check(ref.at("status") == "resolved" && ref.at("layer_definition_index") == 1 &&
              ref.at("candidate_definition_indices") == Json::array({0, 1}) &&
              ref.at("native_record_index") == 1 && native == source_before,
          "stored layer reference uses last saved duplicate ID and keeps every source definition");
    check(layers.data.at("definitions").size() == 2 &&
              layers.bind(8, header).at("layer_definition_index") ==
                  ref.at("layer_definition_index") &&
              layers.data.at("definitions")[1]["record_id"] == UINT64_MAX - 1 &&
              layers.data.at("definitions")[1]["offset"] == 0x10000006cull,
          "models share native layer definitions without copying or truncating source identity");
    check(
        ref.at("material_loading") == "not_evaluated" &&
            ref.at("draw_style_selection") == "not_evaluated" &&
            layers.data.at("definitions")[1]["layer"]["material_reference"]["material_id"] == 31,
        "stored layer relation preserves material input without pretending to load final material");
    check(layers.bind(7, record(21, 64).at("native_graphic_header")).at("reason") ==
              "entity_first_child_layer_query_required",
          "group header layer cannot replace its native recursive first-child getter");
    for (auto id : {UINT32_MAX, UINT32_MAX - 1})
        check(layers.bind(7, record(37, id).at("native_graphic_header")).at("reason") ==
                  "layer_style_context_required",
              "inheritance layer inputs do not select a same-number source row");
    check(layers.bind(7, record(37, 65).at("native_graphic_header")).at("status") == "missing",
          "missing layer input does not silently choose a default layer");
    check(layers.bind(99, header).at("status") == "not_evaluated" &&
              layers.bind(7, Json()).at("status") == "not_evaluated",
          "missing model or source header preserves unknown relation");
    auto changed = catalog;
    changed["model_references"].push_back(changed["model_references"][0]);
    check(NativeSceneLayerCatalog(native, changed).bind(7, header).at("status") == "not_evaluated",
          "duplicate model heads are not merged even if their selected tables agree");
    changed = catalog;
    changed["model_references"][0]["initial_layer_cache"]["status"] = "unresolved";
    check(NativeSceneLayerCatalog(native, changed).bind(7, header).at("status") == "not_evaluated",
          "unresolved initial model table blocks a saved layer association");
    changed = catalog;
    changed["tables"][0]["membership_status"] = "invalid";
    check(NativeSceneLayerCatalog(native, changed).bind(7, header).at("status") == "not_evaluated",
          "invalid table members cannot expose a partial layer lookup");
    auto broken = native;
    broken[1]["layer_definition"]["status"] = "unsupported_version";
    check(NativeSceneLayerCatalog(broken, catalog).bind(7, header).at("status") == "not_evaluated",
          "unknown later source layer does not publish an earlier same-ID candidate");
    return checks;
}
