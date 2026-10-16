#include "internal.hpp"
#include <p3d/block_layers.hpp>
using namespace p3d;
unsigned block_layers_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        require(ok, message);
        ++checks;
    };
    auto set = [](Bytes &b, std::size_t at, auto v) { std::memcpy(b.data() + at, &v, sizeof(v)); };
    auto header = [&](unsigned type, std::uint32_t layer, unsigned flags = 0) {
        Bytes b(260, 0);
        set(b, 4, std::uint16_t(type));
        set(b, 8, std::uint32_t(128));
        set(b, 12, std::uint32_t(128));
        set(b, 16, layer);
        set(b, 112, std::uint16_t(flags));
        return Json{{"data", rawbytes(b)},
                    {"stream", {"models", "M", "graphics", "$1"}},
                    {"offset", 700},
                    {"id", 0xfedcba9876543210ull}};
    };
    Bytes source(68, 0);
    set(source, 4, std::uint16_t(10));
    set(source, 8, std::uint32_t(32));
    set(source, 12, std::uint32_t(32));
    set(source, 16, std::uint32_t(1));
    set(source, 36, std::uint32_t(4));
    for (auto pair :
         {std::pair<std::uint32_t, std::uint32_t>{10, 4}, {20, 7}, {30, 0}, {UINT32_MAX - 1, 1}}) {
        Bytes b(108, 0);
        set(b, 4, std::uint16_t(49));
        set(b, 6, std::uint16_t(0x80));
        set(b, 8, std::uint32_t(52));
        set(b, 12, std::uint32_t(52));
        set(b, 16, std::uint32_t(1));
        set(b, 36, pair.first);
        set(b, 44, std::uint16_t(7));
        set(b, 96, pair.second);
        source.insert(source.end(), b.begin(), b.end());
    }
    auto records = parse_native(source);
    for (auto &r : records)
        r["stream"] = {"system", "layers"};
    const NativeLayerNumericIndex index(records, 0), unknown(Json(), 0);
    check(index.lookup_layer(10)["numeric_key"] == 4 &&
              index.lookup_layer(10)["native_record_index"] == 1 &&
              index.lookup_layer(99)["status"] == "not_found" &&
              unknown.lookup_layer(10)["status"] == "unresolved",
          "exact layer-ID query distinguishes loaded source, confirmed miss and unknown cache");
    auto instance = header(62, 20, 0xfffa), definition = header(32, 10);
    auto bytes = bytesof(instance["data"]);
    set(bytes, 48, std::uint32_t(0x7fffffff));
    set(bytes, 52, std::uint32_t(UINT32_MAX));
    set(bytes, 56, std::uint32_t(UINT32_MAX - 1));
    instance["data"] = rawbytes(bytes);
    const auto before = instance;
    auto parsed = parse_native(bytes)[0]["block_layer_inputs"];
    check(parsed["status"] == "decoded" && parsed["layer_id"] == 20 &&
              parsed["override_flags"] == 0xfffa && parsed["force_layer"] == false &&
              parsed["use_numeric_offset"] == true && parsed["other_override_bits"] == 0xfff8 &&
              parsed["source_symbology"]["line_style"] == 0x7fffffff &&
              parsed["source_symbology"]["line_weight"] == UINT32_MAX &&
              parsed["source_symbology"]["color"] == UINT32_MAX - 1,
          "saved block input flags and unsigned symbology retain independent source fields");
    auto prep = prepare_native_block_parent_layer(instance, definition, index);
    check(prep.data["status"] == "decoded" && prep.rule && prep.rule->numeric_offset == 3 &&
              prep.data["instance_source"]["id"] == 0xfedcba9876543210ull && instance == before,
          "block layer offset uses instance numeric key minus selected definition key without "
          "mutation");
    check(apply_native_parent_layer_rule(10, false, *prep.rule, index)["layer_id"] == 20,
          "prepared offset maps a definition-layer child into the instance layer");
    check(apply_native_parent_layer_rule(10, false, *prep.rule, index)["protected_layer"] == true &&
              apply_native_parent_layer_rule(0, false, *prep.rule, index)["protected_layer"] ==
                  false,
          "offset preparation marks nonzero layers protected but zero-layer bypass leaves the bit "
          "clear");
    check(apply_native_parent_layer_rule(99, false, *prep.rule, index)["layer_id"] == 64 &&
              apply_native_parent_layer_rule(20, false, *prep.rule, index)["layer_id"] == 64,
          "known source and target misses retain the native fallback number");
    prep = prepare_native_block_parent_layer(header(62, 99, 2), definition, index);
    check(prep.rule->numeric_offset == std::uint32_t(0u - 4u) &&
              prep.data["instance_layer_lookup"]["status"] == "not_found",
          "missing instance layer contributes key zero rather than fallback layer 64");
    prep = prepare_native_block_parent_layer(instance, header(32, 99), index);
    check(prep.rule->numeric_offset == 7 &&
              prep.data["definition_layer_lookup"]["status"] == "not_found",
          "missing definition layer independently contributes key zero");
    prep = prepare_native_block_parent_layer(header(62, 99, 2), header(32, 98), index);
    check(prep.rule->numeric_offset == 0,
          "two confirmed missing layers yield a known zero difference");
    prep = prepare_native_block_parent_layer(instance, definition, unknown);
    check(prep.data["status"] == "partial" && prep.rule && !prep.rule->numeric_offset &&
              apply_native_parent_layer_rule(10, false, *prep.rule, unknown)["status"] ==
                  "unresolved",
          "unknown cache does not become a known zero difference or a successful offset query");
    check(apply_native_parent_layer_rule(0, false, *prep.rule, unknown)["layer_id"] == 0 &&
              apply_native_parent_layer_rule(UINT32_MAX - 1, true, *prep.rule,
                                             unknown)["layer_id"] == UINT32_MAX - 1,
          "zero offset-mode input and protected style bypass unused unknown cache state");
    auto forced = prepare_native_block_parent_layer(header(62, 27, 3), definition, unknown);
    check(forced.rule && !forced.rule->numeric_offset &&
              apply_native_parent_layer_rule(10, false, *forced.rule, unknown)["layer_id"] == 27 &&
              apply_native_parent_layer_rule(10, true, *forced.rule, unknown)["layer_id"] == 10,
          "forced layer precedes enabled offset but protected style precedes all parent rules");
    check(apply_native_parent_layer_rule(10, false, *forced.rule, unknown)["protected_layer"] ==
                  false &&
              apply_native_parent_layer_rule(10, true, *forced.rule, unknown)["protected_layer"] ==
                  true,
          "force does not set offset protection while an existing protection bit remains set");
    NativeParentLayerRule plain{false, false, 20, {}};
    check(apply_native_parent_layer_rule(UINT32_MAX - 1, false, plain, unknown)["layer_id"] == 20 &&
              apply_native_parent_layer_rule(UINT32_MAX, false, plain, unknown)["layer_id"] ==
                  UINT32_MAX &&
              apply_native_parent_layer_rule(10, false, plain, unknown)["layer_id"] == 10,
          "nonoffset query replaces only max-minus-one and preserves other input bit patterns");
    NativeParentLayerRule offset{false, true, 20, 3};
    check(apply_native_parent_layer_rule(UINT32_MAX - 1, false, offset, index)["layer_id"] == 10,
          "offset mode looks up the inheritance-valued source before the final sentinel "
          "substitution");
    offset.numeric_offset = std::uint32_t(0u - 3u);
    check(apply_native_parent_layer_rule(10, false, offset, index)["layer_id"] == 20,
          "mapped max-minus-one uses the prepared parent layer after numeric lookup");
    NativeParentLayerRule current{true, true, 10, 3}, previous{true, true, 20, 7};
    auto merged = merge_native_parent_layer_rules(current, previous);
    check(merged.force_layer && merged.use_numeric_offset && merged.layer_id == 20 &&
              merged.numeric_offset == 7,
          "earlier combined ancestor wins force and offset fields without adding offsets");
    merged = merge_native_parent_layer_rules(current, NativeParentLayerRule{false, false, 30, {}});
    check(merged.layer_id == 10 && merged.numeric_offset == 3 && merged.force_layer,
          "ancestor fields without their enable flags do not replace current values");
    current.layer_id = UINT32_MAX - 1;
    current.force_layer = false;
    merged = merge_native_parent_layer_rules(current, NativeParentLayerRule{false, false, 30, {}});
    check(merged.layer_id == 30 && !merged.force_layer,
          "parent layer placeholder resolves before force-flag merging");
    previous.force_layer = false;
    previous.numeric_offset.reset();
    merged = merge_native_parent_layer_rules(current, previous);
    check(
        !merged.numeric_offset &&
            apply_native_parent_layer_rule(10, false, merged, index)["status"] == "unresolved",
        "enabled unknown ancestor offset replaces a known current offset without inventing a sum");
    auto forged = instance;
    forged["block_layer_inputs"] = {{"layer_id", 999}, {"force_layer", true}};
    check(prepare_native_block_parent_layer(forged, definition, index).rule->layer_id == 20,
          "preparation ignores forged decoded source annotations");
    check(!prepare_native_block_parent_layer(instance, header(62, 10), index).rule &&
              !prepare_native_block_parent_layer(header(32, 20), definition, index).rule,
          "instance and selected definition native types cannot be interchanged");
    forged = instance;
    forged["data"] = rawbytes(slice(bytes, 0, 112));
    check(!prepare_native_block_parent_layer(forged, definition, index).rule,
          "truncated override word cannot read into following storage");
    forged = definition;
    forged["data"] = rawbytes(slice(bytesof(definition["data"]), 0, 18));
    check(!prepare_native_block_parent_layer(instance, forged, index).rule,
          "truncated selected definition header remains unresolved");
    return checks;
}
