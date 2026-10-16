#include "internal.hpp"
#include <p3d/block_symbols.hpp>
using namespace p3d;
unsigned block_symbols_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        require(ok, why);
        ++checks;
    };
    auto set = [](Bytes &b, std::size_t at, auto value) {
        std::memcpy(b.data() + at, &value, sizeof(value));
    };
    Bytes table(68, 0);
    set(table, 4, std::uint16_t(10));
    set(table, 8, std::uint32_t(32));
    set(table, 12, std::uint32_t(32));
    set(table, 16, std::uint32_t(1));
    set(table, 36, std::uint32_t(2));
    for (unsigned style : {17u, 9u}) {
        Bytes b(108, 0);
        set(b, 4, std::uint16_t(49));
        set(b, 6, std::uint16_t(0x80));
        set(b, 8, std::uint32_t(52));
        set(b, 12, std::uint32_t(52));
        set(b, 16, std::uint32_t(1));
        set(b, 36, std::uint32_t(64));
        set(b, 44, std::uint16_t(7));
        set(b, 68, std::uint32_t(style));
        set(b, 72, std::uint32_t(13));
        set(b, 76, std::uint32_t(0x22));
        set(b, 80, std::uint32_t(0x122));
        table.insert(table.end(), b.begin(), b.end());
    }
    auto records = parse_native(table);
    for (auto &r : records)
        r["stream"] = {"system", "layers"};
    const NativeLayerNumericIndex index(records, 0), unknown(Json(), 0);
    auto block = [&](std::uint32_t color, std::uint32_t weight, std::uint32_t style,
                     unsigned mask = 0, std::uint32_t layer = 64) {
        Bytes b(116, 0);
        set(b, 4, std::uint16_t(62));
        set(b, 6, std::uint16_t(0x20));
        set(b, 8, std::uint32_t(56));
        set(b, 12, std::uint32_t(56));
        set(b, 16, layer);
        set(b, 48, style);
        set(b, 52, weight);
        set(b, 56, color);
        set(b, 112, std::uint16_t(mask));
        return Json{{"data", rawbytes(b)}, {"links", Json::array()}, {"offset", 700}};
    };
    auto parameter = [&](Json &record, std::uint32_t flags, double scale) {
        Bytes p(12, 0);
        set(p, 0, flags);
        set(p, 4, scale);
        record["links"] = Json::array(
            {{{"header", 0x1007}, {"app", 0x79f9}, {"offset", 116}, {"payload", rawbytes(p)}}});
    };
    auto prepare = [&](const Json &r) {
        return prepare_native_block_parent_symbols(r, records, index);
    };
    auto source = block(UINT32_MAX, UINT32_MAX, 0x7fffffff, 0xffff);
    const auto saved = source;
    auto p = prepare(source);
    check(p["status"] == "decoded" && p["values"]["color"]["value"] == 0x122 &&
              p["values"]["line_weight"]["value"] == 13 &&
              p["values"]["line_style"]["value"] == 9 &&
              p["values"]["color"]["native_record_index"] == 2 &&
              p["symbol_override_mask"] == 0x38 && source == saved,
          "block by-layer values use the selected last loaded source and native extended color");
    check(p["line_style_parameters"]["state"]["scalars"]["scale"] == 1. &&
              p["line_style_parameters"]["selection"] == "defaults_after_graphic_reader_failure",
          "missing instance parameter linkage creates the native defaults independently of layer "
          "slots");
    p = prepare(block(UINT32_MAX, UINT32_MAX, 0x7fffffff, 0, 99));
    check(p["status"] == "decoded" && p["values"]["color"]["value"] == UINT32_MAX &&
              p["values"]["line_style"]["value"] == 0x7fffffff &&
              p["values"]["color"]["selection"] == "missing_layer_preserves_source",
          "known missing layer preserves the source sentinels rather than using another layer");
    p = prepare_native_block_parent_symbols(source, records, unknown);
    check(p["status"] == "partial" && p["values"]["color"]["status"] == "unresolved" &&
              !p["values"]["color"].contains("value"),
          "unknown cache is not a confirmed miss and does not publish a guessed by-layer value");
    p = prepare_native_block_parent_symbols(block(3, 4, 9), records, unknown);
    check(p["status"] == "decoded" && p["values"]["color"]["value"] == 3,
          "explicit values do not require an unused layer cache");
    auto forged = source;
    forged["block_layer_inputs"] = {{"source_symbology", {{"color", 999}}}};
    check(prepare(forged)["values"] == prepare(source)["values"],
          "prepared block symbols use raw input instead of decoded annotations");
    auto wrong = records;
    wrong[2]["data"] = rawbytes(Bytes(108, 0));
    check(prepare_native_block_parent_symbols(source, wrong, index)["status"] == "partial",
          "mismatched selected layer source cannot be silently substituted");
    auto current_record = block(UINT32_MAX - 1, UINT32_MAX - 1, 0x7ffffffe);
    auto previous_record = block(7, 8, 9);
    parameter(current_record, 1, 2.);
    parameter(previous_record, 1, 5.);
    auto current = prepare(current_record), previous = prepare(previous_record);
    auto merged = merge_native_parent_symbols(current, previous);
    check(merged["status"] == "decoded" && merged["values"] == previous["values"] &&
              merged["line_style_parameters"]["state"]["scalars"]["scale"] == 2. &&
              merged["inherited_fields"].size() == 3,
          "by-parent scalar inheritance does not also copy the previous modifier state");
    previous["symbol_override_mask"] = 32;
    merged = merge_native_parent_symbols(current, previous);
    check(merged["line_style_parameters"] == previous["line_style_parameters"] &&
              merged["symbol_override_mask"] == 32 &&
              merged["overridden_fields"] == Json({"line_style"}),
          "previous style override copies all modifiers together with the style number");
    current = prepare(block(1, 2, 3, 8));
    previous["symbol_override_mask"] = 0x38;
    merged = merge_native_parent_symbols(current, previous);
    check(merged["values"] == previous["values"] && merged["symbol_override_mask"] == 0x38 &&
              merged["overridden_fields"].size() == 3,
          "enabled previous combined ancestor overrides explicit current symbol components");
    previous["symbol_override_mask"] = 0;
    check(merge_native_parent_symbols(current, previous)["values"] == current["values"],
          "current override mask does not reverse the previous-entry precedence");
    auto bad = previous_record;
    auto payload = bytesof(bad["links"][0]["payload"]);
    payload.resize(8);
    bad["links"][0]["header"] = 0x1005;
    bad["links"][0]["payload"] = rawbytes(payload);
    auto incomplete = prepare(bad);
    check(incomplete["status"] == "partial" &&
              !incomplete["line_style_parameters"].contains("state"),
          "unsafe modifier read is not replaced with a successful defaults state");
    previous["symbol_override_mask"] = 32;
    check(merge_native_parent_symbols(incomplete, previous)["status"] == "decoded",
          "known overriding ancestor can replace an unused unknown current modifier state");
    previous["symbol_override_mask"] = 0;
    check(merge_native_parent_symbols(incomplete, previous)["status"] == "partial",
          "unknown current modifier remains unknown when the previous entry does not override it");
    previous["symbol_override_mask"] = 32;
    check(merge_native_parent_symbols(current, incomplete)["status"] == "decoded",
          "unknown previous modifiers are irrelevant without their override bit");
    incomplete["symbol_override_mask"] = 32;
    check(merge_native_parent_symbols(current, incomplete)["status"] == "partial",
          "enabled unknown ancestor modifiers prevent publishing a complete result");
    auto reset = block(1, 2, 9);
    parameter(reset, 0x401, 123.);
    p = prepare(reset);
    check(p["status"] == "decoded" && p["line_style_parameters"]["state"]["flags"] == 0 &&
              p["line_style_parameters"]["state"]["scalars"]["scale"] == 1.,
          "native block fallback after a rejected parameter read uses complete fresh defaults");
    auto truncated = source;
    truncated["data"] = rawbytes(Bytes(112, 0));
    check(prepare(truncated)["status"] == "unresolved" &&
              merge_native_parent_symbols(current, Json::object())["status"] == "unresolved",
          "missing source or previous-entry fields do not become a valid parent state");
    return checks;
}
