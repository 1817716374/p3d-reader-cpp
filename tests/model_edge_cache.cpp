#include "internal.hpp"
#include <lz4/lz4.h>
#include <p3d/proxy_cache.hpp>

unsigned model_edge_cache_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto put = [](Bytes &b, std::uint64_t n, unsigned width) {
        for (unsigned i = 0; i < width; ++i)
            b.push_back(std::uint8_t(n >> (8 * i)));
    };
    auto append = [](Bytes &a, const Bytes &b) { a.insert(a.end(), b.begin(), b.end()); };
    auto block = [&](const Bytes &b) {
        Bytes out;
        put(out, b.size(), 4);
        append(out, b);
        return out;
    };
    auto string = [&](const Bytes &b) {
        Bytes out;
        put(out, b.size() / 2, 4);
        append(out, b);
        return out;
    };
    auto header = [&](std::uint32_t version = 51) {
        Bytes out;
        put(out, version, 4);
        put(out, 1, 4);
        out.resize(156);
        if (version == 51)
            for (unsigned i = 0; i < 4; ++i)
                append(out, block(Bytes(i, std::uint8_t(i))));
        return out;
    };
    auto envelope = [&](const Bytes &b, unsigned codec = 1) {
        Bytes out;
        put(out, codec, 2);
        put(out, 0x1234, 2); // Codec dispatch uses the low 16 bits only.
        put(out, b.size(), 4);
        if (codec != 3) {
            append(out, b);
        } else {
            Bytes compressed(LZ4_compressBound(static_cast<int>(b.size())));
            const auto size = LZ4_compress_default(reinterpret_cast<const char *>(b.data()),
                                                   reinterpret_cast<char *>(compressed.data()),
                                                   static_cast<int>(b.size()),
                                                   static_cast<int>(compressed.size()));
            require(size > 0, "fixture compression");
            compressed.resize(size);
            append(out, compressed);
        }
        return out;
    };
    auto attribute = [](unsigned index, const Bytes &b) {
        return Json{{"group", 21}, {"key", 22762}, {"index", index}, {"payload", rawbytes(b)}};
    };
    auto model = [&](unsigned children, bool with_sections = false) {
        Bytes h;
        put(h, 51, 4);
        put(h, 0x87654321, 4);
        put(h, UINT64_MAX, 8);
        h.resize(h.size() + 96, 7);
        append(h, string(Bytes{'A', 0, 0, 0}));
        append(h, string({}));
        h.resize(h.size() + 328 + 32, 9);
        put(h, children, 4);
        append(h, block({}));
        auto b = block(h);
        Bytes table;
        put(table, 3, 4);
        for (unsigned key : {UINT32_MAX, 0u, UINT32_MAX}) {
            put(table, key, 4);
            append(table, string(Bytes{std::uint8_t(table.size()), 0, 0, 0}));
        }
        append(b, block(table));
        if (with_sections) {
            for (const auto &[first, second] : std::vector<std::pair<std::int32_t, std::uint32_t>>{
                     {0, 99}, {-1, UINT32_MAX}, {0, 88}, {1, UINT32_MAX}, {1, 0}}) {
                Bytes section;
                put(section, std::uint32_t(first), 4);
                put(section, second, 4);
                // One actual nested component, containing a point-list command.
                Bytes commands{2, 0};
                put(commands, 3, 2);
                put(commands, 32, 4);
                put(commands, 1, 4);
                for (double value : {10., 20., 30.}) {
                    std::uint64_t bits;
                    std::memcpy(&bits, &value, 8);
                    put(commands, bits, 8);
                }
                Bytes graphic(14);
                append(graphic, commands);
                Bytes group;
                put(group, 1, 4);
                append(group, block(graphic));
                Bytes component(16);
                append(component, block({}));
                append(component, block(group));
                append(component, block({}));
                Bytes registry;
                put(registry, 1, 4);
                put(registry, 42, 8);
                append(registry, block(component));
                append(section, block(registry));
                append(b, block(section));
            }
        }
        return block(b);
    };
    const auto root = model(2, true), branch = model(1), leaf = model(0);
    Json input = Json::array({attribute(65535, Bytes{1, 2}), attribute(3, envelope(leaf)),
                              attribute(0, header()), attribute(1, envelope(root, 3)),
                              attribute(4, envelope(leaf, 2)), attribute(2, envelope(branch)),
                              attribute(65534, Bytes(80, 8)), attribute(99, Bytes{1})});
    const auto decoded = decode_native_model_edge_cache(input);
    check(decoded.at("status") == "decoded", "full cache structure decoded");
    check(decoded.at("semantics_status") == "partial", "cache semantics not overstated");
    check(decoded.at("runtime_attachment_status") == "not_evaluated",
          "no invented runtime targets");
    check(decoded.at("models").size() == 4, "all preorder model attributes read");
    check(decoded.at("next_model_attribute_index") == 5, "counter follows recursion");
    const auto &models = decoded.at("models");
    check(models[0].at("children") == Json({1, 3}) && models[1].at("children") == Json({2}),
          "child counts reconstruct hierarchy, not numeric ID sorting");
    check(models[0].at("parent_model_index").is_null() && models[2].at("parent_model_index") == 1,
          "parent model index retained");
    check(models[0].at("source_link_id") == UINT64_MAX &&
              models[1].at("source_link_id") == UINT64_MAX,
          "repeated full width link IDs do not merge models");
    check(models[0].at("source_strings")[0].at("text") == "A", "model UTF16 source string");
    check(models[0].at("string_table").at("selected_entries")[1].at("source_entry_index") == 2,
          "string table last duplicate wins");
    const auto &selected = models[0].at("selected_sections");
    check(selected.size() == 4 && selected[0].at("first_key") == -1,
          "signed first section key and zero-key equivalence");
    check(selected[1].at("second_key") == 99 && selected[1].at("key_source_section_index") == 0 &&
              selected[1].at("value_source_section_index") == 2,
          "zero first key keeps first key storage but last value");
    check(selected[2].at("second_key") == 0 && selected[3].at("second_key") == UINT32_MAX,
          "unsigned second section key");
    const auto &registry = models[0].at("sections")[0].at("registry");
    check(registry.at("entries")[0]
                  .at("component")
                  .at("graphics")[0]
                  .at("geometry")
                  .at("commands")[0]
                  .at("decoded")
                  .at("points")[0] == Json({10., 20., 30.}),
          "outer attribute connects to proxy geometry");
    check(decoded.at("unconsumed_cache_source_ordinals") == Json({0, 7}),
          "special payload and unused source preserved by occurrence");
    check(decoded.at("display_parameter_lookup").at("status") == "source_available",
          "80-byte separate display parameters accepted");
    check(models[0].at("envelope").at("source_flags") == 0x1234 &&
              models[0].at("envelope").at("codec") == "lz4",
          "native codec header");

    const auto single = [&](const Bytes &body) {
        return Json::array({attribute(0, header()), attribute(1, envelope(body))});
    };
    for (std::size_t n = 0; n < leaf.size(); ++n)
        check(decode_native_model_edge_cache(single(Bytes(leaf.begin(), leaf.begin() + n)))
                      .at("status") == "not_evaluated",
              "every model truncation rejected");
    const auto h = header();
    for (std::size_t n = 0; n < h.size(); ++n)
        check(decode_native_model_edge_cache(
                  Json::array({attribute(0, Bytes(h.begin(), h.begin() + n)),
                               attribute(1, envelope(leaf))}))
                      .at("status") == "not_evaluated",
              "every main header truncation rejected");
    auto broken = input;
    broken.erase(1); // Missing index 3; index 4 must not be substituted.
    const auto missing = decode_native_model_edge_cache(broken);
    check(missing.at("status") == "not_evaluated" && !missing.contains("models") &&
              missing.at("reason") == "missing_edge_cache_model_attribute_3",
          "missing child aborts without presenting a partial tree as complete");
    ModelEdgeCacheLimits limits;
    limits.max_models = 3;
    check(decode_native_model_edge_cache(input, limits).at("reason") ==
              "edge_cache_model_count_limit",
          "global model budget");
    limits = {};
    limits.proxies.max_depth = 2;
    check(decode_native_model_edge_cache(input, limits).at("reason") ==
              "edge_cache_model_depth_limit",
          "model recursion depth budget");
    limits = {};
    limits.max_total_model_bytes = root.size() + branch.size();
    check(decode_native_model_edge_cache(input, limits).at("reason") ==
              "edge_cache_decompressed_byte_limit",
          "global decompression budget");
    limits = {};
    limits.proxies.max_entries = 18; // Root consumes 3 strings + 5*(section, entity, graphic).
    check(decode_native_model_edge_cache(input, limits).at("reason") == "proxy_cache_entry_limit",
          "proxy budget is shared across models");
    for (unsigned version : {15u, 50u, 52u}) {
        const auto result =
            decode_native_model_edge_cache(Json::array({attribute(0, header(version))}));
        check(result.at("status") == "decoded" && result.at("models").empty() &&
                  result.at("model_read_status") == "skipped_version_mismatch",
              "other accepted versions return header without reading model attributes");
    }
    check(decode_native_model_edge_cache(Json::array({attribute(0, header(14))})).at("status") ==
              "ignored",
          "old version ignored");
    check(decode_native_model_edge_cache(Json::array()).at("status") == "absent", "absent cache");
    auto duplicate = single(leaf);
    duplicate.push_back(attribute(1, envelope(leaf, 2)));
    auto result = decode_native_model_edge_cache(duplicate);
    check(result.at("models")[0].at("source_ordinal") == 2, "small collection reverse lookup");
    for (unsigned i = 0; i < 3; ++i)
        duplicate.push_back({{"group", 0}, {"key", i}, {"index", 0}, {"payload", rawbytes({})}});
    result = decode_native_model_edge_cache(duplicate);
    check(result.at("models")[0].at("source_ordinal") == 1,
          "complete collection changes native duplicate lookup; do not prefilter");
    auto stored = envelope(leaf);
    stored.push_back(77);
    result =
        decode_native_model_edge_cache(Json::array({attribute(0, header()), attribute(1, stored)}));
    check(bytesof(result.at("models")[0].at("envelope").at("ignored_suffix")) == Bytes{77},
          "stored envelope native length consumption");
    auto trailing = leaf;
    trailing.push_back(88);
    result = decode_native_model_edge_cache(single(trailing));
    check(bytesof(result.at("models")[0].at("trailing_storage")) == Bytes{88},
          "model suffix preserved");
    auto wrong_codec = single(leaf);
    wrong_codec[1] = attribute(1, envelope(leaf, 4));
    check(decode_native_model_edge_cache(wrong_codec).at("reason") ==
              "unsupported_edge_cache_codec",
          "no guessed codec fallback");
    auto compressed = envelope(leaf, 3);
    compressed.pop_back();
    check(decode_native_model_edge_cache(
              Json::array({attribute(0, header()), attribute(1, compressed)}))
                  .at("reason") == "edge_cache_lz4_size_or_framing_mismatch",
          "truncated LZ4 rejected");
    auto bad_string = leaf;
    // Outer length + header length + version/word/link/96-byte block = 120.
    bad_string[120] = 1;
    bad_string[124] = 'A';
    bad_string[125] = 0;
    check(decode_native_model_edge_cache(single(bad_string)).at("reason") ==
              "unterminated_edge_cache_string",
          "unterminated bounded source string rejected");
    auto display = single(leaf);
    display.push_back(attribute(65534, Bytes(79)));
    check(decode_native_model_edge_cache(display).at("display_parameter_lookup").at("status") ==
              "host_fallback_required",
          "wrong-length display field needs actual host fallback");
    return checks;
}
