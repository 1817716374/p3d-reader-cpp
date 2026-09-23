#include "internal.hpp"
#include <p3d/proxy_cache.hpp>

unsigned proxy_cache_tests() {
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
    auto block = [&](const Bytes &body) {
        Bytes b;
        put(b, body.size(), 4);
        append(b, body);
        return b;
    };
    auto component = [&](const Bytes &groups, const Bytes &children,
                         const Bytes &optional = Bytes{}) {
        Bytes b;
        put(b, 0x12345678, 4);
        put(b, UINT64_MAX, 8);
        put(b, 81, 4);
        append(b, block(optional));
        append(b, block(groups));
        append(b, block(children));
        return block(b);
    };
    auto registry = [&](const std::vector<std::pair<std::uint64_t, Bytes>> &entries) {
        Bytes b;
        put(b, entries.size(), 4);
        for (const auto &[id, c] : entries) {
            put(b, id, 8);
            append(b, c);
        }
        return block(b);
    };
    auto group = [&](unsigned kind, unsigned flags, const Bytes &commands) {
        Bytes payload;
        put(payload, flags, 2);
        payload.resize(14);
        if (flags & 1)
            append(payload, block(Bytes{0, 0, 0, 0}));
        if (flags & 16)
            put(payload, 99, 4);
        append(payload, commands);
        Bytes b;
        put(b, kind, 4);
        append(b, block(payload));
        return b;
    };
    Bytes points{2, 0};
    put(points, 3, 2);
    put(points, 56, 4);
    put(points, 2, 4);
    for (double d : {1., 2., 3., 4., 5., 6.}) {
        std::uint64_t bits;
        std::memcpy(&bits, &d, 8);
        put(points, bits, 8);
    }
    const auto leaf = component({}, {});
    const auto nested =
        registry({{0, leaf}, {UINT64_MAX, leaf}, {0, component({}, {}, Bytes(96, 7))}});
    Bytes groups;
    for (unsigned i = 1; i <= 8; ++i)
        append(groups, group(i, i == 2 ? 17 : 0, points));
    append(groups, group(0, 0, {}));
    // Unknown group consumes only its header; a valid group follows immediately.
    put(groups, 9, 4);
    put(groups, 0xffffffff, 4);
    append(groups, group(6, 0, Bytes{9, 0, 1}));
    const auto data = registry({{7, component(groups, nested, Bytes{1, 2, 3})}});
    const auto result = decode_native_proxy_registry(data);
    check(result.at("status") == "decoded", "proxy registry decoded");
    check(result.at("semantics_status") == "partial", "source semantics not overstated");
    check(result.at("consumed_bytes") == data.size(), "complete registry consumption");
    const auto &root = result.at("registry");
    check(root.at("selected_entries")[0].at("id") == 7, "source ID retained");
    const auto &c = root.at("entries")[0].at("component");
    check(c.at("source_uint64") == UINT64_MAX, "full width source word retained");
    check(!c.at("optional_96_byte_block").at("native_retained").get<bool>(),
          "other sized block skipped");
    check(bytesof(c.at("optional_96_byte_block").at("data")) == Bytes({1, 2, 3}),
          "skipped source retained");
    const auto &g = c.at("graphics");
    check(g.size() == 11, "all group occurrences retained");
    for (unsigned i = 0; i < 8; ++i) {
        check(g[i].at("kind") == i + 1, "source group order retained");
        check(g[i].at("command_status") == "decoded", "command stream decoded");
        check(g[i].at("geometry").at("commands")[0].at("decoded").at("points")[1] ==
                  Json({4., 5., 6.}),
              "geometry points recovered");
        check(g[i].at("graphics_source_word") == (i == 1 ? 99 : 81),
              "group override and inherited value");
        check(g[i].at("native_class") ==
                  (i < 4 ? "ProxyGraphicsDisplaySegment" : "GraphicProxyContainer"),
              "native dispatch");
    }
    check(g[8].at("native_action") == "insert_null_graphic", "kind zero null retained");
    check(g[9].at("native_action") == "skip_header_only", "unknown kind native cursor rule");
    check(g[10].at("command_status") == "not_evaluated", "bad command format retained separately");
    check(c.at("native_graphics_order") == Json({8, 0, 1, 2, 3, 4, 5, 10, 6, 7}),
          "native group order retains equal-kind insertion order and excludes skipped headers");
    const auto &child = c.at("children");
    check(child.at("entries").size() == 3, "duplicate source entries retained");
    check(child.at("selected_entries").size() == 2, "native registry unique keys");
    check(child.at("selected_entries")[0].at("source_entry_index") == 2, "last duplicate selected");
    check(child.at("selected_entries")[1].at("id") == UINT64_MAX, "unsigned key order");
    check(
        child.at("entries")[2].at("component").at("optional_96_byte_block").at("native_retained") ==
            true,
        "96 byte block retained");
    for (std::size_t n = 0; n < data.size(); ++n)
        check(decode_native_proxy_registry(Bytes(data.begin(), data.begin() + n)).at("status") ==
                  "not_evaluated",
              "every truncation rejected");
    check(decode_native_proxy_registry(data, {1, 100}).at("reason") == "proxy_cache_depth_limit",
          "depth limit");
    check(decode_native_proxy_registry(data, {64, 1}).at("reason") == "proxy_cache_entry_limit",
          "entry limit");
    auto trailing = data;
    trailing.push_back(42);
    const auto extra = decode_native_proxy_registry(trailing);
    check(extra.at("status") == "decoded" && bytesof(extra.at("trailing_storage")) == Bytes{42},
          "outside bytes retained");
    check(decode_native_proxy_registry(registry({})).at("status") == "decoded", "empty registry");
    check(decode_native_proxy_registry(registry({}), {0, 100}).at("status") == "not_evaluated",
          "zero depth limit");
    return checks;
}
