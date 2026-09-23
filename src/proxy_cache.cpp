#include "internal.hpp"
#include <p3d/proxy_cache.hpp>

namespace p3d {
namespace {
struct ProxyReader {
    Reader reader;
    ProxyCacheLimits limits;
    std::size_t entries = 0;

    void need(std::size_t n, std::size_t end) {
        require(reader.p <= end && n <= end - reader.p, "truncated_proxy_cache_block");
    }
    std::uint32_t word(std::size_t end) {
        need(4, end);
        return reader.u32();
    }
    std::uint64_t id(std::size_t end) {
        need(8, end);
        return reader.u64();
    }
    Bytes take(std::size_t n, std::size_t end) {
        need(n, end);
        return reader.take(n);
    }
    std::size_t block_end(std::size_t end) {
        const auto size = word(end);
        need(size, end);
        return reader.p + size;
    }
    void count_entry() {
        require(entries < limits.max_entries, "proxy_cache_entry_limit");
        ++entries;
    }

    Json graphics(std::size_t end, std::uint32_t inherited_graphics_word) {
        Json groups = Json::array();
        while (reader.p < end) {
            count_entry();
            const auto start = reader.p;
            const auto kind = word(end), size = word(end);
            Json group = {{"source_offset", start}, {"kind", kind}, {"declared_bytes", size}};
            if (kind >= 9) {
                // The native constructor ignores this header WITHOUT advancing
                // by its length. Its following bytes become the next header.
                group.update({{"native_action", "skip_header_only"}, {"end_offset", reader.p}});
                groups.push_back(std::move(group));
                continue;
            }
            need(size, end);
            const auto payload_start = reader.p, payload_end = reader.p + size;
            need(14, payload_end);
            const auto flags = reader.u16();
            group["flags"] = flags;
            group["display_header"] = rawbytes(take(12, payload_end));
            group["line_style_modifiers"] = nullptr;
            if (flags & 1) {
                const auto modifier_size = word(payload_end);
                group["line_style_modifiers"] = rawbytes(take(modifier_size, payload_end));
            }
            group["graphics_source_word"] =
                flags & 0x10 ? word(payload_end) : inherited_graphics_word;
            group["graphics_word_source"] = flags & 0x10 ? "group_override" : "component";
            const auto command_offset = reader.p;
            const auto commands = take(payload_end - reader.p, payload_end);
            group["payload_offset"] = payload_start;
            group["command_offset"] = command_offset;
            group["command_storage"] = rawbytes(commands);
            if (kind == 0) {
                group["native_action"] = "insert_null_graphic";
            } else {
                require(commands.size() > 2, "proxy_graphic_requires_command_body");
                group["native_action"] = "insert_graphic";
                group["native_class"] =
                    kind <= 4 ? "ProxyGraphicsDisplaySegment" : "GraphicProxyContainer";
                try {
                    group["geometry"] = parse_commands(commands);
                    group["command_status"] = "decoded";
                } catch (const std::exception &e) {
                    // Object construction copies the stream even if the later
                    // geometry reader cannot interpret it. Preserve both states.
                    group["command_status"] = "not_evaluated";
                    group["command_reason"] = e.what();
                }
            }
            group["end_offset"] = reader.p;
            groups.push_back(std::move(group));
        }
        return groups;
    }

    Json component(std::size_t end, std::size_t depth) {
        const auto start = reader.p;
        const auto component_end = block_end(end);
        Json out = {{"source_offset", start}, {"declared_end_offset", component_end}};
        out["source_word"] = word(component_end);
        out["source_uint64"] = id(component_end);
        const auto graphics_word = word(component_end);
        out["graphics_source_word"] = graphics_word;
        const auto transform_size = word(component_end);
        out["optional_96_byte_block"] = {{"source_offset", reader.p},
                                         {"native_retained", transform_size == 96},
                                         {"data", rawbytes(take(transform_size, component_end))}};
        const auto graphic_end = block_end(component_end);
        out["graphics"] = graphics(graphic_end, graphics_word);
        std::multimap<std::uint32_t, std::size_t> order;
        for (std::size_t i = 0; i < out["graphics"].size(); ++i) {
            const auto kind = out["graphics"][i].at("kind").get<std::uint32_t>();
            if (kind < 9)
                order.emplace(kind, i);
        }
        out["native_graphics_order"] = Json::array();
        for (const auto &[kind, index] : order)
            out["native_graphics_order"].push_back(index);
        const auto children_end = block_end(component_end);
        require(children_end == component_end, "proxy_children_extent_mismatch");
        out["children"] = reader.p == children_end ? Json() : registry(children_end, depth + 1);
        // Native recursion returns its consumed cursor, not the outer length.
        out["end_offset"] = reader.p;
        return out;
    }

    Json registry(std::size_t end, std::size_t depth) {
        require(depth < limits.max_depth, "proxy_cache_depth_limit");
        const auto start = reader.p;
        const auto registry_end = block_end(end);
        const auto count = word(registry_end);
        require(count <= limits.max_entries - entries, "proxy_cache_entry_limit");
        Json out = {{"source_offset", start},
                    {"declared_end_offset", registry_end},
                    {"entries", Json::array()}};
        std::map<std::uint64_t, std::size_t> selected;
        for (std::uint32_t i = 0; i < count; ++i) {
            count_entry();
            const auto key = id(registry_end);
            auto value = component(registry_end, depth);
            out["entries"].push_back({{"id", key}, {"component", std::move(value)}});
            selected[key] = i;
        }
        out["selected_entries"] = Json::array();
        for (const auto &[key, index] : selected)
            out["selected_entries"].push_back({{"id", key}, {"source_entry_index", index}});
        out["end_offset"] = reader.p;
        return out;
    }
};
} // namespace

Json decode_native_proxy_registry(const Bytes &bytes, ProxyCacheLimits limits) {
    Json out = {{"status", "not_evaluated"},
                {"scope", "decompressed_native_proxy_registry"},
                {"semantics_status", "partial"},
                {"remaining_semantics",
                 {"component_source_fields", "optional_96_byte_block", "display_header_and_flags",
                  "line_style_modifiers", "model_cache_context_and_final_display"}}};
    try {
        ProxyReader parser{Reader(bytes), limits};
        auto registry = parser.registry(bytes.size(), 0);
        out.update({{"status", "decoded"},
                    {"registry", std::move(registry)},
                    {"consumed_bytes", parser.reader.p},
                    {"trailing_storage", rawbytes(parser.reader.take(parser.reader.left()))}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
