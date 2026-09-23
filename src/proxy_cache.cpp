#include "internal.hpp"
#include "proxy_cache_fields.hpp"
#include <lz4/lz4.h>
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

    Json source_string(std::size_t end) {
        const auto start = reader.p;
        const auto units = word(end);
        require(units <= 2048, "edge_cache_string_length_limit");
        const auto bytes = take(std::size_t(units) * 2, end);
        std::size_t n = 0;
        while (n + 1 < bytes.size() && (bytes[n] || bytes[n + 1]))
            n += 2;
        // Native construction scans for NUL independently of the length. Do
        // not reproduce its unbounded read for malformed string fields.
        require(bytes.empty() || n + 1 < bytes.size(), "unterminated_edge_cache_string");
        return {{"source_offset", start},
                {"storage", rawbytes(bytes)},
                {"text", utf16(slice(bytes, 0, n))}};
    }

    Json model(std::size_t depth) {
        const auto end = block_end(reader.b.size());
        const auto header_end = block_end(end);
        const auto version = word(end);
        require(version == 51, "unsupported_edge_cache_model_version");
        Json out = {{"declared_end_offset", end},
                    {"declared_header_end_offset", header_end},
                    {"version", version},
                    {"source_word", word(end)},
                    {"source_link_id", id(end)}};
        out["source_block_96"] = rawbytes(take(96, end));
        out["source_strings"] = Json::array({source_string(end), source_string(end)});
        const auto reference_start = reader.p;
        const auto reference_bytes = take(328, end);
        out["reference_parameters_storage"] = rawbytes(reference_bytes);
        out["reference_parameters"] = cached_reference_parameters(reference_bytes);
        out["reference_parameters"]["model_source_offset"] = reference_start;
        out["source_block_32"] = rawbytes(take(32, end));
        out["child_model_count"] = word(end);
        const auto optional_size = word(end);
        const auto metadata_start = reader.p;
        const auto metadata_bytes = take(optional_size, end);
        out["optional_object_storage"] = rawbytes(metadata_bytes);
        out["model_metadata"] = cached_model_metadata(metadata_bytes);
        out["model_metadata"]["model_source_offset"] = metadata_start;

        // The native reader validates the header/table extents, but keeps the
        // consumed cursor: neither extent implies a seek past unread bytes.
        const auto table_end = block_end(end);
        const auto count = word(table_end);
        require(count <= limits.max_entries - entries, "proxy_cache_entry_limit");
        out["string_table"] = {{"declared_end_offset", table_end}, {"entries", Json::array()}};
        auto &table = out["string_table"];
        std::map<std::uint32_t, std::size_t> selected_strings;
        for (std::uint32_t i = 0; i < count; ++i) {
            count_entry();
            const auto key = word(table_end);
            table["entries"].push_back({{"key", key}, {"value", source_string(table_end)}});
            selected_strings[key] = i;
        }
        table["selected_entries"] = Json::array();
        for (const auto &[key, index] : selected_strings)
            table["selected_entries"].push_back({{"key", key}, {"source_entry_index", index}});
        table["end_offset"] = reader.p;
        using Key = std::pair<std::int32_t, std::uint32_t>;
        const auto less = [](const Key &a, const Key &b) {
            return a.first != b.first ? a.first < b.first : a.first != 0 && a.second < b.second;
        };
        std::map<Key, std::pair<std::size_t, std::size_t>, decltype(less)> selected(less);
        out["sections"] = Json::array();
        while (reader.p < end) {
            count_entry();
            const auto start = reader.p;
            const auto section_end = block_end(end);
            need(4, section_end);
            const auto first = reader.i32();
            const auto second = word(section_end);
            auto value = registry(section_end, depth);
            const auto index = out["sections"].size();
            auto [it, inserted] =
                selected.emplace(Key{first, second}, std::make_pair(index, index));
            it->second.second = index;
            out["sections"].push_back({{"source_offset", start},
                                       {"declared_end_offset", section_end},
                                       {"first_key", first},
                                       {"second_key", second},
                                       {"registry", std::move(value)},
                                       {"end_offset", reader.p}});
        }
        out["selected_sections"] = Json::array();
        for (const auto &[key, indices] : selected)
            out["selected_sections"].push_back({{"first_key", key.first},
                                                {"second_key", key.second},
                                                {"key_source_section_index", indices.first},
                                                {"value_source_section_index", indices.second}});
        out["consumed_bytes"] = reader.p;
        out["trailing_storage"] = rawbytes(reader.take(reader.left()));
        return out;
    }
};

struct EdgeCacheReader {
    const Json &attributes;
    ModelEdgeCacheLimits limits;
    std::map<std::uint32_t, std::size_t> sources;
    std::set<std::size_t> used;
    std::size_t entries = 0, decoded_bytes = 0;
    std::uint32_t next_index = 1;
    Json models = Json::array();

    EdgeCacheReader(const Json &input, ModelEdgeCacheLimits bounds)
        : attributes(input), limits(bounds) {}

    const Json *attribute(std::uint32_t index) {
        const auto it = sources.find(index);
        if (it == sources.end())
            return nullptr;
        used.insert(it->second);
        return &attributes.at(it->second);
    }

    Bytes decompress(const Bytes &input, Json &metadata) {
        Reader r(input);
        const auto codec = r.u16(), flags = r.u16();
        const auto size = r.u32();
        require(size <= limits.max_model_bytes &&
                    size <= limits.max_total_model_bytes - decoded_bytes,
                "edge_cache_decompressed_byte_limit");
        metadata = {{"codec_id", codec}, {"source_flags", flags}, {"decoded_bytes", size}};
        Bytes bytes;
        if (codec == 1 || codec == 2) {
            bytes = r.take(size);
            metadata["codec"] = "stored";
            metadata["ignored_suffix"] = rawbytes(r.take(r.left()));
        } else {
            require(codec == 3, "unsupported_edge_cache_codec");
            require(size <= INT32_MAX && r.left() <= INT32_MAX, "edge_cache_codec_size_limit");
            bytes.resize(size);
            const auto got =
                LZ4_decompress_safe(reinterpret_cast<const char *>(input.data() + r.p),
                                    reinterpret_cast<char *>(bytes.data()),
                                    static_cast<int>(r.left()), static_cast<int>(size));
            require(got >= 0 && static_cast<std::uint32_t>(got) == size,
                    "edge_cache_lz4_size_or_framing_mismatch");
            metadata["codec"] = "lz4";
        }
        decoded_bytes += size;
        return bytes;
    }

    std::size_t model(std::size_t depth, Json parent) {
        require(depth < limits.proxies.max_depth, "edge_cache_model_depth_limit");
        require(models.size() < limits.max_models, "edge_cache_model_count_limit");
        const auto source_index = next_index++;
        const auto *a = attribute(source_index);
        require(a != nullptr, "missing_edge_cache_model_attribute_" + std::to_string(source_index));
        Json envelope;
        const auto bytes = decompress(bytesof(a->at("payload")), envelope);
        ProxyReader reader{Reader(bytes), limits.proxies, entries};
        auto value = reader.model(depth);
        entries = reader.entries;
        const auto children = value.at("child_model_count").get<std::uint32_t>();
        require(children <= limits.max_models - models.size() - 1, "edge_cache_model_count_limit");
        const auto index = models.size();
        value.update({{"attribute_index", source_index},
                      {"source_ordinal", sources.at(source_index)},
                      {"envelope", std::move(envelope)},
                      {"parent_model_index", std::move(parent)},
                      {"children", Json::array()},
                      {"runtime_target_status", "not_evaluated"}});
        models.push_back(std::move(value));
        for (std::uint32_t i = 0; i < children; ++i) {
            const auto child = model(depth + 1, index);
            models[index]["children"].push_back(child);
        }
        return index;
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

Json decode_native_model_edge_cache(const Json &attributes, ModelEdgeCacheLimits limits) {
    Json out = {
        {"status", "not_evaluated"},
        {"scope", "native_model_edge_cache"},
        {"semantics_status", "partial"},
        {"runtime_attachment_status", "not_evaluated"},
        {"remaining_semantics",
         {"cache_header_fields", "model_source_fields", "model_metadata_remaining_fields",
          "proxy_display_parameters", "runtime_targets_and_final_display", "index_65535_payload"}}};
    try {
        require(attributes.is_array(), "edge_cache_attributes_must_be_array");
        const auto lookup = native_attribute_lookup(attributes);
        require(lookup.at("status") == "resolved", "edge_cache_attribute_lookup_failed");
        EdgeCacheReader parser{attributes, limits};
        for (const auto &key : lookup.at("keys"))
            if (key.at("group") == 21 && key.at("key") == 22762)
                parser.sources.emplace(key.at("index").get<std::uint32_t>(),
                                       key.at("selected_source_ordinal").get<std::size_t>());
        out["attribute_lookup"] = lookup;
        const auto *header_source = parser.attribute(0);
        if (!header_source) {
            out.update({{"status", "absent"}, {"reason", "cache_header_attribute_not_found"}});
            return out;
        }
        const auto bytes = bytesof(header_source->at("payload"));
        ProxyReader reader{Reader(bytes), limits.proxies};
        const auto version = reader.word(bytes.size()), source_word = reader.word(bytes.size());
        Json header = {{"source_ordinal", parser.sources.at(0)},
                       {"version", version},
                       {"source_word", source_word}};
        if (version < 15) {
            out.update({{"status", "ignored"},
                        {"reason", "cache_version_below_15"},
                        {"header", std::move(header)}});
            return out;
        }
        header["source_block_32"] = rawbytes(reader.take(32, bytes.size()));
        header["source_uint64_at_40"] = reader.id(bytes.size());
        header["source_uint64_at_48"] = reader.id(bytes.size());
        header["display_parameters_storage"] = rawbytes(reader.take(80, bytes.size()));
        header["source_word_at_136"] = reader.word(bytes.size());
        header["source_uint64_at_140"] = reader.id(bytes.size());
        header["source_bits_at_148"] = reader.id(bytes.size());
        header["native_version_mismatch"] = version != 51;
        header["native_secondary_mismatch"] = version == 51 && source_word != 1;
        if (version == 51) {
            header["skipped_sections"] = Json::array();
            for (unsigned i = 0; i < 4; ++i) {
                const auto end = reader.block_end(bytes.size());
                header["skipped_sections"].push_back(
                    rawbytes(reader.take(end - reader.reader.p, end)));
            }
        }
        header["consumed_bytes"] = reader.reader.p;
        header["trailing_storage"] = rawbytes(reader.reader.take(reader.reader.left()));
        out["header"] = std::move(header);
        if (version == 51) {
            parser.model(0, nullptr);
            out["models"] = std::move(parser.models);
            out["root_model_index"] = 0;
            out["model_read_status"] = "decoded";
        } else {
            out["models"] = Json::array();
            out["root_model_index"] = nullptr;
            out["model_read_status"] = "skipped_version_mismatch";
        }
        // This is a separate native display-parameter lookup, not a substitute
        // for the 80 bytes stored in index 0. Missing/wrong length uses the host.
        Json display = {{"status", "host_fallback_required"}};
        if (const auto *a = parser.attribute(65534)) {
            const auto data = bytesof(a->at("payload"));
            display["source_ordinal"] = parser.sources.at(65534);
            display["storage"] = rawbytes(data);
            if (data.size() == 80)
                display["status"] = "source_available";
        }
        out["display_parameter_lookup"] = std::move(display);
        out["next_model_attribute_index"] = parser.next_index;
        out["total_decoded_model_bytes"] = parser.decoded_bytes;
        out["unconsumed_cache_source_ordinals"] = Json::array();
        for (std::size_t i = 0; i < attributes.size(); ++i)
            if (attributes[i].at("group") == 21 && attributes[i].at("key") == 22762 &&
                parser.used.count(i) == 0)
                out["unconsumed_cache_source_ordinals"].push_back(i);
        out["status"] = "decoded";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
