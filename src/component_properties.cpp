#include "component_properties.hpp"

namespace p3d {
namespace {
constexpr std::uint64_t none_id = 0x4500450011720048ULL;
constexpr std::uint64_t bool_id = 0x1580142273520992ULL;
constexpr std::uint64_t double_id = 0x7175318778202422ULL;
constexpr std::uint64_t int_id = 0x4569571470222419ULL;
constexpr std::uint64_t uint_id = 0x2477702224190092ULL;
constexpr std::uint64_t string_id = 0x1316456973520092ULL;
constexpr std::uint64_t bytes_id = 0x0059665104550025ULL;
constexpr std::uint64_t vector_id = 0x3131099204415903ULL;
constexpr std::uint64_t map_id = 0x3131099213160368ULL;
constexpr std::uint64_t files_id = 0x0181635124290115ULL;
constexpr std::uint64_t function_id = 0x4827000104282422ULL;
constexpr std::uint64_t property_id = 0x0141762724222207ULL;
constexpr std::uint64_t noumenon_id = 0x0260975554222207ULL;

struct StackDecoder {
    const Bytes &source;
    std::size_t visits = 0;
    bool complete = true;

    void budget(unsigned depth) {
        require(depth <= 80, "component property nesting limit");
        require(++visits <= 1000000, "component property node limit");
    }
    Json text(std::size_t start, std::size_t size) const {
        const auto bytes = slice(source, start, size);
        Json out = {{"bytes_base64", base64(bytes)}, {"bytes", size}};
        if (std::all_of(bytes.begin(), bytes.end(), [](unsigned char c) { return c < 128; })) {
            out["text"] = std::string(bytes.begin(), bytes.end());
            out["encoding"] = "ascii";
        } else
            out["encoding"] = "unspecified_narrow_string";
        return out;
    }
    Json read(std::size_t begin, std::size_t &end, unsigned depth, std::uint64_t expected = 0) {
        budget(depth);
        require(end >= begin && end - begin >= 16, "component property frame is truncated");
        Reader header(source, end - 16);
        const auto size = header.u64(), id = header.u64();
        require(size <= end - begin - 16, "component property payload exceeds its frame");
        const auto start = end - 16 - std::size_t(size);
        Json out = {{"type_id", id}, {"offset", start}, {"bytes", size}, {"status", "decoded"}};
        const auto payload_end = end - 16;
        end = start;
        try {
            require(!expected || expected == id, "component property type mismatch");
            std::size_t cursor = payload_end;
            Reader raw(source, start);
            auto child = [&](std::uint64_t type = 0) {
                auto v = read(start, cursor, depth + 1, type);
                require(!type || v.at("status") == "decoded", "invalid typed component member");
                return v;
            };
            auto count = [&]() {
                auto v = child(uint_id);
                const auto n = v.at("value").get<std::uint64_t>();
                require(n <= (cursor - start) / 16 && n <= 1000000 - visits,
                        "component property count exceeds input or node budget");
                out["count_source_offset"] = v.at("offset");
                return n;
            };
            auto finish = [&]() {
                if (cursor > start) {
                    out["unread_prefix"] = {
                        {"offset", start},
                        {"bytes", cursor - start},
                        {"binary_base64", base64(slice(source, start, cursor - start))}};
                    out["status"] = "partial";
                    complete = false;
                }
            };
            if (id == none_id || id == bool_id) {
                require(size >= 1, "component byte value is truncated");
                out["kind"] = id == none_id ? "none" : "bool";
                out["storage_uint8"] = raw.u8();
                out["value"] = id == none_id ? Json(nullptr) : Json(source[start] != 0);
                cursor = start;
                if (size > 1)
                    out["ignored_suffix_bytes"] = size - 1;
            } else if (id == int_id || id == uint_id || id == double_id) {
                require(size >= 8, "component numeric value is truncated");
                if (id == int_id) {
                    out["kind"] = "int64";
                    out["value"] = raw.i64();
                } else if (id == uint_id) {
                    out["kind"] = "uint64";
                    out["value"] = raw.u64();
                } else {
                    out["kind"] = "double";
                    out["value"] = raw.f64();
                    out["bits_hex"] = hex(slice(source, start, 8));
                }
                cursor = start;
                if (size > 8)
                    out["ignored_suffix_bytes"] = size - 8;
            } else if (id == string_id || id == bytes_id) {
                out["kind"] = id == string_id ? "string" : "bytes";
                if (id == string_id)
                    out.update(text(start, size));
                else
                    out["bytes_base64"] = base64(slice(source, start, size));
                cursor = start;
            } else if (id == noumenon_id || id == vector_id || id == map_id) {
                out["kind"] = id == noumenon_id ? "noumenon" : id == vector_id ? "vector" : "map";
                const auto n = count();
                out["entries"] = Json::array();
                for (std::uint64_t i = 0; i < n; ++i) {
                    if (id == vector_id)
                        out["entries"].push_back(child());
                    else {
                        auto key = child(id == noumenon_id ? string_id : 0);
                        auto value = child(id == noumenon_id ? property_id : 0);
                        out["entries"].push_back(
                            {{"key", std::move(key)}, {"value", std::move(value)}});
                    }
                }
                out["entry_order"] = "native_read_order";
                if (id == map_id)
                    out["runtime_key_comparison"] = "not_evaluated";
                finish();
            } else if (id == property_id) {
                out["kind"] = "property";
                out["value"] = child();
                const auto n = count();
                out["attributes"] = Json::array();
                for (std::uint64_t i = 0; i < n; ++i) {
                    auto key = child(string_id);
                    auto value = child(property_id);
                    out["attributes"].push_back(
                        {{"key", std::move(key)}, {"value", std::move(value)}});
                }
                out["attribute_duplicate_rule"] = "read_into_existing_property";
                finish();
            } else if (id == function_id) {
                out["kind"] = "unified_function";
                out["module_name"] = child(string_id);
                out["operator_name"] = child(string_id);
                finish();
            } else if (id == files_id) {
                out["kind"] = "dependent_files";
                out["root"] = folder(start, cursor, depth + 1);
                finish();
            } else {
                out["kind"] = "unknown";
                out["status"] = "unsupported_type";
                out["payload_base64"] = base64(slice(source, start, size));
                complete = false;
            }
        } catch (const std::exception &e) {
            out["status"] = "invalid";
            out["decode_error"] = e.what();
            // The enclosing binary field retains the source. Do not duplicate a large
            // invalid payload at every level of a malformed recursive property.
            complete = false;
        }
        return out;
    }

    Json folder(std::size_t begin, std::size_t &end, unsigned depth) {
        budget(depth);
        auto typed = [&](std::uint64_t id) {
            auto v = read(begin, end, depth + 1, id);
            require(v.at("status") == "decoded", "invalid dependent file member");
            return v;
        };
        auto count = typed(uint_id);
        const auto n = count.at("value").get<std::uint64_t>();
        require(n <= (end - begin) / 32 && n <= 1000000 - visits,
                "dependent file count exceeds input or node budget");
        Json result = {{"kind", "folder"},
                       {"entries", Json::array()},
                       {"count_source_offset", count.at("offset")}};
        for (std::uint64_t i = 0; i < n; ++i) {
            auto name = typed(string_id);
            auto flag = typed(bool_id);
            auto value =
                flag.at("value").get<bool>() ? folder(begin, end, depth + 1) : typed(bytes_id);
            result["entries"].push_back({{"name", std::move(name)},
                                         {"folder_flag", std::move(flag)},
                                         {"value", std::move(value)}});
        }
        return result;
    }
};

Json source_input(const Json &root, const std::string &key, const std::string &kind) {
    if (!root.contains("entries") || root.at("status") == "invalid")
        return {{"status", "not_evaluated"}};
    const auto &entries = root.at("entries");
    for (std::size_t i = 0; i < entries.size(); ++i) {
        const auto &entry = entries[i];
        if (entry.at("key").value("text", "") != key)
            continue;
        Json out = {{"property_index", i}, {"lookup", "first_exact_byte_key"}};
        const auto &property = entry.at("value");
        if (!property.contains("value")) {
            out["status"] = "not_evaluated";
            return out;
        }
        const auto &value = property.at("value");
        if (value.value("kind", "") != kind) {
            out["status"] = "wrong_value_type";
            return out;
        }
        if (value.at("status") != "decoded") {
            out["status"] = "not_evaluated";
            return out;
        }
        out["status"] = "found";
        out["value_offset"] = value.at("offset");
        out["bytes_base64"] = value.at("bytes_base64");
        out["bytes"] = value.at("bytes");
        return out;
    }
    return {{"status", "absent"}};
}
} // namespace

Json decode_component_properties(const Bytes &bytes) {
    require(bytes.size() <= 64 * 1024 * 1024, "component property input limit");
    StackDecoder decoder{bytes};
    std::size_t end = bytes.size();
    Json out = {{"scope", "stored_component_properties"}};
    try {
        out["root"] = decoder.read(0, end, 0, noumenon_id);
        out["status"] = decoder.complete ? "decoded" : "partial";
        if (out.at("root").at("status") == "invalid")
            out["status"] = "invalid";
        out["consumed_bytes"] = bytes.size() - end;
        if (end)
            out["leading_bytes"] = {{"bytes", end},
                                    {"binary_base64", base64(slice(bytes, 0, end))}};
        out["component_graphics_input"] =
            source_input(out.at("root"), "componentGraphics", "bytes");
        out["component_material_input"] =
            source_input(out.at("root"), "\x07_component_material", "string");
        auto &graphics = out["component_graphics_input"];
        if (graphics.at("status") == "found" && graphics.at("bytes") != 0) {
            try {
                graphics["graphics"] = decode_graphics_bytes(
                    slice(bytes, graphics.at("value_offset").get<std::size_t>(),
                          graphics.at("bytes").get<std::size_t>()));
            } catch (const std::exception &e) {
                graphics["decode_error"] = e.what();
            }
        }
        out["active_graphics_selection"] = "not_established";
        out["runtime_property_restoration"] = "not_evaluated";
    } catch (const std::exception &e) {
        out["status"] = "invalid";
        out["decode_error"] = e.what();
    }
    return out;
}
} // namespace p3d
