#include "internal.hpp"

namespace p3d {
Json native_dependency_link(const Bytes &payload) {
    Json out = {{"encoding", "native_dependency_link"},
                {"reader_profile", "bimbase_2025_dependency_input"},
                {"status", "invalid"},
                {"runtime_resolution", "not_evaluated"},
                {"entries", Json::array()}};
    std::vector<bool> assigned(payload.size(), false);
    auto take = [&](std::size_t offset, std::size_t size) {
        require(offset <= payload.size() && size <= payload.size() - offset,
                "truncated dependency payload");
        std::fill(assigned.begin() + offset, assigned.begin() + offset + size, true);
        return Reader(payload, offset);
    };
    auto id = [&](std::size_t offset) { return take(offset, 8).u64(); };
    try {
        auto header = take(0, 8);
        const auto owner = header.u16(), relation = header.u16(), flags = header.u16(),
                   count = header.u16();
        const unsigned format = (flags >> 10) & 15;
        out.update({{"owner_code", owner},
                    {"relation_code", relation},
                    {"flags", flags},
                    {"reference_format", format},
                    {"entry_count", count},
                    {"dependency_index_input", (flags & 1)  ? "skipped_flag_1"
                                               : format > 8 ? "skipped_reference_format"
                                                            : "eligible"},
                    {"unassigned_flag_bits", flags & ~0x3c01u}});
        require(format <= 8, "unsupported dependency reference format");
        static constexpr std::size_t strides[] = {8, 16, 40, 48, 16, 24, 0, 24, 16};
        const auto stride = strides[format];
        if (format == 6 && count != 0) {
            // This format contains one variable path, not count fixed-size paths.
            const auto n = take(8, 4).u32();
            require(n <= (payload.size() - 12) / 8 && 24ull + 8ull * n <= payload.size(),
                    "truncated dependency owner path");
            Json path = Json::array();
            for (std::size_t i = 0; i < n; ++i)
                path.push_back(id(24 + i * 8));
            out["owner_path"] = {{"ids", std::move(path)},
                                 {"id_count", n},
                                 {"payload_offset", 24},
                                 {"lookup_order", "reverse"},
                                 {"input_iteration", "first_only"}};
            // A zero-length path is not safe for the reader's context-free branch.
            require(n != 0, "empty dependency owner path");
        } else if (format != 6) {
            require(count <= (payload.size() - 8) / stride, "truncated dependency entries");
            out["entry_stride"] = stride;
            const bool reverse_path = format == 0 && owner == 10000 && relation == 4;
            if (reverse_path)
                out["lookup_rule"] = "reverse_owner_path";
            for (std::size_t i = 0; i < count; ++i) {
                const auto offset = 8 + i * stride;
                Json entry = {{"entry_index", i}, {"payload_offset", offset}};
                Json references = Json::array();
                auto current = [&](std::uint64_t element) {
                    references.push_back({{"element_id", element}, {"lookup", "current_owner"}});
                };
                auto through = [&](std::uint64_t element, std::uint64_t reference) {
                    references.push_back(
                        {{"element_id", element},
                         {"owner_reference_id", reference},
                         {"lookup", reference ? "referenced_owner" : "current_owner"}});
                };
                if (format == 0 || format == 1) {
                    const auto element = id(offset);
                    entry["element_id"] = element;
                    if (reverse_path)
                        entry["lookup"] = "reverse_owner_path";
                    else
                        current(element);
                } else if (format == 4 || format == 5) {
                    const auto element = id(offset),
                               reference = id(offset + (format == 4 ? 8 : 16));
                    through(element, reference);
                    current(reference);
                } else if (format == 8) {
                    const auto model = take(offset, 4).i32();
                    const auto element = id(offset + 8);
                    references.push_back(
                        {{"element_id", element},
                         {"model_index", model},
                         {"lookup", model == -1 ? "system_owner" : "model_index"}});
                } else if (format == 2 || format == 3) {
                    const auto kind = take(offset, 1).u8();
                    entry["selector_code"] = kind;
                    const auto first = id(offset + 8);
                    if (kind == 2 || kind == 8) {
                        const auto second = id(offset + 16), a = id(offset + 24),
                                   b = id(offset + 32);
                        through(first, a);
                        through(second, b);
                        current(a);
                        current(b);
                    } else {
                        const auto reference = id(offset + 16);
                        through(first, reference);
                        current(reference);
                    }
                } else if (format == 7) {
                    // The compact form stores the two halves of its ID in the
                    // opposite order from an ordinary little-endian uint64.
                    const auto high = take(offset, 4).u32(), low = take(offset + 4, 4).u32();
                    const auto kind = take(offset + 8, 1).u8();
                    const auto element = (std::uint64_t(high) << 32) | low;
                    entry["element_id"] = element;
                    entry["selector_code"] = kind;
                    if (kind <= 3) {
                        static constexpr unsigned converted[] = {1, 6, 3, 7};
                        entry["expanded_selector_code"] = converted[kind];
                        through(element, 0);
                        current(0);
                    } else {
                        entry["reference_status"] = "unsupported_compact_selector";
                    }
                    // Selector parameters are retained below until their semantics
                    // and normalized representation have been established.
                }
                entry["references"] = std::move(references);
                out["entries"].push_back(std::move(entry));
            }
        }
        out["status"] = "decoded";
    } catch (const std::exception &e) {
        out["error"] = e.what();
        // Never publish a truncated list as a complete dependency list.
        out["entries"] = Json::array();
    }
    Json unassigned = Json::array();
    for (std::size_t i = 0; i < payload.size();) {
        if (assigned[i]) {
            ++i;
            continue;
        }
        const auto begin = i;
        while (i < payload.size() && !assigned[i])
            ++i;
        unassigned.push_back(
            {{"payload_offset", begin}, {"data", rawbytes(slice(payload, begin, i - begin))}});
    }
    if (out["status"] == "decoded" && (!unassigned.empty() || out["unassigned_flag_bits"] != 0))
        out["status"] = "partial";
    out["unassigned_ranges"] = std::move(unassigned);
    return out;
}
} // namespace p3d
