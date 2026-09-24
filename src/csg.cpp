#include <p3d/csg.hpp>
#include "internal.hpp"

namespace p3d {
namespace {
// A view into the archive: child lengths cannot expose a sibling's bytes.
struct Cursor {
    Reader r;
    std::size_t end;
    Cursor(const Bytes &b, std::size_t offset, std::size_t length) : r(b, offset) {
        r.need(length);
        end = offset + length;
    }
    std::size_t left() const {
        return end - r.p;
    }
    void need(std::size_t n) const {
        require(n <= left(), "CSG bounded payload overrun");
    }
    std::int32_t integer() {
        need(4);
        return r.i32();
    }
    std::uint8_t byte() {
        need(1);
        return r.u8();
    }
    Bytes take(std::size_t n) {
        need(n);
        return r.take(n);
    }
    std::size_t count(std::size_t width) {
        auto n = integer();
        require(n >= 0 && std::size_t(n) <= left() / width, "CSG signed count out of bounds");
        return std::size_t(n);
    }
    std::pair<std::size_t, std::size_t> block() {
        const auto n = count(1), offset = r.p;
        r.skip(n);
        return {offset, n};
    }
};
Json range(std::size_t offset, std::size_t size) {
    return {{"source_offset", offset}, {"source_bytes", size}};
}
Json indices(Cursor &r) {
    const auto n = r.count(4);
    Json out = Json::array();
    for (std::size_t i = 0; i < n; ++i)
        out.push_back(r.integer());
    return out;
}
Json nodes(const Bytes &b, std::size_t offset, std::size_t size, std::size_t max_nodes) {
    Json out = {{"nodes", Json::array()}, {"root_index", nullptr}, {"status", "decoded"}};
    if (!size)
        return out;
    struct Pending {
        std::size_t offset, size;
    };
    std::vector<Pending> pending;
    auto add = [&](std::size_t start, std::size_t length) -> Json {
        if (!length)
            return nullptr;
        const auto id = pending.size();
        pending.push_back({start, length});
        out["nodes"].push_back(range(start, length));
        return id;
    };
    out["root_index"] = add(offset, size);
    std::size_t decoded = 0;
    for (std::size_t i = 0; i < pending.size(); ++i) {
        const auto item = pending[i];
        Json node = range(item.offset, item.size);
        node["node_index"] = i;
        node["left_index"] = nullptr;
        node["right_index"] = nullptr;
        node["status"] = "decoded";
        try {
            if (decoded >= max_nodes) {
                node["status"] = "node_limit";
            } else {
                ++decoded;
                Cursor r(b, item.offset, item.size);
                const auto version = r.integer();
                node["version"] = version;
                if (version != 0x1ee290) {
                    node["status"] = "unsupported_version";
                } else {
                    // This word is ignored by the native reader for this version.
                    node["header_offset_word"] = r.integer();
                    node["geometry_indices"] = indices(r);
                    node["cache_indices"] = indices(r);
                    node["matrix_indices"] = indices(r);
                    const auto guid_count = r.count(16);
                    node["geometry_guids_hex"] = Json::array();
                    for (std::size_t j = 0; j < guid_count; ++j)
                        node["geometry_guids_hex"].push_back(hex(r.take(16)));
                    const auto op = r.integer();
                    node["operation"] = op;
                    static const char *names[] = {"union",    "intersection", "difference",
                                                  "compound", "unknown",      "plane_cut"};
                    node["operation_name"] = op >= 0 && op < 6 ? Json(names[op]) : Json();
                    node["guid_hex"] = hex(r.take(16));
                    const auto old = r.byte();
                    node["is_old_value"] = old;
                    node["is_old"] = old != 0;
                    const auto children = r.byte();
                    node["child_code"] = children;
                    node["canonical_child_code"] = children <= 3;
                    if (children) {
                        const auto first = r.block();
                        node[children == 2 ? "right_index" : "left_index"] =
                            add(first.first, first.second);
                        // The native reader handles every nonzero code except 1/2
                        // as two children, rather than testing independent bits.
                        if (children != 1 && children != 2) {
                            const auto second = r.block();
                            node["right_index"] = add(second.first, second.second);
                        }
                    }
                    if (r.left()) {
                        node["trailing_bytes"] = range(r.r.p, r.left());
                        node["status"] = "decoded_with_trailing_bytes";
                    }
                }
            }
        } catch (const std::exception &e) {
            node["status"] = "invalid";
            node["decode_error"] = e.what();
        }
        if (node["status"] != "decoded")
            out["status"] = "partial";
        out["nodes"][i] = std::move(node);
    }
    return out;
}
Json geometry_list(Cursor &r) {
    const auto count = r.count(4);
    Json result = Json::array();
    for (std::size_t i = 0; i < count; ++i) {
        const auto block = r.block();
        Json item = range(block.first, block.second);
        item["source_index"] = i;
        item["status"] = "decoded";
        try {
            item["geometry"] = decode_bgfb(slice(r.r.b, block.first, block.second));
        } catch (const std::exception &e) {
            item["status"] = "not_decoded";
            item["decode_error"] = e.what();
        }
        result.push_back(std::move(item));
    }
    return result;
}
void references(Json &tree, std::size_t geometries, std::size_t caches, std::size_t matrices) {
    const std::pair<const char *, std::size_t> lists[] = {
        {"geometry_indices", geometries}, {"cache_indices", caches}, {"matrix_indices", matrices}};
    for (auto &node : tree["nodes"]) {
        for (const auto &list : lists) {
            if (!node.contains(list.first))
                continue;
            Json refs = Json::array();
            for (const auto &value : node.at(list.first)) {
                const auto index = value.get<std::int32_t>();
                const bool valid = index >= 0 && std::size_t(index) < list.second;
                refs.push_back({{"source_index", value},
                                {"target_index", valid ? value : Json()},
                                {"status", valid ? "in_range" : "out_of_range"}});
            }
            node["references"][list.first] = std::move(refs);
        }
    }
}
} // namespace
Json decode_csg_bytes(const Bytes &b, std::size_t max_nodes) {
    Cursor r(b, 0, b.size());
    const auto marker = r.integer();
    require(marker == 22, "CSG archive marker");
    Json out = {{"_type", "GeCsgTree"},
                {"encoding", "csg_archive"},
                {"marker", marker},
                {"raw_base64", base64(b)},
                {"archive_status", "decoded"},
                {"boolean_evaluation_status", "not_evaluated"},
                {"native_restore_status", "not_evaluated"}};
    out["geometries"] = geometry_list(r);
    out["node_caches"] = geometry_list(r);
    const auto count = r.count(96);
    out["transforms"] = Json::array();
    for (std::size_t i = 0; i < count; ++i) {
        Json item = range(r.r.p, 96);
        item["source_index"] = i;
        item["matrix_3x4_rows"] = Json::array();
        for (unsigned row = 0; row < 3; ++row)
            item["matrix_3x4_rows"].push_back(r.r.doubles(4));
        out["transforms"].push_back(std::move(item));
    }
    out["angle_tolerance"] = r.integer();
    const auto tree = r.block();
    out["tree"] = nodes(b, tree.first, tree.second, max_nodes);
    out["tree"].update(range(tree.first, tree.second));
    references(out["tree"], out["geometries"].size(), out["node_caches"].size(), count);
    out["reference_index_basis"] = "zero_based_within_each_archive_list";
    for (auto key : {"attach_data", "mark_bytes"}) {
        const auto block = r.block();
        out[key] = range(block.first, block.second);
        out[key]["raw_base64"] = base64(slice(b, block.first, block.second));
    }
    if (r.left()) {
        out["trailing_bytes"] = range(r.r.p, r.left());
        out["archive_status"] = "decoded_with_trailing_bytes";
    }
    return out;
}
} // namespace p3d
