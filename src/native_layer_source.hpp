#pragma once
#include "internal.hpp"

namespace p3d::detail {
struct SavedLayerRow {
    std::size_t source;
    Json layer;
};
struct SavedLocalLayers {
    std::map<std::uint32_t, SavedLayerRow> by_id;
    Json rejected = Json::array();
};
// Consume bounded input in source order without retaining a second complete
// decoded-record array. Callers publish final state only after this returns.
template <class Consume>
inline void for_each_saved_layer_input(const Json &records, std::size_t table_index,
                                       bool allow_group, Consume &&consume) {
    require(records.is_array() && table_index < records.size(), "layer_table_index_invalid");
    const auto &table = records.at(table_index);
    const auto body = bytesof(table.at("data"));
    Reader header(body);
    require(header.at<std::uint16_t>(4) == 10 && header.at<std::uint32_t>(16) == 1,
            allow_group ? "native_layer_table_record_required"
                        : "local_layer_table_record_required");
    const auto source_table = decode_native_layer_table(body, table.at("links"));
    require(!source_table.contains("decode_error") &&
                (source_table.at("kind") == "local" ||
                 (allow_group && source_table.at("kind") == "layer_group")),
            allow_group ? "fresh_local_or_group_layer_table_required"
                        : "fresh_local_layer_table_required");
    const auto count = source_table.at("declared_child_count").get<std::uint32_t>();
    require(count <= records.size() - table_index - 1, "layer_table_child_count_invalid");
    auto next = table.at("offset").get<std::uint64_t>();
    auto advance = [&](const Bytes &b) {
        const auto length = std::uint64_t(Reader(b).at<std::uint32_t>(8)) * 2 + 4;
        require(length <= UINT64_MAX - next, "layer_record_offset_overflow");
        next += length;
    };
    advance(body);
    for (std::size_t i = table_index + 1; i <= table_index + count; ++i) {
        const auto &record = records.at(i);
        require(record.at("stream") == table.at("stream") && record.at("offset") == next,
                "layer_table_member_boundary_invalid");
        const auto bytes = bytesof(record.at("data"));
        Reader r(bytes);
        require(r.at<std::uint16_t>(4) == 49 && (r.at<std::uint16_t>(6) & 0x80) &&
                    r.at<std::uint32_t>(16) == 1,
                "layer_table_member_type_invalid");
        auto layer = decode_native_layer(bytes, record.at("links"));
        require(layer.value("status", "") == "decoded" && !layer.contains("decode_error"),
                "layer_table_member_decode_unknown");
        for (const auto &s : layer.at("strings"))
            require(!s.contains("decode_error"), "layer_name_linkage_decode_unknown");
        consume(SavedLayerRow{i, std::move(layer)});
        advance(bytes);
    }
}
inline SavedLocalLayers read_saved_local_layers(const Json &records, std::size_t table_index) {
    SavedLocalLayers out;
    for_each_saved_layer_input(records, table_index, false, [&](SavedLayerRow row) {
        const auto id = row.layer.at("layer_id").get<std::uint32_t>();
        if (id == 0 || id == UINT32_MAX)
            out.rejected.push_back({{"native_record_index", row.source},
                                    {"layer_id", id},
                                    {"reason", "fresh_layer_input_id_rejected"}});
        else
            out.by_id[id] = std::move(row);
    });
    return out;
}
} // namespace p3d::detail
