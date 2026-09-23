#include "internal.hpp"
#include "proxy_cache_fields.hpp"

namespace p3d {
namespace {
void remove_prefix_from_offsets(Json &value) {
    if (value.is_number_integer() || value.is_number_unsigned()) {
        const auto offset = value.get<std::size_t>();
        require(offset >= 4, "invalid_cached_model_source_offset");
        value = offset - 4;
    } else if (value.is_structured()) {
        for (auto &child : value)
            remove_prefix_from_offsets(child);
    }
}
void native_offsets(Json &value) {
    if (value.is_array()) {
        for (auto &child : value)
            native_offsets(child);
    } else if (value.is_object()) {
        for (auto it = value.begin(); it != value.end(); ++it) {
            const auto &key = it.key();
            if (key == "source_offset" || key == "source_offsets" ||
                (key.size() >= 14 && key.compare(key.size() - 14, 14, "_source_offset") == 0))
                remove_prefix_from_offsets(it.value());
            else if (key == "source_offsets_include_stream_prefix")
                it.value() = false;
            else
                native_offsets(it.value());
        }
    }
}
Json source_number(const Bytes &bytes, std::size_t offset) {
    const auto number = Reader(bytes, offset).f64();
    if (std::isfinite(number))
        return number;
    const auto bits = Reader(bytes, offset).u64();
    std::string encoded(16, '0');
    for (unsigned i = 0; i < 16; ++i)
        encoded[15 - i] = "0123456789abcdef"[(bits >> (i * 4)) & 15];
    return {{"floating_point", std::isinf(number) ? "infinity" : "nan"},
            {"negative", bool(bits >> 63)},
            {"ieee754_hex", encoded}};
}
} // namespace

Json cached_model_metadata(const Bytes &bytes) {
    Json out = {{"status", bytes.empty() ? "absent" : "not_evaluated"},
                {"semantics_status", "partial"},
                {"scope", "cached_native_model_header"},
                {"source_offsets_include_stream_prefix", false}};
    if (bytes.empty())
        return out;
    try {
        require(bytes.size() >= 496, "truncated_cached_model_header");
        require(Reader(bytes).u16() == 47 && Reader(bytes, 12).u32() == 32,
                "unsupported_cached_model_header_type");
        // The cache writer copies a native model header without the external
        // stream prefix. Adapt only the framing for the existing record reader.
        Bytes framed(4, 0);
        framed.insert(framed.end(), bytes.begin(), bytes.end());
        auto records = parse_native(framed);
        require(records.size() == 1, "cached_model_requires_one_native_record");
        auto record = std::move(records[0]);
        // Only these decoders use offsets into the framed model record.
        // Linkage decoders may instead use offsets local to their payload.
        for (const auto *key : {"model_unit_state", "model_coordinate_state", "model_view_state",
                                "model_layer_group_reference", "modification_time"})
            if (record.contains(key))
                native_offsets(record[key]);
        record["length"] = record.at("length").get<std::size_t>() - 4;
        record["base_length"] = record.at("base_length").get<std::size_t>() - 4;
        record["data"] = rawbytes(slice(bytes, 0, record.at("base_length").get<std::size_t>()));
        for (auto &link : record["links"])
            link["offset"] = link.at("offset").get<std::size_t>() - 4;
        out.update(
            {{"status", "decoded"},
             {"record", std::move(record)},
             {"remaining_semantics", {"other_model_header_fields", "uninterpreted_linkages"}}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json cached_reference_parameters(const Bytes &bytes) {
    Json out = {{"status", "not_evaluated"},
                {"semantics_status", "partial"},
                {"scope", "cached_reference_construction_parameters"},
                {"target_application_status", "not_evaluated"},
                {"native_use_condition", "constructing_missing_child_reference"},
                {"remaining_semantics", "other_packed_reference_fields"}};
    try {
        require(bytes.size() == 328, "cached_reference_parameters_require_328_bytes");
        auto point = [&](std::size_t offset) {
            return Json{{"value", Json::array({source_number(bytes, offset),
                                               source_number(bytes, offset + 8),
                                               source_number(bytes, offset + 16)})},
                        {"source_offset", offset}};
        };
        out["origin_inputs"] = {{"primary_flags", Reader(bytes).u32()},
                                {"secondary_flags", Reader(bytes, 4).u32()},
                                {"primary_flags_source_offset", 0},
                                {"secondary_flags_source_offset", 4}};
        out["affine_inputs"] = {{"reference_point", point(128)},
                                {"translation_point", point(104)},
                                {"source_offsets_include_stream_prefix", false}};
        Json matrix = Json::array();
        bool finite = std::isfinite(Reader(bytes, 160).f64());
        for (unsigned row = 0; row < 3; ++row) {
            Json values = Json::array();
            for (unsigned column = 0; column < 3; ++column) {
                const auto offset = 168 + (3 * row + column) * 8;
                finite = finite && std::isfinite(Reader(bytes, offset).f64());
                values.push_back(source_number(bytes, offset));
            }
            matrix.push_back(std::move(values));
        }
        // Cached construction directly copies the saved matrix and scale into
        // the reference. The ordinary type-13 loader's zero-scale default and
        // column normalization are not called on this path.
        out["transform"] = {{"status", finite ? "computed" : "not_evaluated"},
                            {"scope", "cached_direct_reference_field_load"},
                            {"matrix", std::move(matrix)},
                            {"scale", source_number(bytes, 160)},
                            {"source_matrix_offset", 168},
                            {"source_scale_offset", 160},
                            {"normalization_applied", false},
                            {"zero_scale_default_applied", false}};
        if (!finite)
            out["transform"]["reason"] = "nonfinite_cached_reference_transform";
        out["status"] = "decoded";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
