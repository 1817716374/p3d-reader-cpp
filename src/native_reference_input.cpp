#include "internal.hpp"

namespace p3d {
NativeReferenceLayout native_reference_layout(const Bytes &source) {
    require(Reader(source, 4).u16() == 13, "reference layout requires type 13");
    const auto words = Reader(source, 8).u32(), base_words = Reader(source, 12).u32();
    require(words >= base_words && base_words >= 16, "invalid reference word counts");
    require(source.size() == 4 + std::uint64_t(base_words) * 2 ||
                source.size() == 4 + std::uint64_t(words) * 2,
            "reference data must contain exactly its base or complete record");
    // Preserve the native short-circuit order. A modern discriminator can stop
    // before any later legacy field is read.
    bool legacy = Reader(source, 160).u32() == 0;
    if (legacy)
        legacy = Reader(source, 342).u16() == 0;
    if (legacy)
        legacy = Reader(source, 344).u16() == 0;
    std::uint16_t entries = 0;
    if (legacy) {
        entries = Reader(source, 346).u16();
        legacy = entries <= 2500 && base_words == 172u + 8u * entries;
    }
    if (!legacy)
        return {source, false, 0};
    require(source.size() >= 348u + 16u * entries, "truncated type 13 legacy entries");
    require(words <= UINT32_MAX - 12u, "reference upgrade word count overflow");
    Bytes data(source.size() + 24, 0);
    auto copy = [&](std::size_t destination, std::size_t origin, std::size_t size) {
        std::copy_n(source.begin() + origin + 4, size, data.begin() + destination + 4);
    };
    auto put32 = [&](std::size_t native_offset, std::uint32_t value) {
        for (unsigned i = 0; i < 4; ++i)
            data[native_offset + 4 + i] = static_cast<std::uint8_t>(value >> (8 * i));
    };
    std::copy_n(source.begin(), 4, data.begin());
    copy(0, 0, 32);
    put32(4, words + 12);
    put32(8, base_words + 12);
    copy(0x20, 0x20, 4);
    put32(0x24, 1);
    copy(0x28, 0x24, 8);
    copy(0x38, 0x30, 12);
    for (unsigned i = 0; i < 8; ++i)
        copy(0x44 + 12 * i, 0x3c, 2);
    copy(0xa8, 0xa0, 24);
    copy(0xc0, 0xb8, 24);
    copy(0xd8, 0xd0, 72);
    copy(0x120, 0x118, 8);
    copy(0x138, 0x120, 16);
    copy(0x148, 0x130, 32);
    copy(0x168, 0x150, 2);
    copy(0x16e, 0x156, 2);
    // Entries and all attribute bytes move together, without decoding or
    // rewriting the attribute tail. This also works for a base-only input.
    copy(0x170, 0x158, source.size() - 348);
    return {std::move(data), true, entries};
}
namespace {
Json source_number(const Bytes &data, std::size_t offset) {
    const auto value = Reader(data, offset).f64();
    if (std::isfinite(value))
        return value;
    const auto bits = Reader(data, offset).u64();
    std::string encoded(16, '0');
    for (unsigned i = 0; i < 16; ++i)
        encoded[15 - i] = "0123456789abcdef"[(bits >> (i * 4)) & 15];
    return {{"floating_point", std::isinf(value) ? "infinity" : "nan"},
            {"negative", bool(bits >> 63)},
            {"ieee754_hex", encoded}};
}
Json reference_clipping(const Bytes &data, bool upgraded) {
    const auto base_size = 4 + std::uint64_t(Reader(data, 12).u32()) * 2;
    const auto count = Reader(data, 370).u16();
    const bool accepted = count >= 4 && count <= 2500;
    const bool complete = 372u + 16u * count <= base_size;
    Json boundary = {
        {"source_point_count", count},
        {"point_count_layout_offset", 370},
        {"point_count_source_offset", upgraded ? 346 : 370},
        {"points_layout_offset", 372},
        {"points_source_offset", upgraded ? 348 : 372},
        {"loader_status", count == 0  ? "empty"
                          : !accepted ? "rejected_count"
                          : complete  ? "accepted"
                                      : "truncated"},
        {"loader_return_code",
         accepted && !complete ? Json() : Json(count == 0 || accepted ? 0 : 1)},
        {"loaded_point_count", accepted && !complete ? Json() : Json(accepted ? count : 0)},
        {"source_points_status", complete ? "decoded" : "truncated"},
        {"source_points", Json::array()}};
    // Rejected counts never cause the native loader to dereference the array.
    // Decode any complete source array separately without making it active.
    if (complete) {
        bool finite = true;
        for (unsigned i = 0; i < count; ++i) {
            const auto offset = 372u + 16u * i;
            boundary["source_points"].push_back(
                {source_number(data, offset), source_number(data, offset + 8)});
            finite = finite && std::isfinite(Reader(data, offset).f64()) &&
                     std::isfinite(Reader(data, offset + 8).f64());
        }
        boundary["all_components_finite"] = finite;
    }
    const auto source_flags = Reader(data, 64).u32();
    const auto upper = Reader(data, 316).f64(), lower = Reader(data, 324).f64();
    // The ordered native comparison also clears both bits for equal endpoints,
    // including equal infinities. An unordered (NaN) comparison preserves them.
    const bool clear_depths = lower >= upper;
    const auto flags = clear_depths ? source_flags & ~0xc00u : source_flags;
    const bool lower_enabled = (flags & 0x400) != 0, upper_enabled = (flags & 0x800) != 0;
    const double selected_lower = lower_enabled ? lower : -4503599627370496.,
                 selected_upper = upper_enabled ? upper : 4503599627370495.;
    Json depths = {{"source_flags", source_flags},
                   {"flags_layout_offset", 64},
                   {"flags_source_offset", upgraded ? 56 : 64},
                   {"flags_after_depth_validation", flags},
                   {"depth_flags_cleared", clear_depths},
                   {"lower_enabled", lower_enabled},
                   {"upper_enabled", upper_enabled},
                   {"source_lower", source_number(data, 324)},
                   {"source_upper", source_number(data, 316)},
                   {"lower_layout_offset", 324},
                   {"upper_layout_offset", 316},
                   {"lower_source_offset", upgraded ? 300 : 324},
                   {"upper_source_offset", upgraded ? 292 : 316}};
    if (std::isfinite(selected_lower) && std::isfinite(selected_upper))
        depths["local_interval"] = {
            {"status", "computed"}, {"lower", selected_lower}, {"upper", selected_upper}};
    else
        depths["local_interval"] = {{"status", "not_evaluated"},
                                    {"reason", "nonfinite_enabled_depth"}};
    Json out = {{"profile", "bimbase_2025_reference_clip_input"},
                {"scope", "local_boundary_and_depth_after_base_load"},
                {"status", accepted && !complete ? "invalid" : "decoded"},
                {"boundary", std::move(boundary)},
                {"depths", std::move(depths)},
                {"world_clip_volume", "not_evaluated"}};
    if (accepted && !complete)
        out["error"] = "truncated accepted reference clipping boundary";
    return out;
}
Json reference_transform(const Bytes &base) {
    Json out = {{"profile", "bimbase_2025_reference_base_input"},
                {"scope", "after_base_matrix_load_before_linkages"},
                {"status", "not_evaluated"},
                {"matrix_layout_offset", 220},
                {"scale_layout_offset", 292}};
    try {
        Matrix3 source{};
        Reader reader(base, 220);
        for (auto &row : source)
            for (auto &v : row)
                v = reader.f64();
        const auto raw_scale = Reader(base, 292).f64();
        out["source_matrix"] = source;
        for (unsigned r = 0; r < 3; ++r)
            for (unsigned c = 0; c < 3; ++c)
                if (!std::isfinite(source[r][c]))
                    out["source_matrix"][r][c] = nullptr;
        out["source_scale"] = std::isfinite(raw_scale) ? Json(raw_scale) : Json();
        require(std::isfinite(raw_scale), "nonfinite reference source scale");
        for (const auto &row : source)
            for (auto v : row)
                require(std::isfinite(v), "nonfinite reference matrix");
        const auto initial_scale = raw_scale == 0 ? 1. : raw_scale;
        auto columns = source;
        Point3 lengths{};
        for (unsigned c = 0; c < 3; ++c) {
            const auto x = source[0][c], y = source[1][c], z = source[2][c];
            const auto length = std::sqrt((x * x + y * y) + z * z);
            require(std::isfinite(length), "reference column length overflow");
            lengths[c] = length;
            if (length > 0) {
                const auto reciprocal = 1 / length;
                require(std::isfinite(reciprocal), "reference column normalization overflow");
                for (unsigned r = 0; r < 3; ++r)
                    columns[r][c] *= reciprocal;
            } else {
                // Native vector normalization replaces a zero column with +X,
                // even when the subsequent rigid-scale predicate fails.
                columns[0][c] = 1;
                columns[1][c] = columns[2][c] = 0;
            }
        }
        const auto largest = std::max({lengths[0], lengths[1], lengths[2]});
        const auto smallest = std::min({lengths[0], lengths[1], lengths[2]});
        const auto ratio = largest > 0 ? smallest / largest : 0.;
        bool orthogonal = true;
        for (unsigned i = 0; i < 3; ++i)
            for (unsigned j = 0; j < 3; ++j) {
                const auto gram = (columns[1][i] * columns[1][j] + columns[0][i] * columns[0][j]) +
                                  columns[2][i] * columns[2][j];
                orthogonal = orthogonal && std::abs(gram - (i == j ? 1. : 0.)) <= 1e-12;
            }
        // The caller aliases the output matrix with the source. The determinant
        // therefore reads normalized columns, including on failed predicates.
        const auto &m = columns;
        const auto determinant =
            (((((m[0][0] * m[1][1]) * m[2][2] + (m[0][1] * m[1][2]) * m[2][0]) +
               (m[0][2] * m[1][0]) * m[2][1]) -
              (m[0][0] * m[1][2]) * m[2][1]) -
             (m[1][0] * m[0][1]) * m[2][2]) -
            (m[0][2] * m[1][1]) * m[2][0];
        const bool rigid_scale = orthogonal && ratio > 0.999999999999 && determinant > 0;
        const bool rescale = rigid_scale && std::abs(largest - 1) > 1e-10;
        auto matrix = columns;
        auto scale = initial_scale;
        if (rescale) {
            scale *= largest;
            const auto reciprocal = 1 / largest;
            for (auto &row : matrix)
                for (auto &v : row)
                    v *= reciprocal;
        }
        require(std::isfinite(scale), "reference scale multiplication overflow");
        for (const auto &row : matrix)
            for (auto v : row)
                require(std::isfinite(v), "reference matrix rescaling overflow");
        out.update({{"status", "computed"},
                    {"initial_scale", initial_scale},
                    {"zero_scale_default_applied", raw_scale == 0},
                    {"column_lengths", lengths},
                    {"axis_ratio", ratio},
                    {"column_normalized_matrix", columns},
                    {"columns_orthogonal", orthogonal},
                    {"normalized_determinant", determinant},
                    {"rigid_scale", rigid_scale},
                    {"matrix_scale", largest},
                    {"scale_adjustment_applied", rescale},
                    {"matrix", matrix},
                    {"scale", scale}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace
Json native_reference_input(const Bytes &source) {
    Json out = {{"encoding", "native_reference_input"}, {"status", "invalid"}};
    try {
        require(source.size() == 4 + std::uint64_t(Reader(source, 8).u32()) * 2,
                "reference input requires a complete record");
        const auto layout = native_reference_layout(source);
        const auto &data = layout.data;
        out["layout"] = {{"kind", layout.upgraded ? "legacy_upgraded" : "current"},
                         {"upgraded", layout.upgraded},
                         {"source_base_word_count", Reader(source, 12).u32()},
                         {"source_record_word_count", Reader(source, 8).u32()},
                         {"base_word_count", Reader(data, 12).u32()},
                         {"record_word_count", Reader(data, 8).u32()}};
        if (layout.upgraded) {
            out["layout"]["legacy_entry_count"] = layout.entry_count;
            out["layout"]["converted_record"] = rawbytes(data);
        }
        const auto base_size = 4 + std::uint64_t(Reader(data, 12).u32()) * 2;
        require(base_size >= 372, "truncated current reference base");
        const auto entries = Reader(data, 370).u16();
        out["entry_count"] = entries;
        out["origin_inputs"] = {{"primary_flags", Reader(data, 60).u32()},
                                {"secondary_flags", Reader(data, 64).u32()},
                                {"primary_flags_source_offset", layout.upgraded ? 52 : 60},
                                {"secondary_flags_source_offset", layout.upgraded ? 56 : 64}};
        out["clipping"] = reference_clipping(data, layout.upgraded);
        require(out["clipping"]["status"] == "decoded", "truncated current reference entries");
        out["transform"] = reference_transform(data);
        out["transform"]["source_matrix_offset"] = layout.upgraded ? 212 : 220;
        out["transform"]["source_scale_offset"] = layout.upgraded ? 284 : 292;
        auto point_input = [&](std::size_t offset, std::size_t source_offset) {
            return Json{
                {"value", Json::array({source_number(data, offset), source_number(data, offset + 8),
                                       source_number(data, offset + 16)})},
                {"source_offset", source_offset}};
        };
        out["affine_inputs"] = {
            {"scope", "after_base_point_load_before_linkages"},
            {"source_offsets_include_stream_prefix", true},
            {"reference_point", point_input(172, layout.upgraded ? 164 : 172)},
            {"translation_point", point_input(196, layout.upgraded ? 188 : 196)}};
        out["status"] = "decoded";
        out["remaining_reference_semantics"] = "not_evaluated";
    } catch (const std::exception &e) {
        out["error"] = e.what();
    }
    return out;
}
} // namespace p3d
