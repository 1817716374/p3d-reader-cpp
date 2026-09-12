#include "internal.hpp"

unsigned native_reference_input_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto put = [](Bytes &b, std::size_t offset, std::uint64_t value, unsigned size) {
        for (unsigned i = 0; i < size; ++i)
            b.at(offset + i) = static_cast<std::uint8_t>(value >> (8 * i));
    };
    auto number = [&](Bytes &b, std::size_t offset, double value) {
        std::uint64_t bits;
        std::memcpy(&bits, &value, 8);
        put(b, offset, bits, 8);
    };
    const Matrix3 identity{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    auto matrix = [&](Bytes &b, std::size_t offset, const Matrix3 &m) {
        for (const auto &row : m)
            for (auto v : row) {
                number(b, offset, v);
                offset += 8;
            }
    };
    auto wire = [&](const Matrix3 &m, double scale) {
        Bytes b(372, 0);
        put(b, 4, 13, 2);
        put(b, 6, 0x80, 2);
        put(b, 8, 184, 4);
        put(b, 12, 184, 4);
        matrix(b, 220, m);
        number(b, 292, scale);
        return b;
    };
    for (unsigned count : {0u, 1u, 2500u}) {
        const unsigned old_base = 348 + 16 * count;
        Bytes old(old_base + 20);
        for (std::size_t i = 0; i < old_base; ++i)
            old[i] = static_cast<std::uint8_t>((i * 37 + 19) % 251 + 1);
        put(old, 4, 13, 2);
        put(old, 6, 0xa0, 2);
        put(old, 8, (old.size() - 4) / 2, 4);
        put(old, 12, (old_base - 4) / 2, 4);
        put(old, 160, 0, 4);
        put(old, 342, 0, 2);
        put(old, 344, 0, 2);
        put(old, 346, count, 2);
        matrix(old, 212, identity);
        number(old, 284, 7.);
        put(old, old_base, 0x1005, 2);
        put(old, old_base + 2, 0x7a55, 2);
        put(old, old_base + 4, 0xfedcba9876543210ULL, 8);
        const auto unchanged = old;
        const auto result = native_reference_layout(old);
        check(result.upgraded && result.entry_count == count && old == unchanged,
              "legacy reference conversion preserves source and accepts entry count endpoints");
        // Independent byte-origin map: -1 means cleared; otherwise the exact
        // source byte, including the physical prefix, must survive unchanged.
        std::vector<int> origin(result.data.size(), -1);
        for (unsigned i = 0; i < 40; ++i)
            origin[i] = i;
        auto map = [&](unsigned output, unsigned input, unsigned size) {
            for (unsigned i = 0; i < size; ++i)
                origin[output + i] = input + i;
        };
        map(44, 40, 8);
        map(60, 52, 12);
        for (unsigned i = 0; i < 8; ++i)
            map(72 + 12 * i, 64, 2);
        map(172, 164, 128);
        map(292, 284, 8);
        map(316, 292, 16);
        map(332, 308, 32);
        map(364, 340, 2);
        map(370, 346, 2);
        map(372, 348, static_cast<unsigned>(old.size() - 348));
        Bytes expected(result.data.size(), 0);
        for (std::size_t i = 0; i < origin.size(); ++i)
            if (origin[i] >= 0)
                expected[i] = old[origin[i]];
        put(expected, 8, (old.size() - 4) / 2 + 12, 4);
        put(expected, 12, (old_base - 4) / 2 + 12, 4);
        put(expected, 40, 1, 4);
        check(result.data == expected,
              "all upgraded bytes follow the field map, repeated words, zero-fill and tail move");
        const auto base = native_reference_layout(slice(old, 0, old_base));
        check(base.data == slice(expected, 0, old_base + 24),
              "base-only conversion retains declared full word count without reading attributes");
        const auto again = native_reference_layout(result.data);
        check(!again.upgraded && again.data == result.data,
              "an upgraded reference is recognized as current and never upgraded twice");
        const auto original_rows = parse_native(old), converted_rows = parse_native(result.data);
        const auto &input = original_rows[0]["reference_input"];
        auto relocated_links = original_rows[0]["links"];
        for (auto &link : relocated_links)
            link["offset"] = link["offset"].get<std::size_t>() + 24;
        check(relocated_links == converted_rows[0]["links"] &&
                  original_rows[0]["zero_padding"] == 8 && converted_rows[0]["zero_padding"] == 8 &&
                  bytesof(original_rows[0]["data"]) == slice(old, 0, old_base) &&
                  bytesof(input["layout"]["converted_record"]) == result.data,
              "record integration preserves original payload, complete attribute linkage and "
              "padding");
        check(input["status"] == "decoded" && input["transform"]["status"] == "computed" &&
                  input["transform"]["scale"] == 7 &&
                  input["transform"]["source_matrix_offset"] == 212 &&
                  input["transform"]["source_scale_offset"] == 284 &&
                  input["transform"]["matrix"] == identity,
              "legacy reference transform reads relocated matrix and scale, not modern source "
              "offsets");
    }
    auto run = [&](const Matrix3 &m, double scale) {
        const auto b = wire(m, scale), saved = b;
        const auto decoded = parse_native(b)[0]["reference_input"];
        check(decoded["status"] == "decoded" && decoded["layout"]["upgraded"] == false &&
                  b == saved && !decoded["layout"].contains("converted_record"),
              "modern references retain their source layout and bytes");
        return decoded["transform"];
    };
    for (double value : {0., -0., -3., 7.}) {
        const auto t = run(identity, value);
        check(t["status"] == "computed" && t["matrix"] == identity &&
                  t["scale"] == (value == 0 ? 1. : value) &&
                  t["zero_scale_default_applied"] == (value == 0) &&
                  t["scale_adjustment_applied"] == false,
              "only exact zero reference scales default to one");
    }
    auto m = identity;
    for (unsigned i = 0; i < 3; ++i)
        m[i][i] = 2;
    auto t = run(m, 3);
    const Matrix3 half{{{0.5, 0, 0}, {0, 0.5, 0}, {0, 0, 0.5}}};
    check(t["scale"] == 6 && t["matrix"] == half && t["rigid_scale"] == true &&
              t["column_normalized_matrix"] == identity,
          "native aliasing normalizes columns before the additional reciprocal matrix scaling");
    m = {{{0, -2, 0}, {2, 0, 0}, {0, 0, 2}}};
    t = run(m, -3);
    const Matrix3 rotated{{{0, -0.5, 0}, {0.5, 0, 0}, {0, 0, 0.5}}};
    check(t["matrix"] == rotated && t["scale"] == -6,
          "reference normalization preserves rotation placement and the sign of source scale");
    m = identity;
    m[0][0] = 2;
    m[1][1] = 3;
    m[2][2] = 4;
    t = run(m, 7);
    check(t["matrix"] == identity && t["scale"] == 7 && t["rigid_scale"] == false &&
              t["columns_orthogonal"] == true && t["axis_ratio"] == 0.5,
          "nonuniform axes still normalize in-place when rigid-scale predicate fails");
    m[0][0] = -2;
    m[1][1] = m[2][2] = 2;
    t = run(m, 7);
    auto reflection = identity;
    reflection[0][0] = -1;
    check(t["matrix"] == reflection && t["scale"] == 7 && t["rigid_scale"] == false &&
              t["normalized_determinant"] == -1,
          "negative determinant disallows scale extraction but preserves normalized reflection");
    m = identity;
    m[0][1] = 1;
    t = run(m, 7);
    check(t["columns_orthogonal"] == false && t["rigid_scale"] == false && t["scale"] == 7 &&
              std::abs(t["matrix"][0][1].get<double>() - std::sqrt(0.5)) < 1e-15,
          "sheared columns normalize without inventing an orthogonal matrix");
    t = run(Matrix3{}, 7);
    const Matrix3 zero_result{{{1, 1, 1}, {0, 0, 0}, {0, 0, 0}}};
    check(t["matrix"] == zero_result && t["scale"] == 7 && t["rigid_scale"] == false,
          "zero columns use native positive-X normalization fallback");
    for (double scale : {1 + 5e-11, 1 + 2e-10}) {
        m = identity;
        for (unsigned i = 0; i < 3; ++i)
            m[i][i] = scale;
        t = run(m, 7);
        check(t["scale_adjustment_applied"] == (scale > 1 + 1e-10),
              "scale extraction uses the native absolute near-one tolerance");
    }
    for (double axis : {0.999999999999, std::nextafter(0.999999999999, 1.)}) {
        m = identity;
        m[0][0] = axis;
        t = run(m, 7);
        check(t["rigid_scale"] == (axis > 0.999999999999),
              "axis ratio must strictly exceed the native threshold");
    }
    for (double bad :
         {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        t = run(identity, bad);
        check(t["status"] == "not_evaluated" && !t.contains("matrix") && !t.contains("scale"),
              "nonfinite reference scale cannot silently become a usable numeric result");
        m = identity;
        m[0][0] = bad;
        t = run(m, 7);
        check(t["status"] == "not_evaluated" && !t.contains("matrix"),
              "nonfinite source matrices retain bytes without a computed transform");
    }
    m = identity;
    m[0][0] = 1e200;
    t = run(m, 7);
    check(t["status"] == "not_evaluated" && !t.contains("matrix"),
          "native length overflow is reported rather than replaced by a different norm algorithm");
    for (unsigned offset : {8u, 12u, 370u}) {
        auto b = wire(identity, 1);
        put(b, offset, offset == 370 ? 4 : 185, offset == 370 ? 2 : 4);
        const auto result = native_reference_input(b);
        check(result["status"] == "invalid", "truncated declarations and entry arrays fail safely");
    }
    for (unsigned count : {0u, 1u, 3u, 4u, 5u, 2499u, 2500u, 2501u, 65535u}) {
        for (bool complete : {false, true}) {
            if (complete && count == 65535)
                continue;
            auto b = wire(identity, 1);
            if (complete)
                b.resize(372 + 16 * count);
            put(b, 8, (b.size() - 4) / 2, 4);
            put(b, 12, (b.size() - 4) / 2, 4);
            put(b, 370, count, 2);
            if (complete)
                for (unsigned i = 0; i < count; ++i) {
                    number(b, 372 + 16 * i, 1.25 + i);
                    number(b, 380 + 16 * i, -3.5 - 2 * i);
                }
            const bool accepted = count >= 4 && count <= 2500;
            const auto decoded = native_reference_input(b);
            check(decoded["status"] == (accepted && !complete ? "invalid" : "decoded"),
                  "only a loader-accepted clipping count requires complete point bytes");
            const auto &boundary = decoded["clipping"]["boundary"];
            check(boundary["source_point_count"] == count &&
                      boundary["loader_return_code"] ==
                          (accepted && !complete ? Json() : Json(count == 0 || accepted ? 0 : 1)) &&
                      boundary["loaded_point_count"] ==
                          (accepted && !complete ? Json() : Json(accepted ? count : 0)) &&
                      boundary["source_points_status"] ==
                          (complete || count == 0 ? "decoded" : "truncated"),
                  "clip loader selection and source array availability remain separate");
            if (complete && count) {
                check(boundary["source_points"].size() == count &&
                          boundary["source_points"][0] == Json::array({1.25, -3.5}) &&
                          boundary["source_points"][count - 1] ==
                              Json::array({0.25 + count, -1.5 - 2 * count}),
                      "two-dimensional clip vertices preserve source order without auto-closing");
            }
        }
    }
    for (auto range : {Point2{10, -5}, Point2{-5, 10}, Point2{3, 3}}) {
        for (unsigned mask : {0u, 0x400u, 0x800u, 0xc00u}) {
            auto b = wire(identity, 1);
            put(b, 64, 0x80401234u | mask, 4);
            number(b, 316, range[0]);
            number(b, 324, range[1]);
            const auto d = native_reference_input(b)["clipping"]["depths"];
            const bool clear = range[1] >= range[0];
            const unsigned effective = clear ? 0 : mask;
            check(d["depth_flags_cleared"] == clear &&
                      d["flags_after_depth_validation"] == (0x80401234u | effective) &&
                      d["local_interval"]["lower"] ==
                          ((effective & 0x400) ? range[1] : -4503599627370496.) &&
                      d["local_interval"]["upper"] ==
                          ((effective & 0x800) ? range[0] : 4503599627370495.),
                  "depth input clears both flags for reversed or equal intervals and uses native "
                  "defaults");
        }
    }
    const auto infinity = std::numeric_limits<double>::infinity();
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    for (auto range : {Point2{infinity, infinity}, Point2{infinity, -infinity}, Point2{nan, 0},
                       Point2{1, nan}}) {
        for (unsigned mask : {0u, 0xc00u}) {
            auto b = wire(identity, 1);
            put(b, 64, mask, 4);
            number(b, 316, range[0]);
            number(b, 324, range[1]);
            const auto clip = native_reference_input(b)["clipping"];
            const auto &d = clip["depths"];
            const bool clear = range[1] >= range[0];
            check(d["depth_flags_cleared"] == clear &&
                      d["local_interval"]["status"] ==
                          (clear || !mask ? "computed" : "not_evaluated") &&
                      Json::parse(clip.dump()) == clip,
                  "unordered and infinite depth values keep native flag branches and lossless IEEE "
                  "metadata");
        }
    }
    auto b = wire(identity, 1);
    b.resize(436, 0);
    put(b, 8, 216, 4);
    put(b, 12, 216, 4);
    put(b, 370, 4, 2);
    number(b, 372, nan);
    number(b, 380, -infinity);
    auto clip = native_reference_input(b)["clipping"];
    check(clip["status"] == "decoded" && clip["boundary"]["loader_status"] == "accepted" &&
              clip["boundary"]["all_components_finite"] == false &&
              clip["boundary"]["source_points"][0][0]["floating_point"] == "nan" &&
              clip["boundary"]["source_points"][0][1]["negative"] == true &&
              Json::parse(clip.dump()) == clip,
          "native clip copying does not reject NaN points, while metadata preserves their bits");
    put(b, 12, 184, 4);
    const auto truncated = native_reference_input(b);
    check(truncated["status"] == "invalid" &&
              truncated["clipping"]["boundary"]["loader_status"] == "truncated" &&
              truncated["clipping"]["boundary"]["source_points"].empty(),
          "attribute tail bytes cannot satisfy missing clipping vertices in the declared base");
    Bytes old(348 + 5 * 16, 0);
    put(old, 4, 13, 2);
    put(old, 8, (old.size() - 4) / 2, 4);
    put(old, 12, (old.size() - 4) / 2, 4);
    put(old, 346, 5, 2);
    put(old, 56, 0xc00, 4);
    number(old, 292, 9);
    number(old, 300, -2);
    for (unsigned i = 0; i < 5; ++i) {
        number(old, 348 + 16 * i, i + 0.5);
        number(old, 356 + 16 * i, -static_cast<double>(i) - 0.75);
    }
    clip = native_reference_input(old)["clipping"];
    check(
        clip["boundary"]["loaded_point_count"] == 5 &&
            clip["boundary"]["points_source_offset"] == 348 &&
            clip["depths"]["flags_source_offset"] == 56 &&
            clip["depths"]["upper_source_offset"] == 292 &&
            clip["depths"]["lower_source_offset"] == 300 &&
            clip["depths"]["local_interval"] ==
                Json{{"status", "computed"}, {"lower", -2.}, {"upper", 9.}},
        "legacy clipping vertices, flags and depths use their own source offsets after conversion");
    return checks;
}
