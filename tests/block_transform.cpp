#include "internal.hpp"
#include <cstring>

unsigned block_transform_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    const Matrix4 identity{{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
    auto wire = [&](const Matrix4 &m) {
        Bytes b(260, 0);
        std::size_t offset = 164;
        auto put = [&](double value) {
            std::uint64_t bits;
            std::memcpy(&bits, &value, sizeof bits);
            for (unsigned i = 0; i < 8; ++i)
                b[offset++] = static_cast<std::uint8_t>(bits >> (i * 8));
        };
        for (unsigned row = 0; row < 3; ++row)
            for (unsigned col = 0; col < 3; ++col)
                put(m[row][col]);
        for (unsigned row = 0; row < 3; ++row)
            put(m[row][3]);
        return b;
    };
    auto determinant = [](const Matrix4 &m) {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
               m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    };
    for (double scale : {1e-200, 1.0, 7.0, 1e200}) {
        auto m = identity;
        m[0][0] = scale;
        m[1][1] = -scale;
        m[2][2] = scale;
        m[0][3] = 19;
        m[1][3] = -23;
        m[2][3] = 42;
        const auto b = wire(m), copy = b;
        auto result = native_block_transform(b);
        check(result["repair"] == "unchanged" && result["source_matrix"] == m &&
                  result["matrix"] == m && b == copy && result["repaired_column_indices"].empty(),
              "uniform nonsingular scales and reflection preserve source coefficients and "
              "translation");
    }
    auto m = identity;
    m[2][2] = 1e-10;
    auto result = native_block_transform(wire(m));
    check(result["repair"] == "unchanged" && result["matrix"] == m,
          "a small determinant accepted by orthogonal inverse factors must not trigger "
          "short-column repair");
    for (double factor : {0.99, 1.01}) {
        m = identity;
        m[2][2] = factor * 1e-13;
        result = native_block_transform(wire(m));
        check(result["repair"] == (factor > 1 ? "unchanged" : "short_columns") &&
                  result["matrix"] == (factor > 1 ? m : identity),
              "orthogonal inverse factors preserve the strict relative squared-length threshold");
    }
    for (double value : {0.000999, 0.001001}) {
        m = identity;
        m[2][2] = 0;
        m[0][2] = value;
        result = native_block_transform(wire(m));
        check(result["repair"] == (value < .001 ? "short_columns" : "rank_augmentation") &&
                  result["repaired_column_indices"] ==
                      (value < .001 ? Json::array({2}) : Json::array()),
              "column repair compares squared length with one-millionth before rank augmentation");
    }
    for (double second : {2.0, 3.0}) {
        m = identity;
        m[0][0] = 2;
        m[1][1] = second;
        m[2][2] = 0;
        m[2][3] = 77;
        result = native_block_transform(wire(m));
        auto expected = m;
        expected[2][2] = second == 2 ? 2 : 1;
        check(result["repair"] == "short_columns" && result["matrix"] == expected &&
                  result["source_matrix"] == m &&
                  result["repaired_column_indices"] == Json::array({2}),
              "missing third axis keeps translation and uses common scale only when the other "
              "lengths equal");
    }
    for (unsigned missing = 0; missing < 3; ++missing) {
        m = identity;
        m[missing][missing] = 0;
        result = native_block_transform(wire(m));
        check(result["matrix"] == identity && result["repair"] == "short_columns" &&
                  result["repaired_column_indices"] == Json::array({missing}),
              "cyclic column repair reconstructs each missing coordinate axis with the native "
              "orientation");
    }
    m = identity;
    m[0][0] = m[1][1] = m[2][2] = 0;
    m[0][3] = 99;
    result = native_block_transform(wire(m));
    auto expected = identity;
    expected[0][3] = 99;
    check(result["repair"] == "rank_augmentation" && result["rank_before_augmentation"] == 0 &&
              result["matrix"] == expected && result["rank_factors_converged"] == true &&
              result["repaired_column_indices"] == Json::array({0, 1, 2}),
          "zero matrix takes sequential column repair and native rank-zero identity completion");
    // All three columns are nonzero but lie in the XY plane. Rank completion
    // must preserve both existing coordinate rows and add the missing normal.
    m = {{{2, 0, 1, 5}, {0, 3, 1, 6}, {0, 0, 0, 7}, {0, 0, 0, 1}}};
    result = native_block_transform(wire(m));
    auto effective = result["matrix"].get<Matrix4>();
    check(result["repair"] == "rank_augmentation" && result["rank_before_augmentation"] == 2 &&
              result["repaired_column_indices"].empty() && effective[0] == m[0] &&
              effective[1] == m[1] && effective[2][3] == 7 && determinant(effective) > 0,
          "rank-two dependent nonzero columns preserve the existing plane and add a positive "
          "missing dimension");
    m = {{{1, 1, 1, 0}, {2, 2, 2, 0}, {3, 3, 3, 0}, {0, 0, 0, 1}}};
    result = native_block_transform(wire(m));
    effective = result["matrix"].get<Matrix4>();
    check(result["repair"] == "rank_augmentation" && result["rank_before_augmentation"] == 1 &&
              result["rank_factors_converged"] == true && std::abs(determinant(effective)) > 1e-6 &&
              result["source_matrix"] == m,
          "rank-one dependent columns run the native triad call-site branch and preserve the "
          "source matrix");
    for (unsigned i = 0; i < 40; ++i) {
        m = identity;
        m[0][0] = 1 + i;
        m[1][1] = 2 + i;
        m[2][2] = 3 + i;
        m[0][1] = .25 * i;
        m[0][2] = -.75 * i;
        m[1][2] = .5 * i;
        result = native_block_transform(wire(m));
        check(
            result["repair"] == "unchanged" && result["matrix"] == m,
            "nonsingular shear and nonuniform scale remain exact rather than being orthogonalized");
    }
    for (unsigned bad = 0; bad < 3; ++bad) {
        m = identity;
        if (bad == 1)
            m[0][0] = std::numeric_limits<double>::infinity();
        if (bad == 2)
            m[1][3] = std::numeric_limits<double>::quiet_NaN();
        auto b = wire(m);
        if (!bad)
            b.resize(259);
        bool rejected = false;
        try {
            native_block_transform(b);
        } catch (const std::exception &) {
            rejected = true;
        }
        check(
            rejected,
            "truncated and nonfinite block transforms do not produce a successful repaired matrix");
    }
    auto b = wire(identity);
    b[4] = 62;
    b[8] = b[12] = 128;
    auto records = parse_native(b);
    check(records.size() == 1 && records[0]["block_transform"]["status"] == "resolved" &&
              records[0]["block_transform"]["matrix"] == identity &&
              bytesof(records[0]["data"]) == b,
          "native parsing caches the transform independently of block definition resolution and "
          "preserves source bytes");
    b.resize(236);
    b[8] = b[12] = 116;
    records = parse_native(b);
    check(records.size() == 1 && records[0]["block_transform"]["status"] == "invalid" &&
              records[0]["block_transform"].contains("error") && bytesof(records[0]["data"]) == b,
          "missing translation retains the native record and reports a transform error instead of "
          "failing the file");
    return checks;
}
