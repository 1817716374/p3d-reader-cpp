#include "internal.hpp"
#include <p3d/color_resources.hpp>
#include <thread>

unsigned model_palette_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool b, const char *why) {
        require(b, why);
        ++checks;
    };
    auto put = [](Bytes &b, std::size_t at, const auto &v) {
        std::memcpy(b.data() + at, &v, sizeof(v));
    };
    auto link = [&](unsigned key, const std::vector<double> &values) {
        Bytes b(8 + 8 * values.size());
        put(b, 0, std::uint16_t(key));
        put(b, 2, std::uint16_t(0x3781));
        put(b, 4, std::uint32_t(values.size()));
        for (std::size_t i = 0; i < values.size(); ++i)
            put(b, 8 + 8 * i, values[i]);
        return Json{{"app", 0x56d5}, {"header", 0x1000}, {"payload", rawbytes(b)}};
    };
    Bytes body(40, 0);
    put(body, 4, std::uint16_t(13));
    auto record = [&](Json links) {
        return Json{{"data", rawbytes(body)}, {"links", std::move(links)}};
    };
    const auto empty = read_native_reference_palette_adjustment(record(Json::array()));
    check(empty.at("status") == "decoded" && empty.at("state").at("flags") == 0 &&
              empty.at("state").at("value_reduction_percent") == 0,
          "native reset before reference link load initializes all four words");
    const auto nan = std::numeric_limits<double>::quiet_NaN(),
               inf = std::numeric_limits<double>::infinity();
    const std::vector<double> values{
        -inf, -2147483648.75, -65537., -65536., -1.,           -.5,         0.,  .5, 99.9,
        100., 101.,           65536.,  65599.,  2147483647.75, 2147483648., inf, nan};
    const std::vector<std::int32_t> converted{
        INT32_MIN, INT32_MIN, -65537, -65536, -1,        0,         0,         0,        99,
        100,       101,       65536,  65599,  INT32_MAX, INT32_MIN, INT32_MIN, INT32_MIN};
    auto signed_low = [](std::int32_t x) {
        const auto low = std::uint32_t(x) & 65535u;
        return low < 32768 ? std::int32_t(low) : std::int32_t(low) - 65536;
    };
    for (std::size_t ai = 0; ai < values.size(); ++ai)
        for (std::size_t bi = 0; bi < values.size(); ++bi) {
            const auto a = values[ai], b = values[bi];
            const auto ia = converted[ai], ib = converted[bi];
            const auto wa = std::uint32_t(ia) & 65535u, wb = std::uint32_t(ib) & 65535u;
            const bool accepted = wa <= 100 && wb <= 100;
            const auto r =
                read_native_reference_palette_adjustment(record(Json::array({link(9, {a, b})})));
            check(r.at("status") == "decoded" &&
                      r.at("state").at("value_reduction_percent") == int(accepted ? wa : 0) &&
                      r.at("state").at("saturation_reduction_percent") == int(accepted ? wb : 0) &&
                      r.at("links")[0].at("converted_signed_32") == Json::array({ia, ib}) &&
                      r.at("links")[0].at("status") == (accepted ? "applied" : "ignored"),
                  "explicit native integer conversion table and unsigned low-word predicate, "
                  "including NaN and "
                  "overflow");
        }
    for (std::size_t fi = 0; fi < values.size(); ++fi) {
        const auto flags = values[fi];
        const auto f = converted[fi];
        const bool accepted = (std::uint32_t(f) & 3) != 0;
        const auto r = read_native_reference_palette_adjustment(
            record(Json::array({link(11, {-65537., 65599., -65538., flags})})));
        check(r.at("status") == "decoded" &&
                  r.at("state").at("flags") == (accepted ? signed_low(f) : 0) &&
                  r.at("state").at("hue_degrees") == (accepted ? -2 : 0) &&
                  r.at("state").at("value_reduction_percent") == (accepted ? -1 : 0),
              "full adjustment selects flags before writing signed native words");
    }
    const auto sequence =
        record(Json::array({link(11, {20, 30, 120, 3}), link(9, {70, 80}), link(11, {1, 2, 3, 4}),
                            link(9, {101, 0}), link(11, {1, 2, 3})}));
    const auto seq = read_native_reference_palette_adjustment(sequence);
    check(seq.at("links")[0].at("conversion_order") == Json::array({1, 2, 3, 0}) &&
              seq.at("links")[2].at("conversion_order") == Json::array({1, 2, 3}) &&
              seq.at("links")[2].at("converted_signed_32")[0].is_null(),
          "native conversion order skips the first double when full adjustment flags are clear");
    check(seq.at("state").at("value_reduction_percent") == 70 &&
              seq.at("state").at("saturation_reduction_percent") == 80 &&
              seq.at("state").at("hue_degrees") == 120 && seq.at("state").at("flags") == 3,
          "ordered two-word link preserves hue/flags and ignored later links never replace "
          "accepted state");
    auto wrong = link(11, {50, 50, 60, 3});
    wrong["header"] = 0;
    check(read_native_reference_palette_adjustment(record(Json::array({wrong}))).at("source") ==
              "initial_zero_state",
          "non-user linkage is not a persisted numeric adjustment");
    auto truncated = link(11, {1, 2, 3, 3});
    auto raw = bytesof(truncated.at("payload"));
    raw.resize(38);
    truncated["payload"] = rawbytes(raw);
    auto bad = read_native_reference_palette_adjustment(
        record(Json::array({link(9, {20, 30}), truncated})));
    check(bad.at("status") == "not_evaluated" && !bad.contains("state") &&
              bad.at("partial_state").at("value_reduction_percent") == 20,
          "unsafe payload retains prefix state without publishing a complete result");
    auto unsupported = link(11, {});
    raw = bytesof(unsupported.at("payload"));
    put(raw, 4, std::uint32_t(0xffffffff));
    unsupported["payload"] = rawbytes(raw);
    check(
        read_native_reference_palette_adjustment(record(Json::array({unsupported}))).at("status") ==
            "decoded",
        "wrong native count ignored before value access");
    const auto payload = bytesof(link(11, {20, 30, 120, 3}).at("payload"));
    Bytes saved(372 + 4 + payload.size(), 0);
    put(saved, 4, std::uint16_t(13));
    put(saved, 8, std::uint32_t((saved.size() - 4) / 2));
    put(saved, 12, std::uint32_t(184));
    put(saved, 160, std::uint32_t(1));
    put(saved, 372, std::uint16_t(0x1000 + ((payload.size() + 4) / 2) - 1));
    put(saved, 374, std::uint16_t(0x56d5));
    std::copy(payload.begin(), payload.end(), saved.begin() + 376);
    const auto parsed = parse_native(saved);
    check(parsed.size() == 1 && parsed[0]
                                        .at("reference_input")
                                        .at("palette_adjustment")
                                        .at("state")
                                        .at("hue_degrees") == 120,
          "normal native record parser automatically exposes persisted palette adjustment");
    std::array<std::uint32_t, 256> base;
    for (unsigned i = 0; i < 256; ++i)
        base[i] = 0xa5000000u | i | (i << 8) | (i << 16);
    const NativeModelColorAdjustment zero{};
    auto prepared = prepare_native_model_palette(base, {zero}, {}, true);
    check(prepared.at("status") == "prepared" &&
              prepared.at("native_storage_action") == "share_base_palette" &&
              prepared.at("packed_colors") == Json(base),
          "unchanged model reuses native base palette and preserves all DWORD bits");
    const auto setting_independent =
        prepare_native_model_palette(base, {zero, {0, 0, 120, 1}, zero}, {}, true);
    check(setting_independent.at("status") == "prepared" &&
              setting_independent.at("packed_colors") == Json(base) &&
              setting_independent.at("selection") == "both_native_setting_branches_share_base",
          "complete ancestry without effective adjustments needs no unknown setting value");
    check(prepare_native_model_palette(base, {{0, 0, 120, 1}}, false, true).at("packed_colors") ==
              Json(base),
          "absolute bit alone with zero reductions and no hue replacement does not trigger palette "
          "build");
    for (int hue : {0, 60, 120, 180, 240, 300, 360, -1})
        for (int val : {0, 50, 100}) {
            NativeModelColorAdjustment s{std::int16_t(100 - val), 0, std::int16_t(hue), 3};
            auto r = prepare_native_model_palette(base, {s}, true, true);
            const auto top = unsigned((val * 255 + 50) / 100);
            std::uint32_t expected = 0;
            switch (hue) {
            case 0:
            case 360:
                expected = top;
                break;
            case 60:
                expected = top | (top << 8);
                break;
            case 120:
                expected = top << 8;
                break;
            case 180:
                expected = (top << 8) | (top << 16);
                break;
            case 240:
                expected = top << 16;
                break;
            case 300:
                expected = top | (top << 16);
                break;
            default:
                expected = top | (top << 8) | (top << 16);
                break;
            }
            bool all = r.at("status") == "prepared";
            if (all)
                for (unsigned i = 0; i < 255; ++i)
                    all = all && r.at("packed_colors")[i] == expected;
            check(all && r.at("packed_colors")[255] == 0xffffffu,
                  "constant HSV primary/secondary/gray colors and background exclusion use "
                  "independent exact RGB oracle");
        }
    prepared = prepare_native_model_palette(base, {{50, 0, 0, 0}}, true, true);
    check(prepared.at("status") == "prepared" && prepared.at("packed_colors")[0] == 0 &&
              prepared.at("packed_colors")[254] == 0x808080u &&
              prepared.at("packed_colors")[255] == 0xffffffu,
          "relative value scale uses rounded integer HSV and rebuild clears high byte");
    // Independent integer chroma/match oracle for the normal HSV range. Native
    // floating-point half ties can differ by one byte from exact rational math.
    auto exact_rgb = [](unsigned h, unsigned s, unsigned v) {
        h %= 360;
        const std::int64_t c = 255ll * v * s, x = c * (60 - std::abs(int(h % 120) - 60));
        const auto m = 255ll * v * 6000 - c * 60;
        const std::array<std::array<std::int64_t, 3>, 6> sectors{{{c * 60, x, 0},
                                                                  {x, c * 60, 0},
                                                                  {0, c * 60, x},
                                                                  {0, x, c * 60},
                                                                  {x, 0, c * 60},
                                                                  {c * 60, 0, x}}};
        NativeRgb out{};
        for (unsigned i = 0; i < 3; ++i)
            out[i] = std::uint8_t((sectors[h / 60][i] + m + 300000) / 600000);
        return out;
    };
    auto varied = base;
    for (unsigned i = 0; i < 256; ++i)
        varied[i] =
            ((i * 47) & 255) | (((i * 101 + 11) & 255) << 8) | (((i * 17 + 99) & 255) << 16);
    for (unsigned absolute = 0; absolute < 2; ++absolute)
        for (int vr : {0, 50, 100})
            for (int sr : {0, 50, 100}) {
                if (!vr && !sr)
                    continue;
                const auto r = prepare_native_model_palette(
                    varied, {{std::int16_t(vr), std::int16_t(sr), 0, std::int16_t(absolute)}}, true,
                    true);
                bool close = r.at("status") == "prepared";
                for (unsigned i = 0; i < 255 && close; ++i) {
                    const auto hsv =
                        native_color_hsv({std::uint8_t(varied[i]), std::uint8_t(varied[i] >> 8),
                                          std::uint8_t(varied[i] >> 16)});
                    const auto v =
                        absolute ? unsigned(100 - vr) : (hsv[2] * unsigned(100 - vr) + 50) / 100;
                    const auto s =
                        absolute ? unsigned(100 - sr) : (hsv[1] * unsigned(100 - sr) + 50) / 100;
                    const auto expected = exact_rgb(hsv[0], s, v);
                    const auto actual = r.at("packed_colors")[i].get<std::uint32_t>();
                    for (unsigned j = 0; j < 3; ++j)
                        close = close &&
                                std::abs(int((actual >> (8 * j)) & 255) - int(expected[j])) <= 1;
                }
                check(close && r.at("packed_colors")[255] == varied[255],
                      "varied RGB and relative/absolute adjustments agree with independent integer "
                      "chroma oracle within native half-tie rounding");
            }
    const auto forced = prepare_native_model_palette(base, {{0, 100, 0, 2}}, true, true);
    check(forced.at("packed_colors")[254] == 0x9999ffu,
          "fixed hue imposes native minimum saturation forty rather than one");
    const auto constant_low_saturation =
        prepare_native_model_palette(base, {{0, 75, 0, 3}}, true, true);
    check(constant_low_saturation.at("packed_colors")[254] == 0xbfbfffu,
          "combined absolute and hue mode uses constant saturation without the relative branch's "
          "forty minimum");
    const std::vector<NativeModelColorAdjustment> chain{
        {50, 20, 0, 0}, {20, 50, 120, 2}, {0, 0, 240, 2}};
    prepared = prepare_native_model_palette(base, chain, true, true);
    check(prepared.at("effective_parameters").at("value_scale").at("value") == .4 &&
              prepared.at("effective_parameters").at("saturation_scale").at("value") == .4 &&
              prepared.at("effective_parameters").at("hue_degrees") == 240,
          "actual ancestry multiplies factors in native order and highest selected hue wins");
    auto root = prepare_native_model_palette(base, chain, false, true);
    check(root.at("effective_parameters").at("value_scale").at("value") == 1. &&
              root.at("selected_ancestors").size() == 1,
          "root-only setting ignores descendant adjustment values");
    check(prepare_native_model_palette(base, chain, {}, true).at("status") == "not_evaluated" &&
              prepare_native_model_palette(base, chain, true, false).at("status") ==
                  "not_evaluated",
          "missing native setting or incomplete actual ancestry is not guessed");
    check(prepare_native_model_palette(base, {{0, 0, 400, 3}}, true, true).at("status") ==
              "not_evaluated",
          "invalid constant hue leaves native temporary uninitialized and cannot publish guessed "
          "colors");
    auto unchanged = prepare_native_model_palette(base, {{0, 0, 400, 2}}, true, true);
    check(unchanged.at("status") == "prepared" && unchanged.at("packed_colors")[150] == 0x969696u,
          "invalid per-color hue sector leaves initialized original RGB untouched");
    std::array<Json, 4> results;
    std::array<std::thread, 4> threads;
    for (unsigned i = 0; i < 4; ++i)
        threads[i] = std::thread(
            [&, i] { results[i] = prepare_native_model_palette(base, chain, true, true); });
    for (auto &t : threads)
        t.join();
    check(std::all_of(results.begin(), results.end(), [&](const Json &v) { return v == prepared; }),
          "parallel palette building is deterministic with immutable source state");
    return checks;
}
