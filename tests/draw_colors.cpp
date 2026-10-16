#include "internal.hpp"
#include <p3d/color_resources.hpp>
#include <thread>

unsigned draw_colors_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool value, const char *why) {
        require(value, why);
        ++checks;
    };
    auto put = [](Bytes &b, unsigned offset, std::uint32_t value) {
        for (unsigned j = 0; j < 4; ++j)
            b[offset + j] = std::uint8_t(value >> (8 * j));
    };
    auto state = [&](std::uint32_t primary, std::uint32_t secondary, unsigned flags = 0) {
        Bytes b(91);
        for (unsigned i = 0; i < b.size(); ++i)
            b[i] = std::uint8_t(i * 13 + 7);
        b[1] = std::uint8_t(flags);
        put(b, 32, primary);
        put(b, 44, secondary);
        put(b, 40, 0xe1918171);
        put(b, 52, 0xd1b1a191);
        return b;
    };
    NativeDrawColorContext context;
    std::array<std::uint32_t, 256> palette;
    for (unsigned i = 0; i < 256; ++i)
        palette[i] = i | ((255 - i) << 8) | ((i ^ 0x55) << 16);
    context.draw_palette = palette;
    context.extended_colors = std::vector<NativeRgb>{{17, 39, 83}, {231, 177, 25}};
    // Explicit expected results, independent of implementation control flow.
    // Upper four bits alias in the extended index; markers still use full IDs.
    const std::vector<std::pair<std::uint32_t, std::optional<std::uint32_t>>> cases{
        {0, 0x55ff00},          {1, 0x54fe01},          {255, 0xaa00ff},
        {256, 0x532711},        {0x1ff, 0x532711},      {512, 0x19b1e7},
        {0x20b, 0x19b1e7},      {0x307, 0x52f807},      {0x10000101, 0x532711},
        {0xf0000209, 0x19b1e7}, {0x10000007, 0x52f807}, {0xfffffffd, 0xa802fd},
        {0xfffffffe, 0xab01fe}, {0xffffffff, 0xaa00ff}, {0xffffff00, std::nullopt}};
    for (unsigned flags = 0; flags < 256; ++flags)
        for (unsigned i = 0; i < cases.size(); ++i) {
            const auto &a = cases[i];
            const auto &b = cases[(i + 5) % cases.size()];
            auto source = state(a.first, b.first, flags), expected = source;
            const auto before = source;
            auto result = prepare_native_draw_colors(source, context);
            const auto primary = (flags & 4) ? std::optional<std::uint32_t>(0xe1918171) : a.second;
            const auto secondary =
                (flags & 8) ? std::optional<std::uint32_t>(0xd1b1a191) : b.second;
            if (primary && !(flags & 4)) {
                put(expected, 40, *primary);
                expected[1] |= 4;
            }
            if (secondary && !(flags & 8)) {
                put(expected, 52, *secondary);
                expected[1] |= 8;
            }
            check(
                result.at("status") == "prepared" &&
                    bytesof(result.at("prepared_state")) == expected && source == before &&
                    result.at("channels")[0].at("status") == (primary ? "resolved" : "unchanged") &&
                    result.at("channels")[1].at("status") == (secondary ? "resolved" : "unchanged"),
                "independent explicit color table, both channels, every flag and full byte oracle");
        }
    auto unknown = context;
    unknown.extended_colors.reset();
    auto input = state(0x107, 7);
    auto result = prepare_native_draw_colors(input, unknown);
    auto partial = input;
    put(partial, 52, 0x52f807);
    partial[1] |= 8;
    check(result.at("status") == "partial" && result.at("prepared_state").is_null() &&
              result.at("channels")[0].at("status") == "unresolved" &&
              result.at("channels")[1].at("status") == "resolved" &&
              bytesof(result.at("partial_state")) == partial,
          "unknown higher-priority table never becomes low-byte palette fallback");
    check(prepare_native_draw_colors(state(0x10000007, 7), unknown).at("status") == "partial",
          "masked zero does not bypass owning-file loading requirement");
    auto absent = context;
    absent.extended_colors = std::vector<NativeRgb>{};
    result = prepare_native_draw_colors(input, absent);
    check(result.at("status") == "prepared" &&
              result.at("channels")[0].at("packed_rgb") == 0x52f807,
          "confirmed empty extended table falls back to actual draw palette");
    absent.draw_palette.reset();
    result = prepare_native_draw_colors(input, absent);
    check(result.at("status") == "partial" && result.at("prepared_state").is_null(),
          "missing draw palette is not silently replaced with module defaults");
    absent.draw_palette_confirmed_absent = true;
    result = prepare_native_draw_colors(input, absent);
    check(result.at("status") == "prepared" && result.at("channels")[0].at("packed_rgb") == 0 &&
              result.at("channels")[1].at("packed_rgb") == 0,
          "known null ordinary context palette returns native zero");
    auto ext_only = context;
    ext_only.draw_palette.reset();
    ext_only.draw_palette_confirmed_absent = true;
    result = prepare_native_draw_colors(state(256, 512), ext_only);
    check(result.at("status") == "prepared" &&
              result.at("channels")[0].at("packed_rgb") == 0x532711 &&
              result.at("channels")[1].at("packed_rgb") == 0x19b1e7,
          "found extended colors do not consume context palette");
    auto packed = context;
    packed.draw_palette->at(7) = 0xab123456;
    result = prepare_native_draw_colors(state(7, 0x307), packed);
    check(result.at("status") == "prepared" &&
              result.at("channels")[0].at("packed_rgb") == 0xab123456 &&
              result.at("channels")[1].at("unassigned_high_byte") == 0xab,
          "native palette DWORD high byte is preserved without being interpreted as alpha");
    auto contradictory = context;
    contradictory.draw_palette_confirmed_absent = true;
    check(prepare_native_draw_colors(state(7, 8), contradictory).at("status") == "partial",
          "contradictory consumed context is rejected");
    check(prepare_native_draw_colors(state(256, 512), contradictory).at("status") == "prepared",
          "irrelevant contradictory palette does not block resolved owner colors");
    result = prepare_native_draw_colors(state(0xffffff00, 0xffffff00));
    check(result.at("status") == "prepared" &&
              result.at("channels")[0].at("status") == "unchanged" &&
              !result.at("channels")[0].contains("rgb"),
          "bypass preserves unresolved marker without publishing a false RGB");
    result = prepare_native_draw_colors(state(0xffffffff, 0x100, 12));
    check(result.at("status") == "prepared" &&
              result.at("channels")[0].at("unassigned_high_byte") == 0xe1 &&
              result.at("channels")[1].at("unassigned_high_byte") == 0xd1,
          "already resolved channels preserve all packed bits without inspecting contexts");
    for (unsigned size = 0; size < 56; ++size) {
        check(prepare_native_draw_colors(Bytes(size), context).at("status") == "unresolved",
              "truncated native style prefix does not publish a prepared state");
    }
    auto minimum = state(256, 512);
    minimum.resize(56);
    check(bytesof(prepare_native_draw_colors(minimum, context).at("prepared_state")).size() == 56,
          "complete minimum color prefix accepted without appending fields");
    auto maximum = context;
    maximum.extended_colors->resize(0xfffff);
    maximum.extended_colors->back() = {249, 127, 3};
    result = prepare_native_draw_colors(state(0x0fffffff, 0xfffffffd), maximum);
    check(result.at("channels")[0].at("packed_rgb") == 0x037ff9 &&
              result.at("channels")[1].at("packed_rgb") == 0x037ff9 &&
              result.at("channels")[1].at("extended_ordinal") == 0xffffe,
          "largest masked source ID and upper-bit alias use last native loaded entry");
    const auto expected = prepare_native_draw_colors(input, context);
    std::array<Json, 4> threaded;
    std::array<std::thread, 4> threads;
    for (unsigned i = 0; i < threads.size(); ++i)
        threads[i] =
            std::thread([&, i] { threaded[i] = prepare_native_draw_colors(input, context); });
    for (auto &t : threads)
        t.join();
    check(
        std::all_of(threaded.begin(), threaded.end(), [&](const Json &v) { return v == expected; }),
        "parallel immutable draw color conversion has no shared cache mutation");
    return checks;
}
