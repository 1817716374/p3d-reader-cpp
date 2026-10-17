#include "internal.hpp"
#include "decode_budget.hpp"
#include <p3d/legacy_graphics.hpp>

unsigned legacy_graphics_type_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        require(ok, why);
        ++checks;
    };
    auto put = [](Bytes &b, auto v) {
        const auto p = b.size();
        b.resize(p + sizeof(v));
        std::memcpy(b.data() + p, &v, sizeof(v));
    };
    auto word = [&](Bytes &b, std::int32_t v) { put(b, v); };
    auto numbers = [&](Bytes &b, unsigned n, double start = 1.) {
        for (unsigned i = 0; i < n; ++i)
            put(b, start + i);
    };
    auto begin = [&](unsigned n = 1) {
        Bytes b;
        put(b, 3.1);
        put(b, std::uint32_t(n));
        return b;
    };
    auto entry = [&](Bytes &b, unsigned kind) {
        put(b, std::uint32_t(kind));
        b.resize(b.size() + 56);
    };
    auto end = [&](Bytes b) {
        b.resize(b.size() + 40);
        return b;
    };
    auto decode = [&](const Bytes &payload, unsigned kind) {
        auto b = begin();
        entry(b, kind);
        b.insert(b.end(), payload.begin(), payload.end());
        return decode_legacy_graphics(end(b));
    };
    auto geometry = [&](const Bytes &payload, unsigned kind) {
        auto out = decode(payload, kind);
        require(out.at("status") == "parsed", out.value("reason", "legacy type parse failed"));
        return out.at("entries")[0].at("geometry");
    };
    auto curves = [&](Bytes &b, unsigned boundary) {
        word(b, boundary);
        word(b, 0);
    };
    auto spline = [&](int rational, int closed, int declared) {
        Bytes b;
        for (int value : {4, 7, rational, 11, 13, 17, closed, declared, 2, 19, 3})
            word(b, value);
        numbers(b, 6, 20.);
        numbers(b, closed ? 7 : 5, 100.);
        if (rational)
            numbers(b, 2, .25);
        return b;
    };
    for (int closed : {0, 1, -2}) {
        const auto g = geometry(spline(2, closed, 9999), 1);
        check(g.at("kind") == "bspline_curve" && g.at("curve_type") == 7 &&
                  g.at("curve_display") == 11 && g.at("polygon_display") == 13 &&
                  g.at("rules_by_length") == 17 && g.at("num_rules") == 19 &&
                  g.at("closed_stored_int32") == closed && g.at("rational_stored_int32") == 2,
              "legacy spline scalar order follows the archive rather than the runtime structure");
        check(g.at("declared_num_knots") == 9999 &&
                  g.at("serialized_knot_count") == (closed ? 7 : 5) &&
                  g.at("poles")[1] == Json::array({23., 24., 25.}) &&
                  g.at("weights") == Json::array({.25, 1.25}) &&
                  g.at("pole_coordinates") == "weighted_xyz",
              "native allocated knot count and weighted poles are independent of the declared "
              "numKnots member");
    }
    const auto unweighted = geometry(spline(0, 0, -1), 1);
    check(unweighted.at("weights").empty() && unweighted.at("pole_coordinates") == "cartesian_xyz",
          "a nonrational spline does not invent a saved weight array");
    auto overflow = Bytes{};
    for (int value : {4, 0, 0, 0, 0, 0, 1, 123, 1, 0, std::numeric_limits<int>::min()})
        word(overflow, value);
    numbers(overflow, 3);
    check(geometry(overflow, 1).at("serialized_knot_count") == 0,
          "native DWORD knot-count wrap is preserved without C++ signed overflow or a huge "
          "allocation");
    auto broken_spline = spline(1, 1, 9999);
    broken_spline.pop_back();
    check(decode(broken_spline, 1).at("status") == "not_parsed",
          "a truncated weight array cannot consume the container footer");

    std::vector<Bytes> solids;
    for (unsigned type = 1; type <= 9; ++type) {
        Bytes b;
        word(b, type);
        switch (type) {
        case 1:
            numbers(b, 12);
            break;
        case 2:
            numbers(b, 14);
            break;
        case 3:
            numbers(b, 16);
            break;
        case 4:
            numbers(b, 14);
            break;
        case 5:
            curves(b, 25);
            numbers(b, 3, 51.);
            break;
        case 6:
            curves(b, 26);
            numbers(b, 7, 61.);
            break;
        case 7:
            word(b, 2);
            curves(b, 27);
            curves(b, 28);
            break;
        case 8:
            curves(b, 29);
            curves(b, 30);
            break;
        case 9:
            curves(b, 31);
            curves(b, 32);
            curves(b, 33);
            break;
        }
        b.push_back(2);
        if (type == 6)
            put(b, std::uint32_t(0xffffffff));
        solids.push_back(b);
        const auto g = geometry(b, 6);
        check(g.at("solid_type_code") == type && g.at("capped") == true &&
                  g.at("capped_stored_byte") == 2 && g.at("source_byte_count") == b.size(),
              "every identified solid code consumes its own parameter layout and cap byte");
    }
    const auto cone = geometry(solids[1], 6);
    check(cone.at("radius_a") == 13. && cone.at("radius_b") == 14. &&
              cone.at("vector90") == Json::array({10., 11., 12.}),
          "legacy cone serializes radius A before B despite the opposite runtime field order");
    const auto sphere = geometry(solids[3], 6);
    check(sphere.at("origin") == Json::array({1., 2., 3.}) &&
              sphere.at("vector_x") == Json::array({4., 5., 6.}) &&
              sphere.at("vector_z") == Json::array({10., 11., 12.}) &&
              sphere.at("start_latitude") == 13. && sphere.at("latitude_sweep") == 14.,
          "sphere transform storage is origin and columns rather than four matrix rows");
    check(geometry(solids[4], 6).at("base_curves").at("boundary_code") == 25 &&
              geometry(solids[4], 6).at("extrusion_vector") == Json::array({51., 52., 53.}) &&
              geometry(solids[5], 6).at("num_v_rules_stored_uint32") == 0xffffffffu &&
              geometry(solids[6], 6).at("sections")[1].at("boundary_code") == 28 &&
              geometry(solids[7], 6).at("path_curves").at("boundary_code") == 30 &&
              geometry(solids[8], 6).at("guide_curves").at("boundary_code") == 33,
          "swept solid profiles, paths, ordered sections, guides and low DWORD rule count remain "
          "separate");
    auto mixed = begin(10);
    entry(mixed, 1);
    const auto curve = spline(1, 1, 10);
    mixed.insert(mixed.end(), curve.begin(), curve.end());
    for (const auto &s : solids) {
        entry(mixed, 6);
        mixed.insert(mixed.end(), s.begin(), s.end());
    }
    const auto ordered = decode_legacy_graphics(end(mixed));
    check(ordered.at("status") == "parsed" && ordered.at("entries").size() == 10 &&
              ordered.at("entries")[9].at("geometry").at("solid_kind") == "lofted_body",
          "spline and all nine solid entries maintain their independent boundaries in one archive");
    auto omitted = begin(5);
    for (unsigned kind : {4, 5, 8, 9})
        entry(omitted, kind);
    entry(omitted, 2);
    word(omitted, 0);
    word(omitted, 4);
    for (unsigned curve_kind : {5, 6, 9, 10})
        word(omitted, curve_kind);
    const auto omitted_result = decode_legacy_graphics(end(omitted));
    bool no_fabrication = omitted_result.at("status") == "parsed";
    for (unsigned i = 0; i < 4; ++i)
        no_fabrication = no_fabrication &&
                         omitted_result.at("entries")[i].at("source_byte_count") == 60 &&
                         omitted_result.at("entries")[i].at("geometry").at("geometry_status") ==
                             "not_serialized";
    for (const auto &curve : omitted_result.at("entries")[4].at("geometry").at("curves"))
        no_fabrication = no_fabrication && curve.at("geometry_status") == "not_serialized" &&
                         curve.at("source_byte_count") == 4 && !curve.contains("points");
    check(no_fabrication,
          "native switch omissions retain type/style but never fabricate missing geometry");

    auto cstring = [&](Bytes &b, const std::vector<std::uint16_t> &units) {
        b.insert(b.end(), {0xff, 0xfe, 0xff, std::uint8_t(units.size())});
        for (auto u : units)
            put(b, u);
    };
    Bytes text;
    numbers(text, 15);
    cstring(text, {0x6587, 0, 0x5b57});
    text.insert(text.end(), {2, 3, 4, 5});
    word(text, 21);
    word(text, 22);
    word(text, 23);
    cstring(text, {65, 66});
    word(text, 7);
    cstring(text, {67});
    word(text, 9);
    text.insert(text.end(), {2, 0});
    put(text, .25);
    text.insert(text.end(), {6, 7});
    for (unsigned i = 1; i <= 5; ++i)
        word(text, 100 + i);
    put(text, std::uint64_t(8));
    text.insert(text.end(), {9, 10});
    for (unsigned i = 1; i <= 5; ++i)
        word(text, 200 + i);
    put(text, std::uint64_t(11));
    word(text, 12);
    put(text, std::uint64_t(13));
    text.push_back(14);
    for (unsigned i = 0; i < 6; ++i)
        word(text, 300 + i);
    word(text, 15);
    word(text, 16);
    numbers(text, 4, 400.);
    text.insert(text.end(), {17, 18, 19});
    put(text, 21.5);
    put(text, 22.5);
    const auto tg = geometry(text, 7);
    check(tg.at("origin") == Json::array({1., 2., 3.}) &&
              tg.at("user_origin") == Json::array({4., 5., 6.}) &&
              tg.at("rotation_matrix_rows")[1] == Json::array({10., 11., 12.}) &&
              tg.at("string").at("code_units") == Json::array({0x6587, 0, 0x5b57}),
          "legacy text keeps both origins, matrix rows and full saved Unicode string");
    check(tg.at("vertical") == true && tg.at("vertical_stored_byte") == 2 &&
              tg.at("justification_code") == 21 && tg.at("font_name").at("text") == "AB" &&
              tg.at("font_type_code") == 7 && tg.at("big_font_name").at("text") == "C" &&
              tg.at("big_font_type_code") == 9 && tg.at("bold") == true &&
              tg.at("italic") == false && tg.at("custom_slant_angle") == .25 &&
              tg.at("font_size") == Json::array({21.5, 22.5}) &&
              tg.at("font_lookup") == "not_performed",
          "saved font and style parameters do not invoke system lookup or runtime defaults");
    const auto &extra = tg.at("unapplied_fields");
    bool ranges = true;
    auto source = begin();
    entry(source, 7);
    source.insert(source.end(), text.begin(), text.end());
    source = end(source);
    for (const auto &v : extra) {
        Reader r(source, v.at("source_offset"));
        const auto type = v.at("type").get<std::string>();
        if (type == "B")
            ranges = ranges && v.at("value") == r.u8();
        if (type == "I")
            ranges = ranges && v.at("value") == r.u32();
        if (type == "Q")
            ranges = ranges && v.at("value") == r.u64();
        if (type == "d")
            ranges = ranges && v.at("value") == r.doubles(2);
        ranges = ranges && r.p == v.at("source_offset").get<std::size_t>() +
                                      v.at("source_byte_count").get<std::size_t>();
    }
    check(ranges && extra.size() == 21 &&
              tg.at("unapplied_symbologies")[1].at("line_weight") == 205 &&
              tg.at("unapplied_colors")[1].at("serialized_rgb") == Json::array({303, 304, 305}),
          "unapplied text fields keep their actual values, widths and source locations instead of "
          "presumed zero padding");

    auto csg = [&](const std::vector<Bytes> &pool) {
        Bytes b;
        word(b, 22);
        word(b, static_cast<int>(pool.size()));
        for (const auto &v : pool) {
            word(b, static_cast<int>(v.size()));
            b.insert(b.end(), v.begin(), v.end());
        }
        word(b, 0);
        word(b, 0);
        word(b, 36);
        // Native node version, header word, four empty arrays, operation/GUID,
        // old flag and child code. No geometry-index inference from pool order.
        Bytes node;
        word(node, 0x1ee290);
        word(node, 30);
        for (unsigned i = 0; i < 4; ++i)
            word(node, 0);
        word(node, 4);
        node.resize(node.size() + 16);
        node.insert(node.end(), {1, 0});
        word(b, static_cast<int>(node.size()));
        b.insert(b.end(), node.begin(), node.end());
        word(b, 0);
        word(b, 0);
        return b;
    };
    auto payload = [&](const Bytes &b) {
        Bytes out;
        word(out, static_cast<int>(b.size()));
        out.insert(out.end(), b.begin(), b.end());
        return out;
    };
    Bytes line(104);
    std::memcpy(line.data(), "bg0001fb", 8);
    auto write = [&](unsigned pos, std::uint32_t v) { std::memcpy(line.data() + pos, &v, 4); };
    write(8, 12);
    line[12] = 8;
    line[14] = 12;
    line[16] = 4;
    line[18] = 8;
    write(20, 8);
    line[24] = 1;
    write(28, 20);
    line[32] = 6;
    line[34] = 56;
    line[36] = 8;
    write(48, 16);
    const auto csg_bytes = csg({line, csg({})});
    const auto cg = geometry(payload(csg_bytes), 10);
    check(cg.at("archive_decode_status") == "decoded" && cg.at("archive").at("marker") == 22 &&
              cg.at("archive").at("geometries")[0].at("encoding") == "bgfb" &&
              cg.at("archive").at("geometries")[1].at("encoding") == "csg_archive" &&
              cg.at("archive").at("tree").at("nodes")[0].at("operation_name") == "unknown",
          "legacy CSG length wraps the native marker-22 archive with bounded nested CSG and BGFB "
          "decoding");
    const auto invalid = geometry(payload(Bytes{1, 2, 3}), 10);
    check(invalid.at("archive_decode_status") == "not_parsed" && !invalid.contains("archive") &&
              invalid.at("payload_byte_count") == 3,
          "invalid CSG payload has a local failure, not an invented empty Boolean tree");
    auto chained = begin(2);
    entry(chained, 10);
    auto data = payload(csg_bytes);
    chained.insert(chained.end(), data.begin(), data.end());
    entry(chained, 7);
    chained.insert(chained.end(), text.begin(), text.end());
    chained = end(chained);
    const auto chained_result = decode_legacy_graphics(chained);
    check(chained_result.at("status") == "parsed" &&
              chained_result.at("entries")[1].at("geometry").at("font_size")[0] == 21.5,
          "CSG length bounds its archive without consuming the following text Entry");
    LegacyGraphicsLimits limits;
    limits.max_values = cg.at("payload_byte_count").get<std::size_t>() + 100;
    auto limited = begin();
    entry(limited, 10);
    limited.insert(limited.end(), data.begin(), data.end());
    limited = end(limited);
    const auto stopped =
        decode_legacy_graphics(limited, LegacyGraphicsProfile::writer_2025, limits);
    check(stopped.at("status") == "not_parsed" && stopped.at("candidate_set_complete") == false &&
              stopped.at("reason") == "legacy graphics value limit",
          "nested CSG budget exhaustion propagates out instead of being swallowed as a local "
          "unsupported geometry");
    bool callback_propagated = false;
    try {
        decode_bgfb(line, [](std::size_t) { throw DecodeBudgetExceeded("test"); });
    } catch (const DecodeBudgetExceeded &) {
        callback_propagated = true;
    }
    check(callback_propagated, "BGFB recursion obeys the enclosing shared decoder budget");
    bool nested_propagated = false;
    unsigned calls = 0;
    try {
        decode_csg_bytes_budgeted(csg_bytes, 100000, 32, [&](std::size_t) {
            if (++calls == 3)
                throw DecodeBudgetExceeded("nested BGFB budget");
        });
    } catch (const DecodeBudgetExceeded &) {
        nested_propagated = true;
    }
    check(nested_propagated && calls == 3, "CSG's local geometry diagnostics cannot swallow a "
                                           "budget exception from its nested BGFB reader");
    auto unknown_solid = Bytes{};
    word(unknown_solid, 10);
    check(decode(unknown_solid, 6).at("unsupported_branch_encountered") == true,
          "unknown solid type remains unsupported without guessing its length");
    return checks;
}
