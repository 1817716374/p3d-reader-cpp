#include "internal.hpp"
#include <p3d/legacy_graphics.hpp>
#include <thread>

unsigned legacy_graphics_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool v, const char *message) {
        require(v, message);
        ++checks;
    };
    auto put = [](Bytes &b, auto v) {
        const auto pos = b.size();
        b.resize(pos + sizeof(v));
        std::memcpy(b.data() + pos, &v, sizeof(v));
    };
    auto u32 = [&](Bytes &b, unsigned v) { put(b, std::uint32_t(v)); };
    auto style = [&](Bytes &b, bool entry) {
        for (unsigned v : {17, 29, 53})
            u32(b, v);
        put(b, std::int32_t(-7));
        u32(b, 0xfffffffe);
        if (!entry)
            put(b, std::int32_t(9));
        put(b, .125);
        put(b, 1.25);
        if (entry) {
            put(b, 2.5);
            put(b, 3.5);
            u32(b, 0xffffffff);
        }
    };
    auto header = [&](unsigned n = 1) {
        Bytes b;
        put(b, 3.1);
        u32(b, n);
        return b;
    };
    auto entry = [&](Bytes &b, unsigned kind) {
        u32(b, kind);
        style(b, true);
    };
    auto footer = [&](Bytes b) {
        style(b, false);
        return b;
    };
    auto doubles = [&](Bytes &b, unsigned count, unsigned dim, double first) {
        u32(b, count);
        for (unsigned i = 0; i < count * dim; ++i)
            put(b, first + i);
    };
    auto ints = [&](Bytes &b, std::initializer_list<std::int32_t> values) {
        u32(b, static_cast<unsigned>(values.size()));
        for (auto v : values)
            put(b, v);
    };
    auto tag = [&](Bytes &b, const std::vector<std::uint16_t> &units) {
        b.insert(b.end(), {0xff, 0xfe, 0xff});
        const auto n = units.size();
        if (n < 0xff)
            put(b, std::uint8_t(n));
        else {
            b.push_back(0xff);
            if (n < 0xfffe)
                put(b, std::uint16_t(n));
            else {
                put(b, std::uint16_t(0xffff));
                u32(b, static_cast<unsigned>(n));
            }
        }
        for (auto v : units)
            put(b, v);
    };
    // Deterministic prefix up to (but not including) the edge vector count.
    auto mesh = [&](bool full, bool writer, const std::vector<std::uint16_t> &name) {
        Bytes b;
        u32(b, 1);
        u32(b, 0);
        u32(b, 0);
        put(b, std::uint8_t(2));
        doubles(b, 3, 3, 1);
        doubles(b, full ? 2 : 0, 2, 10);
        doubles(b, full ? 1 : 0, 3, 20);
        doubles(b, full ? 1 : 0, 3, 30);
        ints(b, full ? std::initializer_list<std::int32_t>{-1, 3}
                     : std::initializer_list<std::int32_t>{});
        ints(b,
             full ? std::initializer_list<std::int32_t>{7} : std::initializer_list<std::int32_t>{});
        u32(b, full ? 1 : 0);
        if (full) {
            put(b, .25f);
            if (writer) {
                put(b, 31.0);
                put(b, 32.0);
            } else {
                put(b, .5f);
                put(b, .75f);
            }
        }
        ints(b, {1, -2, 3, 0});
        ints(b, full ? std::initializer_list<std::int32_t>{1, -2, 0}
                     : std::initializer_list<std::int32_t>{});
        ints(b, full ? std::initializer_list<std::int32_t>{-1, 1, 1, 0}
                     : std::initializer_list<std::int32_t>{});
        ints(b, full ? std::initializer_list<std::int32_t>{2, 1, -2, 0}
                     : std::initializer_list<std::int32_t>{});
        tag(b, name);
        ints(b, full ? std::initializer_list<std::int32_t>{-1, 0x7fffffff}
                     : std::initializer_list<std::int32_t>{});
        u32(b, full ? 1 : 0);
        if (full) {
            for (unsigned i = 0; i < 20; ++i)
                put(b, 100. + i);
            u32(b, 0xffffffff);
            put(b, std::int32_t(-1));
            put(b, std::numeric_limits<std::int32_t>::min());
            put(b, std::int32_t(17));
        }
        return b;
    };
    auto archive = [&](const Bytes &body, unsigned edges = 0) {
        auto b = header();
        entry(b, 3);
        b.insert(b.end(), body.begin(), body.end());
        u32(b, edges);
        return b;
    };
    auto full = footer(archive(mesh(true, true, {0x6728, 0, 0xd83d, 0xde00})));
    const auto decoded = decode_legacy_graphics(full);
    check(decoded.at("status") == "parsed" && decoded.at("candidate_set_complete") == true,
          "all nonempty mesh pools and face metadata parse to the full container boundary");
    const auto &e = decoded.at("entries")[0], &g = e.at("geometry");
    check(g.at("parameters").at("values") ==
                  Json::array({Json::array({10., 11.}), Json::array({12., 13.})}) &&
              g.at("normals").at("values")[0] == Json::array({20., 21., 22.}) &&
              g.at("rgb_factors").at("values")[0] == Json::array({30., 31., 32.}) &&
              g.at("rgb_uints").at("values") == Json::array({0xffffffffu, 3}) &&
              g.at("rgb_table").at("values") == Json::array({7}),
          "distinct parameter, normal and color pools are retained in native order");
    const auto &color = g.at("rgb_floats").at("values")[0];
    check(color.at("red_float32") == .25 && color.at("factor_green_float64") == 31. &&
              color.at("factor_blue_float64") == 32. && !color.contains("green_float32") &&
              g.at("native_reader_wire_compatible") == false &&
              g.at("rgb_floats").at("source_byte_count") == 24,
          "native writer mixes float red and factor doubles without inventing original FloatRgb "
          "green/blue");
    check(
        g.at("point_indices").at("values") == Json::array({1, -2, 3, 0}) &&
            g.at("parameter_indices").at("values") == Json::array({1, -2, 0}) &&
            g.at("normal_indices").at("values") == Json::array({-1, 1, 1, 0}) &&
            g.at("rgb_indices").at("values") == Json::array({2, 1, -2, 0}) &&
            g.at("polygons") == Json::array({Json::array({1, -2, 3})}),
        "signed point and independent attribute indices are not remapped, welded or deduplicated");
    const auto &f = g.at("face_data").at("values")[0];
    check(f.at("parameter_distance_range") == Json::array({100., 101., 102., 103.}) &&
              f.at("normal_range") == Json::array({114., 115., 116., 117., 118., 119.}) &&
              f.at("source_index_stored_uint32") == 0xffffffffu &&
              f.at("face_ids_stored_int32") == Json::array({-1, -2147483648LL, 17}) &&
              f.at("source_byte_count") == 176,
          "face wire fields retain DWORD truncation and signed face IDs instead of assuming the "
          "runtime struct size");
    check(g.at("tag_name").at("code_units") == Json::array({0x6728, 0, 0xd83d, 0xde00}) &&
              g.at("tag_name").at("text").get<std::string>().size() == 8 &&
              bytesof(decoded.at("source_storage")) == full,
          "CString preserves Unicode, embedded NUL, surrogate pairs and original bytes");
    const auto reader_bytes = footer(archive(mesh(true, false, {})));
    const auto reader = decode_legacy_graphics(reader_bytes, LegacyGraphicsProfile::reader_2025);
    check(reader.at("status") == "parsed" &&
              reader.at("entries")[0]
                      .at("geometry")
                      .at("rgb_floats")
                      .at("values")[0]
                      .at("green_float32") == .5 &&
              reader.at("entries")[0].at("geometry").at("native_reader_wire_compatible") == true,
          "explicit reader profile uses three float32 values");
    check(decode_legacy_graphics(reader_bytes).at("status") == "not_parsed" &&
              decode_legacy_graphics(full, LegacyGraphicsProfile::reader_2025).at("status") ==
                  "not_parsed",
          "profiles never silently auto-switch after a failure");
    auto empty = footer(header(0));
    check(decode_legacy_graphics(empty).at("status") == "parsed" &&
              decode_legacy_graphics(empty).at("entries").empty(),
          "empty archive is a container, not a fabricated Entry");
    auto multi = header(3);
    entry(multi, 1);
    u32(multi, 1);
    for (unsigned i = 0; i < 6; ++i)
        put(multi, double(i));
    entry(multi, 2);
    u32(multi, 4);
    u32(multi, 1);
    u32(multi, 8);
    u32(multi, 1);
    u32(multi, 1);
    u32(multi, 2);
    doubles(multi, 2, 3, 1);
    entry(multi, 3);
    const auto simple_mesh = mesh(false, true, {});
    multi.insert(multi.end(), simple_mesh.begin(), simple_mesh.end());
    u32(multi, 0);
    multi = footer(multi);
    const auto many = decode_legacy_graphics(multi);
    check(many.at("status") == "parsed" && many.at("entries").size() == 3 &&
              many.at("entries")[0].at("geometry").at("end") == Json::array({3., 4., 5.}) &&
              many.at("entries")[1]
                      .at("geometry")
                      .at("curves")[0]
                      .at("collection")
                      .at("curves")[0]
                      .at("points")
                      .size() == 2 &&
              many.at("entries")[2].at("geometry").at("points").at("count") == 3,
          "multiple entries and nested curves retain separate style, geometry and order");
    auto points_and_arc = header();
    entry(points_and_arc, 2);
    u32(points_and_arc, 5);
    u32(points_and_arc, 2);
    u32(points_and_arc, 7);
    doubles(points_and_arc, 2, 3, 10);
    u32(points_and_arc, 3);
    for (unsigned i = 0; i < 11; ++i)
        put(points_and_arc, double(i));
    const auto pa = decode_legacy_graphics(footer(points_and_arc));
    check(pa.at("status") == "parsed" &&
              pa.at("entries")[0].at("geometry").at("curves")[0].at("kind") == "point_string" &&
              pa.at("entries")[0].at("geometry").at("curves")[1].at("vector90") ==
                  Json::array({6., 7., 8.}) &&
              pa.at("entries")[0].at("geometry").at("curves")[1].at("sweep_angle") == 10.,
          "point strings and elliptical arcs follow distinct native curve codes and lengths");
    for (unsigned n : {254, 255, 65533, 65534}) {
        const auto b = footer(archive(mesh(false, true, std::vector<std::uint16_t>(n, 65))));
        const auto result = decode_legacy_graphics(b);
        check(result.at("status") == "parsed" &&
                  result.at("entries")[0].at("geometry").at("tag_name").at("unit_count") == n,
              "CString length escape boundaries do not consume name bytes as headers");
    }
    const auto invalid_utf =
        decode_legacy_graphics(footer(archive(mesh(false, true, {0xd800, 65}))));
    check(invalid_utf.at("status") == "parsed" &&
              invalid_utf.at("entries")[0].at("geometry").at("tag_name").at("text_status") ==
                  "invalid_utf16" &&
              invalid_utf.at("entries")[0].at("geometry").at("tag_name").at("code_units") ==
                  Json::array({0xd800, 65}),
          "invalid UTF16 is retained as code units without changing the wire parse");
    auto unique_edge = archive(simple_mesh, 1);
    for (unsigned v : {2, 5, 0})
        u32(unique_edge, v);
    unique_edge = footer(unique_edge);
    const auto mismatch = decode_legacy_graphics(unique_edge);
    check(mismatch.at("status") == "parsed" &&
              mismatch.at("entries")[0]
                      .at("geometry")
                      .at("edge_chains")[0]
                      .at("declared_topology_id_count") == 5 &&
              mismatch.at("entries")[0]
                  .at("geometry")
                  .at("edge_chains")[0]
                  .at("serialized_topology_ids")
                  .empty() &&
              mismatch.at("entries")[0].at("geometry").at("native_reader_wire_compatible") == false,
          "writer edge count mismatch preserves the declaration without fabricating unstored IDs");
    check(decode_legacy_graphics(unique_edge, LegacyGraphicsProfile::reader_2025).at("status") ==
              "not_parsed",
          "reader edge profile does not borrow the writer backtracking rule");
    auto ambiguous = archive(simple_mesh, 2);
    for (unsigned v : {2, 1, 0, 1, 42, 7, 1, 0})
        u32(ambiguous, v);
    ambiguous = footer(ambiguous);
    const auto hypotheses = decode_legacy_graphics(ambiguous);
    check(hypotheses.at("status") == "ambiguous" &&
              hypotheses.at("candidate_set_complete") == true &&
              hypotheses.at("hypotheses").size() == 2 && !hypotheses.contains("entries"),
          "two full-buffer edge interpretations remain hypotheses without a guessed winner");
    auto normal_edge = archive(simple_mesh, 1);
    for (unsigned v : {7u, 1u, 101u, 2u, 1u, 0xffffffffu})
        u32(normal_edge, v);
    normal_edge = footer(normal_edge);
    const auto edge_reader =
        decode_legacy_graphics(normal_edge, LegacyGraphicsProfile::reader_2025);
    check(edge_reader.at("status") == "parsed" &&
              edge_reader.at("entries")[0].at("geometry").at("edge_chains")[0].at("indices") ==
                  Json::array({1, -1}),
          "reader topology count and signed edge index count are independently respected");
    auto bad_index = archive(simple_mesh);
    // point_indices occurs after 13 header bytes, point vector and six pool counts.
    const auto index_offset = 72 + 13 + 4 + 72 + 24 + 4;
    const std::int32_t invalid = -2147483647;
    std::memcpy(bad_index.data() + index_offset, &invalid, 4);
    const auto bad = decode_legacy_graphics(footer(bad_index));
    check(bad.at("status") == "parsed" &&
              bad.at("entries")[0].at("geometry").at("polygon_derivation_status") ==
                  "invalid_point_indices" &&
              !bad.at("entries")[0].at("geometry").contains("polygons"),
          "invalid topology does not erase source indices or silently repair derived polygons");
    auto nonfinite = full;
    const std::uint64_t nanbits = 0x7ff8000000001234ull;
    std::memcpy(nonfinite.data() + 89, &nanbits, 8);
    const auto nf = decode_legacy_graphics(nonfinite);
    check(
        nf.at("status") == "parsed" &&
            nf.at("entries")[0].at("geometry").at("points").at("values")[0][0].at("ieee754_hex") ==
                "341200000000f87f",
        "nonfinite values retain IEEE payload bits with a JSON-safe null numeric view");
    LegacyGraphicsLimits limits;
    limits.max_interpretations = 1;
    const auto capped =
        decode_legacy_graphics(ambiguous, LegacyGraphicsProfile::writer_2025, limits);
    check(capped.at("status") == "not_parsed" && capped.at("candidate_set_complete") == false &&
              capped.at("hypotheses").size() == 1,
          "an interrupted ambiguity search cannot advertise a unique parse");
    limits = {};
    limits.max_work = 0;
    check(decode_legacy_graphics(full, LegacyGraphicsProfile::writer_2025, limits)
                  .at("candidate_set_complete") == false,
          "work limit terminates the whole archive search");
    limits = {};
    limits.max_values = 3;
    check(decode_legacy_graphics(full, LegacyGraphicsProfile::writer_2025, limits).at("status") ==
              "not_parsed",
          "shared value budget includes style and channels");
    limits = {};
    limits.max_entries = 2;
    check(decode_legacy_graphics(multi, LegacyGraphicsProfile::writer_2025, limits).at("status") ==
              "not_parsed",
          "entry count is bounded before parsing");
    limits = {};
    limits.max_depth = 0;
    check(decode_legacy_graphics(multi, LegacyGraphicsProfile::writer_2025, limits).at("status") ==
              "not_parsed",
          "nested curves consume depth budget");
    limits = {};
    limits.max_string_units = 1;
    check(decode_legacy_graphics(full, LegacyGraphicsProfile::writer_2025, limits).at("status") ==
              "not_parsed",
          "CString budget applies before allocating name storage");
    limits = {};
    limits.max_input_bytes = 1;
    const auto oversized = decode_legacy_graphics(full, LegacyGraphicsProfile::writer_2025, limits);
    check(oversized.at("status") == "not_parsed" && !oversized.contains("source_storage"),
          "oversized input is rejected before copying source storage");
    auto unknown = header();
    entry(unknown, 4);
    unknown = footer(unknown);
    check(
        decode_legacy_graphics(unknown).at("unsupported_branch_encountered") == true &&
            decode_legacy_graphics(unknown).at("candidate_set_complete") == false,
        "unsupported CSG archive bytes are not silently skipped or considered an exhausted search");
    auto extra = full;
    extra.push_back(0);
    check(decode_legacy_graphics(extra).at("status") == "not_parsed",
          "trailing bytes invalidate full-buffer consumption");
    for (std::size_t pos : {std::size_t(0), std::size_t(11), std::size_t(71), full.size() - 1}) {
        const auto truncated = Bytes(full.begin(), full.begin() + pos);
        check(decode_legacy_graphics(truncated).at("status") == "not_parsed",
              "truncation never yields a partial successful archive");
    }
    std::array<Json, 3> parallel;
    std::array<std::thread, 3> threads;
    for (unsigned i = 0; i < 3; ++i)
        threads[i] = std::thread([&, i] { parallel[i] = decode_legacy_graphics(full); });
    for (auto &t : threads)
        t.join();
    check(
        std::all_of(parallel.begin(), parallel.end(), [&](const Json &j) { return j == decoded; }),
        "legacy archive readers have no shared mutable state");
    return checks;
}
