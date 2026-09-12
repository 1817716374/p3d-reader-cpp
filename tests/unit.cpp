#include "geometry.hpp"
#include "blob_internal.hpp"
#include "guided.hpp"
#include "electrical_wire.hpp"
#include <iostream>
#include <cstring>
#include <lz4/lz4.h>
using namespace p3d;
unsigned bspline_surface_tests();
unsigned bspline_trim_tests();
unsigned akima_tests();
unsigned interpolation_tests();
unsigned spiral_tests();
unsigned section_loft_tests();
unsigned material_semantics_tests();
unsigned material_legacy_tests();
unsigned material_resource_tests();
unsigned material_numeric_tests();
unsigned mesh_channel_tests();
unsigned mesh_extension_tests();
unsigned native_material_tests();
unsigned block_transform_tests();
unsigned native_input_tests();
unsigned native_attribute_input_tests();
unsigned native_attribute_lookup_tests();
unsigned material_catalog_registration_tests();
unsigned material_auxiliary_tests();
unsigned material_xml_integer_tests();
unsigned material_xml_float_tests();
unsigned material_root_tests();
unsigned material_version_tests();
unsigned material_replicator_tests();
unsigned material_layers_tests();
unsigned material_projection_tests();
unsigned material_projection_link_tests();
unsigned material_projection_math_tests();
unsigned native_list_input_tests();
unsigned native_id_tests();
unsigned native_dependency_tests();
unsigned native_reference_path_tests();
unsigned native_application_tests();
unsigned mesh_normal_tests();
unsigned mesh_tessellation_tests();
unsigned mesh_buffer_tests();
static unsigned checks = 0;
static void check(bool value, const char *message) {
    ++checks;
    require(value, message);
}
template <class F> static void rejects(F f, const char *message) {
    bool threw = false;
    try {
        f();
    } catch (const std::exception &) {
        threw = true;
    }
    check(threw, message);
}
template <class T> static void put(Bytes &b, T x) {
    auto off = b.size();
    b.resize(off + sizeof(x));
    std::memcpy(b.data() + off, &x, sizeof(x));
}
static Bytes wire_bytes(const char *text) {
    std::string s(text);
    Bytes b;
    for (std::size_t i = 0; i < s.size(); i += 2)
        b.push_back(static_cast<std::uint8_t>(std::stoul(s.substr(i, 2), nullptr, 16)));
    return b;
}
static void append_wire(Bytes &b, const char *text) {
    auto raw = wire_bytes(text);
    b.insert(b.end(), raw.begin(), raw.end());
}
static void bspline_tests() {
    auto table = [](unsigned order, bool closed, Json poles, Json weights = nullptr,
                    Json knots = nullptr) {
        return Json{{"_type", "BsplineCurve"}, {"order", order},     {"closed", closed},
                    {"poles", poles},          {"weights", weights}, {"knots", knots}};
    };
    auto near = [](Point3 a, Point3 b, double tolerance = 1e-12) {
        return std::abs(a[0] - b[0]) <= tolerance && std::abs(a[1] - b[1]) <= tolerance &&
               std::abs(a[2] - b[2]) <= tolerance;
    };
    const double w = std::sqrt(0.5);
    auto quarter_source =
        table(3, false, {1, 0, 0, w, w, 0, 0, 1, 0}, {1, w, 1}, {0, 0, 0, 1, 1, 1});
    auto quarter = BsplineCurve::from_bgfb(quarter_source);
    check(quarter.rational() && quarter.poles()[1] == Point3{w, w, 0} &&
              near(quarter.point_at(0.5), {w, w, 0}),
          "BGFB rational poles are already weighted and must not be multiplied again");
    for (unsigned i = 0; i <= 20; ++i) {
        auto p = quarter.point_at(i / 20.0);
        check(std::abs(p[0] * p[0] + p[1] * p[1] - 1) < 1e-12,
              "rational quadratic remains exactly on the unit circle");
    }
    check(near(quarter.point_at(0), {1, 0, 0}) && near(quarter.point_at(1), {0, 1, 0}),
          "clamped rational endpoints");
    auto uniform_source = table(3, false, {0, 0, 0, 1, 2, 0, 2, 0, 0, 3, 2, 0});
    auto uniform = BsplineCurve::from_bgfb(uniform_source);
    check(uniform.source_knots().empty() &&
              uniform.knots() == std::vector<double>({0, 0, 0, 0.5, 1, 1, 1}) &&
              !uniform.rational(),
          "omitted open knots generate a clamped uniform vector without changing source knots");
    auto shifted = quarter_source;
    shifted["knots"] = {2, 2, 2, 6, 6, 6};
    auto shifted_curve = BsplineCurve::from_bgfb(shifted);
    check(shifted_curve.knot_domain() == std::array<double, 2>{2, 6} &&
              near(shifted_curve.point_at(0.5), quarter.point_at(0.5)),
          "source knots remain unnormalized while fractions use their active domain");
    auto periodic_source = table(3, true, {0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0});
    auto periodic = BsplineCurve::from_bgfb(periodic_source);
    check(periodic.knots() ==
                  std::vector<double>({-0.5, -0.25, 0, 0.25, 0.5, 0.75, 1, 1.25, 1.5}) &&
              periodic.periodic_pole_shift() == 0 && near(periodic.point_at(0), {1, 0, 0}) &&
              near(periodic.point_at(1), periodic.point_at(0)) &&
              near(periodic.point_at(0.25), {2, 1, 0}),
          "uniform periodic knots reuse cyclic native poles without duplicating source data");
    periodic_source["knots"] = {-0.7, -0.4, 0, 0.1, 0.3, 0.6, 1, 1.1, 1.3};
    periodic = BsplineCurve::from_bgfb(periodic_source);
    check(near(periodic.point_at(0), {1.6, 0, 0}) &&
              near(periodic.point_at(1), periodic.point_at(0)),
          "nonuniform periodic endpoint basis uses adjacent knot intervals");
    const double s = std::sqrt(3.0) / 2;
    auto circle_source =
        table(3, true, {1, 0, 0, 0.5, s, 0, -0.5, s, 0, -1, 0, 0, -0.5, -s, 0, 0.5, -s, 0, 1, 0, 0},
              {1, 0.5, 1, 0.5, 1, 0.5, 1},
              {-1.0 / 3, 0, 0, 0, 1.0 / 3, 1.0 / 3, 2.0 / 3, 2.0 / 3, 1, 1, 1, 4.0 / 3});
    auto circle = BsplineCurve::from_bgfb(circle_source);
    check(circle.periodic_pole_shift() == -1 && circle.poles().size() == 7 &&
              near(circle.point_at(0), {1, 0, 0}) && near(circle.point_at(1), {1, 0, 0}) &&
              near(circle.point_at(1.0 / 3), {-0.5, s, 0}),
          "native closed clamped-like seam shifts pole indexing while preserving source arrays");
    for (unsigned i = 0; i <= 60; ++i) {
        auto p = circle.point_at(i / 60.0);
        check(std::abs(p[0] * p[0] + p[1] * p[1] - 1) < 1e-12,
              "all three rational periodic conic pieces stay on the unit circle");
    }
    auto discontinuous = BsplineCurve::from_bgfb(
        table(2, false, {0, 0, 0, 1, 0, 0, 10, 0, 0, 11, 0, 0}, nullptr, {0, 0, 0.5, 0.5, 1, 1}));
    check(near(discontinuous.point_at(0.25), {0.5, 0, 0}) &&
              near(discontinuous.point_at(0.5), {10, 0, 0}) &&
              near(discontinuous.point_at(0.75), {10.5, 0, 0}),
          "full internal knot multiplicity retains a discontinuity instead of bridging poles");
    auto singular = BsplineCurve::from_bgfb(table(2, false, {1, 0, 0, 1, 0, 0}, {1, -1}));
    check(singular.weights()[1] == -1 && singular.homogeneous_at(0.5)[3] == 0,
          "negative weights and points at infinity remain available in homogeneous coordinates");
    rejects([&] { singular.point_at(0.5); }, "zero evaluated weight cannot produce a finite point");
    auto zero_control = BsplineCurve::from_bgfb(table(2, false, {1, 0, 0, 2, 0, 0}, {0, 1}));
    check(near(zero_control.point_at(0.5), {3, 0, 0}),
          "zero control weight is valid away from a singular evaluated parameter");
    rejects([&] { quarter.point_at(-0.1); }, "negative fraction is not implicitly wrapped");
    rejects([&] { periodic.point_at(1.1); },
            "periodic curve does not silently wrap caller fraction");
    rejects([&] { quarter.point_at(std::numeric_limits<double>::quiet_NaN()); },
            "NaN fraction is rejected");
    for (auto bad :
         {table(1, false, {0, 0, 0}), table(3, false, {0, 0, 0, 1, 0, 0}),
          table(2, false, {0, 0, 0, 1, 0, 0, 2}), table(2, false, {0, 0, 0, 1, 0, 0}, {1}),
          table(2, true, {0, 0, 0, 1, 0, 0}, nullptr, {0, 0, 1, 1}),
          table(2, false, {0, 0, 0, 1, 0, 0}, nullptr, {0, 1, 0, 1}),
          table(2, false, {0, 0, 0, 1, 0, 0}, nullptr, {0, 0, 0, 0}),
          table(2, false, {0, 0, 0, 1, 0, 0}, {1, std::numeric_limits<double>::infinity()})})
        rejects([&] { BsplineCurve::from_bgfb(bad); }, "invalid B-spline source is rejected");

    // Forward FlatBuffer fixture: root VariantGeometry(tag=3), then BsplineCurve.
    auto encode = [&](Json source) {
        Bytes b(12);
        std::memcpy(b.data(), "bg0001fb", 8);
        auto write = [&](std::size_t p, auto x) { std::memcpy(b.data() + p, &x, sizeof(x)); };
        auto reference = [&](std::size_t p, std::size_t target) {
            write(p, std::uint32_t(target - p));
        };
        auto table_at = [&](std::vector<std::uint16_t> slots, std::uint16_t size) {
            while (b.size() % 4)
                b.push_back(0);
            auto vt = b.size();
            put<std::uint16_t>(b, std::uint16_t(4 + 2 * slots.size()));
            put(b, size);
            for (auto offset : slots)
                put(b, offset);
            while (b.size() % 4)
                b.push_back(0);
            auto obj = b.size();
            b.resize(obj + size);
            write(obj, std::int32_t(obj - vt));
            return obj;
        };
        auto root = table_at({4, 8}, 12);
        reference(8, root);
        b[root + 4] = 3;
        auto curve = table_at({4, 8, 12, std::uint16_t(source["weights"].is_null() ? 0 : 16),
                               std::uint16_t(source["knots"].is_null() ? 0 : 20)},
                              24);
        reference(root + 8, curve);
        write(curve + 4, source["order"].get<std::int32_t>());
        b[curve + 8] = source["closed"].get<bool>();
        for (auto entry :
             {std::pair<const char *, unsigned>{"poles", 12}, {"weights", 16}, {"knots", 20}}) {
            if (source[entry.first].is_null())
                continue;
            while ((b.size() + 4) % 8)
                b.push_back(0);
            auto p = b.size();
            put<std::uint32_t>(b, unsigned(source[entry.first].size()));
            for (const auto &v : source[entry.first])
                put<double>(b, v.get<double>());
            reference(curve + entry.second, p);
        }
        return b;
    };
    const auto decoded = decode_bgfb(encode(circle_source))["geometry"];
    check(decoded["_spline"]["status"] == "valid" &&
              decoded["_spline"]["pole_coordinates"] == "weighted_xyz" &&
              decoded["_spline"]["periodic_pole_shift"] == -1 &&
              decoded["poles"] == circle_source["poles"] &&
              near(BsplineCurve::from_bgfb(decoded).point_at(0.5), {-1, 0, 0}),
          "BGFB decoder exposes confirmed curve semantics and feeds the public evaluator");
    auto generated = decode_bgfb(encode(uniform_source))["geometry"];
    check(generated["knots"].is_null() &&
              generated["_spline"]["knots_source"] == "generated_uniform",
          "missing source knots remain null in decoded data with generation explicitly identified");
    auto invalid = quarter_source;
    invalid["weights"] = {1};
    auto invalid_decoded = decode_bgfb(encode(invalid))["geometry"];
    check(invalid_decoded["weights"] == Json({1}) &&
              invalid_decoded["_spline"]["status"] == "invalid",
          "malformed semantic arrays remain decoded with an explicit diagnostic");
    Bytes packet(32);
    auto bgfb = encode(circle_source);
    put<std::uint64_t>(packet, bgfb.size());
    packet.insert(packet.end(), bgfb.begin(), bgfb.end());
    Bytes body(142);
    body[0] = 1;
    body[134] = 1;
    put<std::uint32_t>(body, unsigned(packet.size()));
    body.insert(body.end(), packet.begin(), packet.end());
    put<std::int32_t>(body, -7);
    body.push_back(3);
    put<std::uint32_t>(body, 0);
    put<std::uint64_t>(body, 0);
    Bytes component(11);
    put<std::uint32_t>(component, 1);
    component.resize(component.size() + 11);
    put<std::uint32_t>(component, unsigned(body.size()));
    component.insert(component.end(), body.begin(), body.end());
    component.resize(component.size() + 25);
    const auto parsed = complex_blob("ParaCmptInstance", component);
    const auto &entry = parsed["instances"][0]["geometry_packets"][0];
    check(
        !entry.contains("decode_error") && entry["raw_base64"] == base64(packet) &&
            near(BsplineCurve::from_bgfb(entry["geometry"]["geometry"]).point_at(0.5), {-1, 0, 0}),
        "component packet reaches public B-spline evaluator while preserving packet bytes");
    // A single clamped span is independently evaluable as a Bernstein polynomial.
    for (unsigned degree = 1; degree <= 8; ++degree) {
        Json poles = Json::array(), weights = Json::array();
        std::vector<Point3> cartesian;
        for (unsigned i = 0; i <= degree; ++i) {
            double weight = 0.5 + i;
            Point3 p{double(i * i), std::sin(double(i)), double(i % 3)};
            cartesian.push_back(p);
            weights.push_back(weight);
            for (double x : p)
                poles.push_back(x * weight);
        }
        const auto c = BsplineCurve::from_bgfb(table(degree + 1, false, poles, weights));
        for (unsigned sample = 0; sample <= 10; ++sample) {
            double t = sample / 10.0, sum = 0, binomial = 1;
            Point3 expected{};
            for (unsigned i = 0; i <= degree; ++i) {
                double term = binomial * std::pow(t, i) * std::pow(1 - t, degree - i) *
                              weights[i].get<double>();
                sum += term;
                for (unsigned axis = 0; axis < 3; ++axis)
                    expected[axis] += term * cartesian[i][axis];
                binomial *= double(degree - i) / double(i + 1);
            }
            for (auto &x : expected)
                x /= sum;
            check(near(c.point_at(t), expected, 1e-10),
                  "general-order rational B-spline agrees with independent Bernstein evaluation");
        }
    }
    auto shifted_seam = circle_source;
    shifted_seam["knots"][3] = 1e-7;
    check(BsplineCurve::from_bgfb(shifted_seam).periodic_pole_shift() == -1,
          "native seam near-zero threshold includes its boundary");
    shifted_seam["knots"][3] = std::nextafter(1e-7, 1.0);
    check(BsplineCurve::from_bgfb(shifted_seam).periodic_pole_shift() == 0,
          "native seam shift does not extend beyond its threshold");
}
static void bgfb_native_tests() {
    struct Fixture {
        Bytes bytes;
        std::size_t body, vtable, groups = 0, first_group = 0;
    };
    auto make = [](unsigned tag, bool capped, bool omit_cap, bool extra = false) {
        Fixture f;
        auto &b = f.bytes;
        b.resize(12);
        std::memcpy(b.data(), "bg0001fb", 8);
        auto write = [&](std::size_t offset, auto value) {
            std::memcpy(b.data() + offset, &value, sizeof(value));
        };
        auto aligned = [&]() {
            while (b.size() % 4)
                b.push_back(0);
            return b.size();
        };
        auto table = [&](std::vector<std::uint16_t> offsets, unsigned size) {
            auto vt = aligned();
            put<std::uint16_t>(b, std::uint16_t(4 + offsets.size() * 2));
            put<std::uint16_t>(b, std::uint16_t(size));
            for (auto offset : offsets)
                put(b, offset);
            auto object = aligned();
            b.resize(object + size);
            write(object, std::int32_t(object - vt));
            return object;
        };
        auto reference = [&](std::size_t field, std::size_t target) {
            write(field, std::uint32_t(target - field));
        };
        auto vector = [&](unsigned n) {
            auto p = aligned();
            put<std::uint32_t>(b, n);
            b.resize(b.size() + n * 4);
            return p;
        };
        auto curve_array = [&](int type) {
            auto p = table({4}, 8);
            write(p + 4, std::int32_t(type));
            return p;
        };
        auto root = table({4, 8}, 12);
        reference(8, root);
        b[root + 4] = std::uint8_t(tag);
        std::vector<std::uint16_t> slots = tag == 20 ? std::vector<std::uint16_t>{4, 8, 12}
                                                     : std::vector<std::uint16_t>{4, 8, 12, 16};
        if (omit_cap)
            slots.pop_back();
        if (extra)
            slots.push_back(20);
        const unsigned size = extra ? 24 : tag == 20 ? 16 : 20;
        f.body = table(slots, size);
        f.vtable = f.body - Reader(b, f.body).i32();
        reference(root + 8, f.body);
        reference(f.body + 4, curve_array(2));
        reference(f.body + 8, curve_array(tag == 20 ? 1 : 3));
        if (!omit_cap)
            b[f.body + (tag == 20 ? 12 : 16)] = capped;
        if (extra)
            write(f.body + 20, std::uint32_t(123));
        if (tag == 21) {
            f.groups = vector(2);
            reference(f.body + 12, f.groups);
            for (unsigned i = 0; i < 2; ++i) {
                auto group = vector(i + 1);
                if (!i)
                    f.first_group = group;
                reference(f.groups + 4 + 4 * i, group);
                for (unsigned j = 0; j <= i; ++j)
                    reference(group + 4 + 4 * j, curve_array(int(i * 2 + j + 1)));
            }
        }
        return f;
    };
    auto sweep = make(20, true, false);
    auto value = decode_bgfb(sweep.bytes)["geometry"];
    check(value["_type"] == "P3DSweptBody" && value["profile"]["type"] == 2 &&
              value["path"]["type"] == 1 && value["capped"] == true,
          "P3D BGFB tag 20 reads swept profile/path/cap instead of public catenary data");
    value = decode_bgfb(make(20, false, true).bytes)["geometry"];
    check(value["capped"] == false && value["_present_fields"] == Json({"profile", "path"}),
          "swept body reads default false when the cap slot is absent from the vtable");
    auto loft = make(21, true, false);
    value = decode_bgfb(loft.bytes)["geometry"];
    check(value["_type"] == "P3DSectionLoft" && value["section0"]["type"] == 2 &&
              value["section1"]["type"] == 3 && value["capped"] == true,
          "P3D BGFB tag 21 retains the native bottom/top section order");
    check(value["guide_groups"].size() == 2 && value["guide_groups"][0].size() == 1 &&
              value["guide_groups"][1].size() == 2 && value["guide_groups"][1][1]["type"] == 4,
          "native guide groups remain nested and keep their source order");
    check(value["_unknown_field_slots"].empty() && !value.contains("_semantic_status"),
          "confirmed native layout replaces the sample-inferred special case");
    auto uncapped = make(21, false, true);
    value = decode_bgfb(uncapped.bytes)["geometry"];
    check(value["capped"] == false && value["guide_groups"].size() == 2 &&
              value["_present_fields"] == Json({"section0", "section1", "guide_groups"}),
          "uncapped loft with a shorter vtable remains a loft instead of public PartialCurve");
    check(decode_bgfb(make(21, false, false).bytes)["geometry"]["capped"] == false,
          "explicit false and omitted cap both retain native boolean semantics");
    auto future = make(21, true, false, true);
    value = decode_bgfb(future.bytes)["geometry"];
    check(value["_type"] == "P3DSectionLoft" && value["capped"] == true &&
              value["_unknown_field_slots"] == Json::array({{{"slot", 4}, {"offset", 20}}}),
          "extra native loft fields are recorded without changing union interpretation");
    auto bad = loft.bytes;
    bad.resize(loft.first_group + 4);
    rejects([&] { decode_bgfb(bad); }, "truncated nested guide vector is rejected");
    bad = loft.bytes;
    const std::uint32_t excessive = UINT32_MAX;
    std::memcpy(bad.data() + loft.groups, &excessive, 4);
    rejects([&] { decode_bgfb(bad); }, "nested guide-group count cannot exceed its source buffer");
    bad = loft.bytes;
    const std::uint16_t crossing = 19;
    std::memcpy(bad.data() + loft.vtable + 4, &crossing, 2);
    rejects([&] { decode_bgfb(bad); }, "relative field must fit wholly inside its table");

    Bytes packet(32);
    put<std::uint64_t>(packet, uncapped.bytes.size());
    packet.insert(packet.end(), uncapped.bytes.begin(), uncapped.bytes.end());
    Bytes body(142);
    body[0] = 1;
    body[134] = 1;
    put<std::uint32_t>(body, unsigned(packet.size()));
    body.insert(body.end(), packet.begin(), packet.end());
    put<std::int32_t>(body, 7);
    body.push_back(3);
    put<std::uint32_t>(body, 0);
    put<std::uint64_t>(body, 0);
    Bytes source(11);
    put<std::uint32_t>(source, 1);
    source.resize(source.size() + 11);
    put<std::uint32_t>(source, unsigned(body.size()));
    source.insert(source.end(), body.begin(), body.end());
    source.resize(source.size() + 25);
    auto parsed = complex_blob("ParaCmptInstance", source);
    const auto &parsed_packet = parsed["instances"][0]["geometry_packets"][0];
    check(!parsed_packet.contains("decode_error") &&
              parsed_packet["geometry"]["geometry"]["_type"] == "P3DSectionLoft" &&
              parsed_packet["geometry"]["geometry"]["capped"] == false &&
              parsed_packet["raw_base64"] == base64(packet),
          "native loft decoding is reached through the P3D component packet and preserves source "
          "bytes");
}
static void view_link_sequence_tests() {
    auto record = [](std::uint32_t count, const std::vector<std::uint64_t> &ids,
                     std::uint16_t flag = 1, unsigned subtype = 33) {
        Bytes b(38, 0);
        put(b, flag);
        put(b, count);
        for (auto id : ids)
            put(b, id);
        std::uint16_t type = 47;
        std::uint32_t words = std::uint32_t((b.size() - 4) / 2);
        std::memcpy(b.data() + 4, &type, 2);
        std::memcpy(b.data() + 8, &words, 4);
        std::memcpy(b.data() + 12, &words, 4);
        std::memcpy(b.data() + 16, &subtype, 4);
        return b;
    };
    auto raw = record(4, {0xfedcba9876543210ull, 7, 7, 0}, 0x8001);
    auto n = parse_native(raw)[0];
    check(n["view_link_sequence"]["entry_ids"] == Json({0xfedcba9876543210ull, 7, 7, 0}) &&
              n["view_link_sequence"]["sequence_flag"] == 0x8001 && bytesof(n["data"]) == raw,
          "native link sequence preserves 64 bit IDs, duplicates, zero entries and raw flag");
    check(n["view_link_sequence"]["entries"][3]["kind"] == "current_model" &&
              n["view_link_sequence"]["entries"][0]["kind"] == "model_link" &&
              n["view_link_sequence"]["entries"][0]["source_id"] == 0xfedcba9876543210ull &&
              n["view_link_sequence"]["entries"][2]["source_index"] == 2,
          "zero sequence sentinel denotes current model without rewriting source IDs");
    check(parse_native(record(0, {}))[0]["view_link_sequence"]["entry_ids"].empty() &&
              !parse_native(record(0, {}, 1, 34))[0].contains("view_link_sequence"),
          "empty view sequence distinguished from another type 47 subtype");
    check(
        parse_native(record(2, {7}))[0]["view_link_sequence"].contains("decode_error") &&
            parse_native(record(0, {7}))[0]["view_link_sequence"].contains("decode_error") &&
            parse_native(record(0xffffffff, {}))[0]["view_link_sequence"].contains("decode_error"),
        "bad view sequence lengths do not create partial orderings");
    auto bad = record(3, {7});
    auto good = record(1, {9});
    bad.insert(bad.end(), good.begin(), good.end());
    auto records = parse_native(bad);
    check(records.size() == 2 && records[0]["view_link_sequence"].contains("decode_error") &&
              records[1]["view_link_sequence"]["entry_ids"] == Json({9}),
          "semantic sequence error preserves following records");
}
static void native_layer_tests() {
    auto set = [](Bytes &b, std::size_t at, auto value) {
        require(at + sizeof(value) <= b.size(), "test layer offset");
        std::memcpy(b.data() + at, &value, sizeof(value));
    };
    auto header = [&](Bytes &b, unsigned subtype = 1) {
        set(b, 4, std::uint16_t(49));
        auto words = std::uint32_t((b.size() - 4) / 2);
        set(b, 8, words);
        set(b, 12, words);
        set(b, 16, std::uint32_t(subtype));
    };
    Bytes raw(244, 0);
    header(raw);
    set(raw, 20, std::uint64_t(0xfedcba9876543210ull));
    set(raw, 36, std::uint32_t(0xffffffff));
    set(raw, 44, std::uint16_t(6));
    set(raw, 46, std::uint16_t(0x4070));
    set(raw, 68, std::int32_t(-2));
    set(raw, 72, std::uint32_t(0xffffffff));
    set(raw, 76, std::uint32_t(0x56));
    set(raw, 80, std::uint32_t(0x123456));
    set(raw, 84, float(0.375));
    set(raw, 92, std::uint32_t(0xfffa091b));
    auto n = parse_native(raw)[0];
    auto d = n["layer_definition"];
    check(n["id"] == 0xfedcba9876543210ull && d["layer_id"] == 0xffffffffu &&
              bytesof(n["data"]) == raw,
          "layer and native element IDs remain distinct and unsigned source is preserved");
    check(d["display"] == true && d["print"] == true && d["frozen"] == true &&
              d["access_mode"] == 1 && d["locked"] == true && d["state_flags"] == 0x4070 &&
              d["view_visibility"] == "not_evaluated",
          "layer state decodes native bits and legacy lock restoration");
    check(d["by_layer_symbology"]["color_index"] == 0x123456 &&
              d["by_layer_symbology"]["line_style"] == -2 &&
              d["by_layer_symbology"]["line_weight"] == 0xffffffffu && d["transparency"] == 0.375 &&
              d["business_code"] == ((0xfffa091bu >> 3) & 0xffff),
          "layer properties retain signed styles, extended colors and source transparency");
    check((d["unassigned_extended_bits"].get<unsigned>() |
           (d["business_code"].get<unsigned>() << 3)) == 0xfffa091bu,
          "business code does not discard unrelated extended flags");
    for (auto pair : {std::pair<unsigned, unsigned>{0, 0}, {0x2000, 2}, {0x3010, 3}}) {
        auto b = raw;
        set(b, 46, std::uint16_t(pair.first));
        auto state = decode_native_layer(b, Json::array());
        check(state["access_mode"] == pair.second && state["locked"] == (pair.second == 1),
              "multi-bit access state is not flattened into any-nonzero locked");
    }
    for (auto pair : {std::pair<unsigned, unsigned>{0x123456, 0x123456},
                      {0, 0},
                      {0xfffffffd, 0xfd},
                      {0xfffffffe, 0xfffffffe},
                      {0xffffffff, 0xffffffff}}) {
        auto b = raw;
        set(b, 76, std::uint32_t(pair.second));
        set(b, 80, std::uint32_t(pair.first));
        unsigned expected = pair.first == 0x123456 ? pair.second : pair.first;
        check(decode_native_layer(b, Json::array())["by_layer_symbology"]["color_index"] ==
                  expected,
              "color compatibility follows native truncation and special sentinel boundaries");
    }
    auto mismatch = raw;
    set(mismatch, 76, std::uint32_t(0x55));
    check(decode_native_layer(mismatch, Json::array())["by_layer_symbology"]["color_index"] == 0x55,
          "inconsistent extended color falls back to the legacy field");
    for (unsigned version = 0; version <= 8; ++version) {
        auto b = slice(raw, 0, 108);
        header(b);
        set(b, 44, std::uint16_t(version));
        auto layer = parse_native(b)[0]["layer_definition"];
        if (version == 0 || version == 8)
            check(layer["status"] == "unsupported_version" && !layer.contains("display"),
                  "unconfirmed layer versions do not inherit a guessed layout");
        else
            check(layer["status"] == "decoded" && layer["business_code"].is_null() == (version < 4),
                  "common layer layout supports old lengths with version-specific flags");
    }
    for (std::size_t length = 36; length < 108; length += 2) {
        auto b = slice(raw, 0, length);
        header(b);
        b.insert(b.end(), raw.begin(), raw.end());
        auto records = parse_native(b);
        check(records.size() == 2 && records[0]["layer_definition"].contains("decode_error") &&
                  records[1]["layer_definition"]["status"] == "decoded",
              "truncated layer body preserves following native records");
    }
    auto wrong = raw;
    header(wrong, 18);
    check(!parse_native(wrong)[0].contains("layer_definition"),
          "another type 49 subtype is not interpreted as a layer");
    auto bad_float = raw;
    set(bad_float, 84, std::uint32_t(0x7fc00001));
    auto bad = decode_native_layer(bad_float, Json::array());
    check(bad["transparency"].is_null() && bad.contains("transparency_error") &&
              bad["transparency_source_bits"] == 0x7fc00001 && bad["display"] == true,
          "nonfinite transparency is explicit and leaves independent layer values available");
    auto link = [](unsigned key, const char *value) {
        Bytes b;
        put<std::uint16_t>(b, key);
        put<std::uint16_t>(b, 0);
        put<std::uint32_t>(b, std::uint32_t(std::strlen(value) + 4));
        append_wire(b, "fffe0100");
        b.insert(b.end(), value, value + std::strlen(value));
        b.push_back(0xa5);
        return Json{{"app", 0x56d2},
                    {"header", 0x1000},
                    {"offset", 0x100000000ull},
                    {"payload", rawbytes(b)}};
    };
    Json links = Json::array({link(1, "Layer A"), link(2, "Description"), link(1, "Layer B"),
                              link(37, "Unassigned"), link(1, "Broken")});
    auto broken = bytesof(links[4]["payload"]);
    set(broken, 4, std::uint32_t(0xffffffff));
    links[4]["payload"] = rawbytes(broken);
    auto named = decode_native_layer(raw, links);
    check(named["name"] == "Layer B" && named["description"] == "Description" &&
              named["strings"].size() == 5 && named["strings"][3]["key"] == 37 &&
              !named["strings"][3].contains("role") &&
              named["strings"][4].contains("decode_error") &&
              named["strings"][0]["source_offset"] == 0x100000000ull &&
              bytesof(named["strings"][0]["trailing_storage"]) == Bytes{0xa5},
          "layer names preserve duplicate links, unknown keys, padding and individual errors");
    for (auto item : {std::pair<const char *, const char *>{"44656661756c74", "Default"},
                      {"e9", u8"é"},
                      {"fffe0100e9", u8"é"},
                      {"fffee900", u8"é"},
                      {"fffde900", u8"é"},
                      {"fffe3dd800de", u8"😀"},
                      {"", ""},
                      {"410042", "A"},
                      {"fffe410000004200", "A"}}) {
        Bytes payload;
        auto bytes = wire_bytes(item.first);
        put<std::uint32_t>(payload, 1);
        put<std::uint32_t>(payload, std::uint32_t(bytes.size()));
        payload.insert(payload.end(), bytes.begin(), bytes.end());
        auto l = link(1, "");
        l["payload"] = rawbytes(payload);
        check(decode_native_layer(raw, Json::array({l}))["name"] == item.second,
              "layer strings follow code page 1200 widening and wide-prefix rules");
    }
    for (auto bytes : {"fffeff", "fffdff", "feff0041", "fdff0041"}) {
        auto content = wire_bytes(bytes);
        Bytes payload;
        put<std::uint32_t>(payload, 1);
        put<std::uint32_t>(payload, std::uint32_t(content.size()));
        payload.insert(payload.end(), content.begin(), content.end());
        auto l = link(1, "");
        l["payload"] = rawbytes(payload);
        auto d = decode_native_layer(raw, Json::array({l}));
        check(!d.contains("name") && d["strings"][0].contains("decode_error"),
              "malformed wide lengths and rejected byte orders are not recoded as narrow text");
    }
}
static void layer_group_tests() {
    // Three source entries: packed bits span several words, while the wire stores
    // one WORD per bit. A nonzero unused word must not become an extra override.
    Bytes b;
    put<std::uint32_t>(b, 3);
    put<std::uint32_t>(b, 95);
    put<std::uint32_t>(b, 33);
    std::vector<std::uint16_t> storage(33, 0);
    storage[0] = 0x5800; // color, style, weight
    storage[1] = 0x0600; // display, print
    storage[2] = 0xffff; // only bit 32 is in range
    storage[32] = 0xabcd;
    for (auto w : storage)
        put(b, w);
    put<std::uint32_t>(b, 95); // Keep a duplicate source ID, without merging entries.
    put<std::uint32_t>(b, 1);
    put<std::uint16_t>(b, 1);
    put<std::uint32_t>(b, 0xffffffff);
    put<std::uint32_t>(b, 0);
    auto d = decode_attribute(0, 4, b, 0);
    auto &e = d["entries"];
    check(e.size() == 3 && e[0]["layer_id"] == 95 && e[1]["layer_id"] == 95 &&
              e[2]["layer_id"] == 0xffffffffu && e[1]["source_offset"] == 78 &&
              e[2]["source_offset"] == 88,
          "layer overrides retain unsigned IDs, duplicate entries and exact offsets");
    check(e[0]["set_property_bits"] == Json({11, 12, 14, 25, 26, 32}) &&
              e[0]["unassigned_property_bits"].empty() &&
              e[1]["unassigned_property_bits"] == Json({0}),
          "layer bitmap ignores out of range packed bits and retains unknown property bits");
    for (auto name : {"color", "line_style", "line_weight", "display", "print", "frozen"})
        check(e[0]["overrides"][name] == true && e[2]["overrides"][name] == false,
              "known override bits and zero length default are decoded independently");
    auto raw = bytesof(e[0]["source_storage"]);
    check(raw.size() == 66 && Reader(raw, 64).u16() == 0xabcd && e[0]["packed_words"].size() == 3 &&
              d["view_visibility"] == "not_evaluated",
          "native expanded word storage retained without interpreting it as visibility");
    for (std::size_t n : {std::size_t(0), std::size_t(4), std::size_t(12), b.size() - 1})
        rejects([&] { decode_attribute(0, 4, slice(b, 0, n), 0); }, "truncated layer bitmap");
    b.push_back(0);
    rejects([&] { decode_attribute(0, 4, b, 0); }, "layer bitmap suffix rejected");
    b = wire_bytes("0100000007000000ffffffff");
    rejects([&] { decode_attribute(0, 4, b, 0); }, "overflowing layer bitmap size rejected");
    check(decode_attribute(0, 4, Bytes(4), 0)["entries"].empty() &&
              decode_attribute(0, 4, Bytes(4), 1)["encoding"] == "opaque",
          "empty layer table and unassigned attribute index remain distinct");
    for (std::uint32_t mode : {0u, 1u, 2u, 0xffffffffu}) {
        Bytes wire;
        put(wire, mode);
        auto sync = decode_attribute(1, 4, wire, 0);
        const char *names[] = {"sync_use_override", "always_unsync", "always_sync"};
        check(sync["sync_state"] == mode &&
                  sync["sync_state_name"] == (mode < 3 ? Json(names[mode]) : Json()),
              "native layer synchronization enum keeps unknown values unnamed");
    }
    rejects([&] { decode_attribute(1, 4, Bytes(3), 0); }, "truncated layer sync state");
    rejects([&] { decode_attribute(1, 4, Bytes(5), 0); }, "layer sync state suffix");
}
static void layer_table_tests() {
    auto set = [](Bytes &b, std::size_t at, auto value) {
        std::memcpy(b.data() + at, &value, sizeof(value));
    };
    auto header = [&](unsigned type, unsigned size, std::uint64_t id) {
        Bytes b(size, 0);
        set(b, 4, std::uint16_t(type));
        set(b, 6, std::uint16_t(type == 49 ? 0x90 : 0x50));
        set(b, 8, std::uint32_t((size - 4) / 2));
        set(b, 12, std::uint32_t((size - 4) / 2));
        set(b, 16, std::uint32_t(1));
        set(b, 20, id);
        return b;
    };
    auto table = header(10, 68, 100);
    set(table, 36, std::uint32_t(2));
    set(table, 44, std::uint64_t(UINT64_MAX - 3));
    auto child = header(49, 108, 101);
    set(child, 36, std::uint32_t(7));
    set(child, 44, std::uint16_t(7));
    auto joined = table;
    joined.insert(joined.end(), child.begin(), child.end());
    set(child, 20, std::uint64_t(102));
    set(child, 36, std::uint32_t(8));
    joined.insert(joined.end(), child.begin(), child.end());
    auto records = parse_native(joined);
    for (auto &n : records)
        n["stream"] = {"ROOT", "SYS", "items"};
    Json index = {{"P3D-SSYS", "SYS"}, {"P3D-SSYSA", "SYSA"}};
    Json overrides = {
        {"group", 0},
        {"key", 4},
        {"index", 0},
        {"decoded",
         {{"encoding", "layer_group_overrides"},
          {"entries", Json::array({{{"layer_id", 8}}, {{"layer_id", 7}}, {{"layer_id", 9}}})}}}};
    Json graphics = Json::array({{{"id", 100},
                                  {"stream", {"ROOT", "SYSA", "attributes"}},
                                  {"attributes", Json::array({overrides})}}});
    auto built = build_layer_tables(index, records, graphics);
    const auto &t = built["tables"][0];
    check(records[0]["layer_table"]["kind"] == "layer_group" &&
              records[0]["layer_table"]["selector"] == UINT64_MAX - 3 &&
              t["member_record_indices"] == Json({1, 2}) &&
              t["attribute_record_indices"] == Json({0}) &&
              built["unassigned_layer_record_indices"].empty(),
          "layer table keeps native membership and scoped attribute identity");
    check(t["override_bindings"][0]["member_record_indices"] == Json({2}) &&
              t["override_bindings"][1]["member_record_indices"] == Json({1}) &&
              t["override_bindings"][2]["status"] == "missing_layer" &&
              t["inheritance_status"] == "not_evaluated",
          "override entries bind by table-local ID instead of array position");
    auto model_header = header(47, 500, 0);
    set(model_header, 16, std::uint32_t(32));
    set(model_header, 492, std::uint64_t(0xfedcba9876543210ull));
    auto reference = parse_native(model_header)[0]["model_layer_group_reference"];
    check(reference["table_id"] == 0xfedcba9876543210ull && reference["status"] == "reference" &&
              reference["source_offset"] == 492,
          "model header references the full-width native layer group table ID");
    set(model_header, 492, std::uint64_t(0));
    check(parse_native(model_header)[0]["model_layer_group_reference"]["status"] == "none",
          "zero model group reference does not select a default or same-name group");
    auto old = header(47, 108, 0);
    set(old, 16, std::uint32_t(32));
    check(parse_native(old)[0]["model_layer_group_reference"]["status"] == "unsupported_header",
          "short model header does not read a table ID from subsequent data");
    Json models = {
        {"18446744073709551615",
         {{"layer_group_references", Json::array({{{"table_id", 100}},
                                                  {{"table_id", 0}},
                                                  {{"table_id", 77}},
                                                  {{"status", "unsupported_header"}}})}}}};
    auto mapped = build_layer_tables(index, records, graphics, models)["model_references"];
    check(
        mapped[0]["model_id"] == "18446744073709551615" &&
            mapped[0]["table_indices"] == Json({0}) && mapped[0]["status"] == "resolved" &&
            mapped[1]["status"] == "none" && mapped[2]["status"] == "missing" &&
            mapped[3]["status"] == "unsupported_header",
        "model references resolve by native table ID with explicit missing and unsupported cases");
    auto foreign = graphics[0];
    foreign["stream"] = {"OTHER", "SYSA", "attributes"};
    graphics.push_back(foreign);
    check(build_layer_tables(index, records, graphics)["tables"][0]["attribute_record_indices"] ==
              Json({0}),
          "same attribute ID in another container is not a table match");
    graphics.push_back(graphics[0]);
    check(build_layer_tables(index, records,
                             graphics)["tables"][0]["override_bindings"][0]["status"] ==
              "ambiguous_attributes",
          "duplicate attribute records remain ambiguous");
    graphics.erase(2);
    records[2]["layer_definition"]["layer_id"] = 7;
    check(build_layer_tables(index, records,
                             graphics)["tables"][0]["override_bindings"][1]["status"] ==
              "ambiguous_layer",
          "duplicate layer IDs retain all candidates");
    records[2]["stream"] = {"OTHER", "SYS", "items"};
    auto invalid = build_layer_tables(index, records, graphics);
    check(invalid["tables"][0]["membership_status"] == "invalid" &&
              invalid["tables"][0]["member_record_indices"].empty() &&
              invalid["unassigned_layer_record_indices"] == Json({1, 2}),
          "table count never consumes records across stream boundaries");
    records[2]["stream"] = records[0]["stream"];
    records[0]["layer_table"]["declared_child_count"] = 0xffffffffu;
    check(build_layer_tables(index, records, graphics)["tables"][0]["membership_status"] ==
              "invalid",
          "untrusted table count does not allocate an oversized member list");
    for (auto selector : {std::uint64_t(0), std::uint64_t(17), std::uint64_t(UINT64_MAX - 1),
                          std::uint64_t(UINT64_MAX), std::uint64_t(UINT64_MAX - 2)}) {
        auto b = table;
        set(b, 44, selector);
        auto d = parse_native(b)[0]["layer_table"];
        check(d["selector"] == selector &&
                  d["kind"] == (selector == 0                ? "local"
                                : selector == 17             ? "model_link"
                                : selector == UINT64_MAX - 1 ? "nested_model_link"
                                                             : "unassigned"),
              "table selector preserves native sentinels without guessed names");
    }
    // A path containing 70 IDs requires the extended native linkage header.
    auto path_table = table;
    set(path_table, 44, std::uint64_t(UINT64_MAX - 1));
    Bytes link;
    put<std::uint16_t>(link, 0x5190); // 144 * 2 words = 576 bytes
    put<std::uint16_t>(link, 0x56f1);
    put<std::uint32_t>(link, 0);
    put<std::uint16_t>(link, 1);
    put<std::uint16_t>(link, 0);
    put<std::uint32_t>(link, 70);
    for (unsigned i = 0; i < 70; ++i)
        put<std::uint64_t>(link, i % 3 ? 42 : UINT64_MAX);
    path_table.insert(path_table.end(), link.begin(), link.end());
    set(path_table, 8, std::uint32_t((path_table.size() - 4) / 2));
    auto path = parse_native(path_table)[0];
    check(path["links"].size() == 1 && path["layer_table"]["path_status"] == "decoded" &&
              path["layer_table"]["linkage_paths"][0]["entry_ids"].size() == 70 &&
              path["layer_table"]["linkage_paths"][0]["entry_ids"][0] == UINT64_MAX &&
              path["layer_table"]["linkage_paths"][0]["entry_ids"][69] == UINT64_MAX,
          "extended link header decodes long nested paths with unsigned IDs and repeats");
    auto malformed = path_table;
    set(malformed, 68 + 12, std::uint32_t(71));
    auto d = parse_native(malformed)[0]["layer_table"];
    check(d["path_status"] == "decode_error" && d["linkage_paths"][0].contains("decode_error"),
          "path internal length error stays attached to its source linkage");
    for (unsigned h : {0x5000, 0x5fff}) {
        auto b = path_table;
        set(b, 68, std::uint16_t(h));
        rejects([&] { parse_native(b); }, "zero and overflowing extended link lengths rejected");
    }
    auto fixed = table;
    put<std::uint16_t>(fixed, 0x0020); // non-user native link is always four words
    put<std::uint16_t>(fixed, 0x1234);
    put<std::uint32_t>(fixed, 0xfedcba98);
    set(fixed, 8, std::uint32_t((fixed.size() - 4) / 2));
    check(parse_native(fixed)[0]["links"][0]["payload"]["bytes"] == 4,
          "fixed native linkage length does not use low-byte user-link encoding");
    auto legacy = header(49, 788, 200);
    set(legacy, 16, std::uint32_t(5));
    legacy.resize(804);
    set(legacy, 8, std::uint32_t(400));
    set(legacy, 764, std::uint16_t(0x100f));
    set(legacy, 766, std::uint16_t(0x56d2));
    set(legacy, 768, std::uint16_t(1));
    set(legacy, 772, std::uint32_t(14));
    const Bytes name = {0xff, 0xfe, 1, 0, 'A', 'n', 'n', 'o', 't', 'a', 't', 'i', 'v', 'e'};
    std::copy(name.begin(), name.end(), legacy.begin() + 776);
    set(legacy, 796, std::uint16_t(0x1003));
    set(legacy, 798, std::uint16_t(0x56de));
    auto recovered = parse_native(legacy)[0];
    check(recovered["base_boundary_adjustment"] == 8 && recovered["links"].size() == 1 &&
              recovered["links"][0]["app"] == 0x56de,
          "legacy crossing string tail is not interpreted as a fixed linkage");
    auto legacy_fixed = header(49, 788, 201);
    set(legacy_fixed, 16, std::uint32_t(5));
    legacy_fixed.insert(legacy_fixed.end(), fixed.end() - 8, fixed.end());
    set(legacy_fixed, 8, std::uint32_t(396));
    check(parse_native(legacy_fixed)[0]["base_boundary_adjustment"] == 0 &&
              parse_native(legacy_fixed)[0]["links"][0]["app"] == 0x1234,
          "valid fixed linkage at legacy base length keeps its original boundary");
}
static void layer_group_state_tests() {
    Json index = {{"P3D-SSYS", "SYS"}, {"P3D-SSYSA", "SYSA"}};
    auto table = [](const char *scope, unsigned count, std::uint64_t id, const char *kind) {
        return Json{{"stream", {"ROOT", scope, "items"}},
                    {"id", id},
                    {"offset", 0},
                    {"length", 68},
                    {"layer_table", {{"declared_child_count", count}, {"kind", kind}}}};
    };
    auto layer = [](const char *scope, unsigned id, unsigned ordinal, unsigned value) {
        return Json{
            {"stream", {"ROOT", scope, "items"}},
            {"id", 100 + ordinal},
            {"offset", 68 + ordinal * 108},
            {"length", 108},
            {"element_flags", 0x80},
            {"layer_definition",
             {{"status", "decoded"},
              {"layer_id", id},
              {"by_layer_symbology",
               {{"color_index", value}, {"line_style", value + 1}, {"line_weight", value + 2}}},
              {"display", value == 10},
              {"print", value == 10},
              {"frozen", value == 10},
              {"transparency", value / 100.0}}}};
    };
    Json records =
        Json::array({table("SYS", 3, 5, "local"), layer("SYS", 1, 0, 10), layer("SYS", 2, 1, 10),
                     layer("SYS", 3, 2, 10), table("SYS", 2, 900, "layer_group"),
                     layer("SYS", 2, 0, 20), layer("SYS", 99, 1, 20)});
    for (std::size_t i = 4; i < records.size(); ++i)
        records[i]["offset"] = records[i]["offset"].get<unsigned>() + 392;
    const auto original = records;
    const std::vector<unsigned> bits = {11, 12, 14, 25, 26, 32, 35};
    const std::vector<std::string> names = {"color_index", "line_style", "line_weight", "display",
                                            "print",       "frozen",     "transparency"};
    auto graphics = [&](unsigned mode, unsigned mask) {
        Bytes payload;
        put<std::uint32_t>(payload, 1);
        put<std::uint32_t>(payload, 2);
        put<std::uint32_t>(payload, 36);
        payload.resize(12 + 36 * 2);
        for (unsigned i = 0; i < bits.size(); ++i)
            if (mask & (1u << i))
                payload[12 + bits[i] / 8] |= 1u << (bits[i] % 8);
        Bytes sync;
        put(sync, mode);
        return Json::array(
            {{{"id", 900},
              {"stream", {"ROOT", "SYSA", "attributes"}},
              {"attributes",
               Json::array({{{"group", 0},
                             {"key", 4},
                             {"index", 0},
                             {"decoded", decode_layer_group_attribute(0, payload)}},
                            {{"group", 1},
                             {"key", 4},
                             {"index", 0},
                             {"decoded", decode_layer_group_attribute(1, sync)}}})}}});
    };
    for (unsigned mode = 0; mode < 3; ++mode)
        for (unsigned mask = 0; mask < 128; ++mask) {
            const auto source_attrs = graphics(mode, mask);
            const auto &bitmap = source_attrs[0]["attributes"][0]["decoded"]["entries"][0];
            check(bitmap["overrides"]["transparency"] == bool(mask & 64) &&
                      bitmap["unassigned_property_bits"].empty(),
                  "transparency bit is named in the source override bitmap");
            const auto state = build_layer_tables(index, records, source_attrs)["group_states"][0];
            check(state["status"] == "evaluated_supported_properties" &&
                      state["layers"].size() == 3 &&
                      state["excluded_group_record_indices"] == Json({6}),
                  "all sync modes reconcile group membership against the file layer table");
            for (unsigned i = 0; i < bits.size(); ++i) {
                bool file = mode == 2 || (mode == 0 && !(mask & (1u << i)));
                const auto &p = state["layers"][1]["properties"][names[i]];
                auto source = file ? 2 : 5;
                const auto &v = records[source]["layer_definition"];
                check(p["native_record_index"] == source &&
                          p["value"] == (i < 3 ? v["by_layer_symbology"][names[i]] : v[names[i]]) &&
                          state["layers"][0]["properties"][names[i]]["selection"] ==
                              "new_file_layer",
                      "seven property choices follow sync mode and independent override bits");
            }
        }
    check(records == original, "derived layer synchronization never mutates native records");
    auto attrs = graphics(0, 0);
    attrs[0]["attributes"] = Json::array();
    auto state = build_layer_tables(index, records, attrs)["group_states"][0];
    check(state["sync_state_source"] == "native_default" &&
              state["layers"][1]["properties"]["display"]["native_record_index"] == 2,
          "absent synchronization attributes use native mode zero without overrides");
    attrs = graphics(0, 1);
    auto &entries = attrs[0]["attributes"][0]["decoded"]["entries"];
    entries.push_back(entries[0]);
    state = build_layer_tables(index, records, attrs)["group_states"][0];
    check(state["status"] == "partially_evaluated" && state["layers"][1]["properties"].empty() &&
              state["layers"][0]["status"] == "evaluated_supported_properties",
          "duplicate override entries affect only the matching layer");
    attrs[0]["attributes"][1]["decoded"]["sync_state"] = 2;
    check(build_layer_tables(index, records, attrs)["group_states"][0]["status"] ==
              "evaluated_supported_properties",
          "always-sync does not consult otherwise ambiguous override entries");
    check(build_layer_tables(index, records, graphics(77, 0))["group_states"][0]["status"] ==
              "not_evaluated",
          "unknown sync mode remains explicit instead of guessing policy");
    auto duplicate = records;
    duplicate[3]["layer_definition"]["layer_id"] = 2;
    state = build_layer_tables(index, duplicate, graphics(2, 0))["group_states"][0];
    check(state["status"] == "partially_evaluated" &&
              state["layers"][1]["file_record_indices"] == Json({2, 3}),
          "duplicate file layer IDs retain candidates without guessed resolution");
    duplicate = records;
    auto foreign = records[0];
    foreign["stream"] = {"OTHER", "SYS", "items"};
    duplicate.push_back(foreign);
    check(build_layer_tables(index, duplicate,
                             graphics(2, 0))["group_states"][0]["file_table_indices"] == Json({0}),
          "file table candidates remain within the group container scope");
    duplicate.push_back(records[0]);
    check(build_layer_tables(index, duplicate, graphics(2, 0))["group_states"][0]["status"] ==
              "not_evaluated",
          "multiple local file tables do not silently select the first");
    attrs = graphics(0, 0);
    attrs[0]["attributes"][0]["decoded"] = {{"decode_error", "bad data"}};
    check(build_layer_tables(index, records, attrs)["group_states"][0]["status"] == "not_evaluated",
          "malformed overrides never become an empty all-sync mask");
}
static void material_index_tests() {
    auto decode = [](const std::string &s, unsigned index) {
        return decode_attribute(4, 10001, Bytes(s.begin(), s.end()), index);
    };
    auto j = decode(
        u8R"({"-2147483648":"材质","2147483647":"钢","01":"wood","-0":"a","2147483648":"b","2":17})",
        1);
    check(j["index_shape_status"] == "unexpected_entries" && j["entries"].size() == 6,
          "material index decodes good members even with malformed neighboring entries");
    unsigned valid = 0;
    for (auto &e : j["entries"])
        if (e["status"] == "recognized") {
            ++valid;
            check(e["part_index"] == -2147483648ll || e["part_index"] == 2147483647,
                  "part key uses exact signed int32 decimal formatting");
        }
    check(valid == 2 && j["geometry_mapping_status"] == "not_established",
          "part indices neither accept aliases nor imply geometric mapping");
    j = decode(u8R"({"木":{"19":"19","2":"3","-1":"-1"},"钢":[19]})", 2);
    check(j["entries"].size() == 3 && j["unrecognized_material_groups"].size() == 1 &&
              j["entries"][0]["material_name"] == u8"木" &&
              j["entries"][2]["source_value"] == "3" &&
              j["entries"][2]["repeated_index_matches_key"] == false,
          "native reverse index is material name to object of repeated decimal index strings");
    check(decode("[]", 1)["index_shape_status"] == "unexpected_shape" &&
              decode("{}", 2)["index_shape_status"] == "recognized" &&
              !decode("{}", 9).contains("entries"),
          "unsupported material shapes and roles are not assigned invented entries");
    j = decode(R"({"2":"first","2":"last"})", 1);
    check(j["entries"].size() == 1 && j["entries"][0]["material_name"] == "last" &&
              j["text"] == R"({"2":"first","2":"last"})",
          "typed material view follows parsed JSON while preserving duplicate source keys");
}
static void attribute_semantics_tests() {
    Bytes b;
    put<std::uint32_t>(b, 0xfedcba98);
    check(decode_attribute(0, 22634, b)["display_style_entry_id"] == 0xfedcba98u,
          "display style reference preserves unsigned id");
    for (unsigned flags = 0; flags < 16; ++flags) {
        b.clear();
        put<std::uint32_t>(b, 19);
        for (unsigned i = 0; i < 4; ++i) {
            put<std::uint32_t>(b, flags);
            put<std::uint32_t>(b, 100 + i);
        }
        auto d = decode_attribute(0, 22626, b);
        check(d["header_word"] == 19, "clip header is retained without guessed semantics");
        unsigned i = 0;
        for (auto name : {"forward", "back", "cut", "outside"}) {
            const auto &v = d["regions"][name];
            check(v["display"] == bool(flags & 1) && v["snap"] == !(flags & 2) &&
                      v["locate"] == !(flags & 4) && v["unassigned_flag_bits"] == (flags & ~7u) &&
                      v["display_style_entry_id"] == 100 + i++,
                  "clip display flag is direct, snap and locate flags are inverted");
        }
    }
    for (std::size_t n = 0; n < b.size(); ++n)
        rejects([&] { decode_attribute(0, 22626, slice(b, 0, n)); }, "truncated clip settings");
    b.push_back(0);
    rejects([&] { decode_attribute(0, 22626, b); }, "unexpected clip suffix");
    b.clear();
    put<std::uint32_t>(b, 0);
    put<std::uint32_t>(b, 77);
    put<std::int32_t>(b, -1);
    for (double x : {12., -34., 56., 2.5, 0., -1., 0., 1., 0., 0., 0., 0., 1.})
        put(b, x);
    put<std::uint64_t>(b, 0xfedcba9876543210ull);
    check(b.size() == 124, "ACS ordinary layout width");
    auto d = decode_attribute(1, 22295, b);
    check(d["origin"] == Json({12., -34., 56.}) && d["scale"] == 2.5 &&
              d["rotation"] == Json({{0., -1., 0.}, {1., 0., 0.}, {0., 0., 1.}}) &&
              d["element_id"] == 0xfedcba9876543210ull && d["coordinate_system_type"] == 77 &&
              d["view_independent"] == false && d["view_independent_value"] == -1 &&
              d["extra_data_status"] == "absent",
          "ACS preserves nonidentity frame, unknown enum, signed flag and 64 bit identity");
    for (std::size_t n = 0; n < b.size(); ++n)
        rejects([&] { decode_attribute(1, 22295, slice(b, 0, n)); }, "truncated ACS");
    Bytes tail(45, 0xa7);
    b.insert(b.end(), tail.begin(), tail.end());
    d = decode_attribute(1, 22295, b);
    check(d["extra_data_status"] == "opaque" && d["extra_data_offset"] == 124 &&
              bytesof(d["extra_data"]) == tail,
          "ACS retains entire unknown suffix");
    b[0] = 1;
    rejects([&] { decode_attribute(1, 22295, b); }, "unsupported ACS header");

    b.clear();
    for (auto c : std::u16string(u"\u6750\u8d28\U0001f332"))
        put<std::uint16_t>(b, c);
    d = decode_attribute(4, 10001, b, 0);
    check(d["material_name"] == u8"材质🌲", "unterminated Unicode material name");
    check(decode_attribute(2, 10001, b, 31)["material_name"] == u8"材质🌲" &&
              decode_attribute(2, 10001, b, 31)["encoding"] == "legacy_part_material_name",
          "legacy per-part names use the same unterminated UTF16 representation");
    rejects([&] { decode_attribute(4, 10001, Bytes{0x00}, 0); }, "odd material name size");
    rejects([&] { decode_attribute(4, 10001, Bytes{0x00, 0xdc}, 0); }, "bad material surrogate");
    for (unsigned index : {1u, 2u, 9u}) {
        std::string text = index == 1   ? u8"{\"-1\":\"材质\",\"19\":\"wood\",\"19\":\"steel\"}"
                           : index == 2 ? "{\"wood\":{\"19\":\"19\"}}"
                                        : "[1,true,\"unknown\"]";
        Bytes raw(text.begin(), text.end());
        auto j = decode_attribute(4, 10001, raw, index);
        check(j["json_status"] == "parsed" && j["text"] == text &&
                  bytesof(j["source_bytes"]) == raw && j["value"] == Json::parse(text),
              "indexed material JSON retains duplicate keys and exact source text");
        if (index == 9)
            check(j["role"] == "unassigned", "unknown material index gets no invented role");
    }
    check(decode_attribute(4, 10001, Bytes{'{'}, 1)["json_status"] == "invalid_json",
          "damaged material JSON remains available as text");
    rejects([&] { decode_attribute(4, 10001, Bytes{0xff}, 1); }, "bad material UTF8");
    Bytes wire, attrs;
    std::vector<Bytes> payloads = {b, Bytes{'{', '"', '4', '"', ':', '"', 'a', '"', '}'},
                                   Bytes{0xff}, Bytes{'{', '}'}};
    for (unsigned i = 0; i < payloads.size(); ++i) {
        put<std::uint16_t>(attrs, 4);
        put<std::uint16_t>(attrs, 10001);
        put<std::uint32_t>(attrs, i);
        put<std::uint32_t>(attrs, payloads[i].size());
        put<std::uint32_t>(attrs, 0);
        attrs.insert(attrs.end(), payloads[i].begin(), payloads[i].end());
    }
    put<std::uint32_t>(wire, 0xa11b);
    put<std::uint32_t>(wire, attrs.size() + 4);
    put<std::uint64_t>(wire, 0);
    put<std::uint64_t>(wire, 17);
    put<std::uint32_t>(wire, payloads.size());
    wire.insert(wire.end(), attrs.begin(), attrs.end());
    put<std::uint32_t>(wire, 0);
    const auto parsed = parse_graphics(wire)[0]["attributes"];
    check(parsed[0]["decoded"]["material_name"] == u8"材质🌲" &&
              parsed[1]["decoded"]["value"]["4"] == "a" &&
              parsed[2]["decoded"].contains("decode_error") &&
              bytesof(parsed[2]["payload"]) == payloads[2] &&
              parsed[3]["decoded"]["json_status"] == "parsed" &&
              parsed[3]["decoded"]["role"] == "unassigned",
          "record reader routes by attribute index and recovers after damaged material JSON");
    Json a = {{"group", 4},  {"key", 10001},           {"index", 0},
              {"offset", 8}, {"payload", rawbytes(b)}, {"decoded", d}};
    Json g = {{"stream", {"root", "model1"}},
              {"id", 7},
              {"offset", 40},
              {"attributes", Json::array({a})}};
    Json h = g;
    h["stream"] = {"root", "model2"};
    h["attributes"][0]["decoded"] = {{"encoding", "opaque"}, {"decode_error", "bad"}};
    auto refs = material_assignment_records(Json::array({g, g, h}));
    check(refs.size() == 3 && refs[0]["stream"] != refs[2]["stream"] &&
              refs[2]["decoded"]["encoding"] == "opaque" && refs[0]["attribute_ordinal"] == 0 &&
              refs[0]["attribute_index"] == 0 && bytesof(refs[0]["payload"]) == b,
          "material assignments preserve duplicates, scope and damaged source records");
    h["attributes"][0]["group"] = 2;
    h["attributes"][0]["index"] = 31;
    refs = material_assignment_records(Json::array({g, h}));
    check(refs.size() == 2 && refs[1]["group"] == 2 && refs[1]["attribute_index"] == 31,
          "legacy and advanced assignment sources remain distinct");
    Bytes header(108, 0);
    header[36] = 0;
    header[37] = 0x14;
    auto state = native_display_state(97, header);
    check(state["permanently_invisible"] == true && state["unassigned_bits"] == 0x1000 &&
              state["source_offset"] == 36 && state["view_visibility"] == "not_evaluated",
          "permanent invisibility is a display header bit, not a computed view result");
    header[37] = 0x10;
    check(native_display_state(37, header)["permanently_invisible"] == false &&
              native_display_state(33, header)["status"] == "unsupported_header" &&
              native_display_state(97, Bytes(36))["status"] == "unsupported_header",
          "non-graphic and short headers are not interpreted using a coincidental byte value");
}
static void section_clip_tests() {
    Bytes b;
    put<std::uint32_t>(b, 0x80000055u); // Left/front/top crop and perspective-up.
    put<std::uint32_t>(b, 0xfedcba98u);
    for (double x : {1.5, 2.5, 3.5, 4.5, 1., 2., 3., 4., 5., 6., 7., 8., 9.})
        put(b, x);
    auto d = decode_attribute(0, 109, b);
    check(b.size() == 112 && d["crop"]["left"] == true && d["crop"]["right"] == false &&
              d["crop"]["front"] == true && d["crop"]["back"] == false &&
              d["crop"]["top"] == true && d["crop"]["bottom"] == false &&
              d["perspective_up"] == true && d["unassigned_flag_bits"] == 0x80000000u &&
              d["unassigned_word"] == 0xfedcba98u,
          "section clipping distinguishes six directions and preserves unknown flag bits");
    check(d["top_height"] == 1.5 && d["bottom_height"] == 2.5 && d["front_depth"] == 3.5 &&
              d["back_depth"] == 4.5,
          "section heights and depths keep their independent wire order");
    for (unsigned i = 0; i < 3; ++i) {
        check(d["serialized_rotation"][i] == Json({1. + 3 * i, 2. + 3 * i, 3. + 3 * i}) &&
                  std::abs(d["rotation"][i][0].get<double>() - (1. + 3 * i)) < 1e-12 &&
                  std::abs(d["rotation"][i][1].get<double>() - (3. + 3 * i)) < 1e-12 &&
                  std::abs(d["rotation"][i][2].get<double>() + (2. + 3 * i)) < 1e-12,
              "section rotation is right-multiplied by positive X rotation, not transposed");
    }
    for (auto size : {0u, 4u, 7u, 8u, 39u, 40u, 103u, 111u})
        rejects([&] { decode_attribute(0, 109, slice(b, 0, size)); },
                "truncated section clip data");
    b.push_back(0);
    rejects([&] { decode_attribute(0, 109, b); },
            "unknown section clip suffix retains opaque payload");
    b.clear();
    for (double x : {12., -34., 56., 0., 0., -2.})
        put(b, x);
    d = decode_attribute(1, 109, b);
    check(d["origin"] == Json({12., -34., 56.}) && d["direction"] == Json({0., 0., -2.}),
          "section frame direction is preserved without normalization");
    b.pop_back();
    rejects([&] { decode_attribute(1, 109, b); }, "truncated section frame");
}
static void inline_material_tests() {
    Bytes b;
    auto text = [&](const std::u16string &s) {
        put<std::uint64_t>(b, s.size() * 2);
        for (auto c : s)
            put<std::uint16_t>(b, c);
    };
    put<std::uint8_t>(b, 0); // isValid, not a format version.
    text(u"material");
    put<std::uint8_t>(b, 0);
    for (double v : {0.1, 0.2, 0.3})
        put(b, v);
    put<std::uint8_t>(b, 1);
    put<double>(b, 0.4);
    put<std::uint8_t>(b, 0);
    auto enum_offset = b.size();
    put<std::int32_t>(b, 3);
    put<std::int32_t>(b, 6);
    text(u"relative/color.jpg");
    for (double v : {2., 3., 4., 5., 90.})
        put(b, v);
    text(u"relative/\u51f9\u51f8.jpg");
    put<double>(b, 0.75);
    for (int i = 0; i < 9; ++i) {
        put<std::uint8_t>(b, i % 2);
        for (int j = 0; j < (i == 0 || i == 2 ? 3 : 1); ++j)
            put<double>(b, 10 * i + j + 0.5);
    }
    const auto base = b;
    auto m = decode_inline_material(b);
    check(m["is_valid"] == false && !m.contains("version"),
          "inline material first byte is validity, not version");
    check(m["parameters"][0]["enabled"] == false &&
              m["parameters"][0]["value"] == Json({0.1, 0.2, 0.3}) &&
              m["parameters"][1]["kind"] == "transparency",
          "disabled material values are still consumed and retained");
    check(m["map_unit_name"] == "absolute" && m["map_mode_name"] == "cylindrical" &&
              m["has_map"] == false && m["uv_scale"] == Json({2., 3.}) &&
              m["uv_offset"] == Json({4., 5.}) && m["rotation_degrees"] == 90.,
          "material mapping enums and two dimensional UV fields");
    check(m["texture_references"][1]["filename"] == u8"relative/凹凸.jpg" &&
              m["bump_factor"] == 0.75 && m["parameters"][10]["value"] == 80.5,
          "nonempty bump filename does not shift following shader values");
    check(m["display_name"] == "material" && m["display_name_source"] == "name_fallback",
          "legacy material without display extension inherits name");
    put<std::uint32_t>(b, 0xabcd);
    text(u"\u663e\u793a\u540d\u79f0");
    m = decode_inline_material(b);
    check(m["display_name"] == u8"显示名称" && m["display_name_source"] == "serialized",
          "display extension reads its length prefixed Unicode name");
    for (std::size_t n = 0; n < b.size(); ++n) {
        if (n == base.size())
            continue;
        rejects([&]() { decode_inline_material(slice(b, 0, n)); },
                "truncated material must not silently succeed");
    }
    auto bad = b;
    bad[0] = 2;
    rejects([&]() { decode_inline_material(bad); }, "nonboolean validity rejected");
    auto unknown = b;
    std::int32_t code = -123;
    std::memcpy(unknown.data() + enum_offset, &code, 4);
    std::memcpy(unknown.data() + enum_offset + 4, &code, 4);
    m = decode_inline_material(unknown);
    check(m["map_unit"] == -123 && m["map_mode"] == -123 &&
              m["map_unit_status"] == "unknown_value" && !m.contains("map_mode_name"),
          "unknown mapping enum values retained without coercion");
    for (std::int32_t unit : {0, 3}) {
        for (std::int32_t mode : {0, 1, 2, 4, 5, 6}) {
            auto mapped = b;
            std::memcpy(mapped.data() + enum_offset, &unit, 4);
            std::memcpy(mapped.data() + enum_offset + 4, &mode, 4);
            m = decode_inline_material(mapped);
            check(m["map_unit"] == unit && m["map_mode"] == mode &&
                      m["map_unit_status"] == "identified" && m["map_mode_status"] == "identified",
                  "all declared native material mapping codes are accepted");
        }
    }
    const auto display = b;
    auto extension = [&](std::u16string value) {
        b = display;
        put<std::uint32_t>(b, 0xabce);
        value.push_back(0);
        text(value);
        return decode_inline_material(b).at("extended_data");
    };
    auto ext = extension(u"{\"custom\":{\"标签\":\"保留\",\"values\":[null,true,42]}}");
    check(ext["json_status"] == "parsed" && ext["value"]["custom"][u8"标签"] == u8"保留" &&
              ext["value"]["custom"]["values"] == Json({nullptr, true, 42}),
          "material JSON extension retains arbitrary nested properties");
    auto valid_extension = b;
    auto raw_extension = bytesof(ext["source_bytes"]);
    check(raw_extension.size() >= 2 && raw_extension.back() == 0 &&
              utf16(slice(raw_extension, 0, raw_extension.size() - 2)) == ext["text"],
          "material JSON preserves original terminated UTF16 separately from its parsed view");
    for (std::size_t n = display.size() + 4; n < b.size(); ++n)
        rejects([&]() { decode_inline_material(slice(valid_extension, 0, n)); },
                "recognized JSON block must not accept a truncated size or payload");
    for (std::uint64_t n : {0ull, 1ull, 3ull, ~0ull}) {
        auto damaged = valid_extension;
        std::memcpy(damaged.data() + display.size() + 4, &n, sizeof(n));
        rejects([&]() { decode_inline_material(damaged); }, "invalid JSON UTF16 byte count");
    }
    auto unterminated = valid_extension;
    unterminated.back() = 1;
    rejects([&]() { decode_inline_material(unterminated); }, "JSON extension terminator required");
    check(extension(u"")["json_status"] == "empty", "empty material JSON extension");
    ext = extension(u"{unrecognized future syntax}");
    check(ext["json_status"] == "invalid_json" && !ext.contains("value") &&
              ext["text"] == "{unrecognized future syntax}",
          "invalid JSON text remains available without inventing semantic fields");
    ext = extension(u"{\"x\":1,\"x\":2}");
    check(ext["text"] == "{\"x\":1,\"x\":2}",
          "duplicate JSON keys remain in authoritative source text");
    b = valid_extension;
    b.push_back(0x5a);
    check(decode_inline_material(b)["unassigned_suffix_hex"] == "5a",
          "unknown suffix after JSON remains unassigned");
    b = display;
    b.push_back(0x5a);
    check(decode_inline_material(b)["unassigned_suffix_hex"] == "5a",
          "future material extension remains explicitly unassigned");
}
static void embedded_texture_tests() {
    // Public PNG file bytes are never decoded or re-encoded by the parser.
    const auto file = unbase64("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/"
                               "x8AAwMCAO+aX1sAAAAASUVORK5CYII=");
    Bytes payload(256, 0x7b);
    std::u16string name = u"../原图.png";
    name.push_back(0);
    for (std::size_t i = 0; i < name.size(); ++i) {
        payload[2 * i] = name[i] & 255;
        payload[2 * i + 1] = name[i] >> 8;
    }
    put<std::uint32_t>(payload, file.size());
    put<std::uint32_t>(payload, 0x12340021);
    payload.insert(payload.end(), file.begin(), file.end());
    auto envelope = [](const Bytes &raw, unsigned version) {
        Bytes b;
        put<std::uint32_t>(b, version);
        put<std::uint32_t>(b, raw.size());
        if (version != 3)
            b.insert(b.end(), raw.begin(), raw.end());
        else {
            Bytes compressed(LZ4_compressBound(int(raw.size())));
            auto n = LZ4_compress_default(reinterpret_cast<const char *>(raw.data()),
                                          reinterpret_cast<char *>(compressed.data()),
                                          int(raw.size()), int(compressed.size()));
            require(n > 0, "fixture compression");
            b.insert(b.end(), compressed.begin(), compressed.begin() + n);
        }
        return b;
    };
    for (unsigned version : {1u, 2u, 3u}) {
        auto b = envelope(payload, version);
        auto t = decode_attribute(33, 22913, b);
        check(t["encoding"] == "embedded_texture_file" && t["version"] == version &&
                  t["filename"] == u8"../原图.png" && t["file_offset"] == 264 &&
                  t["native_map_type"] == 0x12340021 &&
                  t["attribute_group_matches_map_type"] == true,
              "embedded image header has source name and full native map type");
        check(
            bytesof(t["file_data"]) == file && t["file_bytes"] == file.size() &&
                bytesof(t["filename_field"]) == slice(payload, 0, 256) && !t.contains("data"),
            "image bytes and unused filename capacity preserved without duplicating decoded file");
        check(decode_attribute(1, 22913, b)["attribute_group_matches_map_type"] == false,
              "map type mismatch is reported without coercing stored values");
        auto rebuilt = bytesof(t["filename_field"]);
        put<std::uint32_t>(rebuilt, t["file_bytes"]);
        put<std::uint32_t>(rebuilt, t["native_map_type"]);
        auto decoded_file = bytesof(t["file_data"]);
        rebuilt.insert(rebuilt.end(), decoded_file.begin(), decoded_file.end());
        check(rebuilt == payload, "all embedded payload bytes accounted for");
        b.pop_back();
        rejects([&]() { decode_attribute(33, 22913, b); }, "truncated embedded file envelope");
    }
    for (std::size_t n = 0; n < payload.size(); ++n) {
        auto b = envelope(slice(payload, 0, n), 1);
        rejects([&]() { decode_attribute(33, 22913, b); }, "truncated image header or file");
    }
    auto bad = payload;
    std::fill(bad.begin(), bad.begin() + 256, 1);
    rejects([&]() { decode_attribute(33, 22913, envelope(bad, 1)); },
            "unterminated filename is not allowed to consume file size bytes");
    for (std::uint32_t n : {0u, 0xffffffffu}) {
        bad = payload;
        std::memcpy(bad.data() + 256, &n, 4);
        rejects([&]() { decode_attribute(33, 22913, envelope(bad, 1)); },
                "invalid embedded file byte count");
    }
    bad = payload;
    bad.push_back(0);
    rejects([&]() { decode_attribute(33, 22913, envelope(bad, 1)); },
            "unexpected trailing image bytes are not discarded");
    for (unsigned version : {1u, 2u}) {
        auto a = decode_attribute(80, 99, envelope(Bytes({'a', 'b', 'c'}), version));
        check(a["encoding"] == "uncompressed_binary" && a["codec"] == "stored" &&
                  bytesof(a["data"]) == Bytes({'a', 'b', 'c'}),
              "stored attribute envelopes are decoded without decompression");
        Bytes xml;
        for (char16_t c : std::u16string(u"<Material Filename=\"relative.jpg\"/>"))
            put<std::uint16_t>(xml, c);
        a = decode_attribute(0, 20014, envelope(xml, version));
        check(a["encoding"] == "uncompressed_utf16_xml" && a["tree"]["tag"] == "Material",
              "stored UTF16 XML attributes use the same semantic decoder");
    }
    for (unsigned version : {1u, 2u, 3u}) {
        Bytes text;
        for (char16_t c : std::u16string(u"任意属性"))
            put<std::uint16_t>(text, c);
        put<std::uint16_t>(text, 0);
        auto a = decode_attribute(123, 2006, envelope(text, version));
        check(a["text"] == u8"任意属性" && bytesof(a["data"]) == text,
              "native string attributes retain generic Unicode values and original termination");
        check(decode_attribute(123, 2006, envelope(Bytes{0, 0}, version))["text"] == "",
              "empty native string attribute");
        text.pop_back();
        rejects([&]() { decode_attribute(123, 2006, envelope(text, version)); },
                "odd UTF16 string attribute width");
        text.push_back(1);
        rejects([&]() { decode_attribute(123, 2006, envelope(text, version)); },
                "unterminated UTF16 string attribute");
    }
    // Exercise the public record reader's recovery, followed by source-scoped material binding.
    auto record = [&](const Bytes &b, std::uint64_t id) {
        Bytes r;
        put<std::uint32_t>(r, 0xa11b);
        put<std::uint32_t>(r, 20 + b.size());
        put<std::uint64_t>(r, 0);
        put<std::uint64_t>(r, id);
        put<std::uint32_t>(r, 1);
        put<std::uint16_t>(r, 33);
        put<std::uint16_t>(r, 22913);
        put<std::uint32_t>(r, 7);
        put<std::uint32_t>(r, b.size());
        put<std::uint32_t>(r, 0);
        r.insert(r.end(), b.begin(), b.end());
        put<std::uint32_t>(r, 0);
        return parse_graphics(r)[0];
    };
    auto g = record(envelope(payload, 3), 42);
    g["stream"] = {"root", "A", "attributes"};
    auto damaged = record(envelope(bad, 1), 42);
    damaged["stream"] = {"root", "B", "attributes"};
    check(damaged["attributes"][0]["decoded"].contains("decode_error") &&
              bytesof(damaged["attributes"][0]["payload"]) == envelope(bad, 1),
          "failed image parsing preserves the entire original attribute");
    Json materials = {{"definitions", Json::array({{{"stream", g["stream"]}, {"id", 42}}})}};
    auto files = embedded_texture_records(Json::array({g, damaged}), materials);
    check(files.size() == 2 && files[0]["material_candidates"] == Json({0}) &&
              files[0]["material_status"] == "resolved" &&
              files[1]["material_status"] == "missing" && files[0]["attribute_index"] == 7 &&
              files[0]["attribute_ordinal"] == 0,
          "embedded files bind by stream and record identity, never by filename or bare ID");
    materials["definitions"].push_back(materials["definitions"][0]);
    files = embedded_texture_records(Json::array({g, g}), materials);
    check(files.size() == 2 && files[0]["material_candidates"] == Json({0, 1}) &&
              files[0]["material_status"] == "ambiguous",
          "duplicate records and ambiguous material definitions remain separate");
}
static Json command(unsigned op, const Bytes &body, std::size_t offset = 2) {
    return {{"op", op},
            {"offset", offset},
            {"body", rawbytes(body)},
            {"decoded", command_fields(op, body)}};
}
static void box_center_tests() {
    Bytes body;
    for (double v : {0., 2., 0., -3., 0., 0., 10., 20., 30., 14., 25., 40., 4., 6., 2., 8.})
        put<double>(body, v);
    put<std::uint8_t>(body, 1);
    auto cmd = command(30, body);
    check(cmd["decoded"]["origin_convention"] == "face_centers",
          "box stream origins are face centers");
    auto g = reconstruct(Json::array({cmd}), Tessellation{});
    check(g.unknown.empty() && g.vertices.size() == 10 && g.faces.size() == 12,
          "capped box has four sides and two complete caps");
    // Independently calculated corners for unequal, offset faces and nonunit rotated axes.
    const std::vector<Point3> expected = {
        {19., 16., 30.}, {19., 24., 30.}, {1., 24., 30.}, {1., 16., 30.}, {19., 16., 30.},
        {26., 23., 40.}, {26., 27., 40.}, {2., 27., 40.}, {2., 23., 40.}, {26., 23., 40.}};
    check(g.vertices == expected, "box centers do not drift by half a face width");
    body.back() = 0;
    g = reconstruct(Json::array({command(30, body)}), Tessellation{});
    check(g.faces.size() == 8 && g.vertices == expected,
          "uncapped box preserves its two open ends");
}
static void guided_surface_tests() {
    const std::vector<Point3> corners = {{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}};
    auto fixture = [&](bool capped, bool twisted, bool damaged, double bend_height = 1.) {
        Json commands = Json::array({command(56, Bytes{std::uint8_t(capped)})});
        auto polyline = [&](const std::vector<Point3> &p) {
            Bytes raw;
            put<std::uint32_t>(raw, p.size());
            for (const auto &v : p)
                for (auto x : v)
                    put(raw, x);
            commands.push_back(command(1, raw));
        };
        for (unsigned s = 0; s < 2; ++s) {
            commands.push_back(command(s ? 54 : 53, {}));
            commands.push_back(command(21, Bytes{0}));
            for (unsigned i = 0; i < 4; ++i) {
                auto a = corners[i], b = corners[(i + 1) % 4];
                a[2] = b[2] = s * 2.;
                if (i == 0)
                    polyline({a, Point3{.5, 0, s * 2.}, b});
                else
                    polyline({a, b});
            }
            commands.push_back(command(22, {}));
        }
        commands.push_back(command(55, Bytes{1, 0, 0, 0, 4, 0, 0, 0}));
        for (unsigned i = 0; i < 4; ++i) {
            auto a = corners[i], middle = a, b = a;
            middle[0] += 1;
            middle[2] = bend_height;
            b[2] = 2;
            if (twisted && i == 1)
                middle[1] += 1;
            if (damaged && i == 3)
                b[0] += .25;
            commands.push_back(command(20, {}));
            polyline({a, middle, b});
            commands.push_back(command(22, {}));
        }
        commands.push_back(command(19, {}));
        return commands;
    };
    Tessellation policy;
    policy.full_circle_segments = 8;
    auto g = reconstruct(fixture(true, false, false), policy);
    check(g.unknown.empty() && !g.faces.empty(), "bent polyline guided loft produces a surface");
    auto contains = [](const Geometry &mesh, Point3 p) {
        for (auto v : mesh.vertices)
            if (std::abs(v[0] - p[0]) + std::abs(v[1] - p[1]) + std::abs(v[2] - p[2]) < 1e-10)
                return true;
        return false;
    };
    check(contains(g, {1, 0, 1}) && contains(g, {3, 2, 1}) && contains(g, {2, 0, 1}) &&
              contains(g, {.5, 0, 0}),
          "all rail bends and unequal boundary segment positions survive surface construction");
    double volume = 0;
    for (const auto &t : g.faces) {
        auto a = g.vertices[t[0]], b = g.vertices[t[1]], c = g.vertices[t[2]];
        volume += (a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
                   a[2] * (b[0] * c[1] - b[1] * c[0])) /
                  6.;
    }
    check(std::abs(std::abs(volume) - 8.) < 1e-9,
          "closed translated square sweep retains its independently known volume");
    auto open = reconstruct(fixture(false, false, false), policy);
    check(open.unknown.empty() && open.faces.size() < g.faces.size(),
          "guided loft caps are optional");
    auto unequal = reconstruct(fixture(false, false, false, .5), policy);
    auto t = std::sqrt(1.25) / (std::sqrt(1.25) + std::sqrt(3.25));
    auto f = (.5 - t) / (1 - t);
    check(unequal.unknown.empty() && contains(unequal, {2 - f, 0, .5 + 1.5 * f}),
          "unequal guide spans use chord length parameters at the middle surface row");
    policy.full_circle_segments = 4;
    auto coarse = reconstruct(fixture(false, true, false), policy);
    policy.chord_tolerance = .01;
    auto fine = reconstruct(fixture(false, true, false), policy);
    check(coarse.unknown.empty() && fine.unknown.empty() &&
              fine.faces.size() > coarse.faces.size() && contains(fine, {3, 1, 1}),
          "nonplanar guided patches refine while retaining guide corners");
    policy.max_segments = 4;
    auto limited = reconstruct(fixture(true, false, false), policy);
    check(!limited.unknown.empty() && limited.faces.empty(),
          "guided surface budget fails without partial mesh");
    auto bad = reconstruct(fixture(true, false, true), Tessellation{});
    check(!bad.unknown.empty() && bad.faces.empty(),
          "misaligned guide endpoints do not silently distort geometry");
}
static void rational_guided_tests() {
    const double pi = std::acos(-1.);
    std::vector<GuidedBoundary> bottom, top, rails;
    for (unsigned i = 0; i < 4; ++i) {
        GuidedBoundary a;
        a.ellipse = true;
        a.axis_x = {1, 0, 0};
        a.axis_y = {0, 1, 0};
        a.start = i * pi / 2;
        a.sweep = pi / 2;
        bottom.push_back(a);
        a.center = {0, 0, 2};
        top.push_back(a);
        // Every rail is the same semicircle translated to its profile vertex.
        GuidedBoundary r;
        r.ellipse = true;
        r.center = {std::cos(i * pi / 2), std::sin(i * pi / 2), 1};
        r.axis_x = {0, 0, -1};
        r.axis_y = {1, 0, 0};
        r.sweep = pi;
        rails.push_back(r);
    }
    Tessellation policy;
    policy.full_circle_segments = 8;
    auto mesh = guided_surface(bottom, top, rails, policy);
    auto close = [](Point3 a, Point3 b) {
        return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) < 1e-9;
    };
    bool midpoint = false;
    for (const auto &ring : mesh.rings)
        for (auto p : ring)
            midpoint = midpoint || close(p, {1 + std::sqrt(.5), std::sqrt(.5), 1});
    check(midpoint,
          "rational translated profiles follow the analytic circle-plus-semicircle surface");
    check(mesh.note["patches"][0]["u_degree"] == 3 && mesh.note["patches"][0]["u_poles"] == 4 &&
              mesh.note["patches"][0]["v_degree"] == 2 && mesh.note["patches"][0]["v_poles"] == 5,
          "native single-span arc elevation and two-span 180-degree guide basis are preserved");
    for (const auto &ring : mesh.rings) {
        check(close(ring.front(), ring.back()),
              "closed guided surface shares the exact seam sample");
        double z = ring.front()[2], cx = std::sqrt(std::max(0., 1 - (z - 1) * (z - 1)));
        for (auto p : ring)
            check(
                std::abs(p[2] - z) < 1e-9 && std::abs(std::hypot(p[0] - cx, p[1]) - 1) < 1e-9,
                "each rational surface row lies on the independently known translated unit circle");
    }
    policy.chord_tolerance = .05;
    policy.max_segments = 200000;
    auto fine = guided_surface(bottom, top, rails, policy);
    check(fine.note["surface_to_mesh_error_bound"].get<double>() <= .05 &&
              fine.rings.size() > mesh.rings.size(),
          "positive-weight rational derivative bound drives refinement");
    double maximum = 0;
    // Triangle centroids must stay within the bound of the analytically known surface.
    for (std::size_t j = 1; j < fine.rings.size(); ++j)
        for (std::size_t i = 1; i < fine.rings[j].size(); ++i) {
            for (auto vertices : {std::array<Point3, 3>{fine.rings[j - 1][i - 1],
                                                        fine.rings[j - 1][i], fine.rings[j][i]},
                                  std::array<Point3, 3>{fine.rings[j - 1][i - 1], fine.rings[j][i],
                                                        fine.rings[j][i - 1]}}) {
                Point3 p{};
                for (auto v : vertices)
                    for (unsigned k = 0; k < 3; ++k)
                        p[k] += v[k] / 3;
                double cx = std::sqrt(std::max(0., 1 - (p[2] - 1) * (p[2] - 1)));
                maximum = std::max(maximum, std::abs(std::hypot(p[0] - cx, p[1]) - 1));
            }
        }
    check(
        maximum <= .05,
        "rational mesh agrees with independent analytic cross sections within requested tolerance");
    auto limited = policy;
    limited.max_segments = 16;
    rejects([&] { guided_surface(bottom, top, rails, limited); },
            "rational surface budget rejects a whole mesh, never a partial one");
    auto bad = rails;
    bad[0].center[0] += .1;
    rejects([&] { guided_surface(bottom, top, bad, policy); },
            "misaligned rational guides are diagnosed");
    auto closed = bottom;
    closed[0].sweep = 4 * pi;
    rejects([&] { guided_surface(closed, top, rails, policy); },
            "multi-turn conics are not mistaken for one periodic circle");
    auto full_bottom = bottom[0], full_top = top[0];
    full_bottom.sweep = full_top.sweep = 2 * pi;
    auto periodic = guided_surface({full_bottom}, {full_top}, {rails[0]}, policy);
    bool circles = true;
    for (const auto &ring : periodic.rings) {
        double z = ring.front()[2], cx = std::sqrt(std::max(0., 1 - (z - 1) * (z - 1)));
        for (auto p : ring)
            circles = circles && std::abs(std::hypot(p[0] - cx, p[1]) - 1) < 1e-9;
    }
    check(circles && periodic.note["patches"][0]["u_poles"] == 7,
          "periodic circle opens at its source seam and keeps the native seven-pole net");
    std::vector<GuidedBoundary> composite;
    for (const auto &rail : rails) {
        GuidedBoundary group;
        auto a = rail, b = rail;
        a.sweep = b.sweep = pi / 2;
        b.start = pi / 2;
        group.parts = {a, b};
        composite.push_back(group);
    }
    auto joined = guided_surface(bottom, top, composite, policy);
    bool equal = joined.rings.size() == fine.rings.size();
    for (std::size_t i = 0; equal && i < joined.rings.size(); ++i) {
        equal = joined.rings[i].size() == fine.rings[i].size();
        for (std::size_t j = 0; equal && j < joined.rings[i].size(); ++j)
            equal = close(joined.rings[i][j], fine.rings[i][j]);
    }
    check(equal, "two quarter-arc guide commands agree with the independent semicircle sweep");
    auto uneven = composite;
    uneven[0] = rails[0];
    auto weighted = guided_surface(bottom, top, uneven, policy);
    bool weighted_circles = true;
    for (const auto &ring : weighted.rings) {
        double z = ring.front()[2], cx = std::sqrt(std::max(0., 1 - (z - 1) * (z - 1)));
        for (auto p : ring)
            weighted_circles = weighted_circles && std::abs(std::hypot(p[0] - cx, p[1]) - 1) < 1e-8;
    }
    check(weighted.note["composite_guide_parameterization"] ==
                  "sequential_control_polygon_length" &&
              weighted_circles,
          "different native guide part counts select polygon-length joining");
    std::vector<GuidedBoundary> triple;
    for (const auto &rail : rails) {
        GuidedBoundary group;
        for (unsigned i = 0; i < 3; ++i) {
            auto part = rail;
            part.start = i * pi / 3;
            part.sweep = pi / 3;
            group.parts.push_back(part);
        }
        triple.push_back(group);
    }
    auto sequential = guided_surface(bottom, top, triple, policy);
    bool second_joint = false;
    for (const auto &ring : sequential.rings)
        second_joint = second_joint || close(ring.front(), {1 + std::sqrt(3.) / 2, 0, 1.5});
    check(second_joint, "three native guide parts retain the sequential join break at one half");

    Json commands = Json::array({command(56, Bytes{1})});
    auto arc = [&](Point3 origin, std::array<double, 4> rotation, double start, double sweep) {
        Bytes raw;
        for (auto x : origin)
            put(raw, x);
        for (auto x : rotation)
            put(raw, x);
        put(raw, 1.);
        put(raw, 1.);
        put(raw, start);
        put(raw, sweep);
        commands.push_back(command(5, raw));
    };
    for (unsigned s = 0; s < 2; ++s) {
        commands.push_back(command(s ? 54 : 53, {}));
        commands.push_back(command(21, Bytes{0}));
        for (unsigned i = 0; i < 4; ++i)
            arc({0, 0, s * 2.}, {1, 0, 0, 0}, i * pi / 2, pi / 2);
        commands.push_back(command(22, {}));
    }
    commands.push_back(command(55, Bytes{1, 0, 0, 0, 4, 0, 0, 0}));
    // Inverse source quaternion: columns of inverse rotation are -Z, +X, -Y.
    for (unsigned i = 0; i < 4; ++i) {
        commands.push_back(command(20, {}));
        arc(rails[i].center, {.5, -.5, -.5, .5}, 0, pi);
        commands.push_back(command(22, {}));
    }
    commands.push_back(command(19, {}));
    auto geometry = reconstruct(commands, policy);
    check(geometry.unknown.empty() && !geometry.faces.empty(),
          "rational native command boundaries reach the surface evaluator");
    bool command_midpoint = false;
    for (auto p : geometry.vertices)
        command_midpoint = command_midpoint || close(p, {1 + std::sqrt(.5), std::sqrt(.5), 1});
    check(command_midpoint, "source inverse quaternion places curved guides on the intended side");
    double volume = 0;
    for (auto f : geometry.faces) {
        auto a = geometry.vertices[f[0]], b = geometry.vertices[f[1]], c = geometry.vertices[f[2]];
        volume += (a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
                   a[2] * (b[0] * c[1] - b[1] * c[0])) /
                  6;
    }
    check(std::abs(std::abs(volume) - 2 * pi) < .05,
          "closed rational sweep volume approaches its independent 2*pi value");
    Tessellation coarse_policy;
    coarse_policy.full_circle_segments = 8;
    auto base = reconstruct(commands, coarse_policy);
    Bytes affine;
    for (double x : {2., 0., 0., 1000., 0., 3., 0., -2000., 0., 0., .5, 3000.})
        put(affine, x);
    auto transformed = commands;
    transformed.insert(transformed.begin(), command(13, affine));
    transformed.push_back(command(14, {}));
    auto placed = reconstruct(transformed, coarse_policy);
    bool same = placed.unknown.empty() && placed.vertices.size() == base.vertices.size();
    for (std::size_t i = 0; same && i < base.vertices.size(); ++i) {
        auto p = base.vertices[i];
        same = close(placed.vertices[i], {2 * p[0] + 1000, 3 * p[1] - 2000, .5 * p[2] + 3000});
    }
    check(same, "native rational boundary construction commutes with nonuniform affine placement");

    // Unequal profile bulges distinguish native pole blending from pointwise Coons.
    std::vector<GuidedBoundary> lower(4), upper(4), bent(4);
    const std::array<Point3, 4> corners = {{{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}}};
    for (unsigned i = 0; i < 4; ++i) {
        auto a = corners[i], b = corners[(i + 1) % 4];
        lower[i].points = {a, b};
        a[2] = b[2] = 2;
        upper[i].points = {a, b};
        auto c = corners[i];
        bent[i].points = {c, {c[0] + .5, c[1], 1}, a};
    }
    lower[0].points = {{0, 0, 0}, {1, -1, 0}, {2, 0, 0}};
    upper[0].points = {{0, 0, 2}, {1, -2, 2}, {2, 0, 2}};
    auto asymmetric = guided_surface(lower, upper, bent, coarse_policy);
    bool native_blend = false;
    for (const auto &ring : asymmetric.rings)
        for (auto p : ring)
            native_blend = native_blend || close(p, {.75, -.5625, .5});
    // At u=v=1/4, elevated pole coefficients interpolate the top bulge with 1/8.
    // Ordinary parameter blending would instead yield y=-.625.
    check(native_blend, "unequal profiles retain native piecewise pole blending");
}
static void guided_open_tests() {
    std::vector<GuidedBoundary> bottom(2), top(2), rails(3);
    const std::array<Point3, 3> points = {{{0, 0, 0}, {2, 0, 0}, {2, 3, 0}}};
    for (unsigned i = 0; i < 3; ++i) {
        auto q = points[i];
        q[2] = 4;
        rails[i].points = {points[i], q};
        if (i < 2) {
            bottom[i].points = {points[i], points[i + 1]};
            auto r = points[i + 1];
            r[2] = 4;
            top[i].points = {q, r};
        }
    }
    Tessellation policy;
    policy.full_circle_segments = 8;
    auto mesh = guided_surface(bottom, top, rails, policy, false);
    check(mesh.rings.size() == 2 && mesh.rings[0].front() == points[0] &&
              mesh.rings[0].back() == points[2] && mesh.rings[1].back() == Point3({2, 3, 4}) &&
              mesh.note["profile_closed"] == false,
          "open guided strip retains its final endpoint instead of closing onto its first");
    rejects([&] { guided_surface(bottom, top, rails, policy); },
            "closed profile rejects the extra end guide");
    auto missing = rails;
    missing.pop_back();
    rejects([&] { guided_surface(bottom, top, missing, policy, false); },
            "open profile requires a guide at both ends");
    Json commands = Json::array({command(56, Bytes{0})});
    auto line = [&](const std::vector<Point3> &points) {
        Bytes raw;
        put<std::uint32_t>(raw, unsigned(points.size()));
        for (auto p : points)
            for (auto x : p)
                put(raw, x);
        commands.push_back(command(1, raw));
    };
    for (unsigned stage : {53, 54}) {
        commands.push_back(command(stage, {}));
        commands.push_back(command(20, {}));
        for (const auto &b : stage == 53 ? bottom : top)
            line(b.points);
        commands.push_back(command(22, {}));
    }
    Bytes groups;
    put<std::uint32_t>(groups, 1);
    put<std::uint32_t>(groups, 3);
    commands.push_back(command(55, groups));
    for (const auto &rail : rails) {
        commands.push_back(command(20, {}));
        line(rail.points);
        commands.push_back(command(22, {}));
    }
    commands.push_back(command(19, {}));
    auto geometry = reconstruct(commands, policy);
    check(geometry.unknown.empty() && !geometry.faces.empty(),
          "native open-profile command blocks reconstruct a guided surface");
    double area = 0;
    for (auto f : geometry.faces) {
        auto a = geometry.vertices[f[0]], b = geometry.vertices[f[1]], c = geometry.vertices[f[2]];
        Point3 u{}, v{};
        for (unsigned k = 0; k < 3; ++k) {
            u[k] = b[k] - a[k];
            v[k] = c[k] - a[k];
        }
        area += .5 * std::hypot(u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                                u[0] * v[1] - u[1] * v[0]);
        check((a[1] == 0 && b[1] == 0 && c[1] == 0) || (a[0] == 2 && b[0] == 2 && c[0] == 2),
              "open L-profile contains only its two intended side strips");
    }
    check(std::abs(area - 20) < 1e-12,
          "open L-profile area has no artificial closing wall or caps");
    auto mismatched = commands;
    mismatched[7] = command(21, Bytes{1});
    auto failed = reconstruct(mismatched, policy);
    check(!failed.unknown.empty() && failed.vertices.empty() &&
              failed.unknown.at(0)["reason"] == "guided loft profile boundary types differ",
          "mixed open and closed profile types fail without a partial solid");
    auto capped = commands;
    capped[0] = command(56, Bytes{1});
    failed = reconstruct(capped, policy);
    check(!failed.unknown.empty() && failed.vertices.empty(),
          "capped open profile with separated endpoints fails without a partial solid");
    GuidedBoundary arc;
    arc.ellipse = true;
    arc.axis_x = {1, 0, 0};
    arc.axis_y = {0, 1, 0};
    arc.sweep = std::acos(-1.0);
    auto upper = arc;
    upper.center[2] = 4;
    std::vector<GuidedBoundary> end_rails(2);
    end_rails[0].points = {{1, 0, 0}, {1, 0, 4}};
    end_rails[1].points = {{-1, 0, 0}, {-1, 0, 4}};
    policy.chord_tolerance = .01;
    auto curved = guided_surface({arc}, {upper}, end_rails, policy, false);
    bool cylinder = true;
    for (const auto &row : curved.rings) {
        cylinder &= std::abs(row.front()[0] - 1) < 1e-12 && std::abs(row.back()[0] + 1) < 1e-12;
        for (auto p : row)
            cylinder &= std::abs(std::hypot(p[0], p[1]) - 1) < 1e-12 && p[1] >= -1e-12;
    }
    check(cylinder && curved.note["surface_to_mesh_error_bound"].get<double>() <= .01,
          "open rational profile follows the analytic half-cylinder with its tolerance bound");
}
static void guided_cap_tests() {
    const std::vector<Point3> lower = {{0, 0, 0}, {2, 0, 0}, {2, 3, 0}, {0, 3, 0}, {0, 0, 0}};
    auto upper = lower;
    for (auto &p : upper)
        p[2] = 4;
    Tessellation policy;
    policy.full_circle_segments = 8;
    auto boundaries = [](const std::vector<Point3> &points) {
        std::vector<GuidedBoundary> out;
        for (std::size_t i = 1; i < points.size(); ++i) {
            GuidedBoundary b;
            b.points = {points[i - 1], points[i]};
            out.push_back(b);
        }
        return out;
    };
    auto surface = [&](const std::vector<Point3> &a, const std::vector<Point3> &b) {
        std::vector<GuidedBoundary> rails(a.size());
        for (std::size_t i = 0; i < a.size(); ++i)
            rails[i].points = {a[i], b[i]};
        return guided_surface(boundaries(a), boundaries(b), rails, policy, false);
    };
    auto encoded = [&](const std::vector<Point3> &a, const std::vector<Point3> &b,
                       bool closed = false, bool capped = true) {
        Json commands = Json::array({command(56, Bytes{std::uint8_t(capped)})});
        auto line = [&](Point3 p, Point3 q) {
            Bytes raw;
            put<std::uint32_t>(raw, 2);
            for (auto v : {p, q})
                for (auto x : v)
                    put(raw, x);
            commands.push_back(command(1, raw));
        };
        for (unsigned s = 0; s < 2; ++s) {
            commands.push_back(command(s ? 54 : 53, {}));
            commands.push_back(command(closed ? 21 : 20, closed ? Bytes{1} : Bytes{}));
            const auto &points = s ? b : a;
            for (std::size_t i = 1; i < points.size(); ++i)
                line(points[i - 1], points[i]);
            commands.push_back(command(22, {}));
        }
        Bytes groups;
        put<std::uint32_t>(groups, 1);
        const auto guide_count = a.size() - (closed ? 1 : 0);
        put<std::uint32_t>(groups, unsigned(guide_count));
        commands.push_back(command(55, groups));
        for (std::size_t i = 0; i < guide_count; ++i) {
            commands.push_back(command(20, {}));
            line(a[i], b[i]);
            commands.push_back(command(22, {}));
        }
        commands.push_back(command(19, {}));
        return reconstruct(commands, policy);
    };
    auto geometry = encoded(lower, upper);
    check(geometry.unknown.empty() && !geometry.faces.empty() &&
              geometry.notes.at(0)["profile_closed"] == false &&
              geometry.notes.at(0)["patches"].size() == 4,
          "physically closed Open paths cap without changing source type or N+1 guide mapping");
    double area = 0, volume = 0, lower_area = 0, upper_area = 0;
    for (auto f : geometry.faces) {
        auto a = geometry.vertices[f[0]], b = geometry.vertices[f[1]], c = geometry.vertices[f[2]];
        Point3 u{}, v{};
        for (unsigned k = 0; k < 3; ++k) {
            u[k] = b[k] - a[k];
            v[k] = c[k] - a[k];
        }
        Point3 n = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                    u[0] * v[1] - u[1] * v[0]};
        double triangle_area = .5 * std::hypot(n[0], n[1], n[2]);
        area += triangle_area;
        volume += (a[0] * n[0] + a[1] * n[1] + a[2] * n[2]) / 6;
        if (a[2] == 0 && b[2] == 0 && c[2] == 0)
            lower_area += n[2] / 2;
        if (a[2] == 4 && b[2] == 4 && c[2] == 4)
            upper_area += n[2] / 2;
    }
    check(std::abs(area - 52) < 1e-12 && std::abs(volume - 24) < 1e-12,
          "capped Open rectangle has analytic prism area and outward signed volume");
    check(std::abs(lower_area + 6) < 1e-12 && std::abs(upper_area - 6) < 1e-12,
          "lower and upper caps have opposite outward orientation and complete area");

    auto near = lower;
    near.back()[0] = 5e-11;
    check(surface(near, upper).cap_boundaries_closed == std::array<bool, 2>{true, true} &&
              encoded(near, upper).unknown.empty(),
          "closure accepts a small endpoint discrepancy without requiring bitwise equality");
    near.back()[0] = 1e-10;
    check(!surface(near, upper).cap_boundaries_closed[0],
          "native closure comparison is strict at the origin tolerance boundary");
    near.back()[0] = 5e-7;
    check(surface(near, upper).cap_boundaries_closed == std::array<bool, 2>{false, true},
          "bottom cap closure is checked independently from the top");
    auto failed = encoded(near, upper);
    check(!failed.unknown.empty() && failed.vertices.empty(),
          "failed bottom cap leaves no partial guided solid");
    auto distant = upper;
    distant.back()[0] = 5e-7;
    check(surface(lower, distant).cap_boundaries_closed == std::array<bool, 2>{true, false},
          "top cap closure is checked independently from the bottom");
    failed = encoded(lower, distant);
    check(!failed.unknown.empty() && failed.vertices.empty(),
          "failed top cap leaves no partial guided solid");
    for (auto &p : near)
        p[0] += 10000;
    for (auto &p : distant)
        p[0] += 10000;
    check(surface(near, distant).cap_boundaries_closed == std::array<bool, 2>{true, true} &&
              encoded(near, distant).unknown.empty(),
          "native coordinate scale permits the same discrepancy far from the origin");

    std::vector<GuidedBoundary> rails(lower.size());
    for (std::size_t i = 0; i < lower.size(); ++i)
        rails[i].points = {lower[i], upper[i]};
    rails.back().points = {lower.back(), {-.5, 0, 2}, upper.back()};
    auto separate = guided_surface(boundaries(lower), boundaries(upper), rails, policy, false);
    bool separate_interior = false;
    for (const auto &row : separate.rings)
        separate_interior |= std::abs(row.front()[0] - row.back()[0]) > .1;
    check(separate.cap_boundaries_closed == std::array<bool, 2>{true, true} && separate_interior,
          "cap closure does not merge independent first and last source guides");

    near = lower;
    near.back()[0] = 5e-8;
    failed = encoded(near, upper, true, false);
    check(!failed.unknown.empty() && failed.vertices.empty() &&
              failed.unknown.at(0)["reason"] == "closed guided profile endpoints do not coincide",
          "closed lower source profile must close even when caps are disabled");
    distant = upper;
    distant.back()[0] = 5e-8;
    failed = encoded(lower, distant, true, false);
    check(!failed.unknown.empty() && failed.vertices.empty() &&
              failed.unknown.at(0)["reason"] == "closed guided profile endpoints do not coincide",
          "closed upper source profile must close independently of cap generation");
    check(encoded(near, distant, false, false).unknown.empty(),
          "Open source type keeps its separate ends without a closure requirement");
    near.back()[0] = 5e-11;
    failed = encoded(near, upper, true, false);
    check(failed.unknown.empty(), "closed source profile uses the native scaled closure tolerance");
}
static void guided_endpoint_tests() {
    std::vector<GuidedBoundary> bottom(1), top(1), rails(2);
    bottom[0].points = {{0, 0, 0}, {2, 0, 0}};
    top[0].points = {{0, 0, 4}, {2, 0, 4}};
    rails[0].points = {{0, 0, 0}, {0, 0, 4}};
    rails[1].points = {{2, 0, 0}, {2, 0, 4}};
    Tessellation policy;
    policy.full_circle_segments = 8;
    auto shifted = rails;
    shifted[0].points.front() = {1e-5, 1e-5, 1e-5};
    auto mesh = guided_surface(bottom, top, shifted, policy, false);
    check(mesh.rings.front().front() == shifted[0].points.front(),
          "Coons corner accepts the inclusive per-coordinate tolerance without snapping");
    for (unsigned corner = 0; corner < 4; ++corner) {
        auto invalid = rails;
        auto &p = invalid[corner % 2].points[corner / 2];
        p[1] = std::nextafter(1e-5, std::numeric_limits<double>::infinity());
        rejects([&] { guided_surface(bottom, top, invalid, policy, false); },
                "each corner independently rejects a component beyond the native tolerance");
    }
    auto reversed = rails;
    std::reverse(reversed[0].points.begin(), reversed[0].points.end());
    rejects([&] { guided_surface(bottom, top, reversed, policy, false); },
            "native fixed boundary orientation does not guess a reversal for a misplaced guide");

    auto composite = rails;
    for (unsigned i = 0; i < 2; ++i) {
        GuidedBoundary a, b;
        a.points = {{double(2 * i), 0, 0}, {double(2 * i), 0, 1}};
        b.points = {{double(2 * i), 0, 1}, {double(2 * i), 0, 4}};
        composite[i].points.clear();
        composite[i].parts = {a, b};
    }
    auto joined = guided_surface(bottom, top, composite, policy, false);
    auto near = composite;
    near[0].parts[1].points.front() = {5e-9, 5e-9, 1 + 5e-9};
    auto snapped = guided_surface(bottom, top, near, policy, false);
    check(joined.rings == snapped.rings,
          "contiguous join retains the preceding endpoint and omits the following endpoint");
    auto gap = composite;
    gap[0].parts[1].points.front()[1] = 5e-8;
    rejects([&] { guided_surface(bottom, top, gap, policy, false); },
            "native noncontiguous join is not silently converted to a shared endpoint");

    for (auto *set : {&bottom, &top})
        for (auto &b : *set)
            for (auto &p : b.points)
                p[0] += 10000;
    for (auto &b : composite)
        for (auto &part : b.parts)
            for (auto &p : part.points)
                p[0] += 10000;
    joined = guided_surface(bottom, top, composite, policy, false);
    composite[0].parts[1].points.front()[1] = 5e-5;
    snapped = guided_surface(bottom, top, composite, policy, false);
    check(joined.rings == snapped.rings,
          "native contiguous join scales with coordinate extent and preserves its first endpoint");
    composite[0].parts[1].points.front()[1] = 2e-4;
    rejects([&] { guided_surface(bottom, top, composite, policy, false); },
            "coordinate-scaled join still rejects a larger discontinuity");
}
static void guided_ring_tests() {
    auto groups = command_fields(55, Bytes{2, 0, 0, 0, 4, 0, 0, 0, 4, 0, 0, 0});
    check(groups["guide_group_count"] == 2 && groups["guide_counts"] == Json({4, 4}),
          "native guide group counts are decoded without flattening ring identity");
    check(command_fields(55, Bytes{2, 0, 0, 0, 4, 0, 0, 0}).contains("field_decode_error"),
          "truncated guide-group table is diagnosed");
    const std::vector<std::vector<Point3>> loops = {
        {{-2, -2, 0}, {2, -2, 0}, {2, 2, 0}, {-2, 2, 0}},
        {{-1, -1, 0}, {-1, 1, 0}, {1, 1, 0}, {1, -1, 0}}};
    Json commands = Json::array({command(56, Bytes{1})});
    auto line = [&](Point3 a, Point3 b) {
        Bytes raw;
        put<std::uint32_t>(raw, 2);
        for (auto p : {a, b})
            for (auto x : p)
                put(raw, x);
        commands.push_back(command(1, raw));
    };
    for (unsigned s = 0; s < 2; ++s) {
        commands.push_back(command(s ? 54 : 53, {}));
        commands.push_back(command(21, Bytes{1}));
        for (unsigned ring = 0; ring < 2; ++ring) {
            if (ring)
                commands.push_back(command(23, {}));
            for (unsigned i = 0; i < 4; ++i) {
                auto a = loops[ring][i], b = loops[ring][(i + 1) % 4];
                a[2] = b[2] = 3 * s;
                line(a, b);
            }
        }
        commands.push_back(command(22, {}));
    }
    const auto group_at = commands.size();
    commands.push_back(command(55, Bytes{2, 0, 0, 0, 4, 0, 0, 0, 4, 0, 0, 0}));
    for (const auto &ring : loops)
        for (auto a : ring) {
            auto b = a;
            b[2] = 3;
            commands.push_back(command(20, {}));
            line(a, b);
            commands.push_back(command(22, {}));
        }
    commands.push_back(command(19, {}));
    Tessellation policy;
    policy.full_circle_segments = 8;
    auto g = reconstruct(commands, policy);
    check(g.unknown.empty() && !g.faces.empty() && g.notes.size() == 2,
          "native paired rings and corresponding guide groups produce a hollow loft");
    double volume = 0;
    bool empty_hole = true;
    for (auto f : g.faces) {
        auto a = g.vertices[f[0]], b = g.vertices[f[1]], c = g.vertices[f[2]];
        volume += (a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
                   a[2] * (b[0] * c[1] - b[1] * c[0])) /
                  6;
        if (a[2] == b[2] && b[2] == c[2]) {
            double x = (a[0] + b[0] + c[0]) / 3, y = (a[1] + b[1] + c[1]) / 3;
            empty_hole = empty_hole && (std::abs(x) >= 1 || std::abs(y) >= 1);
        }
    }
    check(std::abs(volume - 36) < 1e-9 && empty_hole,
          "hole walls and caps preserve exact hollow-prism volume and leave the hole empty");
    auto bad = commands;
    bad[group_at] = command(55, Bytes{2, 0, 0, 0, 3, 0, 0, 0, 5, 0, 0, 0});
    auto broken = reconstruct(bad, policy);
    check(!broken.unknown.empty() && broken.faces.empty(),
          "invalid native group correspondence cannot return a partial multiring solid");
    policy.max_segments = 20;
    broken = reconstruct(commands, policy);
    check(!broken.unknown.empty() && broken.faces.empty(),
          "multiring surface obeys a total mesh budget");
}
static Geometry triangle() {
    Geometry g;
    g.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    g.faces = {{0, 1, 2}};
    g.face_uvs.push_back(std::array<Point2, 3>{{{0, 0}, {1, 0}, {0, 1}}});
    g.face_source_polygons.push_back(0);
    g.primitive_ranges.push_back(
        {{"channel", "faces"}, {"start", 0}, {"count", 1}, {"style", Json::object()}});
    return g;
}
static void command_metadata_tests() {
    Bytes b;
    put<std::uint32_t>(b, 4);
    put<std::uint32_t>(b, 0xffffffffu);
    auto d = command_fields(40, b);
    check(d["layer_id"] == 0xffffffffu && d["kind"] == 4 && d["value"] == 0xffffffffu,
          "extension layer ID preserves its native sentinel and compatibility views");
    Json style = {{"material_id", 987}};
    apply_symbology_extension(style, d);
    apply_symbology_extension(style, command_fields(40, Bytes(4, 0)));
    check(style["layer_id"] == 0xffffffffu && style["material_id"] == 987,
          "absent extension sections preserve prior layer and material state");
    Bytes fill;
    for (double x : {1.25, 2.5, 3.75})
        put(fill, x);
    put<std::uint16_t>(fill, 1);
    put<std::uint16_t>(fill, 2);
    put<std::uint16_t>(fill, 3);
    put<std::uint16_t>(fill, 0xabcd);
    put(fill, .125);
    for (unsigned char x : {11, 22, 33, 44, 55, 66, 77, 88})
        fill.push_back(x);
    b.clear();
    put<std::uint32_t>(b, 7);
    put<std::uint16_t>(b, fill.size());
    b.insert(b.end(), fill.begin(), fill.end());
    put<std::uint32_t>(b, 0x80000009u);
    put<std::uint32_t>(b, 114);
    d = command_fields(40, b);
    check(!d.contains("field_decode_error") && d["layer_id"] == 114 &&
              d["inheritance_flags"] == 0x80000009u &&
              d["fill_style_block"]["entries"][0]["color_bytes"] == Json({11, 22, 33}) &&
              d["fill_style_block"]["entries"][0]["rgb"] == Json({11, 33, 22}) &&
              d["fill_style_block"]["angle"] == 1.25 &&
              d["fill_style_block"]["white_intensity"] == 2.5 &&
              d["fill_style_block"]["shift"] == 3.75 &&
              d["fill_style_block"]["mode_name"] == "curved" && d["by_layer"]["color"] == true &&
              d["by_layer"]["line_weight"] == true && d["by_layer"]["fill_color"] == false &&
              d["by_layer"]["unassigned_bits"] == 0x80000000u &&
              d["fill_style_block"]["entries"][0]["position"] == .125 &&
              d["fill_style_block"]["zero_padded_bytes"] == 112 &&
              bytesof(d["fill_style_block"]["source_bytes"]) == fill,
          "combined extension consumes variable fill block before inheritance and layer fields");
    apply_symbology_extension(style, d);
    apply_symbology_extension(style, command_fields(40, wire_bytes("010000000000")));
    check(style["native_symbology_extension"]["fill_style_block"]["status"] == "clear" &&
              style["native_symbology_extension"]["inheritance_flags"] == 0x80000009u &&
              style["layer_id"] == 114,
          "clearing a fill block does not clear unrelated extension fields");
    for (auto n : {0u, 3u, 5u, 20u, 53u, 57u, 61u})
        check(command_fields(40, slice(b, 0, n)).contains("field_decode_error"),
              "truncated variable extension is diagnosed");
    check(command_fields(40, wire_bytes("08000000")).contains("field_decode_error"),
          "unknown extension flag does not pretend to have a known payload layout");
    Bytes full_fill(160, 0);
    full_fill[24] = 9;
    Bytes full_extension = wire_bytes("01000000a000");
    full_extension.insert(full_extension.end(), full_fill.begin(), full_fill.end());
    d = command_fields(40, full_extension);
    check(
        d["fill_style_block"]["declared_entry_count"] == 9 &&
            d["fill_style_block"]["entries"].size() == 8,
        "fill entry count is clamped to native eight-entry storage without losing declared count");
    full_extension[4] = 161;
    full_extension.push_back(0);
    check(command_fields(40, full_extension).contains("field_decode_error"),
          "oversized native fill block is rejected instead of overflowing fixed storage");
    Bytes id;
    put<std::uint32_t>(id, 0x45644964);
    put<std::uint32_t>(id, 8);
    append_wire(id, "1500000102");
    d = command_fields(29, id);
    check(!d.contains("field_decode_error") && !d.contains("identifier") &&
              d["curve_identifier"]["type_name"] == "curve_array" &&
              d["curve_identifier"]["topology"]["type_name"] == "curve_array" &&
              d["curve_identifier"]["topology"]["ids"] == Json({0, 1, 2}),
          "curve identifiers have a container and variable-width topology, not a fixed uint64 ID");
    for (unsigned code = 0; code < 3; ++code) {
        id.resize(8);
        id.push_back(24);
        id.push_back(code);
        for (auto v : {1u, 0xffffffffu}) {
            if (!code)
                put<std::uint8_t>(id, v);
            else if (code == 1)
                put<std::uint16_t>(id, v);
            else
                put(id, v);
        }
        d = command_fields(29, id);
        check(d["curve_identifier"]["topology"]["ids"][1] == (code == 0   ? 255u
                                                              : code == 1 ? 65535u
                                                                          : 0xffffffffu),
              "one-, two- and four-byte topology IDs preserve unsigned values");
    }
    id.resize(4);
    put<std::uint32_t>(id, 8u | (5u << 16));
    append_wire(id, "1500000102040003000400");
    d = command_fields(29, id);
    check(d["curve_identifier"]["topology"]["ids"] == Json({0, 1, 2}) &&
              d["curve_identifier"]["compound_draw_state_status"] == "decoded_layout" &&
              d["curve_identifier"]["compound_draw_state_decoded"]["function_code"] == 3 &&
              d["curve_identifier"]["compound_draw_state_decoded"]["unassigned_words"] ==
                  Json({4}) &&
              bytesof(d["curve_identifier"]["compound_draw_state"]) == wire_bytes("03000400"),
          "compound drawing state is split at the declared ID boundary and decodes native words");
    auto odd_state = id;
    odd_state[13] = 5;
    odd_state.push_back(0xab);
    auto odd = command_fields(29, odd_state)["curve_identifier"];
    check(odd["compound_draw_state_status"] == "decoded_with_trailing_byte" &&
              bytesof(odd["compound_draw_state_decoded"]["trailing_bytes"]) == Bytes({0xab}),
          "odd compound-state byte is retained although native loader ignores it");
    id[13] = 3;
    d = command_fields(29, id);
    check(d["curve_identifier"]["compound_draw_state_status"] == "invalid_fallback_to_id_data" &&
              bytesof(d["curve_identifier"]["id_data"]) == slice(id, 8, id.size() - 8),
          "invalid optional extension preserves the native fallback ID bytes");
    d = command_fields(29, wire_bytes("644964450000000015000100"));
    check(d["identifier"] == 281565171023872ull &&
              d["curve_identifier"]["topology_status"] == "opaque_for_native_type" &&
              !d["curve_identifier"].contains("topology"),
          "native cut identifiers are not guessed as topology arrays from coincidental bytes");
    check(command_fields(29, wire_bytes("64496445")).contains("field_decode_error"),
          "recognized curve marker requires its container header");
    auto long_id = wire_bytes("64496445080000001500");
    long_id.resize(long_id.size() + 256, 1);
    auto topology = command_fields(29, long_id)["curve_identifier"]["topology"];
    check(topology["ids"].size() == 256 && topology["native_count"] == 0 &&
              topology["status"] == "exceeds_native_count_range",
          "identifier overflow does not reproduce native uint8 count truncation as data loss");
    for (auto suffix : {"150301", "15020102"}) {
        auto broken_id = wire_bytes("6449644508000000");
        append_wire(broken_id, suffix);
        auto container = command_fields(29, broken_id)["curve_identifier"];
        check(container.contains("topology_decode_error") &&
                  bytesof(container["id_data"]) == wire_bytes(suffix),
              "bad nested topology preserves identifier data with a local diagnostic");
    }
    b.clear();
    put<std::uint16_t>(b, 0x7fff);
    for (unsigned v : {101, 102, 103, 104, 105, 106, 107, 108, 109, 110})
        put(b, v);
    put(b, .375);
    put<std::uint32_t>(b, 111);
    put<std::uint64_t>(b, 0xfedcba9876543210ull);
    put<std::int32_t>(b, -23);
    put<std::uint32_t>(b, 0x12345678);
    d = decode_symbology(b);
    check(d["field_0020"] == 105 && d["field_0010"] == 106 && d["fill_color_index"] == 109 &&
              d["line_weight"] == 110 && d["transparency"] == .375 && d["field_1000"] == 111 &&
              d["material_id"] == 0xfedcba9876543210ull && d["line_style"] == -23 &&
              d["fill_mode"] == 107 && d["subitem_index"] == 111 &&
              d["true_color_packed"] == 0x12345678u,
          "complete symbology bit layout follows native order and preserves 64-bit material IDs");
    check(d["true_color_rgb"] == Json({0x78, 0x56, 0x34}) &&
              d["true_color_unassigned_high_byte"] == 0x12,
          "packed direct color decodes low three RGB bytes and does not guess alpha");
    apply_symbology(style, d);
    apply_symbology(style, {{"flags", 128}, {"color_index", 42}});
    check(!style.contains("true_color_packed") && !style.contains("true_color_rgb") &&
              !style.contains("true_color_unassigned_high_byte") && style["color_index"] == 42,
          "new indexed color supersedes a previous packed color");
    Bytes modifiers;
    put<std::uint32_t>(modifiers, 0x1fffu);
    for (double x : {1., 2., 3., 4., 5., 6., 7., 8., 9., 10., 1., 0., 0., 0.})
        put(modifiers, x);
    put<std::uint32_t>(modifiers, 0xffffffffu);
    put<std::uint32_t>(modifiers, 0x87654321u);
    auto with_modifiers = [](const Bytes &mod) {
        Bytes body;
        put<std::uint16_t>(body, 0x8000);
        put<std::uint16_t>(body, mod.size());
        body.insert(body.end(), mod.begin(), mod.end());
        return body;
    };
    auto mod = decode_symbology(with_modifiers(modifiers))["line_style_modifiers"];
    check(mod["fields"].size() == 9 && mod["fields"][6]["float64_view"] == 7. &&
              mod["orientation_vector"] == Json({8., 9., 10.}) &&
              mod["orientation_quaternion"] == Json({1., 0., 0., 0.}) &&
              mod["fields"][7]["storage_uint32"] == 0xffffffffu &&
              mod["fields"][8]["storage_uint32"] == 0x87654321u &&
              mod["consumed_bytes"] == modifiers.size() && mod["unassigned_flag_bits"] == 0x400u &&
              mod["named_values"]["scale"] == 1. && mod["named_values"]["dash_scale"] == 2. &&
              mod["named_values"]["start_width"] == 4. &&
              mod["named_values"]["fraction_phase"] == 7. &&
              mod["named_values"]["multiline_index"] == 0xffffffffu &&
              mod["centered_shift"] == true,
          "line modifier optional scalar, vector, quaternion and integer fields follow native "
          "layout");
    for (auto n : {4u, 60u, 84u, 116u, 120u, 123u})
        check(command_fields(28, with_modifiers(slice(modifiers, 0, n)))
                  .contains("field_decode_error"),
              "modifier flags cannot read beyond their declared block");
    modifiers.push_back(0x55);
    mod = decode_symbology(with_modifiers(modifiers))["line_style_modifiers"];
    check(bytesof(mod["trailing_bytes"]) == Bytes({0x55}) && mod["raw_hex"] == hex(modifiers),
          "unconsumed modifier extension bytes remain available");
    mod = decode_symbology(with_modifiers(wire_bytes("80000000")))["line_style_modifiers"];
    check(mod["fields"].empty() && mod["consumed_bytes"] == 4,
          "native flag-only modifier bits consume no scalar payload");
    Json line_style = {{"line_style", 1}, {"line_style_modifiers", mod}};
    apply_symbology(line_style, {{"flags", 0x400}, {"line_style", 2}});
    check(!line_style.contains("line_style_modifiers"),
          "changing native line style clears modifiers from the previous line style");
    apply_symbology(line_style,
                    {{"flags", 0x8400}, {"line_style", 3}, {"line_style_modifiers", mod}});
    check(line_style["line_style_modifiers"] == mod,
          "modifier supplied with new line style is applied after clearing old state");

    NativeScene views;
    auto definition = std::make_shared<GeometryDefinition>();
    definition->geometry = triangle();
    views.definitions.push_back(definition);
    views.metadata = {{"color_tables", Json::array()},
                      {"materials", {{"definitions", Json::array()}}}};
    SceneElement element;
    element.metadata = {{"model_id", 13}, {"unknown", Json::array()}};
    GeometryInstance instance;
    instance.definition = 0;
    instance.matrix = identity();
    instance.style = {{"layer_id", 114}, {"true_color_packed", 0x12345678u}};
    element.instances.push_back(instance);
    views.elements.push_back(element);
    views.for_each_primitive([&](const PrimitiveView &v) {
        check(v.style["layer_id"] == 114 &&
                  v.appearance["color"]["rgb"] == Json({0x78, 0x56, 0x34}) &&
                  v.appearance["color"]["source"] == "native_packed_color",
              "primitive color summary never treats packed color as an indexed palette value");
    });
    views.elements[0].instances[0].style["native_symbology_extension"] = {{"inheritance_flags", 1}};
    views.for_each_primitive([&](const PrimitiveView &v) {
        check(v.appearance["color"]["source"] == "native_inheritance_rules_not_evaluated",
              "unresolved native inheritance is not reported as a resolved RGB color");
    });
    views.elements[0].instances[0].style["native_symbology_extension"] = {
        {"fill_style_block", {{"status", "decoded_gradient_fill_with_unassigned_flags"}}}};
    views.for_each_primitive([&](const PrimitiveView &v) {
        check(v.appearance["color"]["source"] == "native_gradient_fill" &&
                  v.appearance["color"]["rgb"].is_null(),
              "gradient fill is not reduced to an unrelated solid palette color");
    });
    Json commands = Json::array({command(40, wire_bytes("0400000072000000"))});
    Bytes line;
    put<std::uint32_t>(line, 2);
    for (double x : {0., 0., 0., 1., 0., 0.})
        put(line, x);
    commands.push_back(command(1, line));
    auto geo = reconstruct(commands, Tessellation{});
    check(geo.unknown.empty() && geo.primitive_ranges[0]["style"]["layer_id"] == 114,
          "native layer change reaches generated primitive ranges");
    commands[0] = command(40, wire_bytes("04000000"));
    geo = reconstruct(commands, Tessellation{});
    check(!geo.unknown.empty(), "damaged metadata commands are no longer silently ignored");
}
static Bytes drawing_fixture(bool substation, unsigned version) {
    Bytes b;
    put<std::uint32_t>(b, 1);
    for (int i = 0; i < 13; ++i)
        put<std::uint64_t>(b, 0);
    put<std::uint32_t>(b, 0);
    auto ids =
        substation ? std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7} : std::vector<int>{4, 5, 3, 6, 7};
    for (std::size_t index = 0; index < ids.size(); ++index) {
        int id = ids[index];
        put<std::uint32_t>(b, 0);
        auto v = !substation && id != 6 ? version : 0;
        put<std::uint32_t>(b, v);
        if (!index)
            put<std::uint32_t>(b, 0);
        put<std::uint32_t>(b, 0);
        put<std::uint32_t>(b, id);
        put<std::uint32_t>(b, 2);
        if (substation) {
            put<double>(b, 901);
            put<std::uint32_t>(b, 902);
        }
        for (double x : {101., 102., 103.})
            put(b, x);
        if (substation) {
            auto n = id == 2 || id == 3 ? 4 : id == 7 ? 3 : 2;
            for (int i = 0; i < n; ++i)
                put<double>(b, 201 + i);
        } else {
            put<std::uint32_t>(b, 3);
            if (id == 4 || id == 5) {
                put<std::uint32_t>(b, 1);
                if (id == 5)
                    put<std::uint32_t>(b, 2);
                for (int i = 0; i < (id == 4 ? 4 : 3); ++i)
                    put<double>(b, 201 + i);
                for (auto x : id == 4 ? Bytes{1, 0, 1, 0} : Bytes{0, 1})
                    put(b, x);
                if (id == 5) {
                    put<double>(b, 205);
                    put<std::uint8_t>(b, 1);
                }
            } else if (id == 3) {
                put<std::uint8_t>(b, 1);
                put<double>(b, 206);
            } else {
                put<std::uint8_t>(b, 0);
                put<std::uint8_t>(b, 1);
                if (id == 7)
                    put<std::uint8_t>(b, 0);
            }
            if (v) {
                if (id == 7)
                    put<std::uint8_t>(b, 0);
                else
                    put<std::uint32_t>(b, 1);
            }
        }
    }
    return b;
}
static Bytes electrical_fixture(unsigned data_version) {
    Bytes b;
    put<std::uint32_t>(b, 1); // ElecParaData
    put<std::uint32_t>(b, 0); // PBWJData cereal type version
    put<std::uint32_t>(b, data_version);
    for (int i = 0; i < 2; ++i)
        put<std::uint64_t>(b, 0); // strings
    put<std::uint32_t>(b, 0);     // PSXSectManager cereal type version
    put<std::uint32_t>(b, 0);
    put<std::uint64_t>(b, std::size(electrical_wire::sections));
    unsigned section_index = 0;
    for (auto text : electrical_wire::sections) {
        if (!section_index)
            put<std::uint32_t>(b, 0); // PSXsectAndMatType registration
        put<std::uint32_t>(b, 0);
        auto group = wire_bytes(text);
        group.resize(800, 0xa5); // deliberately nonzero unused capacity
        b.insert(b.end(), group.begin(), group.end());
        put<std::uint32_t>(b, 700 + section_index++);
    }
    put<std::uint64_t>(b, 2); // material map
    for (int i = 0; i < 2; ++i) {
        put<std::int32_t>(b, i - 1);
        if (!i)
            put<std::uint32_t>(b, 0); // PSsimpleMat type version, once
        put<std::uint32_t>(b, 0);
        put<std::uint32_t>(b, 101 + i);
        put<float>(b, 2.5f + i);
    }
    for (int i = 0; i < 2; ++i) { // two separate integer maps
        put<std::uint64_t>(b, 2);
        for (int j = 0; j < 2; ++j) {
            put<std::int32_t>(b, -7); // retain duplicate source keys
            put<std::int32_t>(b, 100 * i + j);
        }
    }
    bool point_seen = false, force_seen = false;
    auto force = [&](unsigned version) {
        if (!force_seen) {
            put<std::uint32_t>(b, 0);
            force_seen = true;
        }
        put(b, version);
        for (float f : {11.f, 12.f, 13.f, 14.f, 15.f, 16.f})
            put(b, f);
        if (version == 1) {
            put<std::int32_t>(b, -31);
            put<std::uint8_t>(b, 7);
        }
    };
    auto point = [&](unsigned version, bool forces) {
        if (!point_seen) {
            put<std::uint32_t>(b, 0);
            point_seen = true;
        }
        put(b, version);
        for (float v : {1.25f, -2.5f, 3.75f})
            put(b, v);
        if (version == 1) {
            put<std::uint64_t>(b, forces ? 2 : 0);
            if (forces)
                for (unsigned v = 0; v < 2; ++v)
                    force(v);
        }
    };
    put<std::uint64_t>(b, 2); // joints
    for (unsigned v = 0; v < 2; ++v) {
        put<std::int32_t>(b, 50 + v);
        point(v, true);
    }
    put<std::uint64_t>(b, 2); // WJ_Bar
    for (unsigned version = 0; version < 2; ++version) {
        put<std::int32_t>(b, 201 + version);
        if (!version)
            put<std::uint32_t>(b, 0);
        put(b, version);
        for (unsigned i = 0; i < 4; ++i)
            put(b, 300 + i);
        point(0, false);
        point(1, false);
        put<std::uint32_t>(b, 304);
        if (version == 1) {
            put<std::uint64_t>(b, 2);
            for (unsigned v = 0; v < 2; ++v) {
                if (!v)
                    put<std::uint32_t>(b, 0); // WJ_BarForce type version
                put(b, v);
                force(0);
                force(1);
                put<std::uint8_t>(b, 9);
                if (v == 1)
                    put<std::uint32_t>(b, 305);
            }
        }
    }
    put<std::uint8_t>(b, 1);
    append_wire(b, electrical_wire::columns);
    put<std::uint64_t>(b, 3); // TrussBeamPar
    for (unsigned v : {0u, 1u, 3u}) {
        put<std::int32_t>(b, 400 + v);
        if (!v)
            put<std::uint32_t>(b, 0);
        put(b, v);
        for (unsigned offset = 0; offset < (v == 1 ? 36u : 32u); offset += 4)
            if (v != 0 || offset != 4)
                put<std::uint32_t>(b, 500 + offset);
    }
    append_wire(b, electrical_wire::beams);
    append_wire(b, electrical_wire::human_columns);
    if (data_version >= 1) {
        put<std::uint64_t>(b, 2); // LineSubsBeamCols
        for (int i = 0; i < 2; ++i) {
            if (!i)
                put<std::uint32_t>(b, 0);
            put<std::uint32_t>(b, 0);
            put<std::uint64_t>(b, 1);
            put<std::uint64_t>(b, 0x20000000000001ull + i);
            put<std::uint64_t>(b, 0);
        }
    }
    if (data_version >= 2) {
        put<std::uint64_t>(b, 2); // GridAxisData
        for (int i = 0; i < 2; ++i) {
            if (!i)
                put<std::uint32_t>(b, 0);
            put<std::uint32_t>(b, 0);
            put<std::int32_t>(b, -12);
            put<float>(b, 0.5f);
            point(1, true);
            put<double>(b, 19.75);
            put<std::uint64_t>(b, 1);
            put<double>(b, -17.25);
            put<std::uint64_t>(b, 0);
        }
    }
    put<std::uint64_t>(b, 1); // changed_joints shares Pt3D_ST registration
    put<std::int32_t>(b, 81);
    point(1, true);
    return b;
}
int main() {
    try {
        attribute_semantics_tests();
        layer_group_tests();
        layer_table_tests();
        layer_group_state_tests();
        view_link_sequence_tests();
        native_layer_tests();
        material_index_tests();
        section_clip_tests();
        inline_material_tests();
        embedded_texture_tests();
        box_center_tests();
        guided_surface_tests();
        bgfb_native_tests();
        bspline_tests();
        checks += bspline_surface_tests();
        checks += bspline_trim_tests();
        checks += akima_tests();
        checks += interpolation_tests();
        checks += spiral_tests();
        checks += section_loft_tests();
        checks += material_semantics_tests();
        checks += material_legacy_tests();
        checks += material_resource_tests();
        checks += material_numeric_tests();
        checks += mesh_channel_tests();
        checks += mesh_extension_tests();
        checks += native_material_tests();
        checks += block_transform_tests();
        checks += native_input_tests();
        checks += native_attribute_input_tests();
        checks += native_attribute_lookup_tests();
        checks += material_catalog_registration_tests();
        checks += material_auxiliary_tests();
        checks += material_xml_integer_tests();
        checks += material_xml_float_tests();
        checks += material_root_tests();
        checks += material_version_tests();
        checks += material_replicator_tests();
        checks += material_layers_tests();
        checks += material_projection_tests();
        checks += material_projection_link_tests();
        checks += material_projection_math_tests();
        checks += native_list_input_tests();
        checks += native_id_tests();
        checks += native_dependency_tests();
        checks += native_reference_path_tests();
        checks += native_application_tests();
        checks += mesh_normal_tests();
        checks += mesh_tessellation_tests();
        checks += mesh_buffer_tests();
        guided_open_tests();
        guided_cap_tests();
        guided_endpoint_tests();
        rational_guided_tests();
        guided_ring_tests();
        command_metadata_tests();
        for (unsigned version = 0; version <= 2; ++version) {
            auto b = electrical_fixture(version);
            auto decoded = decode_binary_field("CerealDatas", b, "ElecParaData").at("decoded");
            auto &collections = decoded["collections"];
            auto &sections = decoded["section_parameters"]["collection"]["entries"];
            check(sections.size() == std::size(electrical_wire::sections),
                  "all explicit section layouts and generic layout accept nonempty records");
            check(sections[0]["section_group"]["section_type_label"] == "矩" &&
                      sections[2]["section_group"]["section_type_label"] == "工" &&
                      sections.back()["section_group"]["section_type_label"] == "C",
                  "section type labels use native codes including extended type codes");
            for (std::size_t i = 0; i < sections.size(); ++i) {
                auto &section = sections[i]["section_group"];
                auto &members = section["members"];
                check(section["version"] == i % 2 &&
                          sections[i]["member_0xcc"]["storage_uint32"] == 700 + i,
                      "old and current section headers preserve enclosing record boundaries");
                auto tail = bytesof(section["unused_capacity"]["source_bytes"]);
                check(!tail.empty() &&
                          std::all_of(tail.begin(), tail.end(), [](auto v) { return v == 0xa5; }),
                      "section capacity bytes are retained without interpretation as parameters");
                check(members[0]["count"] == 3 && members[0]["entries"][2]["storage_uint32"] == 83,
                      "section extension array is distinct from the fixed section members");
                auto string_index = i % 2 ? 3u : 2u;
                check(members[string_index]["text"] == "Section-A" &&
                          members[string_index + 1]["text"] == "" &&
                          members[string_index + 2]["text"] == "Grade-42",
                      "section strings include empty values and bounded variable lengths");
                for (auto &m : members)
                    if (m.contains("storage_uint16") || m.contains("storage_uint32")) {
                        auto value = m.contains("storage_uint16") ? m["storage_uint16"]
                                                                  : m["storage_uint32"];
                        check(value == 1000 + m["native_member_offset"].get<unsigned>(),
                              "section field widths and native member offsets match wire values");
                    }
            }
            const unsigned member_counts[] = {7, 10, 11, 18, 21, 22, 24};
            if (version == 0) {
                const std::map<std::int16_t, std::vector<std::string>> expected_subtypes = {
                    {31, {"工", "工"}},
                    {32, {"槽", "槽"}},
                    {33, {"L", "不等边L"}},
                    {34,
                     {"等边角钢┓┏", "不等边角钢长边┒┎", "不等边角钢短边┒┎", "等边角钢┓┗",
                      "等边角钢┎  ┒", "不等边等边角钢┎  ┒"}},
                    {35, {"槽][", "槽[]", "槽][", "槽[]", "槽[]", "槽[]"}}};
                for (const auto &expected : expected_subtypes) {
                    std::size_t first = 0;
                    while (first < sections.size() &&
                           sections[first]["section_group"]["section_type"] !=
                               (expected.first == 32 ? 31 : expected.first))
                        ++first;
                    check(first + 1 < sections.size(), "subtype wire fixture exists");
                    for (unsigned header = 0; header < 2; ++header) {
                        const auto &group = sections[first + header]["section_group"];
                        const auto group_offset = group["offset"].get<std::size_t>();
                        std::size_t subtype_offset = 0;
                        for (const auto &member : group["members"])
                            if (member["native_member_offset"] == 8)
                                subtype_offset = group_offset + member["offset"].get<std::size_t>();
                        check(subtype_offset > group_offset,
                              "subtype offset is relative to its group");
                        for (std::int16_t code = -1;
                             code <= static_cast<std::int16_t>(expected.second.size()) + 1;
                             ++code) {
                            auto changed = b;
                            std::memcpy(changed.data() + group_offset, &expected.first, 2);
                            std::memcpy(changed.data() + subtype_offset, &code, 2);
                            auto result =
                                decode_binary_field("CerealDatas", changed, "ElecParaData")
                                    .at("decoded")["section_parameters"]["collection"]["entries"]
                                                  [first + header]["section_group"];
                            const bool known =
                                code > 0 && std::size_t(code) <= expected.second.size();
                            check(result["section_type"] == expected.first &&
                                      result["section_subtype"] == code &&
                                      result.contains("section_type_label") == known,
                                  "section subtype labels require a supported signed subtype code");
                            if (known)
                                check(
                                    result["section_type_label"] == expected.second[code - 1],
                                    "native section labels retain orientation symbols and spaces");
                            for (const auto &member : result["members"])
                                if (member["native_member_offset"] == 8)
                                    check(member["name"] == "section_subtype" &&
                                              member["value"] == code &&
                                              member["storage_uint16"] == std::uint16_t(code) &&
                                              member["enum_status"] ==
                                                  (known ? "identified" : "unknown_value"),
                                          "subtype interpretation retains the exact source word");
                        }
                    }
                }
                for (const auto &section : sections) {
                    const auto &group = section["section_group"];
                    const auto type = group["section_type"].get<int>();
                    if (type == 201 || type == 202)
                        check(
                            group["section_type_label"] ==
                                    (type == 201 ? "矩型钢混凝土" : "圆管型钢混凝土") &&
                                !group.contains("section_subtype"),
                            "composite section labels do not turn size fields into subtype codes");
                }
            }
            auto &columns = collections[5]["entries"];
            for (unsigned v = 0; v <= 6; ++v)
                check(columns[v]["value"]["version"] == v &&
                          columns[v]["value"]["members"].size() == member_counts[v] &&
                          columns[v]["value"].contains("cereal_version") == (v == 0),
                      "all support-column versions preserve cereal registration and member "
                      "boundaries");
            auto &column_tail = columns[6]["value"]["members"];
            {
                auto &segments = column_tail[21];
                check(segments["name"] == "bar_segments_by_elevation" &&
                          segments["entries"][0]["key"]["name"] == "upper_elevation" &&
                          segments["entries"][0]["key"]["value"] == -17.25 &&
                          segments["entries"][0]["value"]["value"] == -123 &&
                          segments["entries"][0]["value"]["index_base"] == 1 &&
                          !segments["entries"][0]["key"].contains("unit"),
                      "elevation map keeps native signed segment ordinals and unknown units");
                auto expanded = b;
                auto offset = segments["offset"].get<std::size_t>();
                const std::uint64_t count = 3;
                std::memcpy(expanded.data() + offset, &count, sizeof(count));
                Bytes more;
                put(more, -17.25);
                put(more, std::int32_t(2));
                put(more, 4.75);
                put(more, std::int32_t(0));
                expanded.insert(expanded.begin() + offset + 8 + 12, more.begin(), more.end());
                auto mapped = decode_binary_field("CerealDatas", expanded, "ElecParaData")
                                  .at("decoded")["collections"][5]["entries"][6]["value"]["members"]
                                                [21]["entries"];
                check(mapped.size() == 3 && mapped[0]["key"]["value"] == -17.25 &&
                          mapped[1]["key"]["value"] == -17.25 &&
                          mapped[0]["value"]["value"] == -123 && mapped[1]["value"]["value"] == 2 &&
                          mapped[2]["key"]["value"] == 4.75 && mapped[2]["value"]["value"] == 0,
                      "segment maps preserve duplicate bounds, zero indices and source ordering");
            }
            for (unsigned v = 0; v <= 6; ++v) {
                auto &members = columns[v]["value"]["members"];
                check(members[6]["name"] == "column_section_count" && members[6]["value"] == 1006 &&
                          members[6]["storage_type"] == "int32" && !members[0].contains("name"),
                      "column count is identified without naming unrelated members");
                if (v >= 1)
                    check(members[7]["name"] == "column_type" && members[7]["value"] == 1007 &&
                              members[7]["enum_status"] == "unknown_value" &&
                              !members[7].contains("enum_label"),
                          "unrecognized native column types retain the original code");
                if (v >= 3)
                    check(members[13]["name"] == "fire_wall_height" &&
                              members[13]["native_property_name"] == "fireWallH" &&
                              members[13]["value"] == members[13]["float64_view"] &&
                              !members[13].contains("unit"),
                          "domain height mapping does not assume a length unit");
            }
            for (unsigned i = 22; i <= 23; ++i) {
                auto &map = column_tail[i];
                check(map["native_property_name"] == (i == 22 ? "colParam" : "beamParam") &&
                          map["entries"][0]["key"]["value"] == -31 &&
                          map["entries"][0]["value"].size() == 2 &&
                          map["entries"][0]["value"][1]["storage_type"] == "float64" &&
                          map["entries"][0]["value"][1]["value"] ==
                              map["entries"][0]["value"][1]["float64_view"],
                      "domain parameter maps preserve vector sizes and signed keys");
            }
            for (std::int32_t code : {0, 1, 2, -9}) {
                auto typed_bytes = b;
                auto offset = column_tail[7]["offset"].get<std::size_t>();
                std::memcpy(typed_bytes.data() + offset, &code, sizeof(code));
                auto changed_type =
                    decode_binary_field("CerealDatas", typed_bytes, "ElecParaData")
                        .at("decoded")["collections"][5]["entries"][6]["value"]["members"][7];
                check(changed_type["value"] == code &&
                          changed_type["storage_uint32"] == static_cast<std::uint32_t>(code) &&
                          changed_type["enum_status"] ==
                              (code == 0 || code == 1 ? "identified" : "unknown_value") &&
                          changed_type.contains("enum_label") == (code == 0 || code == 1),
                      "column enum labels are limited to confirmed values");
                if (code == 0 || code == 1)
                    check(changed_type["enum_label"] == (code == 0 ? "人字柱" : "格构柱"),
                          "column enum labels match their native type guards");
            }
            check(column_tail[21]["entries"][0]["key"]["float64_view"] == -17.25 &&
                      column_tail[22]["entries"][0]["value"][1]["storage_uint64"] ==
                          0x20000000000002ull &&
                      column_tail[23]["entries"][0]["key"]["int32_view"] == -31,
                  "column nested maps retain double keys and full-width vector storage");
            const unsigned beam_counts[] = {16, 19, 24, 29, 31, 33};
            for (unsigned v = 0; v <= 5; ++v) {
                auto &beam = collections[7]["entries"][v];
                auto &m = beam["members"];
                check(beam["version"] == v && m.size() == beam_counts[v] &&
                          !m[0].contains("cereal_version") && !m[2].contains("cereal_version"),
                      "truss versions reuse point and parameter registrations from earlier maps");
                check(m[10]["native_member_offset"] == 0x138 &&
                          m[11]["native_member_offset"] == 0x138 &&
                          m[10]["entries"][0]["storage_uint32"] == 1010 &&
                          m[11]["entries"][0]["storage_uint32"] == 1011,
                      "repeated truss source member occurrences are not merged or overwritten");
                if (v >= 1)
                    check(m[16]["entries"].size() == 2 &&
                              m[17]["entries"][1]["point_forces"][0]["flag_member_0x1c"] == 7,
                          "truss point vectors include nested loads and share global type "
                          "registration");
            }
            const unsigned human_counts[] = {12, 14, 29, 30, 44};
            for (unsigned v = 0; v <= 4; ++v) {
                auto &human = collections[8]["entries"][v];
                auto &m = human["members"];
                check(human["version"] == v && m.size() == human_counts[v] &&
                          m[3]["version"] == 6 && !m[3].contains("cereal_version"),
                      "human column versions reuse the support-column parameter registry");
                check(m[0]["name"] == "start_point" && m[1]["name"] == "end_point" &&
                          m[2]["name"] == "rotation" && m[2]["unit"] == "rad" &&
                          m[2]["value"] == m[2]["float64_view"] && m[2]["storage_uint64"] == 1002 &&
                          m[3]["members"][6]["name"] == "column_section_count",
                      "human column geometry parameters and nested semantic fields are preserved");
                check(m[8]["native_member_offset"] == 0x1a0 &&
                          m[9]["native_member_offset"] == 0x1a0 &&
                          m[8]["entries"][0]["storage_uint32"] == 1008 &&
                          m[9]["entries"][0]["storage_uint32"] == 1009,
                      "repeated human-column source member occurrences remain distinct");
            }
            {
                auto typed_bytes = b;
                auto &human = collections[8]["entries"][0]["members"];
                double angle = -0.75;
                auto offset = human[2]["offset"].get<std::size_t>();
                std::memcpy(typed_bytes.data() + offset, &angle, sizeof(angle));
                auto changed_human = decode_binary_field("CerealDatas", typed_bytes, "ElecParaData")
                                         .at("decoded")["collections"][8]["entries"][0]["members"];
                check(changed_human[2]["value"] == angle && changed_human[2]["unit"] == "rad" &&
                          changed_human[0].dump() == human[0].dump() &&
                          changed_human[1].dump() == human[1].dump(),
                      "native rotations remain radians and do not alter the original endpoints");
            }
            check(collections.size() == 9 + version &&
                      collections[0]["entries"][1]["value"]["members"][1]["float32_view"] == 3.5,
                  "electrical collection boundaries and material type registration");
            check(collections[1]["entries"][0]["key"] == -7 &&
                      collections[1]["entries"][1]["key"] == -7 &&
                      collections[2]["entries"][1]["value"] == 101,
                  "electrical maps preserve signed keys, duplicates and native ordering");
            auto &points = collections[3]["entries"];
            check(points[0]["value"]["position"] == Json({1.25, -2.5, 3.75}) &&
                      points[0]["value"].contains("cereal_version") &&
                      !points[1]["value"].contains("cereal_version"),
                  "mixed point versions share one cereal registration");
            auto &bars = collections[4]["entries"];
            check(!bars[0]["value"].contains("bar_forces") &&
                      bars[1]["value"]["bar_forces"][1]["flag_member_0x48"] == 9 &&
                      bars[1]["value"]["bar_forces"][1]["member_0x4c"]["storage_uint32"] == 305 &&
                      !bars[1]["value"]["bar_forces"][0]["force_member_0x0"].contains(
                          "cereal_version"),
                  "bar loads preserve nested shared point-force types and version-specific tails");
            auto &beams = collections[6]["entries"];
            check(beams[0]["value"]["members"][1]["native_member_offset"] == 8 &&
                      beams[1]["value"]["members"].size() == 9 &&
                      beams[2]["value"]["members"].size() == 8,
                  "truss beam parameter versions have distinct member layouts");
            auto &changed = decoded["changed_joints"]["entries"][0]["value"];
            check(
                !changed.contains("cereal_version") &&
                    !changed["point_forces"][0].contains("cereal_version") &&
                    changed["point_forces"][1]["flag_member_0x1c"] == 7 &&
                    changed["point_forces"][1]["members"][6]["int32_view"] == -31,
                "nested point forces share registration across different maps and keep raw flags");
            if (version >= 1)
                check(collections[9]["entries"][1]["member_0x0"][0]["storage_uint64"] ==
                          0x20000000000002ull,
                      "line group values retain integers above the JSON double precision limit");
            if (version >= 2)
                check(collections[10]["entries"][1]["member_0x40"][0]["float64_view"] == -17.25 &&
                          collections[10]["entries"][1]["member_0x38"]["float64_view"] == 19.75,
                      "grid records decode nested points and both vector boundaries");
            b.pop_back();
            check(!decode_binary_field("CerealDatas", b, "ElecParaData").contains("encoding"),
                  "truncated nonempty electrical collection is rejected");
            b = electrical_fixture(version);
            auto unsupported_offset = beams[0]["value"]["offset"].get<std::size_t>() + 4;
            std::uint32_t unsupported = 4;
            std::memcpy(b.data() + unsupported_offset, &unsupported, 4);
            check(!decode_binary_field("CerealDatas", b, "ElecParaData").contains("encoding"),
                  "unsupported truss beam parameter version is not guessed from neighboring "
                  "versions");
            b = electrical_fixture(version);
            std::uint32_t empty_version = 2;
            std::memcpy(b.data() + unsupported_offset, &empty_version, 4);
            auto payload_begin = unsupported_offset + 4;
            b.erase(b.begin() + payload_begin, b.begin() + payload_begin + 7 * 4);
            auto empty_parameter = decode_binary_field("CerealDatas", b, "ElecParaData");
            check(empty_parameter.contains("encoding"), "native version 2 has no member payload");
            const auto &empty_entries = empty_parameter["decoded"]["collections"][6]["entries"];
            check(empty_entries[0]["value"]["members"].empty() &&
                      empty_entries[0]["value"]["member_payload_status"] == "not_serialized" &&
                      empty_entries[1]["value"]["members"].size() == 9 &&
                      empty_entries[2]["value"]["members"].size() == 8,
                  "empty version 2 record does not consume subsequent entries or invent defaults");
            b = electrical_fixture(version);
            auto oversized = std::numeric_limits<std::uint64_t>::max();
            auto material_offset = collections[0]["offset"].get<std::size_t>();
            std::memcpy(b.data() + material_offset, &oversized, 8);
            check(!decode_binary_field("CerealDatas", b, "ElecParaData").contains("encoding"),
                  "malicious electrical collection length is rejected before allocation");
            for (unsigned collection : {5u, 7u, 8u}) {
                b = electrical_fixture(version);
                auto &entry = collections[collection]["entries"][0];
                auto &value = collection == 5 ? entry["value"] : entry;
                auto pos = value["offset"].get<std::size_t>() + 4;
                std::uint32_t future = 99;
                std::memcpy(b.data() + pos, &future, 4);
                check(!decode_binary_field("CerealDatas", b, "ElecParaData").contains("encoding"),
                      "unsupported design record versions reject the complete field");
            }
            b = electrical_fixture(version);
            auto group_offset = sections[0]["section_group"]["offset"].get<std::size_t>();
            std::uint16_t excessive_length = 801;
            std::memcpy(b.data() + group_offset + 2, &excessive_length, 2);
            check(!decode_binary_field("CerealDatas", b, "ElecParaData").contains("encoding"),
                  "section embedded length cannot exceed enclosing block capacity");
            b = electrical_fixture(version);
            std::int16_t bad_count = 32767;
            std::memcpy(b.data() + group_offset + 4, &bad_count, 2);
            check(!decode_binary_field("CerealDatas", b, "ElecParaData").contains("encoding"),
                  "section extension length cannot consume neighboring records");
            b = electrical_fixture(version);
            auto text_offset =
                group_offset +
                sections[0]["section_group"]["members"][2]["offset"].get<std::size_t>();
            b[text_offset + 2] = 0xff;
            auto altered = decode_binary_field("CerealDatas", b, "ElecParaData").at("decoded");
            auto &source_string = altered["section_parameters"]["collection"]["entries"][0]
                                         ["section_group"]["members"][2];
            check(!source_string.contains("text") &&
                      bytesof(source_string["source_bytes"])[0] == 0xff,
                  "unknown section code pages preserve original bytes without text substitution");
            b = electrical_fixture(version);
            auto negative_length = std::int16_t(-1);
            std::memcpy(b.data() + text_offset, &negative_length, 2);
            check(!decode_binary_field("CerealDatas", b, "ElecParaData").contains("encoding"),
                  "negative section string lengths reject the complete field");
            for (auto offset :
                 {column_tail[22]["offset"].get<std::size_t>(),
                  collections[7]["entries"][1]["members"][16]["offset"].get<std::size_t>()}) {
                b = electrical_fixture(version);
                std::memcpy(b.data() + offset, &oversized, 8);
                check(!decode_binary_field("CerealDatas", b, "ElecParaData").contains("encoding"),
                      "nested parameter-map and point-vector counts are bounded before allocation");
            }
            b = electrical_fixture(version);
            b.resize(decoded["changed_joints"]["offset"].get<std::size_t>());
            std::uint32_t outer_version = 0;
            std::memcpy(b.data(), &outer_version, 4);
            auto outer_zero = decode_binary_field("CerealDatas", b, "ElecParaData").at("decoded");
            // Unconfirmed floating views can be NaN for integer bit patterns;
            // compare the serialized view, which also retains every storage word.
            check(!outer_zero.contains("changed_joints") &&
                      outer_zero["collections"].dump() == collections.dump(),
                  "outer version zero retains all nested records without a changed-joints tail");
        }
        for (unsigned version : {0u, 1u}) {
            auto payload = drawing_fixture(false, version);
            auto decoded = decode_binary_field("CerealDatas", payload, "PSDrawingManager");
            auto &sheets = decoded.at("decoded").at("records");
            auto &plan = sheets[0]["named_values"];
            check(plan["scale_denominator"] == 101 &&
                      plan["connection_number_text_height"] == 103 &&
                      plan["member_number_text_height"] == 203 && plan["beam_end_gap"] == 204,
                  "plan scales and text heights follow native members");
            check(plan["draw_column_leader"] == 1 && plan["draw_section_table"] == 0 &&
                      plan["merged_output"] == 0 && plan["beam_drawing_mode"] == 1,
                  "native drawing flags are not UI combo indices");
            auto &elevation = sheets[1]["named_values"];
            check(elevation["column_drawing_mode"] == 2 && elevation["beam_end_gap"] == 205 &&
                      elevation["show_member_numbers"] == 1 && elevation["merged_output"] == 1,
                  "elevation parameters preserve noncontiguous native layout");
            check(sheets[3]["named_values"]["leader_text_height"] == 102 &&
                      sheets[4]["named_values"]["connection_drawing_type"] == 1 &&
                      sheets[4]["named_values"]["generate_connection_drawing"] == 0,
                  "node and connection settings have distinct meanings");
            check(sheets[0].contains("node_number_mode_code") == (version == 1) &&
                      sheets[4].contains("splice_annotation_origin_flag") == (version == 1),
                  "drawing version-specific parameters are only present in their archive version");
            if (version == 1)
                check(plan["node_number_mode"] == 1 &&
                          sheets[2]["named_values"]["node_number_mode"] == 1 &&
                          sheets[4]["named_values"]["splice_annotation_origin"] == 0,
                      "drawing version 1 tail fields decoded");
            payload.pop_back();
            check(!decode_binary_field("CerealDatas", payload, "PSDrawingManager")
                       .contains("encoding"),
                  "truncated drawing tail is not accepted");
        }
        auto subs =
            decode_binary_field("CerealDatas", drawing_fixture(true, 0), "SubsDrawingManager")
                .at("decoded");
        for (auto &sheet : subs["records"])
            check(sheet["named_values"]["dimension_text_height"] == 101 &&
                      sheet["named_values"]["name_text_height"] == 102 &&
                      sheet["named_values"]["table_text_height"] == 103 &&
                      sheet["unassigned_scale"] == 901,
                  "substation text heights are independent of the retained base double");
        check(subs["records"][2]["named_values"]["section_scale_denominator"] == 203 &&
                  subs["records"][3]["named_values"]["connection_scale_denominator"] == 204 &&
                  subs["records"][7]["named_values"]["elevation_scale_denominator"] == 201,
              "substation per-type scales retain native ordering");
        auto drawing_unknown = drawing_fixture(false, 0);
        // First sheet: 112-byte manager prefix, 16-byte cereal/base header.
        std::uint32_t unknown_paper = 99;
        std::memcpy(drawing_unknown.data() + 132, &unknown_paper, sizeof(unknown_paper));
        drawing_unknown[200] = 7;
        auto unknown_sheet = decode_binary_field("CerealDatas", drawing_unknown, "PSDrawingManager")
                                 .at("decoded")
                                 .at("records")[0];
        check(unknown_sheet["fields"][0]["enum_status"] == "unknown_value" &&
                  unknown_sheet["named_values"]["paper_size"] == 99 &&
                  unknown_sheet["named_values"]["draw_column_leader"] == 7,
              "unknown drawing enums and flag bytes remain lossless");
        std::uint32_t future_sheet_version = 2;
        std::memcpy(drawing_unknown.data() + 116, &future_sheet_version,
                    sizeof(future_sheet_version));
        check(!decode_binary_field("CerealDatas", drawing_unknown, "PSDrawingManager")
                   .contains("encoding"),
              "unsupported future drawing version is not decoded as version zero");
        Bytes zipped = {3,    0,    0,    0,    3,    0,    0,    0,    0x78, 0x9c,
                        0x4b, 0x4c, 0x4a, 0x06, 0x00, 0x02, 0x4d, 0x01, 0x27};
        auto inflated = decode_attribute(7, 99, zipped);
        check(inflated["codec"] == "zlib" && bytesof(inflated["data"]) == Bytes({'a', 'b', 'c'}),
              "zlib attribute payload decoded in every build mode");
        zipped.back() ^= 1;
        check(decode_attribute(7, 99, zipped)["encoding"] == "opaque",
              "corrupt zlib checksum is not accepted as decoded data");
        for (std::size_t n = 0; n < 50; ++n) {
            Bytes b;
            for (std::size_t i = 0; i < n; ++i)
                b.push_back(std::uint8_t(i * 37));
            check(unbase64(base64(b)) == b, "base64 roundtrip");
        }
        check(utf16({0x3d, 0xd8, 0x00, 0xde}) == "\xf0\x9f\x98\x80", "UTF16 surrogate pair");
        check(latin1({0xfc}) == "\xc3\xbc", "DEX Latin1");
        rejects([]() { utf16({0, 0xd8}); }, "UTF16 truncation rejected");
        rejects([]() { parse_commands({2, 0, 25, 0, 255, 255, 255, 127}); },
                "command overrun rejected");
        rejects([]() { parse_native(Bytes(36)); }, "native invalid base rejected");
        Bytes dex = {'P', '3', 'D',  'D', 'E',  'X', 0,   255, 10, 13,   0,    0, 0,    0,
                     0,   0,   0x30, 1,   0xfa, 1,   'R', 2,   0,  0x10, 0xfa, 1, 0xfc, 4};
        std::size_t pos = 0;
        auto obj = parse_dex(dex, pos);
        check(pos == dex.size() && obj["root"]["value"] == "\xc3\xbc", "DEX literal text");
        for (std::size_t n = 0; n < dex.size(); ++n) {
            auto short_dex = slice(dex, 0, n);
            rejects(
                [&]() {
                    std::size_t p = 0;
                    parse_dex(short_dex, p);
                },
                "DEX truncation rejected");
        }
        auto src = triangle();
        auto mirror = identity();
        mirror[0][0] = -1;
        Geometry placed;
        merge_geometry(placed, src, mirror);
        check(placed.faces[0] == Triangle{0, 2, 1}, "mirror winding");
        check((*placed.face_uvs[0])[1] == Point2{0, 1}, "mirror UV attachment");
        Geometry twice;
        merge_geometry(twice, placed, mirror);
        check(twice.faces == src.faces && twice.face_uvs == src.face_uvs &&
                  twice.vertices == src.vertices,
              "two mirrors restore geometry");
        src.texts.push_back({{"text", "annotation"},
                             {"origin", {1., 2., 3.}},
                             {"quaternion", {1., 0., 0., 0.}},
                             {"width", 3.},
                             {"height", 4.},
                             {"placement_matrix", identity()}});
        auto translation = identity();
        translation[1][3] = 5;
        Geometry text_placed;
        merge_geometry(text_placed, src, translation);
        check(text_placed.texts[0]["origin"] == Json({1., 7., 3.}), "text world position");
        check(text_placed.texts[0]["source_origin"] == Json({1., 2., 3.}),
              "text source frame retained");
        auto bad = identity();
        bad[3][0] = 1;
        rejects(
            [&]() {
                Geometry g;
                merge_geometry(g, src, bad);
            },
            "projective matrix rejected");
        Tessellation tolerance;
        tolerance.chord_tolerance = .01;
        auto ns = tolerance.segments(100);
        check(100 * (1 - std::cos(3.141592653589793 / ns)) <= .0100000001, "chord error bound");
        tolerance.max_segments = 2;
        rejects([&]() { tolerance.segments(100); }, "caller segment limit");
        Bytes poly;
        put(poly, std::uint8_t(0));
        put(poly, std::uint32_t(5));
        for (Point3 p : std::vector<Point3>{{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}, {0, 0, 0}})
            for (auto x : p)
                put(poly, x);
        auto g = reconstruct(Json::array({command(9, poly)}), {});
        check(g.faces.size() == 2 && g.unknown.empty(), "polygon triangulation");
        double area = 0;
        for (auto f : g.faces) {
            auto a = g.vertices[f[0]], b = g.vertices[f[1]], c = g.vertices[f[2]];
            area += ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) / 2;
        }
        check(std::abs(area - 4) < 1e-12, "polygon area and winding");
        Bytes pf;
        put(pf, std::uint32_t(0));
        put(pf, std::uint32_t(4));
        put(pf, std::uint32_t(4));
        for (int i : {1, 2, 3, 0})
            put(pf, std::int32_t(i));
        put(pf, std::uint32_t(0));
        put(pf, std::uint32_t(0));
        put(pf, std::uint32_t(3));
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                put(pf, 0.);
        put(pf, std::uint32_t(0));
        put(pf, std::uint32_t(0));
        pf.insert(pf.end(), 24, 0);
        auto degenerate = reconstruct(Json::array({command(25, pf)}), {});
        check(degenerate.faces.size() == 1 && degenerate.unknown.empty(),
              "source degenerate triangle retained");
        Bytes enum_raw;
        put(enum_raw, std::int32_t(-7));
        auto value = decode_binary_field("BeamType", enum_raw);
        check(value["decoded"]["value"] == -7 && value["decoded"]["enum_label"].is_null(),
              "unnamed enum retained");
        auto opaque = decode_binary_field("FutureField", {0x88, 0x77, 0x66});
        check(!opaque.contains("encoding") && opaque["binary_base64"] == base64({0x88, 0x77, 0x66}),
              "unknown bytes retained");
        auto object = [](unsigned cls, unsigned id, const std::string &name, const Json &fields) {
            Json children = Json::array();
            for (auto it = fields.begin(); it != fields.end(); ++it)
                children.push_back({{"name", it.key()}, {"value", it.value()}});
            return Json{{"class_id", cls},
                        {"object_id", id},
                        {"stream", {"test"}},
                        {"offset", id},
                        {"length", 1},
                        {"prefix", rawbytes({})},
                        {"root", {{"name", name}, {"children", children}}}};
        };
        auto one = object(2, 7, "Thing", {{"Value", 1}});
        auto other = object(2, 7, "Thing", {{"Value", 2}});
        auto conflict = build_graph_records(Json::array({one, one, other}), Json::array(),
                                            Json::array(), {}, {});
        check(get_object(conflict, 2, 7)["status"] == "ambiguous" &&
                  get_object(conflict, 2, 7)["records"].size() == 3,
              "conflicting objects never selected");
        auto copies =
            build_graph_records(Json::array({one, one}), Json::array(), Json::array(), {}, {});
        check(get_object(copies, 2, 7)["status"] == "identical_copies",
              "same identity copies retained");
        Json trees = Json::array();
        for (unsigned tree : {100, 200}) {
            trees.push_back(object(
                1, tree, "BimBaseTreeNodeData",
                {{"TreeId", tree}, {"NodeId", 1}, {"ParentNodeId", 65534}, {"NodeType", 9}}));
            trees.push_back(
                object(1, tree + 1, "BimBaseTreeNodeData",
                       {{"TreeId", tree},
                        {"NodeId", 2},
                        {"ParentNodeId", 1},
                        {"NodeType", 11},
                        {"ResourceRefers",
                         {nullptr, nullptr, nullptr, std::to_string((13ull << 32) | 99) + ","}}}));
        }
        auto graph = build_graph_records(trees, Json::array(), Json::array(), {13}, {"13:99"});
        check(graph["hierarchy"]["roots"].size() == 2 &&
                  graph["hierarchy"]["parent_edges"].size() == 2,
              "separate source trees");
        check(element_context(graph, 13, 99)["tree_nodes"] == Json({"1:101", "1:201"}),
              "element reachable through both source trees");
        Json model_nodes = Json::array(
            {object(1, 1, "BPModelTreeSet",
                    {{"Name", "root"}, {"ModelSetChilds", {"Archi"}}, {"ModelItems", {"Archi"}}}),
             object(1, 2, "BPModelTreeSet", {{"Name", "Archi"}}),
             object(2, 3, "BPModelTreeItem", {{"Name", "Archi"}, {"ModelId", 4}})});
        auto mg = build_graph_records(model_nodes, Json::array(), Json::array(), {4}, {});
        check(mg["hierarchy"]["model_edges"][0]["candidates"] == Json({"1:2"}) &&
                  mg["hierarchy"]["model_edges"][1]["candidates"] == Json({"2:3"}),
              "model set/item namespaces remain separate");
        auto cycle = build_graph_records(
            Json::array({object(1, 20, "BimBaseTreeNodeData",
                                {{"TreeId", 1}, {"NodeId", 0}, {"ParentNodeId", 1}}),
                         object(1, 21, "BimBaseTreeNodeData",
                                {{"TreeId", 1}, {"NodeId", 1}, {"ParentNodeId", 0}})}),
            Json::array(), Json::array(), {}, {});
        check(cycle["hierarchy"]["cycle_affected_nodes"].size() == 2, "source tree cycle reported");
        Bytes floor(96, 0);
        floor[0] = 0xa5;
        floor[92] = 0x7e;
        auto design = decode_binary_field("DesignPara", floor, "StructStandardFloorModel");
        check(design["decoded"]["fields"].size() == 21 &&
                  design["decoded"]["abi_fields"][1]["raw_hex"] == "7e000000",
              "ABI residue and padding preserved");
        Bytes floor_values = slice(floor, 0, 8);
        for (std::int32_t v :
             {175, 45, 35, 40, 30, 50, 21, 22, 25, 17, 4, 7, 6, 1, 2, 3, 4, 5, 6, 7, 8})
            put(floor_values, v);
        put(floor_values, std::uint32_t(0x01020304));
        auto floor_result =
            decode_binary_field("DesignPara", floor_values, "StructStandardFloorModel");
        const auto &fd = floor_result["decoded"];
        check(fd["named_values"] == Json({{"slab_thickness", 175},
                                          {"column_concrete_grade", 45},
                                          {"beam_concrete_grade", 35},
                                          {"shear_wall_concrete_grade", 40},
                                          {"slab_concrete_grade", 30},
                                          {"brace_concrete_grade", 50},
                                          {"slab_rebar_cover", 25},
                                          {"column_main_rebar_type", 4},
                                          {"beam_main_rebar_type", 7},
                                          {"wall_main_rebar_type", 6}}),
              "floor fields follow native member offsets");
        check(fd["identified_field_count"] == 10 && fd["unassigned_field_count"] == 11 &&
                  fd["fields"][6]["name"].is_null() && fd["fields"][6]["storage_uint32"] == 21,
              "floor unassigned members are not guessed or discarded");
        check(fd["fields"][0]["unit"] == "mm" && fd["fields"][8]["unit"] == "mm" &&
                  fd["fields"][1]["display_value"] == "C45" &&
                  fd["fields"][11]["enum_label"] == "HPB235" &&
                  unbase64(floor_result["binary_base64"]) == floor_values,
              "floor units, grade display and raw bytes preserved");
        const char *expected_rebar[] = {"HPB300",      "HRB335",      "HRB400",  "HRB500",
                                        "冷轧带肋550", "冷轧带肋600", "HTRB600", "HPB235"};
        for (std::int32_t v = -1; v <= 8; ++v) {
            for (auto off : {48, 52, 56})
                std::memcpy(floor_values.data() + off, &v, sizeof(v));
            auto result = decode_binary_field("DesignPara", floor_values,
                                              "StructStandardFloorModel")["decoded"];
            bool valid = true;
            for (auto index : {10, 11, 12}) {
                const auto &f = result["fields"][index];
                valid &=
                    f["value"] == v &&
                    (v >= 0 && v < 8
                         ? f["enum_label"] == expected_rebar[v] && f["enum_status"] == "identified"
                         : f["enum_label"].is_null() && f["enum_status"] == "unknown_value");
            }
            check(valid, "floor rebar indices include unknown and negative values");
        }
        check(
            !decode_binary_field("DesignPara", floor_values, "UnrelatedClass").contains("decoded"),
            "floor semantics are scoped to the source class");
        floor_values.pop_back();
        check(!decode_binary_field("DesignPara", floor_values, "StructStandardFloorModel")
                   .contains("decoded"),
              "truncated floor memory image is not accepted");
        auto cap_fixture = [](std::int32_t concrete, std::int32_t rebar, std::int32_t cap_type,
                              std::int32_t shape, std::int32_t steps,
                              const std::vector<std::int32_t> &heights) {
            Bytes b(24, 0); // Cap, pile section and pile base cereal/version pairs.
            for (std::int32_t v : {1, concrete, 701, 702})
                put(b, v);
            for (double v : {1.25, 2.5, 3.75})
                put(b, v);
            for (std::int32_t v : {71, 72, 73, 74, 75})
                put(b, v);
            put(b, 10.5);
            put(b, 20.5);
            put(b, std::int32_t(111));
            put(b, std::int32_t(-222));
            put(b, 12.5);
            put(b, 9.75);
            put(b, std::int64_t(123456789));
            for (std::int32_t v : {cap_type, 4321, steps})
                put(b, v);
            put(b, std::uint64_t(heights.size()));
            for (auto v : heights)
                put(b, v);
            put(b, std::int32_t(4));
            put(b, std::uint64_t(0)); // Primary points.
            put(b, std::int32_t(4));
            put(b, std::uint64_t(0)); // Secondary point arrays.
            for (std::int32_t v : {shape, 31, -32, 0, 0, rebar, 175, 22, 1200, 0, 3, 225, 16, 900})
                put(b, v);
            put(b, std::uint8_t(1));
            put(b, std::int32_t(126));
            return b;
        };
        auto decode_cap = [](const Bytes &b) {
            return decode_binary_field("BinaryData", b, "PBStandardSectionProfile",
                                       {{"Type", 0x40002}});
        };
        auto cap_bytes = cap_fixture(3, 0, 2, 1, 2, {400, 600});
        auto cap_result = decode_cap(cap_bytes);
        check(cap_result.value("encoding", "") == "pilecap_section_cereal",
              "cap fixture is decoded in its native class and type");
        const auto &cap = cap_result.at("decoded");
        const auto &pile = cap["pile_section"]["base"];
        check(pile["named_values"] == Json({{"concrete_grade", 3},
                                            {"section_shape", 1},
                                            {"section_width", 702},
                                            {"section_height", 701},
                                            {"pile_length", 10.5},
                                            {"x_offset", 111},
                                            {"y_offset", -222},
                                            {"rotation", 12.5},
                                            {"top_elevation", 9.75}}) &&
                  pile["members"][1]["enum_label"] == "C30" &&
                  pile["members"][14]["unit"] == "mm" && pile["members"][17]["unit"] == "m" &&
                  pile["members"][16]["unit"] == "deg",
              "pile getters and native angle conversion identify units");
        check(cap["named_values"] == Json({{"cap_type", 2},
                                           {"layout_preset_index", 4321},
                                           {"plan_shape", 1},
                                           {"step_count", 2},
                                           {"top_offset_x", 31},
                                           {"top_offset_y", -32}}) &&
                  cap["members"][0]["enum_label"] == "阶形现浇" &&
                  cap["members"][3]["enum_label"] == "矩形" &&
                  cap["step_heights"] == Json({400, 600}) &&
                  cap["step_heights_order"] == "lower_to_upper",
              "cap dialog bindings preserve lower and upper step order");
        check(cap["reinforcement"][0]["named_values"] == Json({{"rebar_type", 0},
                                                               {"spacing", 175},
                                                               {"diameter", 22},
                                                               {"distribution_width", 1200}}) &&
                  cap["reinforcement"][0]["members"][0]["enum_label"] == "HPB235" &&
                  cap["reinforcement"][1]["members"][0]["enum_label"] == "HRB400" &&
                  cap["reinforcement"][1]["named_values"]["diameter"] == 16,
              "reinforcement diameter and spacing follow native storage, with scoped enum");
        check(pile["identified_member_count"] == 9 && pile["unassigned_member_count"] == 9 &&
                  cap["identified_member_count"] == 6 && cap["unassigned_member_count"] == 2 &&
                  cap["members"][7]["name"].is_null() && cap["members"][7]["value"] == 126 &&
                  unbase64(cap_result["binary_base64"]) == cap_bytes,
              "cap annotations retain unknown fields and every original byte");
        const char *cap_rebar[] = {"HPB235", "HPB300", "HRB335",  "HRB400", "HRB500",
                                   "CRB550", "CRB600", "HTRB600", "T63"};
        const char *cap_types[] = {"阶形预制", "锥形预制", "阶形现浇", "锥形现浇"};
        const char *cap_shapes[] = {"圆形", "矩形", "正多边形", "多边形"};
        for (std::int32_t v = -1; v <= 14; ++v) {
            auto d = decode_cap(cap_fixture(v, v, v, v, 1, {800})).at("decoded");
            auto enum_is = [&](const Json &field, int count, const std::string &label) {
                return field["value"] == v &&
                       (v >= 0 && v < count
                            ? field["enum_label"] == label && field["enum_status"] == "identified"
                            : field["enum_label"].is_null() &&
                                  field["enum_status"] == "unknown_value");
            };
            check(enum_is(d["pile_section"]["base"]["members"][1], 14,
                          "C" + std::to_string(15 + 5 * v)) &&
                      enum_is(d["reinforcement"][0]["members"][0], 9,
                              v >= 0 && v < 9 ? cap_rebar[v] : "") &&
                      enum_is(d["members"][0], 4, v >= 0 && v < 4 ? cap_types[v] : "") &&
                      enum_is(d["members"][3], 4, v >= 0 && v < 4 ? cap_shapes[v] : ""),
                  "cap enum tables preserve negative and out-of-range codes");
        }
        check(decode_cap(cap_fixture(0, 0, 0, 0, 1, {800}))["decoded"]["step_heights"] ==
                  Json({800}),
              "single cap step retains its height");
        for (auto steps : {0, 1, 3}) {
            auto d = decode_cap(cap_fixture(0, 0, 0, 0, steps, {400, 600})).at("decoded");
            check(!d.contains("step_heights") &&
                      d["step_heights_status"] == "unsupported_step_layout" &&
                      d["integer_array_0x250"] == Json({400, 600}),
                  "unrecognized cap step layout is retained without a semantic alias");
        }
        check(decode_binary_field("BinaryData", cap_bytes, "PBStandardSectionProfile",
                                  {{"Type", 0x40001}})
                      .value("encoding", "") != "pilecap_section_cereal",
              "cap semantics require the native type discriminator");
        cap_bytes[0] = 1;
        check(decode_cap(cap_bytes).value("encoding", "") != "pilecap_section_cereal",
              "unsupported cap cereal version is not accepted");
        cap_bytes[0] = 0;
        cap_bytes.pop_back();
        check(decode_cap(cap_bytes).value("encoding", "") != "pilecap_section_cereal",
              "truncated cap is not accepted");
        for (std::int32_t shape : {-1, 0, 2, 3, 5, 6, 7, 99}) {
            auto b = cap_fixture(3, 0, 2, 1, 2, {400, 600});
            std::memcpy(b.data() + 24, &shape, sizeof(shape));
            const auto base = decode_cap(b)["decoded"]["pile_section"]["base"];
            const auto &values = base["named_values"];
            check(!values.contains("section_width") && !values.contains("section_height") &&
                      (shape == 2 ? values["section_diameter"] == 701 &&
                                        base["members"][0]["enum_label"] == "圆形" &&
                                        base["members"][3]["name"].is_null()
                                  : !values.contains("section_diameter") &&
                                        base["members"][0]["enum_label"].is_null()),
                  "pile dimensions are scoped to the source shape, with unknown codes retained");
            if (shape == 5 || shape == 6)
                check(values["section_outer_diameter"] == 701 &&
                          values["section_wall_thickness"] == 71 &&
                          base["members"][7]["native_member"] == "0x1f0" &&
                          base["members"][7]["unit"] == "mm" &&
                          base["identified_member_count"] == 9 &&
                          base["members"][0]["enum_status"] == "unknown_value",
                      "hollow circular dimensions do not guess engineering subtype names");
            else if (shape == 7)
                check(
                    values["first_circle_radius"] == 702 && values["second_circle_radius"] == 701 &&
                        values["circle_center_spacing"] == 71 &&
                        !values.contains("section_wall_thickness") &&
                        base["members"][7]["native_member"] == "0x1f0" &&
                        base["members"][7]["unit"] == "mm" &&
                        base["identified_member_count"] == 10 &&
                        base["members"][0]["enum_status"] == "unknown_value",
                    "two-circle pile parameters use radii and center spacing, not tube thickness");
            else
                check(!values.contains("section_outer_diameter") &&
                          !values.contains("section_wall_thickness") &&
                          base["members"][7]["name"].is_null(),
                      "wall thickness interpretation is limited to confirmed pile shapes");
            if (shape != 7)
                check(!values.contains("circle_center_spacing") &&
                          !values.contains("first_circle_radius") &&
                          !values.contains("second_circle_radius"),
                      "two-circle dimensions do not leak into other pile types");
        }
        auto cap_geometry = [&](int shape, int count, int edges, const Json &positions,
                                const Json &profiles) {
            auto raw = cap_fixture(3, 0, 2, shape, 2, {400, 600});
            Bytes b = slice(raw, 0, 160);
            auto points = [&](const Json &array) {
                put(b, std::uint64_t(array.size()));
                for (const auto &p : array) {
                    put(b, std::uint64_t(3));
                    for (const auto &v : p)
                        put(b, v.get<double>());
                }
            };
            put(b, std::int32_t(count));
            points(positions);
            put(b, std::int32_t(edges));
            put(b, std::uint64_t(profiles.size()));
            for (const auto &profile : profiles)
                points(profile);
            auto tail = slice(raw, 184, raw.size() - 184);
            b.insert(b.end(), tail.begin(), tail.end());
            return decode_cap(b);
        };
        const Json positions = {{-500., 100., 0.}, {500., -100., 2.}};
        const Json contours = {{{-900., -800., 0.}, {900., -800., 0.}, {0., 800., 0.}},
                               {{-600., -500., 0.}, {600., -500., 0.}, {0., 500., 0.}}};
        auto parameterized = cap_geometry(3, 2, 3, positions, contours)["decoded"];
        check(parameterized["pile_layout"]["positions"] == positions &&
                  parameterized["pile_layout"]["count_status"] == "consistent" &&
                  parameterized["cap_profiles"]["step_count_status"] == "consistent" &&
                  parameterized["cap_profiles"]["profiles"][0]["vertices"] == contours[0] &&
                  parameterized["cap_profiles"]["profiles"][1]["vertices"] == contours[1],
              "pile locations and lower-to-upper cap polygons remain distinct");
        const Json circles = {{{1200., 0., 0.}}, {{800., 0., 0.}}};
        auto circular = cap_geometry(0, 2, 0, positions, circles)["decoded"];
        check(circular["cap_profiles"]["profiles"][0]["kind"] == "circle" &&
                  circular["cap_profiles"]["profiles"][0]["radius"] == 1200. &&
                  circular["cap_profiles"]["profiles"][1]["radius"] == 800. &&
                  !circular["cap_profiles"]["profiles"][0].contains("vertices") &&
                  circular["secondary_point_arrays"] == circles,
              "native circular radius tuples are not interpreted as polygon vertices");
        auto future_circle = circles;
        future_circle[0][0][1] = 3.;
        auto unknown_profile = cap_geometry(0, 2, 0, positions, future_circle)["decoded"];
        check(unknown_profile["cap_profiles"]["profiles"][0]["kind"] == "unassigned" &&
                  unknown_profile["cap_profiles"]["profiles"][0]["source_values"] ==
                      future_circle[0],
              "unrecognized circular tuple keeps every value without guessing its layout");
        auto mismatch = cap_geometry(3, -1, 4, positions, Json({contours[0]}))["decoded"];
        check(mismatch["pile_layout"]["count_status"] == "mismatch" &&
                  mismatch["cap_profiles"]["step_count_status"] == "mismatch" &&
                  mismatch["cap_profiles"]["profiles"][0]["edge_count_status"] == "mismatch" &&
                  mismatch["pile_layout"]["positions"] == positions,
              "inconsistent source counts are reported without truncation or synthesis");
        auto future_shape = cap_geometry(99, 2, 3, positions, contours)["decoded"];
        check(future_shape["cap_profiles"]["profiles"][0]["kind"] == "unassigned" &&
                  future_shape["cap_profiles"]["profiles"][0]["source_values"] == contours[0],
              "unknown cap shapes retain their raw profile arrays");
        auto link_fixture = [](int version, const std::vector<std::vector<int>> &floors,
                               const std::vector<std::vector<int>> &links,
                               const std::vector<int> &codes) {
            Bytes b;
            for (int v : {version, 0, 0, codes.at(0), codes.at(1)})
                put(b, std::int32_t(v));
            bool first_group = true;
            for (auto *groups : {&floors, &links}) {
                put(b, std::uint64_t(groups->size()));
                for (const auto &values : *groups) {
                    if (first_group) {
                        put(b, std::uint32_t(0));
                        first_group = false;
                    }
                    put(b, std::uint32_t(0));
                    put(b, std::uint64_t(values.size()));
                    for (int value : values)
                        put(b, std::int32_t(value));
                }
            }
            for (int v : {0, 0, codes.at(2), codes.at(3), codes.at(4), codes.at(5)})
                put(b, std::int32_t(v));
            put(b, std::uint64_t(3));
            for (auto v : {0, 1, 0})
                put(b, std::uint8_t(v));
            if (version == 1)
                put(b, std::int64_t(1234567890123));
            return b;
        };
        const std::vector<int> link_codes{1, 3, 2, 4, 5, 3};
        auto link_bytes = link_fixture(1, {{7, 3, 7}, {}}, {{-1, 2147483647}}, link_codes);
        auto link = decode_binary_field("CerealDatas", link_bytes, "AssemblyLinkModel");
        check(link.value("encoding", "") == "assembly_settings_cereal",
              "nonempty native link merge groups are decoded");
        const auto &ld = link["decoded"];
        const auto &lg = ld["link_merge_set"]["collections"];
        check(lg[0]["name"] == "floor_groups" && lg[1]["name"] == "link_groups" &&
                  lg[0]["entries"][0]["values"] == Json({7, 3, 7}) &&
                  lg[0]["entries"][1]["values"] == Json::array() &&
                  lg[1]["entries"][0]["values"] == Json({-1, 2147483647}),
              "merge groups preserve signed values, duplicates and source ordering");
        check(lg[0]["entries"][0]["cereal_version"] == 0 &&
                  !lg[0]["entries"][1].contains("cereal_version") &&
                  !lg[1]["entries"][0].contains("cereal_version") &&
                  ld["boolean_vector"] == Json({0, 1, 0}) &&
                  ld["storey_design_flags"]["source_values"] == Json({0, 1, 0}) &&
                  ld["load_g_para_id"] == 1234567890123LL &&
                  unbase64(link["binary_base64"]) == link_bytes,
              "group type version is shared across collections and trailing data is preserved");
        check(ld["link_merge_set"]["fields"][0]["enum_label"] == "1/1,2/1,1/2,3/2..." &&
                  ld["link_merge_set"]["fields"][1]["enum_label"] == "任选楼层节点连接归并" &&
                  ld["display_control"]["fields"][0]["enum_label"] == "半透明显示" &&
                  ld["display_control"]["fields"][1]["enum_label"] == "精细显示-带焊接标记" &&
                  ld["display_control"]["fields"][2]["enum_label"] == "简化显示" &&
                  ld["display_control"]["fields"][3]["enum_label"] == "精细显示",
              "native merge and display enums use their field-specific mappings");
        auto second_first =
            decode_binary_field("CerealDatas", link_fixture(0, {}, {{42}, {9}}, link_codes),
                                "AssemblyLinkModel")["decoded"];
        check(second_first["link_merge_set"]["collections"][1]["entries"][0]["cereal_version"] ==
                      0 &&
                  second_first["link_merge_set"]["collections"][1]["entries"][1]["values"] ==
                      Json({9}) &&
                  !second_first.contains("load_g_para_id"),
              "first nonempty merge collection carries the shared cereal type version");
        auto unknown_link =
            decode_binary_field("CerealDatas", link_fixture(0, {}, {}, {-1, 99, 3, 2, 4, 0}),
                                "AssemblyLinkModel")["decoded"];
        bool unknown_modes = true;
        for (const auto *block : {"link_merge_set", "display_control"})
            for (const auto &f : unknown_link[block]["fields"])
                unknown_modes &= f["enum_status"] == "unknown_value" && f["enum_label"].is_null();
        check(unknown_modes && unknown_link["display_control"]["enum_codes"] == Json({3, 2, 4, 0}),
              "unrecognized display and merge enum values are not coerced");
        for (auto offset : {28, 32}) {
            auto unsupported = link_bytes;
            unsupported[offset] = 1;
            check(decode_binary_field("CerealDatas", unsupported, "AssemblyLinkModel")
                          .value("encoding", "") != "assembly_settings_cereal",
                  "unsupported merge group cereal or payload version is rejected");
        }
        auto truncated_group = slice(link_bytes, 0, 46);
        check(decode_binary_field("CerealDatas", truncated_group, "AssemblyLinkModel")
                      .value("encoding", "") != "assembly_settings_cereal",
              "truncated group members do not produce a partial success");
        auto oversized_group = link_bytes;
        for (auto offset = 36; offset < 44; ++offset)
            oversized_group[offset] = 0xff;
        check(decode_binary_field("CerealDatas", oversized_group, "AssemblyLinkModel")
                      .value("encoding", "") != "assembly_settings_cereal",
              "group member count is bounded by remaining input");
        NativeScene views;
        auto shared = std::make_shared<GeometryDefinition>();
        shared->source_key = "stream@1";
        shared->geometry = triangle();
        auto independent = std::make_shared<GeometryDefinition>(*shared);
        independent->source_key = "stream@2";
        views.definitions = {shared, independent};
        views.metadata = {
            {"color_tables", Json::array()},
            {"materials",
             {{"definitions",
               Json::array(
                   {{{"scope", "model:13"}, {"id", 7}, {"base_color_rgb", {1., 0., 0.}}},
                    {{"scope", "global"}, {"id", 7}, {"base_color_rgb", {0., 1., 0.}}}})}}}};
        SceneElement ve;
        ve.metadata = {{"model_id", 13}, {"unknown", Json::array()}};
        GeometryInstance vi;
        vi.definition = 0;
        vi.matrix = mirror;
        vi.style = {{"material_id", 7}};
        ve.instances = {vi, vi};
        vi.definition = 1;
        ve.instances.push_back(vi);
        views.elements.push_back(ve);
        std::vector<const Geometry *> borrowed;
        views.for_each_primitive([&](const PrimitiveView &view) {
            borrowed.push_back(view.geometry);
            check(view.material_status == "resolved" &&
                      view.material_candidates == std::vector<std::size_t>{0},
                  "model material takes precedence over global");
            check(view.winding_reversed && view.uv_status == "explicit_source",
                  "primitive mirror and source UV contract");
        });
        check(borrowed.size() == 3 && borrowed[0] == borrowed[1] && borrowed[0] != borrowed[2],
              "native references share arrays; equal independent definitions remain distinct");
        check(views.summary()["reused_definitions"] == 1 &&
                  views.summary()["stored_referenced_triangles"] == 2,
              "native reuse summary counts source identities");
        views.metadata["materials"]["definitions"].push_back(
            views.metadata["materials"]["definitions"][0]);
        views.for_each_primitive([&](const PrimitiveView &view) {
            check(view.material_status == "ambiguous" &&
                      view.material_candidates == std::vector<std::size_t>({0, 2}),
                  "ambiguous local materials not replaced by a global candidate");
        });
        shared->geometry.primitive_ranges[0]["count"] = 2;
        rejects([&]() { views.for_each_primitive([](const PrimitiveView &) {}); },
                "out of bounds primitive range rejected");
        std::cout << checks << " checks passed\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
