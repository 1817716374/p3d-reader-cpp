#include <p3d/swept_mesh.hpp>
#include <p3d/csg.hpp>
#include <p3d/csg_mesh_tree.hpp>
#include "graphics_native.hpp"
#include <cstring>

namespace {
using namespace p3d;
template <class T> void put(Bytes &b, T value) {
    const auto *p = reinterpret_cast<const std::uint8_t *>(&value);
    b.insert(b.end(), p, p + sizeof(value));
}
template <class T> void write(Bytes &b, std::size_t offset, T value) {
    require(offset <= b.size() && sizeof(value) <= b.size() - offset, "fixture write bounds");
    std::memcpy(b.data() + offset, &value, sizeof(value));
}
struct Packet {
    Bytes b = Bytes(16);
    Packet() {
        std::memcpy(b.data(), "bg0001fb", 8);
    }
    void align() {
        while (b.size() % 8)
            b.push_back(0);
    }
    std::size_t table(std::initializer_list<std::uint16_t> offsets, std::uint16_t size) {
        align();
        const auto vt = b.size();
        put(b, std::uint16_t(4 + offsets.size() * 2));
        put(b, size);
        for (auto x : offsets)
            put(b, x);
        align();
        const auto at = b.size();
        b.resize(at + size);
        write(b, at, std::int32_t(at - vt));
        return at;
    }
    void reference(std::size_t from, std::size_t to) {
        require(to >= from + 4 && to - from <= UINT32_MAX, "fixture forward offset");
        write(b, from, std::uint32_t(to - from));
    }
    std::size_t group(const std::vector<Point3> &points, std::int32_t type) {
        const auto g = table({4, 8}, 12);
        write(b, g + 4, type);
        align();
        const auto v = b.size();
        put(b, std::uint32_t(1));
        put(b, std::uint32_t(0));
        reference(g + 8, v);
        const auto variant = table({4, 8}, 12);
        reference(v + 4, variant);
        write(b, variant + 4, std::uint8_t(4));
        const auto line = table({4}, 8);
        reference(variant + 8, line);
        align();
        put(b, std::uint32_t(0)); // vector's doubles begin at an eight-byte boundary
        const auto values = b.size();
        put(b, std::uint32_t(points.size() * 3));
        for (auto p : points)
            for (auto x : p)
                put(b, x);
        reference(line + 4, values);
        return g;
    }
};
void block(Bytes &b, const Bytes &v) {
    put(b, std::int32_t(v.size()));
    b.insert(b.end(), v.begin(), v.end());
}
Bytes archive(const Bytes &geometry, bool repeated, bool translated) {
    Bytes tree;
    put(tree, std::int32_t(0x1ee290));
    put(tree, std::int32_t(0));
    put(tree, std::int32_t(repeated ? 2 : 1));
    put(tree, std::int32_t(0));
    if (repeated)
        put(tree, std::int32_t(0));
    put(tree, std::int32_t(0)); // caches
    put(tree, std::int32_t(translated));
    if (translated)
        put(tree, std::int32_t(0));
    put(tree, std::int32_t(0));     // geometry GUIDs
    put(tree, std::int32_t(0));     // union
    tree.insert(tree.end(), 18, 0); // GUID, isOld, children
    Bytes b;
    put(b, std::int32_t(22));
    put(b, std::int32_t(1));
    block(b, geometry);
    put(b, std::int32_t(0)); // saved caches
    put(b, std::int32_t(translated));
    if (translated)
        for (unsigned i = 0; i < 3; ++i)
            for (unsigned j = 0; j < 4; ++j)
                put(b, j == 3 ? double(i == 0) : double(i == j));
    put(b, std::int32_t(36));
    block(b, tree);
    block(b, {});
    block(b, {});
    return b;
}
double volume(const CsgTreeMesh &mesh) {
    long double v = 0;
    for (auto f : mesh.faces) {
        const auto a = mesh.vertices.at(f[0]), b = mesh.vertices.at(f[1]),
                   c = mesh.vertices.at(f[2]);
        v += a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
             a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    return double(v / 6);
}
} // namespace

p3d::Bytes swept_binary_fixture(bool capped) {
    Packet p;
    const auto root = p.table({4, 8}, 12);
    p.reference(8, root);
    write(p.b, root + 4, std::uint8_t(20));
    const auto sweep = p.table({4, 8, std::uint16_t(capped ? 12 : 0)}, 16);
    p.reference(root + 8, sweep);
    write(p.b, sweep + 12, std::uint8_t(capped));
    p.reference(sweep + 4,
                p.group({{-1, -1, 0}, {1, -1, 0}, {1, 1, 0}, {-1, 1, 0}, {-1, -1, 0}}, 2));
    p.reference(sweep + 8, p.group({{0, 0, 0}, {0, 0, 4}}, 1));
    return p.b;
}

unsigned swept_binary_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool condition, const char *why) {
        ++checks;
        require(condition, why);
    };
    auto spline = [](int order, bool closed, std::optional<int> weights, std::optional<int> knots,
                     unsigned scalars = 6) {
        Packet p;
        const auto root = p.table({4, 8}, 12);
        p.reference(8, root);
        write(p.b, root + 4, std::uint8_t(3));
        const auto curve =
            p.table({4, 8, 12, std::uint16_t(weights ? 16 : 0), std::uint16_t(knots ? 20 : 0)}, 24);
        p.reference(root + 8, curve);
        write(p.b, curve + 4, std::int32_t(order));
        write(p.b, curve + 8, std::uint8_t(closed));
        auto values = [&](std::size_t slot, std::uint32_t count, unsigned stored) {
            p.align();
            put(p.b, std::uint32_t(0));
            const auto start = p.b.size();
            put(p.b, count);
            for (unsigned i = 0; i < stored; ++i)
                put(p.b, double(i));
            p.reference(slot, start);
        };
        values(curve + 12, scalars, 6);
        if (weights)
            values(curve + 16, std::uint32_t(*weights), 3);
        if (knots)
            values(curve + 20, std::uint32_t(*knots), 5);
        return p.b;
    };
    auto native_curve = [](const Bytes &b) {
        Bytes entry(36);
        write(entry, 4, std::int32_t(1));
        write(entry, 28, std::uint64_t(b.size()));
        entry.insert(entry.end(), b.begin(), b.end());
        return graphics_entry_native_input(entry).at("with_project");
    };
    for (bool closed : {false, true})
        for (const auto weights :
             {std::optional<int>{}, std::optional<int>{0}, std::optional<int>{-1},
              std::optional<int>{2}, std::optional<int>{3}})
            for (bool stored_knots : {false, true}) {
                const auto b =
                    spline(2, closed, weights,
                           stored_knots ? std::optional<int>(closed ? 5 : 4) : std::nullopt);
                const auto result = native_curve(b);
                check(result.at("status") == "geometry_constructed",
                      "native B-spline reader constructs supported open/closed weighted input");
                const auto &c = result.at("construction");
                check(c.at("rational") == (weights && *weights > 0) &&
                          c.at("input_poles_already_weighted") == true && c.at("pole_count") == 2 &&
                          c.at("effective_knot_count") == (closed ? 5 : 4),
                      "native B-spline uses weight pointer and original homogeneous poles");
                check(c.at("knots_source") == (stored_knots ? "copied" : "generated_uniform") &&
                          c.at("copied_weight_count") == (weights && *weights > 0 ? 2 : 0),
                      "native B-spline ignores excess weights and generates absent knots");
                check(result.at("entry_restore").at("status") == "retained" &&
                          result.at("parametric_append_input").at("status") == "not_evaluated",
                      "B-spline input construction does not assert a separately unsupported clone");
            }
    for (auto order : {INT32_MIN, -1, 0, 1, 3, INT32_MAX})
        check(native_curve(spline(order, false, {}, {})).at("reason") ==
                  "native_bspline_populate_guard_leaves_uninitialized_temporary",
              "failed native populate must not become a constructed null or empty curve");
    for (auto count : {-1, 1, 3, 5})
        check(native_curve(spline(2, false, {}, count)).at("reason") ==
                  "native_bspline_knot_count_leaves_uninitialized_temporary",
              "native knot count mismatch is distinct from mathematical knot validity");
    check(native_curve(spline(2, false, 1, {})).at("reason") ==
              "native_bspline_weight_vector_overread",
          "short weight vector cannot be treated as a nonrational curve or padded implicitly");
    const auto remainder = native_curve(spline(2, false, {}, {}, 7));
    check(remainder.at("status") == "geometry_constructed" &&
              remainder.at("construction").at("ignored_tail_scalars") == 1,
          "native B-spline ignores an unconsumed scalar remainder");
    auto nonfinite = spline(2, false, {}, {});
    write(nonfinite, nonfinite.size() - 8, UINT64_C(0x7ff8000000000001));
    check(native_curve(nonfinite).at("status") == "geometry_constructed",
          "native B-spline input copies nonfinite data without a geometric validity test");
    auto truncated_spline = spline(2, false, {}, {});
    truncated_spline.pop_back();
    check(native_curve(truncated_spline).at("status") == "not_evaluated",
          "every copied B-spline coordinate must lie within the source geometry bytes");
    for (bool capped : {false, true}) {
        const auto binary = swept_binary_fixture(capped);
        const auto decoded = decode_bgfb(binary);
        const auto &source = decoded.at("geometry");
        check(source.at("_type") == "P3DSweptBody" && source.at("capped") == capped,
              "binary union 20 and absent capped default reach sweep source");
        Bytes entry(36);
        write(entry, 4, std::int32_t(6));
        write(entry, 28, std::uint64_t(binary.size()));
        entry.insert(entry.end(), binary.begin(), binary.end());
        const auto native = graphics_entry_native_input(entry).at("with_project");
        check(native.at("status") == "geometry_constructed" &&
                  native.at("construction").at("profile").at("output_member_count") == 1 &&
                  native.at("construction").at("path").at("output_member_count") == 1,
              "binary swept fixture is accepted by native constructor interpretation");
        const auto mesh = mesh_bgfb_swept_body(source);
        check(mesh.status == "meshed" && mesh.points.size() == 8 &&
                  mesh.point_indices.size() == (capped ? 48 : 32) && mesh.source == source,
              "binary sweep yields capped or uncapped mesh without source mutation");
        check(mesh.normal_indices.size() == mesh.point_indices.size() &&
                  mesh.parameter_indices.size() == mesh.point_indices.size(),
              "binary source generates independent normal and UV channels");
        check(decode_bgfb(binary) == decoded, "native meshing cannot mutate the decoded packet");
    }
    for (bool repeated : {false, true}) {
        const auto binary = archive(swept_binary_fixture(true), repeated, true);
        const auto decoded = decode_csg_bytes(binary);
        const auto result = evaluate_csg_polyface_archive(decoded);
        check(result.result.status == "evaluated" && result.result.meshes.size() == 1,
              "serialized CSG archive sweep reaches actual source meshing");
        check(result.solid_sources.size() == 1 &&
                  result.solid_snapshots.size() == (repeated ? 2 : 1),
              "serialized repeated source indices preserve identity and separate snapshots");
        check(std::abs(volume(result.result.meshes.at(0)) - (repeated ? 24 : 16)) < 1e-8,
              "binary repeated sweep applies successive translation before union");
        for (const auto &face : result.result.meshes.at(0).face_sources)
            check(face.source_kind == CsgSourceKind::solid && face.geometry_index == 0 &&
                      face.solid_snapshot && *face.solid_snapshot < result.solid_snapshots.size(),
                  "binary CSG triangle provenance points to a retained sweep snapshot");
        check(result.solid_sources[0].source == decoded["geometries"][0]["geometry"]["geometry"],
              "binary CSG keeps original source before runtime transformations");
        check(decode_csg_bytes(binary) == decoded, "binary CSG evaluation preserves archive");
    }
    const auto nested =
        decode_csg_bytes(archive(archive(swept_binary_fixture(true), false, true), false, true));
    const auto result = evaluate_csg_polyface_archive(nested);
    check(result.result.status == "evaluated" && result.solid_source_paths.size() == 1 &&
              result.solid_source_paths[0].size() == 2 && result.solid_placements.size() == 2,
          "nested serialized archives preserve sweep identity and sequential placements");
    check(std::abs(volume(result.result.meshes.at(0)) - 16) < 1e-8,
          "nested binary archive placements retain sweep volume");
    auto corrupt = swept_binary_fixture(true);
    corrupt.resize(corrupt.size() - 1);
    const auto failed =
        evaluate_csg_polyface_archive(decode_csg_bytes(archive(corrupt, false, false)));
    check(failed.result.status != "evaluated" && failed.result.meshes.empty(),
          "truncated source vector cannot publish a partial CSG mesh");
    return checks;
}
