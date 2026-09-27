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
    auto native_curve = [](const Bytes &b, int type = 1, bool project = true) {
        const std::size_t header = project ? 36 : 32;
        Bytes entry(header);
        write(entry, 4, std::int32_t(type));
        write(entry, header - 8, std::uint64_t(b.size()));
        entry.insert(entry.end(), b.begin(), b.end());
        return graphics_entry_native_input(entry).at(project ? "with_project" : "without_project");
    };
    auto surface = [](bool closed_u, bool closed_v, std::optional<int> weights,
                      std::optional<int> knots_u, std::optional<int> knots_v, int boundary = 0) {
        Packet p;
        const auto root = p.table({4, 8}, 12);
        p.reference(8, root);
        write(p.b, root + 4, std::uint8_t(14));
        const auto s = p.table({4, std::uint16_t(weights ? 8 : 0), std::uint16_t(knots_u ? 12 : 0),
                                std::uint16_t(knots_v ? 16 : 0), 20, 24, 28, 32, 36, 40, 44,
                                std::uint16_t(boundary ? 48 : 0), 52, 56},
                               64);
        p.reference(root + 8, s);
        write(p.b, s + 20, std::int32_t(2));
        write(p.b, s + 24, std::int32_t(3));
        write(p.b, s + 28, std::int32_t(2));
        write(p.b, s + 32, std::int32_t(2));
        write(p.b, s + 36, std::int32_t(-9));
        write(p.b, s + 40, std::int32_t(13));
        write(p.b, s + 44, std::int32_t(-7));
        write(p.b, s + 52, std::uint8_t(closed_u));
        write(p.b, s + 56, std::uint8_t(closed_v));
        auto values = [&](std::size_t slot, int count) {
            p.align();
            put(p.b, std::uint32_t(0));
            const auto at = p.b.size();
            put(p.b, std::int32_t(count));
            for (int i = 0; i < count; ++i)
                put(p.b, double(i));
            p.reference(slot, at);
        };
        values(s + 4, 18);
        if (weights)
            values(s + 8, *weights);
        if (knots_u)
            values(s + 12, *knots_u);
        if (knots_v)
            values(s + 16, *knots_v);
        if (boundary == 2)
            p.reference(s + 48, p.group({{0, 0, 0}, {1, 0, 0}, {0, 0, 0}}, 2));
        else if (boundary) {
            const auto group = p.table({4, std::uint16_t(boundary == 5 ? 0 : 8)}, 12);
            p.reference(s + 48, group);
            write(p.b, group + 4, std::int32_t(2));
            p.align();
            const auto array = p.b.size();
            put(p.b, std::uint32_t(boundary == 3 || boundary == 4));
            p.reference(group + 8, array);
            if (boundary == 3 || boundary == 4) {
                put(p.b, std::uint32_t(0));
                const auto member = p.table({4, 8}, 12);
                p.reference(array + 4, member);
                if (boundary == 4) {
                    write(p.b, member + 4, std::uint8_t(5));
                    const auto nested = p.table({4, 8}, 12);
                    p.reference(member + 8, nested);
                    p.align();
                    const auto empty = p.b.size();
                    put(p.b, std::uint32_t(0));
                    p.reference(nested + 8, empty);
                }
            }
        }
        return p.b;
    };
    auto surface_table = [](const Bytes &b) {
        const auto root = 8 + Reader(b, 8).u32();
        return root + 8 + Reader(b, root + 8).u32();
    };
    for (bool closed_u : {false, true})
        for (bool closed_v : {false, true})
            for (const auto weights : {std::optional<int>{}, std::optional<int>{0},
                                       std::optional<int>{-1}, std::optional<int>{6}})
                for (unsigned knots = 0; knots < 4; ++knots) {
                    const auto binary =
                        surface(closed_u, closed_v, weights,
                                knots & 1 ? std::optional<int>(closed_u ? 5 : 4) : std::nullopt,
                                knots & 2 ? std::optional<int>(closed_v ? 6 : 5) : std::nullopt);
                    for (bool project : {false, true}) {
                        const auto result = native_curve(binary, 5, project);
                        check(result.at("status") == "geometry_constructed" &&
                                  result.at("entry_restore").at("status") == "retained",
                              "surface factory and Entry retain a bounded matching control net");
                        const auto &c = result.at("construction");
                        check(c.at("pole_count") == 6 && c.at("pole_order") == "u_fastest" &&
                                  c.at("rational") == (weights && *weights > 0) &&
                                  c.at("input_poles_already_weighted") == true,
                              "surface keeps source weighted poles and independent directions");
                        check(c.at("u").at("effective_knot_count") == (closed_u ? 5 : 4) &&
                                  c.at("v").at("effective_knot_count") == (closed_v ? 6 : 5) &&
                                  c.at("u").at("knots_source") ==
                                      (knots & 1 ? "copied" : "generated_uniform") &&
                                  c.at("v").at("knots_source") ==
                                      (knots & 2 ? "copied" : "generated_uniform"),
                              "each surface direction chooses its own copied or default knots");
                        check(c.at("u").at("num_rules") == -9 && c.at("v").at("num_rules") == 13 &&
                                  c.at("source_hole_origin") == -7 && c.at("hole_origin") == true &&
                                  c.at("outer_boundary_active") == false &&
                                  c.at("boundary_sampling_performed") == false,
                              "reader overrides rules and normalizes only the hole origin flag");
                        check(result.at("parametric_append_input").at("status") == "not_evaluated",
                              "surface input construction does not imply downstream copy support");
                    }
                }
    const auto valid_surface = surface(false, false, {}, {}, {});
    const auto decoded_surface = decode_bgfb(valid_surface).at("geometry");
    check(decoded_surface.at("_type") == "BsplineSurface" && decoded_surface.at("numPolesU") == 2 &&
              decoded_surface.at("numPolesV") == 3 && decoded_surface.at("poles").size() == 18,
          "surface constructor fixture also reaches the independent public BGFB field decoder");
    auto changed_surface = valid_surface;
    auto s = surface_table(changed_surface);
    auto poles = s + 4 + Reader(changed_surface, s + 4).u32();
    write(changed_surface, poles, std::uint32_t(20)); // only complete triples are read
    write(changed_surface, changed_surface.size() - 8, UINT64_C(0x7ff8000000000001));
    check(native_curve(changed_surface, 5).at("construction").at("ignored_tail_scalars") == 2,
          "surface input ignores partial triple and copies nonfinite bits without evaluation");
    changed_surface = valid_surface;
    write(changed_surface, s + 44, std::int32_t(0));
    check(native_curve(changed_surface, 5).at("construction").at("outer_boundary_active") == true,
          "surface source hole origin zero activates the default exterior boundary");
    for (unsigned slot : {4u, 28u, 32u, 36u, 40u, 44u, 52u, 56u}) {
        changed_surface = valid_surface;
        const auto vt = s - Reader(changed_surface, s).i32();
        write(changed_surface, vt + 4 + 2 * (slot / 4 - 1), std::uint16_t(0));
        const auto result = native_curve(changed_surface, 5);
        check(result.at("status") == (slot <= 32 ? "rejected" : "geometry_constructed"),
              "omitted surface vectors and scalars follow individual reader defaults");
        if (slot == 36 || slot == 40)
            check(result.at("construction").at(slot == 36 ? "u" : "v").at("num_rules") == 0,
                  "missing rule field overwrites allocator's initial rule count with zero");
    }
    for (const auto slot : {20u, 24u, 28u, 32u}) {
        changed_surface = valid_surface;
        write(changed_surface, s + slot, std::int32_t(1));
        const auto result = native_curve(changed_surface, 5);
        check(result.at("status") == "rejected" && result.at("material_footer") == "not_read" &&
                  result.at("construction").at("native_populate_result") == 1 &&
                  result.at("entry_restore").at("status") == "rejected",
              "surface dimension and order guard returns null and rejects the Entry");
    }
    for (auto n : {4, 5, 7}) {
        const auto result = native_curve(surface(false, false, n, {}, {}), 5);
        check(result.at("status") == "rejected" &&
                  result.at("construction").at("reason") == "native_surface_weight_count_guard",
              "surface requires exact nonempty weight count, unlike the basic curve reader");
    }
    for (unsigned direction : {0u, 1u}) {
        const auto result =
            native_curve(surface(false, false, {}, direction ? std::nullopt : std::optional<int>(3),
                                 direction ? std::optional<int>(4) : std::nullopt),
                         5);
        check(result.at("status") == "rejected" &&
                  result.at("construction").at("reason") == "native_surface_knot_count_guard",
              "surface independently checks explicit node count in both directions");
    }
    check(native_curve(surface(true, true, {}, -1, -1), 5).at("status") == "geometry_constructed",
          "surface reader skips negative signed knot counts before the factory sees vectors");
    changed_surface = valid_surface;
    write(changed_surface, s + 20, std::int32_t(3));
    check(native_curve(changed_surface, 5).at("construction").at("reason") ==
              "native_surface_control_net_size_guard",
          "surface dimensions must multiply to the copied number of complete poles");
    changed_surface = valid_surface;
    write(changed_surface, s + 20, std::int32_t(65536));
    write(changed_surface, s + 24, std::int32_t(65536));
    check(native_curve(changed_surface, 5).at("reason") == "native_surface_pole_product_overflow",
          "native int32 dimension product wrap does not establish safe construction");
    for (int boundary : {1, 3}) {
        const auto result = native_curve(surface(false, false, {}, {}, {}, boundary), 5);
        check(result.at("status") == "geometry_constructed" &&
                  result.at("construction").at("trim_operation") == "clear_empty_root",
              "setTrim skips sampling when the constructed root has no retained members");
    }
    for (int boundary : {2, 4}) {
        const auto result = native_curve(surface(false, false, {}, {}, {}, boundary), 5);
        check(result.at("status") == "not_evaluated" &&
                  result.at("reason") == "native_surface_nonempty_trim_construction_not_evaluated",
              "nonempty trim and nested empty group cannot bypass native trim processing");
    }
    changed_surface = surface(false, false, {}, {}, {}, 2);
    write(changed_surface, surface_table(changed_surface) + 28, std::int32_t(1));
    check(native_curve(changed_surface, 5).at("status") == "rejected",
          "failed surface factory never calls setTrim on an already read nonempty boundary");
    changed_surface = surface(false, false, {}, {}, {}, 5);
    write(changed_surface, surface_table(changed_surface) + 28, std::int32_t(1));
    check(native_curve(changed_surface, 5).at("status") == "not_evaluated",
          "boundary input failure precedes and cannot be hidden by surface factory guard");
    changed_surface = valid_surface;
    write(changed_surface, s + 28, std::int32_t(1));
    changed_surface.pop_back();
    check(native_curve(changed_surface, 5).at("status") == "not_evaluated",
          "unavailable pole bytes precede an otherwise rejecting populate guard");
    for (unsigned slot : {8u, 12u, 16u}) {
        changed_surface = surface(false, false, 6, 4, 5);
        const auto table = surface_table(changed_surface);
        const auto vector = table + slot + Reader(changed_surface, table + slot).u32();
        write(changed_surface, vector, std::int32_t(10000));
        write(changed_surface, table + 28, std::int32_t(1));
        check(native_curve(changed_surface, 5).at("status") == "not_evaluated",
              "every scalar vector must be readable before a rejecting surface factory guard");
    }
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
                const auto &append = result.at("parametric_append_input");
                check(
                    result.at("entry_restore").at("status") == "retained" &&
                        append.at("status") == "appended" &&
                        append.at("geometry_operation") == "copy_bspline_storage" &&
                        append.at("source_geometry_reused") == false &&
                        append.at("bspline_storage_copy").at("knots_regenerated") == false,
                    "B-spline wrapper clone copies constructed storage without regenerating knots");
                for (bool project : {false, true}) {
                    const auto raw = native_curve(b, 4, project);
                    check(raw.at("status") == "geometry_constructed" &&
                              raw.at("bspline_pointer_extraction").at("stored_object_reused") ==
                                  true &&
                              raw.at("bspline_pointer_extraction").at("curve_data_copied") ==
                                  false &&
                              raw.at("bspline_pointer_extraction").at("native_result") == 0,
                          "raw B-spline entry retains the saved curve for either project header");
                    check(raw.at("entry_restore").at("status") == "retained" &&
                              raw.at("parametric_append_input").at("bspline_storage_copy") ==
                                  append.at("bspline_storage_copy") &&
                              raw.at("parametric_append_input").at("source_geometry_reused") ==
                                  false,
                          "raw B-spline append creates new arrays after reference extraction");
                }
            }
    auto collection = [](const Bytes &member, unsigned repetitions) {
        Packet p;
        const auto root = p.table({4, 8}, 12);
        p.reference(8, root);
        write(p.b, root + 4, std::uint8_t(5));
        const auto group = p.table({4, 8}, 12);
        p.reference(root + 8, group);
        write(p.b, group + 4, std::int32_t(2));
        p.align();
        const auto array = p.b.size();
        put(p.b, std::uint32_t(repetitions));
        for (unsigned i = 0; i < repetitions; ++i)
            put(p.b, std::uint32_t(0));
        p.reference(group + 8, array);
        p.align();
        const auto source = p.b.size();
        p.b.insert(p.b.end(), member.begin(), member.end());
        const auto variant = source + 8 + Reader(member, 8).u32();
        for (unsigned i = 0; i < repetitions; ++i)
            p.reference(array + 4 + 4 * i, variant);
        return p.b;
    };
    const auto grouped = native_curve(collection(spline(2, false, 2, {}), 2), 2);
    check(grouped.at("construction").at("output_member_count") == 2 &&
              grouped.at("parametric_append_input").at("status") == "appended" &&
              grouped.at("parametric_append_input").at("source_geometry_reused") == false,
          "repeated binary B-spline members survive native group construction and copying");
    const auto nested_group = native_curve(collection(collection(spline(2, true, 2, 5), 2), 1), 2);
    check(nested_group.at("construction").at("members").at(0).at("output_member_count") == 2 &&
              nested_group.at("parametric_append_input").at("status") == "appended",
          "nested B-spline group copy preserves grouping and ordered members");
    const auto surface_group = native_curve(collection(valid_surface, 1), 2);
    check(surface_group.at("construction").at("output_member_count") == 0 &&
              surface_group.at("construction").at("members").at(0).at("action") == "skip_non_curve",
          "generic surface member construction does not insert a surface in a curve group");
    changed_surface = valid_surface;
    write(changed_surface, surface_table(changed_surface) + 28, std::int32_t(1));
    const auto failed_surface_group = native_curve(collection(changed_surface, 1), 2);
    check(failed_surface_group.at("construction").at("output_member_count") == 0 &&
              failed_surface_group.at("construction").at("members").at(0).at("action") ==
                  "skip_null",
          "surface factory null is skipped by the enclosing generic curve-group reader");
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
    check(native_curve(truncated_spline, 4).at("status") == "not_evaluated" &&
              native_curve(spline(1, false, {}, {}), 4).at("status") == "not_evaluated",
          "raw B-spline extraction cannot turn an unproved source constructor into a null result");
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
