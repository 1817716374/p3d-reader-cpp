#include "internal.hpp"
#include <p3d/material_planar.hpp>

unsigned material_planar_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    const Matrix3 identity{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    Json transform = {{"scope", "native_layer_uv_affine_transform"},
                      {"status", "computed"},
                      {"mapping_mode", 2},
                      {"scale_mode", 0},
                      {"matrix", Matrix2x3{{{1, 0, .1}, {0, 1, -.2}}}}};
    MaterialPlanarRenderContext zero;
    zero.enabled = true;
    zero.geometry_kind = 0;
    zero.vertex_linear_transform = identity;
    zero.origin_basis_transform = identity;
    zero.reference_translation = Point3{.25, .5, .75};
    auto state = prepare_material_planar_sampling(transform, zero);
    check(state.at("status") == "prepared" && state.at("origin") == Point3{.25, .5, .75} &&
              state.at("fallback_u_axis") == Point3{1, 0, 0} &&
              state.at("reference_axis") == Point3{0, 0, 1},
          "zero-kind planar state uses source columns");
    auto expect = [&](const Json &s, const Point3 &p, const std::optional<Point3> &n,
                      const std::optional<MaterialParametricFrame> &frame, Point2 expected) {
        const auto result = sample_material_planar(s, p, n, Point2{nan, nan}, frame);
        for (auto &v : expected)
            v = double(float(v));
        check(result.at("status") == "computed" && result.at("uv") == expected &&
                  result.at("next_state").at("status") == "prepared",
              "analytic planar UV and next state");
        return result;
    };
    auto sample = expect(state, {2, 3, 4}, Point3{0, 0, 1}, {}, {2.35, 3.3});
    check(sample.at("fallback_axis_used") == true && sample.at("basis_v") == Point3{0, 1, 0},
          "parallel normal chooses first-column fallback and reference-axis second cross product");
    expect(state, {2, 3, 4}, Point3{0, 0, -1}, {}, {2.35, 3.3});
    expect(state, {2, 3, 4}, Point3{0, 0, 0}, {}, {2.35, 3.3});
    sample = expect(state, {2, 3, 4}, Point3{1, 0, 0}, {}, {3.6, 4.55});
    check(sample.at("fallback_axis_used") == false && sample.at("basis_u") == Point3{0, 1, 0} &&
              sample.at("basis_v") == Point3{0, 0, 1},
          "nonparallel normal constructs the two axes from cross products");
    expect(state, {2, 3, 4}, Point3{0, 1, 0}, {}, {-1.15, 4.55});
    const MaterialParametricFrame face{{0, 0, 0}, {{{1, 0, 0}, {0, 1, 0}}}};
    const auto before = state.dump();
    expect(state, {2, 3, 4}, {}, face, {2.1, 2.8});
    auto advanced = sample.at("next_state");
    auto with_face = expect(advanced, {2, 3, 4}, {}, face, {2.6, 3.55});
    check(with_face.at("next_state") == advanced && state.dump() == before,
          "per-face query observes persistent offsets without mutating either input snapshot");
    auto reset_axis = expect(advanced, {2, 3, 4}, Point3{0, 0, 1}, {}, {2.35, 3.3});
    check(reset_axis.at("next_state").at("initial_offset") == Point2{.1, -.2},
          "dynamic offset always starts from saved registered offsets, not the previous dynamic "
          "offsets");
    auto absolute = transform;
    absolute["scale_mode"] = 1;
    auto abs_state = prepare_material_planar_sampling(absolute, zero);
    check(abs_state.at("projection_flags") == 5 &&
              abs_state.at("uv_transform") == state.at("uv_transform"),
          "enabled planar mapping applies no extra geometry-scale adjustment in absolute mode");
    expect(abs_state, {2, 3, 4}, Point3{1, 0, 0}, {}, {3.6, 4.55});

    MaterialPlanarRenderContext nonzero = zero;
    nonzero.geometry_kind = 1;
    nonzero.reference_linear_transform = identity;
    nonzero.vertex_translation = Point3{};
    nonzero.normal_transform = identity;
    nonzero.normalize_transformed_normal = false;
    const auto nz = prepare_material_planar_sampling(transform, nonzero);
    expect(nz, {2, 3, 4}, Point3{0, 0, -1}, {}, {2.35, -2.7});
    expect(nz, {2, 3, 4}, Point3{0, 0, 0}, {}, {2.35, .8});
    auto raw_normal = sample_material_planar(nz, {2, 3, 4}, Point3{1e-8, 0, 1e-8});
    nonzero.normalize_transformed_normal = true;
    auto normalized_normal = sample_material_planar(
        prepare_material_planar_sampling(transform, nonzero), {2, 3, 4}, Point3{1e-8, 0, 1e-8});
    check(raw_normal.at("fallback_axis_used") == true &&
              normalized_normal.at("fallback_axis_used") == false,
          "explicit transformed-normal normalization affects the degeneracy branch");
    const float boundary = 1e-6f;
    for (float x : {std::nextafter(boundary, 0.f), boundary, std::nextafter(boundary, 1.f)}) {
        const auto r = sample_material_planar(state, {2, 3, 4}, Point3{x, 0, 1});
        check(r.at("status") == "computed" && r.at("fallback_axis_used") == (x < boundary),
              "float cross-product squared threshold is strict; equality does not fall back");
    }
    auto changed = nonzero;
    changed.vertex_linear_transform = Matrix3{{{0, -2, 0}, {2, 0, 0}, {0, 0, 3}}};
    changed.vertex_translation = Point3{.25, .5, .75};
    changed.reference_linear_transform = Matrix3{{{2, 0, 0}, {0, 3, 0}, {0, 0, 4}}};
    changed.origin_basis_transform = Matrix3{{{1, 2, 0}, {0, 1, 3}, {2, 0, 1}}};
    auto changed_state = prepare_material_planar_sampling(transform, changed);
    check(
        changed_state.at("origin") == Point3{4.75, 13.25, 5.25} &&
            changed_state.at("fallback_u_axis") == Point3{0, 1, 0} &&
            changed_state.at("reference_axis") == Point3{0, 0, 3},
        "reference transform, origin basis and distinct column normalization follow native order");
    changed = nonzero;
    changed.vertex_linear_transform = Matrix3{{{2, 0, 0}, {0, 3, 0}, {0, 0, 4}}};
    auto scaled = prepare_material_planar_sampling(transform, changed);
    expect(scaled, {2, 3, 4}, Point3{0, 0, 1}, {}, {4.35, 9.3});
    changed.normal_transform = Matrix3{{{0, 0, 1}, {0, 1, 0}, {1, 0, 0}}};
    scaled = prepare_material_planar_sampling(transform, changed);
    expect(scaled, {2, 3, 4}, Point3{0, 0, 1}, {}, {9.6, 16.55});
    changed = nonzero;
    changed.vertex_translation = Point3{16777217, 0, 0};
    changed.reference_linear_transform =
        Matrix3{{{1 + std::ldexp(1., -24), 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    changed.reference_translation = Point3{.25, .5, .75};
    auto rounded_state = prepare_material_planar_sampling(transform, changed);
    check(rounded_state.at("origin")[0] == 16777216.25,
          "reference linear result rounds to float before adding double translation, which remains "
          "double");
    auto unused = zero;
    unused.normal_transform = Matrix3{{{nan, nan, nan}, {}, {}}};
    unused.vertex_translation = Point3{nan, nan, nan};
    unused.reference_linear_transform = *unused.normal_transform;
    check(prepare_material_planar_sampling(transform, unused) == state,
          "zero geometry kind does not read nonzero-only transforms");

    MaterialPlanarRenderContext disabled;
    disabled.enabled = false;
    auto suppressed = prepare_material_planar_sampling(transform, disabled);
    auto native = sample_material_planar(suppressed, {nan, nan, nan}, {}, Point2{2, 3});
    check(native.at("status") == "computed" && native.at("parametric_branch") == "native_uv" &&
              native.at("uv") == Point2{double(2.1f), double(2.8f)},
          "explicit planar suppression delegates to parametric UV without planar context");
    disabled.parametric.preparation_frame_present = false;
    disabled.parametric.use_parameter_factors = false;
    disabled.parametric.geometry_scale = 2;
    suppressed = prepare_material_planar_sampling(absolute, disabled);
    native = sample_material_planar(suppressed, {nan, nan, nan}, {}, Point2{2, 3});
    check(native.at("uv") == Point2{double(4.1f), double(5.8f)},
          "clearing planar flag retains absolute bit and the existing registered matrix");

    auto rejected = [&](const Json &t, const MaterialPlanarRenderContext &c) {
        const auto r = prepare_material_planar_sampling(t, c);
        check(r.at("status") == "not_evaluated",
              "unknown or invalid planar context stays unresolved");
    };
    rejected(transform, {});
    auto bad = zero;
    bad.geometry_kind.reset();
    rejected(transform, bad);
    bad = zero;
    bad.vertex_linear_transform.reset();
    rejected(transform, bad);
    bad = zero;
    bad.origin_basis_transform.reset();
    rejected(transform, bad);
    bad = zero;
    bad.reference_translation.reset();
    rejected(transform, bad);
    bad = nonzero;
    bad.normal_transform.reset();
    rejected(transform, bad);
    bad = nonzero;
    bad.normalize_transformed_normal.reset();
    rejected(transform, bad);
    bad = nonzero;
    bad.reference_linear_transform.reset();
    rejected(transform, bad);
    bad = nonzero;
    bad.vertex_translation.reset();
    rejected(transform, bad);
    bad = zero;
    (*bad.vertex_linear_transform)[0][0] = 1e40;
    rejected(transform, bad);
    bad = zero;
    (*bad.origin_basis_transform)[0][0] = nan;
    rejected(transform, bad);
    auto wrong = transform;
    wrong["mapping_mode"] = 1;
    rejected(wrong, zero);
    disabled.parametric = {};
    rejected(absolute, disabled);
    for (const char *key : {"scope", "status", "enabled", "mapping_mode", "projection_flags",
                            "uv_transform", "initial_offset"}) {
        auto invalid = state;
        invalid.erase(key);
        const auto r = sample_material_planar(invalid, {2, 3, 4}, Point3{0, 0, 1});
        check(r.at("status") == "not_evaluated" && !r.contains("uv") && !r.contains("next_state"),
              "invalid dynamic state cannot publish UV or advance the stream");
    }
    check(sample_material_planar(state, {1, 2, 3}).at("status") == "not_evaluated",
          "enabled planar mapping needs a normal unless a face frame is present");
    check(sample_material_planar(state, {1, 2, 3}, Point3{nan, 0, 0}).at("status") ==
              "not_evaluated",
          "invalid used normal does not fall back to source UV");
    expect(state, {2, 3, 4}, Point3{nan, nan, nan}, face, {2.1, 2.8});
    return checks;
}
