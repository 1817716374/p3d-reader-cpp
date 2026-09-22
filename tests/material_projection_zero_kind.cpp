#include "internal.hpp"

unsigned material_projection_zero_kind_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    const Matrix3 identity{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    const Matrix4 affine{{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
    const Matrix2x3 uv{{{1, 0, .25}, {0, 1, -.75}}};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    Json resolved = {{"status", "resolved"},
                     {"mapping_mode", 3},
                     {"preparation", {{"scale_mode", 0}}},
                     {"matrix", Matrix3{{{1, 2, 0}, {0, 1, 3}, {2, 0, 1}}}},
                     {"origin", Point3{4, 6, 8}},
                     {"reference_dimensions", Point3{2, 4, 8}},
                     {"layer_data_flag_bit_2", 0}};
    MaterialProjectionRenderContext zero;
    zero.geometry_kind = 0;
    zero.reference_point = Point3{3, 4, 5};
    zero.uv_transform = uv;
    zero.geometry_scale = 2;
    zero.vertex_linear_transform = Matrix3{{{0, -2, 0}, {2, 0, 0}, {0, 0, 2}}};
    zero.vertex_translation = Point3{10, 20, 30};
    auto state = prepare_material_projection_sampling(resolved, zero);
    check(state.at("status") == "prepared" && state.at("geometry_kind") == 0 &&
              !state.contains("render_origin"),
          "zero-kind preparation has its own context and origin provenance");
    check(state.at("origin") == Point3{6, 22, 36},
          "zero kind transforms origin minus reference point before adding translation");
    check(state.at("matrix") == Matrix3{{{-4, 2, 0}, {-2, 0, 6}, {0, 4, 2}}},
          "projection rows multiply the transpose of the vertex transform");
    check(state.at("inverse_reference_dimensions") == Point3{.125, .0625, .03125} &&
              state.at("uv_transform") == uv && state.at("uv_center") == Point2{.5, .5},
          "relative mode scales all inverse dimensions, not UV coefficients");
    auto sampled = sample_material_projection(state, {2, 24, 42});
    check(sampled.at("status") == "computed" &&
              sampled.at("projection_point") == Point3{20, 44, 20} &&
              sampled.at("uv") == Point2{3.25, 2.5},
          "noncommuting frame and projection matrices produce the analytic directional result");
    auto ignored = zero;
    ignored.reference_transform = affine;
    (*ignored.reference_transform)[0][0] = nan;
    check(prepare_material_projection_sampling(resolved, ignored) == state,
          "zero kind never reads the nonzero-kind reference affine");

    // A uniform scale and quarter turn represent the same geometry in another
    // render frame. Every existing projection branch must retain the same UV.
    MaterialProjectionRenderContext direct;
    direct.geometry_kind = 1;
    direct.reference_point = zero.reference_point;
    direct.reference_transform = affine;
    direct.uv_transform = uv;
    direct.geometry_scale = 2;
    const std::array<Point3, 7> points{
        {{1, 2, 3}, {-3, 1, 2}, {2, -3, 1}, {0, 0, 0}, {0, 0, 3}, {0, 0, -3}, {3, 0, 0}}};
    const std::array<Point3, 6> normals{
        {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}};
    for (int mode : {3, 4, 5, 6, 7}) {
        for (int scale_mode : {0, 1}) {
            for (int cap : {0, 1}) {
                if (mode != 6 && cap != 0)
                    continue;
                auto input = resolved;
                input["mapping_mode"] = mode;
                input["preparation"]["scale_mode"] = scale_mode;
                input["layer_data_flag_bit_2"] = cap;
                const auto local = prepare_material_projection_sampling(input, zero);
                const auto world = prepare_material_projection_sampling(input, direct);
                check(local.at("status") == "prepared" && world.at("status") == "prepared",
                      "both geometry kinds prepare for each mode, scale and cap selection");
                const auto expected_inverse = scale_mode == 0          ? Point3{.125, .0625, .03125}
                                              : mode == 5 || mode == 6 ? Point3{.125, .25, .03125}
                                                                       : Point3{.5, .25, .125};
                check(
                    local.at("inverse_reference_dimensions") == expected_inverse,
                    "absolute curved modes scale inverse X and Z only; box and directional do not");
                check(local.at("uv_transform") ==
                              (scale_mode == 0 ? uv : Matrix2x3{{{.5, 0, .25}, {0, .5, -.75}}}) &&
                          local.at("uv_center") ==
                              (scale_mode == 0 ? Point2{.5, .5} : Point2{1, 1}),
                      "absolute UV rows divide by geometry scale and centers follow their lengths");
                for (const auto &q : points) {
                    const Point3 p{q[0] + 1, q[1] + 2, q[2] + 3};
                    const Point3 transformed{-2 * p[1] + 10, 2 * p[0] + 20, 2 * p[2] + 30};
                    for (const auto &n : normals) {
                        const Point3 rotated_normal{-n[1], n[0], n[2]};
                        const auto a = sample_material_projection(world, p, n);
                        const auto b =
                            sample_material_projection(local, transformed, rotated_normal);
                        check(a.at("status") == "computed" && b.at("status") == "computed" &&
                                  a.at("branch") == b.at("branch") && a.at("uv") == b.at("uv"),
                              "UV and selected face are invariant under the known render frame "
                              "change");
                    }
                }
            }
        }
    }
    auto input = resolved;
    input["reference_dimensions"] = Point3{0, -4, 0};
    check(prepare_material_projection_sampling(input, zero).at("inverse_reference_dimensions") ==
              Point3{.25, -.0625, .25},
          "exact zero dimensions become one before scale adjustment; negative dimensions retain "
          "sign");
    auto negative = zero;
    negative.geometry_scale = -2;
    check(prepare_material_projection_sampling(resolved, negative)
                  .at("inverse_reference_dimensions") == state.at("inverse_reference_dimensions"),
          "relative scale adjustment uses the square even for negative scale");
    input = resolved;
    input["preparation"]["scale_mode"] = -1;
    auto negative_state = prepare_material_projection_sampling(input, negative);
    check(negative_state.at("uv_transform") == Matrix2x3{{{-.5, 0, .25}, {0, -.5, -.75}}} &&
              negative_state.at("uv_center") == Point2{1, 1},
          "absolute linear UV coefficients retain scale sign but row lengths do not");

    auto rounding = zero;
    rounding.geometry_scale = 3;
    const double ninth = 0.111111111938953399658203125;
    auto r = prepare_material_projection_sampling(resolved, rounding);
    check(r.at("inverse_squared_scale") == ninth &&
              r.at("inverse_reference_dimensions") == Point3{ninth * .5, ninth * .25, ninth * .125},
          "reciprocal of the squared geometry scale is computed in float, not double");
    rounding.geometry_scale = 1 + std::ldexp(1., -24);
    rounding.vertex_translation = Point3{16777217, 0, 0};
    rounding.vertex_linear_transform =
        Matrix3{{{1 + std::ldexp(1., -24), 1, 1}, {0, 1, 0}, {0, 0, 1}}};
    input = resolved;
    input["matrix"] = identity;
    input["origin"] = Point3{1, 16777216, -16777216};
    rounding.reference_point = Point3{};
    r = prepare_material_projection_sampling(input, rounding);
    check(r.at("inverse_squared_scale") == 1 && r.at("vertex_translation")[0] == 16777216 &&
              r.at("origin") == Point3{16777217, 16777216, -16777216},
          "frame inputs are float but origin products, cancellation and addition stay double");
    check(r.at("matrix") == Matrix3{{{1, 0, 0}, {1, 1, 0}, {1, 0, 1}}},
          "frame coefficient float rounding precedes matrix multiplication");

    auto rejected = [&](const Json &source, const MaterialProjectionRenderContext &c) {
        const auto result = prepare_material_projection_sampling(source, c);
        check(result.at("status") == "not_evaluated" && !result.contains("uv_transform"),
              "incomplete or unusable zero-kind context cannot publish a prepared transform");
    };
    auto bad = zero;
    bad.vertex_linear_transform.reset();
    rejected(resolved, bad);
    bad = zero;
    bad.vertex_translation.reset();
    rejected(resolved, bad);
    bad = zero;
    bad.geometry_scale.reset();
    rejected(resolved, bad);
    for (double scale : {0., 1e-30, 1e30, nan}) {
        bad = zero;
        bad.geometry_scale = scale;
        rejected(resolved, bad);
    }
    bad = zero;
    (*bad.vertex_linear_transform)[1][2] = nan;
    rejected(resolved, bad);
    bad = zero;
    (*bad.vertex_translation)[0] = 1e40;
    rejected(resolved, bad);
    bad = zero;
    bad.geometry_scale = 1e30;
    input = resolved;
    input["preparation"]["scale_mode"] = 1;
    check(prepare_material_projection_sampling(input, bad).at("status") == "prepared",
          "absolute directional mapping does not compute an unused overflowing scale square");
    input["mapping_mode"] = 5;
    rejected(input, bad);
    bad = zero;
    bad.uv_transform = Matrix2x3{{{0, 0, 1}, {0, 1, 0}}};
    rejected(input, bad);
    auto unused = direct;
    unused.vertex_linear_transform = *zero.vertex_linear_transform;
    (*unused.vertex_linear_transform)[0][0] = nan;
    unused.vertex_translation = Point3{nan, nan, nan};
    check(prepare_material_projection_sampling(resolved, unused) ==
              prepare_material_projection_sampling(resolved, direct),
          "nonzero geometry kind ignores zero-kind-only fields");

    // Exercise the public preparation and matrix resolution pipeline as well.
    MaterialProjectionContext projection;
    projection.mapping_mode = 3;
    projection.reference_point = Point3{4, 6, 8};
    projection.reference_dimensions = Point3{2, 4, 8};
    projection.reference_matrix = identity;
    const Json getter = {{"parameters",
                          {{"pattern_proj_offset", {{"value", Point3{0, 0, 0}}}},
                           {"pattern_proj_scale", {{"value", Point3{1, 1, 1}}}},
                           {"pattern_proj_angles", {{"value", Point3{0, 0, 0}}}},
                           {"origin_uv_pro_matrix_on", {{"value", false}}}}}};
    const auto transform = resolve_material_projection_transform(getter, projection);
    check(transform.at("status") == "resolved", "source projection getter resolves normally");
    const auto integrated = prepare_material_projection_sampling(transform, zero);
    const auto sample = sample_material_projection(integrated, {2, 24, 42});
    check(sample.at("status") == "computed" && sample.at("uv") == Point2{1.25, .25},
          "resolved source projection connects to zero-kind preparation and vertex sampling");
    return checks;
}
