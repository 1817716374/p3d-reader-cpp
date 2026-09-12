#include "internal.hpp"

unsigned material_projection_sampling_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    const Matrix3 identity{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    const Matrix4 affine{{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
    MaterialProjectionContext projection;
    projection.mapping_mode = 3;
    projection.reference_point = Point3{};
    projection.reference_dimensions = Point3{2, 4, 8};
    projection.reference_matrix = identity;
    projection.absolute_unit_factor = 2.;
    const Json getter = {{"parameters",
                          {{"pattern_proj_offset", {{"value", Point3{0, 0, 0}}}},
                           {"pattern_proj_scale", {{"value", Point3{1, 1, 1}}}},
                           {"pattern_proj_angles", {{"value", Point3{0, 0, 0}}}},
                           {"origin_uv_pro_matrix_on", {{"value", false}}}}}};
    MaterialProjectionRenderContext render;
    render.geometry_kind = 1;
    render.reference_point = Point3{};
    render.reference_transform = affine;
    render.uv_transform = Matrix2x3{{{1, 0, 0}, {0, 1, 0}}};
    render.geometry_scale = 2.;
    auto prepare = [&]() {
        const auto resolved = resolve_material_projection_transform(getter, projection);
        check(resolved.at("status") == "resolved", "sampling starts with the actual transform API");
        return prepare_material_projection_sampling(resolved, render);
    };
    auto state = prepare();
    check(state.at("status") == "prepared" && state.at("projection_flags") == 8 &&
              state.at("inverse_reference_dimensions") == Point3{.5, .25, .125},
          "directional drape preparation retains native dimensions and flags");
    auto expect_uv = [&](const Json &s, const Point3 &p, const std::optional<Point3> &n,
                         const Point2 &expected, const char *branch) {
        const auto value = sample_material_projection(s, p, n);
        check(value.at("status") == "computed", "projection sample computed");
        check(value.at("branch") == branch, "native projection branch");
        const auto uv = value.at("uv").get<Point2>();
        for (unsigned i = 0; i < 2; ++i)
            check(std::abs(uv[i] - expected[i]) <= 4e-7 * std::max(1., std::abs(expected[i])),
                  "analytic projection coordinate");
        check(Json::parse(value.dump()).at("uv") == value.at("uv"),
              "finite projection output survives JSON roundtrip");
    };
    expect_uv(state, {1, 2, 7}, std::nullopt, {1, 1}, "directional_drape");
    expect_uv(state, {-3, 6, 0}, std::nullopt, {-1, 2}, "directional_drape");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    expect_uv(state, {1, 2, 0}, Point3{nan, nan, nan}, {1, 1}, "directional_drape");
    for (int mode : {4, 7}) {
        projection.mapping_mode = mode;
        state = prepare();
        expect_uv(state, {1, 1, 1}, Point3{1, 0, 0}, {.75, .625}, "cubic_positive_x");
        expect_uv(state, {1, 1, 1}, Point3{-1, 0, 0}, {.25, .625}, "cubic_negative_x");
        expect_uv(state, {1, 1, 1}, Point3{0, 1, 0}, {0, .625}, "cubic_positive_y");
        expect_uv(state, {1, 1, 1}, Point3{0, -1, 0}, {1, .625}, "cubic_negative_y");
        expect_uv(state, {1, 1, 1}, Point3{0, 0, 1}, {1, .75}, "cubic_positive_z");
        expect_uv(state, {1, 1, 1}, Point3{0, 0, -1}, {0, .75}, "cubic_negative_z");
        expect_uv(state, {1, 1, 1}, Point3{1, 1, 1}, {.75, .625}, "cubic_positive_x");
        expect_uv(state, {1, 1, 1}, Point3{0, 1, 1}, {0, .625}, "cubic_positive_y");
        expect_uv(state, {1, 1, 1}, Point3{}, {.75, .625}, "cubic_positive_x");
        check(sample_material_projection(state, {1, 1, 1}).at("reason") ==
                  "missing_render_vertex_normal",
              "normal-dependent modes do not invent normals");
    }
    projection.mapping_mode = 5;
    state = prepare();
    expect_uv(state, {1, 0, 0}, std::nullopt, {0, .5}, "spherical");
    expect_uv(state, {0, 1, 0}, std::nullopt, {.25, .5}, "spherical");
    expect_uv(state, {-1, 0, 0}, std::nullopt, {.5, .5}, "spherical");
    expect_uv(state, {0, -1, 0}, std::nullopt, {.75, .5}, "spherical");
    expect_uv(state, {0, 0, 1}, std::nullopt, {0, 1}, "spherical");
    expect_uv(state, {0, 0, -1}, std::nullopt, {0, 0}, "spherical");
    expect_uv(state, {}, std::nullopt, {0, .5}, "spherical");
    // Angular grids are an independent inverse construction, not a copy of
    // the evaluator formulas. The radius must not change relative UVs.
    constexpr double pi = 3.14159265358979323846;
    for (int latitude = -8; latitude <= 8; ++latitude)
        for (int longitude = 1; longitude < 32; ++longitude) {
            const double a = longitude * (2 * pi / 32), b = latitude * (pi / 18);
            for (double radius : {.125, 31.})
                expect_uv(state,
                          {radius * std::cos(b) * std::cos(a), radius * std::cos(b) * std::sin(a),
                           radius * std::sin(b)},
                          std::nullopt, {longitude / 32., .5 + latitude / 18.}, "spherical");
        }
    projection.mapping_mode = 6;
    state = prepare();
    expect_uv(state, {0, 1, 2}, std::nullopt, {.25, .75}, "cylindrical_side");
    projection.layer_data_flags = 4;
    state = prepare();
    check(state.at("projection_flags") == 128, "source data bit 2 selects cylindrical caps");
    expect_uv(state, {1, 1, 2}, Point3{0, 0, 1}, {1, 1}, "cylindrical_cap");
    expect_uv(state, {1, 1, 2}, Point3{0, 0, -1}, {1, 1}, "cylindrical_cap");
    expect_uv(state, {1, 1, 2}, Point3{1, 0, 1}, {.125, .75}, "cylindrical_side");
    expect_uv(state, {1, 1, 2}, Point3{0, 1, 1}, {.125, .75}, "cylindrical_side");
    expect_uv(state, {1, 1, 2}, Point3{}, {.125, .75}, "cylindrical_side");
    // Nonuniform matrix scales affect the normal selector using M, not M^-T.
    auto resolved = resolve_material_projection_transform(getter, projection);
    resolved["matrix"] = Matrix3{{{4, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    auto changed = prepare_material_projection_sampling(resolved, render);
    expect_uv(changed, {0, 1, 2}, Point3{1, 0, 2}, {.25, .75}, "cylindrical_side");
    // Absolute mode uses the registered row lengths and applies geometry scale
    // only to the two linear coefficients of each UV row, not its translation.
    projection.scale_mode = -7;
    projection.mapping_mode = 3;
    render.uv_transform = Matrix2x3{{{3, 4, 11}, {-12, 5, -7}}};
    state = prepare();
    check(state.at("projection_flags") == 9 &&
              state.at("uv_transform") == Matrix2x3{{{6, 8, 11}, {-24, 10, -7}}} &&
              state.at("uv_center") == Point2{.05, .5 / 26},
          "absolute mode uses UV row norms and leaves translations unchanged");
    render.uv_transform = Matrix2x3{{{1, 0, 0}, {0, 1, 0}}};
    projection.mapping_mode = 5;
    state = prepare();
    expect_uv(state, {0, 1, 0}, std::nullopt, {pi / 2, pi / 2}, "spherical");
    projection.mapping_mode = 6;
    projection.layer_data_flags = 0;
    state = prepare();
    expect_uv(state, {0, 1, 2}, std::nullopt, {pi / 2, 2.5}, "cylindrical_side");
    projection.layer_data_flags = 4;
    state = prepare();
    expect_uv(state, {1, 1, 2}, Point3{0, 0, -1}, {1.5, 1.5}, "cylindrical_cap");
    // Native origin preparation rounds the affine linear part and translation
    // separately before adding them. A plain double affine product gives 1.
    projection.scale_mode = 0;
    projection.mapping_mode = 3;
    render.reference_point = Point3{16777217., 0, 0};
    auto t = affine;
    t[0][3] = -16777216.;
    render.reference_transform = t;
    state = prepare();
    check(state.at("render_origin") == Point3{}, "render origin preserves staged float rounding");
    render.reference_point = Point3{1, 2, 3};
    t = Matrix4{{{2, 1, 0, 10}, {0, 3, 2, -20}, {1, 0, 4, 30}, {0, 0, 0, 1}}};
    render.reference_transform = t;
    state = prepare();
    check(state.at("render_origin") == Point3{14, -8, 43} &&
              state.at("origin") == Point3{-14, 8, -43},
          "render affine axes, translation and origin subtraction");
    expect_uv(state, {-14, 8, -43}, std::nullopt, {.5, .5}, "directional_drape");
    render.reference_point = Point3{};
    render.reference_transform = affine;
    projection.reference_dimensions = Point3{0, -4, 8};
    resolved = resolve_material_projection_transform(getter, projection);
    resolved["reference_dimensions"] = Point3{0, -4, 8};
    state = prepare_material_projection_sampling(resolved, render);
    expect_uv(state, {1, 2, 0}, std::nullopt, {1.5, 0}, "directional_drape");
    resolved = resolve_material_projection_transform(getter, projection);
    resolved["origin"] = Point3{16777216., 0, 0};
    state = prepare_material_projection_sampling(resolved, render);
    expect_uv(state, {16777217., 0, 0}, std::nullopt, {.5, .5}, "directional_drape");
    const auto snapshot = state;
    for (const Point3 bad : {Point3{nan, 0, 0}, Point3{1e300, 0, 0}}) {
        const auto value = sample_material_projection(state, bad);
        check(value.at("status") == "not_evaluated" && !value.contains("uv"),
              "nonfinite and unrepresentable vertices have no fabricated coordinates");
    }
    check(snapshot == state, "sampling is read-only and does not mutate shared state");
    for (const char *key : {"matrix", "origin", "inverse_reference_dimensions", "uv_center",
                            "uv_transform", "projection_flags"}) {
        changed = state;
        changed.erase(key);
        check(sample_material_projection(changed, {}).at("status") == "not_evaluated",
              "missing prepared fields are reported");
        changed[key] = "invalid";
        check(sample_material_projection(changed, {}).at("status") == "not_evaluated",
              "ill-typed prepared fields are reported");
    }
    changed = state;
    changed["projection_flags"] = 24;
    check(sample_material_projection(changed, {}).at("reason") == "unsupported_projection_flags",
          "conflicting mapping flags do not select an arbitrary branch");
    auto incomplete = render;
    incomplete.geometry_kind.reset();
    check(prepare_material_projection_sampling(resolved, incomplete).at("status") ==
              "not_evaluated",
          "geometry kind is required");
    incomplete = render;
    incomplete.geometry_kind = 0;
    check(prepare_material_projection_sampling(resolved, incomplete).at("status") ==
              "not_evaluated",
          "unrecovered geometry kind zero branch is explicit");
    incomplete = render;
    incomplete.uv_transform.reset();
    check(prepare_material_projection_sampling(resolved, incomplete).at("status") ==
              "not_evaluated",
          "UV transform is not silently replaced by identity");
    incomplete = render;
    (*incomplete.reference_transform)[3][0] = 1;
    check(prepare_material_projection_sampling(resolved, incomplete).at("status") ==
              "not_evaluated",
          "perspective transforms are outside the affine render branch");
    resolved["preparation"]["scale_mode"] = 1;
    incomplete = render;
    incomplete.geometry_scale.reset();
    check(prepare_material_projection_sampling(resolved, incomplete).at("status") ==
              "not_evaluated",
          "absolute mapping requires the native geometry scale");
    incomplete.geometry_scale = 0.;
    check(prepare_material_projection_sampling(resolved, incomplete).at("status") ==
              "not_evaluated",
          "zero absolute UV row length is not repaired with an invented center");
    incomplete.geometry_scale = nan;
    check(prepare_material_projection_sampling(resolved, incomplete).at("status") ==
              "not_evaluated",
          "nonfinite absolute geometry scale is rejected");
    resolved["preparation"]["scale_mode"] = 0;
    check(prepare_material_projection_sampling(resolved, incomplete).at("status") == "prepared",
          "unused geometry scale does not block relative mapping");
    for (const Json invalid :
         {Json(), Json::array(), Json{{"status", false}}, Json{{"status", "prepared"}}})
        check(prepare_material_projection_sampling(invalid, render).at("status") == "not_evaluated",
              "unresolved or malformed transform input is explicit");
    return checks;
}
