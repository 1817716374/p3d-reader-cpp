#include "internal.hpp"

unsigned material_projection_transform_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto near = [](double a, double b) {
        return std::abs(a - b) <= 2e-13 * std::max({1., std::abs(a), std::abs(b)});
    };
    const Matrix3 identity{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    auto verify_inverse = [&](const Matrix3 &m, const char *method) {
        const auto inverse = native_matrix_inverse(m);
        check(inverse.inverted && inverse.method == method, "native inverse branch selection");
        for (unsigned i = 0; i < 3; ++i)
            for (unsigned j = 0; j < 3; ++j) {
                double value = 0;
                for (unsigned k = 0; k < 3; ++k)
                    value += m[i][k] * inverse.matrix[k][j];
                check(near(value, identity[i][j]), "native inverse product is identity");
            }
        return inverse;
    };
    verify_inverse(Matrix3{{{2, 1, 0}, {0, 3, 2}, {1, 0, 4}}}, "scaled_cofactors");
    // Unequal column lengths trigger the fallback; a nontrivial right factor
    // makes this sensitive to accidentally transposing V or the output.
    const auto small =
        verify_inverse(Matrix3{{{6e-11, -8e-11, 0}, {.8, .6, 0}, {0, 0, 1}}}, "orthogonal_factors");
    check(near(small.matrix[0][0], 6e9) && near(small.matrix[1][0], -8e9),
          "orthogonal factors retain inverse magnitude and row convention");
    const auto singular = native_matrix_inverse(Matrix3{{{1, 0, 0}, {0, 1, 0}, {0, 0, 0}}});
    check(!singular.inverted && singular.matrix == identity &&
              singular.method == "rank_deficient_identity",
          "native rank failure writes identity");
    const auto zero = native_matrix_inverse(Matrix3{});
    check(!zero.inverted && zero.matrix == identity && zero.method == "zero_matrix_identity",
          "native zero matrix also writes identity");
    check(!native_matrix_inverse(Matrix3{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1e-14}}}).inverted &&
              native_matrix_inverse(Matrix3{{{1, 0, 0}, {0, 1, 0}, {0, 0, 2e-13}}}).inverted,
          "native relative rank threshold is preserved rather than accepting every nonzero "
          "determinant");
    check(native_matrix_inverse(Matrix3{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1e-8}}}).method ==
              "orthogonal_factors",
          "determinant threshold equality uses orthogonal factors");
    for (const double magnitude : {1e100, 1e-100}) {
        const auto inv = native_matrix_inverse(
            Matrix3{{{magnitude, 0, 0}, {0, magnitude, 0}, {0, 0, magnitude}}});
        check(inv.inverted && inv.method == "scaled_cofactors" &&
                  std::abs(inv.matrix[0][0] * magnitude - 1) < 1e-14 &&
                  inv.matrix[0][0] == inv.matrix[1][1] && inv.matrix[1][1] == inv.matrix[2][2],
              "normalization avoids a naive overflowing or underflowing determinant");
    }
    MaterialProjectionContext context;
    context.mapping_mode = 4;
    context.reference_point = Point3{10, 20, 30};
    context.reference_dimensions = Point3{2, 3, 4};
    context.reference_matrix = Matrix3{{{1, 2, 0}, {0, 1, 0}, {0, 0, 1}}};
    Json getter = {{"object_id", 4},
                   {"frame_source_object_id", 8},
                   {"parameters",
                    {{"pattern_proj_offset", {{"value", Point3{1, 0, 0}}}},
                     {"pattern_proj_scale", {{"value", Point3{2, 4, 8}}}},
                     {"pattern_proj_angles", {{"value", Point3{0, 0, 0}}}},
                     {"origin_uv_pro_matrix_on", {{"value", false}}}}},
                   {"matrix", {{"storage_values", nullptr}}}};
    const auto source = getter;
    auto result = resolve_material_projection_transform(getter, context);
    check(result["status"] == "resolved" && result["matrix_source"] == "computed_transform" &&
              result["matrix"] == Json(Matrix3{{{.5, -1, 0}, {0, .25, 0}, {0, 0, .125}}}),
          "projection inverse is row-scaled, not column-scaled");
    check(result["origin"] == Json(Point3{12, 20, 30}) &&
              result["reference_dimensions"] == Json(Point3{2, 3, 4}) &&
              result["source_object_id"] == 4 && result["frame_source_object_id"] == 8 &&
              getter == source && result["point_mapping"] == "not_evaluated",
          "transform keeps origin dimensions provenance and source without claiming final UV");
    getter["parameters"]["pattern_proj_scale"]["value"] = Point3{0, -0., -2};
    context.reference_matrix = Matrix3{};
    result = resolve_material_projection_transform(getter, context);
    check(result["status"] == "resolved" &&
              result["matrix"] == Json(Matrix3{{{1, 0, 0}, {0, 1, 0}, {0, 0, -.5}}}) &&
              result["computed_transform"]["inverse_succeeded"] == false &&
              result["computed_transform"]["normalized_projection_scale"] == Json(Point3{1, 1, -2}),
          "caller ignores inverse failure and continues row scaling, including negative and zero "
          "scales");
    getter["parameters"]["origin_uv_pro_matrix_on"]["value"] = true;
    getter["matrix"]["storage_values"] = Json::array({2., 3., 4., 5., 6., 7., 8., 9., 10.});
    result = resolve_material_projection_transform(getter, context);
    check(result["matrix_source"] == "local_explicit_matrix" &&
              result["matrix"] == Json(Matrix3{{{2, 3, 4}, {5, 6, 7}, {8, 9, 10}}}) &&
              result["computed_transform"]["inverse_succeeded"] == false,
          "local explicit matrix overrides computed result without inversion scaling or transpose");
    getter["matrix"]["storage_values"] = Json::array({0, 0, 0, 0, 0, 0, 0, 0, 0});
    check(resolve_material_projection_transform(getter, context)["matrix"] == Json(Matrix3{}),
          "explicit zero matrix is not repaired");
    getter["matrix"]["storage_values"] = nullptr;
    result = resolve_material_projection_transform(getter, context);
    check(result["status"] == "not_evaluated" && !result.contains("matrix") &&
              result.contains("computed_transform"),
          "missing enabled override is not replaced with computed transform");
    getter["parameters"]["origin_uv_pro_matrix_on"]["value"] = nullptr;
    check(resolve_material_projection_transform(getter, context)["reason"] ==
              "unavailable_explicit_matrix_switch",
          "unknown switch remains unresolved");
    getter["parameters"]["origin_uv_pro_matrix_on"]["value"] = true;
    getter["matrix"]["storage_values"] =
        Json::array({0, 0, 0, 0, 0, 0, 0, 0, Json{{"floating_point", "nan"}}});
    check(resolve_material_projection_transform(getter, context)["reason"] ==
              "unavailable_or_nonfinite_explicit_matrix",
          "tagged NaN matrix cannot be turned into finite numbers");
    getter["parameters"]["origin_uv_pro_matrix_on"]["value"] = false;
    check(resolve_material_projection_transform(getter, context)["status"] == "resolved",
          "disabled nonfinite override does not obscure computed matrix");
    context.reference_point.reset();
    check(resolve_material_projection_transform(getter, context)["reason"] ==
              "projection_preparation_unavailable",
          "context must initialize before matrix selection");
    context.reference_point = Point3{};
    context.reference_matrix = identity;
    getter["parameters"]["pattern_proj_scale"]["value"] =
        Point3{std::numeric_limits<double>::denorm_min(), 1, 1};
    check(resolve_material_projection_transform(getter, context)["reason"] ==
              "nonfinite_scaled_transform",
          "reciprocal scale overflow cannot publish null JSON as a complete transform");
    getter["parameters"]["pattern_proj_scale"]["value"] = Point3{1, 1, 1};
    context.reference_matrix = Matrix3{{{1e-310, 0, 0}, {0, 1e-310, 0}, {0, 0, 1e-310}}};
    check(resolve_material_projection_transform(getter, context)["reason"] ==
              "nonfinite_inverse_arithmetic",
          "inverse normalization overflow remains explicit");
    return checks;
}
