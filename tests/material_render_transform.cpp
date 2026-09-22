#include "internal.hpp"
#include <p3d/material_planar.hpp>
#include <p3d/material_render_transform.hpp>

unsigned material_render_transform_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    const Matrix3 identity{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    const Matrix3 zero{};
    const auto base = derive_material_render_transform(identity, {1.00000001, 2, 3});
    check(base.at("status") == "computed" && base.at("normal_transform") == identity &&
              base.at("normalize_transformed_normal") == false &&
              base.at("vertex_translation") == Point3{1, 2, 3},
          "identity normal state and single-precision translation");
    check(base.at("geometry_scale_candidates").at("all_three_rows") == 1 &&
              base.at("geometry_scale_candidates").at("first_two_rows") == 1 &&
              !base.contains("geometry_scale"),
          "scale candidates do not silently select an unknown host branch");

    const Matrix3 diagonal{{{2, 0, 0}, {0, 3, 0}, {0, 0, 4}}};
    const auto scaled = derive_material_render_transform(diagonal, {});
    check(scaled.at("normal_transform") ==
                  Matrix3{{{1, 0, 0}, {0, double(float(2. / 3)), 0}, {0, 0, .5}}} &&
              scaled.at("normalize_transformed_normal") == true,
          "nonuniform scale uses one common cofactor length, not per-row normalization");
    check(scaled.at("row_squared_lengths") == Point3{4, 9, 16} &&
              scaled.at("minimum_squared_scale") == 4 &&
              scaled.at("mean_squared_scale") == double(float(29.f / 3.f)) &&
              scaled.at("geometry_scale_candidates").at("first_two_rows") == 3 &&
              scaled.at("geometry_scale_candidates").at("all_three_rows") == 4,
          "two native scale candidates differ on the third row");

    const Matrix3 rotated{{{0, -4, 0}, {4, 0, 0}, {0, 0, 4}}};
    auto r = derive_material_render_transform(rotated, {});
    check(r.at("normal_transform") == Matrix3{{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}}} &&
              r.at("normalize_transformed_normal") == false,
          "positive uniform scale and proper rotation retain unit normal lengths");
    r = derive_material_render_transform(Matrix3{{{-1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}, {});
    check(r.at("normal_transform") == Matrix3{{{1, 0, 0}, {0, -1, 0}, {0, 0, -1}}} &&
              r.at("normal_row_alignment") == Point3{-1, -1, -1} &&
              r.at("normalize_transformed_normal") == true,
          "mirroring retains signed cofactors rather than dividing by a negative determinant");
    r = derive_material_render_transform(Matrix3{{{1, 0, 0}, {0, 1, 0}, {0, 0, 0}}}, {});
    check(r.at("status") == "computed" &&
              r.at("normal_transform") == Matrix3{{{0, 0, 0}, {0, 0, 0}, {0, 0, 1}}} &&
              r.at("normalize_transformed_normal") == true,
          "rank-two transform preserves surviving normal cofactor");
    r = derive_material_render_transform(zero, {});
    check(r.at("status") == "computed" && r.at("normal_transform") == zero &&
              r.at("geometry_scale_candidates").at("all_three_rows") == 0 &&
              r.at("normalize_transformed_normal") == true,
          "zero matrix remains zero without invented unit scale or inverse");

    const Matrix3 shear{{{1, 2, 0}, {0, 1, 0}, {0, 0, 1}}};
    r = derive_material_render_transform(shear, {});
    const auto n = r.at("normal_transform").get<Matrix3>();
    const double inv = 1 / std::sqrt(5.);
    check(n == Matrix3{{{double(float(inv)), 0, 0},
                        {double(float(-2 * inv)), double(float(inv)), 0},
                        {0, 0, double(float(inv))}}} &&
              r.at("row_squared_lengths") == Point3{5, 1, 1},
          "shear uses row scales and the untransposed cofactor matrix");
    // Independent geometric property: transformed normal is perpendicular to
    // both transformed tangents, within the final float rounding error.
    const Point3 tangent_a{2, 1, 0}, tangent_b{0, 0, 1};
    check(std::abs(n[0][0] * tangent_a[0] + n[1][0] * tangent_a[1] + n[2][0] * tangent_a[2]) <
                  1e-7 &&
              std::abs(n[0][0] * tangent_b[0] + n[1][0] * tangent_b[1] + n[2][0] * tangent_b[2]) <
                  1e-7,
          "cofactor-transformed normal remains perpendicular to transformed tangents");

    r = derive_material_render_transform(Matrix3{{{1, 2, 3}, {0, 1, 4}, {5, 6, 0}}}, {});
    Matrix3 dense_normal{{{-24, 20, -5}, {18, -15, 4}, {5, -4, 1}}};
    for (auto &row : dense_normal)
        for (auto &v : row)
            v = double(float(v * (1 / std::sqrt(1001.))));
    check(r.at("normal_transform") == dense_normal &&
              r.at("row_squared_lengths") == Point3{14, 17, 61},
          "dense matrix confirms all nine signed cofactor positions and row ordering");
    r = derive_material_render_transform(Matrix3{{{1e-30, 0, 0}, {0, 1e-30, 0}, {0, 0, 1e-30}}},
                                         {});
    check(r.at("status") == "computed" && r.at("normal_transform") == identity &&
              r.at("geometry_scale_candidates").at("all_three_rows") == 0 &&
              r.at("normalize_transformed_normal") == false,
          "float scale underflow does not erase double cofactor normalization");

    Matrix3 almost_singular{{{1, 1, 0}, {1, 1 + std::ldexp(1., -30), 0}, {0, 0, 1}}};
    r = derive_material_render_transform(almost_singular, {});
    check(r.at("vertex_linear_transform")[1][1] == 1 &&
              r.at("normal_transform")[2][2].get<double>() > 0 &&
              r.at("origin_basis_transform") == almost_singular,
          "double cofactor calculation precedes float matrix conversion");
    const float threshold = 0.9999600052833557f;
    for (const float s :
         {std::nextafter(threshold, 0.f), threshold, std::nextafter(threshold, 1.f)}) {
        const Matrix3 near_uniform{{{1, 0, 0}, {0, 1, 0}, {0, 0, s}}};
        r = derive_material_render_transform(near_uniform, {});
        check(r.at("normalize_transformed_normal") == (s < threshold),
              "normalization threshold is strict and uses native float value");
    }
    for (const double bad : {std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity(), 1e40}) {
        auto invalid = identity;
        invalid[1][2] = bad;
        r = derive_material_render_transform(invalid, {});
        check(r.at("status") == "not_evaluated" && !r.contains("normal_transform"),
              "invalid matrix publishes no partially usable derived state");
        r = derive_material_render_transform(identity, {bad, 0, 0});
        check(r.at("status") == "not_evaluated" && !r.contains("vertex_translation"),
              "invalid translation is rejected");
    }
    r = derive_material_render_transform(Matrix3{{{1e20, 0, 0}, {0, 1, 0}, {0, 0, 1}}}, {});
    check(r.at("status") == "not_evaluated" && !r.contains("normal_transform"),
          "float scale arithmetic overflow cannot silently yield a computed state");

    // Feed the derived state into the public planar path and compare a known
    // transformed point and normal. No manual normal matrix is supplied.
    MaterialPlanarRenderContext planar;
    planar.enabled = true;
    planar.geometry_kind = 1;
    planar.vertex_linear_transform = scaled.at("vertex_linear_transform").get<Matrix3>();
    planar.origin_basis_transform = scaled.at("origin_basis_transform").get<Matrix3>();
    planar.vertex_translation = scaled.at("vertex_translation").get<Point3>();
    planar.normal_transform = scaled.at("normal_transform").get<Matrix3>();
    planar.normalize_transformed_normal = scaled.at("normalize_transformed_normal").get<bool>();
    planar.reference_linear_transform = identity;
    planar.reference_translation = Point3{};
    const Json uv = {{"scope", "native_layer_uv_affine_transform"},
                     {"status", "computed"},
                     {"mapping_mode", 2},
                     {"scale_mode", 0},
                     {"matrix", Matrix2x3{{{1, 0, 0}, {0, 1, 0}}}}};
    const auto prepared = prepare_material_planar_sampling(uv, planar);
    const auto sample = sample_material_planar(prepared, {1, 2, 3}, Point3{0, 0, 1});
    check(sample.at("status") == "computed" && sample.at("uv") == Point2{2, 6},
          "derived nonuniform transform feeds planar point and normal sampling");
    return checks;
}
