#include "internal.hpp"

unsigned material_projection_math_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto near = [](double a, double b) {
        return std::abs(a - b) <= 2e-14 * std::max({1., std::abs(a), std::abs(b)});
    };
    auto frame = [](Point3 angles) {
        return Json{{"object_id", 8},
                    {"frame_source_object_id", 3},
                    {"parameters",
                     {{"pattern_proj_offset", {{"value", Point3{.25, -.5, 2}}}},
                      {"pattern_proj_scale", {{"value", Point3{2, 3, 4}}}},
                      {"pattern_proj_angles", {{"value", angles}}}}}};
    };
    MaterialProjectionContext c;
    c.mapping_mode = 4;
    c.reference_point = Point3{10, 20, 30};
    c.reference_dimensions = Point3{2, 3, 5};
    c.reference_matrix = Matrix3{{{1, 2, 0}, {0, 1, 0}, {0, 0, 3}}};
    c.layer_data_flags = 5;
    auto g = frame({90, 0, 90});
    const auto source = g;
    auto r = prepare_material_projection(g, c);
    check(g == source && r["status"] == "prepared" && r["source_object_id"] == 8 &&
              r["frame_source_object_id"] == 3,
          "preparation consumes a getter snapshot without mutating source or changing provenance");
    const Matrix3 expected{{{0, -1, -2}, {0, 0, -1}, {3, 0, 0}}};
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 3; ++j)
            check(
                near(r["orientation_matrix"][i][j].get<double>(), expected[i][j]),
                "native orientation is reference basis times Rx Ry Rz in column-vector convention");
    check(r["projection_offset"] == Json::array({.5, -1.5, 10.}) &&
              r["origin"] == Json::array({10.5, 18.5, 40.}) &&
              r["projection_scale"] == Json::array({2., 3., 4.}) && r["layer_data_flag_bit_2"] == 1,
          "relative offsets use reference dimensions and then add the reference point");
    c.mapping_mode = 3;
    r = prepare_material_projection(g, c);
    check(near(r["reference_dimensions"][0].get<double>(), 5) &&
              near(r["reference_dimensions"][1].get<double>(), 2) &&
              near(r["reference_dimensions"][2].get<double>(), 3),
          "mode 3 dimensions use the opposite sequential rotation order Rz Ry Rx");
    check(near(r["origin"][0].get<double>(), 11.25) && near(r["origin"][1].get<double>(), 19) &&
              near(r["origin"][2].get<double>(), 36),
          "mode 3 relative offsets use the rotated absolute dimension components");
    c.reference_dimensions = Point3{1, 1, 1};
    r = prepare_material_projection(frame({0, 0, 45}), c);
    check(
        r["reference_dimensions"][0].get<double>() < 1e-14 &&
            near(r["reference_dimensions"][1].get<double>(), std::sqrt(2.)),
        "mode 3 takes absolute value after vector rotation rather than inventing an enclosing box");
    c.reference_dimensions = Point3{2, 3, 5};
    c.scale_mode = 3;
    c.absolute_unit_factor = 4;
    g = frame({0, 0, 0});
    for (const int mode : {3, 4, 5, 6, 7}) {
        c.mapping_mode = mode;
        r = prepare_material_projection(g, c);
        const Point3 dimensions = mode == 5   ? Point3{.5, .75, 1.25}
                                  : mode == 6 ? Point3{.5, 4, 5}
                                              : Point3{4, 4, 4};
        check(r["reference_dimensions"] == Json(dimensions) &&
                  r["projection_offset"] == Json::array({.25, -.5, 2.}) &&
                  r["origin"] == Json::array({10.25, 19.5, 32.}) &&
                  r["orientation_matrix"] == Json(*c.reference_matrix),
              "absolute unit preparation has distinct dimension rules and leaves offsets unscaled");
    }
    c.mapping_mode = 6;
    c.scale_mode = -7;
    check(prepare_material_projection(g, c)["reference_dimensions"] == Json::array({.5, 4., 5.}),
          "every nonzero source scale mode enters the absolute-unit branch");
    c.absolute_unit_factor.reset();
    check(prepare_material_projection(g, c)["reason"] == "missing_absolute_unit_factor",
          "absolute unit factor must come from caller context and is never guessed as one");
    c.scale_mode = 0;
    c.absolute_unit_factor = std::numeric_limits<double>::quiet_NaN();
    check(prepare_material_projection(g, c)["status"] == "prepared",
          "relative branch does not consult an unused unit factor");
    c.scale_mode = 3;
    check(prepare_material_projection(g, c)["reason"] == "nonfinite_absolute_unit_factor",
          "nonfinite used factor is explicit");
    c.absolute_unit_factor = 0;
    r = prepare_material_projection(g, c);
    check(r["reason"] == "nonfinite_preparation_result" && !r.contains("orientation_matrix"),
          "division by zero does not publish apparently complete finite preparation");
    c.mapping_mode = 4;
    check(prepare_material_projection(g, c)["reference_dimensions"] == Json::array({0., 0., 0.}),
          "zero factor is not rejected in branches that only assign it");
    c.scale_mode = 0;
    g["parameters"]["pattern_proj_scale"]["value"] = Point3{0, -1, 2};
    g["status"] = "partial";
    g["matrix"] = nullptr;
    r = prepare_material_projection(g, c);
    check(r["status"] == "prepared" && r["projection_scale"] == Json::array({0., -1., 2.}) &&
              r["point_mapping"] == "not_evaluated",
          "preparation copies frame scale without downstream zero normalization or explicit matrix "
          "evaluation");
    g["parameters"]["pattern_proj_angles"]["value"][0] = {{"floating_point", "nan"},
                                                          {"ieee754_hex", "ffffffffffffffff"}};
    check(prepare_material_projection(g, c)["reason"] ==
              "unavailable_or_nonfinite_projection_frame",
          "tagged source NaN remains source data and cannot masquerade as a finite rotation");
    c.reference_point.reset();
    check(prepare_material_projection(frame({0, 0, 0}), c)["reason"] ==
              "missing_geometry_projection_context",
          "reference coordinates cannot be inferred from absent context");
    c.reference_point = Point3{10, 20, 30};
    c.reference_dimensions = Point3{2, 3, std::numeric_limits<double>::max()};
    check(prepare_material_projection(frame({0, 0, 0}), c)["reason"] ==
              "nonfinite_preparation_result",
          "finite inputs with overflowing offset arithmetic report an unevaluated result");
    for (const int mode : {-1, 0, 1, 2, 8, INT32_MAX}) {
        c.mapping_mode = mode;
        check(prepare_material_projection(g, c)["reason"] ==
                  "mapping_mode_outside_preparation_branch",
              "other native mapping branches are not replaced with this preparation algorithm");
    }
    return checks;
}
