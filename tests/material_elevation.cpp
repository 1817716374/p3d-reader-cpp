#include "internal.hpp"
#include <p3d/material_elevation.hpp>

unsigned material_elevation_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Matrix2x3 identity{{{1, 0, 0}, {0, 1, 0}}};
    Json transform = {{"scope", "native_layer_uv_affine_transform"},
                      {"status", "computed"},
                      {"mapping_mode", 1},
                      {"scale_mode", 0},
                      {"matrix", Matrix2x3{{{2, -1, .25}, {.5, 3, -.75}}}}};
    MaterialElevationRenderContext context;
    context.geometry_kind = 1;
    context.reference_xy = Point2{5, -2};
    context.texture_present = false;
    auto state = prepare_material_elevation_sampling(transform, context);
    check(state.at("status") == "prepared" && state.at("projection_flags") == 2 &&
              state.at("uv_transform") == Matrix2x3{{{2, -1, .25}, {.5, 3, .75}}},
          "absent texture reduces both offsets with floor, including negative offsets");
    auto expect = [&](const Json &s, const Point3 &p,
                      const std::optional<MaterialElevationFrame> &frame, const Point2 &uv,
                      const char *branch) {
        const auto value = sample_material_elevation(s, p, frame);
        check(value.at("status") == "computed" && value.at("uv") == uv &&
                  value.at("branch") == branch,
              "elevation projection matches analytic UV and selected branch");
    };
    expect(state, {3, 4, nan}, {}, {2.25, 14.25}, "nonzero_geometry_kind");
    context.texture_present = true;
    context.texture_axis_flags = std::array<std::int32_t, 2>{0, -2};
    state = prepare_material_elevation_sampling(transform, context);
    check(state.at("offset_reduced") == Json::array({false, true}),
          "axis flags are independent and any nonzero value reduces the offset");
    expect(state, {3, 4, 0}, {}, {14.25, 14.25}, "nonzero_geometry_kind");
    context.texture_axis_flags = std::array<std::int32_t, 2>{0, 0};
    state = prepare_material_elevation_sampling(transform, context);
    expect(state, {3, 4, 0}, {}, {14.25, 9.25}, "nonzero_geometry_kind");
    for (auto scale_mode : {0, 1, -7}) {
        auto t = transform;
        t["scale_mode"] = scale_mode;
        auto c = context;
        c.geometry_kind = -1;
        c.geometry_scale = nan;
        c.preserve_registered_offset = true;
        c.vertex_frame = MaterialElevationFrame{{nan, nan, nan}, {}};
        const auto s = prepare_material_elevation_sampling(t, c);
        check(s.at("uv_transform") == state.at("uv_transform") &&
                  s.at("projection_flags") == (scale_mode == 0 ? 2 : 3),
              "nonzero kind ignores geometry scale, vertex frame and zero-kind offset flag");
    }
    context.texture_present = false;
    context.texture_axis_flags.reset();
    state = prepare_material_elevation_sampling(transform, context);
    const MaterialElevationFrame face{{10, 20, 30}, {{{0, 0, 2}, {0, -1, 0}}}};
    expect(state, {11, 24, 33}, face, {16.25, -8.25}, "per_face_frame");
    context.geometry_kind = 0;
    context.preserve_registered_offset = false;
    context.geometry_scale = 2;
    context.vertex_frame = face;
    state = prepare_material_elevation_sampling(transform, context);
    check(state.at("inverse_squared_scale") == .25 &&
              state.at("uv_transform") == Matrix2x3{{{.5, -.25, .25}, {.125, .75, .75}}},
          "zero kind adjusts reference offsets before scaling only the linear rows");
    expect(state, {11, 24, 33}, {}, {4.25, -1.5}, "zero_geometry_kind");
    const MaterialElevationFrame override_frame{{0, 0, 0}, identity};
    expect(state, {4, 8, 3}, override_frame, {.25, 7.25}, "per_face_frame");
    auto negative_scale = context;
    negative_scale.geometry_scale = -2;
    check(prepare_material_elevation_sampling(transform, negative_scale) == state,
          "geometry scale is squared, including negative source scales");
    auto absolute = transform;
    absolute["scale_mode"] = 1;
    check(prepare_material_elevation_sampling(absolute, context).at("uv_transform") ==
              state.at("uv_transform"),
          "zero kind scales both relative and absolute mapping modes");
    context.preserve_registered_offset = true;
    context.reference_xy.reset();
    context.texture_present.reset();
    state = prepare_material_elevation_sampling(transform, context);
    check(state.at("registered_offset_preserved") == true &&
              state.at("offset_reduced") == Json::array({false, false}),
          "preserved translations do not require unused reference or texture inputs");
    expect(state, {11, 24, 33}, {}, {4.25, -3}, "zero_geometry_kind");

    auto simple = transform;
    simple["matrix"] = identity;
    auto rounding = context;
    rounding.geometry_scale = 1 + std::ldexp(1., -24);
    rounding.vertex_frame = MaterialElevationFrame{{0, 0, 0}, identity};
    auto s = prepare_material_elevation_sampling(simple, rounding);
    check(s.at("inverse_squared_scale") == 1,
          "render scale is rounded to float before squaring and reciprocal");
    const MaterialElevationFrame cancellation{{0, 0, 0}, {{{1, 1, 1}, {0, 0, 0}}}};
    expect(s, {1, 16777216, -16777216}, cancellation, {0, 0}, "per_face_frame");
    rounding.vertex_frame = cancellation;
    s = prepare_material_elevation_sampling(simple, rounding);
    expect(s, {1, 16777216, -16777216}, {}, {0, 0}, "zero_geometry_kind");
    rounding.vertex_frame = MaterialElevationFrame{{16777217, 0, 0}, identity};
    s = prepare_material_elevation_sampling(simple, rounding);
    expect(s, {16777218, 3, 0}, {}, {2, 3}, "zero_geometry_kind");
    auto direct = context;
    direct.geometry_kind = 1;
    direct.reference_xy = Point2{};
    direct.texture_present = true;
    direct.texture_axis_flags = std::array<std::int32_t, 2>{0, 0};
    simple["matrix"][0][2] = -16777216;
    s = prepare_material_elevation_sampling(simple, direct);
    expect(s, {16777217, 3, nan}, {}, {0, 3}, "nonzero_geometry_kind");
    check(sample_material_elevation(s, {1, 2, nan}, override_frame).at("status") == "not_evaluated",
          "face-frame projection actually reads Z even when its Z coefficients are zero");
    check(sample_material_elevation(s, {nan, 2, 0}).at("status") == "not_evaluated",
          "nonfinite used point component is rejected");

    auto rejected = [&](const Json &t, const MaterialElevationRenderContext &c) {
        check(prepare_material_elevation_sampling(t, c).at("status") == "not_evaluated",
              "unknown or unusable elevation context never becomes prepared");
    };
    auto missing = direct;
    missing.geometry_kind.reset();
    rejected(transform, missing);
    missing = direct;
    missing.texture_present.reset();
    rejected(transform, missing);
    missing = direct;
    missing.texture_axis_flags.reset();
    rejected(transform, missing);
    missing = direct;
    missing.reference_xy.reset();
    rejected(transform, missing);
    missing = direct;
    missing.reference_xy = Point2{nan, 0};
    rejected(transform, missing);
    missing = context;
    missing.preserve_registered_offset.reset();
    rejected(transform, missing);
    missing = context;
    missing.geometry_scale.reset();
    rejected(transform, missing);
    missing = context;
    missing.vertex_frame.reset();
    rejected(transform, missing);
    for (double bad_scale : {0., 1e-30, 1e30, nan}) {
        missing = context;
        missing.geometry_scale = bad_scale;
        rejected(transform, missing);
    }
    missing = context;
    missing.vertex_frame->axes[0][1] = nan;
    rejected(transform, missing);
    for (const char *key : {"scope", "status", "matrix", "mapping_mode", "scale_mode"}) {
        auto bad = transform;
        bad.erase(key);
        rejected(bad, direct);
    }
    auto wrong = transform;
    wrong["mapping_mode"] = 3;
    rejected(wrong, direct);
    wrong = transform;
    wrong["scale_mode"] = std::uint64_t{4294967296};
    rejected(wrong, direct);
    wrong = transform;
    wrong["matrix"][1] = Json::array({0, 1});
    rejected(wrong, direct);
    for (const char *key :
         {"scope", "status", "uv_transform", "mapping_mode", "projection_flags", "geometry_kind"}) {
        auto bad = s;
        bad.erase(key);
        const auto result = sample_material_elevation(bad, {1, 2, 3});
        check(result.at("status") == "not_evaluated" && !result.contains("uv"),
              "incomplete sampling states never publish a partial UV");
    }
    auto bad_state = s;
    bad_state["projection_flags"] = 8;
    check(sample_material_elevation(bad_state, {1, 2, 3}).at("status") == "not_evaluated",
          "other projection flags cannot be substituted into elevation state");
    bad_state = state;
    bad_state.erase("vertex_frame");
    check(sample_material_elevation(bad_state, {1, 2, 3}).at("status") == "not_evaluated",
          "zero-kind sampling requires its prepared vertex frame");

    // Source XML -> effective layer -> registered affine -> render state -> UV.
    const Json map = {{"tag", "Map"},
                      {"attributes",
                       {{"Type", "1"},
                        {"pattern_mapping", "1"},
                        {"pattern_scale.x", "2"},
                        {"pattern_scale.y", "4"},
                        {"pattern_offset.x", "7"},
                        {"pattern_offset.y", "11"}}},
                      {"children", Json::array()},
                      {"text", ""}};
    const auto settings = material_settings({{"tag", "Material"},
                                             {"attributes", {{"material_version", "9"}}},
                                             {"children", Json::array({map})},
                                             {"text", ""}});
    const auto layer =
        settings.at("version_conversion").at("layer_mapping_getters").at("entries")[0];
    MaterialUvTransformContext units;
    units.mapping_unit_factor = 10;
    units.elevation_origin = Point2{30, 40};
    const auto registered = build_material_uv_transform(layer, units);
    const auto integrated = prepare_material_elevation_sampling(registered, direct);
    expect(integrated, {100, 150, 99}, {}, {0, 1}, "nonzero_geometry_kind");
    const auto before = integrated.dump();
    sample_material_elevation(integrated, {5, 8, 0});
    check(integrated.dump() == before, "sampling leaves reusable prepared state unchanged");
    return checks;
}
