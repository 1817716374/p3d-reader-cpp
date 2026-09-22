#include "internal.hpp"
#include <p3d/material_parametric.hpp>

unsigned material_parametric_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Matrix2x3 uv{{{2, 3, 5}, {7, 11, 13}}};
    Json transform = {{"scope", "native_layer_uv_affine_transform"},
                      {"status", "computed"},
                      {"mapping_mode", 0},
                      {"scale_mode", 0},
                      {"matrix", uv}};
    MaterialParametricRenderContext context;
    auto state = prepare_material_parametric_sampling(transform, context);
    check(state.at("status") == "prepared" && state.at("projection_flags") == 0 &&
              state.at("uv_transform") == uv && state.at("frame_override").is_null(),
          "relative parametric mapping needs only the registered affine");
    auto expect = [&](const Json &s, const Point3 &point, const std::optional<Point2> &native,
                      const std::optional<MaterialParametricFrame> &frame, const Point2 &expected,
                      const char *branch) {
        const auto result = sample_material_parametric(s, point, native, frame);
        check(result.at("status") == "computed" && result.at("uv") == expected &&
                  result.at("branch") == branch,
              "parametric branch priority and UV match the analytic result");
    };
    expect(state, {nan, nan, nan}, Point2{.25, .5}, {}, {7, 20.25}, "native_uv");
    expect(state, {2, 4, nan}, {}, {}, {21, 71}, "vertex_xy");
    const MaterialParametricFrame frame{{1, 2, 3}, {{{2, 0, 0}, {0, 0, -4}}}};
    expect(state, {4, 8, 5}, Point2{nan, nan}, frame, {-7, -33}, "per_face_frame");
    context.preparation_frame_present = true;
    context.preparation_frame = MaterialParametricFrame{{nan, nan, nan}, {}};
    context.geometry_scale = nan;
    context.parameter_factors = Point2{nan, nan};
    check(prepare_material_parametric_sampling(transform, context) == state,
          "relative preparation does not read absolute-only frame or scale inputs");

    auto absolute = transform;
    absolute["scale_mode"] = 1;
    context = {};
    context.preparation_frame_present = false;
    context.geometry_scale = 2;
    context.use_parameter_factors = false;
    state = prepare_material_parametric_sampling(absolute, context);
    check(state.at("uv_transform") == Matrix2x3{{{4, 6, 5}, {14, 22, 13}}},
          "absolute mapping without a frame scales both UV columns and retains translations");
    expect(state, {nan, nan, nan}, Point2{.25, .5}, {}, {9, 27.5}, "native_uv");
    context.parameter_factors = Point2{nan, nan};
    context.preparation_frame = MaterialParametricFrame{{nan, nan, nan}, {}};
    check(prepare_material_parametric_sampling(absolute, context) == state,
          "disabled parameter factors and explicitly absent preparation frame are not read");
    context.use_parameter_factors = true;
    context.parameter_factors = Point2{3, -4};
    state = prepare_material_parametric_sampling(absolute, context);
    check(state.at("applied_parameter_factors") == Point2{6, -8} &&
              state.at("uv_transform") == Matrix2x3{{{12, -24, 5}, {42, -88, 13}}},
          "independent parameter factors scale columns rather than rows");
    expect(state, {nan, nan, nan}, Point2{.25, .5}, {}, {-4, -20.5}, "native_uv");
    auto negative_mode = absolute;
    negative_mode["scale_mode"] = -4;
    check(prepare_material_parametric_sampling(negative_mode, context) == state,
          "all nonzero scale modes select the same native absolute branch");

    context.preparation_frame_present = true;
    context.preparation_frame = frame;
    context.use_parameter_factors.reset();
    context.parameter_factors.reset();
    state = prepare_material_parametric_sampling(absolute, context);
    check(state.at("uv_transform") == uv &&
              state.at("frame_override").at("origin") == Point3{1, 2, 3} &&
              state.at("frame_override").at("axes") == Matrix2x3{{{2, 0, 0}, {0, 0, -2}}},
          "absolute preparation normalizes each axis before scale; origin and UV matrix stay "
          "unchanged");
    const MaterialParametricFrame unread{{nan, nan, nan}, {{{nan, nan, nan}, {nan, nan, nan}}}};
    expect(state, {4, 8, 5}, Point2{nan, nan}, unread, {5, 11}, "prepared_frame");
    expect(state, {nan, nan, nan}, Point2{.25, .5}, {}, {7, 20.25}, "native_uv");
    expect(state, {4, 8, nan}, {}, {}, {37, 129}, "vertex_xy");
    context.geometry_scale = -2;
    auto negative = prepare_material_parametric_sampling(absolute, context);
    expect(negative, {4, 8, 5}, {}, unread, {5, 15}, "prepared_frame");
    context.geometry_scale = 0;
    auto zero = prepare_material_parametric_sampling(absolute, context);
    expect(zero, {4, 8, 5}, {}, unread, {5, 13}, "prepared_frame");
    context.preparation_frame = MaterialParametricFrame{{0, 0, 0}, {{{3, 4, 0}, {0, 0, 0}}}};
    context.geometry_scale = 3;
    auto rounded = prepare_material_parametric_sampling(absolute, context);
    check(rounded.at("frame_override").at("axes") ==
              Matrix2x3{{{1.80000007152557373046875, 2.400000095367431640625, 0}, {0, 0, 0}}},
          "axis normalization rounds to float before float geometry scale; a zero axis stays zero");
    context.preparation_frame_present = false;
    context.use_parameter_factors = true;
    context.parameter_factors = Point2{1 + std::ldexp(1., -24), 1};
    context.geometry_scale = 1 + std::ldexp(1., -24);
    rounded = prepare_material_parametric_sampling(absolute, context);
    check(rounded.at("uv_transform") == uv,
          "parameter factors and geometry scale each round to float before multiplication");
    auto simple = transform;
    simple["matrix"] = Matrix2x3{{{1, 0, -1}, {0, 1, 0}}};
    auto simple_state = prepare_material_parametric_sampling(simple, {});
    expect(simple_state, {nan, nan, nan}, Point2{1 + std::ldexp(1., -24), 2}, {}, {0, 2},
           "native_uv");
    expect(simple_state, {1 + std::ldexp(1., -24), 2, nan}, {}, {}, {0, 2}, "vertex_xy");

    auto rejected = [&](const Json &t, const MaterialParametricRenderContext &c) {
        const auto result = prepare_material_parametric_sampling(t, c);
        check(result.at("status") == "not_evaluated" && !result.contains("uv_transform"),
              "unknown or invalid preparation inputs never publish a usable transform");
    };
    rejected(absolute, {});
    auto bad = context;
    bad.preparation_frame_present.reset();
    rejected(absolute, bad);
    bad = context;
    bad.geometry_scale.reset();
    rejected(absolute, bad);
    bad = context;
    bad.use_parameter_factors.reset();
    rejected(absolute, bad);
    bad = context;
    bad.parameter_factors.reset();
    rejected(absolute, bad);
    bad = context;
    bad.parameter_factors = Point2{nan, 1};
    rejected(absolute, bad);
    bad = context;
    bad.parameter_factors = Point2{1e30, 1};
    bad.geometry_scale = 1e20;
    rejected(absolute, bad);
    bad = context;
    bad.preparation_frame_present = true;
    bad.preparation_frame.reset();
    rejected(absolute, bad);
    bad.preparation_frame = unread;
    rejected(absolute, bad);
    auto wrong = transform;
    wrong["mapping_mode"] = 2;
    rejected(wrong, {});
    for (const char *key : {"scope", "status", "mapping_mode", "scale_mode", "matrix"}) {
        wrong = transform;
        wrong.erase(key);
        rejected(wrong, {});
    }
    for (const char *key : {"scope", "status", "mapping_mode", "projection_flags", "uv_transform",
                            "frame_override"}) {
        auto invalid = simple_state;
        invalid.erase(key);
        const auto result = sample_material_parametric(invalid, {1, 2, 3});
        check(result.at("status") == "not_evaluated" && !result.contains("uv"),
              "incomplete prepared states cannot produce a partial UV");
    }
    auto invalid = simple_state;
    invalid["projection_flags"] = 2;
    check(sample_material_parametric(invalid, {}).at("status") == "not_evaluated",
          "other mapping flags cannot silently select parametric mapping");
    invalid = simple_state;
    invalid["frame_override"] = state.at("frame_override");
    check(sample_material_parametric(invalid, {}).at("status") == "not_evaluated",
          "relative state cannot contain an absolute frame override");
    check(sample_material_parametric(simple_state, {nan, 1, 2}).at("status") == "not_evaluated",
          "used vertex coordinates must be finite");
    check(sample_material_parametric(simple_state, {1, 2, nan}, {}, frame).at("status") ==
              "not_evaluated",
          "per-face path reads Z even when the fallback path would not");
    check(sample_material_parametric(simple_state, {1, 2, 3}, Point2{nan, 1}).at("status") ==
              "not_evaluated",
          "invalid supplied UV does not fall back to vertex coordinates");

    const auto settings =
        material_settings({{"tag", "Material"},
                           {"attributes", {{"material_version", "9"}}},
                           {"text", ""},
                           {"children", Json::array({{{"tag", "Map"},
                                                      {"attributes", {{"Type", "1"}}},
                                                      {"text", ""},
                                                      {"children", Json::array()}}})}});
    const auto layer =
        settings.at("version_conversion").at("layer_mapping_getters").at("entries")[0];
    const auto registered = build_material_uv_transform(layer, {});
    const auto integrated = prepare_material_parametric_sampling(registered, {});
    expect(integrated, {nan, nan, nan}, Point2{.25, .75}, {}, {.25, -.75}, "native_uv");
    const auto snapshot = integrated.dump();
    sample_material_parametric(integrated, {1, 2, 3});
    check(snapshot == integrated.dump(), "prepared parametric state is read-only and reusable");
    return checks;
}
