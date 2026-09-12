#include "internal.hpp"

unsigned material_uv_transform_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto node = [](const char *tag, Json a = Json::object(), Json children = Json::array()) {
        return Json{{"tag", tag}, {"attributes", a}, {"children", children}, {"text", ""}};
    };
    auto parse = [&](Json a, Json children = Json::array()) {
        a["Type"] = "1";
        return material_settings(
            node("Material", {{"material_version", "9"}}, Json::array({node("Map", a, children)})));
    };
    auto first = [](const Json &settings) {
        return settings.at("version_conversion").at("layer_mapping_getters").at("entries")[0];
    };
    MaterialUvTransformContext context;
    auto layer = first(parse(Json::object()));
    auto result = build_material_uv_transform(layer, context);
    check(result.at("status") == "computed" &&
              result.at("matrix") == Matrix2x3{{{1, 0, 0}, {0, -1, 0}}},
          "constructor mapping produces native V direction rather than identity");
    Json a = {{"pattern_mapping", "3"},
              {"pattern_scale.x", "2"},
              {"pattern_scale.y", "4"},
              {"pattern_offset.x", "7"},
              {"pattern_offset.y", "11"},
              {"pattern_angle", "0"},
              {"Flags", "17"},
              {"scale_z", "0"}};
    auto settings = parse(a);
    auto without_z = a;
    without_z.erase("scale_z");
    check(first(parse(without_z)).at("mapping").at("parameters").at("scale_z").at("value") == 0,
          "successful XY scale assignment clears Z when no later Z value is supplied");
    layer = first(settings);
    check(layer.at("mapping").at("parameters").at("scale_z").at("value") == 0,
          "explicit Z scale is stored verbatim");
    result = build_material_uv_transform(layer, context);
    check(result.at("matrix") == Matrix2x3{{{-.5, 0, 7}, {0, .25, 11}}} &&
              result.at("u_flipped") == true && result.at("v_flipped") == true,
          "flip bits are data bits 4 and 0, with translations retained");
    a["pattern_angle"] = "90";
    a["Flags"] = "0";
    layer = first(parse(a));
    result = build_material_uv_transform(layer, context);
    auto matrix = result.at("matrix").get<Matrix2x3>();
    check(std::abs(matrix[0][0]) < 1e-15 && matrix[0][1] == .5 && matrix[1][0] == .25 &&
              std::abs(matrix[1][1]) < 1e-15 && matrix[0][2] == 7 && matrix[1][2] == 11,
          "rotation acts before signed row scaling and retains direct offsets");
    a["pattern_angle"] = "0";
    a["pattern_scalemode"] = "3";
    layer = first(parse(a));
    check(build_material_uv_transform(layer, context).at("status") == "not_evaluated",
          "absolute mode requires its unit context");
    context.mapping_unit_factor = 10;
    context.geometry_projection_succeeded = false;
    result = build_material_uv_transform(layer, context);
    check(result.at("matrix") == Matrix2x3{{{.05, 0, 7}, {0, -.025, 11}}} &&
              result.at("registration_unit_applied") == false,
          "unit conversion participates in the reciprocal before registration");
    context.geometry_projection_succeeded = true;
    check(build_material_uv_transform(layer, context).at("reason") ==
              "missing_or_nonfinite_registration_unit_factor",
          "successful geometric projection requires a known registration unit factor");
    context.registration_unit_factor = 2;
    result = build_material_uv_transform(layer, context);
    check(result.at("matrix") == Matrix2x3{{{.1, 0, 7}, {0, -.05, 11}}} &&
              result.at("before_registration") == Matrix2x3{{{.05, 0, 7}, {0, -.025, 11}}},
          "registration scales linear coefficients only");
    a["pattern_mapping"] = "1";
    a["pattern_scalemode"] = "0";
    layer = first(parse(a));
    context.elevation_origin = Point2{30, 40};
    result = build_material_uv_transform(layer, context);
    check(result.at("matrix") == Matrix2x3{{{.05, 0, -5}, {0, -.025, 4.75}}},
          "elevation mode transforms scaled offsets plus model origin and uses one-minus V");
    // Rotate a known offset-origin point; the elevation affine maps that point
    // to (0,1) for all rotations, sizes, and independent flips.
    for (int degrees = -177; degrees <= 177; degrees += 13) {
        for (unsigned flags : {0u, 1u, 16u, 17u}) {
            a["pattern_angle"] = std::to_string(degrees);
            a["Flags"] = std::to_string(flags);
            const auto m =
                build_material_uv_transform(first(parse(a)), context).at("matrix").get<Matrix2x3>();
            check(
                std::abs(m[0][0] * 100 + m[0][1] * 150 + m[0][2]) < 3e-14 &&
                    std::abs(m[1][0] * 100 + m[1][1] * 150 + m[1][2] - 1) < 3e-14,
                "elevation reference point maps to its fixed UV independent of rotation and flips");
        }
    }
    a = {{"pattern_mapping", "3"},
         {"pattern_angle", "0"},
         {"pattern_scale.x", "1e-10"},
         {"pattern_scale.y", "-1e-10"},
         {"pattern_scalemode", "4"}};
    context = MaterialUvTransformContext{};
    context.geometry_projection_succeeded = false;
    result = build_material_uv_transform(first(parse(a)), context);
    check(result.at("matrix") == Matrix2x3{{{1, 0, 0}, {0, -1, 0}}},
          "inclusive size threshold uses one without reading an unused unit factor");
    a["pattern_scale.x"] = "1.000000001e-10";
    check(build_material_uv_transform(first(parse(a)), context).at("reason") ==
              "missing_or_nonfinite_mapping_unit_factor",
          "size just above threshold needs the unit factor");
    a = {{"pattern_mapping", "3"}, {"pattern_scale.x", "nan"}};
    layer = first(parse(a));
    check(layer.at("mapping").at("parameters").at("pattern_scale").at("value") == Point2{1, 1},
          "incomplete vector read preserves both constructor coordinates");
    a["pattern_scale.y"] = "2";
    layer = first(parse(a));
    check(build_material_uv_transform(layer, context).at("status") == "not_evaluated",
          "assigned NaN scale stays decoded but is not usable for UV arithmetic");
    a = {{"Filename", "layers.pma"}};
    settings = parse(a, Json::array({node("Layer", {{"LayerType", "layer IMAGE x.jpg"},
                                                    {"pattern_mapping", "5"},
                                                    {"pattern_angle", "13"},
                                                    {"LayerDataFlags", "17"}}),
                                     node("Ignored")}));
    const auto &layers = settings.at("maps")[0].at("initial_layer_state").at("layers");
    check(layers.size() == 2 &&
              layers[0].at("mapping").at("parameters").at("pattern_mapping").at("value") == 5 &&
              layers[1].at("mapping").at("parameters").at("pattern_mapping").at("value") == 0,
          "layer mapping follows accepted child order including a trailing default layer");
    for (int version : {7, 9}) {
        settings =
            material_settings(node("Material", {{"material_version", std::to_string(version)}},
                                   Json::array({node("Map", {{"Type", "1"},
                                                             {"pattern_mapping", "5"},
                                                             {"pattern_angle", "35"},
                                                             {"Flags", "16"}}),
                                                node("Map", {{"Type", "30"},
                                                             {"map_link", "1"},
                                                             {"pattern_mapping", "3"},
                                                             {"Flags", "1"}})})));
        const auto &entries =
            settings.at("version_conversion").at("layer_mapping_getters").at("entries");
        const auto &linked = entries.back();
        check(linked.at("mapping_source_object_id") == 0 &&
                  linked.at("data_flags_source_object_id") == 1 &&
                  linked.at("mapping").at("parameters").at("pattern_mapping").at("value") == 5 &&
                  (linked.at("data_flags").at("value").get<unsigned>() & 17u) == 1,
              "type 30 copies linked mapping but retains local U/V flag bits");
        const auto &local_mapping = settings.at("version_conversion")
                                        .at("layer_containers")
                                        .at("containers")[1]
                                        .at("layers")[0]
                                        .at("mapping");
        check(local_mapping.at("parameters").at("pattern_mapping").at("value") ==
                  (version < 8 ? 5 : 3),
              "older conversion performs mapping copy while later versions wait for the getter");
    }
    // Connect XML -> getter -> UV transform -> render preparation -> point UV.
    settings = parse({{"pattern_mapping", "3"},
                      {"pattern_scale.x", "2"},
                      {"pattern_scale.y", "4"},
                      {"Flags", "16"}});
    context = MaterialUvTransformContext{};
    result = build_material_uv_transform(first(settings), context);
    MaterialProjectionRenderContext render;
    render.geometry_kind = 1;
    render.reference_point = Point3{};
    render.reference_transform = Matrix4{{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
    render.uv_transform = result.at("matrix").get<Matrix2x3>();
    Json transform = {{"status", "resolved"},
                      {"mapping_mode", 3},
                      {"preparation", {{"scale_mode", 0}}},
                      {"matrix", Matrix3{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}},
                      {"origin", Point3{}},
                      {"reference_dimensions", Point3{2, 4, 8}}};
    const auto sampled = sample_material_projection(
        prepare_material_projection_sampling(transform, render), {1, 2, 0});
    check(sampled.at("status") == "computed" && sampled.at("uv") == Point2{-.5, -.25},
          "source-derived UV transform feeds the native point sampler");
    for (const auto *key : {"pattern_mapping", "pattern_scalemode", "pattern_angle",
                            "pattern_scale", "pattern_offset"}) {
        auto bad = first(settings);
        bad["mapping"]["parameters"][key]["value"] = nullptr;
        check(build_material_uv_transform(bad, context).at("status") == "not_evaluated",
              "unavailable mapping field does not get a guessed default");
    }
    return checks;
}
