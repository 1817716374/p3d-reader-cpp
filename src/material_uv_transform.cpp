#include "internal.hpp"

namespace p3d {
Json build_material_uv_transform(const Json &layer, const MaterialUvTransformContext &context) {
    Json out = {{"scope", "native_layer_uv_affine_transform"}, {"status", "not_evaluated"}};
    auto fail = [&](const char *reason) {
        out["reason"] = reason;
        return out;
    };
    if (!layer.is_object() || !layer.contains("mapping") || !layer.contains("data_flags"))
        return fail("layer_mapping_getter_unavailable");
    try {
        const auto &parameters = layer.at("mapping").at("parameters");
        auto value = [&](const char *key) -> const Json & {
            return parameters.at(key).at("value");
        };
        auto number = [](const Json &v) {
            if (!v.is_number() || !std::isfinite(v.get<double>()))
                throw std::invalid_argument("unavailable_or_nonfinite_uv_parameter");
            return v.get<double>();
        };
        auto integer = [&](const Json &v) {
            const double n = number(v);
            if (!v.is_number_integer() || n < -2147483648. || n > 2147483647.)
                throw std::invalid_argument("invalid_uv_mapping_integer");
            return v.get<std::int32_t>();
        };
        auto xy = [&](const Json &v) {
            if (!v.is_array() || (v.size() != 2 && v.size() != 3))
                throw std::invalid_argument("invalid_uv_mapping_vector");
            return Point2{number(v[0]), number(v[1])};
        };
        const auto mode = integer(value("pattern_mapping"));
        const auto scale_mode = integer(value("pattern_scalemode"));
        const auto angle = number(value("pattern_angle")) * 0.017453292519943295;
        const auto scale = xy(value("pattern_scale")), offset = xy(value("pattern_offset"));
        const auto &flag_value = layer.at("data_flags").at("value");
        const auto numeric_flags = number(flag_value);
        if (!flag_value.is_number_integer() || numeric_flags < 0 || numeric_flags > 4294967295.)
            return fail("invalid_uv_mapping_data_flags");
        const auto flags = flag_value.get<std::uint32_t>();
        Point2 factors{1, 1};
        const bool uses_units = mode == 1 || scale_mode != 0;
        const bool needs_units =
            mode == 1 || (uses_units && (std::abs(scale[0]) > 1e-10 || std::abs(scale[1]) > 1e-10));
        double unit = 1;
        if (needs_units) {
            if (!context.mapping_unit_factor || !std::isfinite(*context.mapping_unit_factor))
                return fail("missing_or_nonfinite_mapping_unit_factor");
            unit = *context.mapping_unit_factor;
        }
        for (unsigned i = 0; i < 2; ++i)
            if (std::abs(scale[i]) > 1e-10)
                factors[i] = 1 / (uses_units ? unit * scale[i] : scale[i]);
        factors[0] *= flags & 16u ? -1 : 1;
        factors[1] *= flags & 1u ? -1 : 1;
        const double c = std::cos(angle), s = std::sin(angle);
        Matrix2x3 matrix{{{factors[0] * c, factors[0] * s, offset[0]},
                          {factors[1] * s, -factors[1] * c, offset[1]}}};
        if (mode == 1) {
            if (!context.elevation_origin || !std::isfinite((*context.elevation_origin)[0]) ||
                !std::isfinite((*context.elevation_origin)[1]))
                return fail("missing_or_nonfinite_elevation_origin");
            const Point2 shifted{offset[0] * unit + (*context.elevation_origin)[0],
                                 offset[1] * unit + (*context.elevation_origin)[1]};
            matrix[0][2] = -(matrix[0][1] * shifted[1] + matrix[0][0] * shifted[0]);
            matrix[1][2] = 1 - (matrix[1][1] * shifted[1] + matrix[1][0] * shifted[0]);
        }
        const auto before_registration = matrix;
        bool registered = false;
        if (scale_mode != 0) {
            if (!context.geometry_projection_succeeded)
                return fail("missing_uv_registration_branch");
            if (*context.geometry_projection_succeeded) {
                if (!context.registration_unit_factor ||
                    !std::isfinite(*context.registration_unit_factor))
                    return fail("missing_or_nonfinite_registration_unit_factor");
                for (auto &row : matrix) {
                    row[0] *= *context.registration_unit_factor;
                    row[1] *= *context.registration_unit_factor;
                }
                registered = true;
            }
        }
        for (const auto &m : {before_registration, matrix})
            for (const auto &row : m)
                for (double n : row)
                    if (!std::isfinite(n))
                        return fail("nonfinite_uv_transform_arithmetic");
        out.update({{"status", "computed"},
                    {"matrix", matrix},
                    {"before_registration", before_registration},
                    {"registration_unit_applied", registered},
                    {"mapping_mode", mode},
                    {"scale_mode", scale_mode},
                    {"u_flipped", bool(flags & 16u)},
                    {"v_flipped", bool(flags & 1u)}});
    } catch (const Json::exception &) {
        return fail("missing_or_invalid_layer_mapping_fields");
    } catch (const std::invalid_argument &e) {
        return fail(e.what());
    }
    return out;
}
} // namespace p3d
