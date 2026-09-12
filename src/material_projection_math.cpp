#include "internal.hpp"

namespace p3d {
namespace {
constexpr Matrix3 identity{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
Matrix3 product(const Matrix3 &a, const Matrix3 &b) {
    Matrix3 out{};
    for (unsigned row = 0; row < 3; ++row)
        for (unsigned col = 0; col < 3; ++col)
            out[row][col] = (a[row][1] * b[1][col] + a[row][0] * b[0][col]) + a[row][2] * b[2][col];
    return out;
}
Point3 product(const Matrix3 &a, const Point3 &b) {
    Point3 out{};
    for (unsigned row = 0; row < 3; ++row)
        out[row] = (a[row][1] * b[1] + a[row][0] * b[0]) + a[row][2] * b[2];
    return out;
}
Matrix3 axis_rotation(unsigned axis, double angle) {
    auto out = identity;
    if (angle == 0)
        return out;
    const auto c = std::cos(angle), s = std::sin(angle);
    const auto i = (axis + 1) % 3, j = (axis + 2) % 3;
    out[i][i] = out[j][j] = c;
    out[i][j] = -s;
    out[j][i] = s;
    return out;
}
// Native rotation builds Rx*Ry*Rz, then pre-multiplies the input matrix.
Matrix3 rotate(const Matrix3 &input, const Point3 &angles) {
    auto rotation = axis_rotation(0, angles[0]);
    if (angles[1] != 0)
        rotation = product(rotation, axis_rotation(1, angles[1]));
    if (angles[2] != 0)
        rotation = product(rotation, axis_rotation(2, angles[2]));
    return product(rotation, input);
}
bool finite(const Point3 &p) {
    return std::all_of(p.begin(), p.end(), [](double v) { return std::isfinite(v); });
}
bool finite(const Matrix3 &m) {
    return std::all_of(m.begin(), m.end(), [](const Point3 &p) { return finite(p); });
}
bool point(const Json &parameters, const char *key, Point3 &value) {
    if (!parameters.is_object() || !parameters.contains(key) || !parameters.at(key).is_object())
        return false;
    const auto source = parameters.at(key).value("value", Json());
    if (!source.is_array() || source.size() != 3)
        return false;
    for (unsigned i = 0; i < 3; ++i) {
        if (!source[i].is_number())
            return false;
        value[i] = source[i].get<double>();
    }
    return finite(value);
}
} // namespace

Json prepare_material_projection(const Json &getter, const MaterialProjectionContext &context) {
    Json out = {{"scope", "native_projection_preparation_mapping_modes_3_to_7"},
                {"status", "not_evaluated"},
                {"mapping_mode", context.mapping_mode},
                {"scale_mode", context.scale_mode},
                {"layer_data_flags", context.layer_data_flags},
                {"point_mapping", "not_evaluated"}};
    auto fail = [&](const char *reason) {
        out["reason"] = reason;
        return out;
    };
    if (context.mapping_mode < 3 || context.mapping_mode > 7)
        return fail("mapping_mode_outside_preparation_branch");
    if (!context.reference_point || !context.reference_dimensions || !context.reference_matrix)
        return fail("missing_geometry_projection_context");
    if (!finite(*context.reference_point) || !finite(*context.reference_dimensions) ||
        !finite(*context.reference_matrix))
        return fail("nonfinite_geometry_projection_context");
    const auto parameters =
        getter.is_object() ? getter.value("parameters", Json::object()) : Json();
    Point3 offset{}, scale{}, degrees{};
    if (!point(parameters, "pattern_proj_offset", offset) ||
        !point(parameters, "pattern_proj_scale", scale) ||
        !point(parameters, "pattern_proj_angles", degrees))
        return fail("unavailable_or_nonfinite_projection_frame");
    Point3 radians{};
    for (unsigned i = 0; i < 3; ++i)
        radians[i] = degrees[i] * 0.017453292519943295;
    const auto rotation = rotate(identity, radians);
    auto dimensions = *context.reference_dimensions;
    if (context.scale_mode != 0) {
        if (!context.absolute_unit_factor)
            return fail("missing_absolute_unit_factor");
        const auto factor = *context.absolute_unit_factor;
        if (!std::isfinite(factor))
            return fail("nonfinite_absolute_unit_factor");
        if (context.mapping_mode == 6) {
            dimensions[0] /= factor;
            dimensions[1] = factor;
        } else if (context.mapping_mode == 5) {
            for (auto &d : dimensions)
                d /= factor;
        } else {
            dimensions.fill(factor);
        }
    }
    if (context.mapping_mode == 3) {
        auto extent_rotation = rotate(identity, Point3{radians[0], 0, 0});
        extent_rotation = rotate(extent_rotation, Point3{0, radians[1], 0});
        extent_rotation = rotate(extent_rotation, Point3{0, 0, radians[2]});
        dimensions = product(extent_rotation, dimensions);
        for (auto &d : dimensions)
            d = std::abs(d);
    }
    if (context.scale_mode == 0)
        for (unsigned i = 0; i < 3; ++i)
            offset[i] = dimensions[i] * offset[i];
    Point3 origin{};
    for (unsigned i = 0; i < 3; ++i)
        origin[i] = offset[i] + (*context.reference_point)[i];
    const auto orientation = product(*context.reference_matrix, rotation);
    if (!finite(dimensions) || !finite(offset) || !finite(origin) || !finite(orientation))
        return fail("nonfinite_preparation_result");
    out.update({{"status", "prepared"},
                {"source_object_id", getter.value("object_id", Json())},
                {"frame_source_object_id", getter.value("frame_source_object_id", Json())},
                {"projection_offset", offset},
                {"projection_scale", scale},
                {"projection_angles_degrees", degrees},
                {"orientation_matrix", orientation},
                {"reference_dimensions", dimensions},
                {"origin", origin},
                {"layer_data_flag_bit_2", (context.layer_data_flags >> 2) & 1u}});
    return out;
}
Json resolve_material_projection_transform(const Json &getter,
                                           const MaterialProjectionContext &context) {
    Json out = {{"scope", "native_projection_transform_mapping_modes_3_to_7"},
                {"status", "not_evaluated"},
                {"point_mapping", "not_evaluated"},
                {"preparation", prepare_material_projection(getter, context)}};
    auto fail = [&](const char *reason) {
        out["reason"] = reason;
        return out;
    };
    const auto &prepared = out.at("preparation");
    if (prepared.at("status") != "prepared")
        return fail("projection_preparation_unavailable");
    auto scale = prepared.at("projection_scale").get<Point3>();
    for (auto &x : scale)
        if (x == 0)
            x = 1;
    NativeMatrixInverse inverse;
    try {
        inverse = native_matrix_inverse(prepared.at("orientation_matrix").get<Matrix3>());
    } catch (const std::runtime_error &) {
        return fail("nonfinite_inverse_arithmetic");
    }
    auto computed = inverse.matrix;
    for (unsigned row = 0; row < 3; ++row) {
        const auto factor = 1 / scale[row];
        for (auto &x : computed[row])
            x *= factor;
    }
    if (!finite(computed))
        return fail("nonfinite_scaled_transform");
    out["computed_transform"] = {{"matrix", computed},
                                 {"normalized_projection_scale", scale},
                                 {"inverse_succeeded", inverse.inverted},
                                 {"inverse_method", inverse.method}};
    const auto &parameters = getter.at("parameters");
    const auto flag = parameters.value("origin_uv_pro_matrix_on", Json::object());
    if (!flag.is_object() || !flag.contains("value") || !flag.at("value").is_boolean())
        return fail("unavailable_explicit_matrix_switch");
    const bool explicit_matrix = flag.at("value").get<bool>();
    auto selected = computed;
    if (explicit_matrix) {
        const auto matrix = getter.value("matrix", Json::object());
        const auto storage = matrix.is_object() ? matrix.value("storage_values", Json()) : Json();
        if (!storage.is_array() || storage.size() != 9)
            return fail("unavailable_or_nonfinite_explicit_matrix");
        for (unsigned i = 0; i < 9; ++i) {
            if (!storage[i].is_number())
                return fail("unavailable_or_nonfinite_explicit_matrix");
            selected[i / 3][i % 3] = storage[i].get<double>();
        }
        if (!finite(selected))
            return fail("unavailable_or_nonfinite_explicit_matrix");
    }
    out.update({{"status", "resolved"},
                {"matrix", selected},
                {"matrix_source", explicit_matrix ? "local_explicit_matrix" : "computed_transform"},
                {"source_object_id", prepared.at("source_object_id")},
                {"frame_source_object_id", prepared.at("frame_source_object_id")},
                {"origin", prepared.at("origin")},
                {"reference_dimensions", prepared.at("reference_dimensions")},
                {"mapping_mode", context.mapping_mode},
                {"layer_data_flag_bit_2", prepared.at("layer_data_flag_bit_2")}});
    return out;
}
} // namespace p3d
