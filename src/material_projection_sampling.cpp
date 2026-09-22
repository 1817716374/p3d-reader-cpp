#include "internal.hpp"

namespace p3d {
namespace {
constexpr const char *sampling_scope = "native_material_projection_sampling_modes_3_to_7";
template <class T> bool finite_vector(const T &values) {
    return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
}
template <class T> bool finite_matrix(const T &matrix) {
    return std::all_of(matrix.begin(), matrix.end(),
                       [](const auto &row) { return finite_vector(row); });
}
// Explicit stores reproduce the double -> float -> double boundaries in the
// native vertex kernel. Reject unrepresentable values before a C++ float cast.
float f32(double value) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
        throw std::range_error("nonfinite_or_unrepresentable_float_arithmetic");
    return static_cast<float>(value);
}
Point3 read_point(const Json &value) {
    if (!value.is_array() || value.size() != 3)
        throw std::invalid_argument("invalid_projection_vector");
    Point3 out{};
    for (unsigned i = 0; i < 3; ++i) {
        if (!value[i].is_number())
            throw std::invalid_argument("invalid_projection_vector");
        out[i] = value[i].get<double>();
    }
    if (!finite_vector(out))
        throw std::invalid_argument("nonfinite_projection_vector");
    return out;
}
template <std::size_t N> std::array<Point3, N> read_matrix(const Json &value) {
    if (!value.is_array() || value.size() != N)
        throw std::invalid_argument("invalid_projection_matrix");
    std::array<Point3, N> out{};
    for (std::size_t i = 0; i < N; ++i)
        out[i] = read_point(value[i]);
    return out;
}
std::int32_t read_integer(const Json &value) {
    if (!value.is_number_integer() || value.get<double>() < -2147483648. ||
        value.get<double>() > 2147483647.)
        throw std::invalid_argument("invalid_projection_integer");
    return value.get<std::int32_t>();
}
Point3 transform_float(const Matrix3 &m, const Point3 &p) {
    Point3 out{};
    for (unsigned i = 0; i < 3; ++i)
        out[i] = f32((p[0] * m[i][0] + p[1] * m[i][1]) + p[2] * m[i][2]);
    return out;
}
float longitude(const Point3 &p) {
    // The normalization includes z even for cylindrical side mapping.
    const double length = std::sqrt((p[1] * p[1] + p[0] * p[0]) + p[2] * p[2]);
    const double factor = length == 0 ? 1 : 1 / length;
    const float angle = std::atan2(f32(p[1] * factor), f32(p[0] * factor));
    const float u = angle * 0.15915493667125702f;
    return u < 0 ? 1.f - std::abs(u) : u;
}
} // namespace

Json prepare_material_projection_sampling(const Json &resolved,
                                          const MaterialProjectionRenderContext &context) {
    Json out = {{"scope", sampling_scope}, {"status", "not_evaluated"}};
    auto fail = [&](const char *reason) {
        out["reason"] = reason;
        return out;
    };
    if (!resolved.is_object() || !resolved.contains("status") ||
        resolved.at("status") != "resolved")
        return fail("projection_transform_unavailable");
    if (!context.geometry_kind || !context.reference_point || !context.uv_transform)
        return fail("missing_render_projection_context");
    const bool zero_kind = *context.geometry_kind == 0;
    if (!finite_vector(*context.reference_point) || !finite_matrix(*context.uv_transform))
        return fail("nonfinite_render_projection_context");
    if (zero_kind) {
        if (!context.vertex_linear_transform || !context.vertex_translation ||
            !context.geometry_scale)
            return fail("missing_zero_kind_render_context");
    } else {
        if (!context.reference_transform)
            return fail("missing_render_projection_context");
        if (!finite_matrix(*context.reference_transform))
            return fail("nonfinite_render_projection_context");
        if ((*context.reference_transform)[3] != std::array<double, 4>{0, 0, 0, 1})
            return fail("reference_transform_must_be_affine");
    }
    try {
        const auto mode = read_integer(resolved.at("mapping_mode"));
        if (mode < 3 || mode > 7)
            return fail("unsupported_projection_mapping_mode");
        const bool absolute = read_integer(resolved.at("preparation").at("scale_mode")) != 0;
        int flags = mode == 3 ? 8 : mode == 5 ? 32 : mode == 6 ? 64 : 16;
        if (mode == 6) {
            const auto caps = read_integer(resolved.at("layer_data_flag_bit_2"));
            if (caps != 0 && caps != 1)
                return fail("invalid_cylindrical_cap_selector");
            flags <<= caps;
        }
        if (absolute)
            flags |= 1;
        auto matrix = read_matrix<3>(resolved.at("matrix"));
        auto origin = read_point(resolved.at("origin"));
        auto inverse = read_point(resolved.at("reference_dimensions"));
        Point3 render_origin{};
        const auto &p = *context.reference_point;
        Matrix3 vertex_linear{};
        Point3 vertex_translation{};
        if (zero_kind) {
            vertex_linear = *context.vertex_linear_transform;
            vertex_translation = *context.vertex_translation;
            Point3 relative{};
            for (unsigned i = 0; i < 3; ++i) {
                vertex_translation[i] = f32(vertex_translation[i]);
                relative[i] = origin[i] - p[i];
                for (auto &v : vertex_linear[i])
                    v = f32(v);
            }
            // Inputs are native floats promoted to double. These products and
            // sums remain double until the shared per-vertex kernel.
            for (unsigned i = 0; i < 3; ++i) {
                const auto &r = vertex_linear[i];
                const double x = r[0] * relative[0], y = r[1] * relative[1];
                origin[i] = ((i < 2 ? y + x : x + y) + r[2] * relative[2]) + vertex_translation[i];
            }
            const auto source_matrix = matrix;
            for (unsigned i = 0; i < 3; ++i)
                for (unsigned j = 0; j < 3; ++j) {
                    const auto &m = source_matrix[i];
                    const auto &r = vertex_linear[j];
                    const double x = r[0] * m[0], y = r[1] * m[1];
                    const bool y_first = (i == 0 && j < 2) || (i != 0 && j == 2);
                    matrix[i][j] = (y_first ? y + x : x + y) + r[2] * m[2];
                }
        } else {
            const auto &t = *context.reference_transform;
            for (unsigned i = 0; i < 3; ++i) {
                const auto linear = f32((p[1] * t[i][1] + p[0] * t[i][0]) + p[2] * t[i][2]);
                render_origin[i] = f32(double(linear) + double(f32(t[i][3])));
                origin[i] -= render_origin[i];
            }
        }
        for (auto &d : inverse)
            d = d == 0 ? 1 : 1 / d;
        auto uv = *context.uv_transform;
        Point2 center{.5, .5};
        double scale = 1;
        if (zero_kind || absolute) {
            if (!context.geometry_scale)
                return fail("missing_render_geometry_scale");
            scale = f32(*context.geometry_scale);
        }
        if (absolute) {
            if (zero_kind && scale == 0)
                return fail("zero_render_geometry_scale");
            for (unsigned i = 0; i < 2; ++i) {
                for (unsigned j = 0; j < 2; ++j)
                    uv[i][j] = zero_kind ? uv[i][j] / scale : uv[i][j] * scale;
                const double squared = uv[i][0] * uv[i][0] + uv[i][1] * uv[i][1];
                if (!std::isfinite(squared) || squared == 0)
                    return fail("unusable_absolute_uv_row_length");
                center[i] /= std::sqrt(squared);
            }
        }
        double inverse_squared_scale = 1;
        if (zero_kind && (!absolute || (flags & 0xe0))) {
            const float squared = f32(static_cast<float>(scale) * static_cast<float>(scale));
            if (squared == 0)
                return fail("zero_render_squared_scale");
            inverse_squared_scale = f32(1.f / squared);
            inverse[0] *= inverse_squared_scale;
            if (!absolute)
                inverse[1] *= inverse_squared_scale;
            inverse[2] *= inverse_squared_scale;
        }
        if (!finite_vector(origin) || !finite_vector(inverse) || !finite_vector(center) ||
            !finite_matrix(uv) || !finite_matrix(matrix))
            return fail("nonfinite_projection_sampling_state");
        out.update({{"status", "prepared"},
                    {"mapping_mode", mode},
                    {"projection_flags", flags},
                    {"geometry_kind", *context.geometry_kind},
                    {"matrix", matrix},
                    {"origin", origin},
                    {"inverse_reference_dimensions", inverse},
                    {"uv_transform", uv},
                    {"uv_center", center}});
        if (zero_kind) {
            out["vertex_linear_transform"] = vertex_linear;
            out["vertex_translation"] = vertex_translation;
            out["inverse_squared_scale"] = inverse_squared_scale;
        } else
            out["render_origin"] = render_origin;
    } catch (const Json::exception &) {
        return fail("missing_or_invalid_projection_transform_fields");
    } catch (const std::logic_error &e) {
        return fail(e.what());
    } catch (const std::range_error &e) {
        return fail(e.what());
    }
    return out;
}

Json sample_material_projection(const Json &prepared, const Point3 &point,
                                const std::optional<Point3> &normal) {
    Json out = {{"scope", "native_material_projection_point"}, {"status", "not_evaluated"}};
    auto fail = [&](const char *reason) {
        out["reason"] = reason;
        return out;
    };
    if (!prepared.is_object() || !prepared.contains("status") ||
        prepared.at("status") != "prepared" || !prepared.contains("scope") ||
        prepared.at("scope") != sampling_scope)
        return fail("projection_sampling_state_unavailable");
    try {
        const int flags = read_integer(prepared.at("projection_flags"));
        const int type = flags & ~1;
        if (type != 8 && type != 16 && type != 32 && type != 64 && type != 128)
            return fail("unsupported_projection_flags");
        const bool absolute = (flags & 1) != 0;
        const auto matrix = read_matrix<3>(prepared.at("matrix"));
        const auto origin = read_point(prepared.at("origin"));
        const auto inverse = read_point(prepared.at("inverse_reference_dimensions"));
        const auto uv = read_matrix<2>(prepared.at("uv_transform"));
        const auto &c = prepared.at("uv_center");
        if (!c.is_array() || c.size() != 2 || !c[0].is_number() || !c[1].is_number())
            return fail("invalid_projection_uv_center");
        const Point2 center{c[0].get<double>(), c[1].get<double>()};
        if (!finite_vector(center))
            return fail("nonfinite_projection_uv_center");
        Point3 shifted{}, n{};
        for (unsigned i = 0; i < 3; ++i)
            shifted[i] = f32(double(f32(point[i])) - origin[i]);
        const auto p = transform_float(matrix, shifted);
        if (type == 16 || type == 128) {
            if (!normal)
                return fail("missing_render_vertex_normal");
            for (unsigned i = 0; i < 3; ++i)
                n[i] = f32((*normal)[i]);
            // This is the same matrix as positions, not an inverse transpose.
            n = transform_float(matrix, n);
        }
        float u = 0, v = 0;
        const char *branch = "directional_drape";
        if (type == 8) {
            u = f32(p[0] * inverse[0] + center[0]);
            v = f32(p[1] * inverse[1] + center[1]);
        } else if (type == 16) {
            const auto x = std::abs(n[0]), y = std::abs(n[1]), z = std::abs(n[2]);
            if (x >= y && x >= z) {
                branch = n[0] < 0 ? "cubic_negative_x" : "cubic_positive_x";
                u = f32(p[1] * (n[0] < 0 ? -inverse[1] : inverse[1]) + center[0]);
                v = f32(p[2] * inverse[2] + center[1]);
            } else if (y > x && y >= z) {
                branch = n[1] < 0 ? "cubic_negative_y" : "cubic_positive_y";
                u = f32(p[0] * (n[1] < 0 ? inverse[0] : -inverse[0]) + center[0]);
                v = f32(p[2] * inverse[2] + center[1]);
            } else {
                branch = n[2] < 0 ? "cubic_negative_z" : "cubic_positive_z";
                u = f32(p[0] * (n[2] < 0 ? -inverse[0] : inverse[0]) + center[0]);
                v = f32(p[1] * inverse[1] + center[1]);
            }
        } else if (type == 32) {
            branch = "spherical";
            const double length = std::sqrt((p[1] * p[1] + p[0] * p[0]) + p[2] * p[2]);
            const float z = f32(length == 0 ? p[2] : p[2] * (1 / length));
            u = longitude(p);
            v = 1.f - std::acos(z) * 0.31830987334251404f;
            if (absolute) {
                u *= f32(3.141592653589793 / inverse[0]);
                v *= f32(1.5707963267948966 / inverse[0]);
            }
        } else if (type == 128 && std::abs(n[2]) > std::abs(n[0]) &&
                   std::abs(n[2]) > std::abs(n[1])) {
            branch = "cylindrical_cap";
            const double factor = inverse[absolute ? 1 : 0];
            u = f32(p[0] * factor + center[0]);
            v = f32(p[1] * factor + center[1]);
        } else {
            branch = "cylindrical_side";
            v = f32(p[2] * inverse[absolute ? 1 : 2] + center[1]);
            u = longitude(p);
            if (absolute)
                u *= f32(3.141592653589793 / inverse[0]);
        }
        if (!std::isfinite(u) || !std::isfinite(v))
            return fail("nonfinite_projection_coordinates");
        const Point2 result{f32((double(u) * uv[0][0] + double(v) * uv[0][1]) + uv[0][2]),
                            f32((double(u) * uv[1][0] + double(v) * uv[1][1]) + uv[1][2])};
        out.update({{"status", "computed"},
                    {"branch", branch},
                    {"projection_point", p},
                    {"projection_uv", Point2{u, v}},
                    {"uv", result}});
    } catch (const Json::exception &) {
        return fail("missing_or_invalid_projection_sampling_fields");
    } catch (const std::logic_error &e) {
        return fail(e.what());
    } catch (const std::range_error &e) {
        return fail(e.what());
    }
    return out;
}
} // namespace p3d
