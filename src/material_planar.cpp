#include <p3d/material_planar.hpp>
#include "projection_frame_math.hpp"

namespace p3d {
namespace {
constexpr const char *scope = "native_material_planar_sampling";
using projection_detail::f32;
double finite(double v) {
    if (!std::isfinite(v))
        throw std::range_error("nonfinite_planar_arithmetic");
    return v;
}
std::int32_t integer(const Json &v) {
    if (!v.is_number_integer() || v.get<double>() < -2147483648. || v.get<double>() > 2147483647.)
        throw std::invalid_argument("invalid_planar_integer");
    return v.get<std::int32_t>();
}
template <std::size_t N> std::array<double, N> vector(const Json &v) {
    if (!v.is_array() || v.size() != N)
        throw std::invalid_argument("invalid_planar_vector");
    std::array<double, N> out{};
    for (unsigned i = 0; i < N; ++i) {
        if (!v[i].is_number())
            throw std::invalid_argument("invalid_planar_vector");
        out[i] = finite(v[i].get<double>());
    }
    return out;
}
template <std::size_t N> std::array<Point3, N> matrix(const Json &v) {
    if (!v.is_array() || v.size() != N)
        throw std::invalid_argument("invalid_planar_matrix");
    std::array<Point3, N> out{};
    for (unsigned i = 0; i < N; ++i)
        out[i] = vector<3>(v[i]);
    return out;
}
Point3 rounded(Point3 p) {
    for (auto &v : p)
        v = f32(v);
    return p;
}
Matrix3 rounded(Matrix3 m) {
    for (auto &r : m)
        r = rounded(r);
    return m;
}
Point3 normalized(Point3 p) {
    const double length = std::sqrt((p[1] * p[1] + p[0] * p[0]) + p[2] * p[2]);
    if (length != 0) {
        const double inverse = 1 / length;
        for (auto &v : p)
            v = f32(v * inverse);
    }
    return p;
}
float dot_float(const Point3 &a, const Point3 &b, bool y_first = false) {
    const auto x = f32(static_cast<float>(a[0]) * static_cast<float>(b[0]));
    const auto y = f32(static_cast<float>(a[1]) * static_cast<float>(b[1]));
    const auto z = f32(static_cast<float>(a[2]) * static_cast<float>(b[2]));
    return f32(f32(y_first ? y + x : x + y) + z);
}
Point3 product_float(const Matrix3 &m, const Point3 &p) {
    return {dot_float(m[0], p), dot_float(m[1], p), dot_float(m[2], p)};
}
Point3 cross_float(const Point3 &a, const Point3 &b) {
    Point3 out{};
    for (unsigned i = 0; i < 3; ++i) {
        const unsigned j = (i + 1) % 3, k = (i + 2) % 3;
        out[i] = f32(f32(static_cast<float>(a[j]) * static_cast<float>(b[k])) -
                     f32(static_cast<float>(a[k]) * static_cast<float>(b[j])));
    }
    return out;
}
} // namespace

Json prepare_material_planar_sampling(const Json &transform,
                                      const MaterialPlanarRenderContext &context) {
    Json out = {{"scope", scope}, {"status", "not_evaluated"}};
    auto fail = [&](const char *reason) {
        out["reason"] = reason;
        return out;
    };
    if (!transform.is_object() || !transform.contains("status") ||
        transform.at("status") != "computed" || !transform.contains("scope") ||
        transform.at("scope") != "native_layer_uv_affine_transform")
        return fail("uv_transform_unavailable");
    if (!context.enabled)
        return fail("unknown_planar_enabled_branch");
    try {
        if (integer(transform.at("mapping_mode")) != 2)
            return fail("unsupported_planar_mapping_mode");
        if (!*context.enabled) {
            auto effective = transform;
            effective["mapping_mode"] = 0;
            auto parametric = prepare_material_parametric_sampling(effective, context.parametric);
            if (parametric.at("status") != "prepared") {
                out["parametric_preparation"] = parametric;
                return fail("parametric_preparation_unavailable");
            }
            out.update({{"status", "prepared"},
                        {"mapping_mode", 2},
                        {"enabled", false},
                        {"parametric_state", std::move(parametric)}});
            return out;
        }
        const int flags = integer(transform.at("scale_mode")) == 0 ? 4 : 5;
        const auto uv = matrix<2>(transform.at("matrix"));
        if (!context.geometry_kind || !context.vertex_linear_transform ||
            !context.origin_basis_transform || !context.reference_translation)
            return fail("missing_planar_render_context");
        const bool nonzero = *context.geometry_kind != 0;
        if (nonzero && (!context.reference_linear_transform || !context.vertex_translation ||
                        !context.normal_transform || !context.normalize_transformed_normal))
            return fail("missing_nonzero_kind_planar_context");
        const auto r = rounded(*context.vertex_linear_transform);
        Point3 reference = *context.reference_translation;
        for (auto &v : reference)
            finite(v);
        if (nonzero) {
            const auto t = rounded(*context.vertex_translation);
            const auto &a = *context.reference_linear_transform;
            for (unsigned i = 0; i < 3; ++i) {
                for (const auto v : a[i])
                    finite(v);
                const auto linear = f32((a[i][0] * t[0] + a[i][1] * t[1]) + a[i][2] * t[2]);
                reference[i] = finite(double(linear) + reference[i]);
            }
        }
        Point3 origin{};
        for (unsigned i = 0; i < 3; ++i) {
            const auto &c = (*context.origin_basis_transform)[i];
            for (auto v : c)
                finite(v);
            origin[i] = finite((reference[1] * c[1] + reference[0] * c[0]) + reference[2] * c[2]);
        }
        const auto fallback = normalized({r[0][0], r[1][0], r[2][0]});
        const Point3 reference_axis{r[0][2], r[1][2], r[2][2]};
        Json normal_matrix;
        if (nonzero)
            normal_matrix = rounded(*context.normal_transform);
        out.update({{"status", "prepared"},
                    {"mapping_mode", 2},
                    {"enabled", true},
                    {"projection_flags", flags},
                    {"geometry_kind", *context.geometry_kind},
                    {"uv_transform", uv},
                    {"initial_offset", Point2{uv[0][2], uv[1][2]}},
                    {"origin", origin},
                    {"fallback_u_axis", fallback},
                    {"reference_axis", reference_axis},
                    {"vertex_linear_transform", r}});
        if (nonzero) {
            out["normal_transform"] = std::move(normal_matrix);
            out["normalize_transformed_normal"] = *context.normalize_transformed_normal;
        }
    } catch (const Json::exception &) {
        return fail("missing_or_invalid_planar_transform_fields");
    } catch (const std::logic_error &e) {
        return fail(e.what());
    } catch (const std::range_error &e) {
        return fail(e.what());
    }
    return out;
}

Json sample_material_planar(const Json &state, const Point3 &point,
                            const std::optional<Point3> &normal,
                            const std::optional<Point2> &native_uv,
                            const std::optional<MaterialParametricFrame> &face_frame) {
    Json out = {{"scope", "native_material_planar_point"}, {"status", "not_evaluated"}};
    auto fail = [&](const char *reason) {
        out["reason"] = reason;
        return out;
    };
    if (!state.is_object() || !state.contains("status") || state.at("status") != "prepared" ||
        !state.contains("scope") || state.at("scope") != scope)
        return fail("planar_sampling_state_unavailable");
    try {
        if (integer(state.at("mapping_mode")) != 2 || !state.at("enabled").is_boolean())
            return fail("invalid_planar_sampling_branch");
        if (!state.at("enabled").get<bool>()) {
            auto value = sample_material_parametric(state.at("parametric_state"), point, native_uv,
                                                    face_frame);
            if (value.at("status") != "computed") {
                out["parametric_sampling"] = value;
                return fail("parametric_sampling_unavailable");
            }
            out.update({{"status", "computed"},
                        {"branch", "planar_disabled"},
                        {"parametric_branch", value.at("branch")},
                        {"uv", value.at("uv")},
                        {"projection_uv", value.at("projection_uv")},
                        {"next_state", state}});
            return out;
        }
        const int flags = integer(state.at("projection_flags"));
        if (flags != 4 && flags != 5)
            return fail("invalid_planar_projection_flags");
        auto uv = matrix<2>(state.at("uv_transform"));
        Point2 projected{};
        Json next = state;
        const char *branch;
        if (face_frame) {
            projected = projection_detail::project(*face_frame, point);
            branch = "per_face_frame";
        } else {
            if (!normal)
                return fail("missing_render_vertex_normal");
            const bool nonzero = integer(state.at("geometry_kind")) != 0;
            auto n = rounded(*normal), p = rounded(point);
            if (nonzero) {
                n = product_float(rounded(matrix<3>(state.at("normal_transform"))), n);
                if (!state.at("normalize_transformed_normal").is_boolean())
                    return fail("invalid_normal_normalization_branch");
                if (state.at("normalize_transformed_normal").get<bool>())
                    n = normalized(n);
                p = product_float(rounded(matrix<3>(state.at("vertex_linear_transform"))), p);
            }
            const auto z = rounded(vector<3>(state.at("reference_axis")));
            auto u = cross_float(z, n);
            const float squared = dot_float(u, u, true);
            const bool fallback = squared < 9.999999960041972e-13f;
            u = fallback ? rounded(vector<3>(state.at("fallback_u_axis"))) : normalized(u);
            // The zero-kind fallback changes the second cross-product input
            // to the reference axis; the nonzero-kind branch retains normal.
            const auto v = normalized(cross_float(!nonzero && fallback ? z : n, u));
            projected = {dot_float(p, u, nonzero), dot_float(p, v, nonzero)};
            const auto origin = vector<3>(state.at("origin"));
            const auto initial = vector<2>(state.at("initial_offset"));
            const double ox = finite((u[0] * origin[0] + u[1] * origin[1]) + u[2] * origin[2]);
            const double oy = finite((v[0] * origin[0] + v[1] * origin[1]) + v[2] * origin[2]);
            const Point2 offsets{finite((uv[0][0] * ox + uv[0][1] * oy) + initial[0]),
                                 finite((uv[1][1] * oy + uv[1][0] * ox) + initial[1])};
            for (unsigned i = 0; i < 2; ++i)
                uv[i][2] = offsets[i] - std::floor(offsets[i]);
            next["uv_transform"] = uv;
            out["basis_u"] = u;
            out["basis_v"] = v;
            out["fallback_axis_used"] = fallback;
            branch = nonzero ? "nonzero_geometry_kind" : "zero_geometry_kind";
        }
        const Point2 result{f32((projected[0] * uv[0][0] + projected[1] * uv[0][1]) + uv[0][2]),
                            f32((projected[0] * uv[1][0] + projected[1] * uv[1][1]) + uv[1][2])};
        out.update({{"status", "computed"},
                    {"branch", branch},
                    {"projection_uv", projected},
                    {"uv", result},
                    {"next_state", std::move(next)}});
    } catch (const Json::exception &) {
        return fail("missing_or_invalid_planar_sampling_fields");
    } catch (const std::logic_error &e) {
        return fail(e.what());
    } catch (const std::range_error &e) {
        return fail(e.what());
    }
    return out;
}
} // namespace p3d
