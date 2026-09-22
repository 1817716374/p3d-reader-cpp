#include <p3d/material_parametric.hpp>
#include "projection_frame_math.hpp"

namespace p3d {
namespace {
constexpr const char *scope = "native_material_parametric_sampling";
using projection_detail::f32;
double finite(double v) {
    if (!std::isfinite(v))
        throw std::range_error("nonfinite_parametric_arithmetic");
    return v;
}
std::int32_t integer(const Json &v) {
    if (!v.is_number_integer() || v.get<double>() < -2147483648. || v.get<double>() > 2147483647.)
        throw std::invalid_argument("invalid_parametric_integer");
    return v.get<std::int32_t>();
}
Point3 point3(const Json &value) {
    if (!value.is_array() || value.size() != 3)
        throw std::invalid_argument("invalid_parametric_vector");
    Point3 out{};
    for (unsigned i = 0; i < 3; ++i) {
        if (!value[i].is_number())
            throw std::invalid_argument("invalid_parametric_vector");
        out[i] = finite(value[i].get<double>());
    }
    return out;
}
Matrix2x3 matrix(const Json &v) {
    if (!v.is_array() || v.size() != 2)
        throw std::invalid_argument("invalid_parametric_matrix");
    return {point3(v[0]), point3(v[1])};
}
} // namespace

Json prepare_material_parametric_sampling(const Json &transform,
                                          const MaterialParametricRenderContext &context) {
    Json out = {{"scope", scope}, {"status", "not_evaluated"}};
    auto fail = [&](const char *reason) {
        out["reason"] = reason;
        return out;
    };
    if (!transform.is_object() || !transform.contains("status") ||
        transform.at("status") != "computed" || !transform.contains("scope") ||
        transform.at("scope") != "native_layer_uv_affine_transform")
        return fail("uv_transform_unavailable");
    try {
        if (integer(transform.at("mapping_mode")) != 0)
            return fail("unsupported_parametric_mapping_mode");
        const bool absolute = integer(transform.at("scale_mode")) != 0;
        auto uv = matrix(transform.at("matrix"));
        Json frame_override;
        Point2 factors{1, 1};
        const char *branch = "relative";
        if (absolute) {
            if (!context.preparation_frame_present || !context.geometry_scale)
                return fail("missing_parametric_preparation_context");
            const float scale = f32(*context.geometry_scale);
            if (*context.preparation_frame_present) {
                if (!context.preparation_frame)
                    return fail("missing_parametric_preparation_frame");
                auto frame = projection_detail::rounded_frame(*context.preparation_frame);
                for (auto &axis : frame.axes) {
                    const double length =
                        std::sqrt((axis[1] * axis[1] + axis[0] * axis[0]) + axis[2] * axis[2]);
                    if (length != 0) {
                        const double inverse = 1 / length;
                        for (auto &v : axis)
                            v = f32(v * inverse);
                    }
                    for (auto &v : axis)
                        v = f32(static_cast<float>(v) * scale);
                }
                frame_override = {{"origin", frame.origin}, {"axes", frame.axes}};
                branch = "normalized_preparation_frame";
            } else {
                if (!context.use_parameter_factors)
                    return fail("missing_parameter_factor_branch");
                if (*context.use_parameter_factors && !context.parameter_factors)
                    return fail("missing_parameter_factors");
                for (unsigned i = 0; i < 2; ++i) {
                    const float factor =
                        *context.use_parameter_factors ? f32((*context.parameter_factors)[i]) : 1.f;
                    factors[i] = f32(factor * scale);
                    for (auto &row : uv)
                        row[i] = finite(factors[i] * row[i]);
                }
                branch = *context.use_parameter_factors ? "parameter_factors" : "geometry_scale";
            }
        }
        out.update({{"status", "prepared"},
                    {"mapping_mode", 0},
                    {"projection_flags", absolute ? 1 : 0},
                    {"uv_transform", uv},
                    {"frame_override", frame_override},
                    {"applied_parameter_factors", factors},
                    {"preparation_branch", branch}});
    } catch (const Json::exception &) {
        return fail("missing_or_invalid_parametric_transform_fields");
    } catch (const std::logic_error &e) {
        return fail(e.what());
    } catch (const std::range_error &e) {
        return fail(e.what());
    }
    return out;
}

Json sample_material_parametric(const Json &prepared, const Point3 &point,
                                const std::optional<Point2> &native_uv,
                                const std::optional<MaterialParametricFrame> &face_frame) {
    Json out = {{"scope", "native_material_parametric_point"}, {"status", "not_evaluated"}};
    auto fail = [&](const char *reason) {
        out["reason"] = reason;
        return out;
    };
    if (!prepared.is_object() || !prepared.contains("status") ||
        prepared.at("status") != "prepared" || !prepared.contains("scope") ||
        prepared.at("scope") != scope)
        return fail("parametric_sampling_state_unavailable");
    try {
        const auto flags = integer(prepared.at("projection_flags"));
        if ((flags != 0 && flags != 1) || integer(prepared.at("mapping_mode")) != 0)
            return fail("invalid_parametric_projection_flags");
        const auto uv = matrix(prepared.at("uv_transform"));
        const auto &frame = prepared.at("frame_override");
        if (flags == 0 && !frame.is_null())
            return fail("unexpected_relative_frame_override");
        Point2 p{};
        const char *branch;
        if (face_frame) {
            if (frame.is_null()) {
                p = projection_detail::project(*face_frame, point);
                branch = "per_face_frame";
            } else {
                p = projection_detail::project(
                    {point3(frame.at("origin")), matrix(frame.at("axes"))}, point);
                branch = "prepared_frame";
            }
        } else if (native_uv) {
            p = {f32((*native_uv)[0]), f32((*native_uv)[1])};
            branch = "native_uv";
        } else {
            p = {f32(point[0]), f32(point[1])};
            branch = "vertex_xy";
        }
        const double ux = p[0] * uv[0][0], uy = p[1] * uv[0][1];
        const double vx = p[0] * uv[1][0], vy = p[1] * uv[1][1];
        const double u = !face_frame && !native_uv ? uy + ux : ux + uy;
        const double v = !face_frame && native_uv ? vy + vx : vx + vy;
        const Point2 result{f32(u + uv[0][2]), f32(v + uv[1][2])};
        out.update(
            {{"status", "computed"}, {"branch", branch}, {"projection_uv", p}, {"uv", result}});
    } catch (const Json::exception &) {
        return fail("missing_or_invalid_parametric_sampling_fields");
    } catch (const std::logic_error &e) {
        return fail(e.what());
    } catch (const std::range_error &e) {
        return fail(e.what());
    }
    return out;
}
} // namespace p3d
