#include <p3d/material_elevation.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace p3d {
namespace {
constexpr const char *scope = "native_material_elevation_sampling";
double finite(double value) {
    if (!std::isfinite(value))
        throw std::range_error("nonfinite_elevation_arithmetic");
    return value;
}
float f32(double value) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
        throw std::range_error("nonfinite_or_unrepresentable_float_arithmetic");
    return static_cast<float>(value);
}
std::int32_t integer(const Json &value) {
    if (!value.is_number_integer() || value.get<double>() < -2147483648. ||
        value.get<double>() > 2147483647.)
        throw std::invalid_argument("invalid_elevation_integer");
    return value.get<std::int32_t>();
}
Point3 point3(const Json &value) {
    if (!value.is_array() || value.size() != 3)
        throw std::invalid_argument("invalid_elevation_vector");
    Point3 out{};
    for (unsigned i = 0; i < 3; ++i) {
        if (!value[i].is_number())
            throw std::invalid_argument("invalid_elevation_vector");
        out[i] = finite(value[i].get<double>());
    }
    return out;
}
Matrix2x3 matrix(const Json &value) {
    if (!value.is_array() || value.size() != 2)
        throw std::invalid_argument("invalid_elevation_matrix");
    return {point3(value[0]), point3(value[1])};
}
MaterialElevationFrame rounded_frame(MaterialElevationFrame frame) {
    for (auto &v : frame.origin)
        v = f32(v);
    for (auto &row : frame.axes)
        for (auto &v : row)
            v = f32(v);
    return frame;
}
Point2 project(const MaterialElevationFrame &source, const Point3 &point) {
    const auto frame = rounded_frame(source);
    std::array<float, 3> d{};
    for (unsigned i = 0; i < 3; ++i)
        d[i] = f32(f32(point[i]) - static_cast<float>(frame.origin[i]));
    Point2 out{};
    for (unsigned i = 0; i < 2; ++i) {
        const auto &a = frame.axes[i];
        const float x = f32(d[0] * static_cast<float>(a[0]));
        const float y = f32(d[1] * static_cast<float>(a[1]));
        const float z = f32(d[2] * static_cast<float>(a[2]));
        out[i] = f32(f32(y + x) + z);
    }
    return out;
}
} // namespace

Json prepare_material_elevation_sampling(const Json &transform,
                                         const MaterialElevationRenderContext &context) {
    Json out = {{"scope", scope}, {"status", "not_evaluated"}};
    auto fail = [&](const char *reason) {
        out["reason"] = reason;
        return out;
    };
    if (!transform.is_object() || !transform.contains("status") ||
        transform.at("status") != "computed" || !transform.contains("scope") ||
        transform.at("scope") != "native_layer_uv_affine_transform")
        return fail("uv_transform_unavailable");
    if (!context.geometry_kind)
        return fail("missing_render_geometry_kind");
    try {
        if (integer(transform.at("mapping_mode")) != 1)
            return fail("unsupported_elevation_mapping_mode");
        const auto scale_mode = integer(transform.at("scale_mode"));
        auto uv = matrix(transform.at("matrix"));
        const bool zero_kind = *context.geometry_kind == 0;
        if (zero_kind && !context.preserve_registered_offset)
            return fail("missing_registered_offset_branch");
        const bool preserve = zero_kind && *context.preserve_registered_offset;
        std::array<bool, 2> reduced{};
        if (!preserve) {
            if (!context.reference_xy || !context.texture_present)
                return fail("missing_elevation_reference_or_texture_state");
            if (*context.texture_present && !context.texture_axis_flags)
                return fail("missing_elevation_texture_axis_flags");
            const auto &p = *context.reference_xy;
            finite(p[0]);
            finite(p[1]);
            for (unsigned i = 0; i < 2; ++i) {
                const double offset = finite((p[0] * uv[i][0] + p[1] * uv[i][1]) + uv[i][2]);
                reduced[i] = !*context.texture_present || (*context.texture_axis_flags)[i] != 0;
                uv[i][2] = reduced[i] ? offset - std::floor(offset) : offset;
            }
        }
        Json frame;
        double inverse_squared_scale = 1;
        if (zero_kind) {
            if (!context.geometry_scale || !context.vertex_frame)
                return fail("missing_elevation_vertex_frame_or_scale");
            const float scale = f32(*context.geometry_scale);
            const float squared = f32(scale * scale);
            if (squared == 0)
                return fail("zero_elevation_squared_scale");
            inverse_squared_scale = f32(1.f / squared);
            for (auto &row : uv) {
                row[0] = finite(inverse_squared_scale * row[0]);
                row[1] = finite(inverse_squared_scale * row[1]);
            }
            const auto f = rounded_frame(*context.vertex_frame);
            frame = {{"origin", f.origin}, {"axes", f.axes}};
        }
        out.update({{"status", "prepared"},
                    {"mapping_mode", 1},
                    {"projection_flags", scale_mode == 0 ? 2 : 3},
                    {"geometry_kind", *context.geometry_kind},
                    {"uv_transform", uv},
                    {"registered_offset_preserved", preserve},
                    {"offset_reduced", reduced},
                    {"inverse_squared_scale", inverse_squared_scale}});
        if (zero_kind)
            out["vertex_frame"] = std::move(frame);
    } catch (const Json::exception &) {
        return fail("missing_or_invalid_elevation_transform_fields");
    } catch (const std::logic_error &e) {
        return fail(e.what());
    } catch (const std::range_error &e) {
        return fail(e.what());
    }
    return out;
}

Json sample_material_elevation(const Json &prepared, const Point3 &point,
                               const std::optional<MaterialElevationFrame> &face_frame) {
    Json out = {{"scope", "native_material_elevation_point"}, {"status", "not_evaluated"}};
    auto fail = [&](const char *reason) {
        out["reason"] = reason;
        return out;
    };
    if (!prepared.is_object() || !prepared.contains("status") ||
        prepared.at("status") != "prepared" || !prepared.contains("scope") ||
        prepared.at("scope") != scope)
        return fail("elevation_sampling_state_unavailable");
    try {
        const auto flags = integer(prepared.at("projection_flags"));
        if ((flags != 2 && flags != 3) || integer(prepared.at("mapping_mode")) != 1)
            return fail("invalid_elevation_projection_flags");
        const bool zero_kind = integer(prepared.at("geometry_kind")) == 0;
        const auto uv = matrix(prepared.at("uv_transform"));
        Point2 p{};
        const char *branch;
        if (face_frame) {
            p = project(*face_frame, point);
            branch = "per_face_frame";
        } else if (zero_kind) {
            const auto &frame = prepared.at("vertex_frame");
            p = project({point3(frame.at("origin")), matrix(frame.at("axes"))}, point);
            branch = "zero_geometry_kind";
        } else {
            p = {f32(point[0]), f32(point[1])};
            branch = "nonzero_geometry_kind";
        }
        // The zero-kind branch without a face frame adds the U products in
        // reverse order. Keep the native order, including signed-zero effects.
        const double ux = p[0] * uv[0][0], uy = p[1] * uv[0][1];
        const double u = zero_kind && !face_frame ? uy + ux : ux + uy;
        const Point2 result{f32(u + uv[0][2]), f32((p[0] * uv[1][0] + p[1] * uv[1][1]) + uv[1][2])};
        out.update(
            {{"status", "computed"}, {"branch", branch}, {"projection_uv", p}, {"uv", result}});
    } catch (const Json::exception &) {
        return fail("missing_or_invalid_elevation_sampling_fields");
    } catch (const std::logic_error &e) {
        return fail(e.what());
    } catch (const std::range_error &e) {
        return fail(e.what());
    }
    return out;
}
} // namespace p3d
