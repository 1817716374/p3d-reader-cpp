#include <p3d/material_render_transform.hpp>
#include "projection_frame_math.hpp"
#include <algorithm>

namespace p3d {
namespace {
using projection_detail::f32;
double length(const Point3 &p) {
    return std::sqrt((p[1] * p[1] + p[0] * p[0]) + p[2] * p[2]);
}
float squared_float_length(const Point3 &p) {
    const float x = f32(float(p[0]) * float(p[0]));
    const float y = f32(float(p[1]) * float(p[1]));
    const float z = f32(float(p[2]) * float(p[2]));
    return f32(f32(x + y) + z);
}
float float_dot(const Point3 &a, const Point3 &b) {
    const float x = f32(float(a[0]) * float(b[0]));
    const float y = f32(float(a[1]) * float(b[1]));
    const float z = f32(float(a[2]) * float(b[2]));
    return f32(f32(y + x) + z);
}
} // namespace

Json derive_material_render_transform(const Matrix3 &c, const Point3 &translation) {
    Json out = {{"scope", "native_material_transform_initialization"}, {"status", "not_evaluated"}};
    try {
        Matrix3 r = c;
        Point3 t = translation;
        for (auto &row : r)
            for (auto &v : row)
                v = f32(v);
        for (auto &v : t)
            v = f32(v);

        // Cofactors are formed from C before rounding its entries to float.
        // Do not divide by the determinant: reflection signs and rank-two
        // transforms must retain the native behavior.
        Matrix3 cofactors{};
        for (unsigned i = 0; i < 3; ++i) {
            const unsigned j = (i + 1) % 3, k = (i + 2) % 3;
            for (unsigned a = 0; a < 3; ++a) {
                const unsigned b = (a + 1) % 3, d = (a + 2) % 3;
                cofactors[i][a] = c[j][b] * c[k][d] - c[j][d] * c[k][b];
            }
        }
        const double maximum_length =
            std::max(length(cofactors[2]), std::max(length(cofactors[0]), length(cofactors[1])));
        const double factor = maximum_length > 0 ? 1 / maximum_length : 1;
        Matrix3 n = cofactors;
        for (auto &row : n)
            for (auto &v : row)
                v = f32(v * factor);

        bool normalize = false;
        Point3 normal_row_alignment{};
        for (unsigned i = 0; i < 3; ++i) {
            auto unit = r[i];
            const double row_length = length(unit);
            if (row_length != 0) {
                const double inverse = 1 / row_length;
                for (auto &v : unit)
                    v = f32(v * inverse);
            }
            normal_row_alignment[i] = float_dot(unit, n[i]);
            if (normal_row_alignment[i] < 0.9999600052833557f)
                normalize = true;
        }
        Point3 squared{};
        for (unsigned i = 0; i < 3; ++i)
            squared[i] = squared_float_length(r[i]);
        const float first_two = std::max(float(squared[0]), float(squared[1]));
        const float all_three = std::max(float(squared[2]), first_two);
        const float mean = f32(f32(f32(squared[1] + squared[0]) + float(squared[2])) / 3.f);
        // Publish only after every required calculation succeeds.
        out.update({{"status", "computed"},
                    {"origin_basis_transform", c},
                    {"vertex_linear_transform", r},
                    {"vertex_translation", t},
                    {"normal_transform", n},
                    {"normalize_transformed_normal", normalize},
                    {"normal_row_alignment", normal_row_alignment},
                    {"row_squared_lengths", squared},
                    {"minimum_squared_scale", std::min({squared[0], squared[1], squared[2]})},
                    {"mean_squared_scale", mean},
                    {"geometry_scale_candidates",
                     {{"first_two_rows", f32(std::sqrt(first_two))},
                      {"all_three_rows", f32(std::sqrt(all_three))}}}});
    } catch (const std::exception &e) {
        out["reason"] = "invalid_render_transform_arithmetic";
        out["detail"] = e.what();
    }
    return out;
}
} // namespace p3d
