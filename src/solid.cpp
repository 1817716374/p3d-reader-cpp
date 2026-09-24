#include <p3d/solid.hpp>
#include "geometry.hpp"

namespace p3d {
SolidMeshResult mesh_bgfb_cone(const Json &, const PolyfaceMeshOptions &, unsigned);
SolidMeshResult mesh_bgfb_sphere(const Json &, const PolyfaceMeshOptions &, unsigned);
SolidMeshResult mesh_bgfb_solid(const Json &table, const PolyfaceMeshOptions &options,
                                unsigned circle_segments) {
    SolidMeshResult out;
    out.source = table;
    try {
        if (table.at("_type") == "DgnCone")
            return mesh_bgfb_cone(table, options, circle_segments);
        if (table.at("_type") == "DgnSphere")
            return mesh_bgfb_sphere(table, options, circle_segments);
        require(table.at("_type") == "DgnBox", "BGFB solid tessellation type not supported");
        const auto &detail = table.at("detail");
        auto number = [&](const char *name) {
            const double x = detail.at(name).get<double>();
            require(std::isfinite(x), "BGFB box nonfinite parameter");
            return x;
        };
        const Point3 base{number("baseOriginX"), number("baseOriginY"), number("baseOriginZ")};
        const Point3 top{number("topOriginX"), number("topOriginY"), number("topOriginZ")};
        const Point3 u{number("vectorXX"), number("vectorXY"), number("vectorXZ")};
        const Point3 v{number("vectorYX"), number("vectorYY"), number("vectorYZ")};
        const double bx = number("baseX"), by = number("baseY");
        const double tx = number("topX"), ty = number("topY");
        require(bx != 0 && tx != 0 && by != 0 && ty != 0 && (bx < 0) == (tx < 0) &&
                    (by < 0) == (ty < 0),
                "BGFB collapsed or crossing box not supported");
        auto frame = identity();
        for (unsigned k = 0; k < 3; ++k) {
            frame[k][0] = u[k] * bx;
            frame[k][1] = v[k] * by;
            frame[k][2] = top[k] - base[k];
        }
        const auto sign = determinant(frame);
        require(std::isfinite(sign) && sign != 0, "BGFB box degenerate or overflowing frame");
        const bool capped = detail.at("capped").get<bool>();
        Json points = Json::array(), indices = Json::array();
        // GeBoxInfo::getCorners uses binary U/V ordering at each end.
        for (unsigned end = 0; end < 2; ++end)
            for (unsigned corner = 0; corner < 4; ++corner)
                for (unsigned axis = 0; axis < 3; ++axis) {
                    const double p = (end ? top[axis] : base[axis]) +
                                     (corner & 1 ? u[axis] * (end ? tx : bx) : 0) +
                                     (corner & 2 ? v[axis] * (end ? ty : by) : 0);
                    require(std::isfinite(p), "BGFB box corner overflow");
                    points.push_back(p);
                }
        // GeBoxInfo::getFace table, with the same cap/side enumeration.
        constexpr unsigned faces[6][4] = {{1, 0, 2, 3}, {4, 5, 7, 6}, {0, 1, 5, 4},
                                          {1, 3, 7, 5}, {3, 2, 6, 7}, {2, 0, 4, 6}};
        for (unsigned f = capped ? 0 : 2; f < 6; ++f) {
            for (unsigned k = 0; k < 4; ++k)
                indices.push_back(faces[f][sign < 0 ? 3 - k : k] + 1);
            indices.push_back(0);
            out.face_indices.push_back(f < 2 ? std::array<std::int64_t, 3>{-1, f, 0}
                                             : std::array<std::int64_t, 3>{0, f - 2, 0});
        }
        Json polyface = {{"_type", "Polyface"},
                         {"meshStyle", 1},
                         {"numPerFace", 0},
                         {"point", std::move(points)},
                         {"pointIndex", std::move(indices)}};
        out.derived = mesh_bgfb_polyface(polyface, options);
        out.derived.report["source_kind"] = "DgnBox";
        out.derived.report["scope"] = "derived_planar_solid_faces";
        out.derived.report["native_tessellation_equivalence"] = "not_established";
        out.derived.report["reversed_box_frame"] = sign < 0;
        if (out.derived.status != "meshed")
            out.face_indices.clear();
    } catch (const std::exception &e) {
        out.derived = {};
        out.derived.report["reason"] = e.what();
        out.face_indices.clear();
    }
    return out;
}
} // namespace p3d
