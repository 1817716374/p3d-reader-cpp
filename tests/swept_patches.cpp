#include <p3d/swept_patches.hpp>
#include <future>
#include <stdexcept>
using namespace p3d;
namespace {
Json line(Point3 a, Point3 b) {
    Json p;
    for (unsigned i = 0; i < 3; ++i) {
        p[std::string("point0") + "XYZ"[i]] = a[i];
        p[std::string("point1") + "XYZ"[i]] = b[i];
    }
    return {{"_type", "LineSegment"}, {"segment", p}};
}
Json group(Json values, unsigned type = 1) {
    Json members = Json::array();
    for (const auto &v : values)
        members.push_back({{"geometry", v}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", members}};
}
std::vector<Point3> vertices(double size, bool reverse) {
    std::vector<Point3> p{{-size, -size, 0}, {size, -size, 0}, {size, size, 0}, {-size, size, 0}};
    if (reverse)
        std::reverse(p.begin(), p.end());
    return p;
}
Json square(double size, bool reverse = false) {
    auto p = vertices(size, reverse);
    Json lines = Json::array();
    for (unsigned i = 0; i < 4; ++i)
        lines.push_back(line(p[i], p[(i + 1) % 4]));
    return group(lines, 2);
}
bool near(double a, double b) {
    return std::abs(a - b) < 2e-10;
}
} // namespace
unsigned swept_patches_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        if (!ok)
            throw std::runtime_error(why);
    };
    const auto path = group({line({0, 0, 0}, {0, 0, 4})});
    const auto body = [&](const Json &profile, const Json &trace) {
        return Json{{"_type", "P3DSweptBody"}, {"profile", profile}, {"path", trace}};
    };
    for (bool reverse : {false, true}) {
        const auto table = body(square(1, reverse), path);
        const auto saved = table;
        const auto result = reconstruct_bgfb_swept_body_patches(table);
        check(result.status == "reconstructed" && result.groups.size() == 1 &&
                  result.groups[0].source_curve == 0 && result.groups[0].patches.size() == 1,
              "whole closed rectangle creates one patch, not four primitive patches");
        const auto &patch = result.groups[0].patches[0];
        const auto surface = BsplineSurface::from_bgfb(patch.geometry);
        check(surface.u().pole_count() == 5 && patch.boundary_points.empty(),
              "whole section keeps its closed polygon traversal in one U curve");
        auto p = vertices(1, reverse);
        for (unsigned j = 0; j < 4; ++j)
            for (double t : {0., .25, .7})
                for (double v : {0., .4, 1.}) {
                    const auto a = p[reverse ? (4 - j) % 4 : j],
                               z = p[reverse ? (3 - j) % 4 : (j + 1) % 4];
                    const auto q = surface.point_at((j + t) / 4., v);
                    check(near(q[0], (1 - t) * a[0] + t * z[0]) &&
                              near(q[1], (1 - t) * a[1] + t * z[1]) && near(q[2], 4 * v),
                          "whole-ring geometry agrees with independent prism boundary formula");
                }
        check(table == saved && result.source == saved,
              "patch reconstruction retains immutable source");
        auto capped = table;
        capped["capped"] = true;
        const auto cap_result = reconstruct_bgfb_swept_body_patches(capped);
        check(cap_result.status == "reconstructed" && cap_result.groups.size() == 1 &&
                  cap_result.groups[0].patches[0].geometry == patch.geometry,
              "capped does not invent caps in getPatches stage");
    }
    const auto parity = body(group({square(3), square(1), square(1)}, 4), path);
    const auto rings = reconstruct_bgfb_swept_body_patches(parity);
    check(rings.status == "reconstructed" && rings.groups.size() == 3,
          "multiple rings retain source ordering and duplicate ring occurrences");
    check(rings.groups[0].source_curve == 0 && rings.groups[1].source_curve == 1 &&
              rings.groups[2].source_curve == 2 &&
              rings.groups[1].patches[0].geometry == rings.groups[2].patches[0].geometry,
          "equal independently generated native surfaces are retained separately");
    const auto bent = group({line({0, 0, 0}, {0, 0, 4}), line({0, 0, 4}, {2, 0, 4})});
    const auto corner = reconstruct_bgfb_swept_body_patches(body(square(1), bent));
    check(corner.status == "reconstructed" && corner.groups.size() == 1 &&
              corner.groups[0].patches.size() == 2,
          "bent path has two whole-section patches in traversal order");
    const auto a = BsplineSurface::from_bgfb(corner.groups[0].patches[0].geometry),
               b = BsplineSurface::from_bgfb(corner.groups[0].patches[1].geometry);
    for (double u : {0., .2, .4, .5, .7, .9, 1.}) {
        const auto p = a.point_at(u, 1), q = b.point_at(u, 0);
        check(near(p[0], q[0]) && near(p[1], q[1]) && near(p[2], q[2]),
              "whole-ring patches agree at the native projected corner seam");
    }
    SweptBodyPatchOptions limit;
    const Json arc{{"_type", "EllipticArc"},
                   {"arc",
                    {{"centerX", -10},
                     {"centerY", 0},
                     {"centerZ", 0},
                     {"vector0X", 10},
                     {"vector0Y", 0},
                     {"vector0Z", 0},
                     {"vector90X", 0},
                     {"vector90Y", 0},
                     {"vector90Z", 10},
                     {"startRadians", 0},
                     {"sweepRadians", 3.141592653589793}}}};
    const auto trimmed = reconstruct_bgfb_swept_body_patches(
        body(square(1), group({arc, line({-20, 0, 0}, {-25, 0, 0})})));
    check(trimmed.status == "reconstructed" && trimmed.groups.size() == 1 &&
              trimmed.groups[0].patches.size() > 1,
          "whole-section curved corner reaches the actual patch pipeline");
    std::size_t trim_count = 0;
    for (const auto &patch : trimmed.groups[0].patches) {
        trim_count += patch.boundary_points.size();
        check(patch.geometry.at("boundaries").is_null(),
              "runtime patch trim is separate from BGFB serialized boundaries");
        BsplineMeshOptions options;
        options.max_uv_edge = .25;
        const auto mesh = BsplineSurface::from_bgfb(patch.geometry)
                              .mesh_runtime_boundaries(patch.boundary_points, options);
        check(mesh.report["status"] == "complete" && !mesh.faces.empty(),
              "generated whole-section patch and runtime trims can be consumed together");
    }
    check(trim_count > 0, "curved corner preserves native runtime trim records");
    limit.max_patches = 1;
    const auto limited = reconstruct_bgfb_swept_body_patches(parity, limit);
    check(limited.status == "not_reconstructed" && limited.groups.empty() &&
              limited.report["native_result"].is_null(),
          "public count limit clears partial output and does not claim native rejection");
    limit = {};
    limit.max_work = 1;
    check(reconstruct_bgfb_swept_body_patches(parity, limit).status == "not_reconstructed",
          "whole-section reconstruction obeys work limit");
    limit = {};
    limit.max_control_points = 1;
    check(reconstruct_bgfb_swept_body_patches(parity, limit).status == "not_reconstructed",
          "whole-section reconstruction obeys control limit");
    const auto rejected = reconstruct_bgfb_swept_body_patches(body(square(1), nullptr));
    check(rejected.status == "native_failure" && rejected.groups.empty() &&
              rejected.report["native_result"] == false,
          "source validation rejection remains distinct from unsupported data");
    check(reconstruct_bgfb_swept_body_patches(Json::object()).status == "not_reconstructed",
          "wrong source type returns explicit unsupported result");
    auto task =
        std::async(std::launch::async, [&] { return reconstruct_bgfb_swept_body_patches(parity); });
    const auto concurrent = reconstruct_bgfb_swept_body_patches(parity);
    const auto other = task.get();
    check(other.report == concurrent.report &&
              other.groups[2].patches[0].geometry == concurrent.groups[2].patches[0].geometry,
          "parallel calls share no mutable path state");
    return n;
}
