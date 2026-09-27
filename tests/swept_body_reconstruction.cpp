#include <p3d/swept_body.hpp>
#include <cmath>
#include <future>
#include <limits>
#include <stdexcept>

using namespace p3d;
namespace {
Json line(Point3 a, Point3 z) {
    Json s;
    for (unsigned i = 0; i < 3; ++i) {
        s[std::string("point0") + "XYZ"[i]] = a[i];
        s[std::string("point1") + "XYZ"[i]] = z[i];
    }
    return {{"_type", "LineSegment"}, {"segment", s}};
}
Json group(std::initializer_list<Json> values, unsigned type = 1) {
    Json entries = Json::array();
    for (const auto &v : values)
        entries.push_back({{"geometry", v}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", entries}};
}
Json rectangle(double lo = -1, double hi = 1) {
    return group({line({lo, lo, 0}, {hi, lo, 0}), line({hi, lo, 0}, {hi, hi, 0}),
                  line({hi, hi, 0}, {lo, hi, 0}), line({lo, hi, 0}, {lo, lo, 0})},
                 2);
}
Json body(const Json &profile, const Json &path, bool capped = true) {
    return {{"_type", "P3DSweptBody"}, {"profile", profile}, {"path", path}, {"capped", capped}};
}
} // namespace
unsigned swept_body_reconstruction_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *message) {
        ++n;
        if (!ok)
            throw std::runtime_error(message);
    };
    const auto path = group({line({0, 0, 0}, {0, 0, 4})});
    const auto source = body(rectangle(), path);
    auto result = reconstruct_bgfb_swept_body(source);
    check(result.status == "reconstructed" && result.source == source &&
              result.surfaces.size() == 4 && result.caps.size() == 2 && result.faces.size() == 6,
          "public swept reconstruction owns original source, four sides and two cap regions");
    check(result.groups.size() == 1 && result.groups[0].size() == 4 &&
              result.report["native_result"] == true &&
              result.report["mesh_status"] == "not_evaluated",
          "surface reconstruction exposes native grouping without claiming a mesh");
    for (const auto &f : result.faces) {
        check(result.find_face(f.indices) == &f,
              "native face lookup returns the exact published face");
        if (f.kind == SweptBodyFaceKind::side) {
            const auto &where = *f.location;
            check(result.groups.at(where[0]).at(where[1]).at(where[2]) == f.index,
                  "side query preserves group/member/patch association");
            const auto &s = result.surfaces.at(f.index);
            const auto geometry = BsplineSurface::from_bgfb(s.geometry);
            const auto converted = extract_swept_body_surface_face(s);
            check(converted.status == "extracted" && converted.region["type"] == 2 &&
                      converted.region["curves"].size() == 4 && !converted.surface,
                  "reconstructed prism side reaches native planar region conversion");
            check(std::abs(geometry.point_at(0, 0)[2]) < 1e-10 &&
                      std::abs(geometry.point_at(0, 1)[2] - 4) < 1e-10,
                  "public side table evaluates the original prism height");
        } else
            check(!f.location && result.caps.at(f.index).at("curves").size() == 4,
                  "cap query retains four separate boundary curves");
    }
    for (const auto &id :
         std::vector<std::array<std::int64_t, 3>>{{-1, 2, 0},
                                                  {0, 4, 0},
                                                  {0, -1, 0},
                                                  {1, 0, 0},
                                                  {0, 0, 1},
                                                  {0, std::numeric_limits<std::int64_t>::max(), 0}})
        check(result.find_face(id) == nullptr, "invalid or absent full face selector is rejected");
    auto uncapped = source;
    uncapped.erase("capped");
    auto open = reconstruct_bgfb_swept_body(uncapped);
    check(open.status == "reconstructed" && open.caps.empty() && open.faces.size() == 4 &&
              !open.find_face({-1, 0, 0}) && open.find_face({0, 0, 0}) == &open.faces.front(),
          "omitted native capped default is false; side numbering is independent of caps");
    auto multiring =
        reconstruct_bgfb_swept_body(body(group({rectangle(), rectangle(-.25, .25)}, 4), path));
    check(multiring.status == "reconstructed" && multiring.groups.size() == 2 &&
              multiring.surfaces.size() == 8 && multiring.faces.size() == 10 &&
              multiring.caps[0]["type"] == 4 &&
              multiring.caps[0]["curves"][1]["geometry"]["type"] == 3,
          "public parity sweep keeps ring order, all sides and native inner cap child");
    auto repeated =
        reconstruct_bgfb_swept_body(body(group({rectangle(), rectangle()}, 4), path, false));
    check(repeated.status == "reconstructed" && repeated.surfaces.size() == 8 &&
              repeated.groups[0][0][0] != repeated.groups[1][0][0],
          "equal independent source rings are not deduplicated");
    const auto bend = group({line({0, 0, 0}, {0, 0, 4}), line({0, 0, 4}, {2, 0, 4})});
    auto bent = reconstruct_bgfb_swept_body(body(rectangle(), bend, false));
    check(bent.status == "reconstructed" && bent.surfaces.size() == 8,
          "bent path reaches the public two-patch per member output");
    std::size_t trims = 0;
    for (const auto &s : bent.surfaces) {
        trims += s.boundary_points.size();
        check(s.boundary_curves.empty() || s.boundary_curves.size() == s.boundary_points.size(),
              "runtime trim curves remain associated with their active point records");
    }
    check(trims == 0, "linear miter adjusts surfaces without inventing runtime trims");
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
    auto curved = reconstruct_bgfb_swept_body(body(rectangle(), group({arc}), false));
    check(curved.status == "reconstructed" && curved.surfaces.size() == 4 &&
              curved.surfaces[0].geometry["orderV"] == 3 &&
              curved.surfaces[0].geometry["numPolesV"] == 5 &&
              !curved.surfaces[0].geometry["weights"].is_null(),
          "rational semicircle reconstruction exposes the combined native two-span surface");
    auto trimmed = reconstruct_bgfb_swept_body(
        body(rectangle(), group({arc, line({-20, 0, 0}, {-25, 0, 0})}), false));
    check(trimmed.status == "reconstructed" && trimmed.surfaces.size() == 8,
          "arc to straight corner constructs all final side surfaces");
    for (const auto &s : trimmed.surfaces) {
        check(s.geometry["boundaries"].is_null() && s.boundary_points.size() == 1 &&
                  s.boundary_points[0].size() >= 5 && s.boundary_curves.empty(),
              "runtime UV cache remains available separately from null BGFB and pcurve records");
        const auto boundary = extract_swept_body_surface_boundary(s);
        check(boundary.status == "extracted" && !boundary.curves.empty() &&
                  !boundary.report["spans"].empty() && boundary.report["outer"].empty(),
              "actual reconstructed curved corner retains a usable spatial trim boundary");
    }
    auto native_failed =
        reconstruct_bgfb_swept_body(body(group({line({-1, 0, 0}, {1, 0, 0})}), path));
    check(native_failed.status == "native_failure" && !native_failed.surfaces.empty() &&
              native_failed.caps.empty() && native_failed.faces.empty() &&
              !native_failed.find_face({0, 0, 0}),
          "open cap boundary retains completed sides but cannot masquerade as a completed body");
    for (unsigned failure = 0; failure < 5; ++failure) {
        auto invalid = source;
        if (failure == 0)
            invalid["_type"] = "other";
        if (failure == 1)
            invalid.erase("path");
        if (failure == 2)
            invalid["capped"] = "true";
        if (failure == 3)
            invalid["path"]["curves"][0]["geometry"]["_type"] = "UnknownCurve";
        if (failure == 4)
            invalid["profile"] = 7;
        auto failed = reconstruct_bgfb_swept_body(invalid);
        check(failed.status == "not_reconstructed" && failed.source == invalid &&
                  failed.surfaces.empty() && failed.faces.empty() &&
                  failed.report.contains("reason") && failed.report["native_result"].is_null(),
              "malformed or unsupported source is distinct from confirmed native rejection");
    }
    for (unsigned failure = 0; failure < 5; ++failure) {
        SweptBodyOptions options;
        if (failure == 0)
            options.max_work = 1;
        if (failure == 1)
            options.max_control_points = 1;
        if (failure == 2)
            options.max_faces = 5;
        if (failure == 3)
            options.max_faces = 0;
        if (failure == 4)
            options.max_work = result.report.at("work_used").get<std::size_t>() - 1;
        auto limited = reconstruct_bgfb_swept_body(source, options);
        check(limited.status == "not_reconstructed" && limited.surfaces.empty() &&
                  limited.groups.empty() && limited.faces.empty() && limited.caps.empty(),
              "resource failure never publishes partial derived output as native failure");
    }
    auto future =
        std::async(std::launch::async, [&] { return reconstruct_bgfb_swept_body(source); });
    auto other = future.get();
    check(other.status == result.status && other.report == result.report &&
              other.groups == result.groups && other.caps == result.caps &&
              other.surfaces[0].geometry == result.surfaces[0].geometry,
          "concurrent callers own independent deterministic reconstruction state");
    auto moved = std::move(other);
    check(moved.find_face({0, 0, 0}) && moved.source == source,
          "face lookup and original source survive result moves");
    return n;
}
