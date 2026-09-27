#include <p3d/swept_mesh.hpp>
#include <p3d/solid.hpp>
#include "native_tube_mesh_source.hpp"
#include <future>
using namespace p3d;
namespace {
Json polyline(const std::vector<Point3> &points) {
    Json p = Json::array();
    for (const auto &v : points)
        for (auto x : v)
            p.push_back(x);
    return {{"_type", "LineString"}, {"points", p}};
}
Json group(Json values, unsigned type = 1) {
    Json members = Json::array();
    for (const auto &v : values)
        members.push_back({{"geometry", v}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", members}};
}
Json square(double size) {
    return group({polyline({{-size, -size, 0},
                            {size, -size, 0},
                            {size, size, 0},
                            {-size, size, 0},
                            {-size, -size, 0}})},
                 2);
}
double volume(const SweptBodyMeshResult &m) {
    double sum = 0;
    for (std::size_t i = 0; i < m.point_indices.size(); i += 4) {
        const auto &a = m.points.at(std::abs(m.point_indices[i]) - 1);
        const auto &b = m.points.at(std::abs(m.point_indices[i + 1]) - 1);
        const auto &c = m.points.at(std::abs(m.point_indices[i + 2]) - 1);
        sum += a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
               a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    return sum / 6;
}
} // namespace
unsigned swept_mesh_tests() {
    unsigned checks = 0;
    auto check = [&](bool b, const char *why) {
        ++checks;
        require(b, why);
    };
    const auto path = group({polyline({{0, 0, 0}, {0, 0, 4}})});
    const Json body{
        {"_type", "P3DSweptBody"}, {"profile", square(1)}, {"path", path}, {"capped", true}};
    swept_detail::TubeBudget budget;
    auto flags = swept_detail::tube_mesh_source_conditions(body["profile"], path, true, budget);
    check(flags.accepted && flags.profile_closed && !flags.path_closed && flags.cap_eligible &&
              flags.report["path_planarity"]["planar"] == true,
          "native straight path cap conditions");
    auto loop = group({polyline({{0, 0, 0}, {3, 0, 0}, {3, 3, 0}, {0, 3, 0}, {0, 0, 0}})});
    auto closed = swept_detail::tube_mesh_source_conditions(square(1), loop, true, budget);
    check(closed.path_closed && !closed.cap_eligible &&
              closed.report["path_closure_evaluated"] == true,
          "planar closed path suppresses caps");
    auto spatial_loop = group({polyline({{0, 0, 0}, {3, 0, 0}, {3, 3, 2}, {0, 3, 0}, {0, 0, 0}})});
    auto spatial = swept_detail::tube_mesh_source_conditions(square(1), spatial_loop, true, budget);
    check(spatial.accepted && !spatial.path_closed && spatial.cap_eligible &&
              spatial.report["path_planarity"]["planar"] == false &&
              spatial.report["path_closure_evaluated"] == false,
          "nonplanar source loop keeps native forced-open mesh-path flag");
    const auto open = group({polyline({{0, 0, 0}, {1, 0, 0}})});
    auto parity = group({square(1), open}, 4);
    auto rejected = swept_detail::tube_mesh_source_conditions(parity, path, false, budget);
    check(!rejected.accepted && rejected.report["failure"] == "open_parity_profile_child",
          "open parity child rejects mesh entry even when no caps requested");
    auto not_closed = swept_detail::tube_mesh_source_conditions(open, path, true, budget);
    check(not_closed.accepted && !not_closed.profile_closed && !not_closed.cap_eligible,
          "ordinary open profile accepted without caps");
    auto empty = group(Json::array());
    auto no_path = swept_detail::tube_mesh_source_conditions(square(1), empty, true, budget);
    check(no_path.accepted && no_path.cap_eligible && !no_path.path_closed,
          "native frame failure follows false-planarity branch before getPatches rejection");
    auto nullable = body["profile"];
    nullable["curves"].insert(nullable["curves"].begin(), nullptr);
    nullable["curves"].push_back({{"geometry", nullptr}});
    check(swept_detail::tube_mesh_source_conditions(nullable, path, true, budget).profile_closed,
          "null ordinary members do not replace endpoint pair");
    auto bad_parity = group({polyline({{0, 0, 0}, {1, 0, 0}, {0, 0, 0}})}, 4);
    bool threw = false;
    try {
        swept_detail::tube_mesh_source_conditions(bad_parity, path, true, budget);
    } catch (const std::exception &) {
        threw = true;
    }
    check(threw, "parity child without curve-array interface is unsafe rather than open");
    const auto before = body;
    const auto mesh = mesh_bgfb_swept_body(body);
    check(mesh.status == "meshed" && mesh.points.size() == 8 && mesh.point_indices.size() == 48,
          "public source sweep entry reaches complete prism mesh");
    auto solid = mesh_bgfb_solid(body);
    check(solid.derived.status == "meshed" && solid.derived.geometry.faces.size() == 12 &&
              solid.derived.geometry.vertices == mesh.points && solid.source == body,
          "general solid dispatcher consumes the source sweep mesh");
    check(solid.derived.geometry.face_normal_indices.size() == 12 &&
              solid.derived.geometry.face_uv_indices.size() == 12 &&
              solid.derived.report["native_mesh_metadata"]["face_data"].size() == 6 &&
              solid.derived.report["native_mesh_metadata"]["face_data_indices"] ==
                  mesh.face_data_indices &&
              solid.face_indices.empty(),
          "general dispatcher retains runtime records without inventing grouped facet IDs");
    PolyfaceMeshOptions limited;
    limited.max_curve_work = 1;
    check(mesh_bgfb_solid(body, limited).derived.status != "meshed",
          "general solid dispatcher bounds the source reconstruction work");
    limited = {};
    limited.max_corners = 4;
    check(mesh_bgfb_solid(body, limited).derived.status != "meshed",
          "general solid dispatcher bounds native output index storage");
    check(std::abs(std::abs(volume(mesh)) - 16) < 1e-10, "public prism mesh analytic volume");
    check(body == before && mesh.source == body && mesh.two_sided,
          "public mesh keeps immutable source and native sidedness");
    check(mesh.normal_indices.size() == 48 && mesh.parameter_indices.size() == 48 &&
              mesh.face_data_indices.size() == 48 && mesh.face_data.size() == 6,
          "public output retains all independent attributes and face records");
    check(mesh.layout["mesh_style"] == 1 && mesh.layout["pools"]["parameters"]["active"] == true &&
              mesh.layout["indices"]["face_data"]["active"] == true &&
              mesh.report["material_part_mapping_status"] == "not_evaluated",
          "layout available without inventing material part IDs");
    for (std::size_t i = 0; i < mesh.point_indices.size(); ++i) {
        check((mesh.point_indices[i] == 0) == (mesh.face_data_indices[i] == 0),
              "public zero-terminated face-data correspondence");
        if (mesh.point_indices[i]) {
            check(mesh.normal_indices[i] > 0 &&
                      std::size_t(mesh.normal_indices[i]) <= mesh.normals.size(),
                  "normal reference in range");
            check(mesh.parameter_indices[i] > 0 &&
                      std::size_t(mesh.parameter_indices[i]) <= mesh.parameters.size(),
                  "UV reference in range");
        }
    }
    for (unsigned mask = 0; mask < 4; ++mask) {
        SweptBodyMeshOptions o;
        o.normals = mask & 1;
        o.parameters = mask & 2;
        o.parameter_mode = mask;
        const auto result = mesh_bgfb_swept_body(body, o);
        check(result.status == "meshed" && result.normals.empty() == !o.normals &&
                  result.parameters.empty() == !o.parameters &&
                  result.face_data.empty() == !o.parameters,
              "public optional channels and parameter modes");
    }
    auto no_caps = body;
    no_caps["capped"] = false;
    check(mesh_bgfb_swept_body(no_caps).point_indices.size() == 32,
          "source capped false suppresses only cap faces");
    auto with_hole = body;
    with_hole["profile"] = group({square(2), square(1)}, 4);
    auto hole = mesh_bgfb_swept_body(with_hole);
    check(hole.status == "meshed" && std::abs(std::abs(volume(hole)) - 48) < 1e-9,
          "source parity orientation and multi-ring cap assembly preserve hole volume");
    auto bent = body;
    bent["path"] = group({polyline({{0, 0, 0}, {0, 0, 4}, {3, 0, 4}})});
    check(mesh_bgfb_swept_body(bent).status == "meshed", "bent source path reaches mesh assembly");
    auto invalid = body;
    invalid["profile"] = parity;
    check(mesh_bgfb_swept_body(invalid).status == "native_failure",
          "public parity rejection differs from unsupported input");
    for (const auto &bad :
         {Json(), Json{{"_type", "Polyface"}},
          Json{{"_type", "P3DSweptBody"}, {"profile", nullptr}, {"path", path}}}) {
        const auto r = mesh_bgfb_swept_body(bad);
        check(r.status == "not_meshed" && r.points.empty() && r.point_indices.empty(),
              "bad public source publishes no partial mesh");
    }
    const auto used = mesh.report["work_used"].get<std::size_t>();
    for (auto limit : {std::size_t(0), used / 2, used - 1}) {
        SweptBodyMeshOptions o;
        o.max_work = limit;
        const auto r = mesh_bgfb_swept_body(body, o);
        check(r.status == "not_meshed" && r.points.empty() && r.face_data.empty() &&
                  r.layout.empty(),
              "late public output failure clears every mesh channel");
    }
    SweptBodyMeshOptions small;
    small.max_control_points = 3;
    check(mesh_bgfb_swept_body(body, small).status == "not_meshed",
          "public control budget enforced");
    small = {};
    small.max_patches = 1;
    check(mesh_bgfb_swept_body(bent, small).status == "not_meshed", "public patch count enforced");
    auto future = std::async(std::launch::async, [&] { return mesh_bgfb_swept_body(body); });
    const auto again = future.get();
    check(again.point_indices == mesh.point_indices && again.report == mesh.report,
          "public source meshing is deterministic and independently callable");
    return checks;
}
