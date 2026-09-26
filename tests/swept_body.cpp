#include <p3d/solid.hpp>
#include <future>
#include <stdexcept>

using namespace p3d;
unsigned swept_body_tests() {
    unsigned n = 0;
    const Matrix4 identity_matrix{
        {{1., 0., 0., 0.}, {0., 1., 0., 0.}, {0., 0., 1., 0.}, {0., 0., 0., 1.}}};
    auto check = [&](bool ok, const char *why) {
        ++n;
        if (!ok)
            throw std::runtime_error(why);
    };
    auto group = [](Json members) {
        Json curves = Json::array();
        for (auto &member : members)
            curves.push_back({{"geometry", member}});
        return Json{{"_type", "CurveVector"}, {"type", 1}, {"curves", curves}};
    };
    const Json line{{"_type", "LineString"}, {"points", {1., 2., 3., 4., 5., 6.}}};
    const Json spline{{"_type", "BsplineCurve"},   {"order", 2},
                      {"closed", false},           {"poles", {1., 2., 3., 4., 5., 6.}},
                      {"knots", {0., 0., 1., 1.}}, {"weights", {0., -2.}}};
    const Json input{{"_type", "P3DSweptBody"},
                     {"profile", group({group({line}), spline})},
                     {"path", group({line, line})},
                     {"capped", true}};
    const Matrix4 m{{{-2., .5, 0., 11.}, {0., 3., 0., -7.}, {0., 0., .25, 5.}, {0., 0., 0., 1.}}};
    const auto before = input;
    const auto r = transform_bgfb_curve_solid(input, m);
    check(r.status == "transformed" && input == before && r.transformed["capped"] == true,
          "path sweep source placement keeps the original and cap request intact");
    const Json ordinary{10., -1., 5.75, 5.5, 8., 6.5};
    check(
        r.transformed["path"]["curves"][0]["geometry"]["points"] == ordinary &&
            r.transformed["profile"]["curves"][0]["geometry"]["curves"][0]["geometry"]["points"] ==
                ordinary,
        "path and nested profile receive the same world-space affine placement");
    const auto &weighted = r.transformed["profile"]["curves"][1]["geometry"];
    check(weighted["poles"] == Json({-1., 6., .75, -27.5, 29., -8.5}) &&
              weighted["weights"] == spline["weights"] && weighted["knots"] == spline["knots"],
          "path sweep rational XYZ already weighted, including zero and negative weights");
    check(r.report["point_count"] == 8 && r.report["curve_nodes"] == 7 &&
              r.report["mesh_status"] == "not_evaluated",
          "source transformation accounts for both arrays without claiming surface generation");
    for (unsigned mask = 0; mask < 4; ++mask) {
        auto source = input;
        if (!(mask & 1))
            source["path"] = nullptr;
        if (!(mask & 2))
            source["profile"] = nullptr;
        const auto moved = transform_bgfb_curve_solid(source, m);
        check(moved.status == "transformed" && moved.transformed["path"].is_null() == !(mask & 1) &&
                  moved.transformed["profile"].is_null() == !(mask & 2),
              "nullable path and profile remain null independently");
    }
    auto empty = input;
    empty["path"] = empty["profile"] = group(Json::array());
    check(transform_bgfb_curve_solid(empty, m).report["curve_nodes"] == 2,
          "explicit empty arrays are visited and not replaced by null");
    auto limited = CurveSolidTransformOptions{};
    limited.max_points = 3;
    auto failed = transform_bgfb_curve_solid(input, m, limited);
    check(failed.transformed.is_null() && failed.report["source_path"] == "/path/curves/1/geometry",
          "path is visited first and failure reports the exact sibling source path");
    limited.max_points = 7;
    failed = transform_bgfb_curve_solid(input, m, limited);
    check(failed.transformed.is_null() &&
              failed.report["source_path"] == "/profile/curves/1/geometry",
          "both arrays share the point budget and no partial transformed solid escapes");
    limited.max_points = 8;
    limited.max_curve_nodes = 6;
    check(transform_bgfb_curve_solid(input, m, limited).transformed.is_null(),
          "path and profile share the curve-node budget");
    auto invalid = input;
    invalid["path"]["curves"][0]["geometry"]["_type"] = "unknown_curve";
    check(transform_bgfb_curve_solid(invalid, m).transformed.is_null(),
          "unsupported path primitive is not silently left untransformed");
    invalid = input;
    invalid.erase("path");
    check(transform_bgfb_curve_solid(invalid, m).transformed.is_null(),
          "incomplete JSON source is distinct from an explicitly absent native pointer");
    auto singular = m;
    singular[0] = singular[1] = singular[2] = {0., 0., 0., 0.};
    check(transform_bgfb_curve_solid(input, singular).status == "transformed",
          "path sweep placement does not add a nonsingularity requirement");
    auto tiny = identity_matrix;
    tiny[0][3] = 1e-11;
    auto small = transform_bgfb_curve_solid(input, tiny);
    check(small.report["bspline_skipped"] == 1 &&
              small.transformed["profile"]["curves"][1]["geometry"] == spline &&
              small.transformed["path"]["curves"][0]["geometry"] != line,
          "native near-identity suppression is specific to spline curves");
    const Json arc{{"_type", "EllipticArc"},
                   {"arc",
                    {{"centerX", 0.},
                     {"centerY", 0.},
                     {"centerZ", 0.},
                     {"vector0X", 1e-6},
                     {"vector0Y", 0.},
                     {"vector0Z", 0.},
                     {"vector90X", 0.},
                     {"vector90Y", 2.},
                     {"vector90Z", 0.},
                     {"startRadians", 0.},
                     {"sweepRadians", 6.283185307179586}}}};
    auto arcs = input;
    arcs["path"] = arcs["profile"] = group({arc, arc});
    const auto replaced = transform_bgfb_curve_solid(arcs, identity_matrix);
    check(replaced.status == "transformed" && replaced.report["arc_replacements"].size() == 4,
          "both source arrays use native collapsed-ellipse replacement");
    for (unsigned i = 0; i < 4; ++i) {
        const std::string key = i < 2 ? "path" : "profile";
        check(replaced.report["arc_replacements"][i]["source_path"] ==
                      "/" + key + "/curves/" + std::to_string(i % 2) + "/geometry" &&
                  replaced.transformed[key]["curves"][i % 2]["geometryType"] == 1,
              "replacement reports preserve path-first order and each independent sibling path");
    }
    auto asynchronous =
        std::async(std::launch::async, [&] { return transform_bgfb_curve_solid(input, m); });
    check(asynchronous.get().transformed == r.transformed,
          "independent path sweep placements do not share mutable state");
    check(mesh_bgfb_solid(input).derived.status != "meshed",
          "source placement capability does not pretend unsupported path patches are meshed");
    return n;
}
