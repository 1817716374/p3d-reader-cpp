#include "native_polyface_smooth_normals.hpp"
#include <future>
#include <limits>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_polyface_smooth_normals_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto near = [](double a, double b) { return std::abs(a - b) < 1e-12; };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "smooth normals reject unsafe arithmetic and exhausted budgets");
    };
    auto square = [] {
        NativePolyfaceFaceDataState s;
        s.mesh.coordinates.points = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 1, 3, 4, 0};
        return s;
    };
    auto run = [](const NativePolyfaceFaceDataState &s, double edge, double accumulated,
                  bool visible) {
        TubeBudget b;
        return build_native_polyface_approximate_normals(s, edge, accumulated, visible, b);
    };
    auto folded = [](double angle, double scale) {
        NativePolyfaceFaceDataState s;
        s.mesh.coordinates.points = {{0, 0, 0},
                                     {1, 0, 0},
                                     {0, 1, 0},
                                     {0, -scale * std::cos(angle), scale * std::sin(angle)}};
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 2, 1, 4, 0};
        return s;
    };
    const auto original = square();
    {
        auto s = original;
        s.mesh.indices.indices[point_channel] = {1, -2, 3, 0};
        const auto r = run(s, 1, 1, true);
        check(r.native_succeeded && r.complete && r.output.normal_pool_active &&
                  r.output.mesh.indices.active[normal_channel],
              "successful smoothing activates normal channels");
        check(r.report["base_nodes"] == Json::array({4, 2, 0}) &&
                  r.output.mesh.coordinates.normals == std::vector<Point3>(3, Point3{0, 0, 1}) &&
                  r.output.mesh.indices.indices[normal_channel] ==
                      std::vector<std::int32_t>{3, 1, 2, 0},
              "isolated triangle uses descending native base order without deduplicating equal "
              "normals");
        check(r.output.mesh.indices.indices[point_channel] == std::vector<std::int32_t>{1, 2, 3, 0},
              "requested visibility replaces source boundary signs");
        check(run(s, 1, 1, false).output.mesh.indices.indices[point_channel] ==
                  s.mesh.indices.indices[point_channel],
              "unrequested visibility preserves source signs");
    }
    {
        const auto r = run(original, 1, 1, true);
        check(r.complete && r.report["base_nodes"] == Json::array({8, 6, 4, 0}) &&
                  r.output.mesh.coordinates.normals == std::vector<Point3>(4, Point3{0, 0, 1}) &&
                  r.output.mesh.indices.indices[normal_channel] ==
                      std::vector<std::int32_t>{4, 2, 1, 0, 4, 1, 3, 0},
              "two flat triangles share one normal index per native vertex sector");
        check(r.report["hidden_half_edges"] == 2 &&
                  r.output.mesh.indices.indices[point_channel] ==
                      std::vector<std::int32_t>{1, 2, -3, 0, -1, 3, 4, 0},
              "visibility hides exactly the shared edge by normal index identity");
        for (double edge : {-1.0, 0.0, std::nextafter(1e-5, 0.0)}) {
            const auto sharp = run(original, edge, 1, true);
            check(sharp.complete && sharp.report["angle_base_count"] == 2 &&
                      sharp.output.mesh.coordinates.normals.size() == 6 &&
                      sharp.report["hidden_half_edges"] == 0,
                  "native edge-angle margin makes even a flat edge sharp below 1e-5 radians");
        }
        const auto boundary = run(original, 1e-5, 1, true);
        check(boundary.complete && boundary.report["angle_base_count"] == 0 &&
                  boundary.output.mesh.coordinates.normals.size() == 4,
              "equality at native edge-angle margin is accepted");
        for (double accumulated : {0.0, -1.0}) {
            const auto separate = run(original, 1, accumulated, true);
            check(separate.complete && separate.output.mesh.coordinates.normals.size() == 6 &&
                      separate.report["averaged_sectors"] == 0 &&
                      separate.report["hidden_half_edges"] == 0,
                  "strict accumulated cutoff retains equal-valued normals as separate visible "
                  "sectors");
        }
        TubeBudget b;
        const auto uv = build_native_polyface_parameters(r.output, 2, b);
        const auto faces = build_native_polyface_face_data(uv.output, b);
        const auto joined = assemble_native_builder_polyfaces({faces.output.mesh}, {}, b);
        check(uv.complete && faces.complete && joined.complete &&
                  joined.coordinates.normals.size() == 1 &&
                  joined.indices.indices[point_channel] ==
                      r.output.mesh.indices.indices[point_channel],
              "smooth normals compose with UV, face data and native matched builder mapping");
    }
    {
        const double pi = std::acos(-1.0);
        for (double degrees : {30.0, 45.0, 50.0, 90.0, 180.0}) {
            const double theta = degrees * pi / 180;
            auto s = folded(theta, 1);
            const auto r = run(s, 4, 10, true);
            const bool accepted = degrees <= 45;
            check(r.complete && r.report["angle_base_count"] == 0 &&
                      r.output.mesh.coordinates.normals.size() == (accepted ? 4 : 6) &&
                      r.report["hidden_half_edges"] == (accepted ? 2 : 0),
                  "sector mean dot threshold independently rejects broad folds despite permissive "
                  "edge angles");
            std::size_t pairs = 0;
            for (const auto &sector : r.report["sectors"])
                if (sector["corner_count"] == 2) {
                    ++pairs;
                    check(
                        near(sector["accumulated_angle"].get<double>(), theta) &&
                            near(sector["average_normal_dot"].get<double>(), std::cos(theta / 2)) &&
                            sector["averaged"] == accepted,
                        "two-face analytic fold has the expected mean and angle");
                }
            check(pairs == 2, "both shared-edge vertices have a two-corner sector");
            if (accepted) {
                const auto &normals = r.output.mesh.coordinates.normals;
                const auto &ni = r.output.mesh.indices.indices[normal_channel];
                const auto normal = normals[static_cast<std::size_t>(ni[0] - 1)];
                check(near(normal[0], 0) && near(normal[1], std::sin(theta / 2)) &&
                          near(normal[2], std::cos(theta / 2)),
                      "accepted fold normal is the analytic unweighted angle bisector");
                const auto unequal = run(folded(theta, 100), 4, 10, true);
                const auto other = unequal.output.mesh.coordinates.normals[static_cast<std::size_t>(
                    unequal.output.mesh.indices.indices[normal_channel][0] - 1)];
                check(near(other[0], normal[0]) && near(other[1], normal[1]) &&
                          near(other[2], normal[2]),
                      "face area does not weight the native average");
                double measured = 0;
                for (const auto &sector : r.report["sectors"])
                    if (sector["corner_count"] == 2)
                        measured = sector["accumulated_angle"].get<double>();
                check(run(s, 4, measured, true).output.mesh.coordinates.normals.size() == 6 &&
                          run(s, 4, std::nextafter(measured, 10.0), true)
                                  .output.mesh.coordinates.normals.size() == 4,
                      "accumulated-angle equality rejects the whole sector and the next double "
                      "accepts it");
                check(run(s, theta / 2, 10, true).report["angle_base_count"] == 2,
                      "single-edge cutoff splits sectors before averaging");
            }
        }
        // Exactly opposite normals exercise the zero-sum normalization fallback.
        auto opposite = original;
        opposite.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 3, 2, 1, 0};
        const auto r = run(opposite, 4, 10, true);
        check(r.complete && r.output.mesh.coordinates.normals.size() == 6 &&
                  r.report["averaged_sectors"] == 0,
              "exactly opposite normals do not manufacture a smoothed direction from a zero sum");
    }
    {
        auto s = original;
        s.mesh.coordinates.points = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0}};
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 1, 3, 4, 0, 1, 4, 5, 0, 1, 5, 2, 0};
        const auto r = run(s, 1, 1, true);
        check(r.complete && r.report["interior_base_count"] == 1 &&
                  r.output.mesh.coordinates.normals.size() == 5 &&
                  r.report["hidden_half_edges"] == 8,
              "closed interior vertex ring adds one representative sector");
        const auto &ni = r.output.mesh.indices.indices[normal_channel];
        check(ni[0] == ni[4] && ni[0] == ni[8] && ni[0] == ni[12],
              "all four interior corners share a sector independently of exterior boundaries");
        // A folded pyramid tests whole-ring accumulation without the final-to-first angle.
        s.mesh.coordinates.points[0][2] = .1;
        const auto folded_ring = run(s, 4, 10, true);
        const auto &sector = folded_ring.report["sectors"].back();
        const double adjacent = std::acos(1.0 / 1.02);
        check(sector["corner_count"] == 4 &&
                  near(sector["accumulated_angle"].get<double>(), 3 * adjacent),
              "closed vertex sector accumulates three consecutive angles and excludes the closing "
              "angle");
        const double measured = sector["accumulated_angle"].get<double>();
        const auto rejected = run(s, 4, measured * .9, true);
        check(rejected.report["sectors"].back()["averaged"] == false &&
                  rejected.report["sectors"].back()["corner_count"] == 4,
              "accumulated cutoff rejects the entire interior sector without incremental "
              "subdivision");
    }
    {
        auto s = original;
        s.mesh.num_per_face = 4;
        check(run(s, 1, 1, false).output.mesh.indices.indices[normal_channel] ==
                  run(original, 1, 1, false).output.mesh.indices.indices[normal_channel],
              "fixed padded blocks retain the same read-position assignments");
        s = original;
        s.mesh.indices.indices[point_channel] = {0, 1, 2, 3, 0, 0, 1, 3, 4};
        auto r = run(s, 1, 1, false);
        check(r.complete && r.output.mesh.indices.indices[normal_channel].size() == 9 &&
                  r.output.mesh.indices.indices[normal_channel][0] == 0 &&
                  r.output.mesh.indices.indices[normal_channel][5] == 0,
              "variable leading zeros and unterminated last face preserve source index layout");
        s = original;
        s.mesh.coordinates.normals = {{9, 8, 7}};
        s.mesh.indices.indices[normal_channel] = {999};
        s.mesh.indices.indices[point_channel].clear();
        r = run(s, 1, 1, true);
        check(!r.native_succeeded && !r.complete &&
                  r.report["reason"] == "per_face_generation_failed" &&
                  r.output.mesh.coordinates.normals == s.mesh.coordinates.normals,
              "empty point-index input preserves preexisting normal data");
        s = original;
        s.mesh.coordinates.points = std::vector<Point3>(4, Point3{7, 8, 9});
        s.mesh.coordinates.normals = {{9, 8, 7}};
        s.mesh.indices.indices[point_channel] = {1, -2, 3, 0};
        r = run(s, 1, 1, true);
        check(
            !r.native_succeeded && !r.complete && r.report["normal_pool_replaced"] == false &&
                r.output.mesh.coordinates.normals.empty() &&
                r.output.mesh.indices.indices[normal_channel] == std::vector<std::int32_t>(4) &&
                r.output.mesh.indices.indices[point_channel] ==
                    s.mesh.indices.indices[point_channel] &&
                s.mesh.coordinates.normals == std::vector<Point3>{{9, 8, 7}},
            "failed smoothing retains per-face preparation but never mutates source or visibility");
        s = original;
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 1, 2, 99, 0, 1, 3, 4, 0};
        r = run(s, 1, 1, true);
        check(r.native_succeeded && !r.complete && r.report["unassigned_nonzero_indices"] == 6 &&
                  r.report["normal_pool_replaced"] == true,
              "native success with a partially visited mesh never implies complete normals");
        s = original;
        s.mesh.indices.indices[point_channel] = {1, 2, 3, 0, 1, 2, 4, 0};
        r = run(s, 4, 10, true);
        check(!r.complete && r.report["connectivity"]["complete"] == false,
              "same-direction native edge pairing retains the inconsistent topology diagnostic");
    }
    {
        TubeBudget measured;
        build_native_polyface_approximate_normals(original, 1, 1, true, measured);
        for (auto limit : {std::size_t(0), measured.work / 2, measured.work - 1}) {
            TubeBudget b;
            b.max_work = limit;
            rejects([&] { build_native_polyface_approximate_normals(original, 1, 1, true, b); });
        }
        for (auto limit : {std::size_t(0), std::size_t(25), std::size_t(85)}) {
            TubeBudget b;
            b.max_control_points = limit;
            rejects([&] { build_native_polyface_approximate_normals(original, 1, 1, true, b); });
        }
        for (double bad :
             {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
            rejects([&] { run(original, bad, 1, true); });
            rejects([&] { run(original, 1, bad, true); });
        }
        auto s = original;
        s.mesh.coordinates.points[0][0] = std::numeric_limits<double>::infinity();
        rejects([&] { run(s, 1, 1, true); });
        auto task = [&] { return run(original, 1, 1, true); };
        auto a = std::async(std::launch::async, task), b = std::async(std::launch::async, task);
        auto x = a.get(), y = b.get();
        check(x.complete && y.complete && x.report == y.report &&
                  x.output.mesh.indices.indices == y.output.mesh.indices.indices &&
                  x.output.mesh.coordinates.normals == y.output.mesh.coordinates.normals,
              "parallel operations use independent deterministic state");
        check(original.mesh.coordinates.normals.empty() &&
                  original.mesh.indices.indices[point_channel] ==
                      square().mesh.indices.indices[point_channel],
              "all success and failure paths leave the source unchanged");
    }
    return checks;
}
