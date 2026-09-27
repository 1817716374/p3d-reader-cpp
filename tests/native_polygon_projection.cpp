#include "native_polygon_projection.hpp"
#include "native_tube_mesh_index_rules.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_polygon_projection_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto close = [&](double a, double b) {
        check(std::abs(a - b) < 2e-12 * (1 + std::abs(b)), "native projection coordinate");
    };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "native projection invalid input or budget rejected");
    };
    auto project = [&](const std::vector<Point3> &p) {
        TubeBudget b;
        return prepare_native_polygon_projection(p, b);
    };
    const double marker = std::numeric_limits<double>::max();
    Matrix4 identity{};
    for (unsigned i = 0; i < 4; ++i)
        identity[i][i] = 1;
    const std::vector<Point3> rectangle{{10, 20, 30}, {12, 20, 30}, {12, 23, 30}, {10, 23, 30}};
    const auto a = project(rectangle);
    check(a.frame_succeeded && a.report.at("square_normalization_succeeded") == true &&
              a.report.at("selected_edge") == 1 && a.report.at("native_area") == 6,
          "translated rectangle builds original unit axes at first point");
    check(a.points == std::vector<Point3>{{0, 0, 0}, {2, 0, 0}, {2, 3, 0}, {0, 3, 0}},
          "projection preserves rectangle scale rather than normalizing its range");
    check(a.local_to_world[0][3] == 10 && a.local_to_world[1][3] == 20 &&
              a.local_to_world[2][3] == 30,
          "frame is located at first point rather than centroid");
    // A vertical plane: native X is world +Z, Y is world -Y, normal world +X.
    const std::vector<Point3> vertical{{4, 5, 6}, {4, 5, 8}, {4, 2, 8}, {4, 2, 6}};
    auto v = project(vertical);
    check(v.report.at("normal") == Json(Point3{1, 0, 0}) && v.points == a.points,
          "vertical ring uses spatial normal and original first edge");
    auto reversed = rectangle;
    std::reverse(reversed.begin(), reversed.end());
    auto reverse = project(reversed);
    check(reverse.report.at("normal") == Json(Point3{0, 0, -1}) &&
              reverse.points[2] == Point3{2, 3, 0},
          "reversed winding reverses native normal with no loop reordering");
    // Independent rigid basis with rational entries, followed by inverse checks.
    auto tilted = rectangle;
    for (auto &p : tilted)
        p = {0.6 * p[0] - 0.8 * p[2], p[1], 0.8 * p[0] + 0.6 * p[2]};
    auto t = project(tilted);
    for (std::size_t i = 0; i < tilted.size(); ++i)
        for (unsigned r = 0; r < 3; ++r) {
            close(t.points[i][r], a.points[i][r]);
            long double back = t.local_to_world[r][3];
            for (unsigned c = 0; c < 3; ++c)
                back += static_cast<long double>(t.local_to_world[r][c]) * t.points[i][c];
            close(double(back), tilted[i][r]);
        }
    auto nonplanar = rectangle;
    nonplanar[2][2] += 2;
    auto spatial = project(nonplanar);
    check(spatial.frame_succeeded &&
              std::any_of(spatial.points.begin(), spatial.points.end(),
                          [](const Point3 &p) { return std::abs(p[2]) > .1; }),
          "coordinate frame success does not flatten or certify nonplanar polygon");
    // Only the prefix before the first marker chooses the frame. Later rings
    // must still be transformed, retaining all markers and source indices.
    for (unsigned axis = 0; axis < 3; ++axis) {
        auto p = rectangle;
        Point3 separator{7, 8, 9};
        separator[axis] = marker;
        p.push_back(separator);
        p.push_back({12, 23, 35});
        p.push_back(separator);
        auto result = project(p);
        check(result.report.at("first_loop_points") == 4 && result.report.at("native_area") == 6 &&
                  result.report.at("disconnect_markers_preserved") == 2 &&
                  result.points[4] == separator && result.points[6] == separator &&
                  result.points[5] == Point3{2, 3, 5},
              "any positive DBL_MAX component separates loops without modifying markers");
    }
    for (const auto &p :
         {std::vector<Point3>{}, std::vector<Point3>{{1, 2, 3}},
          std::vector<Point3>{{1, 2, 3}, {2, 3, 4}},
          std::vector<Point3>{{marker, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}},
          std::vector<Point3>{
              {0, 0, 0}, {1, 0, 0}, {0, marker, 0}, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}}}) {
        const auto r = project(p);
        check(!r.frame_succeeded && r.local_to_world == identity && r.world_to_local == identity &&
                  r.points.empty(),
              "short first loop fails without using a later loop or line fallback");
    }
    for (unsigned count : {3u, 5u}) {
        const auto same = project(std::vector<Point3>(count, {4, 5, 6}));
        check(!same.frame_succeeded && same.points.empty() && same.report.at("frame_attempts") == 0,
              "all identical points supply no qualifying edge");
    }
    const auto line = project({{4, 5, 6}, {4, 7, 6}, {4, 9, 6}});
    check(line.frame_succeeded && line.report.at("native_area") == 0 &&
              line.report.at("square_normalization_succeeded") == false &&
              line.points == std::vector<Point3>{{0, 0, 0}, {0, 2, 0}, {0, 4, 0}},
          "zero normal can use identity axes and succeed despite square normalization failure");
    auto duplicate = rectangle;
    duplicate.insert(duplicate.begin() + 1, duplicate.front());
    auto dup = project(duplicate);
    check(dup.report.at("selected_edge") == 2 && dup.points.size() == 5 &&
              dup.points[0] == dup.points[1],
          "duplicate first point skipped for frame but retained in projected output");
    for (double short_x : {0.0005, 0.001, std::nextafter(.001, 1.)}) {
        auto r = project({{0, 0, 0}, {short_x, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}});
        check(r.report.at("edge_length_threshold") == .001 &&
                  r.report.at("selected_edge") == (short_x > .001 ? 1 : 2),
              "edge acceptance uses strict area-scaled native threshold");
    }
    // Nonplanar spikes parallel to the accumulated normal exercise both triad
    // reference choices. No mathematical face repair or removal is applied.
    auto spike_z = project({{0, 0, 0}, {0, 0, 1}, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}});
    check(spike_z.frame_succeeded && spike_z.local_to_world == identity &&
              spike_z.report.at("square_normalization_succeeded") == false,
          "normal-parallel edge uses original Y-reference triad fallback");
    auto spike_x = project({{0, 0, 0}, {1, 0, 0}, {0, 0, 0}, {0, 1, 0}, {0, 0, 1}});
    check(spike_x.frame_succeeded && spike_x.local_to_world[0][2] == 1 &&
              spike_x.local_to_world[1][0] == 1 && spike_x.local_to_world[2][1] == 1 &&
              spike_x.report.at("square_normalization_succeeded") == false,
          "normal-parallel edge uses original Z-reference triad fallback");
    // Three-point normal chooses its starting corner and, when needed, a
    // median subtriangle. Four entries (even with a closure) use the fan path.
    const std::vector<Point3> skinny{{0, 0, 0}, {2, 0, 0}, {1, 1e-4, 0}};
    auto tri = project(skinny);
    check(tri.report.at("triangle_origin") == 0 && tri.report.at("triangle_median_used") == true,
          "native triangle corner tie and small-angle median rule preserved");
    close(tri.report.at("native_area").get<double>(), 5e-5);
    auto closed = skinny;
    closed.push_back(closed.front());
    auto fan = project(closed);
    close(fan.report.at("native_area").get<double>(), 1e-4);
    check(fan.report.at("triangle_median_used") == false,
          "explicit closure changes native three-point special branch to polygon fan");
    auto origin1 = project({{0, 0, 0}, {1, 0, 0}, {3, 1, 0}});
    auto origin2 = project({{0, 0, 0}, {3, 0, 0}, {2, 1, 0}});
    check(origin1.report.at("triangle_origin") == 1 && origin2.report.at("triangle_origin") == 2,
          "both unequal-side native triangle origin branches reached");
    TubeBudget b;
    auto prepared = native_facet_index_plan(duplicate, b);
    check(
        prepared.route == NativeFacetIndexPlan::Route::projected_loops &&
            prepared.completed && !prepared.indices.empty() && prepared.projection &&
            prepared.projection->frame_succeeded &&
            prepared.projection->report.at("triangulated") == false,
        "large facet completes triangulation while projection report describes only preparation");
    auto short_plan = native_facet_index_plan(rectangle, b);
    check(!short_plan.projection, "quad path does not run polygon projection");
    check(prepared.projection->points.size() == duplicate.size() + 1 &&
              prepared.projection->points.front() == prepared.projection->points.back(),
          "large-face caller retains the native visitor extra closure in projection input");
    rejects([&] {
        TubeBudget z;
        z.max_control_points = duplicate.size();
        native_facet_index_plan(duplicate, z);
    });
    rejects([&] {
        auto z = rectangle;
        z[2][0] = std::numeric_limits<double>::quiet_NaN();
        project(z);
    });
    rejects([&] {
        auto z = rectangle;
        z[2][1] = std::numeric_limits<double>::infinity();
        project(z);
    });
    rejects([&] { project({{0, 0, 0}, {-marker, 0, 0}, {0, 1, 0}}); });
    rejects([&] {
        TubeBudget z;
        z.max_control_points = 3;
        prepare_native_polygon_projection(rectangle, z);
    });
    rejects([&] {
        TubeBudget z;
        z.max_work = 4;
        prepare_native_polygon_projection(rectangle, z);
    });
    TubeBudget measured;
    prepare_native_polygon_projection(rectangle, measured);
    rejects([&] {
        TubeBudget z;
        z.max_work = measured.work - 1;
        prepare_native_polygon_projection(rectangle, z);
    });
    auto original = tilted;
    auto task = std::async(std::launch::async, [&] { return project(tilted); });
    const auto concurrent = project(tilted);
    const auto other = task.get();
    check(other.points == concurrent.points && other.report == concurrent.report &&
              tilted == original,
          "projection is deterministic under concurrent use and does not modify input");
    return n;
}
