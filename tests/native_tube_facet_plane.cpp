#include "native_tube_facet_plane.hpp"
#include "native_control_lines.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
Json surface(bool second = false, unsigned order = 2, bool weighted = false) {
    Json xyz = Json::array(), w = Json::array();
    for (unsigned row = 0; row < order; ++row)
        for (unsigned i = 0; i < 2; ++i) {
            const double v = double(row) / (order - 1), weight = weighted ? (row + 1) * 2. : 1.;
            const Point3 p =
                second ? Point3{2., 2 * v - 1., double(i) + .01} : Point3{v, 0., double(i)};
            for (double x : p)
                xyz.push_back(x * weight);
            w.push_back(weight);
        }
    return {{"_type", "BsplineSurface"},
            {"orderU", 2},
            {"orderV", order},
            {"numPolesU", 2},
            {"numPolesV", order},
            {"closedU", false},
            {"closedV", false},
            {"knotsU", nullptr},
            {"knotsV", nullptr},
            {"poles", xyz},
            {"weights", weighted ? w : Json()},
            {"boundaries", nullptr},
            {"numRulesU", 5},
            {"numRulesV", 7},
            {"holeOrigin", 0}};
}
TubeFacetSeamReferences seam(double intercept) {
    TubeFacetSeamReferences s;
    s.incoming = std::make_shared<TubeFacetSeamStorage<Point3>>(
        TubeFacetSeamStorage<Point3>{{1, 0, 0}, true});
    s.outgoing = std::make_shared<TubeFacetSeamStorage<Point3>>(
        TubeFacetSeamStorage<Point3>{{0, 1, 0}, true});
    s.plane = std::make_shared<TubeFacetSeamStorage<std::array<Point3, 2>>>(
        TubeFacetSeamStorage<std::array<Point3, 2>>{{Point3{intercept, 0, 0}, Point3{1, 1, 0}},
                                                    true});
    return s;
}
bool near(Point3 a, Point3 b) {
    for (unsigned k = 0; k < 3; ++k)
        if (std::abs(a[k] - b[k]) > 1e-12 * (1 + std::abs(b[k])))
            return false;
    return true;
}
} // namespace
unsigned native_tube_facet_plane_tests() {
    unsigned count = 0;
    auto check = [&](bool ok, const char *why) {
        ++count;
        require(ok, why);
    };
    auto rejects = [&](auto f) {
        bool caught = false;
        try {
            f();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "native plane seam rejects invalid references or unbounded arithmetic");
    };
    const std::array<Point3, 2> plane{Point3{2, 0, 0}, Point3{7, 0, 0}};
    auto ray = native_ray_plane_intersection({0, 3, 4}, {2, 0, 0}, plane);
    check(ray.divided && ray.parameter == 1 && ray.point == Point3{2, 3, 4},
          "ray-plane query retains source normal and direction scales");
    auto parallel = native_ray_plane_intersection({0, 3, 4}, {0, 1, 0}, plane);
    check(!parallel.divided && parallel.parameter == 0 && parallel.point == Point3{0, 3, 4},
          "protected division returns original ray point for a parallel direction");
    for (double denom : {std::nextafter(1e-15, 0.), 1e-15, std::nextafter(1e-15, 1.)}) {
        auto r = native_ray_plane_intersection({0, 0, 0}, {denom, 0, 0},
                                               {Point3{1, 0, 0}, Point3{1, 0, 0}});
        check(r.divided == (denom > 1e-15),
              "ray-plane protection uses strict numerator-relative tolerance");
    }
    const auto a = surface(), b = surface(true);
    auto prepare = [&](const Json &x, const Json &y, const TubeFacetSeamReferences &s) {
        TubeBudget budget;
        return prepare_tube_facet_plane_seam(x, y, s, budget);
    };
    auto normal = prepare(a, b, seam(2));
    check(normal.status == TubeFacetSeamStatus::complete && normal.classifier == 0,
          "skew ruled control lines can complete by separate plane projections");
    for (unsigned i = 0; i < 2; ++i) {
        check(near(normal.first_projected[i], {2, 0, double(i)}) &&
                  near(normal.second_projected[i], {2, 0, double(i) + .01}),
              "plane projections retain the source spatial gap");
        check(near(BsplineSurface::from_bgfb(normal.first).poles()[2 + i],
                   normal.first_projected[i]) &&
                  near(BsplineSurface::from_bgfb(normal.second).poles()[i],
                       normal.second_projected[i]),
              "ruled plane branch applies projected rows without changing outer controls");
    }
    auto work_a = a, work_b = b;
    auto live = seam(2);
    TubeBudget integrated_budget;
    auto applied = apply_tube_facet_seam(work_a, work_b, live, integrated_budget);
    check(applied.status == TubeFacetSeamStatus::complete && live.classifier == 0 &&
              work_a == normal.first && work_b == normal.second,
          "direct skew detection connects to native plane fallback");
    auto weighted_a = surface(false, 2, true), weighted_b = surface(true, 2, true);
    auto weighted = prepare(weighted_a, weighted_b, seam(2));
    check(weighted.first["weights"] == weighted_a["weights"] &&
              near(BsplineSurface::from_bgfb(weighted.first).poles()[2], {4, 0, 0}),
          "plane fallback preserves native first-row reweighting of the last control row");
    auto early_first_seam = seam(-2);
    early_first_seam.outgoing.reset();
    auto first_skip = prepare(weighted_a, b, early_first_seam);
    check(first_skip.status == TubeFacetSeamStatus::complete && first_skip.classifier == 2 &&
              first_skip.first == weighted_a && first_skip.second == b &&
              first_skip.report["second"].empty(),
          "first retreat skip restores its row and never dereferences the unused outgoing tangent");
    auto second_skip = prepare(weighted_a, b, seam(4));
    check(second_skip.status == TubeFacetSeamStatus::complete && second_skip.classifier == 2 &&
              second_skip.second == b && second_skip.first["weights"] == weighted_a["weights"] &&
              BsplineSurface::from_bgfb(second_skip.first).poles()[2] == Point3{1, 0, 0},
          "second retreat skip leaves the first row unweighted as the native early return does");
    auto boundary = prepare(a, b, seam(0));
    check(boundary.classifier == 0,
          "retreat equal to control distance does not trigger strict skip");
    auto scaled = seam(0);
    scaled.incoming->value = {.5, 0, 0};
    check(prepare(a, b, scaled).classifier == 2,
          "retreat compares raw ray parameter with control distance without rescaling direction");
    auto protected_seam = seam(2);
    protected_seam.plane->value[1] = {0, 0, 1};
    auto protected_result = prepare(a, b, protected_seam);
    check(protected_result.classifier == 0 && protected_result.first == a &&
              protected_result.second == b,
          "ignored plane division failure retains projected origin rather than inventing an "
          "intersection");
    for (unsigned order : {3u, 4u, 8u, 26u}) {
        const auto high_a = surface(false, order, true), high_b = surface(true, order, true);
        const auto prepared = prepare(high_a, high_b, seam(100));
        check(prepared.status == TubeFacetSeamStatus::pending_general && prepared.classifier == 1 &&
                  prepared.first_extension == 1 && prepared.second_extension == 0,
              "higher-order extension uses positive tangent-line fractions independently of the "
              "plane offset");
        check(prepared.first_original.size() == 2 && prepared.first_weights.size() == 2 &&
                  prepared.first_weights[0] == 2. * order && prepared.second_weights[0] == 2.,
              "higher-order preparation retains source Cartesian rows and actual endpoint weights");
        check(near(BsplineSurface::from_bgfb(prepared.first).poles().back(),
                   BsplineSurface::from_bgfb(high_a).poles().back()),
              "higher-order preparation restores weights before the extension consumer");
    }
    auto high_a = surface(false, 3, true), high_b = surface(true, 3, true);
    auto failed_seam = seam(2);
    failed_seam.outgoing->value = {1, 0, 0};
    auto native_failure = prepare(high_a, high_b, failed_seam);
    check(native_failure.status == TubeFacetSeamStatus::native_failure &&
              native_failure.classifier == -2 &&
              BsplineSurface::from_bgfb(native_failure.first).poles()[4] == Point3{1, 0, 0},
          "parallel extension tangents fail before native row reweighting and classifier update");
    const auto source_a = high_a, source_b = high_b;
    auto higher_seam = seam(2);
    TubeBudget pending_budget;
    check(apply_tube_facet_seam(high_a, high_b, higher_seam, pending_budget).status ==
                  TubeFacetSeamStatus::pending_general &&
              high_a == source_a && high_b == source_b && higher_seam.classifier == -2,
          "unimplemented extension keeps the externally carried current seam unchanged for "
          "resumption");
    auto missing = seam(2);
    missing.incoming.reset();
    rejects([&] { prepare(a, b, missing); });
    auto freed = seam(2);
    freed.plane->alive = false;
    rejects([&] { prepare(a, b, freed); });
    auto zero = weighted_a;
    zero["weights"][2] = 0.;
    rejects([&] { prepare(zero, b, seam(2)); });
    const auto self = surface(false, 2, true);
    auto self_seam = seam(1);
    self_seam.outgoing->value = {-1, 0, 0};
    auto self_result = prepare(self, self, self_seam);
    check(self_result.status == TubeFacetSeamStatus::complete &&
              self_result.first == self_result.second && self_result.report["self_surface"] == true,
          "self plane seam has one shared working table and both sequential row writes");
    auto measured_seam = seam(2);
    TubeBudget measured;
    prepare_tube_facet_plane_seam(weighted_a, weighted_b, measured_seam, measured);
    TubeBudget short_work;
    short_work.max_work = measured.work - 1;
    rejects(
        [&] { prepare_tube_facet_plane_seam(weighted_a, weighted_b, measured_seam, short_work); });
    TubeBudget short_storage;
    short_storage.max_control_points = 7;
    rejects([&] { prepare_tube_facet_plane_seam(a, b, measured_seam, short_storage); });
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(
            std::async(std::launch::async, [&] { return prepare(a, b, seam(2)).first; }));
    for (auto &job : jobs)
        check(job.get() == normal.first, "plane working state is private to each concurrent call");
    return count;
}
