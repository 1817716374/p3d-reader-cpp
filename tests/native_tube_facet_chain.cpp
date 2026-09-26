#include "native_tube_facet_chain.hpp"
#include "loft_curve.hpp"
#include "native_curve_affine.hpp"
#include "bspline_frame.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
BsplineCurve curve(unsigned order, Json xyz, Json knots = nullptr, Json weights = nullptr,
                   bool closed = false) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", order},
                                    {"closed", closed},
                                    {"poles", xyz},
                                    {"knots", knots},
                                    {"weights", weights}});
}
bool near(const Point3 &a, const Point3 &b) {
    for (unsigned k = 0; k < 3; ++k)
        if (std::abs(a[k] - b[k]) > 2e-10 * (1 + std::abs(b[k])))
            return false;
    return true;
}
} // namespace
unsigned native_tube_facet_chain_tests() {
    unsigned count = 0;
    auto check = [&](bool b, const char *why) {
        ++count;
        require(b, why);
    };
    auto rejects = [&](auto f, const char *why) {
        bool caught = false;
        try {
            f();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, why);
    };
    const auto right_angle = native_tube_facet_seam({2, 0, 0}, {0, 3, 0}, {4, 5, 6});
    check(right_angle.classifier == -2 && right_angle.plane &&
              (*right_angle.plane)[0] == Point3{4, 5, 6} &&
              (*right_angle.plane)[1] == Point3{.5, .5, 0},
          "native right-angle seam keeps origin and unnormalized bisector-plane normal");
    check(right_angle.incoming == Point3{1, 0, 0} && right_angle.outgoing == Point3{0, 1, 0},
          "stored seam tangents are normalized independently");
    for (const auto &b : std::vector<Point3>{{1, 0, 0}, {-2, 0, 0}, {0, 0, 0}}) {
        auto seam = native_tube_facet_seam({1, 0, 0}, b, {0, 0, 0});
        check(seam.classifier == 2 && !seam.plane && !seam.incoming && !seam.outgoing,
              "parallel opposite and zero tangents do not invent a seam plane");
    }
    const auto shallow = native_tube_facet_seam({1, 0, 0}, {1, 1e-6, 0}, {0, 0, 0});
    check(shallow.classifier == 2 && !shallow.plane && !shallow.incoming,
          "nonparallel seam can still have a native near-zero plane normal");
    const auto sharper = native_tube_facet_seam({1, 0, 0}, {1, 1e-3, 0}, {0, 0, 0});
    check(sharper.classifier == -2 && sharper.plane,
          "seam outside near-zero normal threshold retains its plane");
    rejects([&] { native_tube_facet_seam({NAN, 0, 0}, {0, 1, 0}, {0, 0, 0}); },
            "seam cannot retain nonfinite source geometry");
    const auto section = curve(2, {0., 0., 0., .2, 0., 0.});
    auto build = [&](const BsplineCurve &path) {
        TubeBudget b;
        return build_tube_facet_chain(section, path, false, b);
    };
    const auto line = curve(2, {0., 0., 0., 2., 0., 0.});
    auto straight = build(line);
    check(straight.success && straight.nodes.size() == 1 &&
              straight.nodes[0].end_seam.classifier == 2,
          "open single-span path builds one independent patch with no final seam");
    const auto corner = curve(2, {0., 0., 0., 1., 0., 0., 1., 1., 0.});
    auto bent = build(corner);
    check(bent.success && bent.nodes.size() == 2 && bent.nodes[0].end_seam.plane &&
              (*bent.nodes[0].end_seam.plane)[0] == Point3{1, 0, 0} &&
              near((*bent.nodes[0].end_seam.plane)[1], Point3{.5, .5, 0}) &&
              bent.nodes[1].end_seam.classifier == 2,
          "two spans attach the shared seam to the preceding node");
    const auto line_pieces = curve(2, {0., 0., 0., 1., 0., 0., 2., 0., 0.});
    auto collinear = build(line_pieces);
    check(collinear.nodes.size() == 2 && collinear.nodes[0].end_seam.classifier == 2,
          "parallel spans stay separate without geometric deduplication");
    const auto broken =
        curve(2, {0., 0., 0., 1., 0., 0., 8., 0., 0., 9., 0., 0.}, {0., 0., .5, .5, 1., 1.});
    const auto discontinuous = build(broken);
    check(discontinuous.nodes.size() == 2 &&
              BsplineSurface::from_bgfb(discontinuous.nodes[1].surface).poles().front() ==
                  Point3{1, 0, 0} &&
              discontinuous.report["trace_preparation"]["replaced_leading_controls"].size() == 1,
          "native prepared shared endpoint replaces the incoming control across a discontinuity");
    const auto triangle = curve(2, {0., 0., 0., 1., 0., 0., 0., 1., 0., 0., 0., 0.});
    auto loop = build(triangle);
    check(loop.report["source_physically_closed"] == true && loop.nodes.size() == 3 &&
              loop.nodes.back().end_seam.plane &&
              (*loop.nodes.back().end_seam.plane)[0] == Point3{0, 0, 0},
          "geometric closure attaches final seam even when source closed flag is false");
    const auto bezier_loop = curve(4, {0., 0., 0., 1., 0., 0., 0., 1., 0., 0., 0., 0.});
    auto split = build(bezier_loop);
    check(split.success && split.nodes.size() == 2 &&
              split.report["special_split"]["requested"] == true &&
              split.report["special_split"]["insertion"]["added"] == 1,
          "single active interval with nonparallel closed ends inserts raw half knot");
    check(split.nodes.front().end_seam.classifier == 2 && split.nodes.back().end_seam.plane,
          "inserted smooth seam and original sharp closure keep distinct classifications");
    auto nonunit = build(curve(4, {0., 0., 0., 1., 0., 0., 0., 1., 0., 0., 0., 0.},
                               {2., 2., 2., 2., 4., 4., 4., 4.}));
    check(nonunit.success && nonunit.nodes.size() == 1 &&
              nonunit.report["special_split"]["requested"] == true &&
              nonunit.report["special_split"]["insertion"]["success"] == false &&
              nonunit.report["special_split"]["insertion"]["reason"] == "outside_domain",
          "native ignored insertion failure preserves raw .5 instead of mapping a fraction");
    const Json periodic_knots = {-1., -1., 0., 0., 0., 0., 1., 1., 1., 1., 2.};
    const auto periodic_loop =
        curve(4, {0., 0., 0., 1., 0., 0., 0., 1., 0., 0., 0., 0.}, periodic_knots, nullptr, true);
    auto periodic = build(periodic_loop);
    check(periodic.success && periodic.nodes.size() == 2 &&
              periodic.report["special_split"]["insertion"]["corrected_periodic_poles"] == true,
          "closed clamped-like path retains corrected cyclic control indexing during insertion");
    Json insertion;
    const auto inserted =
        loft_detail::insert_periodic_native_knot(periodic_loop, .5, 0, 1, 100, insertion);
    check(inserted.closed() && inserted.poles().size() == 5 &&
              inserted.knots() == std::vector<double>{-1, -1, 0, 0, 0, 0, .5, 1, 1, 1, 1, 2},
          "cyclic insertion preserves exterior knots and closed representation");
    for (unsigned i = 0; i <= 32; ++i)
        check(near(inserted.point_at(double(i) / 32), periodic_loop.point_at(double(i) / 32)),
              "periodic split retains independent source curve evaluations");
    auto mismatched =
        curve(4, {0., 0., 0., 1., 0., 0., 0., 1., 0., .5, 0., 0.}, periodic_knots, nullptr, true);
    loft_detail::insert_periodic_native_knot(mismatched, .5, 0, 1, 100, insertion);
    check(insertion["corrected_periodic_poles"] == false,
          "cyclic correction tests stored endpoint coordinates as well as knot pattern");
    // Endpoint discrepancy exceeds geometric-closure tolerance but remains
    // inside the separate 12ce40 control-range tolerance for pole correction.
    const auto flagged = build(curve(4, {0., 0., 0., 1., 0., 0., 0., 1., 0., 1e-7, 0., 0.},
                                     periodic_knots, nullptr, true));
    check(flagged.report["source_physically_closed"] == false,
          "closed-flag fixture must have distinct native endpoint positions");
    check(flagged.nodes.back().end_seam.plane.has_value(),
          "native closed flag requests final seam even without geometric endpoint agreement");
    const auto leftwrap = curve(4, {0., 0., 0., 1., 0., 0., 0., 1., 0., 0., 0., 0.},
                                {-1., -1., 0., 0., 1e-8, 2e-8, 1., 1., 1., 1., 2.}, nullptr, true);
    const auto lw = loft_detail::insert_periodic_native_knot(leftwrap, 1e-9, 0, 1, 100, insertion);
    check(insertion["control_copy"] == "left_wrap" && lw.poles().size() == 5,
          "near-start cyclic insertion replaces a control block across the left boundary");
    const auto uniform = curve(3, {0., 0., 0., 1., 0., 0., 2., 1., 0., 1., 2., 0., 0., 1., 0.},
                               nullptr, nullptr, true);
    const auto rw = loft_detail::insert_periodic_native_knot(uniform, .95, 0, 1, 100, insertion);
    check(insertion["control_copy"] == "right_wrap" && rw.poles().size() == 6,
          "near-end cyclic insertion replaces a control block across the right boundary");
    const auto unchanged =
        loft_detail::insert_periodic_native_knot(uniform, 2., 0, 1, 100, insertion);
    check(insertion["success"] == false && unchanged.knots() == uniform.knots() &&
              unchanged.poles() == uniform.poles(),
          "failed raw periodic insertion retains original curve");
    rejects([&] { loft_detail::insert_periodic_native_knot(uniform, .5, 0, 1, 5, insertion); },
            "periodic insertion bounds its output allocation");
    for (unsigned order = 2; order <= 26; ++order) {
        const unsigned n = 2 * order + 3;
        Json poles = Json::array(), weights = Json::array();
        for (unsigned i = 0; i < n; ++i) {
            poles.push_back(std::sin(double(i)));
            poles.push_back(std::cos(double(i)));
            poles.push_back(double(i % 4));
            weights.push_back(1. + .2 * (i % 5));
        }
        for (bool rational : {false, true}) {
            const auto source = curve(order, poles, nullptr, rational ? weights : Json(), true);
            const auto refined =
                loft_detail::insert_periodic_native_knot(source, .5, 0, 1, 1000, insertion);
            check(refined.closed() && refined.poles().size() == n + 1 &&
                      refined.rational() == rational && refined.order() == order,
                  "periodic insertion retains order and weight representation from 2 through 26");
            bool geometry = true;
            for (unsigned i = 0; i <= 24; ++i)
                geometry &= near(source.point_at(double(i) / 24), refined.point_at(double(i) / 24));
            check(geometry,
                  "periodic interior insertion retains source geometry across native orders");
        }
    }
    const auto weighted = curve(3, {.3, .7, .2, 1.1, 2.3, .4, 3.7, .8, .6, 4.2, 2., 1.}, nullptr,
                                {1.7, 2.9, 1.3, 2.1});
    TubeBudget wb;
    auto wc = build_tube_facet_chain(section, weighted, false, wb);
    TubeBudget pb;
    auto prep = prepare_tube_trace(weighted, pb);
    const auto sf = native_bspline_frame_working(weighted, 0);
    Matrix3 frame{};
    for (unsigned r = 0; r < 3; ++r)
        for (unsigned a = 0; a < 3; ++a)
            frame[r][a] = sf.report["frame"][a][(r + 1) % 3];
    for (std::size_t i = 0; i < prep.segments.size(); ++i) {
        TubeBudget patch_budget;
        auto p = tube_facet_patch(section, prep.segments[i], frame, false, patch_budget);
        check(wc.nodes[i].surface == *p.surface,
              "chain prepares source before first frame mutation and carries callback frame only");
        frame = p.final_frame;
    }
    check(wc.working_source_poles == sf.working_poles &&
              weighted.poles().front() == Point3{.3, .7, .2},
          "source frame working poles are returned without mutating original input");
    TubeBudget exact;
    exact.max_work = wb.work;
    check(build_tube_facet_chain(section, weighted, false, exact).nodes.size() == wc.nodes.size(),
          "chain succeeds with exact cumulative work budget");
    TubeBudget short_work;
    short_work.max_work = wb.work - 1;
    rejects([&] { build_tube_facet_chain(section, weighted, false, short_work); },
            "chain checks shared work at end tangent query too");
    TubeBudget short_controls;
    short_controls.max_control_points = 7;
    rejects([&] { build_tube_facet_chain(section, corner, false, short_controls); },
            "chain bounds total control storage across independent nodes");
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 8; ++i)
        jobs.push_back(std::async(std::launch::async,
                                  [&] { return build(bezier_loop).nodes.back().surface; }));
    for (auto &job : jobs)
        check(job.get() == split.nodes.back().surface, "facet chains are reentrant");
    return count;
}
