#include "native_tube_mesh_path.hpp"
#include <future>
#include <limits>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
BsplineCurve curve(Json poles, Json weights = nullptr) {
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", 2},
                                    {"closed", false},
                                    {"poles", poles},
                                    {"weights", weights},
                                    {"knots", nullptr}});
}
TubeMeshControlRange range(const BsplineCurve &c) {
    TubeBudget b;
    return tube_mesh_control_range(c, b);
}
Json surface(bool pin_start = false, bool flat = false) {
    Json p = Json::array();
    for (unsigned v = 0; v < 3; ++v)
        for (unsigned u = 0; u < 2; ++u) {
            p.push_back(u);
            p.push_back(flat ? 0 : double(v) * .5 * (pin_start ? u : u + 1));
            p.push_back(flat ? 0 : (v == 2 ? (pin_start ? u : 1) : 0));
        }
    return {{"_type", "BsplineSurface"},
            {"orderU", 2},
            {"orderV", 3},
            {"numPolesU", 2},
            {"numPolesV", 3},
            {"closedU", false},
            {"closedV", false},
            {"poles", p},
            {"weights", nullptr},
            {"knotsU", nullptr},
            {"knotsV", nullptr},
            {"numRulesU", 0},
            {"numRulesV", 0},
            {"holeOrigin", 0},
            {"boundaries", nullptr}};
}
TubeMeshPathSamples sample(const std::vector<BsplineSurface> &s, bool closed, double chord = .07,
                           double angle = 0) {
    TubeBudget b;
    return sample_tube_mesh_path(s, closed, chord, angle, 100000, b);
}
Json group(Json values, unsigned type = 1) {
    Json members = Json::array();
    for (const auto &v : values)
        members.push_back({{"geometry", v}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", members}};
}
Json line(Point3 a, Point3 b) {
    Json p;
    for (unsigned i = 0; i < 3; ++i) {
        p[std::string("point0") + "XYZ"[i]] = a[i];
        p[std::string("point1") + "XYZ"[i]] = b[i];
    }
    return {{"_type", "LineSegment"}, {"segment", p}};
}
bool near(double a, double b) {
    return std::abs(a - b) < 1e-10;
}
} // namespace
unsigned native_tube_mesh_path_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "native path/grid preparation enforces finite arithmetic and budgets");
    };
    const auto c = curve({0, 0, 0, 2, 3, 4});
    auto r = range(c);
    check(!r.degenerate && !r.native_null && r.squared_extent == 29 && r.accepted == 2,
          "control-range diagonal includes all three axes");
    auto loop = range(curve({0, 0, 0, 2, 3, 4, 0, 0, 0}));
    check(loop.squared_extent == 29 && !loop.degenerate,
          "closed endpoints do not imply degenerate curve");
    auto constant = range(curve({2, 4, 6, 4, 8, 12}, {2, 4}));
    check(constant.degenerate && constant.low == Point3{1, 2, 3} && constant.high == constant.low,
          "weighted control range deweights with reciprocal multiplication");
    auto mixed = range(curve({2, 4, 6, -8, -4, -2}, {2, -2}));
    check(mixed.squared_extent == 13 && mixed.high == Point3{4, 2, 3},
          "negative weights retain projected range");
    auto tiny = range(curve({0, 0, 0, 1e-6, 0, 0}));
    check(tiny.degenerate && tiny.squared_extent > 0,
          "native near-zero predicate applies to squared extent");
    check(!range(curve({0, 0, 0, 1e-4, 0, 0})).degenerate,
          "visible control range survives near-zero predicate");
    auto ignored = range(curve({2, 3, 4, 5, 6, 7}, {1e-12, -1e-12}));
    check(ignored.native_null && ignored.degenerate && ignored.rejected_weights == 2,
          "weights at signed native threshold are skipped inclusively");
    auto partial = range(curve({1, 2, 3, 4, 5, 6}, {0, 2}));
    check(partial.accepted == 1 && partial.rejected_weights == 1 && partial.degenerate,
          "one remaining weighted control is a zero-sized range");
    const double max = std::numeric_limits<double>::max();
    auto disconnected = range(curve({max, 0, 0, 2, 3, 4}));
    check(disconnected.disconnected == 1 && disconnected.low == Point3{2, 3, 4},
          "disconnect sentinel in any axis skips the complete point");
    auto huge = range(curve({0, 0, 0, 1e100, 0, 0}));
    check(huge.native_null && huge.squared_extent == 0 && huge.degenerate,
          "native range magnitude cutoff returns zero extent before squaring");
    auto allzero = range(curve({1, 2, 3, 4, 5, 6}, {0, 0}));
    check(allzero.native_null && allzero.accepted == 0, "all-zero weights retain null range");
    const auto s = BsplineSurface::from_bgfb(surface());
    auto open = sample({s}, false), closed = sample({s}, true);
    check(open.success && closed.success && open.curves.size() == 3 && closed.curves.size() == 3,
          "single-strip sampling retains three native selectors");
    check(open.curves[0].u == 0 && open.curves[1].u == .5 && open.curves[2].u == 1,
          "open single strip uses both ends and midpoint");
    check(closed.curves[0].u == 0 && closed.curves[1].u == .325 && closed.curves[2].u == .75,
          "closed single strip preserves 0.325 rather than replacing it by one third");
    for (const auto &selected : closed.curves)
        for (double v : {0., .2, .8, 1.}) {
            const auto p = selected.curve.point_at(v);
            check(near(p[0], selected.u) && near(p[1], (selected.u + 1) * v) && near(p[2], v * v),
                  "selected V curves agree with analytic tensor-product surface");
        }
    check(open.parameters == std::vector<double>{0, .5, 1} && closed.parameters == open.parameters,
          "shared V tree matches independent parabola chord-error formula");
    auto multiple = sample({s, s}, false), multiple_closed = sample({s, s}, true);
    check(multiple.success && multiple.curves.size() == 5 && multiple_closed.curves.size() == 4,
          "multi-strip selectors retain equal independent strips and conditional last endpoint");
    check(multiple.curves[0].strip == 0 && multiple.curves[1].u == .5 &&
              multiple.curves[2].strip == 1 && multiple.curves[2].u == 0 &&
              multiple.curves[4].strip == 1 && multiple.curves[4].u == 1,
          "multi-strip source indices and traversal order remain stable");
    const auto pinned = BsplineSurface::from_bgfb(surface(true));
    auto filtered = sample({pinned}, false);
    check(filtered.success && filtered.curves.size() == 2 &&
              filtered.curves[0].selection_index == 1 && filtered.curves[1].selection_index == 2 &&
              filtered.report["selections"][0]["removed"] == true,
          "degenerate curve removal preserves remaining native selection indices and order");
    auto flat = BsplineSurface::from_bgfb(surface(false, true));
    auto none = sample({flat}, false);
    check(!none.success && none.parameters.empty() &&
              none.report["failure"] == "all_selected_curves_degenerate",
          "mesh caller rejects an entirely degenerate selection before shared-tree construction");
    auto weights = surface();
    weights["weights"] = std::vector<double>(6, 0.);
    auto no_weights = sample({BsplineSurface::from_bgfb(weights)}, false);
    check(!no_weights.success &&
              no_weights.report["selections"][0]["range"]["rejected_weights"] == 3,
          "isocurve zero-weight fallback does not bypass native range filtering");
    check(!sample({}, false).success, "empty strip vector is not a successfully sampled patch");
    auto no_tolerance = sample({s}, false, 0, 0);
    check(!no_tolerance.success && no_tolerance.parameters.empty() &&
              no_tolerance.report["failure"] == "adaptive_sampling",
          "shared-tree failure remains explicit");
    SweptBodyPatchGroup synthetic;
    synthetic.source_curve = 17;
    synthetic.patches.push_back({surface(), {}});
    synthetic.patches.push_back({surface(), {}});
    TubeBudget gb;
    auto grid = prepare_tube_mesh_grid(synthetic, false, .07, 0, 100000, gb);
    check(grid.success &&
              grid.section.interval_samples == std::vector<std::vector<double>>{{0, 1}} &&
              grid.patches.size() == 2 && grid.report["source_curve"] == 17 &&
              grid.report["mesh_generated"] == false,
          "group preparation combines first-patch U sampling with every original patch");
    check(grid.patches[0].path.parameters == open.parameters &&
              grid.patches[1].path.parameters == open.parameters,
          "independent patches retain their own shared V parameter lists");
    auto mismatch = synthetic;
    auto extra = surface();
    extra["numPolesU"] = 3;
    extra["knotsU"] = nullptr;
    extra["poles"] = {0, 0, 0, .5, 0, 0, 1, 0,  0,   0, .5, 0, .5, .75,
                      0, 1, 1, 0,  0, 1, 1, .5, 1.5, 1, 1,  2, 1};
    mismatch.patches[1].geometry = extra;
    TubeBudget mb;
    auto bad = prepare_tube_mesh_grid(mismatch, false, .07, 0, 100000, mb);
    check(!bad.success && bad.patches.size() == 1 &&
              bad.report["failure_step"] == "strip_section_count",
          "later patch strip-count mismatch retains diagnostics without publishing complete group");
    TubeBudget eb;
    check(!prepare_tube_mesh_grid({}, false, .1, 0, 1000, eb).success,
          "empty patch group explicitly fails");
    auto invalid = synthetic;
    invalid.patches[1].boundary_points = {{{.5, 0}, {1, 0}}};
    TubeBudget ib;
    auto invalid_result = prepare_tube_mesh_grid(invalid, false, .07, 0, 100000, ib);
    check(!invalid_result.success && invalid_result.patches.size() == 1 &&
              invalid_result.report["failure_step"] == "patch_preparation",
          "native boundary rejection stops later patch preparation");
    const Json profile = group({{{"_type", "LineString"},
                                 {"points", {-1, -1, 0, 1, -1, 0, 1, 1, 0, -1, 1, 0, -1, -1, 0}}}},
                               2);
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
                     {"sweepRadians", std::acos(-1.)}}}};
    auto actual = reconstruct_bgfb_swept_body_patches(
        {{"_type", "P3DSweptBody"},
         {"profile", profile},
         {"path", group({arc, line({-20, 0, 0}, {-25, 0, 0})})}});
    check(actual.status == "reconstructed" && actual.groups.size() == 1,
          "actual arc-line path fixture reconstructs");
    TubeBudget ab;
    auto actual_grid = prepare_tube_mesh_grid(actual.groups[0], true, .01, .2, 100000, ab);
    check(actual_grid.success && actual_grid.patches.size() == actual.groups[0].patches.size() &&
              actual_grid.section.interval_samples.size() == 4,
          "actual trimmed path patches share the four original U intervals");
    bool refined = false;
    for (const auto &p : actual_grid.patches) {
        check(p.path.success && p.preparation.strips.size() == 4 && p.path.curves.size() == 8,
              "each actual closed-profile patch samples eight native V selection curves");
        refined = refined || p.path.parameters.size() > 2;
    }
    check(refined, "actual curved path requires adaptive V interior parameters");
    rejects([&] {
        TubeBudget b;
        b.max_work = 0;
        sample_tube_mesh_path({s}, false, .1, 0, 1000, b);
    });
    rejects([&] {
        TubeBudget b;
        b.max_control_points = 8;
        sample_tube_mesh_path({s}, false, .1, 0, 1000, b);
    });
    rejects([&] {
        TubeBudget b;
        sample_tube_mesh_path({s}, false, .1, 0, 2, b);
    });
    rejects([&] {
        TubeBudget b;
        b.max_control_points = 10;
        prepare_tube_mesh_grid(synthetic, false, .1, 0, 1000, b);
    });
    auto future = std::async(std::launch::async, [&] { return sample({s}, false).parameters; });
    check(future.get() == open.parameters &&
              s.poles() == BsplineSurface::from_bgfb(surface()).poles(),
          "concurrent independent path sampling preserves immutable input");
    return n;
}
