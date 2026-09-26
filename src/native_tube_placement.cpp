#include "native_tube.hpp"
#include "bspline_frame.hpp"
#include "native_curve_affine.hpp"
#include "native_pcurve_points.hpp"
#include "native_tube_orientation.hpp"

namespace p3d::swept_detail {
namespace {
void charge(TubeBudget &budget, std::size_t n) {
    require(budget.work <= budget.max_work && n <= budget.max_work - budget.work,
            "native swept placement work budget exceeded");
    budget.work += n;
}
double finite(double x) {
    require(std::isfinite(x), "native swept placement nonfinite arithmetic");
    return x;
}
} // namespace

TubePlacement place_tube_profile(const Json &profile, const BsplineCurve &trace,
                                 TubeBudget &budget) {
    require(trace.order() <= 26 && trace.poles().size() <= budget.max_control_points &&
                trace.poles().size() <= INT32_MAX,
            "native swept placement trace budget exceeded");
    for (unsigned i = 0; i < 13; ++i)
        charge(budget, trace.poles().size());
    charge(budget, std::size_t(trace.order()) * trace.order() * trace.order());
    const auto source_frame = native_bspline_frame_working(trace, 0);
    auto working = curve_detail::with_poles(trace, source_frame.working_poles);
    Point3 origin = working.poles().front();
    if (working.closed())
        origin = detail::pcurve_point(working, 0).point;
    else if (working.rational()) {
        const double inverse = finite(1 / working.weights().front());
        for (auto &x : origin)
            x = finite(x * inverse);
    }
    Matrix3 linear{};
    Matrix4 frame{};
    for (unsigned axis = 0; axis < 3; ++axis) {
        for (unsigned c = 0; c < 3; ++c)
            frame[axis][c] = linear[axis][c] =
                source_frame.report.at("frame").at(axis).at((c + 1) % 3).get<double>();
        frame[axis][3] = origin[axis];
    }
    frame[3][3] = 1;
    const auto inverse = native_matrix_inverse(linear);
    Matrix4 placement{};
    for (unsigned r = 0; r < 4; ++r)
        placement[r][r] = 1;
    if (inverse.inverted)
        for (unsigned r = 0; r < 3; ++r) {
            for (unsigned c = 0; c < 3; ++c)
                placement[r][c] = inverse.matrix[r][c];
            // Native affine inverse negates origin components before their
            // products, then accumulates Y, X, Z. Do not negate a finished dot.
            placement[r][3] =
                finite(((-origin[1] * placement[r][1]) + (-origin[0] * placement[r][0])) +
                       (-origin[2] * placement[r][2]));
        }
    const bool skip = curve_detail::bspline_identity(placement);
    // Conversion precedes transformation. In particular, fitting an already
    // transformed LineString would change its native chord and error tests.
    auto converted = convert_tube_profile(profile, budget);
    for (auto &section : converted.curves) {
        for (unsigned i = 0; i < 8; ++i)
            charge(budget, section.poles().size());
        if (!skip) {
            auto poles = section.poles();
            for (std::size_t i = 0; i < poles.size(); ++i)
                poles[i] = curve_detail::affine_point(
                    placement, poles[i], section.rational() ? section.weights()[i] : 1.);
            section = curve_detail::with_poles(section, poles);
        }
    }
    return {std::move(working),
            std::move(converted.curves),
            {{"scope", "native_swept_profile_placement"},
             {"source_frame", source_frame.report},
             {"origin_rule", trace.closed() ? "closed_curve_point" : "first_stored_control"},
             {"frame", frame},
             {"placement", placement},
             {"inverse_succeeded", inverse.inverted},
             {"inverse_method", inverse.method},
             {"identity_transform_skipped", skip},
             {"profile_conversion", std::move(converted.report)},
             {"work_used", budget.work},
             {"source_geometry_reused", false}}};
}

TubeSurfaces prepare_swept_tube_surfaces(const Json &profile, const Json &path,
                                         TubeBudget &budget) {
    auto converted_path = convert_tube_curve_array(path, budget);
    auto placed = place_tube_profile(profile, converted_path.curve, budget);
    std::vector<Json> surfaces;
    Json rings = Json::array();
    std::size_t total = 0;
    for (std::size_t i = 0; i < placed.sections.size(); ++i) {
        auto tube = tube_surface(placed.sections[i], placed.trace, false, budget);
        // All source rings refer to the same native path working object. Do
        // not reset it to the original controls between calls.
        for (unsigned j = 0; j < 8; ++j)
            charge(budget, placed.trace.poles().size());
        placed.trace = curve_detail::with_poles(placed.trace, tube.working_trace_poles);
        Json closure = nullptr;
        if (placed.trace.closed()) {
            auto closed = close_tube_surface_v(BsplineSurface::from_bgfb(tube.surface), budget);
            tube.surface = std::move(closed.surface);
            closure = std::move(closed.report);
        }
        const auto count = tube.surface.at("poles").size() / 3;
        require(count <= budget.max_control_points - total,
                "native swept surfaces total control budget exceeded");
        total += count;
        surfaces.push_back(std::move(tube.surface));
        rings.push_back({{"source_ring_index", i},
                         {"tube", std::move(tube.report)},
                         {"v_closure", std::move(closure)}});
    }
    auto orientation = orient_tube_surfaces_from_trace(surfaces, placed.trace, budget);
    const bool complete = orientation.report.at("completed_all_rings");
    const bool native_result = orientation.report.at("native_result");
    return {std::move(orientation.surfaces),
            std::move(placed.trace),
            {{"scope", "native_swept_surfaces_with_orientation"},
             {"path_conversion", std::move(converted_path.report)},
             {"placement", std::move(placed.report)},
             {"rings", std::move(rings)},
             {"total_control_points", total},
             {"work_used", budget.work},
             {"source_validation", "native_geom_num_dispatch_not_applied"},
             {"orientation_applied", complete},
             {"native_generation_result", native_result},
             {"orientation", std::move(orientation.report)},
             {"caps_generated", false},
             {"native_face_indices", nullptr},
             {"surface_validity", "not_certified"}}};
}
} // namespace p3d::swept_detail
