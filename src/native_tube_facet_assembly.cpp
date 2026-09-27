#include "native_tube_facet_assembly.hpp"
#include "native_tube_facet_extension.hpp"
#include "native_pcurve_points.hpp"
#include "native_bezier.hpp"
#include "native_curve_affine.hpp"
namespace p3d::swept_detail {
namespace {
void charge(TubeBudget &b, std::size_t n) {
    curve_detail::BezierWork{b.work, b.max_work}.charge(n);
}
void append(std::vector<Point2> &out, Point2 p, TubeBudget &b) {
    require(out.size() < b.max_control_points, "native facet boundary point budget");
    require(std::isfinite(p[0]) && std::isfinite(p[1]), "native facet boundary overflow");
    charge(b, 1);
    out.push_back(p);
}
std::vector<Point2> first_boundary(const TubeFacetSurface &s, TubeBudget &b) {
    if (s.boundaries.empty())
        return {};
    require(s.boundaries[0].size() <= b.max_control_points, "native facet boundary copy budget");
    charge(b, s.boundaries[0].size());
    return s.boundaries[0];
}
// 107080, called by 1076e0 when only one combination input has trim records:
// build four unit-square pcurves, then restroke with the source surface's knots.
void add_unit_boundary(TubeFacetSurface &s, TubeBudget &b, Json &report) {
    require(s.boundaries.empty() && s.pcurves.empty(), "native unit trim needs untrimmed surface");
    charge(b, 8);
    const Point3 corners[]{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 0}};
    std::vector<BsplineCurve> loop;
    for (unsigned i = 0; i < 4; ++i) {
        const auto &a = corners[i], &z = corners[i + 1];
        loop.push_back(BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                                {"order", 2},
                                                {"closed", false},
                                                {"poles", {a[0], a[1], a[2], z[0], z[1], z[2]}},
                                                {"weights", nullptr},
                                                {"knots", {0, 0, 1, 1}}}));
    }
    require(b.work < b.max_work, "native unit trim evaluation budget");
    PCurveLoopStrokeOptions options;
    options.max_points =
        static_cast<unsigned>(std::min<std::size_t>(b.max_control_points, UINT_MAX));
    options.max_evaluations =
        static_cast<unsigned>(std::min<std::size_t>(b.max_work - b.work, UINT_MAX));
    const auto strokes =
        detail::sample_initial_pcurve_loops(BsplineSurface::from_bgfb(s.geometry), {loop}, options);
    charge(b, strokes.report.at("evaluations").get<std::size_t>());
    require(strokes.report.at("status") == "complete" && strokes.loops.size() == 1,
            "native unit boundary restroking incomplete");
    std::vector<Point2> points;
    for (const auto &sample : strokes.loops[0])
        append(points, {sample.parameter[0], sample.parameter[1]}, b);
    s.boundaries.push_back(std::move(points));
    s.pcurves.push_back(std::move(loop));
    s.geometry["holeOrigin"] = 1;
    report = strokes.report;
}
void copy_transformed_trims(TubeFacetSurface &out, const TubeFacetSurface &source, double scale,
                            double shift, TubeBudget &b) {
    require(source.pcurves.empty() || source.pcurves.size() == source.boundaries.size(),
            "native facet trim curve record layout");
    const Matrix4 transform{{{1, 0, 0, 0}, {0, scale, 0, shift}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
    const bool skip_curves = curve_detail::bspline_identity(transform);
    for (std::size_t i = 0; i < source.boundaries.size(); ++i) {
        require(out.boundaries.size() < b.max_control_points, "native combined trim count budget");
        std::vector<Point2> points;
        for (const auto &p : source.boundaries[i])
            // Base3157 transforms cached XY even for a near-identity matrix.
            append(points, {(p[0] + 0.) + 0. * p[1], (0. * p[0] + shift) + scale * p[1]}, b);
        out.boundaries.push_back(std::move(points));
        out.pcurves.emplace_back();
        if (source.pcurves.empty())
            continue;
        for (const auto &curve : source.pcurves[i]) {
            require(curve.poles().size() <= b.max_control_points,
                    "native trim curve control budget");
            auto poles = curve.poles();
            charge(b, poles.size());
            if (!skip_curves)
                for (std::size_t j = 0; j < poles.size(); ++j) {
                    charge(b, 32);
                    poles[j] =
                        curve.rational()
                            ? curve_detail::affine_point(transform, poles[j], curve.weights()[j])
                            : curve_detail::affine_polynomial_point(transform, poles[j]);
                }
            out.pcurves.back().push_back(curve_detail::with_poles(curve, poles));
        }
    }
}
} // namespace
std::vector<Point2> splice_tube_facet_boundaries(const std::vector<Point2> &first,
                                                 const std::vector<Point2> &second, TubeBudget &b) {
    require(first.size() <= b.max_control_points && second.size() <= b.max_control_points,
            "native facet boundary input budget");
    if ((first.empty() && second.empty()) || (!first.empty() && first.size() < 5) ||
        (!second.empty() && second.size() < 5))
        return {};
    std::vector<Point2> out;
    if (first.empty()) {
        append(out, {0, 0}, b);
        append(out, {1, 0}, b);
    } else {
        for (std::size_t i = 0; i < first.size() - 3; ++i)
            append(out, {first[i][0], first[i][1] * .5}, b);
    }
    if (second.empty()) {
        append(out, {1, 1}, b);
        append(out, {0, 1}, b);
    } else {
        for (std::size_t i = 2; i < second.size() - 1; ++i) {
            const double v = second[i][1] * .5;
            append(out, {second[i][0], v + .5}, b);
        }
    }
    append(out, out.front(), b);
    return out;
}
TubeFacetGroupAssembly assemble_tube_facet_groups(TubeFacetGroupGeneration generated,
                                                  TubeFacetPathClassification classified,
                                                  TubeBudget &b) {
    TubeFacetGroupAssembly out;
    out.generation = std::move(generated);
    out.classification = std::move(classified);
    out.report = {{"scope", "native_facet_group_assembly"},
                  {"native_result", false},
                  {"operations", Json::array()}};
    auto failure = [&](const char *reason) {
        out.report["reason"] = reason;
        out.report["completed_profile_groups"] = out.groups.size();
        out.report["work_used"] = b.work;
        return std::move(out);
    };
    if (!out.generation.success || !out.classification.success)
        return failure("generation_or_classification_failed");
    const auto &classes = out.classification.groups;
    require(out.generation.groups.size() <= b.max_control_points &&
                classes.size() <= b.max_control_points,
            "native facet assembly group budget");
    for (std::size_t g = 0; g < out.generation.groups.size(); ++g) {
        auto &members = out.generation.groups[g];
        std::vector<std::vector<std::size_t>> completed;
        require(members.size() <= b.max_control_points, "native facet assembly member budget");
        for (std::size_t m = 0; m < members.size(); ++m) {
            charge(b, 1);
            auto &surfaces = members[m].surfaces;
            require(!classes.empty() && !classes.back().empty(),
                    "native facet assembly cannot read absent last path index");
            if (surfaces.empty() || classes.back().back() < 0 ||
                static_cast<std::size_t>(classes.back().back()) != surfaces.size() - 1)
                return failure("path_classification_surface_count_mismatch");
            require(surfaces.size() <= b.max_control_points,
                    "native facet assembly surface budget");
            std::vector<std::size_t> indices;
            for (std::size_t c = 0; c < classes.size(); ++c) {
                const auto &path_group = classes[c];
                require(!path_group.empty() && path_group.size() <= b.max_control_points,
                        "native facet assembly empty or excessive path group");
                auto index = [&](std::size_t i) {
                    const auto n = path_group[i];
                    require(n >= 0 && static_cast<std::size_t>(n) < surfaces.size(),
                            "native facet assembly surface index out of bounds");
                    return static_cast<std::size_t>(n);
                };
                auto current = index(0);
                bool linear = surfaces[current].geometry.at("orderV") == 2;
                for (std::size_t i = 1; i < path_group.size(); ++i) {
                    const auto next = index(i);
                    charge(b, 1);
                    auto first = first_boundary(surfaces[current], b);
                    auto second = first_boundary(surfaces[next], b);
                    const bool next_linear = surfaces[next].geometry.at("orderV") == 2;
                    Json operation{{"group", g},
                                   {"member", m},
                                   {"path_group", c},
                                   {"first_surface", current},
                                   {"next_surface", next}};
                    if (linear != next_linear) {
                        indices.push_back(current);
                        current = next;
                        linear = next_linear;
                        operation["action"] = "split_at_v_order_two_transition";
                    } else {
                        // The native helper may create trim curves on either SOURCE
                        // before replacing current. Preserve that incoming-side effect.
                        const auto left = BsplineSurface::from_bgfb(surfaces[current].geometry);
                        auto joined = combine_tube_surfaces_v(
                            left, BsplineSurface::from_bgfb(surfaces[next].geometry), b);
                        const bool has_trim = !surfaces[current].boundaries.empty() ||
                                              !surfaces[next].boundaries.empty();
                        if (has_trim) {
                            if (surfaces[current].boundaries.empty())
                                add_unit_boundary(surfaces[current], b,
                                                  operation["first_unit_trim"]);
                            if (surfaces[next].boundaries.empty())
                                add_unit_boundary(surfaces[next], b, operation["next_unit_trim"]);
                        }
                        joined.surface["holeOrigin"] = surfaces[current].geometry.at("holeOrigin");
                        TubeFacetSurface combined{std::move(joined.surface), {}};
                        if (has_trim) {
                            // 107910 accumulates each column's same knot plan; the
                            // trim helper divides the accumulated join by nU.
                            const double knot = combined.geometry.at("knotsV")
                                                    .at(left.v().pole_count())
                                                    .get<double>();
                            double sum = knot;
                            for (std::size_t u = 1; u < left.u().pole_count(); ++u) {
                                charge(b, 1);
                                sum += knot;
                            }
                            const double fraction = sum / double(left.u().pole_count());
                            require(std::isfinite(fraction),
                                    "native combined trim fraction overflow");
                            copy_transformed_trims(combined, surfaces[current], fraction, 0, b);
                            copy_transformed_trims(combined, surfaces[next], 1 - fraction, fraction,
                                                   b);
                        }
                        surfaces[current] = std::move(combined);
                        operation["action"] = "combine_along_v";
                    }
                    auto &target = surfaces[current];
                    if (!target.boundaries.empty()) {
                        if (first.empty() && second.empty()) {
                            out.report["operations"].push_back(std::move(operation));
                            return failure("trim_records_have_no_first_boundary_points");
                        }
                        target.boundaries.clear();
                        target.pcurves.clear();
                        auto boundary = splice_tube_facet_boundaries(first, second, b);
                        if (boundary.empty()) {
                            out.report["operations"].push_back(std::move(operation));
                            return failure("boundary_splice_returned_empty");
                        }
                        target.boundaries.push_back(std::move(boundary));
                        target.geometry["holeOrigin"] = 1;
                        operation["boundary_rebuilt"] = true;
                    } else
                        operation["boundary_rebuilt"] = false;
                    out.report["operations"].push_back(std::move(operation));
                }
                indices.push_back(current);
            }
            completed.push_back(std::move(indices));
        }
        out.groups.push_back(std::move(completed));
    }
    out.success = true;
    out.report["native_result"] = true;
    out.report["work_used"] = b.work;
    return out;
}
TubeFacetGroupAssembly assemble_tube_facet_groups(TubeFacetGroupGeneration generated,
                                                  TubeBudget &b) {
    if (!generated.success) {
        TubeFacetPathClassification unavailable;
        unavailable.report = {{"reason", "surface_generation_failed"}};
        return assemble_tube_facet_groups(std::move(generated), std::move(unavailable), b);
    }
    auto classified = classify_generated_tube_facet_groups(generated, b);
    return assemble_tube_facet_groups(std::move(generated), std::move(classified), b);
}
} // namespace p3d::swept_detail
