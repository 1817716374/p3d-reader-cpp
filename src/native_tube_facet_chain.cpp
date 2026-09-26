#include "native_tube_facet_chain.hpp"
#include "native_tube_path_placement.hpp"
#include "native_curve_conversion.hpp"
#include "native_curve_affine.hpp"
#include "native_curve_segment.hpp"
#include "native_pcurve_points.hpp"
#include "bspline_frame.hpp"
namespace p3d::swept_detail {
namespace {
double finite(double x) {
    require(std::isfinite(x), "native facet chain nonfinite arithmetic");
    return x;
}
void charge(TubeBudget &b, std::size_t n) {
    curve_detail::BezierWork{b.work, b.max_work}.charge(n);
}
Point3 cross(const Point3 &a, const Point3 &b) {
    return {finite(a[1] * b[2] - a[2] * b[1]), finite(a[2] * b[0] - a[0] * b[2]),
            finite(a[0] * b[1] - a[1] * b[0])};
}
Point3 normalized(Point3 p) {
    const double n = finite(std::sqrt((p[0] * p[0] + p[1] * p[1]) + p[2] * p[2]));
    if (!n)
        return {1, 0, 0}; // GeVec3d, not GePoint3d.
    const double inverse = finite(1 / n);
    for (auto &x : p)
        x = finite(x * inverse);
    return p;
}
detail::NativePCurvePointTangent tangent(const BsplineCurve &c, double f, TubeBudget &b) {
    charge(b, 8 * std::size_t(c.order()) * c.order() + c.knots().size());
    auto q = detail::pcurve_point_tangent(c, f);
    const auto d = c.knot_domain();
    const double span = finite(d[1] - d[0]);
    for (auto &x : q.tangent)
        x = finite(x * span);
    return q;
}
bool physically_closed(const BsplineCurve &c, TubeBudget &b) {
    charge(b, 8 * std::size_t(c.order()) * c.order() + 2 * c.knots().size());
    return curve_detail::endpoint_pair_closed(detail::pcurve_point(c, 0).point,
                                              detail::pcurve_point(c, 1).point);
}
Json knot_data(const BsplineCurve &c, TubeBudget &b) {
    std::vector<double> compressed{c.knots().front()};
    std::vector<std::size_t> counts{1};
    std::size_t left = 0, right = 0;
    for (std::size_t i = 1; i < c.knots().size(); ++i) {
        charge(b, 8);
        const double a = compressed.back(), k = c.knots()[i];
        if (std::abs(finite(a - k)) < finite(((std::abs(a) + 1) + std::abs(k)) * 1e-14))
            ++counts.back();
        else {
            compressed.push_back(k);
            counts.push_back(1);
        }
        if (i == c.order() - 1) {
            left = compressed.size() - 1;
            compressed.back() = k;
        }
        if (i == c.knots().size() - c.order()) {
            right = compressed.size() - 1;
            compressed.back() = k;
        }
    }
    return {{"values", compressed},
            {"multiplicities", counts},
            {"left_active_index", left},
            {"right_active_index", right},
            {"single_active_interval", right - left == 1}};
}
BsplineCurve split_closed_span(const BsplineCurve &source, TubeBudget &b, Json &report) {
    require(b.max_control_points <= UINT32_MAX, "native facet insertion control limit");
    for (unsigned i = 0; i < 16; ++i)
        charge(b, source.poles().size());
    charge(b, source.knots().size());
    auto poles = source.poles();
    const double tolerance = native_bspline_knot_tolerance(source, poles);
    const auto working = curve_detail::with_poles(source, poles);
    report["tolerance"] = tolerance;
    Json insertion;
    BsplineCurve result = working;
    if (source.closed()) {
        charge(b, 8 * source.knots().size() + 8 * std::size_t(source.order()) * source.order());
        result = loft_detail::insert_periodic_native_knot(
            working, .5, tolerance, 1, unsigned(b.max_control_points), insertion);
    } else {
        loft_detail::Curve c;
        c.degree = source.order() - 1;
        c.rational = source.rational();
        c.knots = source.knots();
        for (std::size_t i = 0; i < working.poles().size(); ++i) {
            const auto &p = working.poles()[i];
            c.poles.push_back({p[0], p[1], p[2], working.rational() ? working.weights()[i] : 1.});
        }
        curve_detail::insert_open_native_knot(c, .5, tolerance, 1, unsigned(b.max_control_points),
                                              {b.work, b.max_work}, insertion);
        result = BsplineCurve::from_bgfb(c.table());
    }
    // f8a90 ignores the insertion return code and still prepares this copy.
    report["insertion"] = std::move(insertion);
    return result;
}
} // namespace
TubeFacetSeam native_tube_facet_seam(Point3 incoming, Point3 outgoing, const Point3 &point) {
    for (const auto &p : {incoming, outgoing, point})
        for (double x : p)
            finite(x);
    TubeFacetSeam out;
    if (native_path_vectors_parallel(incoming, outgoing)) {
        out.classifier = 2;
        return out;
    }
    incoming = normalized(incoming);
    outgoing = normalized(outgoing);
    out.incoming = incoming;
    out.outgoing = outgoing;
    update_native_tube_facet_plane(out, point);
    return out;
}
void update_native_tube_facet_plane(TubeFacetSeam &out, const Point3 &point) {
    if (!out.incoming || !out.outgoing)
        return;
    for (const auto &p : {*out.incoming, *out.outgoing, point})
        for (double x : p)
            finite(x);
    Point3 neg{}, middle{};
    for (unsigned i = 0; i < 3; ++i) {
        neg[i] = -(*out.incoming)[i];
        middle[i] = finite(neg[i] + (*out.outgoing)[i]);
    }
    const double largest =
        std::max({std::abs(middle[0]), std::abs(middle[1]), std::abs(middle[2])});
    if (largest * 1e-12 < 2)
        for (auto &x : middle)
            x = finite(x * .5);
    const auto normal = cross(middle, cross(*out.outgoing, neg));
    if (curve_detail::endpoint_pair_closed(normal, {0, 0, 0})) {
        out.classifier = 2;
        out.incoming.reset();
        out.outgoing.reset();
    } else
        out.plane = std::array<Point3, 2>{point, normal};
}
TubeFacetChain build_tube_facet_chain(const BsplineCurve &section, const BsplineCurve &source,
                                      bool rigid, TubeBudget &budget) {
    require(source.order() <= 26 && section.order() <= 26 &&
                source.poles().size() <= budget.max_control_points &&
                section.poles().size() <= budget.max_control_points,
            "native facet chain source limits");
    TubeFacetChain out;
    // 1533e0 -> f950 copies the source for both physical-closure queries.
    // The subsequent source-frame mutation does not change that wrapper copy.
    const bool closed = physically_closed(source, budget);
    auto preparation_source = source;
    Json special = {{"requested", false}};
    if (closed) {
        special["knot_data"] = knot_data(source, budget);
        if (special["knot_data"]["single_active_interval"].get<bool>()) {
            const auto first = tangent(source, 0, budget), last = tangent(source, 1, budget);
            if (!native_path_vectors_parallel(first.tangent, last.tangent)) {
                special["requested"] = true;
                preparation_source = split_closed_span(source, budget, special);
            }
        }
    }
    // Preparation precedes the first frame query on the original source.
    auto prepared = prepare_tube_trace(preparation_source, budget);
    for (unsigned i = 0; i < 16; ++i)
        charge(budget, source.poles().size());
    charge(budget, source.knots().size() +
                       8 * std::size_t(source.order()) * source.order() * source.order() + 128);
    auto source_frame = native_bspline_frame_working(source, 0);
    out.working_source_poles = std::move(source_frame.working_poles);
    Matrix3 frame{};
    for (unsigned row = 0; row < 3; ++row)
        for (unsigned axis = 0; axis < 3; ++axis)
            frame[row][axis] = source_frame.report.at("frame").at(axis).at((row + 1) % 3);
    // A native section may reference the same curve as the trace. The first
    // source-frame query precedes section reads by the patch callback.
    std::optional<BsplineCurve> shared_section;
    if (&section == &source)
        shared_section = curve_detail::with_poles(section, out.working_source_poles);
    const auto &effective_section = shared_section ? *shared_section : section;
    const auto first_tangent = frame[2];
    out.report = {{"scope", "native_independent_facet_chain"},
                  {"source_physically_closed", closed},
                  {"special_split", special},
                  {"trace_preparation", prepared.report},
                  {"source_frame", source_frame.report},
                  {"seams_applied", false}};
    Json patches = Json::array();
    Point3 previous_end{};
    std::size_t controls = 0;
    for (std::size_t i = 0; i < prepared.segments.size(); ++i) {
        auto &segment = prepared.segments[i];
        if (i) {
            const auto start = tangent(segment, 0, budget);
            out.nodes.back().end_seam =
                native_tube_facet_seam(previous_end, start.tangent, start.value.point);
        }
        auto patch = tube_facet_patch(effective_section, segment, frame, rigid, budget);
        frame = patch.final_frame;
        patches.push_back(std::move(patch.report));
        if (!patch.success) {
            out.nodes.clear();
            out.final_frame = frame;
            out.report["status"] = "native_failure";
            out.report["patches"] = std::move(patches);
            out.report["work_used"] = budget.work;
            return out;
        }
        const auto count = patch.surface->at("numPolesU").get<std::size_t>() *
                           patch.surface->at("numPolesV").get<std::size_t>();
        require(count <= budget.max_control_points - controls,
                "native facet chain cumulative control budget");
        controls += count;
        out.nodes.push_back({std::move(*patch.surface), {}});
        // cf5a0's private copy does not mutate this stack Bezier.
        const auto end = tangent(segment, 1, budget);
        previous_end = end.tangent;
        if (i + 1 == prepared.segments.size()) {
            if (source.closed() || closed)
                out.nodes.back().end_seam =
                    native_tube_facet_seam(previous_end, first_tangent, end.value.point);
            else
                out.nodes.back().end_seam.classifier = 2;
        }
    }
    out.final_frame = frame;
    out.success = true;
    out.report["patches"] = std::move(patches);
    out.report["status"] = "complete";
    out.report["control_points"] = controls;
    out.report["work_used"] = budget.work;
    return out;
}
} // namespace p3d::swept_detail
