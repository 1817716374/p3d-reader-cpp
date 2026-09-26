#include "native_tube_path_placement.hpp"
#include "native_curve_segment.hpp"
#include "native_curve_closest.hpp"
#include "native_curve_affine.hpp"
#include "native_pcurve_points.hpp"
#include "bspline_frame.hpp"
namespace p3d::swept_detail {
namespace {
double finite(double x) {
    require(std::isfinite(x), "native path placement nonfinite arithmetic");
    return x;
}
bool near(double a, double b) {
    return std::abs(finite(a - b)) <= finite(((std::abs(a) + 1) + std::abs(b)) * 1e-10);
}
double dot(const Point3 &a, const Point3 &b) {
    return finite((finite(a[1] * b[1]) + finite(a[0] * b[0])) + finite(a[2] * b[2]));
}
double squared(const Point3 &p) {
    return finite((finite(p[0] * p[0]) + finite(p[1] * p[1])) + finite(p[2] * p[2]));
}
Point3 cross(const Point3 &a, const Point3 &b) {
    return {finite(finite(a[1] * b[2]) - finite(a[2] * b[1])),
            finite(finite(a[2] * b[0]) - finite(a[0] * b[2])),
            finite(finite(a[0] * b[1]) - finite(a[1] * b[0]))};
}
void normalize(Point3 &p, bool vector_fallback) {
    const double length = finite(std::sqrt(squared(p)));
    if (length > 0) {
        const double inverse = finite(1 / length);
        for (auto &x : p)
            x = finite(x * inverse);
    } else if (vector_fallback)
        p = {1, 0, 0};
}
Point3 column(const Matrix4 &m, unsigned index) {
    return {m[0][index], m[1][index], m[2][index]};
}
Matrix4 transform(const Point3 &origin, const Point3 &x, const Point3 &y, const Point3 &z) {
    Matrix4 m{};
    for (unsigned i = 0; i < 3; ++i)
        m[i] = {x[i], y[i], z[i], origin[i]};
    m[3][3] = 1;
    return m;
}
Matrix4 section_frame(const Matrix4 &f) {
    return transform(column(f, 3), column(f, 1), column(f, 2), column(f, 0));
}
Matrix4 inverse(const Matrix4 &f, Json &report) {
    Matrix3 linear{};
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 3; ++j)
            linear[i][j] = f[i][j];
    const auto result = native_matrix_inverse(linear);
    Matrix4 m{};
    for (unsigned i = 0; i < 4; ++i)
        m[i][i] = 1;
    if (result.inverted)
        for (unsigned i = 0; i < 3; ++i) {
            for (unsigned j = 0; j < 3; ++j)
                m[i][j] = result.matrix[i][j];
            m[i][3] = finite((finite(-f[1][3] * m[i][1]) + finite(-f[0][3] * m[i][0])) +
                             finite(-f[2][3] * m[i][2]));
        }
    report = {{"frame", f},
              {"inverse", m},
              {"inverse_succeeded", result.inverted},
              {"inverse_method", result.method}};
    return m; // Native writes identity on failure; caller ignores that status.
}
Matrix4 product(const Matrix4 &a, const Matrix4 &b) {
    Matrix4 out{};
    out[3][3] = 1;
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j < 3; ++j)
            out[i][j] = finite((finite(a[i][0] * b[0][j]) + finite(a[i][1] * b[1][j])) +
                               finite(a[i][2] * b[2][j]));
        out[i][3] = finite(((finite(a[i][0] * b[0][3]) + a[i][3]) + finite(a[i][1] * b[1][3])) +
                           finite(a[i][2] * b[2][3]));
    }
    return out;
}
Matrix3 rotation_product(const Matrix3 &a, const Matrix3 &b) {
    Matrix3 out{};
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 3; ++j)
            out[i][j] = finite((finite(a[i][0] * b[0][j]) + finite(a[i][1] * b[1][j])) +
                               finite(a[i][2] * b[2][j]));
    return out;
}
void charge(const BsplineCurve &c, TubeBudget &b) {
    require(c.order() <= 26 && c.poles().size() <= b.max_control_points,
            "native path frame control/order limit");
    curve_detail::BezierWork w{b.work, b.max_work};
    for (unsigned i = 0; i < 16; ++i)
        w.charge(c.poles().size());
    w.charge(c.knots().size());
    w.charge(8 * std::size_t(c.order()) * c.order() * c.order() + 128);
}
Matrix4 frame(BsplineCurve &c, double f, TubeBudget &b, Json &report) {
    charge(c, b);
    auto evaluated = native_bspline_frame_working(c, f);
    c = curve_detail::with_poles(c, evaluated.working_poles);
    report = std::move(evaluated.report);
    report["fraction"] = f;
    return report.at("frame").get<Matrix4>();
}
BsplineCurve &geometry(TubeFacetPathBranches &p, TubePathBranch &branch) {
    require(!branch.empty_curve_object, "native path frame cannot read an empty curve object");
    if (branch.reused_curve_index) {
        require(*branch.reused_curve_index < p.path.selection.curves.size(),
                "native path branch reference");
        return p.path.selection.curves[*branch.reused_curve_index];
    }
    require(branch.constructed.has_value(), "native path frame missing branch");
    return *branch.constructed;
}
bool present(const TubePathBranch &b) {
    return b.empty_curve_object || b.constructed || b.reused_curve_index;
}
bool relocate(const BsplineCurve &c, const Point3 &fixed, double &f, TubeBudget &b, Json &report) {
    charge(c, b);
    const auto closest = curve_detail::native_curve_closest_point(c, fixed, {b.work, b.max_work});
    require(closest.found, "native branch closest query has no initialized result");
    f = closest.fraction;
    const bool accepted = native_path_points_equal(fixed, closest.point);
    report = {{"fraction", f}, {"point", closest.point}, {"target", fixed}, {"accepted", accepted}};
    return accepted;
}
Point3 tangent(const BsplineCurve &c, double f, TubeBudget &b) {
    charge(c, b);
    auto result = detail::pcurve_point_tangent(c, f).tangent;
    const auto domain = c.knot_domain();
    const double span = finite(domain[1] - domain[0]);
    for (auto &x : result)
        x = finite(x * span);
    return result;
}
bool reverse(BsplineCurve &c, TubeBudget &b, Json &report) {
    require(b.max_control_points <= UINT32_MAX, "native path reversal control limit");
    return curve_detail::reverse_native_working_curve(c, unsigned(b.max_control_points),
                                                      {b.work, b.max_work}, report);
}
BsplineCurve reversed_copy(const BsplineCurve &source, TubeBudget &b, Json &report) {
    auto copy = source;
    report["success"] = reverse(copy, b, report["operation"]);
    const auto &operation = report.at("operation");
    // A new native destination starts empty. Opening rejection leaves that
    // destination empty, unlike an in-place reversal which retains its input.
    require(!operation.contains("opening") || operation.at("opening").at("success") == true,
            "native temporary reversal left an empty frame input");
    return copy;
}
// Native middle-planar joint orientation. Input frame has N,B,T columns.
Matrix4 joint_alignment(Matrix4 f, Point3 incoming, Json &report) {
    normalize(incoming, true);
    const auto z = column(f, 2);
    const double agreement = dot(z, incoming);
    report["normalized_incoming_tangent"] = incoming;
    report["tangent_dot"] = agreement;
    if (agreement > .99999) {
        report["method"] = "nearly_aligned_frame";
        return f;
    }
    auto axis = cross(z, incoming);
    normalize(axis, false);
    const double x = dot(axis, column(f, 0)), y = dot(axis, column(f, 1));
    const double angle = x == 0 && y == 0 ? 0 : finite(std::atan2(y, x));
    Matrix3 rotation{};
    for (unsigned i = 0; i < 3; ++i)
        rotation[i][i] = 1;
    if (angle != 0) {
        const double cosine = finite(std::cos(angle)), sine = finite(std::sin(angle));
        Matrix3 zrotation{{{cosine, -sine, 0}, {sine, cosine, 0}, {0, 0, 1}}};
        // The native Z-only entry still multiplies an identity matrix.
        rotation = rotation_product(rotation, zrotation);
    }
    auto second = cross(incoming, axis);
    normalize(second, false);
    const Matrix3 rows{axis, second, incoming};
    const auto rotated = rotation_product(rotation, rows);
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 3; ++j)
            f[i][j] = rotated[j][i];
    report["method"] = "rotated_joint_basis";
    report["axis"] = axis;
    report["angle"] = angle;
    return f;
}
} // namespace
bool native_path_points_equal(const Point3 &a, const Point3 &b) {
    const double x = finite(a[0] - b[0]), y = finite(a[1] - b[1]), z = finite(a[2] - b[2]);
    const double distance = finite((finite(y * y) + finite(x * x)) + finite(z * z));
    double scale = finite(finite(a[0] * a[0]) + finite(a[1] * a[1]));
    scale = finite(scale + finite(a[2] * a[2]));
    for (double x : b)
        scale = finite(scale + finite(x * x));
    scale = finite(scale + 1);
    return distance < finite(scale * 1.0000000000000001e-20);
}
bool native_path_vectors_parallel(const Point3 &a, const Point3 &b) {
    const auto q = cross(a, b);
    const double q2 = finite((finite(q[1] * q[1]) + finite(q[0] * q[0])) + finite(q[2] * q[2]));
    return q2 <= finite(finite(squared(a) * 1e-24) * squared(b));
}
TubeFacetPathPlacement place_tube_facet_paths(TubeFacetPathBranches paths, TubeBudget &budget) {
    TubeFacetPathPlacement out;
    out.branches = std::move(paths);
    auto &p = out.branches;
    double fraction = p.path.selection.fraction;
    const auto index = p.path.selection.index;
    require(index < p.path.selection.curves.size() && std::isfinite(fraction) &&
                (present(p.prefix) || present(p.suffix)),
            "native path placement prepared input");
    const bool whole = p.whole_path_planarity.at("planar").get<bool>();
    const bool member = p.path.selected_member_planarity.at("planar").get<bool>();
    out.report = {{"scope", "native_facet_path_placement"},
                  {"branch", p.plan.report.at("branch")},
                  {"face_patches_generated", false}};
    auto fail = [&] {
        out.report["status"] = "native_failure";
        out.report["reason"] = "relocated_point_not_equal";
        out.report["work_used"] = budget.work;
        return std::move(out);
    };
    if (whole) {
        auto &c = geometry(p, p.suffix);
        if (!relocate(c, p.path.location.point, fraction, budget, out.report["relocation"]))
            return fail();
        const auto selected = frame(c, fraction, budget, out.report["selected_frame"]);
        const auto start = frame(c, 0, budget, out.report["start_frame"]);
        const auto t = column(selected, 0), binormal = column(start, 2);
        const auto normal = cross(binormal, t);
        out.suffix_transform = inverse(transform(column(selected, 3), normal, binormal, t),
                                       out.report["suffix_placement"]);
    } else if (member) {
        if (index != 0 && index + 1 == p.path.selection.curves.size()) {
            auto &c = geometry(p, p.prefix);
            if (!near(fraction, 1) &&
                !relocate(c, p.path.location.point, fraction, budget, out.report["relocation"]))
                return fail();
            const auto f = frame(c, fraction, budget, out.report["selected_frame"]);
            out.prefix_transform = inverse(section_frame(f), out.report["prefix_placement"]);
        } else {
            auto &suffix = geometry(p, p.suffix);
            if (!near(fraction, 0) && !relocate(suffix, p.path.location.point, fraction, budget,
                                                out.report["relocation"]))
                return fail();
            auto f = frame(suffix, fraction, budget, out.report["selected_frame"]);
            out.suffix_transform = inverse(section_frame(f), out.report["suffix_placement"]);
            if (index != 0) {
                auto &prefix = geometry(p, p.prefix);
                const auto before = tangent(prefix, 1, budget), after = tangent(suffix, 0, budget);
                const bool parallel = native_path_vectors_parallel(before, after);
                out.report["joint_tangents"] = {
                    {"prefix", before}, {"suffix", after}, {"parallel", parallel}};
                auto reversed = reversed_copy(prefix, budget, out.report["temporary_reversal"]);
                const auto pf = frame(reversed, 0, budget, out.report["reversed_prefix_frame"]);
                const auto pi = inverse(section_frame(pf), out.report["reversed_prefix_placement"]);
                if (parallel && near(fraction, 0)) {
                    out.prefix_transform = pi;
                    out.report["joint_method"] = "parallel_at_start";
                } else {
                    f = section_frame(frame(suffix, 0, budget, out.report["suffix_start_frame"]));
                    auto alignment = f;
                    if (!parallel)
                        alignment = joint_alignment(f, before, out.report["joint_alignment"]);
                    out.prefix_transform = product(pi, product(alignment, *out.suffix_transform));
                    out.report["joint_method"] = parallel ? "parallel_relocated" : "aligned_joint";
                    out.report["alignment_frame"] = alignment;
                }
            }
        }
    } else {
        if (present(p.suffix)) {
            auto &c = geometry(p, p.suffix);
            const auto f = frame(c, 0, budget, out.report["suffix_start_frame"]);
            out.suffix_transform = inverse(section_frame(f), out.report["suffix_placement"]);
        }
        if (present(p.prefix)) {
            auto reversed =
                reversed_copy(geometry(p, p.prefix), budget, out.report["temporary_reversal"]);
            const auto f = frame(reversed, 0, budget, out.report["reversed_prefix_frame"]);
            out.prefix_transform = inverse(section_frame(f), out.report["prefix_placement"]);
        }
    }
    // Native final prefix reversal occurs after successful placement. The
    // caller ignores its operation status, including member-planar branches.
    if (present(p.prefix))
        out.report["prefix_reversal_success"] =
            reverse(geometry(p, p.prefix), budget, out.report["prefix_reversal"]);
    out.success = true;
    out.report["status"] = "complete";
    out.report["resolved_fraction"] = fraction;
    out.report["work_used"] = budget.work;
    return out;
}
TubeFacetPathPlacement prepare_tube_facet_path_placement(const Json &profile, const Json &path,
                                                         TubeBudget &budget) {
    return place_tube_facet_paths(prepare_tube_facet_path_branches(profile, path, budget), budget);
}
} // namespace p3d::swept_detail
