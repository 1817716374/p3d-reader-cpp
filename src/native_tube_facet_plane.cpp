#include "native_tube_facet_plane.hpp"
#include "native_tube_facet_extension.hpp"
#include "native_control_lines.hpp"
#include "native_curve_conversion.hpp"
#include "native_bezier.hpp"
namespace p3d::swept_detail {
namespace {
double finite(double v) {
    require(std::isfinite(v), "native facet plane nonfinite arithmetic");
    return v;
}
void charge(TubeBudget &b, std::size_t n) {
    curve_detail::BezierWork{b.work, b.max_work}.charge(n);
}
Point3 point(const Json &table, std::size_t index) {
    return {table["poles"][3 * index].get<double>(), table["poles"][3 * index + 1].get<double>(),
            table["poles"][3 * index + 2].get<double>()};
}
void set(Json &table, std::size_t index, const Point3 &p) {
    for (unsigned k = 0; k < 3; ++k)
        table["poles"][3 * index + k] = finite(p[k]);
}
void weight_row(Json &table, const BsplineSurface &s, std::size_t begin, bool divide,
                TubeBudget &b) {
    if (!s.rational())
        return;
    charge(b, 4 * s.u().pole_count());
    for (std::size_t i = 0; i < s.u().pole_count(); ++i) {
        const double weight = s.weights()[begin + i];
        const double factor = divide ? finite(1 / weight) : weight;
        auto p = point(table, begin + i);
        for (auto &v : p)
            v = finite(v * factor);
        set(table, begin + i, p);
    }
}
Point3 add(const Point3 &a, const Point3 &b, bool subtract = false) {
    Point3 out{};
    for (unsigned k = 0; k < 3; ++k)
        out[k] = finite(subtract ? a[k] - b[k] : a[k] + b[k]);
    return out;
}
double distance(const Point3 &a, const Point3 &b) {
    const auto d = add(a, b, true);
    return finite(std::sqrt((d[1] * d[1] + d[0] * d[0]) + d[2] * d[2]));
}
template <class T> const T &read(const std::shared_ptr<TubeFacetSeamStorage<T>> &p) {
    require(p && p->alive, "native facet plane requires live referenced storage");
    return p->value;
}
void account(const Json &s, std::size_t &total, TubeBudget &b) {
    require(s.is_object() && s.contains("poles") && s["poles"].is_array() &&
                s["poles"].size() % 3 == 0,
            "native facet plane surface table");
    const auto n = s["poles"].size() / 3;
    require(n <= b.max_control_points - total, "native facet plane cumulative control limit");
    total += n;
    for (unsigned i = 0; i < 8; ++i)
        charge(b, n);
}
} // namespace
TubeFacetPlanePreparation prepare_tube_facet_plane_seam(const Json &first, const Json &second,
                                                        const TubeFacetSeamReferences &seam,
                                                        TubeBudget &budget) {
    std::size_t total = 0;
    account(first, total, budget);
    const bool self = &first == &second;
    if (!self)
        account(second, total, budget);
    const auto a = BsplineSurface::from_bgfb(first), b = BsplineSurface::from_bgfb(second);
    require(a.boundaries().is_null() && b.boundaries().is_null() &&
                a.u().pole_count() == b.u().pole_count(),
            "native facet plane requires matching untrimmed columns");
    const std::size_t nu = a.u().pole_count(), end = (a.v().pole_count() - 1) * nu;
    const bool ruled = a.v().order() == 2 && b.v().order() == 2;
    TubeFacetPlanePreparation out;
    out.first = first;
    if (!self)
        out.second = second;
    Json &wa = out.first, &wb = self ? out.first : out.second;
    out.classifier = seam.classifier;
    out.report = {{"scope", "native_facet_plane_preparation"},
                  {"first", Json::array()},
                  {"second", Json::array()},
                  {"extensions", Json::array()},
                  {"self_surface", self}};
    auto finish = [&](TubeFacetSeamStatus status, const char *reason) {
        out.status = status;
        out.report["reason"] = reason;
        out.report["classifier"] = out.classifier;
        out.report["work_used"] = budget.work;
        if (self)
            out.second = out.first;
        return std::move(out);
    };
    weight_row(wa, a, end, true, budget);
    const auto &incoming = read(seam.incoming);
    const auto &plane = read(seam.plane);
    for (std::size_t i = 0; i < nu; ++i) {
        charge(budget, 128);
        const auto origin = point(wa, end + i);
        const auto hit = native_ray_plane_intersection(origin, incoming, plane);
        const bool moved = !curve_detail::endpoint_pair_closed(hit.point, origin);
        out.report["first"].push_back({{"parameter", hit.parameter},
                                       {"divided", hit.divided},
                                       {"point", hit.point},
                                       {"moved", moved}});
        if (ruled && moved && hit.parameter < 0 &&
            -distance(origin, point(wa, end - nu + i)) > hit.parameter) {
            out.classifier = 2;
            weight_row(wa, a, end, false, budget);
            return finish(TubeFacetSeamStatus::complete, "first_retreat_exceeds_control_distance");
        }
        out.first_projected.push_back(hit.point);
        out.first_original.push_back(origin);
        if (a.rational())
            out.first_weights.push_back(a.weights()[end + i]);
    }
    weight_row(wb, b, 0, true, budget);
    const auto &outgoing = read(seam.outgoing);
    Point3 backward{};
    for (unsigned k = 0; k < 3; ++k)
        backward[k] = finite(-outgoing[k]);
    for (std::size_t i = 0; i < nu; ++i) {
        charge(budget, 128);
        const auto origin = point(wb, i);
        const auto hit = native_ray_plane_intersection(origin, backward, plane);
        const bool moved = !curve_detail::endpoint_pair_closed(hit.point, origin);
        out.report["second"].push_back({{"parameter", hit.parameter},
                                        {"divided", hit.divided},
                                        {"point", hit.point},
                                        {"moved", moved}});
        if (ruled && moved && hit.parameter < 0 &&
            -distance(origin, point(wb, nu + i)) > hit.parameter) {
            out.classifier = 2;
            // Native early return restores ONLY the second surface's row.
            // The first end row remains unweighted, including self aliases.
            weight_row(wb, b, 0, false, budget);
            return finish(TubeFacetSeamStatus::complete, "second_retreat_exceeds_control_distance");
        }
        out.second_projected.push_back(hit.point);
        out.second_original.push_back(origin);
        if (b.rational())
            out.second_weights.push_back(b.weights()[i]);
    }
    if (ruled) {
        charge(budget, 8 * nu);
        out.classifier = 0;
        for (std::size_t i = 0; i < nu; ++i) {
            auto p = out.first_projected[i];
            if (a.rational())
                for (auto &v : p)
                    v = finite(v * a.weights()[i]);
            set(wa, end + i, p);
        }
        for (std::size_t i = 0; i < nu; ++i) {
            auto p = out.second_projected[i];
            if (b.rational())
                for (auto &v : p)
                    v = finite(v * b.weights()[i]);
            set(wb, i, p);
        }
        return finish(TubeFacetSeamStatus::complete, "projected_control_rows_applied");
    }
    // Non-ruled entry initializes the native extension flag to true. Maxima
    // come from tangent-line closest fractions, NOT the plane-hit parameters.
    for (std::size_t i = 0; i < nu; ++i) {
        charge(budget, 128);
        const auto pa = point(wa, end + i), pb = point(wb, i);
        const auto hit =
            native_control_line_pair(pa, add(pa, incoming), pb, add(pb, outgoing, true), .0001);
        Json probe = {{"success", hit.success}};
        if (hit.success) {
            probe["first_fraction"] = hit.first_fraction;
            probe["second_fraction"] = hit.second_fraction;
        }
        out.report["extensions"].push_back(std::move(probe));
        if (!hit.success)
            return finish(TubeFacetSeamStatus::native_failure, "parallel_extension_tangents");
        if (hit.first_fraction > out.first_extension)
            out.first_extension = hit.first_fraction;
        if (hit.second_fraction > out.second_extension)
            out.second_extension = hit.second_fraction;
    }
    weight_row(wa, a, end, false, budget);
    weight_row(wb, b, 0, false, budget);
    out.classifier = 1;
    out.report["first_extension"] = out.first_extension;
    out.report["second_extension"] = out.second_extension;
    return finish(TubeFacetSeamStatus::pending_general, "surface_extension_required");
}
TubeFacetSeamResult apply_tube_facet_seam(Json &first, Json &second, TubeFacetSeamReferences &seam,
                                          TubeBudget &budget) {
    auto direct = apply_tube_ruled_facet_seam(first, second, seam, budget);
    if (direct.status != TubeFacetSeamStatus::pending_general)
        return direct;
    auto plane = prepare_tube_facet_plane_seam(first, second, seam, budget);
    if (plane.status == TubeFacetSeamStatus::pending_general)
        extend_tube_facet_plane_seam(plane, seam, budget);
    TubeFacetSeamResult out;
    out.status = plane.status;
    out.report = {{"direct", std::move(direct.report)}, {"plane", std::move(plane.report)}};
    if (plane.status == TubeFacetSeamStatus::complete) {
        const bool self = &first == &second;
        first = std::move(plane.first);
        if (!self)
            second = std::move(plane.second);
        seam.classifier = plane.classifier;
    }
    // Invalid/unsupported extension throws before publishing either side.
    return out;
}
} // namespace p3d::swept_detail
