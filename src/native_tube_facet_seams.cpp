#include "native_tube_facet_seams.hpp"
#include "native_control_lines.hpp"
#include "native_curve_conversion.hpp"
#include "native_bezier.hpp"
namespace p3d::swept_detail {
namespace {
double finite(double x) {
    require(std::isfinite(x), "native facet seam nonfinite arithmetic");
    return x;
}
void charge(TubeBudget &b, std::size_t n) {
    curve_detail::BezierWork{b.work, b.max_work}.charge(n);
}
Point3 control(const BsplineSurface &s, std::size_t i) {
    Point3 p = s.poles().at(i);
    if (!s.rational())
        return p;
    const double w = s.weights().at(i);
    const double largest = std::max({std::abs(p[0]), std::abs(p[1]), std::abs(p[2])});
    // Base295 protected division is not the unWeightPoles array routine.
    if (largest * 1e-12 >= std::abs(w))
        return p;
    const double inverse = finite(1 / w);
    for (auto &v : p)
        v = finite(v * inverse);
    return p;
}
void write_row(Json &table, const BsplineSurface &s, std::size_t begin,
               const std::vector<Point3> &points) {
    for (std::size_t i = 0; i < points.size(); ++i)
        for (unsigned k = 0; k < 3; ++k) {
            // Native f464a/f46a3 use the BASE of the weight array even for
            // the first surface's last row, not that row's weight offset.
            const double value =
                s.rational() ? finite(points[i][k] * s.weights()[i]) : points[i][k];
            table["poles"][3 * (begin + i) + k] = value;
        }
}
void budget_surface(const Json &j, std::size_t &count, TubeBudget &b) {
    require(j.is_object() && j.contains("poles") && j["poles"].is_array() &&
                j["poles"].size() % 3 == 0,
            "native facet seam surface table");
    const auto n = j["poles"].size() / 3;
    require(n <= b.max_control_points - count, "native facet seam control budget");
    count += n;
    for (unsigned i = 0; i < 8; ++i)
        charge(b, n);
}
} // namespace
TubeFacetSeamResult apply_tube_ruled_facet_seam(Json &first, Json &second,
                                                TubeFacetSeamReferences &seam, TubeBudget &b) {
    TubeFacetSeamResult out;
    out.report = {{"scope", "native_facet_control_line_seam"},
                  {"status", "pending_general"},
                  {"columns", Json::array()}};
    std::size_t count = 0;
    budget_surface(first, count, b);
    if (&first != &second)
        budget_surface(second, count, b);
    const auto a = BsplineSurface::from_bgfb(first), c = BsplineSurface::from_bgfb(second);
    require(a.boundaries().is_null() && c.boundaries().is_null(),
            "native facet seam requires untrimmed surfaces");
    if (a.v().order() != 2 || c.v().order() != 2) {
        out.report["reason"] = "curved_v_requires_general_branch";
        return out;
    }
    const auto nu = a.u().pole_count(), nv = a.v().pole_count();
    require(nu == c.u().pole_count(), "native facet seam column count mismatch");
    const auto end = (nv - 1) * nu;
    std::vector<Point3> left, right;
    for (std::size_t i = 0; i < nu; ++i) {
        charge(b, 128);
        const auto hit = native_control_line_pair(control(a, end - nu + i), control(a, end + i),
                                                  control(c, i), control(c, nu + i), .0001);
        Json col = {{"column", i}, {"native_line_success", hit.success}};
        if (!hit.success) {
            out.report["columns"].push_back(std::move(col));
            out.status = TubeFacetSeamStatus::native_failure;
            out.report["status"] = "native_failure";
            out.report["reason"] = "parallel_or_degenerate_control_lines";
            return out;
        }
        const bool near = curve_detail::endpoint_pair_closed(hit.first, hit.second);
        col["first_fraction"] = hit.first_fraction;
        col["second_fraction"] = hit.second_fraction;
        col["first_location"] = hit.first_location;
        col["second_location"] = hit.second_location;
        col["first_point"] = hit.first;
        col["second_point"] = hit.second;
        col["points_near"] = near;
        out.report["columns"].push_back(std::move(col));
        if (!near) {
            out.report["reason"] = "skew_control_lines_require_general_branch";
            return out;
        }
        left.push_back(hit.first);
        right.push_back(hit.second);
    }
    charge(b, 8 * nu);
    seam.classifier = 0;
    // Native unweights these rows immediately before overwriting EVERY point.
    // Those discarded intermediate values need no storage or division here.
    write_row(first, a, end, left);
    write_row(second, c, 0, right);
    out.status = TubeFacetSeamStatus::complete;
    out.report["status"] = "complete";
    out.report["first_end_uses_first_row_weights"] = a.rational();
    out.report["work_used"] = b.work;
    return out;
}
TubeFacetSeamResult process_tube_facet_seams(TubeFacetComposition &chain, TubeBudget &b,
                                             std::size_t start) {
    require(chain.prepared && chain.nodes.size() == chain.seams.size() &&
                start <= chain.nodes.size(),
            "native facet seam pass requires a prepared chain and valid cursor");
    if (chain.report.contains("seam_processing")) {
        const auto &previous = chain.report.at("seam_processing");
        require(previous.at("status") == "pending_general" &&
                    previous.at("next_seam").get<std::size_t>() == start,
                "native facet seam pass must resume at the pending node");
    } else
        require(start == 0, "native facet seam pass cannot skip unvisited nodes");
    TubeFacetSeamResult out;
    out.report = {{"scope", "native_adjacent_facet_seams"},
                  {"visits", Json::array()},
                  {"chain_finalized", false}};
    if (chain.nodes.empty()) {
        out.status = TubeFacetSeamStatus::native_failure;
        out.report["status"] = "native_failure";
        chain.prepared = false;
        chain.report["status"] = "native_failure";
        chain.report["seams_applied"] = false;
        chain.report["seam_processing"] = out.report;
        return out;
    }
    for (std::size_t i = start; i < chain.nodes.size(); ++i) {
        charge(b, 1);
        const auto next = (i + 1) % chain.nodes.size();
        Json visit = {{"node", i}, {"next", next}};
        if (chain.seams[i].classifier == 2) {
            visit["status"] = "skipped_classifier_2";
            out.report["visits"].push_back(std::move(visit));
            continue;
        }
        auto result = apply_tube_ruled_facet_seam(chain.nodes[i].surface, chain.nodes[next].surface,
                                                  chain.seams[i], b);
        visit["seam"] = std::move(result.report);
        out.report["visits"].push_back(std::move(visit));
        chain.nodes[i].end_seam.classifier = chain.seams[i].classifier;
        if (result.status != TubeFacetSeamStatus::complete) {
            out.status = result.status;
            out.report["next_seam"] = i;
            out.report["status"] = result.status == TubeFacetSeamStatus::native_failure
                                       ? "native_failure"
                                       : "pending_general";
            chain.report["seams_applied"] = false;
            chain.report["status"] = out.report["status"];
            chain.report["seam_processing"] = out.report;
            if (result.status == TubeFacetSeamStatus::native_failure) {
                chain.nodes.clear();
                chain.seams.clear();
                chain.prepared = false;
            }
            return out;
        }
    }
    out.status = TubeFacetSeamStatus::complete;
    out.report["status"] = "complete";
    out.report["next_seam"] = chain.nodes.size();
    out.report["work_used"] = b.work;
    chain.report["seams_applied"] = true;
    chain.report["status"] = "seams_applied_before_finalization";
    chain.report["seam_processing"] = out.report;
    return out;
}
} // namespace p3d::swept_detail
