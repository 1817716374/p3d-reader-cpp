#include "native_tube.hpp"

namespace p3d::swept_detail {
namespace {
void charge(TubeBudget &b, std::size_t n) {
    require(b.work <= b.max_work && n <= b.max_work - b.work,
            "native curve combination work budget exceeded");
    b.work += n;
}
double finite(double x) {
    require(std::isfinite(x), "native curve combination nonfinite arithmetic");
    return x;
}
Point3 unweight(Point3 p, double w) {
    const double inverse = finite(1.0 / w);
    for (auto &x : p)
        x = finite(x * inverse);
    return p;
}
bool contiguous(const BsplineCurve &a, const BsplineCurve &b) {
    auto p = a.poles().back(), q = b.poles().front();
    if (a.rational())
        p = unweight(p, a.weights().back());
    if (b.rational())
        q = unweight(q, b.weights().front());
    double difference = 0, scale = 0;
    for (unsigned k = 0; k < 3; ++k) {
        difference = std::max(difference, std::abs(finite(p[k] - q[k])));
        scale = std::max(scale, std::max(std::abs(p[k]), std::abs(q[k])));
    }
    return difference < 1e-8 || difference < scale * 1e-8 + 1e-8;
}
double polygon_length(std::vector<Point3> &poles, const std::vector<double> &weights) {
    if (!weights.empty())
        for (std::size_t i = 0; i < poles.size(); ++i)
            poles[i] = unweight(poles[i], weights[i]);
    double length = 0;
    for (std::size_t i = 1; i < poles.size(); ++i) {
        const double x = poles[i - 1][0] - poles[i][0];
        const double y = poles[i - 1][1] - poles[i][1];
        const double z = poles[i - 1][2] - poles[i][2];
        length = finite(length + std::sqrt((x * x + y * y) + z * z));
    }
    // These operations change the stored homogeneous coordinates by rounding.
    // Copying the original controls after computing the length is not equivalent.
    if (!weights.empty())
        for (std::size_t i = 0; i < poles.size(); ++i)
            for (auto &x : poles[i])
                x = finite(x * weights[i]);
    return length;
}
BsplineCurve curve(unsigned order, const std::vector<Point3> &poles,
                   const std::vector<double> &weights, const std::vector<double> &knots) {
    Json flat = Json::array();
    for (const auto &p : poles)
        for (auto x : p)
            flat.push_back(x);
    return BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                    {"order", order},
                                    {"closed", false},
                                    {"poles", std::move(flat)},
                                    {"weights", weights.empty() ? Json(nullptr) : Json(weights)},
                                    {"knots", knots}});
}
} // namespace

TubeCurve combine_open_tube_curves(const BsplineCurve &left, const BsplineCurve &right,
                                   bool force_contiguous, bool reparameterize, TubeBudget &budget) {
    require(!left.closed() && !right.closed() && left.order() == right.order(),
            "native combination requires open curves with equal orders");
    const auto n1 = left.poles().size(), n2 = right.poles().size();
    require(n1 <= budget.max_control_points && n2 <= budget.max_control_points && n1 <= INT32_MAX &&
                n2 <= INT32_MAX - n1,
            "native curve combination source control budget exceeded");
    charge(budget, 16);
    const bool joined = force_contiguous || contiguous(left, right);
    const auto count = n1 + n2 - (joined ? 1 : 0);
    const auto order = left.order();
    require(count <= budget.max_control_points && count <= INT32_MAX - order,
            "native curve combination output control budget exceeded");
    // Account for local control copies, length passes, knot/weight assembly,
    // JSON materialization and validation before allocating the large arrays.
    for (unsigned pass = 0; pass < 32; ++pass)
        charge(budget, n1 + n2);
    auto lp = left.poles(), rp = right.poles();
    double length1 = 1, length2 = 1;
    Json report{{"scope", "native_open_curve_combination"},
                {"force_contiguous", force_contiguous},
                {"contiguous", joined},
                {"reparameterized", reparameterize},
                {"normalized_knots", false},
                {"source_geometry_reused", false},
                {"work_used", budget.work}};
    if (reparameterize) {
        length1 = polygon_length(lp, left.weights());
        length2 = polygon_length(rp, right.weights());
        report["control_polygon_lengths"] = {length1, length2};
        const bool small1 = length1 < 1e-12 || length2 / length1 > 1e10;
        const bool small2 = length2 < 1e-12 || length1 / length2 > 1e10;
        require(!(small1 && small2), "native curve combination has two negligible polygons");
        if (small1 || small2) {
            report["retained_source"] = small1 ? "right" : "left";
            report["discarded_incoming_first_control"] = false;
            const auto &source = small1 ? right : left;
            return {curve(order, small1 ? rp : lp, source.weights(), source.knots()),
                    std::move(report)};
        }
    }
    report["retained_source"] = "both";
    report["discarded_incoming_first_control"] = joined;
    const auto first_count = n1 + order - (joined ? 1 : 0);
    std::vector<double> knots;
    knots.reserve(count + order);
    for (std::size_t i = 0; i < first_count; ++i)
        knots.push_back(finite(left.knots()[i] * length1));
    const double join = finite(left.knots()[n1] * length1);
    for (std::size_t i = order; i < right.knots().size(); ++i)
        knots.push_back(finite(join + (right.knots()[i] - right.knots()[order - 1]) * length2));
    const double low = knots[order - 1], range = finite(knots[count] - low);
    // The native caller ignores normalization failure, leaving a tiny domain
    // unchanged. Successful open normalization explicitly sets trailing knots.
    if (std::abs(range) >= 1e-10) {
        for (auto &u : knots)
            u = finite((u - low) / range);
        std::fill(knots.begin() + count, knots.end(), 1.0);
        report["normalized_knots"] = true;
    }
    lp.reserve(count);
    lp.insert(lp.end(), rp.begin() + (joined ? 1 : 0), rp.end());
    std::vector<double> weights;
    if (left.rational() || right.rational()) {
        weights.reserve(count);
        for (std::size_t i = 0; i < n1; ++i)
            weights.push_back(left.rational() ? left.weights()[i] : 1.0);
        for (std::size_t i = joined ? 1 : 0; i < n2; ++i)
            weights.push_back(right.rational() ? right.weights()[i] : 1.0);
    }
    return {curve(order, lp, weights, knots), std::move(report)};
}
} // namespace p3d::swept_detail
