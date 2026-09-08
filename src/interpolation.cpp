#include "internal.hpp"
namespace p3d {
namespace {
constexpr double tolerance = 1e-5;
double distance_squared(Point3 a, Point3 b) {
    const double x = a[0] - b[0], y = a[1] - b[1], z = a[2] - b[2];
    const double d = (x * x + y * y) + z * z;
    require(std::isfinite(d), "interpolation point distance overflow");
    return d;
}
bool option(const Json &table, const char *name) {
    const auto &v = table.at(name);
    require(v.is_number_integer() && v >= std::numeric_limits<int>::min() &&
                v <= std::numeric_limits<int>::max(),
            "interpolation option is not int32");
    return v.get<int>() != 0;
}
// Three nonzero cubic basis values at a simple interpolation knot. The fourth
// value is zero. The same local recursion is valid at periodic seam knots.
std::array<double, 3> coefficients(const std::vector<double> &knots, std::size_t span) {
    const double t = knots[span];
    std::array<double, 4> b{1, 0, 0, 0}, left{}, right{};
    for (unsigned j = 1; j <= 3; ++j) {
        left[j] = t - knots[span + 1 - j];
        right[j] = knots[span + j] - t;
        double saved = 0;
        for (unsigned r = 0; r < j; ++r) {
            const double length = right[r + 1] + left[j - r];
            require(length > 0 && std::isfinite(length), "interpolation knot interval");
            const double value = b[r];
            b[r] = saved + value * (right[r + 1] / length);
            saved = value * (left[j - r] / length);
        }
        b[j] = saved;
    }
    return {b[0], b[1], b[2]};
}
// Each right-hand side is XYZ. Storage and work are linear in point count.
std::vector<Point3> solve(const std::vector<double> &lower, std::vector<double> diagonal,
                          const std::vector<double> &upper, std::vector<Point3> rhs) {
    for (std::size_t i = 0; i < diagonal.size(); ++i) {
        require(diagonal[i] != 0 && std::isfinite(diagonal[i]),
                "interpolation singular tridiagonal system");
        if (i + 1 == diagonal.size())
            break;
        const double ratio = lower[i + 1] / diagonal[i];
        diagonal[i + 1] -= ratio * upper[i];
        for (unsigned k = 0; k < 3; ++k)
            rhs[i + 1][k] -= ratio * rhs[i][k];
    }
    for (std::size_t i = diagonal.size(); i-- > 0;)
        for (unsigned k = 0; k < 3; ++k) {
            if (i + 1 < diagonal.size())
                rhs[i][k] -= upper[i] * rhs[i + 1][k];
            rhs[i][k] /= diagonal[i];
            require(std::isfinite(rhs[i][k]), "interpolation solution overflow");
        }
    return rhs;
}
Point3 bessel_handle(Point3 p0, Point3 p1, Point3 p2, double h0, double h1) {
    Point3 handle{};
    // Endpoint derivative of the quadratic through the first three fit points,
    // converted to a cubic B-spline endpoint handle. No unit normalization.
    for (unsigned k = 0; k < 3; ++k)
        handle[k] = p0[k] + ((2 * h0 + h1) / (h0 + h1) * (p1[k] - p0[k]) -
                             h0 / h1 * (h0 / (h0 + h1)) * (p2[k] - p1[k])) /
                                3;
    return handle;
}
} // namespace
InterpolationCurve InterpolationCurve::from_bgfb(const Json &table) {
    require(table.at("_type") == "InterpolationCurve", "expected BGFB InterpolationCurve table");
    require(table.at("closed").is_boolean(), "interpolation closed flag");
    bool closed = table.at("closed").get<bool>();
    const bool chord = option(table, "isChordLenKnots"),
               colinear = option(table, "isColinearTangents"),
               natural = option(table, "isNaturalTangents");
    option(table, "isChordLenTangents");
    option(table, "order");
    const auto &flat = table.at("fitPoints");
    require(flat.is_array() && flat.size() % 3 == 0, "interpolation XYZ triplets");
    const auto count = flat.size() / 3;
    require(count >= 2 && count <= std::size_t(std::numeric_limits<int>::max()),
            "interpolation source point count");
    std::vector<Point3> points;
    std::vector<std::size_t> indices;
    Json ignored = Json::array();
    for (std::size_t i = 0; i < count; ++i) {
        Point3 p{};
        for (unsigned k = 0; k < 3; ++k) {
            const auto &v = flat[3 * i + k];
            require(v.is_number(), "interpolation coordinate is not numeric");
            p[k] = v.get<double>();
            require(std::isfinite(p[k]), "interpolation coordinate is not finite");
        }
        if (!points.empty() && distance_squared(points.back(), p) < tolerance * tolerance) {
            ignored.push_back({{"source_index", i}, {"reason", "native_adjacent_proximity"}});
            continue;
        }
        points.push_back(p);
        indices.push_back(i);
    }
    // The native ensure-two-points fallback duplicates the first point, which
    // then fails the endpoint distance check. Reject without inventing geometry.
    require(points.size() >= 2, "interpolation coincident fit points");
    double end_distance = std::sqrt(distance_squared(points.front(), points.back()));
    if (points.size() == 3 && end_distance <= tolerance) {
        ignored.push_back(
            {{"source_index", indices.back()}, {"reason", "native_three_point_closure"}});
        points.pop_back();
        indices.pop_back();
        end_distance = std::sqrt(distance_squared(points.front(), points.back()));
    }
    bool appended = false;
    if (points.size() <= 2) {
        require(end_distance > tolerance, "interpolation endpoints within native tolerance");
        closed = false;
    } else if (closed) {
        if (end_distance > tolerance) {
            points.push_back(points.front());
            indices.push_back(indices.front());
            appended = true;
        }
        if (points.size() == 4)
            closed = false;
    }
    const auto n = points.size();
    require(n <= 4998, "native interpolation prepared point count exceeds 4998");
    std::vector<double> parameters(n, 0);
    if (closed && !chord) {
        // The native default knot generator accumulates the uniform increment.
        const double step = 1. / double(n - 1);
        for (std::size_t i = 1; i + 1 < n; ++i)
            parameters[i] = parameters[i - 1] + step;
    } else {
        for (std::size_t i = 1; i < n; ++i)
            parameters[i] =
                parameters[i - 1] + std::sqrt(distance_squared(points[i - 1], points[i]));
        require(parameters.back() > 0 && std::isfinite(parameters.back()),
                "interpolation chord parameter overflow");
        const double total = parameters.back();
        for (double &t : parameters)
            t /= total;
    }
    parameters.back() = 1;
    for (std::size_t i = 1; i < n; ++i)
        require(parameters[i] > parameters[i - 1], "interpolation collapsed parameter interval");
    const auto pole_count = closed ? n - 1 : n + 2;
    std::vector<double> knots(pole_count + (closed ? 7 : 4), 0);
    std::copy(parameters.begin() + 1, parameters.end() - 1, knots.begin() + 4);
    for (unsigned i = 0; i < 4; ++i) {
        if (closed) {
            knots[i] = knots[i + pole_count] - 1;
            knots[pole_count + 3 + i] = knots[3 + i] + 1;
        } else
            knots[n + 2 + i] = 1;
    }
    std::vector<Point3> poles;
    std::string condition;
    bool aligned = false;
    if (closed) {
        const auto m = n - 1;
        std::vector<double> lower(m), diagonal(m), upper(m);
        std::vector<Point3> rhs(points.begin(), points.end() - 1);
        for (std::size_t i = 0; i < m; ++i) {
            const auto b = coefficients(knots, i + 3);
            lower[i] = b[0];
            diagonal[i] = b[1];
            upper[i] = b[2];
        }
        // Rank-one correction of the cyclic tridiagonal system. Unknown Q_i
        // is native pole (i+1) modulo m; preserve this native pole ordering.
        const double alpha = lower.front(), beta = upper.back(), gamma = -diagonal.front();
        require(gamma != 0 && std::isfinite(gamma), "interpolation periodic pivot");
        diagonal.front() -= gamma;
        diagonal.back() -= alpha * beta / gamma;
        auto x = solve(lower, diagonal, upper, std::move(rhs));
        std::vector<Point3> u(m);
        u.front()[0] = gamma;
        u.back()[0] = beta;
        const auto z = solve(lower, diagonal, upper, std::move(u));
        const double denominator = 1 + z.front()[0] + alpha * z.back()[0] / gamma;
        require(denominator != 0 && std::isfinite(denominator),
                "interpolation singular periodic system");
        for (unsigned k = 0; k < 3; ++k) {
            const double factor = (x.front()[k] + alpha * x.back()[k] / gamma) / denominator;
            for (std::size_t i = 0; i < m; ++i)
                x[i][k] -= z[i][0] * factor;
        }
        poles.resize(m);
        for (std::size_t i = 0; i < m; ++i)
            poles[(i + 1) % m] = x[i];
        condition = "periodic_c2";
    } else {
        std::vector<double> lower(n), diagonal(n, 1), upper(n);
        std::vector<Point3> rhs = points;
        if (n == 2) {
            for (unsigned k = 0; k < 3; ++k) {
                rhs.front()[k] = points.front()[k] + (points.back()[k] - points.front()[k]) / 3;
                rhs.back()[k] = points.back()[k] + (points.front()[k] - points.back()[k]) / 3;
            }
            condition = "two_point_line";
        } else if (natural) {
            upper.front() = -parameters[1] / parameters[2];
            diagonal.front() = 1 - upper.front();
            lower.back() = -(1 - parameters[n - 2]) / (1 - parameters[n - 3]);
            diagonal.back() = 1 - lower.back();
            condition = "natural_second_derivative_zero";
        } else {
            rhs.front() = bessel_handle(points[0], points[1], points[2], parameters[1],
                                        parameters[2] - parameters[1]);
            rhs.back() =
                bessel_handle(points[n - 1], points[n - 2], points[n - 3], 1 - parameters[n - 2],
                              parameters[n - 2] - parameters[n - 3]);
            if (colinear && n >= 4 &&
                std::sqrt(distance_squared(points.front(), points.back())) < tolerance) {
                const double a = std::sqrt(distance_squared(points.front(), rhs.front())),
                             b = std::sqrt(distance_squared(points.back(), rhs.back())),
                             length = std::sqrt(distance_squared(rhs.front(), rhs.back()));
                Point3 direction{};
                if (length > 0)
                    for (unsigned k = 0; k < 3; ++k)
                        direction[k] = (rhs.front()[k] - rhs.back()[k]) / length;
                for (unsigned k = 0; k < 3; ++k) {
                    rhs.front()[k] = points.front()[k] + a * direction[k];
                    rhs.back()[k] = points.back()[k] - b * direction[k];
                }
                aligned = true;
            }
            condition = "bessel_quadratic";
        }
        for (std::size_t i = 1; i + 1 < n; ++i) {
            const auto b = coefficients(knots, i + 3);
            lower[i] = b[0];
            diagonal[i] = b[1];
            upper[i] = b[2];
        }
        poles = solve(lower, std::move(diagonal), upper, std::move(rhs));
        poles.insert(poles.begin(), points.front());
        poles.push_back(points.back());
    }
    Json flat_poles = Json::array();
    for (const auto &p : poles)
        for (double x : p) {
            require(std::isfinite(x), "interpolation non-finite derived pole");
            flat_poles.push_back(x);
        }
    InterpolationCurve result(BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                                       {"order", 4},
                                                       {"closed", closed},
                                                       {"poles", flat_poles},
                                                       {"weights", nullptr},
                                                       {"knots", knots}}));
    result.source_ = table;
    result.points_ = std::move(points);
    result.indices_ = std::move(indices);
    result.parameters_ = std::move(parameters);
    result.report_ = {{"status", "valid"},
                      {"representation", "derived_cubic_bspline"},
                      {"source_point_count", count},
                      {"prepared_point_indices", result.indices_},
                      {"ignored_points", ignored},
                      {"closure_point_appended", appended},
                      {"effective_closed", closed},
                      {"proximity_tolerance", tolerance},
                      {"knot_spacing", closed && !chord ? "uniform" : "chord_length"},
                      {"endpoint_condition", condition},
                      {"endpoint_handles_aligned", aligned},
                      {"bgfb_inactive_fields", Json::array({"order", "knots", "startTangent",
                                                            "endTangent", "isChordLenTangents"})},
                      {"derived_pole_count", pole_count}};
    return result;
}
} // namespace p3d
