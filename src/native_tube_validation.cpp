#include "native_tube.hpp"

namespace p3d::swept_detail {
namespace {
struct SourceCheck {
    TubeBudget &budget;
    std::size_t geometry_visits = 0, numeric_visits = 0, scalar_checks = 0;
    std::string failure_path, failure_reason;
    explicit SourceCheck(TubeBudget &b) : budget(b) {}

    void charge(std::size_t n) {
        require(budget.work <= budget.max_work && n <= budget.max_work - budget.work,
                "native swept source validation work budget exceeded");
        budget.work += n;
    }
    bool fail(const std::string &path, const char *reason) {
        failure_path = path;
        failure_reason = reason;
        return false;
    }
    bool number(const Json &v, const std::string &path) {
        charge(1);
        ++scalar_checks;
        require(v.is_number(), "native swept source scalar layout");
        return std::isfinite(v.get<double>()) || fail(path, "nonfinite_scalar");
    }
    bool numbers(const Json &v, const std::string &path, bool xyz) {
        require(v.is_array() && (!xyz || v.size() % 3 == 0), "native swept source array layout");
        require(xyz ? v.size() / 3 <= budget.max_control_points
                    : v.size() / 3 + (v.size() % 3 != 0) <= budget.max_control_points,
                "native swept source array budget exceeded");
        for (std::size_t i = 0; i < v.size(); ++i)
            if (!number(v[i], path + "/" + std::to_string(i)))
                return false;
        return true;
    }
    bool point(const Json &v, const std::string &prefix, const std::string &path) {
        for (const char *axis : {"X", "Y", "Z"}) {
            const auto key = prefix + axis;
            if (!number(v.at(key), path + "/" + key))
                return false;
        }
        return true;
    }
    bool visit(const Json &v, bool numeric, const std::string &path, unsigned depth) {
        require(depth <= 80, "native swept source validation nesting limit exceeded");
        charge(1);
        ++(numeric ? numeric_visits : geometry_visits);
        if (v.is_null())
            return fail(path, "null_geometry");
        require(v.is_object() && v.contains("_type") && v.at("_type").is_string(),
                "native swept source requires a decoded geometry table");
        const auto type = v.at("_type").get<std::string>();
        if (type == "CurveVector") {
            const auto &members = v.at("curves");
            require(members.is_array(), "native swept source member layout");
            if (!numeric && members.empty())
                return fail(path, "empty_curve_vector");
            // Neither native validator consults the boundary type. Keep the
            // conversion dispatch separate (including parity/union policy).
            for (std::size_t i = 0; i < members.size(); ++i)
                if (!visit(members[i].at("geometry"), numeric,
                           path + "/curves/" + std::to_string(i) + "/geometry", depth + 1))
                    return false;
            return true;
        }
        if (type == "LineString" || type == "PointString" || type == "AkimaCurve") {
            const auto &points = v.at("points");
            require(points.is_array() && points.size() % 3 == 0,
                    "native swept source point array layout");
            if (!numeric)
                return !points.empty() || fail(path, "empty_point_array");
            return numbers(points, path + "/points", true);
        }
        if (type == "LineSegment") {
            if (!numeric)
                return true;
            const auto &s = v.at("segment");
            if (s.is_null())
                return fail(path + "/segment", "null_detail");
            return point(s, "point0", path + "/segment") && point(s, "point1", path + "/segment");
        }
        if (type == "EllipticArc") {
            if (!numeric)
                return true;
            const auto &a = v.at("arc");
            if (a.is_null())
                return fail(path + "/arc", "null_detail");
            return point(a, "center", path + "/arc") && point(a, "vector0", path + "/arc") &&
                   point(a, "vector90", path + "/arc") &&
                   number(a.at("startRadians"), path + "/arc/startRadians") &&
                   number(a.at("sweepRadians"), path + "/arc/sweepRadians");
        }
        if (type == "BsplineCurve") {
            if (!numeric)
                return true;
            // Native validation copies stored homogeneous controls, weights,
            // and knots. It does not divide by weights, test knot ordering,
            // require positive weights, or evaluate a point on the curve.
            if (!numbers(v.at("poles"), path + "/poles", true))
                return false;
            for (const char *key : {"weights", "knots"})
                if (!v.at(key).is_null() && !numbers(v.at(key), path + "/" + key, false))
                    return false;
            return true;
        }
        throw std::runtime_error("unsupported native swept source validation: " + type);
    }
};
} // namespace

Json validate_tube_sources(const Json &profile, const Json &path, TubeBudget &budget) {
    for (const auto *v : {&path, &profile})
        require(v->is_null() ||
                    (v->is_object() && v->value("_type", std::string()) == "CurveVector"),
                "native swept source root requires CurveVector");
    SourceCheck check{budget};
    const bool geometry =
        check.visit(path, false, "/path", 0) && check.visit(profile, false, "/profile", 0);
    Json numeric = nullptr;
    bool accepted = false;
    if (geometry) {
        accepted = check.visit(path, true, "/path", 0) && check.visit(profile, true, "/profile", 0);
        numeric = accepted;
    }
    return {{"scope", "native_swept_source_validation"},
            {"geometry_valid", geometry},
            {"numeric_valid", numeric},
            {"accepted", accepted},
            {"failure_path", check.failure_path.empty() ? Json(nullptr) : Json(check.failure_path)},
            {"failure_reason",
             check.failure_reason.empty() ? Json(nullptr) : Json(check.failure_reason)},
            {"geometry_visits", check.geometry_visits},
            {"numeric_visits", check.numeric_visits},
            {"scalar_checks", check.scalar_checks},
            {"work_used", budget.work}};
}
} // namespace p3d::swept_detail
