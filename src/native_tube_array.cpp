#include "native_tube.hpp"
#include "native_curve_conversion.hpp"
#include "loft_curve.hpp"

namespace p3d::swept_detail {
namespace {
void charge(TubeBudget &b, std::size_t amount) {
    require(b.work <= b.max_work && amount <= b.max_work - b.work,
            "native curve group work budget exceeded");
    b.work += amount;
}
unsigned boundary(const Json &v) {
    require(v.is_object() && v.value("_type", std::string()) == "CurveVector",
            "native curve group requires CurveVector");
    const auto &t = v.at("type");
    require(t.is_number_integer() && t >= 0 && t <= 5, "native curve group boundary type");
    const auto &members = v.at("curves");
    require(members.is_array() && !members.empty(), "native curve group has no members");
    return t.get<unsigned>();
}
} // namespace
TubeCurve convert_tube_curve_array(const Json &value, TubeBudget &budget) {
    const auto type = boundary(value);
    require(type <= 3, "native single swept curve rejects region collections");
    const auto &members = value.at("curves");
    charge(budget, members.size());
    Json conversions = Json::array(), joins = Json::array();
    std::optional<BsplineCurve> combined;
    Point3 first{}, last{};
    bool have_endpoints = false;
    for (std::size_t i = 0; i < members.size(); ++i) {
        const auto &v = members[i].at("geometry");
        auto converted = convert_tube_primitive(v, budget);
        // Source polyline and ellipse endpoints differ from fitted/converted
        // representations. The source is immutable, so this query can share
        // the primitive copy without changing the initial closure predicate.
        const auto ends = curve_detail::primitive_endpoints(v, &converted.curve);
        if (ends) {
            if (!have_endpoints)
                first = (*ends)[0];
            last = (*ends)[1];
            have_endpoints = true;
        }
        converted.report["member_index"] = i;
        conversions.push_back(std::move(converted.report));
        if (!combined)
            combined = std::move(converted.curve);
        else {
            auto joined = combine_tube_curves(*combined, converted.curve, false, true, budget);
            joined.report["member_index"] = i;
            joins.push_back(std::move(joined.report));
            combined = std::move(joined.curve);
        }
    }
    require(combined.has_value(), "native swept curve group has no converted curve");
    const bool source_closed = have_endpoints && curve_detail::endpoint_pair_closed(first, last);
    Json closure{
        {"requested", source_closed}, {"success", nullptr}, {"closed", combined->closed()}};
    if (source_closed) {
        require(combined->poles().size() <= UINT32_MAX, "native curve group closure control range");
        for (unsigned i = 0; i < 8; ++i)
            charge(budget, combined->poles().size());
        require(combined->order() <= 26 || combined->closed(), "native curve group closure order");
        charge(budget, 4 * std::size_t(combined->order()) * combined->order());
        auto closed =
            loft_detail::close_native_curve(*combined, unsigned(combined->poles().size()));
        closure = {{"requested", true},
                   {"success", closed.success},
                   {"closed", closed.closed},
                   {"conversion", std::move(closed.report)}};
        if (closed.success) {
            auto table = closed.curve.table();
            table["closed"] = closed.closed;
            combined = BsplineCurve::from_bgfb(table);
        }
    }
    Json report{{"scope", "native_swept_curve_array"},
                {"boundary_type", type},
                {"source_endpoint_closed", source_closed},
                {"members", std::move(conversions)},
                {"joins", std::move(joins)},
                {"closure", std::move(closure)},
                {"work_used", budget.work},
                {"source_geometry_reused", false}};
    return {std::move(*combined), std::move(report)};
}
TubeProfile convert_tube_profile(const Json &value, TubeBudget &budget) {
    const auto type = boundary(value);
    require(type <= 4, "native swept profile rejects union regions");
    TubeProfile result;
    result.report = {{"scope", "native_swept_profile_conversion"},
                     {"boundary_type", type},
                     {"rings", Json::array()},
                     {"source_geometry_reused", false}};
    if (type <= 3) {
        auto c = convert_tube_curve_array(value, budget);
        result.curves.push_back(std::move(c.curve));
        result.report["rings"].push_back(std::move(c.report));
    } else {
        const auto &members = value.at("curves");
        charge(budget, members.size());
        std::size_t controls = 0;
        for (std::size_t i = 0; i < members.size(); ++i) {
            const auto &child = members[i].at("geometry");
            const auto child_type = boundary(child);
            require(child_type == 2 || child_type == 3,
                    "native parity profile requires outer/inner groups");
            auto c = convert_tube_curve_array(child, budget);
            require(c.report.at("source_endpoint_closed").get<bool>(),
                    "native parity profile requires closed source endpoints");
            require(c.curve.poles().size() <= budget.max_control_points - controls,
                    "native parity profile total control budget exceeded");
            controls += c.curve.poles().size();
            c.report["member_index"] = i;
            result.curves.push_back(std::move(c.curve));
            result.report["rings"].push_back(std::move(c.report));
        }
    }
    result.report["work_used"] = budget.work;
    return result;
}
} // namespace p3d::swept_detail
