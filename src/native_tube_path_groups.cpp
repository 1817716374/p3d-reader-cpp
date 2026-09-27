#include "native_tube_path_groups.hpp"
#include "native_tube_facet_chain.hpp"
#include "native_tube_path_placement.hpp"
#include "native_curve_conversion.hpp"
#include "native_bezier.hpp"
#include "native_tube_facet_groups.hpp"
namespace p3d::swept_detail {
namespace {
void charge(TubeBudget &b, std::size_t n) {
    curve_detail::BezierWork{b.work, b.max_work}.charge(n);
}
double number(const Json &v) {
    require(v.is_number(), "native facet path coordinate type");
    const double x = v.get<double>();
    require(std::isfinite(x), "native facet path coordinate must be finite");
    return x;
}
Point3 direction(const Point3 &a, const Point3 &z) {
    Point3 out{};
    for (unsigned i = 0; i < 3; ++i) {
        out[i] = z[i] - a[i];
        require(std::isfinite(out[i]), "native facet path direction overflow");
    }
    return out;
}
const Json &members(const Json &source) {
    require(source.is_object() && source.value("_type", std::string()) == "CurveVector" &&
                source.contains("curves") && source.at("curves").is_array(),
            "native facet path needs a source CurveVector");
    return source.at("curves");
}
} // namespace
TubeFacetPathClassification classify_tube_facet_path(const Json &source, TubeBudget &b) {
    TubeFacetPathClassification out;
    out.report = {{"scope", "native_facet_path_classification"},
                  {"native_result", false},
                  {"visits", Json::array()}};
    if (source.is_null() || (source.is_object() && source.value("type", 0) == 4)) {
        out.report["reason"] = "null_or_parity_path";
        out.report["work_used"] = b.work;
        return out;
    }
    const auto &curves = members(source);
    require(curves.size() <= b.max_control_points, "native facet path member budget");
    std::vector<std::int32_t> pending;
    std::size_t next = 0;
    bool previous_linear = false;
    Point3 previous_direction{};
    auto flush = [&] {
        if (!pending.empty()) {
            charge(b, pending.size());
            out.groups.push_back(std::move(pending));
            pending.clear();
        }
    };
    auto append = [&] {
        require(next < b.max_control_points && next <= INT32_MAX,
                "native facet path patch index budget");
        charge(b, 1);
        pending.push_back(static_cast<std::int32_t>(next++));
    };
    auto linear = [&](const Point3 &d) {
        charge(b, 32);
        if (!previous_linear || !native_path_vectors_parallel(d, previous_direction))
            flush();
        previous_linear = true;
        append();
        previous_direction = d;
    };
    for (std::size_t i = 0; i < curves.size(); ++i) {
        charge(b, 1);
        require(curves[i].is_object() && curves[i].contains("geometry") &&
                    curves[i].at("geometry").is_object(),
                "native facet path has an uninitialized primitive");
        const auto &g = curves[i].at("geometry");
        const auto type = g.at("_type").get<std::string>();
        Json visit{{"source_member", i}, {"source_type", type}, {"first_patch_index", next}};
        if (type == "LineSegment") {
            const auto &s = g.at("segment");
            Point3 a{}, z{};
            for (unsigned k = 0; k < 3; ++k) {
                a[k] = number(s.at(std::string("point0") + "XYZ"[k]));
                z[k] = number(s.at(std::string("point1") + "XYZ"[k]));
            }
            linear(direction(a, z));
        } else if (type == "LineString") {
            const auto &flat = g.at("points");
            require(flat.is_array() && flat.size() % 3 == 0 &&
                        flat.size() / 3 <= b.max_control_points,
                    "native facet path LineString point layout/budget");
            if (flat.size() < 6) {
                visit["status"] = "native_failure";
                out.report["visits"].push_back(std::move(visit));
                out.report["reason"] = "linestring_has_fewer_than_two_points";
                out.report["discarded_pending_indices"] = pending;
                out.report["work_used"] = b.work;
                return out;
            }
            Point3 a{number(flat[0]), number(flat[1]), number(flat[2])};
            for (std::size_t j = 3; j < flat.size(); j += 3) {
                const Point3 z{number(flat[j]), number(flat[j + 1]), number(flat[j + 2])};
                linear(direction(a, z));
                a = z;
            }
        } else if (type == "BsplineCurve" || type == "EllipticArc") {
            if (type == "EllipticArc") {
                flush();
                charge(b, 128);
            }
            const auto c = type == "BsplineCurve" ? BsplineCurve::from_bgfb(g)
                                                  : curve_detail::ellipse_to_bspline(g);
            require(c.poles().size() <= b.max_control_points,
                    "native facet path curve control budget");
            auto knots = native_tube_facet_knot_data(c, b);
            const auto low = knots.at("left_active_index").get<std::size_t>();
            const auto high = knots.at("right_active_index").get<std::size_t>();
            // 128d00 validates this plan; the caller ignores its bool. Use the
            // native compressed active indices even when no interval remains.
            for (auto k = low; k < high; ++k) {
                if (type == "BsplineCurve")
                    flush();
                append();
            }
            if (type == "BsplineCurve")
                flush();
            previous_linear = false;
            visit["knots"] = std::move(knots);
        } else {
            // Native switch skips other primitive kinds without resetting the
            // pending group or the most recently observed straight direction.
            visit["ignored_by_native_classifier"] = true;
        }
        visit["next_patch_index"] = next;
        out.report["visits"].push_back(std::move(visit));
    }
    flush();
    out.success = true;
    out.report["native_result"] = true;
    out.report["patch_count"] = next;
    out.report["work_used"] = b.work;
    return out;
}
TubeFacetPathClassification partition_tube_facet_path(const Json &source, bool direct,
                                                      std::size_t selected, std::size_t total,
                                                      TubeBudget &b) {
    if (direct) {
        auto out = classify_tube_facet_path(source, b);
        out.report["partition_method"] = "direct_source_classification";
        return out;
    }
    const auto &curves = members(source);
    require(selected < curves.size(), "native facet path selected member index");
    require(curves.size() <= b.max_control_points && total <= b.max_control_points &&
                total <= INT32_MAX,
            "native facet path partition budget");
    TubeFacetPathClassification out;
    out.report = {{"scope", "native_facet_path_partition"},
                  {"native_result", false},
                  {"selected_member", selected},
                  {"generated_patch_count", total}};
    auto subset = [&](std::size_t first, std::size_t last) {
        Json list = Json::array();
        for (auto i = first; i < last; ++i) {
            charge(b, 1);
            list.push_back(curves[i]);
        }
        return Json{{"_type", "CurveVector"}, {"type", 1}, {"curves", std::move(list)}};
    };
    auto prefix = classify_tube_facet_path(subset(0, selected), b);
    out.report["prefix"] = prefix.report;
    if (!prefix.success) {
        out.report["reason"] = "prefix_classification_failed";
        out.report["work_used"] = b.work;
        return out;
    }
    auto suffix = classify_tube_facet_path(subset(selected + 1, curves.size()), b);
    out.report["suffix"] = suffix.report;
    if (!suffix.success) {
        out.report["reason"] = "suffix_classification_failed";
        out.report["work_used"] = b.work;
        return out;
    }
    const auto before = prefix.report.at("patch_count").get<std::size_t>();
    const auto after = suffix.report.at("patch_count").get<std::size_t>();
    // Native appends the prefix groups before checking whether a positive
    // number of middle patches remains; failure retains those appended groups.
    out.groups = std::move(prefix.groups);
    if (before >= total || after >= total - before) {
        out.report["reason"] = "no_generated_patch_for_selected_member";
        out.report["work_used"] = b.work;
        return out;
    }
    const auto middle = total - before - after;
    for (std::size_t i = 0; i < middle; ++i) {
        charge(b, 1);
        out.groups.push_back({static_cast<std::int32_t>(before + i)});
    }
    for (auto &g : suffix.groups) {
        charge(b, g.size());
        for (auto &index : g)
            index += static_cast<std::int32_t>(before + middle);
        out.groups.push_back(std::move(g));
    }
    out.success = true;
    out.report["native_result"] = true;
    out.report["selected_member_patch_count"] = middle;
    out.report["work_used"] = b.work;
    return out;
}
TubeFacetPathClassification
classify_generated_tube_facet_groups(const TubeFacetGroupGeneration &generated, TubeBudget &b) {
    require(generated.success && generated.preparation.placement && !generated.groups.empty() &&
                !generated.groups[0].empty(),
            "native facet path classification cannot read absent first member");
    const auto &path = generated.preparation.placement->branches.path;
    auto out = partition_tube_facet_path(
        path.sources.path.path, path.selected_member_planarity.at("planar").get<bool>(),
        path.selection.index, generated.groups[0][0].surfaces.size(), b);
    out.report["caller"] = "native_generated_facet_groups";
    out.report["selected_working_member"] = path.selection.index;
    return out;
}
} // namespace p3d::swept_detail
