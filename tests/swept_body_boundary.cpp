#include <p3d/swept_body.hpp>
#include <cmath>
#include <future>
#include <limits>
#include <stdexcept>
using namespace p3d;
namespace {
SweptBodySurface plane() {
    SweptBodySurface s;
    s.geometry = {{"_type", "BsplineSurface"},
                  {"orderU", 2},
                  {"orderV", 2},
                  {"numPolesU", 2},
                  {"numPolesV", 2},
                  {"closedU", false},
                  {"closedV", false},
                  {"poles", {0, 0, 0, 2, 0, 0, 0, 3, 0, 2, 3, 0}},
                  {"weights", nullptr},
                  {"knotsU", nullptr},
                  {"knotsV", nullptr},
                  {"boundaries", nullptr},
                  {"holeOrigin", 1},
                  {"numRulesU", 2},
                  {"numRulesV", 2}};
    return s;
}
bool near(Point3 a, Point3 b) {
    for (unsigned k = 0; k < 3; ++k)
        if (std::abs(a[k] - b[k]) > 1e-10)
            return false;
    return true;
}
} // namespace
unsigned swept_body_boundary_tests() {
    unsigned n = 0;
    auto check = [&](bool yes, const char *why) {
        ++n;
        if (!yes)
            throw std::runtime_error(why);
    };
    const auto original = plane();
    auto s = original;
    auto outer = extract_swept_body_surface_boundary(s);
    check(outer.status == "extracted" && outer.curves.size() == 4,
          "untrimmed open surface publishes four native outer isocurves even with holeOrigin=1");
    const Point3 corners[]{{0, 0, 0}, {2, 0, 0}, {2, 3, 0}, {0, 3, 0}, {0, 0, 0}};
    for (std::size_t i = 0; i < 4; ++i)
        check(near(outer.curves[i].point_at(0), corners[i]) &&
                  near(outer.curves[i].point_at(1), corners[i + 1]),
              "native outer curve ordering and reversal make a directed parameter-domain cycle");
    SweptBodyBoundaryOptions no_outer;
    no_outer.include_outer = false;
    check(extract_swept_body_surface_boundary(s, no_outer).status == "native_empty",
          "suppressed outer without runtime records produces native null result");
    s.boundary_points = {{}, {{.2, .2}}};
    auto short_records = extract_swept_body_surface_boundary(s);
    check(short_records.status == "native_empty" &&
              short_records.report["skipped_short_records"] == Json({0, 1}),
          "present empty/singleton records are skipped but still suppress outer by hole origin");
    s.geometry["holeOrigin"] = 0;
    check(extract_swept_body_surface_boundary(s).curves.size() == 4,
          "holeOrigin=0 requests outer even when all active records are short");
    s = original;
    s.boundary_points = {{{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}}};
    auto edges = extract_swept_body_surface_boundary(s);
    check(edges.status == "extracted" && edges.curves.size() == 4 && edges.report["outer"].empty(),
          "active rectangle trim yields its four segmented edges without inferred outer");
    for (std::size_t i = 0; i < 4; ++i) {
        check(near(edges.curves[i].point_at(0), corners[i]) &&
                  near(edges.curves[i].point_at(1), corners[i + 1]),
              "UV boundary segmentation keeps saved edge directions");
        check(edges.report["spans"][i]["branch"] == "isocurve_segment" &&
                  edges.report["spans"][i]["first"] == i && edges.report["spans"][i]["count"] == 2,
              "edge spans share their corner and retain source point provenance");
    }
    s.geometry["holeOrigin"] = 0;
    auto duplicate_outer = extract_swept_body_surface_boundary(s);
    check(duplicate_outer.curves.size() == 8 && duplicate_outer.report["outer"].size() == 4,
          "native explicit trim and equal outer geometry are not deduplicated");
    s = original;
    s.boundary_points = {{{.3, .2}, {.30000001, .5}, {.3, .9}}};
    auto iso = extract_swept_body_surface_boundary(s);
    check(iso.status == "extracted" && iso.curves.size() == 1 &&
              iso.report["spans"][0]["iso_axis"] == "U" &&
              near(iso.curves[0].point_at(0), {.6, .6, 0}) &&
              near(iso.curves[0].point_at(1), {.6, 2.7, 0}),
          "near constant U chooses first saved U without averaging middle UVs");
    s.geometry["knotsU"] = {2, 2, 5, 5};
    s.geometry["knotsV"] = {7, 7, 11, 11};
    s.boundary_points = {{{.3, 7.5}, {.3, 10}}};
    auto raw_domain = extract_swept_body_surface_boundary(s);
    check(raw_domain.status == "extracted" &&
              raw_domain.report["spans"][0]["segment_fractions"] == Json({.125, .75}) &&
              near(raw_domain.curves[0].point_at(0), {.6, .375, 0}) &&
              near(raw_domain.curves[0].point_at(1), {.6, 2.25, 0}),
          "iso fixed coordinate is a fraction but varying UV endpoints map from raw knot domain");
    s = original;
    s.boundary_points = {{{.9, .4}, {.6, .40000001}, {.2, .4}}};
    auto reverse = extract_swept_body_surface_boundary(s);
    check(reverse.status == "extracted" && reverse.report["spans"][0]["iso_axis"] == "V" &&
              near(reverse.curves[0].point_at(0), {1.8, 1.2, 0}) &&
              near(reverse.curves[0].point_at(1), {.4, 1.2, 0}),
          "decreasing varying parameter reverses the extracted constant V segment");
    s.boundary_points = {{{.2, .2}, {.4, .3}, {.4, .3}, {.8, .8}}};
    auto polyline = extract_swept_body_surface_boundary(s);
    check(polyline.status == "extracted" && polyline.curves.size() == 1 &&
              polyline.curves[0].order() == 2 && !polyline.curves[0].closed() &&
              polyline.curves[0].poles().size() == 4,
          "general UV span keeps every sampled point including repeated points");
    check(near(polyline.curves[0].poles()[1], {.8, .9, 0}) &&
              polyline.curves[0].poles()[1] == polyline.curves[0].poles()[2] &&
              polyline.curves[0].knots() == std::vector<double>({0, 0, 1. / 3, 2. / 3, 1, 1}),
          "general span is uniform degree one, not chord-length fitted or simplified");
    auto ignored_curves = s;
    ignored_curves.boundary_curves = {outer.curves};
    auto ignored = extract_swept_body_surface_boundary(ignored_curves);
    check(ignored.status == "extracted" && ignored.report == polyline.report &&
              ignored.curves[0].poles() == polyline.curves[0].poles(),
          "this native query consumes UV caches rather than replacing them with pcurves");
    auto degenerate = original;
    degenerate.boundary_points = {{{.4, .4}, {.4, .4}}};
    auto fallback = extract_swept_body_surface_boundary(degenerate);
    check(
        fallback.status == "extracted" && fallback.curves.size() == 1 &&
            fallback.report["spans"][0]["segment_success"] == false &&
            fallback.report["spans"][0]["branch"] == "sampled_polyline" &&
            fallback.curves[0].poles()[0] == fallback.curves[0].poles()[1],
        "native iso segmentation failure falls back to saved vertices without dropping duplicates");
    auto quadratic = original;
    quadratic.geometry["orderV"] = 3;
    quadratic.geometry["numPolesV"] = 3;
    quadratic.geometry["poles"] = {0, 0, 0, 2, 0, 0, 0, 1.5, 1, 2, 1.5, 1, 0, 3, 0, 2, 3, 0};
    quadratic.boundary_points = {{{.25, .1}, {.25, .9}}};
    auto curved_iso = extract_swept_body_surface_boundary(quadratic);
    check(curved_iso.status == "extracted" && curved_iso.curves.size() == 1 &&
              curved_iso.curves[0].order() == 3 &&
              near(curved_iso.curves[0].point_at(.5), {.5, 1.5, .5}),
          "iso extraction preserves the curved analytic section instead of its endpoint chord");
    auto rational = s;
    rational.geometry["weights"] = {2, 2, 2, 2};
    for (auto &x : rational.geometry["poles"])
        x = 2 * x.get<double>();
    auto weighted = extract_swept_body_surface_boundary(rational);
    check(weighted.status == "extracted" && !weighted.curves[0].rational() &&
              weighted.curves[0].poles().size() == polyline.curves[0].poles().size(),
          "UV vertex evaluation deweights rational source before building polynomial polyline");
    for (std::size_t i = 0; i < weighted.curves[0].poles().size(); ++i)
        check(near(weighted.curves[0].poles()[i], polyline.curves[0].poles()[i]),
              "deweighted boundary point agrees geometrically while retaining native rounding");
    rational.geometry["weights"] = {0, 0, 0, 0};
    auto zero_weight = extract_swept_body_surface_boundary(rational);
    check(zero_weight.status == "not_extracted" && zero_weight.curves.empty(),
          "surface vertex evaluation at zero denominator is not replaced by zero-weight fallback");
    s = original;
    const double tol = 1e-10, high = 1 - tol;
    for (const auto &record : std::vector<std::vector<Point2>>{{{tol, .2}, {tol, .7}},
                                                               {{tol, .2}, {0, .7}},
                                                               {{high, .2}, {high, .7}},
                                                               {{.2, .2}, {0, .4}, {0, .8}},
                                                               {{0, 0}, {0, 1}, {1, 1}}}) {
        s.boundary_points = {record};
        auto r = extract_swept_body_surface_boundary(s);
        check(r.status == "extracted", "edge tolerance boundary records remain extractable");
        const auto edge = r.report["spans"][0]["edge"].get<unsigned>();
        if (record[0][0] == tol)
            check(edge == (record[1][0] == tol ? 0u : 1u),
                  "low edge uses inclusive initial and strict subsequent comparison");
        else if (record[0][0] == high)
            check(edge == 2, "high edge keeps inclusive comparison on later points");
        else if (record[0][0] == .2)
            check(edge == 0 && r.report["spans"][0]["count"] == 2 &&
                      r.report["spans"][1]["first"] == 1,
                  "general span includes its first edge vertex and shares it with next span");
        else
            check(edge == 1, "corner starts prefer U edge before V edge");
    }
    for (unsigned mask = 0; mask < 4; ++mask) {
        s = original;
        s.geometry["closedU"] = bool(mask & 1);
        s.geometry["closedV"] = bool(mask & 2);
        auto closed = extract_swept_body_surface_boundary(s);
        check(closed.status == "extracted" &&
                  closed.curves.size() == (mask == 1 || mask == 2 ? 2 : 4),
              "single periodic direction gives two outer curves; both periodic retain native four");
        if (mask == 1)
            check(closed.report["outer"][0]["axis"] == "V" &&
                      closed.report["outer"][0]["fraction"] == 0,
                  "closed U leaves V zero then reversed V one outer curves");
        if (mask == 2)
            check(closed.report["outer"][0]["axis"] == "U" &&
                      closed.report["outer"][0]["fraction"] == 1,
                  "closed V leaves U one then reversed U zero outer curves");
    }
    for (unsigned kind = 0; kind < 4; ++kind) {
        SweptBodyBoundaryOptions options;
        if (kind == 0)
            options.max_curves = 3;
        if (kind == 1)
            options.max_control_points = 3;
        if (kind == 2)
            options.max_work = 1;
        if (kind == 3)
            options.max_curves = 0;
        auto limited = extract_swept_body_surface_boundary(original, options);
        check(limited.status == "not_extracted" && limited.curves.empty() &&
                  limited.report.contains("reason"),
              "control/work/output budget failures discard partial boundaries");
    }
    auto invalid = original;
    invalid.boundary_points = {{{.2, .3}, {std::numeric_limits<double>::quiet_NaN(), .8}}};
    check(extract_swept_body_surface_boundary(invalid).status == "not_extracted",
          "nonfinite UV never silently selects an edge or produces a fake boundary");
    invalid = original;
    invalid.geometry["boundaries"] = {{"_type", "CurveVector"}, {"type", 2}};
    check(extract_swept_body_surface_boundary(invalid).status == "not_extracted",
          "serialized BGFB trim tree is not silently substituted for runtime UV records");
    auto future = std::async(std::launch::async,
                             [&] { return extract_swept_body_surface_boundary(original); });
    auto concurrent = future.get();
    check(concurrent.report == outer.report &&
              concurrent.curves[0].poles() == outer.curves[0].poles() &&
              original.geometry == plane().geometry,
          "independent calls do not mutate source or share native diagnostic counters");
    return n;
}
