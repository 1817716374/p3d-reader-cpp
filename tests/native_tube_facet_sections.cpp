#include "native_tube_facet_sections.hpp"
#include "native_curve_affine.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
namespace {
Matrix4 translation(double x) {
    Matrix4 m{};
    for (unsigned i = 0; i < 4; ++i)
        m[i][i] = 1;
    m[0][3] = x;
    return m;
}
Json line(Point3 a, Point3 b) {
    Json p;
    for (unsigned i = 0; i < 3; ++i) {
        p[std::string("point0") + "XYZ"[i]] = a[i];
        p[std::string("point1") + "XYZ"[i]] = b[i];
    }
    return {{"_type", "LineSegment"}, {"segment", p}};
}
Json group(Json values, unsigned type = 1) {
    Json members = Json::array();
    for (const auto &v : values)
        members.push_back({{"geometry", v}});
    return {{"_type", "CurveVector"}, {"type", type}, {"curves", members}};
}
auto curve() {
    return std::make_shared<const BsplineCurve>(
        BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                 {"order", 2},
                                 {"closed", false},
                                 {"poles", {1., 2., 3., 4., 5., 6.}},
                                 {"knots", nullptr},
                                 {"weights", nullptr}}));
}
} // namespace
unsigned native_tube_facet_sections_tests() {
    unsigned count = 0;
    auto check = [&](bool ok, const char *why) {
        ++count;
        require(ok, why);
    };
    auto rejects = [&](auto fn, const char *why) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, why);
    };
    const auto source = curve(), independent = curve();
    const TubeCurveViews views{source, source, independent};
    const auto prefix = translation(10), suffix = translation(20), id = translation(0);
    auto place = [&](const TubeCurveViews &v, const Matrix4 *a, const Matrix4 *b, bool flag) {
        TubeBudget budget;
        return place_tube_facet_sections(v, a, b, flag, budget);
    };
    for (bool flag : {false, true}) {
        auto r = place(views, &prefix, &suffix, flag);
        check(r.success && r.prefix.size() == 3 && r.suffix.size() == 3,
              "both path branches prepare all section occurrences");
        check(r.prefix[0] != r.prefix[1] && r.prefix[1] != r.prefix[2] &&
                  r.prefix[0] != r.suffix[0],
              "prefix clone separates occurrences and branches");
        check(r.original == r.suffix && r.suffix[0] == r.suffix[1] && r.suffix[0] != r.suffix[2],
              "suffix retains actual aliases without deduplication");
        check(r.prefix[0]->poles()[0][0] == (flag ? 11 : 14) && r.prefix[1]->poles()[0][0] == 11,
              "prefix reversal changes only first independent clone");
        check(r.suffix[0]->poles()[0][0] == (flag ? 44 : 41) &&
                  r.suffix[1]->poles() == r.suffix[0]->poles() && r.suffix[2]->poles()[0][0] == 21,
              "suffix transforms every occurrence before first-curve alias reversal");
        check(source->poles()[0] == Point3{1, 2, 3} && independent->poles()[0] == Point3{1, 2, 3},
              "branch preparation leaves original inputs immutable");
        auto left = place(views, &prefix, nullptr, flag);
        check(left.success && left.original == left.prefix && left.suffix.empty() &&
                  left.prefix[0] == left.prefix[1] &&
                  left.prefix[0]->poles()[0][0] == (flag ? 21 : 24),
              "prefix-only branch shares native conversion list including reversal");
        auto right = place(views, nullptr, &suffix, flag);
        check(right.success && right.prefix.empty() && right.original == right.suffix &&
                  right.suffix[0]->poles()[0][0] == (flag ? 44 : 41),
              "suffix-only branch shares native conversion list including reversal");
    }
    auto identity = place(views, &id, &id, false);
    check(identity.prefix[0] != source && identity.prefix[1] != source &&
              identity.prefix[0] != identity.prefix[1] && identity.suffix[0] == source,
          "identity transform still clones prefix while suffix retains native identity");
    const TubeCurveViews invalid{source, nullptr, source};
    auto closed = std::make_shared<const BsplineCurve>(
        BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                 {"order", 2},
                                 {"closed", true},
                                 {"poles", {0., 0., 0., 1., 0., 0., 1., 1., 0., 0., 1., 1.}},
                                 {"weights", nullptr},
                                 {"knots", {0., 2., 4., 6., 8., 10., 12.}}}));
    auto retained = place({closed, closed}, &id, nullptr, false);
    check(retained.success && retained.report["reversal"]["native_result"] == false &&
              retained.prefix[0]->closed() && retained.original == retained.prefix &&
              retained.prefix[0] == retained.prefix[1] &&
              retained.prefix[0]->poles() == closed->poles(),
          "in-place reversal opening failure retains shared geometry and does not fail caller");
    auto rational = std::make_shared<const BsplineCurve>(
        BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                 {"order", 2},
                                 {"closed", false},
                                 {"poles", {2., 4., 6., 12., 15., 18.}},
                                 {"weights", {2., 3.}},
                                 {"knots", nullptr}}));
    auto weighted = place({rational}, &prefix, &suffix, true);
    check(weighted.prefix[0]->poles()[0] == Point3{22, 4, 6} &&
              weighted.suffix[0]->poles()[0] == Point3{72, 15, 18} &&
              weighted.suffix[0]->weights() == std::vector<double>{3, 2},
          "rational branch transform uses homogeneous translation before reversing controls and "
          "weights");
    auto failed = place(invalid, &prefix, &suffix, false);
    check(!failed.success && failed.prefix.size() == 1 && failed.suffix.empty() &&
              failed.original == invalid && failed.report["failure_step"] == "prefix_transform",
          "prefix clone failure prevents suffix transform and all reversals");
    failed = place(invalid, &prefix, nullptr, false);
    check(!failed.success && failed.original == failed.prefix &&
              failed.prefix[0] == failed.prefix[2] && failed.prefix[0]->poles()[0][0] == 11,
          "prefix-only failure preserves partial shared state without reversing");
    failed = place(invalid, nullptr, &suffix, true);
    check(!failed.success && failed.suffix[0]->poles()[0][0] == 21 &&
              failed.report["failure_step"] == "suffix_transform",
          "suffix failure preserves partial state without selected reversal");
    rejects([&] { place({}, &id, nullptr, true); }, "empty branch first-curve read is rejected");
    rejects([&] { place(views, nullptr, nullptr, false); },
            "missing both path branches is rejected");
    TubeBudget measured;
    place_tube_facet_sections(views, &prefix, &suffix, false, measured);
    TubeBudget exact;
    exact.max_work = measured.work;
    check(place_tube_facet_sections(views, &prefix, &suffix, false, exact).success &&
              exact.work == measured.work,
          "section preparation shares deterministic work budget");
    TubeBudget short_budget;
    short_budget.max_work = measured.work - 1;
    rejects([&] { place_tube_facet_sections(views, &prefix, &suffix, false, short_budget); },
            "section reversal cannot exceed shared budget");

    const auto profile = group({line({-1, -1, 0}, {1, -1, 0}), line({1, -1, 0}, {1, 1, 0}),
                                line({1, 1, 0}, {-1, 1, 0}), line({-1, 1, 0}, {-1, -1, 0})},
                               2);
    const auto path = group({line({0, 0, 0}, {0, 0, 4})});
    const auto original_profile = profile, original_path = path;
    TubeBudget budget;
    auto input = prepare_tube_facet_inputs(profile, path, budget);
    check(input.success && input.placement->success && input.profile.groups.size() == 1 &&
              input.profile.groups[0].size() == 4,
          "full source preparation retains profile members");
    check(input.orientation["flags"] == Json::array({false}) &&
              input.orientation["source_rings"] == Json::array({0}),
          "source orientation uses selected original path tangent");
    for (std::size_t i = 0; i < 4; ++i) {
        auto r = prepare_tube_facet_sections(input, 0, i, budget);
        check(r.success && r.prefix.empty() && r.suffix.size() == 1 &&
                  r.report["partition_member"] == i && r.report["orientation_source_ring"] == 0,
              "member preparation connects placement and positional orientation");
        const auto p = input.profile.groups[0][i].geometry().at("segment");
        const Point3 point{p.at("point0X").get<double>(), p.at("point0Y").get<double>(),
                           p.at("point0Z").get<double>()};
        check(r.suffix[0]->poles()[0] ==
                  curve_detail::affine_polynomial_point(*input.placement->suffix_transform, point),
              "source vertex reaches selected frame");
    }
    check(profile == original_profile && path == original_path,
          "complete preparation retains original source JSON");
    auto open_type_closed = profile;
    open_type_closed["type"] = 1;
    auto failed_orientation = prepare_tube_facet_inputs(open_type_closed, path, budget);
    check(!failed_orientation.success && failed_orientation.placement->success &&
              failed_orientation.profile.groups.empty() &&
              failed_orientation.report["failure_step"] == "source_orientation",
          "native closed open-type orientation rejection prevents partitioning");
    const auto open_ring = group({line({-1, 0, 0}, {1, 0, 0})});
    auto mixed = prepare_tube_facet_inputs(group({profile, open_ring, profile}, 4), path, budget);
    check(mixed.success && mixed.profile.groups.size() == 3 &&
              mixed.orientation["source_rings"] == Json::array({0, 2}),
          "source preparation preserves native compacted flags and uncompressed profile groups");
    check(prepare_tube_facet_sections(mixed, 1, 0, budget).report["orientation_source_ring"] == 2,
          "compacted orientation flag is read by native group index without repairing "
          "correspondence");
    rejects([&] { prepare_tube_facet_sections(mixed, 2, 0, budget); },
            "later group cannot read beyond compacted native orientation vector");
    rejects([&] { prepare_tube_facet_sections(input, 1, 0, budget); }, "invalid group rejected");
    auto compact = input;
    compact.orientation["flags"] = Json::array();
    rejects([&] { prepare_tube_facet_sections(compact, 0, 0, budget); },
            "missing native orientation index is not inferred or defaulted");
    compact = input;
    compact.orientation["source_rings"][0] = 3;
    check(prepare_tube_facet_sections(compact, 0, 0, budget).report["orientation_source_ring"] == 3,
          "native flag index remains positional even if source-ring map differs");
    auto rejected = prepare_tube_facet_inputs(profile, nullptr, budget);
    check(!rejected.success && !rejected.placement && rejected.orientation.is_null() &&
              rejected.report["failure_step"] == "source_validation",
          "source rejection prevents placement and orientation");
    std::vector<std::future<bool>> jobs;
    for (unsigned i = 0; i < 8; ++i)
        jobs.push_back(std::async(std::launch::async, [&] {
            TubeBudget local;
            auto prepared = prepare_tube_facet_inputs(profile, path, local);
            return prepare_tube_facet_sections(prepared, 0, 0, local).success;
        }));
    for (auto &job : jobs)
        check(job.get(), "facet input and section preparation is reentrant");
    return count;
}
