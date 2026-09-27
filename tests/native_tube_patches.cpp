#include "native_tube_patches.hpp"
#include "native_tube_working_paths.hpp"
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_tube_patches_tests() {
    unsigned n = 0;
    auto check = [&](bool ok, const char *why) {
        ++n;
        require(ok, why);
    };
    auto rejects = [&](auto f, const char *why) {
        bool caught = false;
        try {
            f();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, why);
    };
    const auto curve = BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                                {"order", 2},
                                                {"closed", false},
                                                {"poles", {0., 0., 0., 2., 0., 0.}},
                                                {"knots", nullptr},
                                                {"weights", nullptr}});
    const auto ptr = std::make_shared<const BsplineCurve>(curve);
    const Matrix4 identity{{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
    TubeBudget budget;
    auto shared = place_tube_patch_sections({ptr, ptr}, &identity, nullptr, {false, false}, budget);
    check(shared.success && shared.original[0] == shared.original[1] &&
              shared.prefix[0] == shared.original[0] && shared.prefix[0]->poles() == curve.poles(),
          "two positional reversals on one shared curve cancel while preserving aliases");
    check(shared.report["reversals"].size() == 2 && ptr->poles() == curve.poles(),
          "native reversals do not mutate caller input");
    TubeBudget both_budget;
    auto both =
        place_tube_patch_sections({ptr, ptr}, &identity, &identity, {false, true}, both_budget);
    check(both.success && both.prefix[0] != both.prefix[1] && both.suffix[0] == both.suffix[1] &&
              both.prefix[0] != both.suffix[0],
          "two branch path clones prefix occurrences but retains original suffix aliases");
    check(both.prefix[0]->poles()[0][0] == 2 && both.prefix[1]->poles()[0][0] == 0 &&
              both.suffix[0]->poles()[0][0] == 2 && both.original[1] == both.suffix[1],
          "positional flag reverses only selected prefix or suffix working identity");
    TubeBudget partial_budget;
    auto partial =
        place_tube_patch_sections({ptr, ptr}, nullptr, &identity, {true}, partial_budget);
    check(partial.success && partial.suffix[1]->poles()[0][0] == 2,
          "compact flag list stays positional and actual aliases still carry reversal");
    rejects(
        [&] {
            TubeBudget b;
            place_tube_patch_sections({ptr}, &identity, &identity, {true, false}, b);
        },
        "selected out-of-range flag is never silently repaired");
    TubeBudget unused_budget;
    auto unused = place_tube_patch_sections({ptr}, &identity, nullptr, {true, true}, unused_budget);
    check(unused.success && unused.prefix[0]->poles() == curve.poles(),
          "unused flags perform no native indexed curve access");
    TubeBudget null_budget;
    auto null =
        place_tube_patch_sections({ptr, nullptr}, &identity, &identity, Json::array(), null_budget);
    check(!null.success && null.prefix.size() == 1 &&
              null.report["failure_step"] == "prefix_transform",
          "native null curve rejects transform with explicit partial working list");
    auto make_prepared = [&] {
        TubePatchPreparation p;
        p.success = true;
        p.placement.emplace();
        p.placement->success = true;
        p.placement->suffix_transform = identity;
        p.placement->branches.suffix.reused_curve_index = 0;
        p.placement->branches.path.selection.curves.push_back(
            BsplineCurve::from_bgfb({{"_type", "BsplineCurve"},
                                     {"order", 2},
                                     {"closed", false},
                                     {"poles", {1. / 37., .7, .2, 1.1, 2.3, .4}},
                                     {"weights", {45. / 23., 2.9}},
                                     {"knots", nullptr}}));
        p.sections.success = true;
        p.sections.original = {ptr, ptr};
        p.sections.suffix = p.sections.original;
        return p;
    };
    auto p = make_prepared();
    TubeBudget generation_budget;
    auto generated = generate_tube_patch_groups(p, generation_budget);
    double expected = 1. / 37.;
    const double weight = 45. / 23.;
    for (unsigned i = 0; i < 4; ++i)
        expected = (expected * (1 / weight)) * weight;
    check(generated.success && generated.groups.size() == 2 &&
              generated.preparation.placement->branches.path.selection.curves[0].poles()[0][0] ==
                  expected,
          "whole-ring generation carries all ordered path frame round trips across rings");
    check(p.placement->branches.path.selection.curves[0].poles()[0][0] == 1. / 37.,
          "whole-ring working path mutations leave prepared input unchanged");
    auto aliased = p;
    aliased.placement->prefix_transform = identity;
    aliased.placement->branches.prefix.reused_curve_index = 0;
    aliased.sections.prefix = aliased.sections.suffix;
    TubeBudget alias_budget;
    auto alias_result = generate_tube_patch_groups(aliased, alias_budget);
    expected = 1. / 37.;
    for (unsigned i = 0; i < 8; ++i)
        expected = (expected * (1 / weight)) * weight;
    check(alias_result.success && alias_result.report["shared_path"] == true &&
              alias_result.groups[0].surfaces.size() == 2 &&
              alias_result.preparation.placement->branches.path.selection.curves[0].poles()[0][0] ==
                  expected,
          "shared path branches accumulate both operations without duplicate state resets");
    auto empty = p;
    empty.sections.original.clear();
    empty.sections.suffix.clear();
    TubeBudget empty_budget;
    const auto empty_result = generate_tube_patch_groups(empty, empty_budget);
    check(!empty_result.success && empty_result.groups.empty() &&
              empty_result.report["failure_step"] == "empty_output",
          "zero whole-section groups is a native failure");
    auto failed = p;
    failed.sections.suffix[1].reset();
    TubeBudget failure_budget;
    const auto failed_result = generate_tube_patch_groups(failed, failure_budget);
    check(!failed_result.success && failed_result.groups.size() == 1 &&
              failed_result.report["failure_step"] == "empty_curve_surfaces",
          "empty later patch chain preserves earlier native output groups");
    auto bad_index = p;
    bad_index.placement->branches.suffix.reused_curve_index = 999;
    rejects(
        [&] {
            TubeBudget b;
            generate_tube_patch_groups(bad_index, b);
        },
        "invalid path alias index rejects instead of accessing invalid storage");
    rejects(
        [&] {
            TubeBudget b;
            b.max_work = 0;
            generate_tube_patch_groups(p, b);
        },
        "patch generation enforces cumulative work limit");
    return n;
}
