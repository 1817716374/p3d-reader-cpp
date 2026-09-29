#include "internal.hpp"
#include <p3d/view_frame.hpp>
#include "view_frame_oracle.hpp"
using namespace p3d;
unsigned view_frame_tests() {
    unsigned checks = 0;
    auto check = [&](bool value, const char *message) { ++checks; require(value, message); };
    auto near = [](double a, double b) { return std::abs(a-b) <= 2e-12 * (1 + std::abs(a) + std::abs(b)); };
    const auto oracle = Json::parse(view_frame_oracle);
    for (const auto &row : oracle.at("cases")) {
        NativeViewFrameInput input;
        input.range = row.at("range").get<std::array<double,6>>();
        if (!row.at("orientation").is_null()) input.orientation = row.at("orientation").get<std::array<double,9>>();
        if (!row.at("viewport").is_null()) input.viewport = row.at("viewport").get<std::array<std::int32_t,4>>();
        input.preserve_spatial_orientation = row.at("preserve_spatial_orientation").get<bool>(); input.slot = row.at("slot");
        const auto result = project_native_view_frame(input);
        check(result.resolved && result.reason.empty(), "bounded native frame input must resolve");
        bool equal = near(result.half_depth, row.at("half_depth").get<double>()) && result.slot_index == row.at("slot_index");
        for (unsigned i = 0; i < 3; ++i)
            equal &= near(result.origin[i], row.at("origin")[i].get<double>()) && near(result.delta[i], row.at("delta")[i].get<double>());
        for (unsigned i = 0; i < 9; ++i) equal &= near(result.orientation[i], row.at("result_orientation")[i].get<double>());
        check(equal, "range projection, transpose origin, aspect margin and signed slot match original full constructor");
    }
    NativeViewFrameInput input; input.range = {1,2,3,11,22,33};
    check(project_native_view_frame(input).resolved, "absent matrix proves identity without directory state");
    input.orientation = std::array<double,9>{1,1e-13,0,0,1,0,0,0,1};
    check(project_native_view_frame(input).resolved && !project_native_view_frame(input).orientation_reduced,
          "near identity is preserved by both directory paths");
    (*input.orientation)[1] = .2;
    check(!project_native_view_frame(input).resolved, "nonidentity matrix needs the model-directory predicate");
    input.preserve_spatial_orientation = false;
    check(project_native_view_frame(input).orientation_reduced, "ordinary predicate reduces nonidentity spatial matrix");
    input.range[0] = std::numeric_limits<double>::max();
    check(!project_native_view_frame(input).resolved, "native disconnect value cannot become a regular geometric bound");
    input.range[0] = std::numeric_limits<double>::quiet_NaN();
    check(!project_native_view_frame(input).resolved, "nonfinite range is explicitly outside verified numeric framing");
    input.range[0] = 0; (*input.orientation)[0] = std::numeric_limits<double>::infinity();
    check(!project_native_view_frame(input).resolved, "nonfinite orientation cannot produce a nominally complete frame");
    return checks;
}
