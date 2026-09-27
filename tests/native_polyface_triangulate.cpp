#include "native_polyface_triangulate.hpp"
#include <future>
using namespace p3d;
using namespace p3d::swept_detail;
unsigned native_polyface_triangulate_tests() {
    unsigned checks = 0;
    auto check = [&](bool x, const char *why) {
        ++checks;
        require(x, why);
    };
    auto rejects = [&](auto fn) {
        bool caught = false;
        try {
            fn();
        } catch (const std::exception &) {
            caught = true;
        }
        check(caught, "native visitor rejects invalid memory access or exhausted budget");
    };
    NativePolyfaceVisitorFacet square;
    square.points = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    square.client_indices = {std::vector<std::int32_t>{10, 20, 30, 40, 10},
                             {6, 2, 8, 4, 6},
                             {15, 11, 18, 12, 15},
                             {50, 20, 70, 30, 50},
                             {5, 5, 9, 9, 5}};
    square.visible = {0, 1, 1, 1, 0};
    NativePolyfaceIndexState original;
    original.active.fill(true);
    for (auto &v : original.indices)
        v = {99, 0};
    auto run = [&](std::vector<NativePolyfaceVisitorFacet> f, NativePolyfaceIndexState s) {
        TubeBudget b;
        return triangulate_native_polyface_facets(f, s, b);
    };
    const auto expected = run({square}, original);
    check(expected.native_succeeded && expected.complete &&
              expected.output.active == original.active,
          "all independent prepared channels publish successfully");
    check(expected.output.indices[point_channel] ==
              std::vector<std::int32_t>{-11, -21, 41, 0, -41, 21, 31, 0},
          "point channel combines local diagonal sign with original hidden edge");
    check(expected.output.indices[parameter_channel] ==
              std::vector<std::int32_t>{7, 3, 5, 0, 5, 3, 9, 0},
          "parameter indices follow their independent source mapping without hidden signs");
    check(expected.output.indices[normal_channel] ==
              std::vector<std::int32_t>{16, 12, 13, 0, 13, 12, 19, 0},
          "normal indices use their own source pool");
    check(expected.output.indices[color_channel] ==
              std::vector<std::int32_t>{51, 21, 31, 0, 31, 21, 71, 0},
          "color indices use their own source pool");
    check(expected.output.indices[face_channel] ==
              std::vector<std::int32_t>{6, 6, 10, 0, 10, 6, 10, 0},
          "face data remains per-corner and is not inferred from triangle numbering");
    for (const auto &v : original.indices)
        check(v == std::vector<std::int32_t>{99, 0}, "original index buffers remain immutable");
    for (unsigned bits = 0; bits < 32; ++bits) {
        auto s = original;
        for (unsigned k = 0; k < 5; ++k)
            s.active[k] = bool(bits & (1 << k));
        const auto r = run({square}, s);
        check(r.complete, "all explicit activity combinations are supported");
        for (unsigned k = 0; k < 5; ++k)
            check(r.output.indices[k] == (s.active[k]          ? expected.output.indices[k]
                                          : k == point_channel ? std::vector<std::int32_t>{}
                                                               : s.indices[k]),
                  "inactive point buffer is replaced while other inactive buffers are retained");
    }
    for (bool color : {false, true}) {
        auto short_face = square;
        const auto channel = color ? color_channel : normal_channel;
        short_face.client_indices[channel].resize(1);
        const auto r = run({square, short_face, square}, original);
        check(!r.output.active[channel] && r.output.indices[channel].empty(),
              "missing optional channel disables and clears even earlier successful output");
        check(r.native_succeeded == !color && !r.complete &&
                  r.report.at("native_error_count") == int(color),
              "native normal shortage is not an error but color shortage is");
        check(r.output.indices[point_channel].size() == 24,
              "channel failure retains every successful geometric face");
        check(r.output.active[parameter_channel] &&
                  r.output.indices[parameter_channel].size() == 24,
              "missing optional channel does not disturb an independent parameter channel");
    }
    {
        auto short_face = square;
        short_face.client_indices[normal_channel].clear();
        short_face.client_indices[color_channel].clear();
        const auto r = run({short_face, square}, original);
        check(!r.native_succeeded && r.report.at("normal_channel_drops") == 1 &&
                  r.report.at("color_channel_drops") == 1 && r.report.at("native_error_count") == 1,
              "normal and color disable paths have distinct error counting");
    }
    {
        auto bad = square;
        bad.points = {{0, 0, 0}, {4, 4, 0}, {0, 4, 0}, {4, 0, 0}, {0, 0, 0}};
        // This face must be skipped before any visitor attribute access.
        for (auto &v : bad.client_indices)
            v.clear();
        bad.visible.clear();
        const auto r = run({square, bad, square}, original);
        check(!r.native_succeeded && !r.complete && r.report.at("failed_facets") == 1 &&
                  r.output.indices[point_channel].size() == 16,
              "failed large face is skipped while successful faces publish");
        for (unsigned k = 0; k < 5; ++k) {
            auto doubled = expected.output.indices[k];
            doubled.insert(doubled.end(), expected.output.indices[k].begin(),
                           expected.output.indices[k].end());
            check(r.output.indices[k] == doubled,
                  "failed facet cannot leak partial local indices into any channel");
        }
    }
    {
        auto large = square;
        large.points = {{0, 0, 0}, {2, 0, 0}, {3, 1, 0}, {1, 3, 0}, {0, 1, 0}};
        for (unsigned k = 0; k < 5; ++k) {
            large.client_indices[k].clear();
            for (unsigned i = 0; i < 6; ++i)
                large.client_indices[k].push_back(100 * k + int(i % 5));
        }
        large.visible.assign(6, 1);
        const auto r = run({large}, original);
        check(r.complete && r.output.indices[0].size() == 12,
              "projected-loop triangles feed independent visitor channels");
        for (std::size_t i = 0; i < r.output.indices[0].size(); ++i)
            for (unsigned k = 1; k < 5; ++k)
                check(r.output.indices[k][i] ==
                          (r.output.indices[0][i] ? 100 * int(k) + std::abs(r.output.indices[0][i])
                                                  : 0),
                      "large-face output keeps separate per-corner attribute identifiers");
    }
    {
        auto short_face = square;
        short_face.points.resize(2);
        const auto r = run({short_face}, original);
        check(r.native_succeeded && r.output.indices[0] == std::vector<std::int32_t>{-11, 21, 0},
              "native low-count facet is preserved rather than turned into a triangle");
        auto no_faces = run({}, original);
        check(no_faces.complete, "empty traversal completes");
        for (const auto &v : no_faces.output.indices)
            check(v.empty(), "active output buffers replace old buffers on empty traversal");
    }
    for (unsigned channel : {point_channel, parameter_channel, face_channel}) {
        auto f = square;
        f.client_indices[channel].clear();
        rejects([&] { run({f}, original); });
    }
    {
        auto f = square;
        f.visible.clear();
        rejects([&] { run({f}, original); });
    }
    {
        auto f = square;
        f.client_indices[normal_channel][0] = INT32_MAX;
        rejects([&] { run({f}, original); });
    }
    {
        auto f = square;
        f.client_indices[color_channel][0] = -1;
        rejects([&] { run({f}, original); });
    }
    TubeBudget measured;
    triangulate_native_polyface_facets({square}, original, measured);
    for (auto limit : {std::size_t(0), measured.work / 2, measured.work - 1}) {
        TubeBudget b;
        b.max_work = limit;
        rejects([&] { triangulate_native_polyface_facets({square}, original, b); });
        check(original.indices[0] == std::vector<std::int32_t>{99, 0},
              "resource failure cannot partially mutate original mesh state");
    }
    {
        TubeBudget b;
        b.max_control_points = 4;
        rejects([&] { triangulate_native_polyface_facets({square}, original, b); });
    }
    auto task = [=] {
        TubeBudget b;
        auto r = triangulate_native_polyface_facets({square}, original, b);
        return Json{{"report", r.report}, {"indices", r.output.indices}};
    };
    const auto reference = task();
    std::vector<std::future<Json>> jobs;
    for (unsigned i = 0; i < 4; ++i)
        jobs.push_back(std::async(std::launch::async, task));
    for (auto &job : jobs)
        check(job.get() == reference, "visitor output and channel activity are local to each call");
    return checks;
}
