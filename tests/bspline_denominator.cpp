#include "internal.hpp"
#include "bspline_denominator.hpp"

unsigned bspline_denominator_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const std::string &message) {
        ++checks;
        require(ok, message);
    };
    auto surface = [](const std::vector<double> &wu, const std::vector<double> &wv) {
        Json p = Json::array(), w = Json::array();
        for (std::size_t j = 0; j < wv.size(); ++j)
            for (std::size_t i = 0; i < wu.size(); ++i) {
                // Homogeneous numerators P=(u,v,1), denominator W(u)*W(v).
                p.push_back(double(i) / (wu.size() - 1));
                p.push_back(double(j) / (wv.size() - 1));
                p.push_back(1.);
                w.push_back(wu[i] * wv[j]);
            }
        return Json{{"_type", "BsplineSurface"},
                    {"numPolesU", wu.size()},
                    {"numPolesV", wv.size()},
                    {"orderU", wu.size()},
                    {"orderV", wv.size()},
                    {"closedU", false},
                    {"closedV", false},
                    {"poles", p},
                    {"weights", w},
                    {"knotsU", nullptr},
                    {"knotsV", nullptr},
                    {"numRulesU", 0},
                    {"numRulesV", 0},
                    {"boundaries", nullptr},
                    {"holeOrigin", 0}};
    };
    BsplineMeshOptions options;
    options.max_uv_edge = .25;
    for (const auto middle : {0., -.1, -.9})
        for (const auto sign : {1., -1.}) {
            const auto table = surface({sign, sign * middle, sign}, {1, -.2, 1});
            const auto mesh = BsplineSurface::from_bgfb(table).mesh(options);
            check(mesh.report.at("status") == "complete" && !mesh.faces.empty(),
                  "regular mixed/zero-weight tensor surface meshes: " + mesh.report.dump());
            check(mesh.report.at("denominator").at("method") == "bernstein_interval_subdivision",
                  "mixed signs require a denominator proof");
            for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
                const auto uv = mesh.parameters[i];
                const auto u = uv[0], v = uv[1];
                const double a = sign * ((1 - u) * (1 - u) + 2 * middle * u * (1 - u) + u * u);
                const double b = (1 - v) * (1 - v) - .4 * v * (1 - v) + v * v;
                const auto &p = mesh.vertices[i];
                require(std::abs(p[0] - u / (a * b)) < 1e-11 &&
                            std::abs(p[1] - v / (a * b)) < 1e-11 &&
                            std::abs(p[2] - 1 / (a * b)) < 1e-11,
                        "independent rational polynomial evaluation");
            }
            check(mesh.vertices.size() == mesh.parameters.size(),
                  "all rational mesh vertices verified");
        }
    for (const auto weights : {std::vector<double>{1, -1, 1}, {1, -2, 1}, {0, 1, 1}, {0, 0, 0}}) {
        const auto mesh = BsplineSurface::from_bgfb(surface(weights, {1, 1})).mesh(options);
        check(mesh.report.at("status") == "incomplete" && mesh.faces.empty() &&
                  mesh.vertices.empty() && mesh.parameters.empty(),
              "zero/touching/crossing denominators publish no partial mesh");
    }
    // A very narrow zero crossing between coarse mesh vertices must not be
    // accepted merely because sampled vertices happen to be finite.
    const auto crossing = surface(
        {.0123 * .0123 - 1e-10, .0123 * .0123 - .0123 - 1e-10, (1 - .0123) * (1 - .0123) - 1e-10},
        {1, 1});
    const auto narrow = BsplineSurface::from_bgfb(crossing).mesh(options);
    check(narrow.report.at("status") == "incomplete" && narrow.faces.empty(),
          "narrow unsampled singular interval is not mistaken for a regular surface");
    auto narrow_regular = surface(
        {.0123 * .0123 + 1e-7, .0123 * .0123 - .0123 + 1e-7, (1 - .0123) * (1 - .0123) + 1e-7},
        {1, 1});
    const auto narrow_proof =
        certify_surface_denominator(BsplineSurface::from_bgfb(narrow_regular), 10000);
    check(narrow_proof.at("status") == "verified",
          "a strictly positive narrow minimum can be certified within a bounded budget");
    auto limited = options;
    limited.max_denominator_steps = 0;
    auto regular = surface({1, -.1, 1}, {1, 1});
    auto mesh = BsplineSurface::from_bgfb(regular).mesh(limited);
    check(mesh.report.at("denominator").at("reason") == "surface denominator work budget" &&
              mesh.vertices.empty(),
          "proof budget exhaustion remains explicit");
    check(BsplineSurface::from_bgfb(surface({1, 1, 1}, {1, 1})).mesh(limited).report.at("status") ==
              "complete",
          "uniform-sign fast path does not need the subdivision budget");

    // Refining the original knot vector represents exactly the same W:
    // quadratic [1,-.1,1] inserted at .5 gives [1,.45,.45,1].
    // Force the interval extraction path using a regular zero-weight V axis.
    auto inserted = surface({1, .45, .45, 1}, {1, 0, 1});
    inserted["orderU"] = 3;
    inserted["knotsU"] = {2, 2, 2, 3.5, 5, 5, 5};
    auto proof = certify_surface_denominator(BsplineSurface::from_bgfb(inserted), 1000000);
    check(proof.at("status") == "verified" && proof.at("verified_knot_rectangles") == 2,
          "local extraction works across internal knots and a non-normalized domain");

    // Full knot multiplicity permits separate one-sided denominator signs.
    auto jump = surface({1, 1, -1, -1}, {1, 1});
    jump["orderU"] = 2;
    jump["knotsU"] = {0, 0, .5, .5, 1, 1};
    mesh = BsplineSurface::from_bgfb(jump).mesh(options);
    check(mesh.report.at("status") == "complete" &&
              mesh.report.at("denominator").at("verified_knot_rectangles") == 2,
          "a discontinuous sign jump is not a continuous denominator root");

    // Uniform periodic quadratic denominator has isolated negative source
    // coefficients, but remains positive after span extraction/subdivision.
    auto periodic = surface({1, -.1, 1, 1}, {1, 1});
    periodic["closedU"] = true;
    periodic["orderU"] = 3;
    proof = certify_surface_denominator(BsplineSurface::from_bgfb(periodic), 1000000);
    check(proof.at("status") == "verified" && proof.at("verified_knot_rectangles") == 4,
          "periodic wrapped control indices participate in every active denominator span");
    mesh = BsplineSurface::from_bgfb(periodic).mesh(options);
    check(mesh.report.at("status") == "complete", "untrimmed periodic mixed-weight surface meshes");
    auto shifted = surface({1, -.1, 1, -.1, 1, -.1, 1}, {1, 0, 1, 0, 1, 0, 1});
    shifted["closedU"] = shifted["closedV"] = true;
    shifted["orderU"] = shifted["orderV"] = 3;
    shifted["knotsU"] =
        shifted["knotsV"] = {-1. / 3, 0, 0, 0, 1. / 3, 1. / 3, 2. / 3, 2. / 3, 1, 1, 1, 4. / 3};
    const auto shifted_surface = BsplineSurface::from_bgfb(shifted);
    proof = certify_surface_denominator(shifted_surface, 1000000);
    check(shifted_surface.u().periodic_pole_shift() == -1 &&
              shifted_surface.v().periodic_pole_shift() == -1 && proof.at("status") == "verified" &&
              proof.at("verified_knot_rectangles") == 9,
          "both native closed clamped-like pole shifts are applied before extracting weights");
    // Common power-of-two weight scaling cannot change the zero set.
    for (int exponent : {-400, 400}) {
        auto scaled = regular;
        for (auto &w : scaled["weights"])
            w = std::ldexp(w.get<double>(), exponent);
        proof = certify_surface_denominator(BsplineSurface::from_bgfb(scaled), 1000000);
        check(proof.at("status") == "verified",
              "denominator certificate respects projective scaling");
    }
    return checks;
}
