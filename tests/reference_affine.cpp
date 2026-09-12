#include "internal.hpp"

unsigned reference_affine_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto put = [](Bytes &b, std::size_t at, std::uint64_t v, unsigned size) {
        for (unsigned i = 0; i < size; ++i)
            b.at(at + i) = std::uint8_t(v >> (8 * i));
    };
    auto number = [&](Bytes &b, std::size_t at, double v) {
        std::uint64_t bits;
        std::memcpy(&bits, &v, 8);
        put(b, at, bits, 8);
    };
    auto wire = [&](bool legacy, Point3 p, Point3 q, double scale = 2.) {
        Bytes b(legacy ? 348 : 372, 0);
        put(b, 4, 13, 2);
        put(b, 8, (b.size() - 4) / 2, 4);
        put(b, 12, (b.size() - 4) / 2, 4);
        for (unsigned i = 0; i < 3; ++i) {
            number(b, (legacy ? 188 : 196) + 8 * i, p[i]);
            number(b, (legacy ? 164 : 172) + 8 * i, q[i]);
        }
        const Matrix3 r{{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}}};
        for (unsigned i = 0; i < 3; ++i)
            for (unsigned j = 0; j < 3; ++j)
                number(b, (legacy ? 212 : 220) + 24 * i + 8 * j, r[i][j]);
        number(b, legacy ? 284 : 292, scale);
        return b;
    };
    auto query = [&](Point3 p, Point3 q, double scale = 2.) {
        return native_reference_input(wire(false, p, q, scale));
    };
    auto input = query({10, 20, 30}, {1, 2, 3});
    ReferenceAffineContext c;
    c.origin.model_attached = false;
    check(reference_affine_transform(input, c)["reason"] ==
              "reference_scale_provider_identity_unknown",
          "unknown extension context must not default to no scale provider");
    c.provider_id = 0;
    auto a = reference_affine_transform(input, c);
    const Matrix4 expected{{{0, -2, 0, 14}, {2, 0, 0, 18}, {0, 0, 2, 24}, {0, 0, 0, 1}}};
    check(a["status"] == "computed" && a["matrix"] == expected &&
              a["axis_scales"] == Point3{2, 2, 2},
          "reference rotation scale and right translation follow the native affine construction");
    for (bool legacy : {false, true}) {
        const auto v = parse_native(wire(legacy, {10, 20, 30}, {1, 2, 3}))[0]["reference_input"];
        check(v["affine_inputs"]["translation_point"]["value"] == Point3{10, 20, 30} &&
                  v["affine_inputs"]["reference_point"]["value"] == Point3{1, 2, 3} &&
                  v["affine_inputs"]["translation_point"]["source_offset"] ==
                      (legacy ? 188 : 196) &&
                  v["affine_inputs"]["reference_point"]["source_offset"] == (legacy ? 164 : 172) &&
                  reference_affine_transform(v, c)["matrix"] == expected,
              "modern and legacy point loading preserve physical offsets and produce the same "
              "transform");
    }
    c.force_z_scale = false;
    check(reference_affine_transform(input, c)["reason"] == "reference_model_z_scale_state_unknown",
          "local affine query needs model state when Z scale is not forced");
    c.model_z_scale_enabled = false;
    a = reference_affine_transform(input, c);
    check(a["matrix"][2] == std::array<double, 4>{0, 0, 1, 27},
          "disabled model Z scaling uses one");
    check(compose_reference_chain_transforms({a})["status"] == "not_evaluated",
          "native chain query cannot consume a local query with unforced Z scaling");
    c.model_z_scale_enabled = true;
    check(reference_affine_transform(input, c)["matrix"] == expected,
          "model Z scaling can enable reference scale");
    c.force_z_scale = true;
    c.model_z_scale_enabled.reset();
    c.provider_id = 0xffff;
    check(reference_affine_transform(input, c)["matrix"] == expected,
          "provider selection tests only high 16 bits");
    c.provider_id = 0x10000;
    check(reference_affine_transform(input, c)["reason"] ==
              "reference_scale_provider_result_unknown",
          "enabled provider cannot be presumed to return no adjustment");
    c.provider_scale_available = false;
    check(reference_affine_transform(input, c)["matrix"] == expected,
          "provider failure leaves Y scale unchanged");
    c.provider_scale_available = true;
    check(reference_affine_transform(input, c)["reason"] ==
              "reference_scale_provider_factor_unknown",
          "successful provider requires its actual output factor");
    for (double f : {0., -3., 0.5}) {
        c.provider_y_scale = f;
        a = reference_affine_transform(input, c);
        check(a["matrix"][0] == std::array<double, 4>{0, -2 * f, 0, 10 + 4 * f} &&
                  a["provider_scale_applied"] == true,
              "provider factor affects the Y linear column and corresponding reference point "
              "translation");
    }
    c.provider_y_scale = std::numeric_limits<double>::infinity();
    check(reference_affine_transform(input, c)["status"] == "not_evaluated",
          "nonfinite provider output is unusable");
    c.provider_id = 0;
    c.origin.model_attached.reset();
    check(reference_affine_transform(input, c)["origin_correction"]["reason"] ==
              "reference_model_attachment_unknown",
          "final affine query does not silently discard unresolved origin correction");
    c.origin.model_coordinates =
        Json{{"status", "decoded"}, {"reference_origin", {{"value", Point3{2, 3, 4}}}}};
    a = reference_affine_transform(input, c);
    check(a["matrix"][0][3] == 20 && a["matrix"][1][3] == 14 && a["matrix"][2][3] == 16,
          "model correction is subtracted before right reference point translation");
    c.origin = {};
    c.origin.model_attached = false;
    const auto maximum = std::numeric_limits<double>::max();
    for (unsigned axis = 0; axis < 3; ++axis) {
        Point3 q{1, 2, 3};
        q[axis] = -maximum;
        a = reference_affine_transform(query({10, 20, 30}, q), c);
        check(a["status"] == "computed" && a["native_translation_sentinel"] == true &&
                  a["matrix"][0][3] == -q[0] && a["matrix"][1][3] == -q[1] &&
                  a["matrix"][2][3] == -q[2],
              "one negative maximum source component preserves the entire negated reference point "
              "sentinel");
        check(compose_reference_chain_transforms({a})["contains_native_translation_sentinel"] ==
                  true,
              "chain keeps the source sentinel diagnostic when copying a single result");
    }
    a = reference_affine_transform(query({0, 0, 0}, {maximum, 0, 0}), c);
    check(a["status"] == "not_evaluated" && !a.contains("matrix"),
          "positive maximum source is not the sentinel and can overflow");
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    auto n = query({nan, 0, 0}, {0, 0, 0});
    check(n["affine_inputs"]["translation_point"]["value"][0]["floating_point"] == "nan" &&
              Json::parse(n["affine_inputs"].dump()) == n["affine_inputs"] &&
              reference_affine_transform(n, c)["status"] == "not_evaluated",
          "point source nonfinite bits survive JSON without a fabricated affine result");
    a = reference_affine_transform(input, c);
    auto b = reference_affine_transform(query({7, 11, 13}, {0, 0, 0}, 3), c);
    const Matrix4 ab{{{-6, 0, 0, -8}, {0, -6, 0, 32}, {0, 0, 6, 50}, {0, 0, 0, 1}}};
    auto chain = compose_reference_chain_transforms({a, b});
    check(chain["matrix"] == ab && chain["processed_count"] == 2,
          "native chain composes current times host, including noncommuting translations");
    check(compose_reference_chain_transforms({b, a})["matrix"] != ab,
          "reversing selected chain order changes the transform");
    check(compose_reference_chain_transforms({a, b, a})["matrix"] ==
              Matrix4{{{0, 12, 0, -92}, {-12, 0, 0, -76}, {0, 0, 12, 194}, {0, 0, 0, 1}}},
          "third reference continues multiplying the accumulated matrix on the right");
    check(compose_reference_chain_transforms({})["matrix"] ==
              Matrix4{{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}},
          "empty selected chain produces identity");
    check(compose_reference_chain_transforms({a})["matrix"] == a["matrix"],
          "one reference chain directly copies its matrix");
    auto broken = b;
    broken["status"] = "not_evaluated";
    chain = compose_reference_chain_transforms({a, broken});
    check(chain["status"] == "not_evaluated" && chain["processed_count"] == 1 &&
              !chain.contains("matrix"),
          "unresolved chain member cannot expose a partial matrix as complete");
    broken = b;
    broken["matrix"][3][0] = 1;
    check(compose_reference_chain_transforms({broken})["status"] == "not_evaluated",
          "projective matrix is not an affine reference");
    broken = b;
    broken["matrix"][0][0] = maximum;
    check(compose_reference_chain_transforms({broken, b})["status"] == "not_evaluated",
          "chain product overflow has no usable matrix");
    // The native multiplication adds the left translation after the first
    // product, before the remaining terms. Algebraic regrouping loses +1.
    auto left = a, right = b;
    left["matrix"] = Matrix4{{{1, 1, 0, -1e16}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
    right["matrix"] = Matrix4{{{1, 0, 0, 1e16}, {0, 1, 0, 1}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
    check(compose_reference_chain_transforms({left, right})["matrix"][0][3] == 1,
          "chain translation preserves native rounding order");
    return checks;
}
