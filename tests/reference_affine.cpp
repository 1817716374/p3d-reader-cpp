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
    const Matrix4 ba{{{-6, 0, 0, -47}, {0, -6, 0, 53}, {0, 0, 6, 85}, {0, 0, 0, 1}}};
    check(compose_owner_reference_chain_transforms({a, b})["matrix"] == ba,
          "owner path reference chain premultiplies each host in traversal order");
    check(compose_owner_reference_chain_transforms({a, b, b})["matrix"] ==
              Matrix4{{{0, 18, 0, -152}, {-18, 0, 0, -130}, {0, 0, 18, 268}, {0, 0, 0, 1}}},
          "third owner reference continues premultiplication without deduplicating instances");
    const Matrix4 id{{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
    check(compose_owner_reference_chain_transforms({})["matrix"] == id &&
              compose_owner_reference_chain_transforms({a})["matrix"] == a["matrix"],
          "empty owner chain is identity and first reference is copied");
    check(compose_owner_reference_chain_transforms({right, left})["matrix"][0][3] == 1,
          "owner chain retains native translation rounding during premultiplication");
    broken = b;
    broken["force_z_scale"] = false;
    check(compose_owner_reference_chain_transforms({a, broken})["status"] == "not_evaluated",
          "owner chain uses forced-Z reference query");
    broken["status"] = "not_evaluated";
    check(!compose_owner_reference_chain_transforms({a, broken}).contains("matrix"),
          "owner chain failure does not expose usable partial result");
    auto block_record = [&](const Matrix4 &matrix) {
        return Json{{"element_type", 62},
                    {"block_transform",
                     {{"reader_profile", "bimbase_2025_block_transform_input"},
                      {"status", "resolved"},
                      {"matrix", matrix}}}};
    };
    auto translation = id, scale = id;
    translation[0][3] = 10;
    scale[0][0] = 2;
    const auto bt = block_record(translation), bs = block_record(scale);
    auto records = Json::array({bt, bs, {{"element_type", 62}}});
    OwnerReferencePathTransformContext pc;
    pc.owner_kind = 0;
    pc.terminal_index = 2;
    auto path = owner_reference_path_transform(records, {a, b}, pc);
    check(path["status"] == "computed" &&
              path["matrix"] ==
                  Matrix4{{{-12, 0, 0, -107}, {0, -6, 0, 53}, {0, 0, 6, 85}, {0, 0, 0, 1}}},
          "owner chain acts after reverse-collected block transformations");
    check(path["applied_blocks"] == Json::array({1, 0}) &&
              path["geometry_transformation"] == "not_evaluated",
          "terminal object is excluded even when its block matrix is unavailable");
    auto repaired = records;
    repaired[0]["block_transform"]["source_matrix"] = scale;
    check(owner_reference_path_transform(repaired, {a, b}, pc)["matrix"] == path["matrix"],
          "path uses effective accepted block matrix rather than saved source coefficients");
    records = Json::array({bt, {{"element_type", 13}}, bs, {{"element_type", 19}}});
    pc.terminal_index = 3;
    check(owner_reference_path_transform(records, {a, b}, pc)["matrix"] == path["matrix"],
          "non-block collected objects are skipped without changing block order");
    records[1] = nullptr;
    auto stopped = owner_reference_path_transform(records, {}, pc);
    check(stopped["status"] == "computed" && stopped["matrix"] == scale &&
              stopped["local_stop"] == "null_object" &&
              stopped["applied_blocks"] == Json::array({2}),
          "null collected object stops local traversal but preserves later processed block");
    pc.terminal_index = 9;
    stopped = owner_reference_path_transform(records, {a}, pc);
    check(stopped["status"] == "computed" && stopped["matrix"] == a["matrix"] &&
              stopped["local_stop"] == "outside_collection",
          "out-of-range local index stops local traversal but still applies owner chain");
    pc.terminal_index = 0;
    check(owner_reference_path_transform(Json::array({bt}), {}, pc)["reason"] ==
              "single_object_handler_result_required",
          "single block cannot stand in for unknown handler query result");
    pc.single_object_transform = {{"status", "absent"}};
    check(owner_reference_path_transform(Json::array({bt}), {}, pc)["matrix"] == id,
          "proven absent single-object handler contributes identity");
    pc.single_object_transform = {{"status", "computed"}, {"matrix", translation}};
    check(owner_reference_path_transform(Json::array({bt}), {a, b}, pc)["matrix"] ==
              Matrix4{{{-6, 0, 0, -107}, {0, -6, 0, 53}, {0, 0, 6, 85}, {0, 0, 0, 1}}},
          "single-object handler matrix precedes owner-chain transformation");
    pc.single_object_transform = {{"status", "native_failure"}};
    auto failure = owner_reference_path_transform(Json::array({bt}), {broken}, pc);
    check(failure["status"] == "native_failure" && failure["native_status"] == 1 &&
              !failure.contains("owner_chain") && !failure.contains("matrix"),
          "failed single-object handler stops before inspecting owner chain");
    pc.owner_kind = 8;
    pc.terminal_index.reset();
    failure = owner_reference_path_transform(Json(), {broken}, pc);
    check(failure["status"] == "native_failure" && failure["native_status"] == 0x11006,
          "native owner category eight rejects before collector and chain processing");
    pc.owner_kind.reset();
    check(owner_reference_path_transform(records, {}, pc)["reason"] == "owner_kind_required",
          "unknown owner category is not replaced by saved model kind or default");
    pc.owner_kind = 0;
    check(owner_reference_path_transform(records, {}, pc)["reason"] ==
              "collector_terminal_index_required",
          "collector terminal position must be explicitly known");
    pc.terminal_index = UINT32_MAX;
    check(owner_reference_path_transform(records, {}, pc)["status"] == "not_evaluated",
          "high-bit collector index is not treated as portable array size");
    pc.terminal_index = 1;
    failure =
        owner_reference_path_transform(Json::array({{{"element_type", "unknown"}}, {}}), {}, pc);
    check(failure["status"] == "not_evaluated" && !failure.contains("matrix"),
          "unknown collected record type is not treated as a known non-block");
    auto invalid_block = bt;
    invalid_block["block_transform"]["status"] = "invalid";
    failure = owner_reference_path_transform(Json::array({invalid_block, {}}), {}, pc);
    check(failure["status"] == "not_evaluated" && !failure.contains("matrix"),
          "unknown effective block matrix is not silently skipped");
    invalid_block = bt;
    invalid_block["block_transform"]["matrix"][0][3] = maximum;
    auto over = a;
    over["matrix"] = scale;
    failure = owner_reference_path_transform(Json::array({invalid_block, {}}), {over}, pc);
    check(failure["status"] == "not_evaluated" && !failure.contains("matrix"),
          "path composition overflow cannot publish an effective matrix");
    check(Json::parse(path.dump()) == path,
          "computed path transform and provenance survive JSON roundtrip");
    Bytes native_block(260, 0);
    put(native_block, 4, 62, 2);
    put(native_block, 8, 128, 4);
    put(native_block, 12, 128, 4);
    for (unsigned row = 0; row < 3; ++row) {
        for (unsigned column = 0; column < 3; ++column)
            number(native_block, 164 + 24 * row + 8 * column, translation[row][column]);
        number(native_block, 236 + 8 * row, translation[row][3]);
    }
    const auto parsed = parse_native(native_block);
    pc.terminal_index = 1;
    check(owner_reference_path_transform(Json::array({parsed.at(0), {}}), {}, pc)["matrix"] ==
              translation,
          "actual parsed native block record is consumed without a private field adapter");
    return checks;
}
