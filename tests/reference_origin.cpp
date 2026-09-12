#include "internal.hpp"

unsigned reference_origin_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    auto put = [](Bytes &b, std::size_t offset, std::uint64_t value, unsigned size) {
        for (unsigned i = 0; i < size; ++i)
            b.at(offset + i) = static_cast<std::uint8_t>(value >> (8 * i));
    };
    auto number = [&](Bytes &b, std::size_t offset, double value) {
        std::uint64_t bits;
        std::memcpy(&bits, &value, 8);
        put(b, offset, bits, 8);
    };
    auto write_point = [&](Bytes &b, std::size_t offset, Point3 p) {
        for (auto v : p) {
            number(b, offset, v);
            offset += 8;
        }
    };
    auto model_wire = [&](unsigned kind, unsigned flags, Point3 primary, Point3 auxiliary) {
        Bytes b(500, 0);
        put(b, 4, 47, 2);
        put(b, 8, 248, 4);
        put(b, 12, 248, 4);
        put(b, 16, 32, 4);
        put(b, 38, kind, 2);
        put(b, 72, flags, 4);
        write_point(b, 404, primary);
        write_point(b, 116, auxiliary);
        return b;
    };
    for (unsigned kind : {0u, 1u, 2u, 65535u})
        for (unsigned mask : {0u, 1u, 0x400u, 0x401u}) {
            const auto b = model_wire(kind, 0x80000002u | mask, {2, 3, 4}, {-4, 5, 6});
            const auto c = parse_native(b)[0]["model_coordinate_state"];
            const auto effective_kind = kind == 0 && (mask & 0x400) ? 2u : kind;
            const auto expected_flags =
                (0x80000002u | (mask & 1)) | (effective_kind == 2 ? 0x400u : 0u);
            check(c["status"] == "decoded" && c["model_kind"]["source_value"] == kind &&
                      c["model_kind"]["value"] == effective_kind && c["flags"] == expected_flags &&
                      c["reference_origin"]["source_value"] == Point3{2, 3, 4} &&
                      c["reference_origin"]["value"] == Point3{2, 3, (mask & 1) ? 4. : 0.} &&
                      c["auxiliary_origin"]["value"] == Point3{-4, 5, 6},
                  "model coordinate loading preserves kind, normalized flag and Z selection rules");
        }
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    auto coordinates = decode_model_coordinates(model_wire(0, 0, {2, 3, nan}, {-4, 5, 6}));
    check(coordinates["reference_origin"]["source_value"][2]["floating_point"] == "nan" &&
              coordinates["reference_origin"]["value"] == Point3{2, 3, 0} &&
              Json::parse(coordinates.dump()) == coordinates,
          "disabled model origin Z preserves raw nonfinite source but supplies finite zero");
    auto short_model = model_wire(0, 0, {}, {});
    short_model.resize(499);
    check(decode_model_coordinates(short_model)["status"] == "unsupported_header",
          "short model coordinate layouts are not defaulted");
    auto reference = [&](unsigned primary_flags, unsigned secondary_flags, double scale = 3.) {
        Bytes b(372, 0);
        put(b, 4, 13, 2);
        put(b, 8, 184, 4);
        put(b, 12, 184, 4);
        put(b, 60, primary_flags, 4);
        put(b, 64, secondary_flags, 4);
        write_point(b, 220, {0, -1, 0});
        write_point(b, 244, {1, 0, 0});
        write_point(b, 268, {0, 0, 1});
        number(b, 292, scale);
        return native_reference_input(b);
    };
    auto input = reference(0x8000, 0);
    ReferenceOriginContext context;
    check(reference_origin_correction(input, context)["status"] == "not_evaluated",
          "unknown attachment is not treated as a disconnected reference");
    context.model_attached = false;
    auto result = reference_origin_correction(input, context);
    check(result["offset"] == Point3{0, 0, 0} && result["native_return_code"] == 1,
          "known unattached references need no model coordinates or correction");
    context.model_attached = true;
    check(reference_origin_correction(input, context)["reason"] ==
              "selected_model_coordinates_required",
          "attached model coordinates cannot be replaced by zero defaults");
    context.model_coordinates = coordinates;
    context.model_attached.reset();
    result = reference_origin_correction(input, context);
    check(result["offset"] == Point3{-9, 6, 0} && result["native_return_code"] == 0 &&
              result["origin_selected"] == "reference_origin",
          "an explicitly selected model determines the primary correction with its Z branch");
    context.model_coordinates = decode_model_coordinates(model_wire(0, 1, {2, 3, 4}, {-4, 5, 6}));
    context.auxiliary_origin_suppressed = true;
    context.auxiliary_origin_gate = 65535;
    result = reference_origin_correction(input, context);
    check(result["offset"] == Point3{-9, 6, 12},
          "primary nonzero origin takes precedence over all auxiliary suppression conditions");
    context = {};
    context.model_coordinates = decode_model_coordinates(model_wire(0, 1, {0, 0, 0}, {-4, 5, 6}));
    result = reference_origin_correction(input, context);
    check(result["reason"] == "auxiliary_origin_conditions_unknown" && !result.contains("offset"),
          "a nonzero auxiliary origin cannot bypass unresolved native gates");
    context.auxiliary_origin_gate = 0;
    context.auxiliary_origin_suppressed = false;
    result = reference_origin_correction(input, context);
    check(result["offset"] == Point3{-15, -12, 18} &&
              result["origin_selected"] == "auxiliary_origin",
          "enabled auxiliary origin is scaled before reference rotation");
    for (unsigned which : {0u, 1u, 2u}) {
        auto c = context;
        auto r = input;
        if (which == 0)
            r = reference(0, 0);
        if (which == 1)
            c.auxiliary_origin_gate = 1;
        if (which == 2)
            c.auxiliary_origin_suppressed = true;
        check(reference_origin_correction(r, c)["native_return_code"] == 1,
              "each auxiliary gate independently prevents correction");
    }
    context.model_coordinates = decode_model_coordinates(model_wire(0, 1, {2, 3, 4}, {-4, 5, 6}));
    result = reference_origin_correction(reference(0x8000, 0x400000), context);
    check(result["offset"] == Point3{-15, -12, 18},
          "secondary flag disables primary origin and allows the auxiliary path");
    context.model_coordinates.reset();
    context.model_attached = true;
    result = reference_origin_correction(reference(0, 0x400000), context);
    check(result["offset"] == Point3{0, 0, 0},
          "both disabled paths prove zero without dereferencing model coordinates");
    context = {};
    context.model_coordinates = decode_model_coordinates(model_wire(0, 1, {0, 0, 0}, {0, 0, 0}));
    result = reference_origin_correction(input, context);
    check(result["offset"] == Point3{0, 0, 0} &&
              result["branch"] == "zero_auxiliary_origin_independent_of_gate",
          "source zero auxiliary coordinates prove either unknown gate outcome has no correction");
    context.model_coordinates = decode_model_coordinates(model_wire(0, 1, {nan, 0, 0}, {0, 0, 0}));
    result = reference_origin_correction(input, context);
    check(result["status"] == "not_evaluated" && !result.contains("offset"),
          "nonfinite primary origin cannot fall back to an unrelated zero origin");
    context.model_coordinates = decode_model_coordinates(model_wire(0, 1, {1e-100, 0, 0}, {}));
    result = reference_origin_correction(reference(0, 0, 1e-300), context);
    check(result["offset"] == Point3{0, 0, 0} && result["native_return_code"] == 0,
          "nonzero source origin retains native success even when arithmetic underflows to zero");
    context.model_coordinates = decode_model_coordinates(model_wire(0, 1, {1e308, 0, 0}, {}));
    result = reference_origin_correction(input, context);
    check(result["status"] == "not_evaluated" && !result.contains("offset"),
          "origin scale overflow cannot produce an apparently usable correction");
    return checks;
}
