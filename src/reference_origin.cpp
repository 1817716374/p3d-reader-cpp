#include "internal.hpp"

namespace p3d {
Json reference_origin_correction(const Json &input, const ReferenceOriginContext &context) {
    Json out = {{"profile", "bimbase_2025_reference_origin_query"}, {"status", "not_evaluated"}};
    auto fail = [&](const char *reason) {
        out["reason"] = reason;
        return out;
    };
    auto none = [&](const char *branch) {
        out.update({{"status", "computed"},
                    {"branch", branch},
                    {"native_return_code", 1},
                    {"offset", Point3{0, 0, 0}},
                    {"origin_selected", "none"}});
        return out;
    };
    auto point = [](const Json &j) -> std::optional<Point3> {
        if (!j.is_array() || j.size() != 3)
            return std::nullopt;
        Point3 p{};
        for (unsigned i = 0; i < 3; ++i) {
            if (!j[i].is_number())
                return std::nullopt;
            p[i] = j[i].get<double>();
            if (!std::isfinite(p[i]))
                return std::nullopt;
        }
        return p;
    };
    auto zero = [](const Point3 &p) { return p[0] == 0 && p[1] == 0 && p[2] == 0; };
    try {
        if (input.value("status", "") != "decoded")
            return fail("decoded_reference_input_required");
        const auto attached = context.model_attached      ? context.model_attached
                              : context.model_coordinates ? std::optional<bool>(true)
                                                          : std::nullopt;
        if (!attached)
            return fail("reference_model_attachment_unknown");
        if (!*attached)
            return none("model_not_attached");
        const auto &flags = input.at("origin_inputs");
        const auto primary_flags = flags.at("primary_flags").get<std::uint32_t>();
        const auto secondary_flags = flags.at("secondary_flags").get<std::uint32_t>();
        const bool primary_disabled = (secondary_flags & 0x400000) != 0;
        const bool auxiliary_disabled =
            !(primary_flags & 0x8000) ||
            (context.auxiliary_origin_gate && *context.auxiliary_origin_gate != 0) ||
            (context.auxiliary_origin_suppressed && *context.auxiliary_origin_suppressed);
        if (primary_disabled && auxiliary_disabled)
            return none("both_origin_paths_disabled");
        if (!context.model_coordinates ||
            context.model_coordinates->value("status", "") != "decoded")
            return fail("selected_model_coordinates_required");
        const auto &model = *context.model_coordinates;
        Point3 origin{};
        const char *selected = "reference_origin";
        if (!primary_disabled) {
            const auto p = point(model.at("reference_origin").at("value"));
            if (!p)
                return fail("nonfinite_or_invalid_model_reference_origin");
            origin = *p;
        }
        if (zero(origin)) {
            if (auxiliary_disabled)
                return none("auxiliary_origin_disabled");
            const auto p = point(model.at("auxiliary_origin").at("value"));
            // If both possible outcomes are zero, no unknown runtime gate is
            // needed to prove the correction. Do not claim which gate ran.
            if (p && zero(*p))
                return none("zero_auxiliary_origin_independent_of_gate");
            if (!context.auxiliary_origin_gate || !context.auxiliary_origin_suppressed)
                return fail("auxiliary_origin_conditions_unknown");
            if (!p)
                return fail("nonfinite_or_invalid_model_auxiliary_origin");
            origin = *p;
            selected = "auxiliary_origin";
        }
        const auto &transform = input.at("transform");
        if (transform.value("status", "") != "computed")
            return fail("computed_reference_base_transform_required");
        const auto scale = transform.at("scale").get<double>();
        require(std::isfinite(scale), "nonfinite reference origin scale");
        const auto &matrix = transform.at("matrix");
        require(matrix.is_array() && matrix.size() == 3, "invalid reference origin matrix");
        Point3 scaled = origin, offset{};
        for (auto &v : scaled) {
            v *= scale;
            require(std::isfinite(v), "reference origin scale overflow");
        }
        for (unsigned i = 0; i < 3; ++i) {
            const auto row = point(matrix[i]);
            require(row.has_value(), "nonfinite or invalid reference origin matrix");
            offset[i] = (scaled[1] * (*row)[1] + scaled[0] * (*row)[0]) + scaled[2] * (*row)[2];
            require(std::isfinite(offset[i]), "reference origin matrix multiplication overflow");
        }
        out.update({{"status", "computed"},
                    {"branch", "scaled_and_rotated_origin"},
                    {"native_return_code", 0},
                    {"origin_selected", selected},
                    {"selected_origin", origin},
                    {"offset", offset}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
