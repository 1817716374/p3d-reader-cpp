#include "internal.hpp"

namespace p3d {
namespace {
double finite_number(const Json &v) {
    require(v.is_number(), "reference affine input is not a number");
    const auto n = v.get<double>();
    require(std::isfinite(n), "nonfinite reference affine input");
    return n;
}
Point3 finite_point(const Json &v) {
    require(v.is_array() && v.size() == 3, "reference affine input requires three coordinates");
    return {finite_number(v[0]), finite_number(v[1]), finite_number(v[2])};
}
Matrix4 identity() {
    return {{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
}
Matrix4 affine_matrix(const Json &rows) {
    require(rows.is_array() && rows.size() == 4, "reference requires a 4 by 4 matrix");
    Matrix4 matrix{};
    for (unsigned i = 0; i < 4; ++i) {
        require(rows[i].is_array() && rows[i].size() == 4, "invalid reference matrix row");
        for (unsigned j = 0; j < 4; ++j)
            matrix[i][j] = finite_number(rows[i][j]);
    }
    require(matrix[3] == std::array<double, 4>{0, 0, 0, 1}, "reference requires affine matrix");
    return matrix;
}
Matrix4 affine_product(const Matrix4 &a, const Matrix4 &b) {
    auto product = identity();
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j < 3; ++j) {
            product[i][j] = (a[i][0] * b[0][j] + a[i][1] * b[1][j]) + a[i][2] * b[2][j];
            require(std::isfinite(product[i][j]), "reference linear product overflow");
        }
        product[i][3] = ((a[i][0] * b[0][3] + a[i][3]) + a[i][1] * b[1][3]) + a[i][2] * b[2][3];
        require(std::isfinite(product[i][3]), "reference translation product overflow");
    }
    return product;
}
} // namespace

Json reference_affine_transform(const Json &input, const ReferenceAffineContext &context) {
    Json out = {{"profile", "bimbase_2025_reference_affine_query"},
                {"scope", "selected_reference_context"},
                {"status", "not_evaluated"}};
    try {
        require(input.value("status", "") == "decoded", "decoded_reference_input_required");
        const auto &transform = input.at("transform");
        require(transform.value("status", "") == "computed",
                "computed_reference_base_transform_required");
        const auto scale = finite_number(transform.at("scale"));
        const auto &rows = transform.at("matrix");
        require(rows.is_array() && rows.size() == 3, "invalid reference base matrix");
        Matrix4 matrix = identity();
        for (unsigned i = 0; i < 3; ++i) {
            const auto row = finite_point(rows[i]);
            std::copy(row.begin(), row.end(), matrix[i].begin());
        }
        require(context.force_z_scale || context.model_z_scale_enabled.has_value(),
                "reference_model_z_scale_state_unknown");
        Point3 scales{scale, scale,
                      context.force_z_scale || *context.model_z_scale_enabled ? scale : 1.};
        require(context.provider_id.has_value(), "reference_scale_provider_identity_unknown");
        const bool provider_enabled = (*context.provider_id >> 16) != 0;
        bool provider_applied = false;
        if (provider_enabled) {
            require(context.provider_scale_available.has_value(),
                    "reference_scale_provider_result_unknown");
            if (*context.provider_scale_available) {
                require(context.provider_y_scale.has_value(),
                        "reference_scale_provider_factor_unknown");
                require(std::isfinite(*context.provider_y_scale),
                        "nonfinite reference provider scale");
                scales[1] *= *context.provider_y_scale;
                require(std::isfinite(scales[1]), "reference Y scale overflow");
                provider_applied = true;
            }
        }
        out["origin_correction"] = reference_origin_correction(input, context.origin);
        const auto &correction = out.at("origin_correction");
        require(correction.at("status") == "computed", "reference_origin_correction_unresolved");
        auto translation =
            finite_point(input.at("affine_inputs").at("translation_point").at("value"));
        auto reference = finite_point(input.at("affine_inputs").at("reference_point").at("value"));
        if (correction.at("native_return_code") == 0) {
            const auto offset = finite_point(correction.at("offset"));
            for (unsigned i = 0; i < 3; ++i) {
                translation[i] -= offset[i];
                require(std::isfinite(translation[i]), "reference translation correction overflow");
            }
        }
        for (unsigned i = 0; i < 3; ++i) {
            for (unsigned j = 0; j < 3; ++j) {
                matrix[i][j] *= scales[j];
                require(std::isfinite(matrix[i][j]), "reference affine axis scaling overflow");
            }
            reference[i] = -reference[i];
        }
        const bool sentinel = std::any_of(reference.begin(), reference.end(), [](double v) {
            return v == std::numeric_limits<double>::max();
        });
        for (unsigned i = 0; i < 3; ++i) {
            // Native point transformation returns the entire input unchanged
            // if any coordinate is DBL_MAX. It is not an ordinary large point.
            matrix[i][3] = sentinel ? reference[i]
                                    : ((reference[1] * matrix[i][1] + reference[0] * matrix[i][0]) +
                                       reference[2] * matrix[i][2]) +
                                          translation[i];
            require(std::isfinite(matrix[i][3]), "reference affine point translation overflow");
        }
        out.update({{"status", "computed"},
                    {"matrix", matrix},
                    {"axis_scales", scales},
                    {"provider_id", *context.provider_id},
                    {"provider_enabled", provider_enabled},
                    {"provider_scale_applied", provider_applied},
                    {"native_translation_sentinel", sentinel},
                    {"force_z_scale", context.force_z_scale}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

namespace {
Json compose_chain(const std::vector<Json> &transforms, bool owner_path) {
    Json out = {{"profile", owner_path ? "bimbase_2025_owner_reference_chain_composition"
                                       : "bimbase_2025_reference_chain_composition"},
                {"scope", "explicit_selected_chain"},
                {"status", "not_evaluated"},
                {"order", owner_path ? "current_to_host_next_times_accumulated"
                                     : "current_to_host_accumulated_times_next"}};
    auto accumulated = identity();
    std::size_t completed = 0;
    bool contains_sentinel = false;
    try {
        for (const auto &entry : transforms) {
            require(entry.value("profile", "") == "bimbase_2025_reference_affine_query" &&
                        entry.value("status", "") == "computed",
                    "computed_reference_affine_result_required");
            require(entry.value("force_z_scale", false),
                    "reference_chain_requires_forced_z_scale_query");
            contains_sentinel =
                contains_sentinel || entry.value("native_translation_sentinel", false);
            const auto next = affine_matrix(entry.at("matrix"));
            if (completed == 0) {
                accumulated = next; // Native copies the first matrix without multiplication.
            } else {
                accumulated = owner_path ? affine_product(next, accumulated)
                                         : affine_product(accumulated, next);
            }
            ++completed;
        }
        out.update({{"status", "computed"}, {"matrix", accumulated}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    out["processed_count"] = completed;
    out["input_count"] = transforms.size();
    out["contains_native_translation_sentinel"] = contains_sentinel;
    return out;
}
} // namespace

Json compose_reference_chain_transforms(const std::vector<Json> &transforms) {
    return compose_chain(transforms, false);
}

Json compose_owner_reference_chain_transforms(const std::vector<Json> &transforms) {
    return compose_chain(transforms, true);
}

Json owner_reference_path_transform(const Json &collected_records,
                                    const std::vector<Json> &owner_reference_chain,
                                    const OwnerReferencePathTransformContext &context) {
    Json out = {{"profile", "bimbase_2025_owner_reference_path_transform"},
                {"scope", "explicit_collected_objects_and_owner_chain"},
                {"status", "not_evaluated"},
                {"applied_blocks", Json::array()},
                {"path_resolution", "not_evaluated"},
                {"geometry_transformation", "not_evaluated"}};
    try {
        require(context.owner_kind.has_value(), "owner_kind_required");
        if (*context.owner_kind == 8) {
            out.update({{"status", "native_failure"}, {"native_status", 0x11006}});
            return out;
        }
        require(context.terminal_index.has_value(), "collector_terminal_index_required");
        // The native loop uses signed 32-bit index arithmetic. Do not interpret
        // high-bit values as a huge portable array index or guess wrapped state.
        require(*context.terminal_index <= INT32_MAX, "invalid_collector_terminal_index");
        require(collected_records.is_array(), "collected_records_array_required");
        out["terminal_index"] = *context.terminal_index;
        auto accumulated = identity();
        if (*context.terminal_index > 0) {
            out["local_mode"] = "preceding_block_instances";
            out["local_stop"] = "beginning";
            for (std::size_t i = *context.terminal_index; i > 0;) {
                --i;
                if (i >= collected_records.size() || collected_records[i].is_null()) {
                    out["local_stop"] =
                        i >= collected_records.size() ? "outside_collection" : "null_object";
                    out["local_stop_index"] = i;
                    break;
                }
                const auto &record = collected_records[i];
                const auto &type = record.at("element_type");
                require(type.is_number_integer() && type >= 0 && type <= UINT16_MAX,
                        "native_collected_record_type_required");
                if (type != 62)
                    continue;
                const auto &block = record.at("block_transform");
                require(block.value("status", "") == "resolved" &&
                            block.value("reader_profile", "") ==
                                "bimbase_2025_block_transform_input",
                        "accepted_block_transform_required");
                // Read the effective input matrix, including native repair,
                // not the saved source coefficients.
                accumulated = affine_product(affine_matrix(block.at("matrix")), accumulated);
                out["applied_blocks"].push_back(i);
            }
        } else {
            out["local_mode"] = "single_object_handler";
            const auto &handler = context.single_object_transform;
            require(handler.is_object(), "single_object_handler_result_required");
            const auto status = handler.value("status", "");
            if (status == "native_failure") {
                out.update({{"status", "native_failure"}, {"native_status", 1}});
                return out;
            }
            if (status == "computed")
                accumulated = affine_product(affine_matrix(handler.at("matrix")), accumulated);
            else
                require(status == "absent", "single_object_handler_result_unresolved");
            out["single_object_transform_status"] = status;
        }
        const auto chain = compose_owner_reference_chain_transforms(owner_reference_chain);
        out["owner_chain"] = chain;
        require(chain.at("status") == "computed", "owner_reference_chain_unresolved");
        accumulated = affine_product(affine_matrix(chain.at("matrix")), accumulated);
        out.update({{"status", "computed"}, {"matrix", accumulated}});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
