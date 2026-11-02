#include "internal.hpp"

namespace p3d {
namespace {
std::optional<std::uint64_t> nonnegative_u64(const Json &value) {
    if (value.is_number_unsigned())
        return value.get<std::uint64_t>();
    if (value.is_number_integer()) {
        const auto signed_value = value.get<std::int64_t>();
        if (signed_value >= 0)
            return static_cast<std::uint64_t>(signed_value);
    }
    return {};
}
} // namespace

Json build_native_graphic_record_candidates(const Json &graphics_records,
                                            const Json &native_records) {
    std::map<std::uint64_t, std::vector<std::size_t>> native_by_id;
    if (native_records.is_array())
        for (std::size_t i = 0; i < native_records.size(); ++i) {
            const auto &record = native_records[i];
            if (!record.is_object())
                continue;
            const auto id = nonnegative_u64(record.value("id", Json()));
            if (id)
                native_by_id[*id].push_back(i);
        }

    Json rows = Json::array();
    std::size_t unique_count = 0, ambiguous_count = 0, missing_count = 0, invalid_count = 0;
    if (graphics_records.is_array())
        for (std::size_t i = 0; i < graphics_records.size(); ++i) {
            const auto &graphic = graphics_records[i];
            if (!graphic.is_object()) {
                ++invalid_count;
                rows.push_back({{"graphics_record_index", i},
                                {"candidate_status", "invalid_graphics_record"},
                                {"native_record_candidates", Json::array()}});
                continue;
            }

            const auto id = nonnegative_u64(graphic.value("id", Json()));
            Json candidates = Json::array();
            if (id) {
                const auto found = native_by_id.find(*id);
                if (found != native_by_id.end())
                    for (const auto native_index : found->second) {
                        const auto &native = native_records[native_index];
                        candidates.push_back({{"native_record_index", native_index},
                                              {"native_record_id", native.value("id", Json())},
                                              {"element_type", native.value("element_type", Json())},
                                              {"element_flags", native.value("element_flags", Json())},
                                              {"record_subtype", native.value("record_subtype", Json())},
                                              {"base_length", native.value("base_length", Json())},
                                              {"offset", native.value("offset", Json())},
                                              {"stream", native.value("stream", Json())}});
                    }
            }

            const auto status = !id                     ? "invalid_graphics_record_id"
                                : candidates.empty()    ? "no_native_record_id_candidate"
                                : candidates.size() == 1 ? "single_native_record_id_candidate"
                                                         : "ambiguous_native_record_id";
            if (!id)
                ++invalid_count;
            else if (candidates.empty())
                ++missing_count;
            else if (candidates.size() == 1)
                ++unique_count;
            else
                ++ambiguous_count;

            rows.push_back({{"graphics_record_index", i},
                            {"graphics_record_id", graphic.value("id", Json())},
                            {"graphics_record_offset", graphic.value("offset", Json())},
                            {"graphics_record_flags", graphic.value("flags", Json())},
                            {"graphics_record_stream", graphic.value("stream", Json())},
                            {"candidate_status", status},
                            {"native_record_candidate_count", candidates.size()},
                            {"native_record_candidates", std::move(candidates)}});
        }

    const auto status = rows.empty() ? "no_graphics_records"
                        : ambiguous_count || missing_count || invalid_count ? "partial_candidates"
                                                                            : "all_unique_candidates";
    return {{"status", status},
            {"match_basis", "document_wide_numeric_record_id_equality"},
            {"semantic_policy", "candidate_only_no_record_merge_or_payload_role_claim"},
            {"graphics_record_count", rows.size()},
            {"single_candidate_count", unique_count},
            {"ambiguous_candidate_count", ambiguous_count},
            {"missing_candidate_count", missing_count},
            {"invalid_record_count", invalid_count},
            {"records", std::move(rows)}};
}
} // namespace p3d
