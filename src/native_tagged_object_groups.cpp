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

using ObjectCandidates = std::map<std::uint64_t, std::map<std::string, Json>>;

Json candidate_lookup(const ObjectCandidates &by_id, const Json &id) {
    Json candidates = Json::array();
    const auto parsed = nonnegative_u64(id);
    if (parsed) {
        const auto found = by_id.find(*parsed);
        if (found != by_id.end())
            for (const auto &[object_key, candidate] : found->second) {
                (void)object_key;
                candidates.push_back(candidate);
            }
    }
    const auto status = !parsed ? "invalid_object_id"
                        : candidates.empty() ? "no_document_object_id_candidate"
                        : candidates.size() == 1 ? "single_document_object_id_candidate"
                                                 : "ambiguous_document_object_id";
    return { {"source_object_id", id},
             {"status", status},
             {"candidate_count", candidates.size()},
             {"candidates", std::move(candidates)},
             {"match_scope", "document_wide_numeric_object_id_equality"} };
}
} // namespace

Json build_native_tagged_object_groups(const Json &graphics_records, const Json &objects) {
    ObjectCandidates by_id;
    if (objects.is_array())
        for (const auto &object : objects) {
            if (!object.is_object())
                continue;
            const auto id = nonnegative_u64(object.value("object_id", Json()));
            const auto class_id = nonnegative_u64(object.value("class_id", Json()));
            if (!id || !class_id)
                continue;
            const auto object_key = std::to_string(*class_id) + ":" + std::to_string(*id);
            auto &candidate = by_id[*id][object_key];
            if (candidate.is_null()) {
                candidate = {{"object_key", object_key},
                             {"class_id", *class_id},
                             {"object_id", *id},
                             {"class_name", object.value("root", Json::object())
                                                 .value("name", Json())},
                             {"records", Json::array()}};
            }
            candidate["records"].push_back(
                {{"stream", object.value("stream", Json())},
                 {"offset", object.value("offset", Json())},
                 {"length", object.value("length", Json())}});
        }

    Json records = Json::array();
    if (graphics_records.is_array())
        for (std::size_t record_index = 0; record_index < graphics_records.size(); ++record_index) {
            const auto &graphic = graphics_records[record_index];
            if (!graphic.is_object() || !graphic.value("attributes", Json()).is_array())
                continue;
            for (std::size_t attribute_index = 0;
                 attribute_index < graphic["attributes"].size(); ++attribute_index) {
                const auto &attribute = graphic["attributes"][attribute_index];
                if (!attribute.is_object() || attribute.value("group", -1) != 0 ||
                    attribute.value("key", -1) != 22909)
                    continue;
                const auto decoded = attribute.value("decoded", Json::object());
                Json row = {{"source_graphics_record_index", record_index},
                            {"source_graphics_record_id", graphic.value("id", Json())},
                            {"source_graphics_record_offset", graphic.value("offset", Json())},
                            {"source_graphics_record_stream", graphic.value("stream", Json())},
                            {"source_attribute_index", attribute_index},
                            {"source_attribute", attribute},
                            {"owner_object_id_candidate",
                             candidate_lookup(by_id, graphic.value("id", Json()))},
                            {"status", "unsupported_or_unrecognized_payload"}};
                if (decoded.value("encoding", std::string()) !=
                    "key_22909_tagged_object_groups") {
                    row["status"] = decoded.value("encoding", std::string()).empty()
                                        ? "decoded_payload_unavailable"
                                        : "no_tagged_group_projection_for_payload_encoding";
                    row["decoded_payload"] = decoded;
                    records.push_back(std::move(row));
                    continue;
                }
                if (!decoded.value("groups", Json()).is_array()) {
                    row["status"] = "invalid_decoded_groups";
                    row["decoded_payload"] = decoded;
                    records.push_back(std::move(row));
                    continue;
                }
                Json groups = Json::array();
                for (const auto &group : decoded["groups"]) {
                    Json members = Json::array();
                    const auto entries = group.value("entries", Json::array());
                    if (entries.is_array())
                        for (const auto &entry : entries)
                            members.push_back(
                                {{"source_entry", entry},
                                 {"object_id_candidate",
                                  candidate_lookup(by_id, entry.value("object_id", Json()))}});
                    groups.push_back({{"source_group", group}, {"members", std::move(members)}});
                }
                row["status"] = "candidate_references_decoded";
                row["tag_semantics"] = "unresolved";
                row["entry_type_semantics"] = "unresolved";
                row["reference_scope"] = "candidate_match_only";
                row["groups"] = std::move(groups);
                records.push_back(std::move(row));
            }
        }
    return {{"status", records.empty() ? "no_group_0_key_22909_records" : "decoded"},
            {"records", std::move(records)},
            {"matching_policy", "document_wide_object_id_equality_only"},
            {"semantic_policy", "group_tag_and_entry_type_meanings_unresolved"}};
}
} // namespace p3d
