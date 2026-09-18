#include "internal.hpp"

namespace p3d {
Json native_file_header(const Bytes &index_stream, const Bytes &payload) {
    Json out = {{"reader_profile", "bimbase_2025_initial_file_header"},
                {"status", "unresolved"},
                {"stream", Json::array({"P3DSIndexes"})},
                {"header_payload", rawbytes(payload)},
                {"header_compressed_data_offset", 40},
                {"id_counter_header_offset", 0x128},
                {"flags_header_offset", 0x310}};
    if (index_stream.size() < 32) {
        out["reason"] = "truncated_index_control_header";
        out["native_error_code"] = 0x14007;
        return out;
    }
    Bytes control(20);
    for (std::size_t i = 0; i < control.size(); ++i)
        control[i] = index_stream[12 + i] ^ (i ? index_stream[11 + i] : 0x26);
    const auto flags = Reader(control).u16();
    out["control_header_offset"] = 12;
    out["decoded_control_header"] = rawbytes(control);
    out["control_flags"] = flags;
    out["unassigned_control_bytes"] = rawbytes(slice(control, 2, 18));
    if (payload.size() >= 0x130)
        out["source_id_counter"] = Reader(payload, 0x128).u64();
    if (payload.size() >= 0x314)
        out["source_header_flags"] = Reader(payload, 0x310).u32();
    if (flags & 2) {
        // Initial header probing has a null auxiliary argument: the native
        // reader initializes a blank header instead of consuming its payload.
        out["initial_probe"] = {
            {"action", "initialize_blank_header"}, {"id_counter", 0}, {"header_flags", 2}};
        out["status"] = "resolved";
    } else if (payload.size() < 0x610) {
        out["reason"] = "incomplete_file_header_payload";
    } else {
        out["initial_probe"] = {{"action", "read_header_payload"},
                                {"id_counter", out["source_id_counter"]},
                                {"header_flags", Reader(payload, 0x310).u32() | flags}};
        out["status"] = "resolved";
        if (payload.size() > 0x610)
            out["header_payload_suffix"] = rawbytes(slice(payload, 0x610, payload.size() - 0x610));
    }
    return out;
}

static Json initial_id_assignments(const Json &list, const Json &records, std::uint64_t counter) {
    Json out = {{"status", "unresolved"}, {"roots", Json::array()}, {"registry", Json::array()}};
    out["initial_id_counter"] = counter;
    struct Source {
        std::size_t record, occurrence;
        std::uint64_t block;
    };
    std::map<std::uint64_t, Source> registry;
    std::size_t occurrence = 0;
    try {
        for (const auto &root : list.at("roots")) {
            Json result = {{"native_record_index", root.at("native_record_index")},
                           {"block_number", root.at("block_number")},
                           {"counter_before", counter},
                           {"records", Json::array()}};
            // The entire subtree is assigned IDs before any of its nodes is
            // registered. A later descendant may raise the counter used by a
            // duplicate at the root. Do not combine these into one pass.
            for (const auto &header : root.at("headers")) {
                require(header.at("status") == "resolved", "unresolved list header preparation");
                const auto ni = header.at("native_record_index").get<std::size_t>();
                const auto source = records.at(ni).at("id").get<std::uint64_t>();
                auto prepared = source;
                if (prepared == 0)
                    prepared = ++counter;
                else
                    counter = std::max(counter, prepared);
                result["records"].push_back(
                    {{"native_record_index", ni},
                     {"input_occurrence_index", occurrence++},
                     {"parent_record_index", header.at("parent_record_index")},
                     {"source_id", source},
                     {"prepared_id", prepared}});
            }
            result["counter_after_subtree_preparation"] = counter;
            for (auto &item : result["records"]) {
                auto id = item["prepared_id"].get<std::uint64_t>();
                Json collisions = Json::array();
                while (id != 0) {
                    counter = std::max(counter, id);
                    const auto existing = registry.find(id);
                    if (existing == registry.end())
                        break;
                    collisions.push_back(
                        {{"id", id},
                         {"existing_native_record_index", existing->second.record},
                         {"existing_input_occurrence_index", existing->second.occurrence}});
                    id = ++counter; // uint64 wrap is part of the native behavior.
                }
                item["assigned_id"] = id;
                item["collisions"] = std::move(collisions);
                item["registration_status"] = id ? "registered" : "unindexed_zero_id";
                item["native_registration_return_code"] = id ? 0 : 1;
                if (id)
                    registry.emplace(id, Source{item["native_record_index"].get<std::size_t>(),
                                                item["input_occurrence_index"].get<std::size_t>(),
                                                root["block_number"].get<std::uint64_t>()});
            }
            result["counter_after_registration"] = counter;
            out["roots"].push_back(std::move(result));
        }
        out["status"] = list.value("status", Json()) == "resolved" ? "resolved" : "partial";
    } catch (const std::exception &e) {
        out["status"] = "partial";
        out["error"] = e.what();
    }
    out["final_id_counter"] = counter;
    for (const auto &[id, source] : registry)
        out["registry"].push_back({{"id", id},
                                   {"native_record_index", source.record},
                                   {"input_occurrence_index", source.occurrence},
                                   {"block_number", source.block}});
    return out;
}

Json native_system_id_assignments(const Json &list, const Json &records, const Json &file_header) {
    Json out = {{"scope", "first_system_input_with_empty_id_registry"},
                {"status", "unresolved"},
                {"roots", Json::array()},
                {"registry", Json::array()},
                {"runtime_callbacks", "not_evaluated"},
                {"model_input_order", "not_evaluated"}};
    if (file_header.value("status", Json()) != "resolved" ||
        !file_header.contains("initial_probe") ||
        file_header["initial_probe"].value("action", Json()) != "read_header_payload") {
        out["reason"] = "ordinary_initial_file_header_required";
        return out;
    }
    if (list.value("system_bootstrap_required", Json()) != true) {
        out["reason"] = "system_container_required";
        return out;
    }
    out.update(initial_id_assignments(
        list, records, file_header.at("initial_probe").at("id_counter").get<std::uint64_t>()));
    return out;
}

Json native_model_id_assignments(const Json &containers, const Json &records, const Json &index,
                                 const StreamPath &model, std::uint64_t initial_counter) {
    Json out = {{"scope", "fresh_model_control_then_graphics_id_registration"},
                {"status", "unresolved"},
                {"model_storage", model},
                {"initial_id_counter", initial_counter},
                {"inputs", Json::array()},
                {"runtime_callbacks", "not_evaluated"},
                {"attribute_attachment", "not_evaluated"}};
    try {
        require(!model.empty(), "explicit_model_storage_required");
        Json combined = {{"status", "resolved"}, {"roots", Json::array()}};
        std::vector<Json> root_sources;
        for (const auto kind : {"P3D-SMC", "P3D-SMG"}) {
            require(index.contains(kind), "model_container_alias_unavailable");
            auto path = model;
            path.push_back(index.at(kind).get<std::string>());
            const Json *found = nullptr;
            for (const auto &container : containers) {
                if (container.at("container") != path)
                    continue;
                require(found == nullptr, "ambiguous_model_input_container");
                found = &container;
            }
            const auto input_index = out["inputs"].size();
            Json source = {{"input_list_index", input_index}, {"kind", kind}, {"container", path}};
            if (!found) {
                source["status"] = "not_loaded";
                source["reason"] = "missing_record_container";
                out["inputs"].push_back(source);
                continue;
            }
            require(found->at("kind") != "P3D-SSYS", "model_alias_resolves_to_system_container");
            const auto &list = found->at("list_preparation");
            require(list.at("system_bootstrap_required") == false, "ordinary_model_list_required");
            for (const auto &root : list.at("roots")) {
                combined["roots"].push_back(root);
                root_sources.push_back(source);
            }
            source["status"] = list.at("status");
            source["root_count"] = list.at("roots").size();
            out["inputs"].push_back(source);
            if (list.at("status") != "resolved") {
                combined["status"] = "partial";
                out["stop_reason"] = "unresolved_preceding_model_list";
                break; // Its unknown IDs can affect every later assignment.
            }
        }
        out.update(initial_id_assignments(combined, records, initial_counter));
        std::map<std::size_t, Json> occurrence_sources;
        for (std::size_t i = 0; i < out["roots"].size(); ++i) {
            const auto &source = root_sources.at(i);
            out["roots"][i].update(source);
            for (auto &item : out["roots"][i]["records"]) {
                item.update(source);
                occurrence_sources.emplace(item.at("input_occurrence_index").get<std::size_t>(),
                                           source);
            }
        }
        for (auto &item : out["registry"])
            item.update(
                occurrence_sources.at(item.at("input_occurrence_index").get<std::size_t>()));
    } catch (const std::exception &e) {
        out["status"] = "unresolved";
        out["reason"] = e.what();
    }
    return out;
}

Json Document::native_model_id_assignments(const StreamPath &model_storage,
                                           std::uint64_t initial_id_counter) const {
    const auto containers = p3d::native_input_containers(streams(), index(), native_records());
    return p3d::native_model_id_assignments(containers, native_records(), index(), model_storage,
                                            initial_id_counter);
}
} // namespace p3d
