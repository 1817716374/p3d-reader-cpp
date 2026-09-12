#include "internal.hpp"

namespace p3d {
Json native_block_header(const Bytes &raw) {
    Json out = {{"status", "invalid"}, {"source_offset", 0}, {"source_bytes", 16}};
    if (raw.size() < 16) {
        out["reason"] = "truncated_block_header";
        out["native_error_code"] = 0x14007;
        return out;
    }
    Bytes decoded(16);
    for (std::size_t i = 0; i < decoded.size(); ++i)
        decoded[i] = raw[i] ^ (i ? raw[i - 1] : 0x26);
    Reader r(decoded);
    const auto capacity = r.u32(), flags = r.u32();
    out.update({{"status", "resolved"},
                {"decoded_header", rawbytes(decoded)},
                {"record_capacity_hint", capacity},
                {"capacity_uses_list_default", capacity == 0 || capacity > 0x7fffffffu},
                {"control_flags", flags},
                {"stop_after_block", (flags & 1) != 0},
                {"record_input", capacity ? "read_until_record_reader_stops" : "skip"},
                {"block_return_code", (flags & 1) ? 0x12003 : 0},
                {"unassigned_control_bits", flags & ~1u},
                {"unassigned_tail", rawbytes(slice(decoded, 8, 8))}});
    return out;
}

static Json block_records(const Stream &stream, const Json &records,
                          const std::vector<std::size_t> &indices, const Json &header) {
    Json out = {{"scope", "block_record_input_if_reached"},
                {"status", "unresolved"},
                {"roots", Json::array()},
                {"skipped_record_indices", Json::array()}};
    if (header.at("record_input") == "skip") {
        out["status"] = "skipped";
        out["reason"] = "zero_capacity_hint";
        return out;
    }
    // These are the layouts for which Document exposes a record-only buffer.
    // Do not mistake an unknown wrapper or raw header for the first record.
    if (!stream.decoded || (stream.compression_offset != 24 && stream.raw->size() != 16)) {
        out["reason"] = "record_payload_layout_unconfirmed";
        return out;
    }
    const auto payload_bytes = stream.raw->size() == 16 ? 0 : stream.decoded->size();
    std::uint32_t counter = 0;
    std::uint64_t next_offset = 0;
    std::size_t p = 0;
    try {
        while (p < indices.size()) {
            const auto ni = indices[p];
            const auto &record = records.at(ni);
            require(record.at("offset") == next_offset, "block record boundary discontinuity");
            const auto filter = native_record_input_filter(record, bytesof(record.at("data")));
            if (filter["status"] == "read_error") {
                out["status"] = "read_error";
                out["error_record_index"] = ni;
                out["native_error_code"] = filter["native_error_code"];
                out["end_counter"] = counter;
                return out;
            }
            if (filter["descendant_counter_effect"] == "incremented")
                ++counter;
            if (filter["status"] == "skipped") {
                out["skipped_record_indices"].push_back(ni);
                next_offset += record.at("length").get<std::uint64_t>();
                ++p;
                continue;
            }
            auto tree = native_record_input_subtree(ni, records, counter);
            if (tree["status"] != "resolved") {
                out["status"] = "read_error";
                out["failed_root_record_index"] = ni;
                out["failed_input_tree"] = std::move(tree);
                return out;
            }
            counter = tree.at("end_counter").get<std::uint32_t>();
            auto last = ni;
            if (!tree["consumed_record_indices"].empty())
                last = tree["consumed_record_indices"].back().get<std::size_t>();
            require(p + tree["consumed_record_indices"].size() < indices.size() &&
                        indices[p + tree["consumed_record_indices"].size()] == last,
                    "block subtree source index mismatch");
            p += tree["consumed_record_indices"].size() + 1;
            next_offset = records[last].at("offset").get<std::uint64_t>() +
                          records[last].at("length").get<std::uint64_t>();
            out["roots"].push_back({{"native_record_index", ni}, {"input_tree", std::move(tree)}});
        }
        require(next_offset == payload_bytes, "block record payload has unvisited bytes");
        out["status"] = "resolved";
        out["termination"] = "end_of_record_payload";
        out["end_counter"] = counter;
    } catch (const std::exception &e) {
        out["status"] = "unresolved";
        out["error"] = e.what();
    }
    return out;
}

Json native_input_containers(const std::vector<Stream> &streams, const Json &index,
                             const Json &records) {
    std::map<StreamPath, std::vector<std::size_t>> containers, record_indices;
    std::map<StreamPath, std::string> kinds;
    for (std::size_t si = 0; si < streams.size(); ++si) {
        const auto &path = streams[si].path;
        if (path.size() < 2)
            continue;
        for (auto key : {"P3D-SSYS", "P3D-SMC", "P3D-SMG"}) {
            if (!index.contains(key) || path[path.size() - 2] != index.at(key))
                continue;
            StreamPath container(path.begin(), path.end() - 1);
            containers[container].push_back(si);
            kinds[container] = key;
            break;
        }
    }
    for (std::size_t ni = 0; ni < records.size(); ++ni)
        if (records[ni].contains("stream"))
            record_indices[records[ni]["stream"].get<StreamPath>()].push_back(ni);
    Json out = Json::array();
    for (const auto &[path, source_indices] : containers) {
        Json result = {{"container", path},
                       {"kind", kinds.at(path)},
                       {"scope", "container_input_if_opened"},
                       {"reader_profile", "bimbase_2025_native_block_input"},
                       {"runtime_registration", "not_evaluated"},
                       {"blocks", Json::array()},
                       {"input_order", Json::array()}};
        std::map<std::string, std::vector<std::size_t>> named;
        for (auto si : source_indices) {
            const auto &s = streams[si];
            named[s.path.back()].push_back(result["blocks"].size());
            auto h = s.raw ? native_block_header(*s.raw) : native_block_header({});
            Json block = {{"stream_index", si},
                          {"stream", s.path},
                          {"header", h},
                          {"reachability", "not_reached"}};
            if (h["status"] == "resolved")
                block["record_input"] = block_records(s, records, record_indices[s.path], h);
            result["blocks"].push_back(std::move(block));
        }
        // Lookup each consecutive logical name. Sorting physical CFB names or
        // skipping a missing number changes the native loader's behavior.
        for (std::uint64_t number = 1;; ++number) {
            const auto logical = "$" + std::to_string(number);
            Json stop = {{"block_number", number}, {"logical_name", logical}};
            if (!index.contains(logical)) {
                stop["reason"] = "missing_block_alias";
                result["stop"] = std::move(stop);
                break;
            }
            const auto physical = index.at(logical).get<std::string>();
            auto it = named.find(physical);
            if (it == named.end() || it->second.size() != 1) {
                stop["reason"] =
                    it == named.end() ? "missing_block_stream" : "ambiguous_block_stream";
                result["stop"] = std::move(stop);
                break;
            }
            const auto bi = it->second.front();
            auto &block = result["blocks"][bi];
            block["reachability"] = "reached";
            result["input_order"].push_back(
                {{"block_number", number}, {"logical_name", logical}, {"block_index", bi}});
            const auto &header = block["header"];
            if (header["status"] != "resolved" || header["stop_after_block"] == true) {
                stop["reason"] =
                    header["status"] == "resolved" ? "last_block_flag" : "invalid_block_header";
                stop["native_block_return_code"] =
                    header.value("native_error_code", header.value("block_return_code", Json()));
                result["stop"] = std::move(stop);
                break;
            }
        }
        result["list_preparation"] = native_list_input_preparation(result, records);
        out.push_back(std::move(result));
    }
    return out;
}

Json Document::native_input_containers() const {
    auto result = p3d::native_input_containers(streams(), index(), native_records());
    for (auto &container : result)
        if (container["kind"] == "P3D-SSYS") {
            container["system_id_assignments"] = native_system_id_assignments(
                container["list_preparation"], native_records(), file_header());
            container["initial_material_table"] =
                native_system_material_table(container["list_preparation"], native_records(),
                                             container["system_id_assignments"]);
        }
    return result;
}
} // namespace p3d
