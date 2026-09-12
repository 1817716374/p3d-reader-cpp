#include "internal.hpp"

namespace p3d {
Json native_system_attribute_input(const std::vector<Stream> &streams, const Json &index,
                                   const StreamPath &system, const Json &ids) {
    Json out = {{"scope", "first_system_input_with_empty_id_registry"},
                {"reader_profile", "bimbase_2025_system_attribute_input"},
                {"status", "unresolved"},
                {"blocks", Json::array()},
                {"attachments", Json::array()},
                {"attribute_order", "source_read_order"},
                {"runtime_attribute_sort", "initial_collection_lookup"}};
    if (ids.value("status", Json()) != "resolved") {
        out["reason"] = "complete_system_id_assignments_required";
        return out;
    }
    if (system.empty() || !index.contains("P3D-SSYSA")) {
        out["reason"] = "attribute_container_alias_unavailable";
        return out;
    }
    auto path = system;
    path.back() = index.at("P3D-SSYSA").get<std::string>();
    out["container"] = path;
    std::map<std::string, const Stream *> named;
    for (const auto &s : streams) {
        if (s.path.size() != path.size() + 1 ||
            !std::equal(path.begin(), path.end(), s.path.begin()))
            continue;
        if (!named.emplace(s.path.back(), &s).second) {
            out["reason"] = "ambiguous_attribute_stream";
            return out;
        }
    }
    if (named.empty()) {
        out["status"] = "absent";
        return out;
    }
    if (!index.contains("P3D~MPH")) {
        // Native lookup can create a new alias. Its generated name depends on
        // earlier index operations, so missing aliases are not guessed here.
        out["reason"] = "master_header_alias_unavailable";
        return out;
    }
    auto master = named.find(index.at("P3D~MPH").get<std::string>());
    if (master == named.end()) {
        out["status"] = "not_loaded";
        out["reason"] = "missing_master_header_stream";
        return out;
    }
    const auto &ms = *master->second;
    if (!ms.raw || ms.compression_offset != static_cast<std::size_t>(-1) || ms.raw->size() < 8) {
        out["reason"] = "master_header_layout_unavailable";
        return out;
    }
    Reader mh(*ms.raw);
    out["master_header"] = {{"stream", ms.path}, {"source_word", mh.u32()}};
    const auto block_count = mh.u32();
    out["master_header"]["block_count"] = block_count;
    std::map<std::uint64_t, Json> registry;
    for (const auto &item : ids.at("registry"))
        registry.emplace(item.at("id").get<std::uint64_t>(), item);
    std::map<std::uint64_t, std::size_t> attached;
    out["status"] = "resolved";
    for (std::uint64_t number = 1; number <= block_count; ++number) {
        const auto logical = "$" + std::to_string(number);
        Json block = {
            {"block_number", number}, {"logical_name", logical}, {"records", Json::array()}};
        if (!index.contains(logical)) {
            block["status"] = "unresolved";
            block["reason"] = "block_alias_unavailable";
            out["blocks"].push_back(std::move(block));
            out["status"] = "partial";
            break; // Alias creation can affect subsequent physical lookups.
        }
        const auto found = named.find(index.at(logical).get<std::string>());
        if (found == named.end()) {
            block["status"] = "not_loaded";
            block["reason"] = "missing_block_stream";
            out["blocks"].push_back(std::move(block));
            continue; // The master-header loop ignores an individual block failure.
        }
        const auto &s = *found->second;
        block["stream"] = s.path;
        if (!s.raw || s.raw->size() < 16) {
            block["status"] = "stopped";
            block["reason"] = "truncated_block_header";
            out["blocks"].push_back(std::move(block));
            continue;
        }
        std::uint32_t count = 0;
        for (unsigned i = 0; i < 4; ++i)
            count |= std::uint32_t((*s.raw)[i] ^ (i ? (*s.raw)[i - 1] : 0x26)) << (8 * i);
        block["record_count"] = count;
        block["control_flags_used"] = false;
        if (count == 0) {
            block["status"] = "completed";
            block["end_offset"] = 0;
            out["blocks"].push_back(std::move(block));
            continue;
        }
        if (!s.decoded || s.compression_offset != 24) {
            block["status"] = "unresolved";
            block["reason"] = "record_payload_layout_unconfirmed";
            out["blocks"].push_back(std::move(block));
            out["status"] = "partial";
            break;
        }
        Reader r(*s.decoded);
        bool skipped = false;
        block["status"] = "completed";
        for (std::uint64_t ordinal = 0; ordinal < count; ++ordinal) {
            const auto offset = r.p;
            if (r.left() < 24) {
                // Native skip advances its position without decreasing its
                // remaining-byte counter. Do not simulate the resulting read
                // beyond the decoded buffer after an unindexed record.
                block["status"] = skipped ? "unresolved" : "stopped";
                block["reason"] =
                    skipped ? "post_skip_read_exceeds_decoded_buffer" : "short_record_header";
                if (!skipped)
                    r.skip(r.left());
                break;
            }
            const auto magic = r.u32(), length = r.u32();
            const auto flags = r.u64(), id = r.u64();
            if (magic != 0xa11b) {
                block["status"] = "stopped";
                block["reason"] = "invalid_record_magic";
                break;
            }
            Json record = {{"record_ordinal", ordinal},
                           {"offset", offset},
                           {"declared_length", length},
                           {"flags", flags},
                           {"id", id}};
            auto object = registry.find(id);
            if (object == registry.end()) {
                record["action"] = "skip_unindexed_id";
                // P3DDC's skip adds this exact value to the current cursor.
                // Do not substitute the source parser's (28 + length) boundary.
                if (length > r.left()) {
                    record["action"] = "unresolved_skip";
                    block["status"] = "unresolved";
                    block["reason"] = "skip_exceeds_decoded_buffer";
                } else {
                    r.skip(length);
                    skipped = length != 0 || skipped;
                }
            } else {
                record["target"] = object->second;
                record["native_block_mismatch"] = object->second.at("block_number") != number;
                if (attached.count(id)) {
                    record["action"] = "reject_existing_collection";
                    record["attachment_index"] = attached.at(id);
                    record["native_reader_return_code"] = 1;
                    // The caller ignores this return code; no body is consumed.
                } else {
                    try {
                        const auto source_count = r.u32();
                        const auto read_count = static_cast<std::uint16_t>(source_count);
                        record["declared_attribute_count"] = source_count;
                        record["read_attribute_count"] = read_count;
                        Json attributes = Json::array();
                        for (unsigned ai = 0; ai < read_count; ++ai) {
                            const auto attr_offset = r.p;
                            const auto group = r.u16(), key = r.u16();
                            const auto ix = r.u32(), sf = r.u32(), reserved = r.u32();
                            auto raw = r.take(sf & 0x7fffffffu);
                            Json decoded;
                            try {
                                decoded = decode_attribute(group, key, raw, ix);
                            } catch (const std::exception &e) {
                                decoded = {{"encoding", "opaque"}, {"decode_error", e.what()}};
                            }
                            attributes.push_back({{"source_ordinal", ai},
                                                  {"offset", attr_offset},
                                                  {"group", group},
                                                  {"key", key},
                                                  {"index", ix},
                                                  {"size_flags", sf},
                                                  {"reserved", reserved},
                                                  {"payload", rawbytes(raw)},
                                                  {"decoded", std::move(decoded)}});
                        }
                        const auto trailer = r.u32();
                        record["trailer"] = trailer;
                        record["native_reader_return_code"] = 0;
                        if (source_count == 0) {
                            record["action"] = "empty_without_collection";
                        } else {
                            const auto attachment = out["attachments"].size();
                            attached.emplace(id, attachment);
                            record["action"] = "attach";
                            record["attachment_index"] = attachment;
                            auto lookup = native_attribute_lookup(attributes);
                            out["attachments"].push_back({{"target", object->second},
                                                          {"stream", s.path},
                                                          {"block_number", number},
                                                          {"record_ordinal", ordinal},
                                                          {"offset", offset},
                                                          {"attributes", std::move(attributes)},
                                                          {"lookup", std::move(lookup)},
                                                          {"trailer_bit_0", (trailer & 1) != 0}});
                        }
                    } catch (const std::exception &e) {
                        record["action"] = "unresolved_attribute_read";
                        record["error"] = e.what();
                        block["status"] = "unresolved";
                        block["reason"] = "incomplete_attribute_body";
                    }
                }
            }
            record["end_offset"] = r.p;
            block["records"].push_back(std::move(record));
            if (block["status"] == "unresolved")
                break;
        }
        block["end_offset"] = r.p;
        block["unconsumed_bytes"] = r.left();
        const bool unresolved = block["status"] == "unresolved";
        out["blocks"].push_back(std::move(block));
        if (unresolved) {
            out["status"] = "partial";
            break; // A partial allocation/read could affect later attachment.
        }
    }
    return out;
}
} // namespace p3d
