// Differential corpus verifier. Links only the independent parser library.
#include "internal.hpp"
#include <p3d/model_bounds.hpp>
#include <fstream>
#include <iostream>

namespace {
using namespace p3d;
struct Verifier {
    Json differences = Json::array();
    std::size_t checks = 0, entities = 0, attributes = 0, models = 0;
    std::string context;
    const Document *document = nullptr;
    std::map<std::size_t, Json> prepared_headers;
    std::size_t complete_headers = 0, expanded_ranges = 0;
    void same(const std::string &field, const Json &ours, const Json &native) {
        ++checks;
        if (ours != native)
            differences.push_back({{"context", context}, {"field", field},
                                   {"ours", ours}, {"native", native}});
    }
    std::uint64_t compare(const Json &ids, const Json &attrs, const Json &native, bool system) {
        same("ids.status", ids.at("status"), "resolved");
        same("attributes.status", attrs.at("status"), "resolved");
        require(ids.at("status") == "resolved" && attrs.at("status") == "resolved",
                "complete independent input projection required");
        Json entries = Json::array(), lookups = Json::array();
        std::array<Json, 2> blocks{Json::array(), Json::array()};
        for (const auto &root : ids.at("roots")) {
            auto &list = blocks.at(system ? 0 : root.at("input_list_index").get<std::size_t>());
            const auto number = root.at("block_number");
            if (list.empty() || list.back().at("number") != number)
                list.push_back({{"number", number}, {"roots", Json::array()}});
            list.back()["roots"].push_back(entries.size());
            for (const auto &entry : root.at("records")) {
                require(entry.at("input_occurrence_index") == entries.size(), "occurrence order");
                entries.push_back(entry);
            }
        }
        for (const auto &entry : ids.at("registry"))
            lookups.push_back({{"id", entry.at("id")}, {"entity", entry.at("input_occurrence_index")}});
        same("registry_lookups", lookups, native.at("registry_lookups"));
        same("entity_count", entries.size(), native.at("entities").size());
        require(entries.size() == native.at("entities").size(), "entity count mismatch");
        require(native.at("lists").size() == 2, "two native entity lists required");
        for (std::size_t i = 0; i < 2; ++i)
            same("list[" + std::to_string(i) + "].blocks", blocks[i], native.at("lists")[i].at("blocks"));
        std::map<std::size_t, Json> attached;
        for (const auto &attachment : attrs.at("attachments"))
            require(attached.emplace(attachment.at("target").at("input_occurrence_index").get<std::size_t>(),
                                     attachment).second, "duplicate attached collection");
        std::vector<Json> children(entries.size(), Json::array());
        for (std::size_t i = 0; i < entries.size(); ++i) {
            const auto &parent = entries[i].at("parent_input_occurrence_index");
            if (!parent.is_null()) children.at(parent.get<std::size_t>()).push_back(i);
        }
        for (std::size_t i = 0; i < entries.size(); ++i) {
            const auto &entry = entries[i], &observed = native.at("entities")[i];
            const auto field = "entity[" + std::to_string(i) + "].";
            same(field + "id", entry.at("assigned_id"), observed.at("id"));
            same(field + "parent", entry.at("parent_input_occurrence_index"), observed.at("parent"));
            same(field + "children", children[i], observed.at("children"));
            // Corpus hypothesis: apart from physical range expansion and the
            // independently prepared common header, these payloads are unchanged.
            // Compare every byte; never accept unsupported size conversions.
            const auto ni = entry.at("native_record_index").get<std::size_t>();
            const auto &record = document->native_records().at(ni);
            const auto &prepared = prepared_headers.at(ni);
            const auto path = record.at("stream").get<StreamPath>();
            Bytes input;
            for (const auto &stream : document->streams()) {
                if (stream.path != path) continue;
                const auto offset = record.at("offset").get<std::size_t>();
                const auto size = record.at("length").get<std::size_t>();
                require(offset <= stream.decoded->size() && size <= stream.decoded->size() - offset,
                        "complete source record required");
                input = slice(*stream.decoded, offset, size);
                break;
            }
            require(input.size() >= 36 && input.size() == 4 + 2 * prepared.at("output_record_word_count").get<std::size_t>(),
                    "unsupported record size conversion in full-header comparison");
            auto write = [&](std::size_t offset, auto value) {
                require(offset + sizeof(value) <= input.size(), "header write bounds");
                std::memcpy(input.data() + offset, &value, sizeof(value));
            };
            const auto bounds = decode_native_record_bounds_header(bytesof(record.at("data")));
            if (bounds.at("status") == "decoded") {
                for (unsigned axis = 0; axis < 6; ++axis)
                    write(60 + 8 * axis, bounds.at("integer_range")[axis].get<std::int64_t>());
                ++expanded_ranges;
            } else require(bounds.at("status") == "not_present", "invalid source bounds header");
            write(6, prepared.at("output_element_flags").get<std::uint16_t>());
            write(8, prepared.at("output_record_word_count").get<std::uint32_t>());
            write(12, prepared.at("output_base_word_count").get<std::uint32_t>());
            write(20, entry.at("assigned_id").get<std::uint64_t>());
            const auto &count = prepared.at("descendant_count_update");
            if (!count.is_null() && count.at("written") == true)
                write(4 + count.at("header_offset").get<std::size_t>(), count.at("output_value").get<std::uint32_t>());
            same(field + "complete_header", hex(slice(input, 4, input.size() - 4)), observed.at("loaded_header"));
            ++complete_headers;
            Json values = Json::array(), flags;
            const auto attachment = attached.find(i);
            if (attachment != attached.end()) {
                flags = attachment->second.at("trailer_bit_0").get<bool>() ? 1 : 0;
                for (const auto &attr : attachment->second.at("attributes"))
                    values.push_back({{"key", attr.at("group").get<std::uint32_t>() |
                                               (attr.at("key").get<std::uint32_t>() << 16)},
                        {"index", attr.at("index")}, {"flag", attr.at("size_flags").get<std::uint32_t>() >> 31},
                        {"ordinal", attr.at("source_ordinal")}, {"payload_hex", hex(bytesof(attr.at("payload")))}});
            }
            same(field + "attributes", values, observed.at("attributes"));
            same(field + "attribute_flags", flags, observed.at("attribute_flags"));
            attributes += observed.at("attributes").size();
        }
        entities += entries.size();
        return ids.at("final_id_counter").get<std::uint64_t>();
    }
};
} // namespace

int main(int argc, char **argv) {
    using namespace p3d;
    try {
        require(argc == 3, "usage: verify-main-entities oracle-file.json verification.json");
        Json native;
        std::ifstream(std::filesystem::u8path(argv[1])) >> native;
        Document document(std::filesystem::u8path(native.at("source").at("path").get<std::string>()));
        const auto containers = native_input_containers(document.streams(), document.index(), document.native_records());
        Verifier verifier;
        verifier.document = &document;
        for (const auto &container : containers)
            for (const auto &root : container.at("list_preparation").at("roots"))
                for (const auto &header : root.at("headers"))
                    verifier.prepared_headers[header.at("native_record_index").get<std::size_t>()] = header;
        verifier.context = "file";
        const auto &loading = native.at("model_loading");
        verifier.same("unchanged", native.at("source_and_copy_unchanged"), true);
        verifier.same("scope", native.at("scope"), "original_system_first_then_directory_models_core_console_input");
        verifier.same("system_return", loading.at("system_return"), 0);
        std::uint64_t counter = 0;
        std::size_t systems = 0;
        for (const auto &container : containers) {
            if (container.at("kind") != "P3D-SSYS") continue;
            ++systems;
            verifier.context = "system";
            const auto ids = native_system_id_assignments(container.at("list_preparation"), document.native_records(), document.file_header());
            const auto attrs = native_system_attribute_input(document.streams(), document.index(), container.at("container").get<StreamPath>(), ids);
            counter = verifier.compare(ids, attrs, loading.at("system_initial"), true);
            verifier.same("counter_after", counter, loading.at("system_counter"));
            verifier.same("control_loaded", loading.at("system_initial").at("lists")[0].at("loaded"), true);
        }
        verifier.same("system_container_count", systems, 1);
        require(systems == 1, "exactly one source system container required");
        std::map<std::uint32_t, StreamPath> paths;
        for (const auto &stream : document.streams()) {
            if (stream.path.size() != 3 || stream.path.back() != document.index().value("P3D~MMH", std::string())) continue;
            for (auto it = document.index().begin(); it != document.index().end(); ++it) {
                if (it.value() != stream.path[1] || it.key().empty() || it.key().front() != '#') continue;
                const auto id = std::stoul(it.key().substr(1), nullptr, 16);
                require(id < 0x80000000 && paths.emplace(static_cast<std::uint32_t>(id), StreamPath(stream.path.begin(), stream.path.begin() + 2)).second,
                        "unique model storage required");
            }
        }
        verifier.context = "file";
        verifier.same("model_count", paths.size(), loading.at("models").size());
        verifier.same("active_model_count", paths.size(), loading.at("active_model_count"));
        verifier.same("held_model_count", paths.size(), loading.at("held_model_count"));
        verifier.same("file_reference_count", loading.at("file_reference_count"), 1);
        std::set<std::uint32_t> seen;
        for (const auto &model : loading.at("models")) {
            const auto id = model.at("model_id").get<std::uint32_t>();
            require(seen.insert(id).second, "duplicate model observation");
            verifier.context = "model[" + std::to_string(id) + "]";
            verifier.same("counter_before", counter, model.at("counter_before"));
            const auto &path = paths.at(id);
            const auto ids = native_model_id_assignments(containers, document.native_records(), document.index(), path, counter);
            const auto attrs = native_model_attribute_input(document.streams(), document.index(), path, ids);
            counter = verifier.compare(ids, attrs, model, false);
            verifier.same("counter_after", counter, model.at("counter_after"));
            verifier.same("return_code", model.at("return_code"), 0);
            verifier.same("reference_count", model.at("model_reference_count"), 1);
            verifier.same("readonly", model.at("readonly"), true);
            for (const auto &list : model.at("lists")) verifier.same("list_loaded", list.at("loaded"), true);
            ++verifier.models;
        }
        verifier.context = "file";
        verifier.same("final_counter", counter, loading.at("final_counter"));
        verifier.same("system_preserved_across_models", loading.at("system_initial"), loading.at("system"));
        Json result = {{"checks", verifier.checks}, {"models", verifier.models},
            {"complete_headers", verifier.complete_headers}, {"expanded_ranges", verifier.expanded_ranges},
            {"entities", verifier.entities}, {"attributes", verifier.attributes}, {"differences", verifier.differences}};
        std::ofstream(std::filesystem::u8path(argv[2])) << result.dump(2) << '\n';
        std::cout << verifier.checks << " checks, " << verifier.differences.size() << " differences\n";
        return verifier.differences.empty() ? 0 : 1;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 2;
    }
}
