// Standalone corpus inventory and differential check; no vendor linkage.
#include "internal.hpp"
#include <p3d/model_header_input.hpp>
#include <fstream>
#include <iostream>

namespace {
using namespace p3d;
Json inventory(const Document &document) {
    Json result = Json::array();
    for (const auto &stream : document.streams()) {
        if (stream.path.size() != 3 ||
            stream.path.back() != document.index().value("P3D~MMH", std::string()))
            continue;
        const auto &data = *stream.decoded;
        require(data.size() >= 4596, "complete model header required");
        const auto words = Reader(data, 4104).u32();
        require(std::uint64_t(words) * 2 <= data.size() - 4100, "model header length");
        std::string key;
        for (auto it = document.index().begin(); it != document.index().end(); ++it)
            if (it.value() == stream.path[1]) key = it.key();
        require(!key.empty() && key.front() == '#', "unmapped model storage");
        const auto source = slice(data, 4096, 4 + std::size_t(words) * 2);
        result.push_back({{"storage", stream.path}, {"model_key", key},
            {"version", Reader(source, 36).u16()},
            {"header_hex", hex(slice(source, 4, source.size() - 4))},
            {"source_record", rawbytes(source)},
            {"prefix_hex", hex(slice(data, 0, 4096))}});
    }
    return result;
}
} // namespace

int main(int argc, char **argv) {
    using namespace p3d;
    try {
        require(argc == 4, "usage: verify-main-model-headers --inventory manifest.json output.json OR --verify oracle.json output.json");
        Json input;
        std::ifstream(std::filesystem::u8path(argv[2])) >> input;
        const std::string mode = argv[1];
        require(mode == "--inventory" || mode == "--verify", "unknown mode");
        if (mode == "--inventory") {
            Json rows = Json::array();
            for (const auto &file : input.at("files")) {
                Document document(std::filesystem::u8path(file.at("path").get<std::string>()));
                rows.push_back({{"source", file}, {"headers", inventory(document)}});
            }
            std::ofstream(std::filesystem::u8path(argv[3])) << rows.dump(2) << '\n';
            std::cout << rows.size() << " files inventoried\n";
            return 0;
        }
        std::size_t checks = 0, models = 0, converted = 0;
        Json differences = Json::array();
        std::string path, field_prefix;
        auto same = [&](const std::string &field, const Json &ours, const Json &native) {
            ++checks;
            if (ours != native)
                differences.push_back({{"source", path}, {"field", field_prefix + field},
                                       {"ours", ours}, {"native", native}});
        };
        for (const auto &file : input.at("cases")) {
            path = file.at("source").at("path").get<std::string>();
            field_prefix.clear();
            Document document(std::filesystem::u8path(path));
            std::map<std::uint32_t, Json> headers;
            for (const auto &header : inventory(document)) {
                auto id = std::stoul(header.at("model_key").get<std::string>().substr(1), nullptr, 16);
                require(headers.emplace(id, header).second, "duplicate model source");
            }
            const auto &loading = file.at("model_loading");
            same("unchanged", file.at("source_and_copy_unchanged"), true);
            same("acquired_count", headers.size(), loading.at("models").size());
            same("active_count", headers.size(), loading.at("active_model_count"));
            same("held_count", headers.size(), loading.at("held_model_count"));
            same("file_reference_count", loading.at("file_reference_count"), 1);
            std::set<std::uint32_t> seen;
            for (const auto &native : loading.at("models")) {
                const auto id = native.at("model_id").get<std::uint32_t>();
                require(seen.insert(id).second, "duplicate acquired model");
                const auto &header = headers.at(id);
                const auto projected = initial_native_model_header_input(bytesof(header.at("source_record")));
                field_prefix = "model[" + std::to_string(id) + "].";
                same("projection_status", projected.at("status"), "resolved");
                require(projected.at("status") == "resolved", "unsupported projection");
                same("complete_loaded_header", hex(bytesof(projected.at("loaded_header"))), native.at("header_hex"));
                same("prefix_4096_bytes", header.at("prefix_hex"), native.at("prefix_hex"));
                same("spatial", projected.at("spatial"), native.at("spatial"));
                same("model_vtable", native.at("model_vtable"), "0x5333c8");
                same("readonly", native.at("readonly"), true);
                same("model_reference_count", native.at("model_reference_count"), 1);
                same("cached_model_reused", native.at("cached_model_reused"), true);
                same("entity_lists_loaded", native.at("entity_lists_loaded"), false);
                same("prefix_return_code", native.at("prefix_return_code"), 0);
                same("document_integration", document.models().at(std::to_string(id)).at("initial_header_input"), projected);
                ++models;
                converted += header.at("version") == 7;
            }
        }
        Json result = {{"scope", "real_main_file_model_header_input_and_prefix"},
            {"files", input.at("cases").size()}, {"models", models},
            {"version_7_conversions", converted}, {"checks", checks},
            {"differences", differences}, {"entity_loading", "not_executed"}};
        std::ofstream(std::filesystem::u8path(argv[3])) << result.dump(2) << '\n';
        std::cout << checks << " checks, " << models << " models, "
                  << differences.size() << " differences\n";
        return differences.empty() ? 0 : 1;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
