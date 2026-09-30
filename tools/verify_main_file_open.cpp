// Compare our persisted-file parsing against probe_main_file_open.py output.
// Build against the existing library; vendor binaries are never linked here.
#include "internal.hpp"
#include <fstream>
#include <iostream>

int main(int argc, char **argv) {
    using namespace p3d;
    try {
        require(argc == 3, "usage: verify-main-file-open observations.json result.json");
        std::ifstream input(std::filesystem::u8path(argv[1]));
        Json oracle;
        input >> oracle;
        Json differences = Json::array();
        std::size_t checks = 0, models = 0, mappings = 0;
        std::string source;
        auto same = [&](const std::string &field, const Json &ours, const Json &native) {
            ++checks;
            if (ours != native)
                differences.push_back({{"source", source}, {"field", field},
                                       {"ours", ours}, {"native", native}});
        };
        for (const auto &row : oracle.at("cases")) {
            source = row.at("source").at("path").get<std::string>();
            Document document(std::filesystem::u8path(source));
            same("return_code", row.at("return_code"), 0);
            same("readonly", row.at("readonly"), true);
            same("unchanged", row.at("source_and_copy_unchanged"), true);
            same("provider", row.at("provider_vtable"), "0x534a40");
            same("no_model_loading", row.at("loaded_model_count"), 0);
            const auto &file_header = document.file_header();
            same("header_status", file_header.at("status"), "resolved");
            Bytes header = bytesof(file_header.at("header_payload"));
            require(header.size() >= 0x610, "ordinary full header required");
            require(file_header.at("initial_probe").at("action") == "read_header_payload",
                    "ordinary header profile required");
            header.resize(0x610);
            auto flags = file_header.at("initial_probe").at("header_flags").get<std::uint32_t>();
            for (unsigned i = 0; i < 4; ++i)
                header[0x310 + i] = std::uint8_t(flags >> (8 * i));
            same("complete_loaded_header", hex(header), row.at("header_hex"));
            same("stream_name_map", document.index(), row.at("name_map"));
            mappings += document.index().size();
            const auto directory = document.native_model_directory();
            same("directory_status", directory.at("status"), "decoded");
            same("directory_native_status", directory.at("native_input_status"), "decoded");
            same("directory_version", directory.at("version"), row.at("directory_version"));
            same("directory_count", directory.at("entries").size(), row.at("models").size());
            require(directory.at("entries").size() == row.at("models").size(),
                    "directory count mismatch prevents ordered comparison");
            std::size_t index = 0;
            for (const auto &entry : directory.at("entries")) {
                const auto &native = row.at("models")[index];
                const auto prefix = "directory[" + std::to_string(index++) + "].";
                for (auto key : {"model_id", "kind", "unassigned_u16"})
                    same(prefix + key, entry.at(key), native.at(key));
                same(prefix + "name", entry.at("name").at("text"), native.at("name"));
                same(prefix + "secondary_string", entry.at("secondary_string").at("text"),
                     native.at("secondary_string"));
                const auto f = entry.at("source_flags").get<unsigned>();
                const Json flag_bytes = {unsigned(bool(f & 1)), unsigned(bool(f & 0x8000)),
                    unsigned(bool(f & 8)), unsigned(bool(f & 4)), unsigned(bool(f & 16)), 0};
                same(prefix + "flags", flag_bytes, native.at("flags"));
                same(prefix + "float64_bits", hex(slice(bytesof(entry.at("source_header")), 8, 8)),
                     native.at("unassigned_float64_bits"));
                const auto &mask = entry.at("bitmap");
                Json bitmap = nullptr;
                if (mask.at("status") == "decoded")
                    bitmap = {{"effective_bit_count", mask.at("effective_bit_count")},
                              {"packed_words", mask.at("packed_words")}};
                same(prefix + "bitmap", bitmap, native.at("bitmap"));
                same(prefix + "extensions", entry.at("extensions").size(), native.at("extension_count"));
                ++models;
            }
        }
        Json report = {{"scope", "persisted_main_file_header_mapping_and_ordered_directory"},
            {"files", oracle.at("cases").size()}, {"models", models}, {"stream_mappings", mappings},
            {"checks", checks}, {"differences", differences}, {"model_entity_loading", "not_executed"}};
        std::ofstream(std::filesystem::u8path(argv[2])) << report.dump(2) << '\n';
        std::cout << report.dump() << '\n';
        return differences.empty() ? 0 : 1;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
