#include <chrono>
#include <fstream>
#include <iostream>
#include <p3d/reader.hpp>
#ifdef _WIN32
#include <windows.h>
#include <shellapi.h>
#include <psapi.h>
#endif
int main(int argc, char **argv) {
    try {
        std::vector<std::filesystem::path> args;
#ifdef _WIN32
        int n;
        auto w = CommandLineToArgvW(GetCommandLineW(), &n);
        for (int i = 1; i < n; ++i)
            args.emplace_back(w[i]);
        LocalFree(w);
#else
        for (int i = 1; i < argc; ++i)
            args.emplace_back(argv[i]);
#endif
        if (args.empty()) {
            std::cout
                << "p3d-inspect FILE [--threads N] [--dump FILE] [--scene FILE] [--graph FILE]\n";
            return 0;
        }
        p3d::Options opt;
        std::filesystem::path dump, scene, graph, streams_dir;
        bool native = false, expanded = false;
        for (std::size_t i = 1; i < args.size(); ++i) {
            auto k = args[i].string();
            if (k == "--threads" && i + 1 < args.size())
                opt.threads = std::stoul(args[++i].string());
            else if (k == "--dump" && i + 1 < args.size())
                dump = args[++i];
            else if (k == "--scene" && i + 1 < args.size())
                scene = args[++i];
            else if (k == "--graph" && i + 1 < args.size())
                graph = args[++i];
            else if (k == "--streams-dir" && i + 1 < args.size())
                streams_dir = args[++i];
            else if (k == "--native-summary")
                native = true;
            else if (k == "--expanded-summary")
                expanded = true;
            else
                throw std::runtime_error("unknown/missing option: " + k);
        }
        auto t = std::chrono::steady_clock::now();
        p3d::Document d(args[0], opt);
        auto done = std::chrono::steady_clock::now();
        auto summary = d.summary();
        summary["parse_seconds"] = std::chrono::duration<double>(done - t).count();
        summary["threads"] = opt.threads;
        auto write = [](const std::filesystem::path &path, const p3d::Json &v) {
            std::ofstream f(path, std::ios::binary);
            if (!f)
                throw std::runtime_error("cannot open output");
            f << v.dump();
            if (!f)
                throw std::runtime_error("output write failed");
        };
        if (!dump.empty())
            write(dump, {{"index", d.index()},
                         {"file_header", d.file_header()},
                         {"objects", d.objects()},
                         {"native_records", d.native_records()},
                         {"graphics_records", d.graphics_records()},
                         {"bindings", d.bindings()},
                         {"relationships", d.relationships()},
                         {"materials", d.materials()},
                         {"schemas", d.schemas()},
                         {"models", d.models()},
                         {"color_tables", d.color_tables()},
                         {"layer_tables", d.layer_tables()}});
        if (!graph.empty())
            write(graph, d.object_graph());
        if (!streams_dir.empty()) {
            std::filesystem::create_directories(streams_dir);
            p3d::Json manifest = p3d::Json::array();
            for (std::size_t i = 0; i < d.streams().size(); ++i) {
                const auto &stream = d.streams()[i];
                auto raw_name = std::to_string(i) + ".raw";
                auto decoded_name = std::to_string(i) + ".decoded";
                auto binary = [&](const std::string &name, const p3d::Bytes &data) {
                    std::ofstream f(streams_dir / name, std::ios::binary);
                    if (!f)
                        throw std::runtime_error("cannot open stream output");
                    f.write(reinterpret_cast<const char *>(data.data()), data.size());
                    if (!f)
                        throw std::runtime_error("stream output write failed");
                };
                binary(raw_name, *stream.raw);
                binary(decoded_name, *stream.decoded);
                manifest.push_back({{"path", stream.path},
                                    {"raw_file", raw_name},
                                    {"decoded_file", decoded_name},
                                    {"raw_bytes", stream.raw->size()},
                                    {"decoded_bytes", stream.decoded->size()}});
            }
            write(streams_dir / "manifest.json", manifest);
        }
        if (native || expanded || !scene.empty()) {
            auto start = std::chrono::steady_clock::now();
            auto ns = d.native_scene();
            summary["native_scene_seconds"] =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            summary["native_scene"] = ns.summary();
            if (expanded || !scene.empty()) {
                auto start_expand = std::chrono::steady_clock::now();
                auto result = ns.expanded();
                summary["expansion_seconds"] =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - start_expand)
                        .count();
                if (!scene.empty())
                    write(scene, result);
            }
        }
#ifdef _WIN32
        PROCESS_MEMORY_COUNTERS counters{};
        if (GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters)))
            summary["peak_working_set_bytes"] = counters.PeakWorkingSetSize;
#endif
        std::cout << summary.dump() << std::endl;
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
