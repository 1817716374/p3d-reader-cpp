#include <p3d/reader.hpp>
#include <iostream>
#ifdef _WIN32
#include <windows.h>
#include <shellapi.h>
#endif
int main(int argc, char **argv) {
    try {
        std::filesystem::path input;
#ifdef _WIN32
        int count = 0;
        auto wide = CommandLineToArgvW(GetCommandLineW(), &count);
        if (!wide || count != 2) {
            if (wide)
                LocalFree(wide);
            return 2;
        }
        input = wide[1];
        LocalFree(wide);
#else
        if (argc != 2)
            return 2;
        input = std::filesystem::u8path(argv[1]);
#endif
        p3d::Options options;
        options.threads = 4;
        p3d::Document document(input, options);
        auto scene = document.native_scene();
        std::size_t primitives = 0;
        scene.for_each_primitive([&](const p3d::PrimitiveView &view) {
            ++primitives;
            if (!view.geometry || !view.source_range)
                throw std::runtime_error("missing primitive view");
        });
        auto summary = scene.summary();
        summary["primitives"] = primitives;
        std::cout << summary.dump() << '\n';
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
