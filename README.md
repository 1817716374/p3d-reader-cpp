# P3D Reader

P3D Reader 是一个用于读取 **BIMBase / PKPM P3D** 文件的独立、只读 C++17 库，提供几何、属性、结构树、材质和底层记录的访问接口。

库直接解析文件，不需要安装原建模软件。数据以文档对象和场景对象提供，并保留源文件中的定义、实例及引用关系。

## 功能

| 数据 | 接口提供的内容 |
| --- | --- |
| 几何 | 网格、曲线及曲面重建结果、源 B 样条曲线求值、地形、线和文字、实例变换、图元范围、显式源 UV |
| 属性 | 属性树、二进制字段、对象身份、构件绑定、模型属性和对象关系 |
| 结构树 | 源结构树、模型树、父子关系、构件引用、块定义及嵌套块引用 |
| 材质 | 材质定义、参数、颜色表、材质候选、贴图路径引用和嵌入图片文件 |
| 图层与视图记录 | 图层表与模型引用、组合属性同步及来源、显式模型链接顺序 |
| 底层数据 | OLE 流、解压数据、DEX 对象、原生记录、图形命令及原始载荷 |

源文件中的共享几何以定义和实例表达；独立源几何保持独立。未绑定对象、重复记录、缺失引用和歧义候选均保留，未知字段保留原始载荷。

## 构建

需要 CMake 3.20+ 和支持 C++17 的 C/C++ 工具链。依赖源码随仓库提供；非 Windows 平台还需要 iconv。

在源码根目录运行：

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

主要目标为静态库 `p3d::reader` 和命令行工具 `p3d-inspect`。构建选项及安装方式见[接入指南](docs/接入指南.md)。

## CMake 集成

通过子项目集成：

```cmake
set(P3D_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(P3D_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/p3d-reader-cpp)
target_link_libraries(your_app PRIVATE p3d::reader)
```

也可以先安装库，然后使用 `find_package(p3d CONFIG REQUIRED)` 引入同一个目标。

## 读取文件

```cpp
#include <p3d/reader.hpp>
#include <iostream>

int main() {
    try {
        p3d::Options options;
        options.threads = 4;

        p3d::Document document(std::filesystem::u8path(u8"example.p3d"), options);
        auto scene = document.native_scene();

        std::cout << scene.summary().dump(2) << '\n';
        const auto& properties = document.objects();
        const auto& materials = document.materials();
        const auto& graph = scene.metadata.at("document_graph");
        // 使用这些接口访问几何、属性、材质和层级关系。
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
```

`NativeScene` 保存共享定义与实例，`for_each_primitive()` 提供逐图元访问。需要烘焙变换后的逐构件数据时，可调用 `scene.expanded()`。

命令行工具支持查看统计及导出记录：

```sh
p3d-inspect example.p3d --threads 4 --native-summary
p3d-inspect example.p3d --dump document.json --graph graph.json --scene scene.json
```

请使用构建目录中的可执行文件；多配置生成器通常将其放在对应配置子目录下。

## 数据约定

- 坐标使用 double，保留源坐标系；不自动换算单位或推定地理坐标系。
- 曲线重建默认整圆 64 段，可配置弦高与分段上限。
- 材质参数和贴图引用按源记录提供，外部文件路径由调用方处理。
- 对象、模型、定义索引和源记录身份具有文档作用域。
- 未支持或含义未明的内容通过原始载荷、错误字段及引用状态提供。

具体数据边界见[已知限制](docs/已知限制.md)。

## 文档

- [接入指南](docs/接入指南.md)：构建、几何、属性、结构树、材质和命令行用法。
- [格式说明](docs/格式说明.md)：文件结构与数据接口的对应关系。
- [已知限制](docs/已知限制.md)：尚未支持的结构及数据使用注意事项。
- [贡献指南](CONTRIBUTING.md)：问题反馈、代码和文档贡献方式。
- [第三方依赖](THIRD_PARTY.md)：依赖来源与许可证。

## 许可证

项目原创代码采用 [MIT 许可证](LICENSE)，第三方依赖保留各自的许可证。
