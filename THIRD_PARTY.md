# 第三方依赖说明

本项目原创解析代码采用 [MIT 许可证](LICENSE)。以下依赖继续适用各自的许可证；许可证正文和版权声明保留原文。

| 组件 | 固定版本 / 来源 | 用途 | 许可证 |
| --- | --- | --- | --- |
| nlohmann/json | 3.12.0 | 属性树和 JSON 接口 | [MIT](third_party/nlohmann/LICENSE.MIT) |
| LZ4 | 1.10.0 | P3D 流和属性解压 | [许可证](third_party/lz4/LICENSE) |
| pugixml | 1.15 | 材质、模式及关系 XML | [许可证](third_party/pugixml/LICENSE.md) |
| mapbox/earcut.hpp | 2.2.4 | 多边形及带孔多边形三角化 | [许可证](third_party/mapbox/LICENSE) |
| miniz | 3.1.0，仅链接 tinfl | zlib 属性载荷解压 | [许可证](third_party/miniz/LICENSE) |
| Bentley BGFB 模式 | 模式快照，来源见 `src/data.cpp` | BGFB 结构解码 | [Apache-2.0](third_party/BGFB_LICENSE.txt) |
| GDAL DGN 默认颜色表 | v3.10.0 | 默认索引颜色 | [MIT 与原版权声明](third_party/GDAL_PALETTE_LICENSE.txt) |

依赖文件来源、字节数与 SHA-256 校验值见 [manifest.json](third_party/manifest.json)。BGFB 模式的源地址和快照校验值保存在 `src/data.cpp`。`miniz_export.h` 是本工程静态构建的导出宏配置；构建配置禁用未链接的 zlib 包装 API。

使用 MinGW 构建的检查工具静态链接 GCC 运行时。GCC 16.1.0 的 [GPLv3](third_party/gcc-runtime/COPYING3)、[运行库例外](third_party/gcc-runtime/COPYING.RUNTIME)、[MinGW-w64 声明](third_party/gcc-runtime/MINGW_COPYING)及 [winpthreads 声明](third_party/gcc-runtime/WINPTHREADS_COPYING)随工程保留。源码构建不要求使用预编译检查工具。

依赖的许可证正文和版权声明应随对应源码或二进制分发一并保留。
