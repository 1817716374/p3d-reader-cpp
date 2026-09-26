# 第三方依赖说明

本项目原创解析代码采用 [MIT 许可证](LICENSE)。以下依赖继续适用各自的许可证；许可证正文和版权声明保留原文。

| 组件 | 固定版本 / 来源 | 用途 | 许可证 |
| --- | --- | --- | --- |
| nlohmann/json | 3.12.0 | 属性树和 JSON 接口 | [MIT](third_party/nlohmann/LICENSE.MIT) |
| LZ4 | 1.10.0 | P3D 流和属性解压 | [许可证](third_party/lz4/LICENSE) |
| pugixml | 1.15 | 材质、模式及关系 XML | [许可证](third_party/pugixml/LICENSE.md) |
| mapbox/earcut.hpp | 2.2.4 | 多边形及带孔多边形三角化 | [许可证](third_party/mapbox/LICENSE) |
| SGI GLU / Chromium 快照 | `82532b9046be34a2ca93b650c5808a0827ecff16` | 原生网格消费路径的多边形三角化 | [SGI FreeB 2.0](third_party/glu/LICENSE.txt)、[Google BSD](third_party/glu/LICENSE.GOOGLE) |
| miniz | 3.1.0，仅链接 tinfl | zlib 属性载荷解压 | [许可证](third_party/miniz/LICENSE) |
| Bentley BGFB 模式 | 模式快照，来源见 `src/data.cpp` | BGFB 结构解码 | [Apache-2.0](third_party/BGFB_LICENSE.txt) |
| Bentley 几何算法 | `2350843ad1580a751ee24cc552460644015b07dc` 的 [bspdsurf.cpp](https://github.com/iTwin/imodel-native/blob/2350843ad1580a751ee24cc552460644015b07dc/iModelCore/GeomLibs/geom/src/bspline/bspdsurf.cpp)、[bsputil.cpp](https://github.com/iTwin/imodel-native/blob/2350843ad1580a751ee24cc552460644015b07dc/iModelCore/GeomLibs/geom/src/bspline/bsputil.cpp)、[bspcurv.cpp](https://github.com/iTwin/imodel-native/blob/2350843ad1580a751ee24cc552460644015b07dc/iModelCore/GeomLibs/geom/src/bspline/bspcurv.cpp) | `src/native_tube.cpp` 的扫掠曲面片、截面朝向、控制线求交及曲面片拼接算法基础，按 P3D 行为改写 | [Apache-2.0](third_party/BENTLEY_GEOMETRY_LICENSE.md) |
| GDAL DGN 默认颜色表 | v3.10.0 | 默认索引颜色 | [MIT 与原版权声明](third_party/GDAL_PALETTE_LICENSE.txt) |
| Manifold | `213b6557f28fd66f0e7fd44deb8531589fd288af`，3.5.1 源码快照 | 独立三角网格布尔求值 | [Apache-2.0](third_party/manifold/LICENSE)、[嵌入声明](third_party/manifold/LICENSE.EMBEDDED)、[NVIDIA BSD](third_party/manifold/LICENSE.NVIDIA)、[Clipper2 Boost](third_party/manifold/LICENSE.CLIPPER2) |

依赖文件来源、字节数与 SHA-256 校验值见 [manifest.json](third_party/manifest.json)。BGFB 模式的源地址和快照校验值保存在 `src/data.cpp`。`miniz_export.h` 是本工程静态构建的导出宏配置；构建配置禁用未链接的 zlib 包装 API。

使用 MinGW 构建的检查工具静态链接 GCC 运行时。GCC 16.1.0 的 [GPLv3](third_party/gcc-runtime/COPYING3)、[运行库例外](third_party/gcc-runtime/COPYING.RUNTIME)、[MinGW-w64 声明](third_party/gcc-runtime/MINGW_COPYING)及 [winpthreads 声明](third_party/gcc-runtime/WINPTHREADS_COPYING)随工程保留。源码构建不要求使用预编译检查工具。

依赖的许可证正文和版权声明应随对应源码或二进制分发一并保留。

`src/native_tube_elevate.cpp` 的升阶算法参考上述固定提交的 `bspcurv.cpp` 与 `bsputil.cpp`，保留 Bentley 版权及 Apache-2.0 标记。实现复用本库求导与周期打开内核，按 P3D 的节点归一化、节点分组、指定侧求导和权重舍入行为改写，并使用有界的局部工作数组。

`src/native_tube.cpp`、`src/native_tube_path.cpp` 与 `src/native_tube_fit.cpp` 保留 Bentley 版权及 Apache-2.0 标记，使用本库的有界存储、曲线求值与参数表，并保留 P3D 的斜截面分量、数值容差和接缝控制行规则。路径分段还参考同一固定提交中的 [MSBsplineCurve_ByBezier.cpp](https://github.com/iTwin/imodel-native/blob/2350843ad1580a751ee24cc552460644015b07dc/iModelCore/GeomLibs/geom/src/bspline/MSBsplineCurve_ByBezier.cpp) 与 [bezierDPoint4d.cpp](https://github.com/iTwin/imodel-native/blob/2350843ad1580a751ee24cc552460644015b07dc/iModelCore/GeomLibs/geom/src/bezier/bezierDPoint4d.cpp)。折线转换参考 [BsplineCurveFit.cpp](https://github.com/iTwin/imodel-native/blob/2350843ad1580a751ee24cc552460644015b07dc/iModelCore/GeomLibs/geom/src/bspline/BsplineCurveFit.cpp) 与 [MSBsplineCurve_Modify.cpp](https://github.com/iTwin/imodel-native/blob/2350843ad1580a751ee24cc552460644015b07dc/iModelCore/GeomLibs/geom/src/bspline/MSBsplineCurve_Modify.cpp)，仅实现原生扫掠使用的一次拟合分支，保留弦长参数和累计误差节点删减规则。上述算法按本库的预算控制、不可变输入和来源报告改写，不依赖 Bentley 运行库。内部曲面生成流程不代表已经支持完整路径扫掠实体的重建。

Manifold 仅随库编译 C++ 核心，不下载依赖，也不要求额外运行时 DLL。上游源码保持原文，构建时将内部命名空间隔离为 `p3d_bundled_manifold` 和 `p3d_bundled_linalg`，避免与调用方自行链接的版本冲突；本库公开头文件不暴露其类型。当前内核使用串行后端，独立求值调用可并发执行。`LICENSE.EMBEDDED` 汇集上游头文件中的 MIT 和 Sun 声明，便于二进制分发时保留。
