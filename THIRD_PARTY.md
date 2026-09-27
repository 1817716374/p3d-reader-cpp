# 第三方依赖说明

`src/native_vu_exterior.cpp` 的面积初筛、候选面选择和边界奇偶遍历参考下表固定 Bentley 提交中的 `vumerge.cpp`、`vusubset.cpp`。保留 Bentley 版权及 Apache-2.0 标记，实现采用 P3D 的零面积阈值、临时数组读取位置及遍历顺序；以局部有界工作栈和事务式掩码更新实现，不依赖厂商运行库。

`src/native_vu_regularize.cpp` 的双向面规则化与扇区判定参考下表固定 Bentley 提交中的 `vureg.cpp`、`vucoord.cpp`，连接边插入同时参考 `vu.cpp`。保留 Bentley 版权及 Apache-2.0 标记；实现按 P3D 的极值排序、活动链删除、端点插值和掩码返回顺序改写。排序状态与工作数组均为调用局部数据，不依赖厂商运行库。

`src/native_vu_connections.cpp` 的簇边收集与顶点连接参考下表固定 Bentley 提交中的 `vucluster.cpp`；`src/native_vu_graph.cpp` 的标记边删除与空闲节点复用同时参考 `vumem.cpp`。保留 Bentley 版权及 Apache-2.0 标记，并按 P3D 的重复边模式、双次排序、掩码和链表顺序实现。节点存储槽位复用不代表源几何去重。

`src/native_vu_intersections.cpp` 的横向交点扫描及缩放二维方程求解参考下表固定 Bentley 提交中的 `vumerge2.cpp`、`bsibasegeom_for_vu_static.c`，保留 Bentley 版权及 Apache-2.0 标记。实现保留 P3D 的退化阈值、运算顺序、边范围及分组顺序，与近顶点投影共用原生边分裂内核；交点分裂不等于顶点连接或完整面生成。

`src/native_vu_near_vertices.cpp` 的近顶点边分裂参考下表固定 Bentley 提交中的 `vumerge2.cpp`，保留 Bentley 版权及 Apache-2.0 标记。实现保留原生候选范围、投影阈值、分数组合、插值及节点顺序；不把投影分裂视为完整交点连接。

`src/native_vu_cluster.cpp` 的坐标分组参考下表固定 Bentley 提交中的 `vucluster.cpp`，保留 Bentley 版权及 Apache-2.0 标记。实现按 P3D 的严格平面距离、排序交换、访问顺序及整组 XYZ 回写规则处理，并复用本库已确认的原生排序内核；不把坐标分组视为完整拓扑合并。

`src/native_vu_graph.cpp` 的索引环构造、边分裂与顶点重接参考下表固定 Bentley 提交中的 `VuOps.cpp`、`vu.cpp`，保留 Bentley 版权及 Apache-2.0 标记。实现采用有界节点索引存储，保留 P3D 的节点遍历顺序、掩码及源角点编号，不依赖厂商分配器或运行库。

`src/native_polygon_projection.cpp` 的多边形法向及三点环中点分支参考下表固定 Bentley 提交中的 `PolygonOps.cpp`、`polygon3d.cpp`，保留 Bentley 版权及 Apache-2.0 标记。坐标系采用 P3D 的首环、首个合格边和退化回退规则；未采用较新参考实现的秩分类和直线回退。投影准备不包含原生平面图三角化。

`src/native_tube_mesh_index_rules.cpp` 的四边形对角线与带符号三角索引选择参考下表固定 Bentley 提交中的 `Polyface.cpp`，保留 Bentley 版权及 Apache-2.0 标记。实现按 P3D 的点积算序和低边数保留规则改写，独立处理原生首末列的边可见性；未采用较新参考实现的平面性筛选和可选面过滤器。

本项目原创解析代码采用 [MIT 许可证](LICENSE)。以下依赖继续适用各自的许可证；许可证正文和版权声明保留原文。

`src/native_curve_sampling.cpp` 的节点压缩、不连续点查询和独立 Bézier 分段参考下表固定 Bentley 提交中的 `MSBsplineCurve.cpp`、`MSBsplineCurve_ByBezier.cpp` 和 `bsputil.cpp`，保留 Bentley 版权及 Apache-2.0 标记。实现按 P3D 的单侧端点、切线容差、齐次控制多边形距离、近单位权重处理及非单位节点域换算规则改写，使用有界独立工作存储。

`src/native_bezier_roots.cpp`、`src/native_curve_plane.cpp`、`src/native_curve_closest.cpp` 与 `src/native_curve_range.cpp` 参考下表固定 Bentley 几何提交中的 `bezroot.cpp`、`bezeval.cpp`、`quadeqn.cpp`、`bezierDPoint4d.cpp` 和 `MSBsplineCurve_ByBezier.cpp`，保留 Bentley 版权及 Apache-2.0 标记。实现按 P3D 的多项式乘积、非解析求根分支、数值容差、端点追加顺序、最近点比较、齐次投影、极值范围和原始节点参数换算改写；使用有界工作存储及不可变输入，不依赖厂商运行库。

`src/loft_open.cpp` 的周期曲线打开与循环节点插入参考下表固定提交的 `bspcurv.cpp`、`bsputil.cpp`，保留 Bentley 版权及 Apache-2.0 标记。实现按原始节点参数处理接缝，保留特殊周期曲线的控制点偏移、重复端点去除及原生节点归一化规则，使用本库有界工作存储。

`src/native_curve_area.cpp` 的原始曲线组面积参考同一固定提交的 `cv_properties.cpp`，保留 Bentley 版权及 Apache-2.0 标记。实现按 P3D 的组类型分派、原始曲线访问、面积矩算序及最大子区域法向规则改写，并共享本库的逐区间 B 样条积分内核。

| 组件 | 固定版本 / 来源 | 用途 | 许可证 |
| --- | --- | --- | --- |
| nlohmann/json | 3.12.0 | 属性树和 JSON 接口 | [MIT](third_party/nlohmann/LICENSE.MIT) |
| LZ4 | 1.10.0 | P3D 流和属性解压 | [许可证](third_party/lz4/LICENSE) |
| pugixml | 1.15 | 材质、模式及关系 XML | [许可证](third_party/pugixml/LICENSE.md) |
| mapbox/earcut.hpp | 2.2.4 | 多边形及带孔多边形三角化 | [许可证](third_party/mapbox/LICENSE) |
| SGI GLU / Chromium 快照 | `82532b9046be34a2ca93b650c5808a0827ecff16` | 原生网格消费路径的多边形三角化 | [SGI FreeB 2.0](third_party/glu/LICENSE.txt)、[Google BSD](third_party/glu/LICENSE.GOOGLE) |
| miniz | 3.1.0，仅链接 tinfl | zlib 属性载荷解压 | [许可证](third_party/miniz/LICENSE) |
| Bentley BGFB 模式 | 模式快照，来源见 `src/data.cpp` | BGFB 结构解码 | [Apache-2.0](third_party/BGFB_LICENSE.txt) |
| Bentley 几何算法 | `2350843ad1580a751ee24cc552460644015b07dc` 的 [bspdsurf.cpp](https://github.com/iTwin/imodel-native/blob/2350843ad1580a751ee24cc552460644015b07dc/iModelCore/GeomLibs/geom/src/bspline/bspdsurf.cpp)、[bsputil.cpp](https://github.com/iTwin/imodel-native/blob/2350843ad1580a751ee24cc552460644015b07dc/iModelCore/GeomLibs/geom/src/bspline/bsputil.cpp)、[bspcurv.cpp](https://github.com/iTwin/imodel-native/blob/2350843ad1580a751ee24cc552460644015b07dc/iModelCore/GeomLibs/geom/src/bspline/bspcurv.cpp) | `src/native_tube.cpp` 的扫掠曲面片、截面朝向、控制线求交及曲面片拼接算法基础，以及 `src/native_curve_segment.cpp` 的子曲线截取与节点插入，按 P3D 行为改写 | [Apache-2.0](third_party/BENTLEY_GEOMETRY_LICENSE.md) |
| GDAL DGN 默认颜色表 | v3.10.0 | 默认索引颜色 | [MIT 与原版权声明](third_party/GDAL_PALETTE_LICENSE.txt) |
| Manifold | `213b6557f28fd66f0e7fd44deb8531589fd288af`，3.5.1 源码快照 | 独立三角网格布尔求值 | [Apache-2.0](third_party/manifold/LICENSE)、[嵌入声明](third_party/manifold/LICENSE.EMBEDDED)、[NVIDIA BSD](third_party/manifold/LICENSE.NVIDIA)、[Clipper2 Boost](third_party/manifold/LICENSE.CLIPPER2) |

依赖文件来源、字节数与 SHA-256 校验值见 [manifest.json](third_party/manifest.json)。BGFB 模式的源地址和快照校验值保存在 `src/data.cpp`。`miniz_export.h` 是本工程静态构建的导出宏配置；构建配置禁用未链接的 zlib 包装 API。

使用 MinGW 构建的检查工具静态链接 GCC 运行时。GCC 16.1.0 的 [GPLv3](third_party/gcc-runtime/COPYING3)、[运行库例外](third_party/gcc-runtime/COPYING.RUNTIME)、[MinGW-w64 声明](third_party/gcc-runtime/MINGW_COPYING)及 [winpthreads 声明](third_party/gcc-runtime/WINPTHREADS_COPYING)随工程保留。源码构建不要求使用预编译检查工具。

依赖的许可证正文和版权声明应随对应源码或二进制分发一并保留。

`src/native_curve_planarity.cpp` 参考上述固定 Bentley 提交中的 `cp_line.cpp`、`cp_linestring.cpp`、`cp_arc.cpp`、`CurvePrimitiveBsplineCurve.cpp`、`CurveVector.cpp` 和 `cv_ops.cpp` 的范围与平面性查询流程，保留 Bentley 版权及 Apache-2.0 标记。实现按 P3D 的原始成员选择、变换算序、椭圆极值角度判据及局部范围容差改写，复用本库参考系与 B 样条范围实现，不依赖厂商运行库。

`src/native_bezier.cpp` 参考上述 Bentley 固定提交中的 `bezeval.cpp`、`bezierDPoint4d.cpp`、`cv_properties.cpp` 和 `BSIQuadrature.cpp`，保留 Bentley 版权及 Apache-2.0 标记。实现包含齐次 Bézier 求值、边数估计和面积矩积分，并按 P3D 的权重阈值、退化向量、细分及积分算序改写；使用有界工作计数和独立存储，不依赖 Bentley 运行库。

`src/native_xy_newton.cpp` 参考同一固定提交中的 `newton.cpp`，保留 Bentley 版权及 Apache-2.0 标记。实现按 P3D 的独立对角项回退、原始参数重新起算、返回值与写回判定、固定迭代设置及有理 XY 求值规则改写，使用有界工作计数和独立迭代状态。

`src/native_curve_xy.cpp` 参考同一固定提交中的 `bezierDPoint4d.cpp`、`polyline3d.cpp` 和 `MSBsplineCurve_ByBezier.cpp`，保留 Bentley 版权及 Apache-2.0 标记。实现按 P3D 的固定采样、折线候选顺序与数量上限、参数接受范围及原始曲线节点换算改写；不引入较新参考实现中的重合区间处理。

`src/native_bezier_support.hpp` 与 `src/native_bspline_area.cpp` 参考同一固定提交中的 `MSBsplineCurve_ByBezier.cpp`、`bezierDPoint4d.cpp` 和 `cv_properties.cpp`，保留上述版权及许可标记。实现共享局部支撑提取和节点插入，以独立段流式计算完整 B 样条的面积矢量与形心矩；按 P3D 的节点容差、源端点求值和累加顺序改写，源数据保持只读。

`src/native_pcurve_points.cpp` 中的 `native_iso_v_curve` 参考同一固定提交的 `bspconv.cpp` 中 `MSBsplineSurface::GetIsoVCurve`，并在函数处保留 Bentley 版权及 Apache-2.0 标记。实现按 P3D 的完整节点域求值、原始权重返回及重新加权规则改写；共享固定参数的只读基函数，直接读取带步长的控制点，避免为每列复制临时曲线。

`src/native_knot_normalize.hpp` 与 `src/native_tube_orientation.cpp` 中的曲面反向参考同一固定提交的 `bsputil.cpp` 和 `bspsurf.cpp`，保留 Bentley 版权及 Apache-2.0 标记。实现按 P3D 的逐项除法、节点阈值及闭合延伸节点规则改写；输入保持只读，显式报告归一化失败及多环处理的提前返回状态。

`src/native_pcurve_points.cpp` 的基函数导数递推参考同一固定提交中 `bsputil.cpp` 的 `bsputil_knotToBlendingFuncs`，保留 Bentley 版权及 Apache-2.0 标记。实现按 P3D 的节点选择、零分母、除法及乘加顺序改写，并以独立工作数组提供点与切线查询，不对源控制点执行除权重和重新加权。

同文件的 `native_surface_sample` 参考同一固定提交 `bspsurf.cpp` 的 `bspsurf_evaluateSurfacePoint`，保留 Bentley 版权及 Apache-2.0 标记。实现使用 P3D 的固定上界参数换算、原节点域一阶导数及 U 外层/V 内层累加顺序，采用只读数组并显式拒绝无效索引和非有限结果。

`src/native_tube_mesh_edges.cpp` 的矩形网格索引构造参考同一固定提交 `BlockedVector.cpp` 的 `AddTerminatedGridBlocks`，保留 Bentley 版权与 Apache-2.0 标记。实现限定为扫掠矩形条带通路，按 P3D 的原面次序、完整坐标行和独立属性数组处理，并加入存储预算与失败回滚；共享边对应另按原生调用顺序实现。

`src/native_tube_elevate.cpp` 的升阶算法参考上述固定提交的 `bspcurv.cpp` 与 `bsputil.cpp`，保留 Bentley 版权及 Apache-2.0 标记。实现复用本库求导与周期打开内核，按 P3D 的节点归一化、节点分组、指定侧求导和权重舍入行为改写，并使用有界的局部工作数组。

`src/native_tube_surface_elevate.cpp` 的曲面升阶参考同一固定提交的 `bspmisc.cpp`，保留 Bentley 版权及 Apache-2.0 标记。实现按第一列建立共享节点方案，保留各列独立的控制点工作状态，使用只读输入、有界局部存储及本库求导内核。

`src/native_tube.cpp`、`src/native_tube_path.cpp` 与 `src/native_tube_fit.cpp` 保留 Bentley 版权及 Apache-2.0 标记，使用本库的有界存储、曲线求值与参数表，并保留 P3D 的斜截面分量、数值容差和接缝控制行规则。路径分段还参考同一固定提交中的 [MSBsplineCurve_ByBezier.cpp](https://github.com/iTwin/imodel-native/blob/2350843ad1580a751ee24cc552460644015b07dc/iModelCore/GeomLibs/geom/src/bspline/MSBsplineCurve_ByBezier.cpp) 与 [bezierDPoint4d.cpp](https://github.com/iTwin/imodel-native/blob/2350843ad1580a751ee24cc552460644015b07dc/iModelCore/GeomLibs/geom/src/bezier/bezierDPoint4d.cpp)。折线转换参考 [BsplineCurveFit.cpp](https://github.com/iTwin/imodel-native/blob/2350843ad1580a751ee24cc552460644015b07dc/iModelCore/GeomLibs/geom/src/bspline/BsplineCurveFit.cpp) 与 [MSBsplineCurve_Modify.cpp](https://github.com/iTwin/imodel-native/blob/2350843ad1580a751ee24cc552460644015b07dc/iModelCore/GeomLibs/geom/src/bspline/MSBsplineCurve_Modify.cpp)，仅实现原生扫掠使用的一次拟合分支，保留弦长参数和累计误差节点删减规则。上述算法按本库的预算控制、不可变输入和来源报告改写，不依赖 Bentley 运行库。内部曲面生成流程不代表已经支持完整路径扫掠实体的重建。

Manifold 仅随库编译 C++ 核心，不下载依赖，也不要求额外运行时 DLL。上游源码保持原文，构建时将内部命名空间隔离为 `p3d_bundled_manifold` 和 `p3d_bundled_linalg`，避免与调用方自行链接的版本冲突；本库公开头文件不暴露其类型。当前内核使用串行后端，独立求值调用可并发执行。`LICENSE.EMBEDDED` 汇集上游头文件中的 MIT 和 Sun 声明，便于二进制分发时保留。
`src/native_surface_plane.cpp` 参考上述固定提交中的 `DPoint3dOps.cpp`、`eigensys3d.cpp` 和 `refdmatrix4d.cpp`，保留 Bentley 版权与 Apache-2.0 标记。实现采用本库的有界存储与工作预算，按 P3D 的惯性积、Jacobi 迭代、严格主轴排序和实际点范围规则改写；未采用较新参考实现的范围包围盒转换和附加正交检查。
