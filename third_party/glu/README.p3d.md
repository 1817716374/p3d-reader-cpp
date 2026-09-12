# GLU 三角化依赖

固定来源为 Chromium 保存的 SGI GLU 三角化快照：
[82532b9046be34a2ca93b650c5808a0827ecff16](https://chromium.googlesource.com/external/skia/third_party/glu/+/82532b9046be34a2ca93b650c5808a0827ecff16/)。上游说明保留在 `README.skia`，各文件来源和校验值见上一级 `manifest.json`。

这里只编译多边形三角化部分，不依赖系统 OpenGL、GLU 动态库或绘图上下文。`priorityq-heap.c` 由 `priorityq.c` 包含，不单独编译。

本工程通过 `p3d_prefix.h` 为公开函数和内部链接符号增加私有前缀；`gluos.h`、`memalloc.h` 增加该头文件引用。文件统一为 LF 行尾并移除尾随空白，算法保持上游版本。清单同时记录上游原始文件和当前文件的 SHA-256。SGI 和 Google 的许可证分别保留在 `LICENSE.txt`、`LICENSE.GOOGLE` 及源文件版权头中。
