# 测试说明

[unit.cpp](unit.cpp) 使用合成输入覆盖数据读取、几何变换、引用关系及异常处理。

启用 `P3D_BUILD_TESTS=ON` 后构建并运行：

```sh
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

增加测试时，应使用最小输入明确表达预期行为。构建选项见[接入指南](../docs/接入指南.md)。

真实 P3D 文件保存在仓库外，可用 `tools/audit_corpus.py` 运行审计。来源、复现命令和覆盖边界见[验证与逆向进度](../docs/验证与逆向进度.md)。
