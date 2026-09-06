# 测试说明

[unit.cpp](unit.cpp) 使用合成输入覆盖数据读取、几何变换、引用关系及异常处理。

启用 `P3D_BUILD_TESTS=ON` 后构建并运行：

```sh
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

增加测试时，应使用最小输入明确表达预期行为。构建选项见[接入指南](../docs/接入指南.md)。
