# 验证说明

这些测试检查本次模式精简是否破坏接口、采集协议、性能日志及窗口启动，不验证实机轨迹精度。

在工程根目录可直接运行：

```bash
python3 -m unittest discover -s tests -p test_source_contract.py -v
python3 tests/check_live_console.py
python3 tests/check_keyframe_policy.py
```

关键帧测试中的 `oldPolicy` 是原版策略副本，保留旧模式枚举用于对照；它不参与项目算法编译。脚本每次从当前 `Tracking.cc` 读取新策略，比较 100,000 组状态的决定和 BA 中断副作用。

以下测试在已有 ARM64 Docker 编译完成后运行，接收构建产物：

- `native_gray_test.py --archive ARCHIVE`：NEON 像素、元数据行排除、边界/对齐/无效长度。
- `native_capture_test.py --archive ARCHIVE`：模拟 B 通道 V4L2，验证两种采集包格式与计时开关。
- `native_mode_test.py ARCHIVE`：链接实际包中核心库，验证非法模式在访问配置/传感器前被拒绝；需要 CMake 生成的 g2o config.h，默认读取 `/build/mono-imu-vio/work`，可用 `MONO_IMU_BUILD_ROOT` 指定构建根目录。
- `bridge_timing_test.py`：模拟串口、IMU 时钟拟合和实时图像，启动实际原生窗口并核对 CSV 关联及分析报告。
- `replay_smoke.py`：合成 60 帧封存数据，验证导出与原生窗口回放。

最后两项使用 `MONO_IMU_ARCHIVE` 指定安装包绝对路径，默认 `mono_imu_vio.tar.gz`。模拟场景是静态纹理，没有惯性初始化所需运动，预期报告“没有初始化后成功跟踪帧”；不得据此宣称惯性精度通过或初始化性能提升。
