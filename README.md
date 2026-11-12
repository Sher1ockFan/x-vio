# mono_imu_vio

从 `name_test` 精简得到的单目＋IMU VIO 工程，面向 Jetson Orin NX。保留频率/延迟测试、NEON 灰度提取、实时双窗口、录制、回放及轨迹导出。`LIVE...` 后终端保持静默。旧 `name_test`、`vio_nx` 独立保留。

这是 **ORB-SLAM3 的修改版**，基于提交 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`，不再是完整官方快照。公开传感器枚举只保留 `IMU_MONOCULAR`；删除双目、双目惯性、RGB-D、RGB-D惯性、无IMU单目的输入接口、配置读取、跟踪分支及示例。CMake 只构建核心库和 `mono_inertial_euroc`。

单目惯性路径的初始化条件、特征阈值、优化算法和标定参数不作调参。共享后端仍保留部分历史字段和优化边类型（如右目索引/深度占位），以维护 Frame、KeyFrame、Optimizer 的兼容关系；它们不是可选择的第二传感器模式。本次精简不承诺解决初始化困难或漂移。

| 目录 | 用途 |
| --- | --- |
| `ORB_SLAM3/` | 精简后的算法源码、依赖与词典；包含本次核心修改 |
| `adapter/` | 单路相机、IMU、时间映射、性能计时及算法调用 |
| `params/` | 原 `fit_05` 参数，数值不变 |
| `docker/` | ARM64 编译、打包与动态链接检查 |
| `jetson/` | 启动、性能报告、录制、回放及轨迹工具 |
| `tests/` | 模式限制、灰度与模拟输入验证 |
| `provenance/` | 上游基线清单与本次源码变更记录 |
| `dist/` | 生成的 Jetson 安装包和验证摘要（不提交 Git） |

实时和录制均使用 `/dev/gmsl/B`（驱动卡 `10-003b`、bus `:0`）。原始 1920×1441 UYVY 去掉首行元数据，NEON 提取 1920×1440 灰度；ORB 内部缩放为 960×720。IMU 921600 波特率，标定采样率 500 Hz。仍沿用 C 通道得到的 +16.881057 ms 时偏，B 通道的物理时延需实机复验。

## 在 Mac 构建

```bash
cd "$HOME/Documents/vio_ws/mono_imu_vio"
ORB_CONTAINER=orin-r365-dev bash build.sh
```

使用已有 Ubuntu 22.04 ARM64 容器及 `/build/orb-kit` 下的 NVIDIA OpenCV 4.8.0、Pangolin，编译缓存与原版项目隔离在 `/build/mono-imu-vio`，不会从原版缓存直接拿 `libORB_SLAM3.so`。源码清单校验通过后才编译；包输出为 `dist/mono_imu_vio.tar.gz`。

以后修改 `ORB_SLAM3/` 后先运行 `python3 tools/update_manifest.py` 更新当前源码清单，再构建。该命令不会修改 `provenance/upstream.sha256` 的官方基线。编译适配层无需更新核心清单。

Jetson 安装、启动、测试和回放步骤见 [使用说明](jetson/README.md)。实机速度需比较 `performance.txt` 中灰度提取中位数/P95及算法处理频率；Docker 模拟测试不能代表 Jetson 的真实提速或轨迹精度。

## 代码阅读顺序

`jetson/start.sh` → `adapter/orb_replay/live.py`（同步与批次）→ `adapter/viewer.cc`（调用 `TrackMonocular`）→ `ORB_SLAM3/src/System.cc` → `Tracking.cc` → `LocalMapping.cc` / `LoopClosing.cc` / `Optimizer.cc`。

提交 Git 可执行 `git add .`；构建目录与运行数据已忽略。保留 [ORB-SLAM3 GPLv3 许可证](ORB_SLAM3/LICENSE) 和依赖许可证，发布修改版时同时提供对应源码。
