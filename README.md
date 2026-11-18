# mono_vio_60

用于 Jetson Orin NX 的单目鱼眼＋IMU VIO。默认使用 `/dev/video0` 的 `jl_gmsl` 相机，设置 60 fps，直接以 **1920×1536** 输入 ORB-SLAM3。图像灰度化后保持完整尺寸。可视化窗口的显示大小可独立于算法输入。

## 在 Jetson 启动

```bash
cd "$HOME/Documents/mono_vio_60"
bash check.sh
bash start.sh profile
DISPLAY=:0 bash start.sh live
```

前两项检查软件依赖及参数，最后一项开始采集。按一次 Ctrl+C 收尾。无窗口使用 `bash start.sh live-headless`，在另一终端用 `bash start.sh status` 查询跟踪状态。启动前用户需能访问相机和串口。`record` 保存相机和 IMU 数据，`record-view` 额外打开一个相机预览窗口；两者都按相机配置录制全分辨率画面，后续用 `replay` 建图。录制、回放和参数覆盖见 [Jetson 使用说明](jetson/README.md)。

## 相机模型与参数

| 文件 | 当前用途 |
| --- | --- |
| `params/camera_profile.json` | video0、1920×1536 UYVY、无裁剪、Tegra 帧率控制、CNTVCT 时间戳 |
| `params/orb_trial.yaml` | 联合标定 `fit_03` 的 Omni-radtan 内参、相机到 IMU 的外参及 IMU 配置 |
| `params/timing.json` | 把相机时间映射到当前主机 RAW 时钟后，一次性加上 +14.502238 ms，与 IMU 时间轴配对 |
| `params/calibration_60/` | 原始 Kalibr 参数及来源 |
| `params/legacy_fit05/` | 以前试跑参数的留档 |

活动配置显示 `calibrated_omni_candidate`。当前联合标定使用新相机的 60 fps 录制，独立复验按要求跳过。未独立验收定位精度及曝光时间戳事件。更换设备或参数时按 [参数说明](params/README.md) 配套更换活动文件。

相机模型使用 Kalibr 的统一球面投影和 radtan 畸变，初始化和后续匹配可使用模型定义域内的侧向光线。无有效逆投影的像素不参与几何约束。默认参考图为 `maps/workspace_omni_60.osa`，与历史地图分开。需要同一标定和时间配置才能复用参考图。60 fps 是采集目标，输入和 VIO 处理率可以分别从运行日志及计时文件观察。

## 在 Mac 编译打包

```bash
cd "$HOME/Documents/vio_ws/mono_vio_60"
python3 tools/update_manifest.py
ORB_CONTAINER=orin-r365-dev bash build.sh
```

使用 Ubuntu 22.04 ARM64 Docker，缓存位于容器的 `/build/mono-vio-60/`，压缩包及校验值输出至 `dist/`。目标为 Ubuntu 22.04 ARM64 / Jetson Linux R36.5，OpenCV 4.5.4 运行库随包提供。部署后可用 `bash check.sh` 检查动态链接。

| 目录 | 用途 |
| --- | --- |
| `vio_src/` | 基于官方提交 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` 的单目惯性核心和 Omni-radtan |
| `adapter/` | 硬件采样、时间同步、灰度提取和算法入口 |
| `jetson/` | Jetson 启动、录制、回放、状态和轨迹工具 |
| `docker/` | ARM64 编译、依赖检查、打包 |
| `tests/`、`provenance/` | 软件测试和源码记录 |

见 [源码来源与修改记录](provenance/README.md) 与 [GPLv3 许可证](vio_src/LICENSE)。发布二进制时须按许可证提供对应源码。
