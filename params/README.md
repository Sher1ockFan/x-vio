# 参数文件更换

Mac 源工程中的参数位于 `~/Documents/vio_ws/mono_vio_60/params/`。Jetson 使用安装目录里的 `params/`，默认是 `~/Documents/mono_vio_60/params/`。修改 Mac 文件后，需将更新的配置同步到 Jetson。

## 当前活动配置

| 文件 | 用途 | 当前值 |
| --- | --- | --- |
| `orb_trial.yaml` | ORB-SLAM3 相机模型、内参、图像尺寸、相机到 IMU 的变换及 IMU 参数 | 来自 `calibrate_nx/fit/fit_03` 的 Omni-radtan，1920×1536、60 fps，IMU 为 500 Hz |
| `timing.json` | 残余时间偏移、时钟映射约定及标定与相机配置的绑定指纹 | +0.014502237951455254 s，运行时取 +14502238 ns |
| `camera_profile.json` | 相机节点、驱动身份、像素格式、原始图像布局、帧率和时钟域 | `/dev/video0`，`jl_gmsl 10-003a`，1920×1536 UYVY，无裁剪，Tegra `frame_rate` 请求并回读 60 fps，时钟域为 `cntvct` |

`calibration_60/` 保存标定原件与联合拟合报告摘要。`fit_03` 来自新相机 60 fps 联合采集 `run_02`。独立复验已跳过，活动状态是 `calibrated_omni_candidate`。`legacy_fit05/` 保存原旧相机参数，`borrowed_fit05_trial/` 保存临时借用旧参数的三件套。

## 录入新标定

1. 给 `orb_trial.yaml` 保留 `%YAML:1.0` 和 `File.version: "1.0"`。Omni-radtan 在 Kalibr 的 `intrinsics` 顺序为 `[xi, fx, fy, cx, cy]`，分别填到 `Camera1.xi/fx/fy/cx/cy`。把 `distortion_coeffs` 中的 `[k1, k2, p1, p2]` 填到对应字段。`Camera.type` 写成 `"OmniRadtan"`。
2. 将 Kalibr 的 `T_cam_imu` 求逆后按行填入 `IMU.T_b_c1` 的 `data`，单位是米。`T_cam_imu` 将 IMU 坐标转换到相机，ORB 用的矩阵将相机坐标转换到 IMU。更新对应的 `IMU.NoiseGyro/NoiseAcc/GyroWalk/AccWalk/Frequency`。当前 FORSENSE 数据接入按 500 Hz 工作。
3. 将实际输出尺寸和请求帧率填到 `Camera.width/height/fps`，使其与 `camera_profile.json` 相符。当前全分辨率方案将 `crop_top/crop_bottom` 设为 0，省略 `Camera.newWidth/newHeight`。当前驱动需要 `"frame_rate_control": "tegra_frame_rate_microhz"` 才能打开标定时的 60 fps 模式。换设备时按实际驱动配置相机采集方式。
4. 将 Kalibr `timeshift_cam_imu` 原值写入 `timing.json` 的秒数字段，纳秒取 `round(seconds × 1e9)`。VIO 先将相机时间戳映射到本次主机的 RAW 时钟，再加一次该时偏。确认新采集模式下的时间域及偏移符号。
5. 用下面的只读命令取得新的 `camera_profile_sha256`，写进 `timing.json`。将 YAML 的 `Camera.calibrationStatus` 与 JSON 的 `calibration_status` 设为相同的值。完成整套独立验证后可使用 `validated`。候选标定与正式标定使用不同的地图文件。

## 启动前核对

在 Jetson 上运行下列只读命令，它们不打开相机或串口。

```bash
kit="$HOME/Documents/mono_vio_60"
bash "$kit/start.sh" profile
python3 "$kit/camera_config.py" validate \
  "${VIO_CAMERA_PROFILE:-$kit/params/camera_profile.json}" \
  "${VIO_SETTINGS:-$kit/params/orb_trial.yaml}" \
  "${VIO_TIMING_FILE:-$kit/params/timing.json}"
```

在 Mac 源工程里可把命令中的 `"$kit/camera_config.py"` 改为 `jetson/camera_config.py`，输入文件路径改为源工程的 `params/`。环境变量 `VIO_SETTINGS`、`VIO_TIMING_FILE`、`VIO_CAMERA_PROFILE` 可覆盖相应的活动文件。

启动时会快照当前生效的三个参数文件。回放直接读取录制时的快照。活动参数可直接替换，换模型或改 C/C++ 才需要重新编译。`maps/workspace_omni_60.osa` 是当前 Omni-radtan 候选的默认参考地图。地图文件和参数指纹绑定，启动会拒绝加载来自旧参数的参考地图。
