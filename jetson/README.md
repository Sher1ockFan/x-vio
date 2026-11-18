# mono_vio_60：Jetson 使用说明

默认相机为 `/dev/video0`（ISX031 M3A），按联合标定时的模式请求 60 fps。ORB-SLAM3 使用当前相机的 Omni-radtan 参数和 1920×1536 输入图像。实际的相机采集频率和 VIO 处理频率可分别在运行日志里查看。

## 安装与检查

将 `mono_vio_60.tar.gz` 放到 Jetson 家目录，解压到独立目录：

```bash
mkdir -p "$HOME/Documents"
tar -xzf "$HOME/mono_vio_60.tar.gz" -C "$HOME/Documents"
kit="$HOME/Documents/mono_vio_60"
bash "$kit/check.sh"
```

宿主机须为 Ubuntu 22.04 ARM64 / Jetson Linux R36.5，并满足剩余系统图形库和 `python3-numpy` 依赖。本副本使用 OpenCV 4.5.4 编译；所需 OpenCV 4.5、Boost serialization、GLU 运行库及许可证随包放在私有目录。动态链接是否完整以 `check.sh` 为准，不修改宿主机 OpenCV。软件检查成功不证明采集或标定有效。`live-headless` 不创建窗口、不要求 DISPLAY，但当前程序仍链接图形运行库。

包内可阅读的核心源码位于 `source/vio_src/`，采集与算法入口源码位于 `source/adapter/`。完整可编译工程的核心源码目录为 `vio_src/`。

## 相机与新标定

```bash
kit="$HOME/Documents/mono_vio_60"
bash "$kit/start.sh" profile
# 自定义配置时使用绝对路径
export VIO_CAMERA_PROFILE="$kit/params/camera_profile.json"
```

`profile` 只读取配置并输出指纹。默认使用联合标定 `run_02` 的 1920×1536 UYVY、零裁剪模式。采集程序在打开数据流前写入并回读 Tegra `frame_rate` 的 60000000 微赫兹设置。`camera_video1.json` 需要匹配另一路相机的标定。

三个活动文件已配置：

- `params/orb_trial.yaml` 保存 `fit_03` 的 Omni-radtan 内参、相机到 IMU 的外参，以及 500 Hz IMU 噪声参数。ORB 输入和处理尺寸均为 1920×1536。
- `params/timing.json` 保存 +14.502238 ms 残余时偏。采集时先将相机时间戳映射到 RAW，再加一次时偏。
- `params/camera_profile.json` 保存设备节点、相机身份、1920×1536 零裁剪布局及 60 fps 设置，以及联合标定时选出的 `cntvct` 时钟域。

活动配置标记为 `calibrated_omni_candidate`。启动前打印该候选提示，每次运行重新估计传感器时间戳到本机的映射。本次标定已按用户要求跳过独立复验，精度仍是候选状态。

后续完成新标定时更换活动文件，字段及顺序见安装包 `params/README.md`（源码见 [参数说明](../params/README.md)）。文件在其他位置时可指定绝对路径：

```bash
export VIO_SETTINGS="/绝对路径/参数/orb_trial.yaml"
export VIO_TIMING_FILE="/绝对路径/参数/timing.json"
```

`bash "$kit/check.sh" --devices` 只检查节点与权限，不测试相机出帧或 IMU 通信。

IMU 入口使用 CP2102 的 `/dev/serial/by-id` 路径、FORSENSE 协议和 921600 波特率，数据配置为 500 Hz。

## 实时、录制与回放

以下命令会打开相机和串口。实时可选双窗口或无窗口。省略秒数或传入 `0` 会持续到 Ctrl+C。

```bash
DISPLAY=:0 bash "$kit/start.sh" live
bash "$kit/start.sh" live-headless

# 每次录制指定新的目录，两个命令任选其一
bash "$kit/start.sh" record "$kit/runs/take_01"
DISPLAY=:0 bash "$kit/start.sh" record-view "$kit/runs/take_02"

# 对已正常封存的数据回放或导出
DISPLAY=:0 bash "$kit/start.sh" replay "$kit/runs/take_01"
bash "$kit/start.sh" trajectory "$kit/runs/take_01"
```

`record` 和 `record-view` 共用相同的相机、IMU 采集和写盘程序。两者按同一个相机配置保存完整的 1920×1536 Mono8 图像，目标采集频率为 60 fps。录制按实际收到的帧保存，实际频率与丢帧情况可在 `frames.csv` 和 `camera.log` 查看。两种录制模式都打开物理相机；`record-view` 额外提供一个相机画面窗口。窗口只读取最新的完整图像，允许跳过仅用于预览的旧帧，关闭窗口后录制继续。回放时用 `replay` 跟踪和建图。

按一次 Ctrl+C 后等待收尾。录制 `capture.json` 的 `COMPLETE` 代表封存完成；异常录制可用 `recover TAKE` 检查。每次运行/录制保存相机配置、标定及时间文件快照，回放和导出使用对应快照。继承的旧录制若缺少新快照，不能直接当作新包输入。

`live` / `live-headless` 打印 `LIVE...` 后终端静默；启动错误给出分类原因和日志路径，运行详情写入 `session/console.log`、`session/live.json` 及诊断 CSV。频率与延迟分析保留在 `performance.txt`；实测采集频率、ORB 处理频率和延迟应分别查看。

## 状态与地图

另一个终端可主动查询，不打开传感器、不停止实时运行：

```bash
bash "$kit/start.sh" status
bash "$kit/start.sh" status --watch
bash "$kit/start.sh" status --json
```

状态核对进程身份、数据新鲜度和最新有效帧。`ready` / `stable_ready` 是运行状态，不是精度验收；查询退出码 0 也不表示 VIO 已就绪。

实时默认 `VIO_MAP_MODE=auto`，当前候选的参考地图路径为 `$kit/maps/workspace_omni_60.osa`。`new` 使用未占用的参考路径，`off` 关闭复用和持久化，`VIO_MAP_FILE` 可指定路径。地图绑定相机配置、相机模型及标定状态，`workspace_trial.osa` 中的旧地图无法用当前参数加载。回放独立建图。

地图加载成功后仍须视觉重定位与惯性恢复。恢复保留至少 1.2 秒拟合与 0.6 秒后续独立复验；健康检查与输出有效性策略未修改。正常结束、成熟地图及诊断检查通过后才首次发布参考图，已有参考图不自动覆盖。`session/map_save.json`、`map_session.json`、`recovery_events.csv`、`recovery_timing.csv` 和 `state_diagnostics.csv` 记录结果；保存成功不代表精度通过。

继承的 Shutdown 段错误问题尚未解决。出现非正常退出时应保留日志和返回码，不将其视为正常封存或已修复。

活动配置 `camera_profile.json`、`orb_trial.yaml`、`timing.json` 可随本次标定更新；`check.sh` 检查程序和只读模板的校验和，启动时另行检查活动配置的尺寸、时偏与绑定指纹。原始安装包完整性可通过外部 `.sha256` 文件核对。
