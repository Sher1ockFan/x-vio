# mono_imu_vio

从 `name_test` 精简得到的单目＋IMU VIO 工程，面向 Jetson Orin NX。保留频率/延迟测试、NEON 灰度提取、实时双窗口或无窗口运行、只读状态查询、录制、回放及轨迹导出。`LIVE...` 后终端保持静默。旧 `name_test`、`vio_nx` 独立保留。

这是 **ORB-SLAM3 的修改版**，基于提交 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`，不再是完整官方快照。公开传感器枚举只保留 `IMU_MONOCULAR`；删除双目、双目惯性、RGB-D、RGB-D惯性、无IMU单目的输入接口、配置读取、跟踪分支及示例。CMake 只构建核心库和 `mono_inertial_euroc`。

首次单目惯性建图保留原有初始化条件和标定参数；启用地图复用时，新增独立视觉核对、速度/零偏/位姿异常检查，以及使用后续新数据复验的恢复流程。共享后端仍保留部分历史字段和优化边类型（如右目索引/深度占位），以维护 Frame、KeyFrame、Optimizer 的兼容关系；它们不是可选择的第二传感器模式。地图复用依赖旧图的质量和当前视觉匹配；它不能保证遮挡或高速抖动下无漂移。

| 目录 | 用途 |
| --- | --- |
| `vio_src/` | 精简后的算法源码、依赖与词典；包含本次核心修改 |
| `adapter/` | 单路相机、IMU、时间映射、性能计时及算法调用 |
| `params/` | 原 `fit_05` 参数，数值不变 |
| `docker/` | ARM64 编译、打包与动态链接检查 |
| `jetson/` | 启动、性能报告、录制、回放及轨迹工具 |
| `tests/` | 模式限制、灰度、地图保存加载及 IMU 恢复验证 |
| `provenance/` | 上游基线清单与本次源码变更记录 |
| `dist/` | 生成的 Jetson 安装包和验证摘要（不提交 Git） |

实时和录制均使用 `/dev/gmsl/B`（驱动卡 `10-003b`、bus `:0`）。原始 1920×1441 UYVY 去掉首行元数据，NEON 提取 1920×1440 灰度；ORB 内部缩放为 960×720。IMU 921600 波特率，标定采样率 500 Hz。仍沿用 C 通道得到的 +16.881057 ms 时偏，B 通道的物理时延需实机复验。

## 在 Mac 构建

```bash
cd "$HOME/Documents/vio_ws/mono_imu_vio"
ORB_CONTAINER=orin-r365-dev bash build.sh
```

使用已有 Ubuntu 22.04 ARM64 容器及 `/build/orb-kit` 下的 NVIDIA OpenCV 4.8.0、Pangolin，编译缓存与原版项目隔离在 `/build/mono-imu-vio`，不会从原版缓存直接拿 `libORB_SLAM3.so`。源码清单校验通过后才编译；包输出为 `dist/mono_imu_vio.tar.gz`。

以后修改 `vio_src/` 后先运行 `python3 tools/update_manifest.py` 更新当前源码清单，再构建。该命令不会修改 `provenance/upstream.sha256` 的官方基线。编译适配层无需更新核心清单。

Jetson 安装、启动、测试和回放步骤见 [使用说明](jetson/README.md)。实机速度需比较 `performance.txt` 中灰度提取中位数/P95及算法处理频率；Docker 模拟测试不能代表 Jetson 的真实提速或轨迹精度。

## 代码阅读顺序

`jetson/start.sh` → `adapter/orb_replay/live.py`（同步与批次）→ `adapter/viewer.cc`（调用 `TrackMonocular`）→ `vio_src/src/System.cc` → `Tracking.cc` → `LocalMapping.cc` / `LoopClosing.cc` / `Optimizer.cc`。

提交 Git 可执行 `git add .`；构建目录与运行数据已忽略。保留 [ORB-SLAM3 GPLv3 许可证](vio_src/LICENSE) 和依赖许可证，发布修改版时同时提供对应源码。

## 旧地图优先与持久化

实时入口默认启用地图复用。没有参考图时正常建图；退出时只保存已完成惯性初始化和 BA2、至少 12 个关键帧及 50 个地图点的地图。第一次正常结束且诊断检查通过后，地图发布为安装目录的 `maps/workspace_v2.osa`；下次启动自动加载。每次运行的 `session/atlas.osa` 独立保存，旧参考图不会自动替换。配套 `.osa.meta.json` 一起保留。地图格式属于此修改版，不能假定与官方 `.osa` 互换。

本次格式为 `MONO_IMU_ATLAS_2`，保存恢复根关键帧的动态先验及其不确定性。旧版 `maps/workspace.osa` 留在原处用于诊断，不自动迁移或加载。显式指定旧格式会被拒绝；请重新建立 v2 参考图，不要通过改名或编辑清单绕过检查。

失跟后先在成熟旧图检索与几何验证；找到后优先连续跟踪最近验证帧和局部地图，局部失败才在同一帧回到全局搜索。局部和全局共享至少 50 个唯一地图点及 4 个区域的验证门槛。再用连续新图像和新 IMU 样本求解当前速度、陀螺仪零偏及加速度计零偏，并用未参与拟合的后续数据复验。视觉找回不等于惯性恢复；恢复未通过时位姿标记无效。恢复成功后建立新的关键帧时间链，不连接上次会话的 IMU 积分。恢复根节点的速度与零偏作为有不确定性的先验进入局部优化，而非固定真值。地图检索和恢复失败会按超时策略回退，新旧地图的坐标系不能直接混成同一条连续轨迹。

普通跟踪也持续检查独立视觉与融合位姿的一致性、数值有限性、速度/零偏和位姿突跳。静止候选需要同时有图像特征低运动和 IMU 证据；程序不根据 IMU 平稳就断言零速度，也不把异常位姿裁切到 1.8 米工作区或冻结成“准确轨迹”。不通过检查的帧不发布数值位姿、不作为新关键帧插入，转入恢复。严重数值异常会使本次参考发布为 `HEALTH_REVIEW_REQUIRED`；诊断缺失或损坏同样不会自动发布。保存会话地图用于排查不代表精度通过。

已移除普通跟踪中“视觉或融合重投影 RMS 超过 8 像素就拒绝”的门槛。有限的 `visual_rms_px` / `fused_rms_px` 继续记录，不再因其数值大小单独触发恢复；NaN/Inf 检查、视觉支持、视觉/融合位姿差、速度/零偏、位姿突跳和静止冲突检查仍保留。旧地图恢复中的 IMU 拟合与独立未来数据复验未修改。本次改动不包含此前实机报告的 Shutdown 段错误修复。

静止证据已确认且图像/陀螺仪仍支持时，对短时加速度统计瞬变最多保持 0.5 秒，逐帧记录 `stationary_evidence_held`。可观测运动立即释放保持，不根据这项机制锁定位姿或强制零速度。

另保留同一有效段内不可回写的上次已接受输出，检查 `output_pose_step_m`，避免后端同时修正地图和内部上一帧状态而掩盖对外输出跳变。`map_updated` 时的输出不连续单独记录，不能直接解释为设备真实运动；不跨新图、恢复或失跟间隔计算该步长。

`VIO_MAP_MODE=auto|new|off` 控制实时入口，`VIO_MAP_FILE=/绝对路径/文件.osa` 选择参考图。`new` 需要未占用路径，`off` 关闭复用与持久化。录制/回放默认隔离，不使用日常实时参考图。详见 [Jetson 操作说明](jetson/README.md)。

状态与诊断写入运行目录：`map_plan.json`、`session/map_save.json`、`session/map_session.json`、`session/recovery_events.csv`、`session/poses.csv` 和逐帧 `session/state_diagnostics.csv`。新诊断包含世界系 IMU 主体速度、IMU 坐标系零偏、视觉/融合残差、恢复拟合与复验结果以及异常原因；未知值为 `nan`。保留原有频率/延迟 CSV 与 `performance.py`。`LIVE...` 后这些消息都只入日志，终端静默。

启动错误现在保留分类代码和简短原因（路径、权限、显示、设备、地图校验等），并给出日志路径。LIVE 后终端仍完全静默；详见 [Jetson 使用说明](jetson/README.md)。

## 无窗口与状态

Jetson 使用 `bash "$HOME/Documents/mono_imu_vio/start.sh" live-headless` 持续运行到 Ctrl+C，无需 DISPLAY；`live` 保留原双窗口。两种模式都保留频率/延迟测试、地图加载保存、独立 IMU 复验及 LIVE 后静默。另一个终端执行 `bash "$HOME/Documents/mono_imu_vio/start.sh" status --watch` 主动查看当前状态，或 `status --json` 给其他程序读取。状态查询会核对进程身份和数据新鲜度，不把历史有效帧误报为当前正常；详细语义见 Jetson 使用说明。

恢复窗口仍至少要求 1.2 秒拟合和 0.6 秒后续独立复验，不强行沿用旧速度/零偏。新增 `recovery_timing.csv` 记录连续跟踪/全局回退和搜索耗时，供实机对比。Atlas v2 序列化格式及标定未改变，已有通过校验的 v2 参考地图可继续使用。
