# mono_imu_vio

单目惯性定位、旧地图重定位及输入频率/延迟测量包。部署到 `~/Documents/mono_imu_vio`，不覆盖现有 `vio_nx` 或 `name_test`。实时使用 B 口和原标定参数，可选择两个原生窗口或真正无窗口运行；两种模式在 `LIVE...` 后终端都保持静默。

适用于 Jetson Orin NX 的仅支持单目＋IMU 的 ORB-SLAM3 修改版。Mac 端在 ARM64 Docker 中编译，Jetson 解压后在宿主机运行。核心精简自上游提交 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`。窗口是原生地图与当前图像。

原分辨率配置来自在 C 通道复核过的 `fit_05` 标定。相机采集 1920×1441 UYVY，跳过首行元数据后得到 1920×1440 灰度图，ORB 内部缩放到 960×720。IMU 为 500 Hz，实时采集在相机时间映射后应用一次 **+16,881,057 ns**，回放沿用录制时已经同步的时间戳。原分辨率配置在 `params/orb_trial.yaml` 和 `params/timing.json`。

本版灰度提取使用 ARM NEON，每次从 UYVY 中提取 16 个原始 Y 值；相机采集程序使用 `-O3 -march=armv8-a` 编译。仍跳过首行元数据、输出 1920×1440 灰度图，再由 ORB 内部缩放到 960×720。相机像素、时间戳与标定参数不变；核心只保留单目惯性输入路径。

优化前后应在 Jetson 上用相近场景和运行负载对比 `performance.txt` 中的灰度提取中位数/P95、采集帧率和图像到跟踪返回延迟。Docker 验证像素一致和程序可运行，不将 Mac 容器耗时当作 Jetson 提速结果。

## 安装与检查

把 `mono_imu_vio.tar.gz` 上传到 Jetson 用户家目录，在 Jetson 桌面终端执行。

```bash
mkdir -p "$HOME/Documents"
tar -xzf "$HOME/mono_imu_vio.tar.gz" -C "$HOME/Documents"
kit="$HOME/Documents/mono_imu_vio"
bash "$kit/check.sh"
```

看到 `ORB_HUB_CHECK_OK` 后可以启动。包内带词典和项目运行库。宿主机需保留 Ubuntu 22.04 ARM64、Jetson Linux R36.5.0、NVIDIA OpenCV 4.8 `.so.408` 运行库、图形运行库和 `python3-numpy`。原生图形窗口使用桌面 X11 会话。`live-headless` 不创建 Viewer、不需要 DISPLAY 或 Xvfb，可通过 SSH 启动；当前可执行文件仍链接相同的图形运行库，因此这些运行库仍需安装。

本次修复版使用 `MONO_IMU_ATLAS_2` 地图格式，默认参考路径改为 `maps/workspace_v2.osa`。旧版 `maps/workspace.osa` 及运行记录会保留，不自动迁移、覆盖或加载。若你之前设置过 `VIO_MAP_FILE`，应改为新的未占用路径；显式指定旧格式会在启动日志中报 `MAP_E_FORMAT`。不要把旧文件直接改名为 v2 文件，也不要编辑校验清单绕过检查。

实时入口使用 `/dev/gmsl/B`。实时与录制入口统一使用同一个 B 通道。IMU 使用原 CP2102 `/dev/serial/by-id` 设备链接和 921600 波特率。账号需要能读写相机和读取串口。若缺少权限，用 `ls -lL /dev/gmsl/B /dev/serial/by-id/*` 查看设备和用户组，确认后加入对应的 `video` 或 `dialout` 组，再重新登录。`--devices` 检查 B 通道和 IMU 的设备节点及权限。

## 无窗口运行与主动状态查询

在终端一运行 `bash "$kit/start.sh" live-headless`，另开终端二查询：

```bash
kit="$HOME/Documents/mono_imu_vio"
bash "$kit/start.sh" status
bash "$kit/start.sh" status --watch
# 指定历史运行或会话目录；自动化读取 JSON
bash "$kit/start.sh" status "$kit/runs/live_xxxxxx/session" --json
```

不指定目录时查询最近一次启动；`--watch` 每秒刷新，并跟随新的运行目录。按 Ctrl+C 仅停止查询，不停止终端一的 VIO。查询不访问相机/IMU，不写运行文件，不要求图形桌面；频率/延迟仍由原来的 `performance.py` 在运行结束后整理。

`当前可用: YES` 要求监督进程身份与当前系统启动周期吻合、位姿记录和图像时间仍新鲜、最新完整帧为 `TRACKING` 且视觉/IMU/位姿均有效。`连续有效 ≥2 秒: YES` 额外要求同一地图/恢复段连续有效至少两秒。这些只是运行状态检查，不是精度验收。查询退出码 0 表示查询成功，自动化应读取 JSON 的 `ready` / `stable_ready`，不能把退出码当作 VIO 已就绪。

`STOPPED`、`STALE`、`UNKNOWN` 不会显示当前可用。文件超过两秒未更新、PID 被复用、历史日志复制到其他主机等都会与“正在正常输出”区分。启动尚未创建会话时会显示启动日志位置，无法证实时保留 UNKNOWN。地图已加载/已保存与当前跟踪就绪分别展示；`Map load: SUCCESS` 不会被当作 IMU 已恢复。

查询还显示当前阶段、帧号、地图 ID、有效连续时长、恢复方式、搜索耗时、窗口重置次数和异常原因。运行元数据在 `session/runtime.json`，恢复搜索诊断在 `session/recovery_timing.csv`。已有 `poses.csv`、`state_diagnostics.csv`、`tracking_timing.csv` 等表头保持不变。

## 实时和录制

关闭其他占用相机或 IMU 串口的程序。实时预览默认将日志放入 `$kit/runs/live_*`。录制时每次指定新的目录。

```bash
# B 口原分辨率实时查看。省略秒数或写 0 都运行到 Ctrl+C
DISPLAY=:0 bash "$kit/start.sh" live

# 或无窗口实时运行；不需要 DISPLAY，使用相同标定、地图及计时记录
bash "$kit/start.sh" live-headless

# B 口录制原始图像和 IMU，任选其一
take="$kit/runs/take_01"
bash "$kit/start.sh" record "$take"
# 或者另用一个新目录，同时开启两个原生预览窗口
bash "$kit/start.sh" record-view "$kit/runs/take_02"
```

录制时按一次 Ctrl+C，等终端返回。`capture.json` 中的 `COMPLETE` 表示录制封存完成。按 30 fps，每分钟原始录制约占 5 GB，离线转换还会另存 PNG 图像。实时预览与正式录制的用途不同，评估逐帧轨迹时使用封存数据回放。

实时入口校验 B 通道的驱动卡名 `pnd_gmsl_mc 10-003b` 和采集总线 `platform:tegra-capture-vi:0`。实时程序打开 IMU 串口后先读过前 0.5 秒的启动数据，随后校验连续数据并采集约 10 秒估计时钟映射。B 通道暂用 C 通道标定的时间偏移，时延需实机复验。

实时入口打印 `LIVE: move the rig with both translation and rotation in a textured scene.` 后不再输出运行进度、报错详情或收尾提示。`live` 模式的两个原生窗口仍运行，`live-headless` 不创建窗口；按一次 Ctrl+C 后等待终端返回。启动失败显示“分类错误码＋简短原因”，下一行给出 `Log:` 日志路径；例如 `VIO_E_DISPLAY` 提示 DISPLAY 未设置/无法访问，`VIO_E_PATH` 指出缺失路径，`VIO_E_PERMISSION` 指出权限问题。地图校验保留原有 `MAP_E_FORMAT`、`MAP_E_CALIBRATION` 等具体代码，未分类错误仍显示 `VIO_E_STARTUP` 及原始简短原因。详情保存在 `runs/startup_*.log`，已创建会话时保存在 `session/console.log`。运行中异常保留非零退出状态，详情查 `session/live.json` 和 `session/console.log`。终端自身显示的 `^C` 或命令提示符不属于程序输出。

启动命令的环境变量是 `DISPLAY`，不是 `DSIPLAY`。例如：

```bash
kit="$HOME/Documents/mono_imu_vio"
VIO_MAP_MODE=auto VIO_MAP_FILE="$kit/maps/workspace_v2.osa" \
DISPLAY=:0 bash "$kit/start.sh" live
```

| 启动错误码 | 检查方向 |
|---|---|
| `VIO_E_DISPLAY` | DISPLAY 拼写、图形桌面与 X11 权限 |
| `VIO_E_PATH` / `VIO_E_CONFIG` | 报错中的文件路径、执行权限、配置文件 |
| `VIO_E_PERMISSION` / `VIO_E_FILESYSTEM` | 串口/相机/输出目录权限、磁盘空间 |
| `VIO_E_CAMERA` / `VIO_E_IMU` | 相机驱动、串口数据、CRC 或时钟异常 |
| `VIO_E_DEPENDENCY` / `VIO_E_PLATFORM` | Python/动态库依赖、是否在 Jetson ARM64 上运行 |
| `MAP_E_*` | 保留地图校验/加载的原始原因，不折叠成统一代码 |
| `VIO_E_ARGUMENT` / `VIO_E_ORB` / `VIO_E_STARTUP` | 启动参数、ORB 后端或未分类错误；按简短原因和 Log 路径查看 |

启动时会明确打印旧地图状态：

- `Map load: LOADING ...`：参考文件校验通过，准备由 ORB 核心加载；尚不代表成功。
- `Map load: SUCCESS id=... keyframes=...`：核心已实际反序列化并选中成熟旧地图。**当前相机位姿、速度和零偏仍需重定位及 IMU 状态恢复**，不等于已恢复跟踪。
- `Map load: NO_REFERENCE ...`：没有参考地图，本次从新地图开始。
- `Map load: DISABLED ...`：`VIO_MAP_MODE=off`，不加载旧地图。
- `MAP_E_*: 地图准备或加载失败；...`：保留具体失败原因和日志路径，不打印成功。

成功提示直接来自核心的 `MAP_REFERENCE_LOADED` 记录，不以文件存在或 `ORB_LIVE_READY` 作为加载成功依据。以上提示只在 LIVE 之前显示，LIVE 后仍保持终端静默。

`auto` 模式下参考地图文件不存在时会新建会话，并不把“尚无地图”误报为路径错误。程序不会自动更改显示环境、设备权限或删除地图。

`session/console.log` 保留原有进度、`sent` 和 `superseded`，后二者分别为完整写入 ORB 管道的帧数和被新帧替代的帧数；完整写入不等于 ORB 已经完成跟踪。`session/camera.log` 继续保留采集日志。

## 旧地图优先与保存加载

`live` 默认启用旧地图优先重定位。第一次没有参考地图时正常建立地图；在光照充足、有稳定纹理的工作区做平缓平移和转动，使 IMU 初始化和地图优化完成。按一次 Ctrl+C，等命令返回，原生线程退出后才会检查保存结果。只有核心认为成熟、保存成功、本次采集正常结束且状态诊断检查通过的地图，才会首次发布为 `$kit/maps/workspace_v2.osa`；配套 `.osa.meta.json` 记录标定、时间基准、文件校验及发布前的诊断结论。

请用 Ctrl+C 正常结束采集。直接关闭原生窗口会结束跟踪并收尾保存，但实时采集将记为中止，因此不会首次发布新的参考地图。

```bash
kit="$HOME/Documents/mono_imu_vio"
# 第一次建立参考地图；以后同一命令会自动加载它
DISPLAY=:0 bash "$kit/start.sh" live 0

# 退出后查看保存/发布结果，终端不会自动打印这些内容
d=$(ls -td "$kit"/runs/live_*/session | head -n 1)
cat "$d/map_save.json"
cat "$d/map_session.json"
```

`map_save.json` 的 `SAVED` 表示本次 `session/atlas.osa` 写入成功；`SKIPPED` 表示没有达到保存条件。当前成熟条件是 IMU 已初始化、完成第二阶段惯性优化（BA2）、至少 12 个关键帧和 50 个地图点；它是复用门槛，不是精度证书。`map_session.json` 的 `REFERENCE_PUBLISHED` 表示已首次发布参考地图，`EXISTING_REFERENCE_PRESERVED` 表示保留原有参考地图，本次结果仍单独保存在会话中。`NO_MATURE_MAP` 表示需要继续完成可靠初始化/建图；`NOT_PUBLISHED` 表示采集未正常结束。

`HEALTH_REVIEW_REQUIRED` 表示本次有严重数值/视觉惯性不一致事件，或逐帧诊断缺失、损坏、没有健康有效惯性帧，因此没有自动发布参考图。即使后来重新跟踪成功，也不自动清除本次严重异常记录。会话地图和原因仍保留用于排查；已有参考文件不被覆盖。查看同一 JSON 的 `health_review`，其中有拒绝原因、统计和前 20 个严重异常帧。正常遮挡或特征不足导致的搜索阶段不等于严重数值异常。`PASS` 只表示该检查没有发现严重异常，不是绝对精度验收。

加载或保存错误写在 `orb.log`、`console.log` 和相应 JSON 中。启动失败在终端保留具体的 `MAP_E_*` 错误码和原因；LIVE 后的地图保存错误只写日志。

已有参考地图永不自动覆盖。加载时校验地图内容、词典兼容性、相机/IMU 标定及 ORB 金字塔层数/尺度；辅助清单额外核对相机—IMU 时间偏移及时间基准。修改内外参、图像尺寸、同步参数或金字塔配置后，应新建参考地图，不能删除清单来绕过检查。相机与 IMU 的安装关系、镜头或图像裁切发生变化时也应先重新标定。搬运地图时必须把 `.osa` 与 `.osa.meta.json` 一起复制；`.lock` 是文件操作锁，不是地图数据。

```bash
# 为另一组场景/标定建立新地图；new 拒绝覆盖已有目标
VIO_MAP_MODE=new VIO_MAP_FILE="$kit/maps/desk_02.osa" \
  DISPLAY=:0 bash "$kit/start.sh" live 0

# 以后加载这一份地图
VIO_MAP_MODE=auto VIO_MAP_FILE="$kit/maps/desk_02.osa" \
  DISPLAY=:0 bash "$kit/start.sh" live 0

# 做关闭地图复用的对照运行；不加载、不保存、不发布参考地图
VIO_MAP_MODE=off DISPLAY=:0 bash "$kit/start.sh" live 0
```

重定位需要重新看见旧地图中足够稳定且分布合理的特征；遮挡期间 IMU 不能消除位置漂移。旧地图给出视觉定位参考，不能直接恢复关机前或失跟前的速度与偏置。程序在重新取得视觉约束后使用新 IMU 数据拟合惯性状态，再用未参与拟合的后续图像与 IMU 数据复验；此阶段应保持画面清晰、动作平缓。已经找到成熟旧图时，静止也可能恢复；首次新建图仍需平移与转动激励。恢复通过之前，位姿日志保持 `valid=0`，不会把旧地图的“IMU 已初始化”标志当成当前状态已恢复。

恢复后的速度和零偏以带不确定性的先验进入局部优化，不再把新时间链起点的动态量固定为真值。普通跟踪继续对独立视觉位姿、融合位姿、残差、速度、零偏及位置突跳进行检查。静止候选需要图像特征低运动与 IMU 数据共同支持，不能只凭加速度/角速度稳定判断设备没有平移。出现不一致时将位姿标为无效并重新恢复，不强制锁住显示位置，也不把轨迹裁切到 1.8 米范围内。

已确认静止后，若图像特征仍保持低运动、陀螺仪仍支持低转动，短时加速度统计瞬变不会立即清除静止证据；这种保持最多 0.5 秒，并在 `stationary_evidence_held=1` 时明确记录。出现可观测图像运动或转动证据时立即释放；未先确认静止、视觉不足或 IMU 数据缺失时不启用此保持。它只维持异常检查所需的静止候选证据，不施加零速度约束，也不锁定输出位姿。

首次几何重定位成功后，后续恢复帧优先对最近验证过的视觉帧及参考关键帧的共视邻域做投影匹配和视觉优化。局部跟踪失败时同一帧回退到全局 BoW/PnP；若仍在同一地图且时间、位姿连续，则不因局部尝试失败单独清空 IMU 恢复窗口。局部与全局均要求至少 50 个不重复地图点、覆盖至少 4 个图像区域。完全失配、换图、超过 0.25 秒的视觉间隔、异常位姿跳变或独立复验失败仍重新累计窗口。

`recovery_timing.csv` 中 `LOCAL` 表示连续局部跟踪，`GLOBAL` 表示全局搜索，`GLOBAL_FALLBACK` 表示局部失败后全局搜索；`WAITING` / `TIMEOUT` 分别表示后台安全边界等待和恢复超时，`NONE` 表示本帧未执行恢复搜索。`recovery_search_ms` 是该帧局部/全局几何搜索的合计墙钟耗时，不含 IMU 拟合、复验或后台暂停等待；`recovery_window_resets` 是本次 recovery_id 内清空已有恢复数据窗口的次数。搜索预算为软预算，单次优化调用可能超时；局部尝试不超过约四分之一预算（默认最多 10 ms），失败时保留一次短全局尝试机会。该优化减少重复检索，不承诺固定恢复秒数，Jetson 实际效果需结合这些字段与输入/输出频率比较。

恢复先用至少约 1.2 秒的有效数据拟合，再冻结该结果，用至少 8 个后续视觉位姿、至少约 0.6 秒的新数据独立预测复验；复验数据不回填进原拟合来制造低残差。默认总共尝试约 12 秒；未通过时保留旧地图，给新地图约 5 秒初始化机会。如果仍未初始化，会再次尝试旧地图。恢复和重新初始化期间当前图像继续刷新；恢复失败不通过强行沿用旧位姿来伪造连续轨迹。

| `phase` | 含义 |
| --- | --- |
| `MAPPING` | 新地图的视觉或惯性初始化尚未就绪 |
| `SEARCHING` | 正在查找并验证旧地图位置，或等待后台线程暂停 |
| `ALIGNING_IMU` | 视觉已找到旧地图，正在拟合速度与 IMU 零偏 |
| `VALIDATING_IMU` | 拟合结果已冻结，正在用后续新数据独立复验 |
| `TRACKING` | 本帧正常跟踪且当前惯性状态就绪 |

`session/recovery_events.csv` 记录阶段、地图 ID、匹配内点数、视觉/惯性就绪和原因的变化；`session/poses.csv` 逐帧记录相机在当前地图坐标系中的 `T_wc`，平移单位为米，四元数顺序为 `qx,qy,qz,qw`。启用复用时，`valid=1` 还要求本帧通过健康检查；其他行使用 `nan`。这些是跟踪当时的估计，不是回环后统一优化的最终轨迹，也不能证明绝对精度。绘图时按 `map_id` 和有效段分段，不跨失跟间隔或新地图原点连线。

`session/state_diagnostics.csv` 每次完成跟踪调用都写一行，包含正常帧、被拒绝的候选和恢复阶段，不只记录状态变化。`health_evaluated=0` 表示还没有检查所需状态；未知量为 `nan`，不能把它当作零速度。`hard_fault=1` 是核心显式判定的严重数值/一致性事件，不根据日志文字猜测。开启 `VIO_MAP_MODE=off` 的对照运行保留原来的有效位姿判据，不启用这些健康门槛；新 CSV 仍会生成，但未计算的诊断显示 `nan`。

| 诊断列 | 单位和含义 |
| --- | --- |
| `velocity_w_*_mps` | 当前 IMU 主体速度，当前地图世界系，米/秒 |
| `gyro_bias_*_radps` / `accel_bias_*_mps2` | 当前陀螺仪/加速度计零偏，IMU 坐标系，rad/s 与 m/s² |
| `visual_inliers` / `visual_cells` | 独立视觉解的内点数与图像覆盖单元数 |
| `visual_rms_px` / `fused_rms_px` | 视觉/融合位姿对应的重投影残差，像素；不是位置误差真值 |
| `visual_translation_error_m` / `visual_rotation_error_rad` | 同一帧独立视觉与融合位姿的差异 |
| `map_updated` | 本帧是否检测到后端地图更新；地图坐标修正不能直接解释为设备真实运动 |
| `pose_step_m` / `pose_step_dt_s` | 当前内部状态相对上一帧内部状态的位置变化与时间间隔；上一帧内部状态可能随地图更新而修正 |
| `output_pose_baseline` | 是否有同一连续有效段中、不可回写的上次已接受输出作为比较基线；新图、恢复或失跟间隔不跨段比较 |
| `output_pose_step_m` / `output_pose_step_dt_s` | 当前候选相对上次已接受输出对应的 IMU 主体世界位置变化与时间间隔；基线不随后端更新而改写，不是直接对 `poses.csv` 相机平移做差 |
| `flow_*` / `imu_*` / `stationary_*` | 特征位移、短窗 IMU 统计及静止候选证据；候选不等于已知真实零速 |
| `stationary_evidence_held` | 已确认静止且视觉/陀螺仪证据仍支持时，是否正在使用最多 0.5 秒的证据保持 |
| `recovery_fit_velocity_w_*` / `recovery_fit_*_bias_*` | 恢复窗口拟合出的初始动态状态，与持续跟踪状态分列记录 |
| `recovery_fit_*_rms_*` / `recovery_holdout_*_rms_*` | 拟合段与未参与拟合的后续复验段残差 |
| `recovery_fit_condition_number` | 速度/加速度计零偏拟合的条件数，反映数值可辨识性，不是精度概率 |
| `health_reason` | 本帧检查或拒绝原因 |

后台优化可能同时改写地图和内部上一帧状态，因此内部 `pose_step_m` 很小也不能说明已发布轨迹连续。程序额外保留不可回写的上次已接受输出，检查 `output_pose_step_m`。若在 `map_updated=1` 时发现超限跳变，日志明确标为后台地图更新期间的输出不连续；这不是对设备真实运动速度的测量，也不会把坐标跳变直接解释成硬件移动。异常帧标为无效并进入恢复，不用改写历史输出来隐藏跳变。

`record`、`record-view`、`replay` 和 `trajectory` 保持独立的新会话流程，不读取实时参考地图，也不会因 `VIO_MAP_*` 环境变量改写已封存录制的参数。频率/延迟测试持续保留在实时入口。

首次实机复验时，先在有纹理的场景平缓移动，完成建图；再把整套组件固定静止至少 30 秒。随后短暂遮挡镜头，移开遮挡并保持场景清晰，等待恢复后再静止至少 30 秒。记录每段的大致开始时间。不要把原生窗口仍在刷新或 `TRACKING` 标签作为精度通过依据；需要比较固定位置的轨迹及对应状态日志。若更换了相机/IMU 的相对安装位置或图像裁切，先重新标定。

正常退出后，可在终端检查本次异常与发布结果：

```bash
d=$(ls -td "$kit"/runs/live_*/session | head -n 1)
cat "$d/map_session.json"
python3 - "$d" <<'PY'
import csv, sys
from pathlib import Path
p = Path(sys.argv[1]) / "state_diagnostics.csv"
with p.open(newline="") as f:
    rows = list(csv.DictReader(f))
faults = [r for r in rows if r["hard_fault"] == "1"]
print("完成调用帧数:", len(rows))
print("有效位姿帧数:", sum(r["valid"] == "1" for r in rows))
print("严重异常帧数:", len(faults))
for r in faults[:20]:
    print(r["frame_idx"], r["camera_ns"], r["health_reason"])
PY
```

这些是主动检查命令的输出；主程序在 `LIVE...` 后仍静默。若需要关闭复用/健康检查的对照，使用上面的 `VIO_MAP_MODE=off` 命令，并分别保留两个完整 `session`，不要只比较最后一行位姿。

## 输入频率与延迟测试

实时入口自动开启计时，结束后静默生成 `session/performance.txt`（中文报告）和 `session/performance.json`（结构化数据）。测试日志保留在适配层；单目惯性的参数、采集格式和选帧策略保持原样。核心接口和源码已精简为单目＋IMU。

```bash
kit="$HOME/Documents/mono_imu_vio"
# 此次测量运行 60 秒；词典加载、约 10 秒 IMU 时钟拟合另计
DISPLAY=:0 bash "$kit/start.sh" live 60
# 也可使用 live 0，自己按一次 Ctrl+C 收尾
```

前一段在有纹理、光照充足的场景做平缓平移和转动，让惯性初始化有足够运动；初始化后按正常使用方式继续。不要只原地静止等待初始化。运行结束后再执行：

```bash
d=$(ls -td "$kit"/runs/live_*/session | head -n 1)
cat "$d/performance.txt"
```

报告区分相机驱动时间戳频率、主机收图速度、实际入 ORB 的图像/IMU 频率和跟踪调用完成频率，给出各环节中位、P95、最大耗时。P95 表示 95% 的有效测量值不超过它。成功且 IMU 就绪的帧单独统计；没有此类帧会明确标为不可评估。判断数据质量需同时看运行状态和失跟情况，不能只看速度。

新增文件用途：

| 文件 | 记录内容 |
| --- | --- |
| `camera_timing.csv` | 驱动时间戳、映射后的 RAW 时间、取缓冲区、灰度提取和管道写入时间 |
| `imu_timing.csv` | 串口每次读取中有效 IMU 样本数、设备计时器范围、主机读取时间，含启动时钟拟合阶段 |
| `camera_read.csv` | Python 完整读到图像的时间 |
| `input_timing.csv` | 选帧、IMU 配齐、整包送入算法的时间和样本数 |
| `tracking_timing.csv` | ORB 适配器完整接包、TrackMonocular 开始/返回、跟踪状态和 IMU 就绪代理 |
| `performance.txt/json` | 汇总结果；退出后由 `performance.py` 读取上述记录生成 |
| `console.log` | 启动及运行详情，包括被终端隐藏的输出 |
| `poses.csv` | 逐帧相机位姿、有效性、地图 ID 和恢复阶段；无效帧不输出数值位姿 |
| `runtime.json` | 监督进程身份和启动/运行/收尾状态，供只读查询使用 |
| `recovery_timing.csv` | 逐帧恢复搜索方式、搜索耗时、全局搜索标记、窗口重置计数 |
| `recovery_events.csv` | 重定位和惯性恢复阶段变化、地图 ID、内点数及原因 |
| `state_diagnostics.csv` | 逐帧速度、零偏、残差、复验与异常判定，包括无效候选 |
| `map_save.json` / `map_session.json` | 核心保存结果与参考地图发布结果 |

使用 `CLOCK_MONOTONIC_RAW` 统一主机计时。相机源时间先映射到 RAW，再应用原有 +16.881057 ms 配置偏移。此偏移不是处理耗时，也不在本次测试中重新标定。驱动事件到 TrackMonocular 返回是软件可测的图像年龄；尚未验证驱动时间戳与真实曝光时刻的关系，因此不把它称为物理曝光到输出延迟。窗口呈现、异步建图和回环结束时间不包含在 TrackMonocular 调用耗时内。最新 IMU 样本年龄也受原有时钟映射影响，不代表已校准的串口传输延迟。

`imu_initialized_proxy` 在 `tracking_state==2` 且本帧惯性状态就绪时为真。启用地图复用时，它读取恢复状态接口：加载旧图或失跟后的视觉重定位成功，还需要新惯性状态拟合/复验和本帧健康检查通过才会为真。关闭地图复用时保持原先的保守判据：当前帧有效地图点属于同一张已初始化地图；没有地图点、跨图混合或失跟时为假。不调用初始化前可能尚不可用的 `GetTimeFromIMUInit()` 调试接口；该代理不用于证明轨迹精度。IMU 按图像分批输入，不会因此每秒输出 500 个位姿。CSV 会有计时和磁盘写入开销，报告反映本测试版本的实际表现。

需要重新生成报告时，可执行 `python3 "$kit/performance.py" "$d"`；该命令也保持静默，不调用传感器或重新运行 SLAM。

## 回放和轨迹导出

```bash
# 回放录制，显示两个原生窗口
bash "$kit/start.sh" replay "$take"

# 单目惯性 EuRoC 示例离线求轨迹，路径见终端输出
bash "$kit/start.sh" trajectory "$take"
```

`trajectory` 生成 `CameraTrajectory.txt` 和 `KeyFrameTrajectory.txt`。用浏览器打开包内 `trajectory_view.html` 可回看前者。原生单目惯性轨迹输出 IMU 主体位姿，返回误差工具使用外参转换到相机坐标系。

## A→B→A 检查

在第一个终端启动新录制。

```bash
kit="$HOME/Documents/mono_imu_vio"
take="$kit/runs/take_03"
bash "$kit/start.sh" record "$take"
```

看到录制目录的 `record.log` 出现 `Recorded` 后，先对着有纹理的场景做约 20 秒平移和转动。保持录制运行，在第二个终端执行下列命令，按提示记录固定姿态 A、B 和返回 A，每段静止 3 秒。

```bash
kit="$HOME/Documents/mono_imu_vio"
take="$kit/runs/take_03"
bash "$kit/start.sh" mark "$take"
```

标记完成后再录几秒，在第一个终端按 Ctrl+C 封存。用 `bash "$kit/start.sh" trajectory "$take"` 回放，成功检查时可查看 `accuracy.json` 里的 A→B 位移、返回 A 偏差和静止段抖动。返回偏差衡量重复性，绝对误差需要独立实测参考。

`source/ORB_SLAM3` 是随包提供的精简修改版核心源码，`source/adapter` 是项目采集和运行适配源码。启动使用包内已编译程序。
