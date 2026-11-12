# name_test

相机/IMU 输入频率与延迟测量包。部署到 `~/Documents/name_test`，不覆盖现有 `vio_nx`。实时测试继续使用 B 口、原标定参数及两个原生窗口，`LIVE...` 后终端静默。

适用于 Jetson Orin NX 的原版单目惯性 ORB-SLAM3。Mac 端在 ARM64 Docker 中编译，Jetson 解压后在宿主机运行。核心使用上游提交 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`。窗口是原生地图与当前图像。

原分辨率配置来自在 C 通道复核过的 `fit_05` 标定。相机采集 1920×1441 UYVY，跳过首行元数据后得到 1920×1440 灰度图，ORB 内部缩放到 960×720。IMU 为 500 Hz，实时采集在相机时间映射后应用一次 **+16,881,057 ns**，回放沿用录制时已经同步的时间戳。原分辨率配置在 `params/orb_trial.yaml` 和 `params/timing.json`。

## 安装与检查

把 `name_test.tar.gz` 上传到 Jetson 用户家目录，在 Jetson 桌面终端执行。

```bash
mkdir -p "$HOME/Documents"
tar -xzf "$HOME/name_test.tar.gz" -C "$HOME/Documents"
kit="$HOME/Documents/name_test"
bash "$kit/check.sh"
```

看到 `ORB_HUB_CHECK_OK` 后可以启动。包内带词典和项目运行库。宿主机需保留 Ubuntu 22.04 ARM64、Jetson Linux R36.5.0、NVIDIA OpenCV 4.8 `.so.408` 运行库、图形运行库和 `python3-numpy`。原生图形窗口使用桌面 X11 会话。

实时入口使用 `/dev/gmsl/B`。原分辨率录制入口使用标定时的 C 通道 `/dev/video6`。IMU 使用原 CP2102 `/dev/serial/by-id` 设备链接和 921600 波特率。账号需要能读写相机和读取串口。若缺少权限，用 `ls -lL /dev/gmsl/B /dev/video6 /dev/serial/by-id/*` 查看设备和用户组，确认后加入对应的 `video` 或 `dialout` 组，再重新登录。`--devices` 检查 C 通道录制入口和 IMU 的设备节点及权限。

## 实时和录制

关闭其他占用相机或 IMU 串口的程序。实时预览默认将日志放入 `$kit/runs/live_*`。录制时每次指定新的目录。

```bash
# B 口原分辨率实时查看。0 表示运行到按 Ctrl+C
bash "$kit/start.sh" live 0

# C 口录制原始图像和 IMU，任选其一
take="$kit/runs/take_01"
bash "$kit/start.sh" record "$take"
# 或者另用一个新目录，同时开启两个原生预览窗口
bash "$kit/start.sh" record-view "$kit/runs/take_02"
```

录制时按一次 Ctrl+C，等终端返回。`capture.json` 中的 `COMPLETE` 表示录制封存完成。按 30 fps，每分钟原始录制约占 5 GB，离线转换还会另存 PNG 图像。实时预览与正式录制的用途不同，评估逐帧轨迹时使用封存数据回放。

实时入口校验 B 通道的驱动卡名 `pnd_gmsl_mc 10-003b` 和采集总线 `platform:tegra-capture-vi:0`。实时程序打开 IMU 串口后先读过前 0.5 秒的启动数据，随后校验连续数据并采集约 10 秒估计时钟映射。B 通道暂用 C 通道标定的时间偏移，时延需实机复验。

实时入口打印 `LIVE: move the rig with both translation and rotation in a textured scene.` 后不再输出运行进度、报错详情或收尾提示。原生两个窗口仍运行；按一次 Ctrl+C 后等待终端返回。启动失败仅显示错误码 `VIO_E_STARTUP`，详情保存在 `runs/startup_*.log`，已创建会话时保存在 `session/console.log`。运行中异常保留非零退出状态，详情查 `session/live.json` 和 `session/console.log`。终端自身显示的 `^C` 或命令提示符不属于程序输出。

`session/console.log` 保留原有进度、`sent` 和 `superseded`，后二者分别为完整写入 ORB 管道的帧数和被新帧替代的帧数；完整写入不等于 ORB 已经完成跟踪。`session/camera.log` 继续保留采集日志。

## 输入频率与延迟测试

实时入口自动开启计时，结束后静默生成 `session/performance.txt`（中文报告）和 `session/performance.json`（结构化数据）。这只增加适配层日志；官方 ORB-SLAM3 核心、标定参数、采集模式和选帧策略保持原样。

```bash
kit="$HOME/Documents/name_test"
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

使用 `CLOCK_MONOTONIC_RAW` 统一主机计时。相机源时间先映射到 RAW，再应用原有 +16.881057 ms 配置偏移。此偏移不是处理耗时，也不在本次测试中重新标定。驱动事件到 TrackMonocular 返回是软件可测的图像年龄；尚未验证驱动时间戳与真实曝光时刻的关系，因此不把它称为物理曝光到输出延迟。窗口呈现、异步建图和回环结束时间不包含在 TrackMonocular 调用耗时内。最新 IMU 样本年龄也受原有时钟映射影响，不代表已校准的串口传输延迟。

`imu_initialized_proxy` 在 `tracking_state==2` 且当前帧有效地图点属于同一张已初始化地图时为真。适配器通过公开接口读取地图标志；没有地图点、跨图混合或失跟时保守记为假。不调用初始化前可能尚不可用的 `GetTimeFromIMUInit()` 调试接口；该代理不用于精确判断初始化完成时刻或证明轨迹精度。IMU 按图像分批输入，不会因此每秒输出 500 个位姿。新增 CSV 会有少量计时和磁盘写入开销，报告反映本测试版本的实际表现。

需要重新生成报告时，可执行 `python3 "$kit/performance.py" "$d"`；该命令也保持静默，不调用传感器或重新运行 SLAM。

## 回放和轨迹导出

```bash
# 回放录制，显示两个原生窗口
bash "$kit/start.sh" replay "$take"

# 官方 EuRoC 示例离线求轨迹，路径见终端输出
bash "$kit/start.sh" trajectory "$take"
```

`trajectory` 生成 `CameraTrajectory.txt` 和 `KeyFrameTrajectory.txt`。用浏览器打开包内 `trajectory_view.html` 可回看前者。原生单目惯性轨迹输出 IMU 主体位姿，返回误差工具使用外参转换到相机坐标系。

## A→B→A 检查

在第一个终端启动新录制。

```bash
kit="$HOME/Documents/name_test"
take="$kit/runs/take_03"
bash "$kit/start.sh" record "$take"
```

看到录制目录的 `record.log` 出现 `Recorded` 后，先对着有纹理的场景做约 20 秒平移和转动。保持录制运行，在第二个终端执行下列命令，按提示记录固定姿态 A、B 和返回 A，每段静止 3 秒。

```bash
kit="$HOME/Documents/name_test"
take="$kit/runs/take_03"
bash "$kit/start.sh" mark "$take"
```

标记完成后再录几秒，在第一个终端按 Ctrl+C 封存。用 `bash "$kit/start.sh" trajectory "$take"` 回放，成功检查时可查看 `accuracy.json` 里的 A→B 位移、返回 A 偏差和静止段抖动。返回偏差衡量重复性，绝对误差需要独立实测参考。

`source/ORB_SLAM3` 是随包提供的选定上游源码，`source/adapter` 是项目采集和运行适配源码。启动使用包内已编译程序。
