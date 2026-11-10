# orb-slam3_hub

适用于 Jetson Orin NX 的原版单目惯性 ORB-SLAM3。Mac 端在 ARM64 Docker 中编译，Jetson 解压后在宿主机运行。核心使用上游提交 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`。窗口是原生地图与当前图像。

原分辨率配置来自在 C 通道复核过的 `fit_05` 标定。相机采集 1920×1441 UYVY，跳过首行元数据后得到 1920×1440 灰度图，ORB 内部缩放到 960×720。IMU 为 500 Hz，实时采集在相机时间映射后应用一次 **+16,881,057 ns**，回放沿用录制时已经同步的时间戳。原分辨率配置在 `params/orb_trial.yaml` 和 `params/timing.json`。

## 安装与检查

把 `orb-slam3_hub.tar.gz` 上传到 Jetson 用户家目录，在 Jetson 桌面终端执行。

```bash
tar -xzf "$HOME/orb-slam3_hub.tar.gz" -C "$HOME"
kit="$HOME/orb-slam3_hub"
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

查看本次目录下 `session/camera.log` 的 `delivered_hz` 核对采集帧率。实时终端的 `sent` 和 `superseded` 分别记录送入 ORB 的帧和被新帧替代的帧。

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
kit="$HOME/orb-slam3_hub"
take="$kit/runs/take_03"
bash "$kit/start.sh" record "$take"
```

看到录制目录的 `record.log` 出现 `Recorded` 后，先对着有纹理的场景做约 20 秒平移和转动。保持录制运行，在第二个终端执行下列命令，按提示记录固定姿态 A、B 和返回 A，每段静止 3 秒。

```bash
kit="$HOME/orb-slam3_hub"
take="$kit/runs/take_03"
bash "$kit/start.sh" mark "$take"
```

标记完成后再录几秒，在第一个终端按 Ctrl+C 封存。用 `bash "$kit/start.sh" trajectory "$take"` 回放，成功检查时可查看 `accuracy.json` 里的 A→B 位移、返回 A 偏差和静止段抖动。返回偏差衡量重复性，绝对误差需要独立实测参考。

`source/ORB_SLAM3` 是随包提供的选定上游源码，`source/adapter` 是项目采集和运行适配源码。启动使用包内已编译程序。
