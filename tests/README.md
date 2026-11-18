# 验证说明

本目录检查软件接口、像素提取、配置一致性、运行状态及继承的地图恢复逻辑。Docker 软件测试覆盖相机接口、模型及离线处理。实机处理率、延迟和轨迹精度须根据 Jetson 的运行记录判断。

## 本次适配的验证

- `omni_radtan_initialization_test.cc`：带噪声和误匹配的球面双视图初始化，覆盖大于 90° 的有效视线和纯旋转拒绝。
- `native_omni_settings_test.cc`：加载活动配置，检验 1920×1536 输入、与独立 Kalibr 公式一致的投影、解析导数、带方向的三角化、优化残差和 Atlas 相机序列化。

- `test_camera_profile.py`：候选配置解析、动态尺寸、时间域、配置指纹与快照一致性。
- `test_trial_calibration.py`：Omni 候选标定来源、尺寸、时间偏移、临时试跑隔离、地图指纹和启动提示。读取配置及测试快照。
- `native_camera_profile_test.py`：原生图像尺寸、行跨度、裁剪和灰度处理回归，以及模拟 Tegra 60 fps 控制设置与读回。

在工程根目录运行，Python 需可导入 NumPy；原生完整测试需 Linux ARM64（可用本地构建容器），使用模拟设备，不打开真实相机：

```bash
python3 -m unittest discover -s tests -p 'test_*.py' -v
python3 tests/native_camera_profile_test.py
# 原生 ARM64 主机也可单独验证 NEON 像素边界
python3 tests/native_camera_profile_test.py --gray-only
```

`native_viewer_size_test.py 归档路径` 在 Linux ARM64 上用打包后的真实无窗口程序检查 1920×1536、640×512 的输入边界及截断帧拒绝，使用合成数据，不访问硬件。

这些测试使用当前源码；归档动态链接另由打包检查和 Jetson 的 `check.sh` 验证。不能用旧包的通过结果替代本次适配验证。

## 可复用的既有检查

在工程根目录可执行不依赖实机的终端、状态与核心策略检查：

```bash
python3 tests/check_live_console.py
python3 tests/check_map_session.py
python3 tests/check_map_startup.py
python3 tests/check_status.py
python3 tests/check_keyframe_policy.py
python3 tests/check_tracking_health.py
```

这些检查分别覆盖 LIVE 后静默、地图加载提示、状态新鲜度与进程身份、关键帧决策及健康检查。有限重投影 RMS 不再单独拒绝的策略、非有限值/速度/零偏/位姿检查和独立恢复复验均继承原实现。

`native_recovery_test.py` 等合成核心测试仍有参考价值，运行前须核对其构建根目录、OpenCV 路径和链接依赖是否已适配新环境。新缓存为 `/build/mono-vio-60/work`，新 OpenCV 编译版本为 4.5.4；不能依靠旧环境的硬编码路径验证新产物。解析真值的合成轨迹检查恢复算法，不代表实机误差；测试中标记成熟的地图不改变生产初始化条件。

## 继承测试的适用边界

部分旧测试仍硬编码 `mono_imu_vio` 包名、`/build/mono-imu-vio/work`、OpenCV 4.8 路径、旧标定、固定 1920×1441 输入或旧快照格式，需先适配再用于新包。尤其是 `bridge_timing_test.py`、`replay_smoke.py`、`native_mode_test.py`、`native_atlas_test.py` 等包/录制集成测试，不能仅换归档文件名就宣称覆盖了新快照。`test_source_contract.py` 已更新为新相机配置接口。`check_map_session.py` 与 `check_map_startup.py` 已改为使用合成的新配置快照，运行它们需要 NumPy。旧文档提到但本副本不存在的测试不作为本次验证依据。

完整验证还需在授权的实机阶段分别确认：相机身份和有效画面、像素格式/行跨度/裁剪、持续采集频率、时间戳域与事件、IMU 协议/频率、新标定、处理频率/延迟及真实轨迹表现。软件通过、地图保存成功和状态 `ready` 均不能替代这些测量。

继承的 Shutdown 段错误问题尚未解决；任何本次通过的配置或回归测试都不应被解释为该问题已修复。
