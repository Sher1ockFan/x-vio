# 验证说明

这些测试检查单目惯性接口、采集协议、性能日志、窗口启动，以及地图保存加载和状态恢复，不验证实机轨迹精度。

在工程根目录可直接运行：

```bash
python3 -m unittest discover -s tests -p test_source_contract.py -v
python3 tests/check_live_console.py
python3 tests/check_map_startup.py
python3 tests/check_status.py
python3 tests/check_keyframe_policy.py
python3 tests/check_map_session.py
python3 tests/check_build_sync.py
```

关键帧测试中的 `oldPolicy` 是原版策略副本，保留旧模式枚举用于对照；它不参与项目算法编译。脚本每次从当前 `Tracking.cc` 读取新策略，比较 100,000 组状态的决定和 BA 中断副作用。

以下测试在已有 ARM64 Docker 编译完成后运行，接收构建产物：

- `native_gray_test.py --archive ARCHIVE`：NEON 像素、元数据行排除、边界/对齐/无效长度。
- `native_capture_test.py --archive ARCHIVE`：模拟 B 通道 V4L2，验证两种采集包格式与计时开关。
- `native_mode_test.py ARCHIVE`：链接实际包中核心库，验证非法模式在访问配置/传感器前被拒绝；需要 CMake 生成的 g2o config.h，默认读取 `/build/mono-imu-vio/work`，可用 `MONO_IMU_BUILD_ROOT` 指定构建根目录。
- `bridge_timing_test.py`：模拟串口、IMU 时钟拟合和实时图像，启动实际原生窗口并核对 CSV 关联及分析报告。
- `replay_smoke.py`：合成 60 帧封存数据，验证导出与原生窗口回放。

最后两项使用 `MONO_IMU_ARCHIVE` 指定安装包绝对路径，默认 `mono_imu_vio.tar.gz`。模拟场景是静态纹理，没有惯性初始化所需运动，预期报告“没有初始化后成功跟踪帧”；不得据此宣称惯性精度通过或初始化性能提升。

新增 ARM64 地图验证：

```bash
python3 tests/native_recovery_test.py
python3 tests/native_atlas_test.py /build/mono-imu-vio/work/mono_imu_vio.tar.gz
```

`native_recovery_test.py` 直接链接当前构建的核心库，使用有解析真值的静止、旋转与平移轨迹，检查非零零偏/速度恢复及 IMU 缺口、位姿异常和非有限数值的拒绝。它不代表实际硬件达到这些误差。

`native_atlas_test.py` 使用当前鱼眼模型和非平凡相机—IMU 外参构造已知几何、已标记成熟的**测试地图**。这是测试夹具，不在生产代码中强制设置 IMU 已初始化。它验证真实核心的序列化、加载后恢复、遮挡失跟/恢复、超时后保留旧图并再次找回、连续新关键帧时间链、重复保存加载、参数不兼容拒绝，以及回环开关和正常停止/保存失败路径。不能据此宣称首次物理建图或实际轨迹精度已通过。

加 `KEEP_NATIVE_ATLAS_QA=1` 可在容器 `/tmp/mono-imu-atlas-*` 保留模拟数据和调试产物。失败时测试输出 native 调用栈；这些测试不接触相机或串口。

`check_build_sync.py` 对真实同步代码使用微型 make 构建图：证明旧的保留时间戳方法可漏掉源码修改，新方法会重新构建，同时不破坏未修改文件的缓存。

## 恢复健康修复验证（2026-11-13）

新增 `check_tracking_health.py` 验证真实记录中的 9.136 米单帧跳变、静止一致性冲突、正常移动及数据缺口。`native_recovery_test.py` 包括冻结拟合参数的未来数据复验，错误速度、零偏和未来测量不能用重新拟合偷偷吸收。

`native_recovery_prior_test.py PACKAGE.tar.gz` 直接调用包内核心的 `LocalInertialBA`，证明恢复时间链根节点的速度/零偏能够被视觉和 IMU 约束纠正，同时普通时间链边界行为不变；检查先验保存加载和世界坐标旋转。

`native_atlas_test.py` 还包含 46 秒以上合成静止、静止画面中突然增加 IMU 偏差、v1 地图拒绝等集成场景。它们有确定的合成几何，只用于回归验证，不替代 Jetson 上的真实静止/遮挡测试。

`mapjump` 集成场景对整张地图施加 9 米共同平移，同时保持输入图像/IMU 不变，验证内部重算上一帧不能掩盖实际输出的不连续：异常帧无效、记录 hard fault，随后可恢复。`NATIVE_ATLAS_FOCUS=mapjump` 可单独运行该场景；默认仍运行完整测试集。

`check_map_startup.py` 验证启动日志按完整记录识别实际旧地图加载成功、跨读取边界和异常记录处理，以及准备阶段的 LOADING / NO_REFERENCE / DISABLED 状态。该测试不把加载成功当作当前位姿或 IMU 恢复成功。

## 连续恢复、无窗口和状态查询

`native_atlas_test.py` 新增 continuity 场景：核对 LOCAL 搜索确实使用、局部失败时全局回退、同图有效回退保留窗口、超过 250 ms 间隔仍丢弃旧窗口，以及 IMU 拟合和未来复验条件未缩短。`NATIVE_ATLAS_FOCUS=continuity` 可单独运行；默认完整套件还覆盖原静止、偏差注入、地图跳变和保存加载。

`MONO_IMU_HEADLESS=1` 运行 `bridge_timing_test.py` 时不设置 DISPLAY，也不启动 Xvfb；实际原生程序以无窗口模式运行，向无限采集发送 SIGINT，核对正常封存、频率/延迟文件和新增恢复计时日志。去掉此变量则运行原生双窗口测试。

`check_status.py` 验证运行、停止、陈旧/不完整日志、PID 复用、外来会话、历史地图与当前就绪的区分。`check_live_console.py` 同时覆盖两种实时入口和运行元数据写入失败，保证 LIVE 后主终端静默。软件模拟通过不等于 Jetson 上恢复时间或轨迹精度已验收。
