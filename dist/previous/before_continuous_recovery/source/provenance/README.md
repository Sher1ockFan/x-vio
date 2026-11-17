# 源码来源与修改范围

本工程是 ORB-SLAM3 的单目惯性修改版，基于官方提交 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`。它不是未经修改的官方版本。完整可编译工程位于 Mac 的 `Documents/vio_ws/mono_imu_vio`；Jetson 安装包中的 `source/` 是便于阅读的源码子集。

三份记录各自的用途：

| 文件 | 用途 |
| --- | --- |
| `upstream.sha256` | 固定官方基线的 955 个文件及其 SHA-256；不随本工程修改而更新 |
| `source_changes.json` | 相对基线的新增、修改、删除路径及前后校验值，完整保留此前精简删除的 691 个路径 |
| `core_changes.patch` | 保留文件的文本修改和新增文本文件；删除路径由 JSON 单独记录，因此它不是包含全部裁剪操作的一键补丁 |

工程根目录的 `source.sha256` 记录当前实际参与构建的核心源码清单。修改核心后需要用 `tools/update_manifest.py` 更新它。适配器、启动器、测试脚本属于本项目代码，不属于上述官方核心基线；它们随完整工程保存。

本次地图复用的主要改动：

| 核心文件 | 修改目的 |
| --- | --- |
| `src/Tracking.cc`、`include/Tracking.h` | 失跟和加载旧图后的搜索、几何验证、惯性恢复状态机；恢复期间刷新图像，超时后保留旧图并退回新建图 |
| `src/KeyFrameDatabase.cc`、`include/KeyFrameDatabase.h` | 在指定成熟地图内选择有限数量的重定位候选 |
| `src/ImuRecovery.cc`、`include/ImuRecovery.h`（新增） | 用当前连续视觉位姿和新 IMU 样本估计速度及陀螺仪/加速度计零偏；校验时间覆盖、残差和可解性 |
| `src/System.cc`、`include/System.h`、`include/MapReuseStatus.h`（新增） | 地图加载保存入口、恢复状态快照、调用线程统一完成退出和保存 |
| `src/Atlas.cc`、`include/Atlas.h`、`include/AtlasPersistence.h`（新增） | 成熟地图筛选、格式/词典/标定兼容检查、完整写入后原子替换会话 Atlas |
| `src/KeyFrame.cc` | 新会话的时间链根节点不伪造跨会话 IMU 边；明确初始化持久化占位数据 |
| `src/LocalMapping.cc`、`include/LocalMapping.h` | 处理恢复后的新关键帧时间链，避免把旧会话的时间跨度当成连续 IMU 积分 |
| `src/LoopClosing.cc`、`include/LoopClosing.h` | 提供恢复所需的暂停边界，协调回环/全局优化与地图切换及保存 |
| `src/Viewer.cc` | 窗口停止时请求主调用线程收尾，避免 GUI 线程与保存过程竞争 |
| `src/MLPnPsolver.cpp` | 补全析构定义，让反复重定位采用局部对象生命周期而不泄漏求解器 |
| `CMakeLists.txt` | 编译新增惯性恢复实现 |

已有单目惯性精简涉及 `Frame`、`Settings`、`System`、`Tracking`、示例入口及构建配置等；完整累计差异以 JSON 和补丁为准，不能只把上表当成所有历史修改。

核心以外，`jetson/map_session.py` 负责参考地图清单、同步参数兼容检查和首次发布；`start.sh` 只修改每次实时运行的参数副本。`live_console.py` 保持 LIVE 后静默。`adapter/viewer.cc` 输出逐帧 `poses.csv` 和状态变化 `recovery_events.csv`，恢复未通过时位姿无效；原有计时 CSV 列不变。`params/` 的本次标定数值保持不变。

恢复先验中的不确定度采用工程近似；软件通过测试不等于实机轨迹精度已经验收。地图门槛、几何内点和残差检查不能保证高速抖动、重复纹理或遮挡期间没有漂移。需要用 Jetson 上同一工作区的遮挡恢复、重启加载和 A→B→A 录制进行独立验收。

## 2026-11-13 恢复状态修复

新增 `TrackingHealth` 和逐帧数值诊断；恢复训练解通过未来新数据验证后才提交。`Tracking` 在插入关键帧、发布位姿前检查独立视觉与融合状态、速度/零偏和静止候选一致性。`KeyFrame` 保存恢复动力学软先验，`Optimizer` 允许修正恢复时间链根节点的速度及零偏，并拒绝非有限局部优化结果。`System` 使用 Atlas v2 格式，不能用新版直接读取旧 v1 地图。启动器默认参考为 `workspace_v2.osa`，旧文件保留；严重数值异常或诊断缺失时不自动发布参考图。

`state_diagnostics.csv` 保留每帧速度、零偏、拟合/后续验证残差、视觉支持和健康判断；`poses.csv`、`tracking_timing.csv` 等原有列保持不变。上述检查提高错误可见性和拒绝能力，不代表实机漂移或绝对精度已通过验收。
