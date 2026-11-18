# 60 fps 单目惯性标定

来源为 `calibrate_nx/cam_imu_motion/run_02` 和 `calibrate_nx/fit/fit_03`。采集使用全分辨率 1920×1536、60 fps，联合拟合选中 CNTVCT 时间映射，IMU 配置为 500 Hz。

- `camera.yaml` 保存原始 Omni-radtan 相机内参。
- `camchain-imucam.yaml` 保存联合标定的内参、IMU 到相机的 `T_cam_imu` 和 +14.502238 ms 的时间偏移。
- `imu.yaml` 是 Kalibr 输出的 IMU 配置，拟合使用的噪声和随机游走是先验值。
- `candidate.json` 记录拟合残差和候选状态。记录中的平均重投影残差约 0.809 px，独立复验已跳过。

活动文件位于上一级的 `orb_trial.yaml`、`camera_profile.json` 和 `timing.json`。运行时将 Kalibr 的 `T_cam_imu` 求逆后用作 `IMU.T_b_c1`。驱动时间戳先转换到每次运行的本机 RAW 时钟，再加相机对 IMU 的残余时偏。
