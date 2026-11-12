# name_test

专用于相机/IMU 实际输入频率和软件链路延迟测试的独立工程。由此次已验证的测量版本复制，默认运行结果保存在本项目 `runs/`，与 `vio_nx` 分开；算法核心与标定参数保持一致。

Jetson Orin NX 上的单目惯性 ORB-SLAM3。B 口实时采集 1920×1441 UYVY，跳过首行后送入 1920×1440 灰度图。ORB 使用 `fit_05` 配置并在内部缩放到 960×720。B 口的相机–IMU 时偏暂用 C 口标定值。

| 目录 | 用途 |
| --- | --- |
| `ORB_SLAM3/` | 完整官方源码快照，提交 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` |
| `adapter/` | 相机和 IMU 采集、时间映射、记录及 ORB-SLAM3 调用 |
| `params/` | `fit_05` 标定参数 |
| `docker/` | 容器内编译、打包和运行库检查 |
| `jetson/` | Jetson 启动、检查、回放和轨迹导出工具 |
| `dist/` | 生成的 Jetson 安装包 |

上游许可证在 [ORB_SLAM3/LICENSE](ORB_SLAM3/LICENSE)，依赖的许可文件随上游源码和 Jetson 安装包提供。

## 构建与运行

在仓库根目录运行 `ORB_CONTAINER=orin-r365-dev bash build.sh`。构建使用准备好的 Ubuntu 22.04 ARM64 容器，容器需提供 GCC 11、CMake、Ninja、ccache，以及 `/build/orb-kit` 下的 NVIDIA OpenCV 4.8.0 和 Pangolin。编译复用容器中受锁保护的 `/build/orb-hub` 缓存，安装包单独命名为 `name_test.tar.gz`。源码清单用于校验仓库内的官方快照及容器中的编译副本。容器没有该源码副本时，构建脚本从仓库导入。

实时入口在 `LIVE...` 提示后保持终端静默，详细输出保存在 `session/console.log`。适配层记录实际传感器/算法输入频率与调用耗时，退出后由 `jetson/performance.py` 生成报告；官方算法源码不改动。

安装包输出到 `dist/name_test.tar.gz`。Jetson 安装和原生双窗口运行方式见 [使用说明](jetson/README.md)。

## 暂存到 Git

在仓库根目录初始化或打开 Git 仓库后，先执行普通暂存，再按清单补齐上游忽略规则中的源码和许可文件。

```bash
git add -- .
awk '{ print "ORB_SLAM3/" substr($0, 69) }' ORB_SLAM3.sha256 |
  git --literal-pathspecs add --force --pathspec-from-file=-
git status --short
```

`ORB_SLAM3.sha256` 应记录 955 个文件。Docker 产物位于 `dist/`，由仓库的忽略规则排除。
