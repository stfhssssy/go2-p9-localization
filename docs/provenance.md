# P9 来源、版本与证据边界

## 实际运行的算法来源

历史 P9 (`localization_current`) 使用服务器工作区 `/data/t9/liuxu/go2-autonomy` 的已有构建。Point-LIO 源码目录为：

```text
/data/t9/liuxu/go2-autonomy/src/autonomy_stack_go2/src/slam/point_lio_unilidar
```

仓库 origin 为 [`jizhang-cmu/autonomy_stack_go2`](https://github.com/jizhang-cmu/autonomy_stack_go2)。2026-11-08 只读审计直接读取 `.git/HEAD` 及 ref，得到分支 `integration/gazebo-jazzy`，ref `43d5f54b389b251713f0097893c30fa76c870d54`。这是该工作树的提交参考，不是 clean-tree 声明；本次 `git status` 被所有权检查拒绝，没有修改服务器 Git 配置。发布时以实际复制文件的哈希清单和记录的构建补丁为准，不能只标上游 URL 或提交号。

保存于 P9 `algorithm_binaries.json` 的二进制身份：

| 程序 | SHA-256 |
|---|---|
| `pointlio_mapping` | `b7513098067ccf0d44b5c835dc34c69c4be94e0ced923c6115f09d8f81900ea7` |
| `terrainAnalysis` | `8afa103dc3f95069f42b280012823ef274872b979ef096eb52cda58707b5c152` |
| `terrainAnalysisExt` | `935c573ce6b1fc3cc589e5a5b35016757f72068c9124bb5cfa2e5194b7071b59` |

2026-11-08 读取原 `build/point_lio_unilidar/pointlio_mapping`，哈希仍与上表相同，`file` 确认为 x86-64 ELF。ARM64 发行版必须从记录的源码重编译，具有新的二进制 SHA；不得宣称与该二进制相同，或仅以源码相似推断数值结果逐位一致。

当日实际服务器关键源码 SHA-256：

| 相对 Point-LIO 目录的文件 | SHA-256 |
|---|---|
| `src/laserMapping.cpp` | `aaa6fdc595a0d162df92f87e9a226cebe6ac79d9bfcc790dce7b6a06945b2ffe` |
| `src/preprocess.cpp` | `885db240dd192c1c9c574afa98dbdb4ff8e7e88b6b602804359cd3e8779cabee` |
| `src/parameters.cpp` | `39a8ef3f75da3e3a79bd2a068134e2c2796fddbd391d9aede2724ceb82470bda` |
| `src/IMU_Processing.hpp` | `1ae25097bc55e669061be2f9597c3e3e62d9d241902eca68c2b00b7c620c972c` |
| `src/Estimator.cpp` | `fc5615c59591f44416e3062957bfd748ce5218212f26d68ac4a2c1ef5d1a9ce5` |

这是当前源快照审计，不是用可复现构建证明历史二进制与所有当前源文件一一对应。历史运行同时保留了入口脚本、配置和二进制哈希；镜像发布另行记录快照与补丁。

## 不混用的开发分支

早期另一个本地 `work/autonomy_stack_go2` 工作树包含 gyro-only、协方差、重力初始化等开发修改。其 `laserMapping.cpp` 哈希为 `75f573d6ac077ec36fd23ee89db1028df89e555d98befd602cf81e6108fd3943`，`parameters.cpp`、`IMU_Processing.hpp`、`Estimator.cpp` 也与服务器 P9 不同。`preprocess.cpp` 相同不能证明整个算法相同。P9 发布必须选用上节实际目标源快照，不应无意混入该开发分支。

## ARM64 构建差异

审计的 Point-LIO CMake 使用 C++17、`-O3`、pthread、异常支持和可用的 OpenMP，没有显式 `-march=native`。其并行宏按架构选择：x86 多核分支可启用 `MP_EN`/`MP_PROC_NUM=4`，非 x86 默认为 `MP_PROC_NUM=1`。因此 ARM64 的吞吐、积压和 CPU 占用必须单独测量，不能套用 x86 回放成绩。若发行补丁改变线程数或编译 flags，必须在发布清单中列明。

构建还依赖 ROS/PCL/Eigen/Python 开发库。基础镜像标签 `go2-control:arm64-v4` 本身不固定内容；发布记录应同时保存镜像 ID 或可获得的 digest、体系结构、工具链、依赖版本和最终 image ID。运行时不自动下载依赖。

## 数据和已验证范围

P9 原始 localization MCAP SHA-256 为 `e2848dbfe8c6bd3929dd961349c14a53dd5659da238f8c9162e4406a518f6539`。派生 bag 只处理相机加速度覆盖/插值；云与设备 odom 的源消息保持，云 header 偏移在运行 adapter 施加一次。具体 conversion manifest 保存输入/输出计数、时间覆盖和验证结果。

历史回放保留 1779 云、23099 相机 IMU，得到 1771 组同刻位姿/配准扫描。保存轨迹约 114.97 秒，两个地形话题各 1771 帧。定位输出是相机 IMU 原点；后续统一到名义机身的 RViz 展示变换没有回写估计器。几何 `map.pcd` 是外部 5 cm 体素累积，不是直接导出的 Point-LIO ikd-tree。

宇树 odom 仅用于事后设备参考。没有独立 GT、没有真实比赛机标定，也没有用本次录包证明比赛导航闭环。历史 Gazebo 闭环属于使用仿真传感器名义 R/T 的独立实验，不能替代实机标定与验收。新在线重采样、ARM64 二进制及每台机器人需保留各自验证记录；历史离线成功不能自动扩展成这些结论。

## 发布内容边界

发布保存必要源文件、原许可证、构建补丁、示例配置、测试和 `provenance/source-manifest.json` 来源清单。vendored 算法位于 `src/point_lio_unilidar`。原始录包、地图、私有竞赛文件、团队密钥、SSH 私钥、认证 token、个人绝对路径和运行日志不因创建私有仓库而自动成为发布内容。本文中的服务器路径只用于定位历史构建，不是运行时依赖。

发布前应核对 [第三方说明](../THIRD_PARTY_NOTICES.md) 与实际打包目录一致。镜像接收者应能获得该镜像对应的源码快照和构建方式；私有仓库与私有镜像不改变各组件的原有授权条款。
