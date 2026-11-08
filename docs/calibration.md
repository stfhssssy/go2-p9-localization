# P9 坐标、时间与标定

## 坐标合同

使用列向量，`T_A_B` 将 B 坐标中的点变换到 A：`p_A = R_A_B p_B + t_A_B`，长度单位 m。Point-LIO 的外参为 `T_I_L`，其中 I 是 D435i IMU optical frame，L 是输入原生 Go2 雷达 frame：

```text
p_camera_imu_optical = R_IL * p_native_lidar + t_IL
```

不得在输入消息中先旋转一次，又在 Point-LIO 外参中重复旋转。IMU 光学轴是 x 右、y 下、z 前；是否采用该轴以实际 TF、消息 frame 和驱动实现为准，不能根据话题名称猜测。Point-LIO 需要同一物理 IMU 原点和同一坐标轴中的三轴角速度及加速度比力。保留真实重力响应，不造零加速度，也不因静止正负号看似异常就翻某一轴。

## 历史实验值不是机器人标定

P9 使用以下冻结旋转（行主序），来自另一个 two-laps 录包的 Go2 与 D435i 陀螺仪拟合。它还假设 Go2 原始 gyro 与原始雷达同轴。本次 localization 包没有重新拟合：

```text
R_IL = [[ 0.8355866054446094,  0.5483605551895937, -0.03310175695815267],
        [-0.11474335096813629, 0.23313628860067767, 0.9656507828123501 ],
        [ 0.5372420201456313, -0.8030866531387898,  0.25772628763899647]]
t_IL = [0, 0, 0] m
```

零平移是**未标定假设**，不是传感器实际共点；这组旋转也不是比赛机器人通用安装值。`gravity_align=true` 只是初始化重力方向，不会估计平移、修复加速度语义或替代传感器标定。原配置 `use_imu_as_input=false` 使用该实现的 IMU 观测模式，仍使用三轴 gyro 和 accel。

## 使用比赛机器人的安装信息

在每台机器人运行时读取 `/opt/sar/calibration/go2_camera_realsense.yaml`，记录文件哈希。组织方提供的是 Go2 前相机与 RealSense color/depth/camera_link 的关系，并声明精度不保证；它不是现成的 LiDAR→IMU 外参。

若 C 为 `camera_link`、G 为 Go2 前相机 optical frame，组织方提供 `T_G_C`，驱动 TF 提供 `T_C_I`，还必须获得同一机器人的 `T_G_L`：

```text
T_I_L = inverse(T_C_I) * inverse(T_G_C) * T_G_L
```

缺少链中任意一项时，不能把旧录包旋转或 `T=0` 宣称为已标定值。检查旋转正交且 det=+1、单位为米、父子方向正确，并以多个姿态和独立运动段验证。若 raw accel 与 gyro 原点或轴不同，先解决物理传感器变换；直接在不同原点间替换加速度会忽略角加速度和向心加速度项。

已知 `T_B_I` 后，估计器的 IMU 位姿可转换为机身位姿：`T_W_B = T_W_I * inverse(T_B_I)`。配准点云已经在 W 中，不能为“跟随机身”再乘一次位姿。历史 `map→camera_init` 是恒等别名，`camera_init→aft_mapped` 的 child 是 IMU 原点；这不提供回环或无漂移保证。

## 时间合同

定义 `corrected_lidar_header_ns = original_lidar_header_ns + cloud_offset_ns`。逐点 `time` 为相对该扫描起点的秒值，保持不变。保存偏移的符号、单位、来源和实际应用位置，避免派生 bag 与在线 adapter 双重修正。

| 录包 | 历史 cloud offset | 相机 IMU offset | 用途 |
|---|---:|---:|---|
| two-laps | 2167609347640 ns | 0 ns | 历史实验，仅该时钟关系 |
| localization / P9 | 2167645092767 ns | 0 ns | 相机时间域，独立时钟审计 |

两次常量的估计定义也有差异；不能用它们的差估计硬件时钟漂移。header 与 bag record 时间差包含传输/记录延迟，只能支持时钟基准诊断，不是高精度物理延迟标定。`/clock`、回放速率和 `use_sim_time` 不会修复 header 差值。

真机需要重新检查时钟 epoch、单位、单调性、持续漂移和重启后的变化。先记录足够的原始流，利用陀螺仪等共同运动对时间关系做独立验证，并保留未用于估计的验证段。不要用“地图更薄”或与设备 odom 更接近来调偏移和外参；独立 GT 或实测场景尺寸尚未提供。

## 离线与在线加速度插值

历史 converter 在 merged IMU 的时间戳处读取左右 raw accel，线性插值并只替换 `linear_acceleration`。不外推；非精确命中的双侧 bracket 间隔上限为 30 ms；精确命中样本无需邻居。原 merged gyro、header 和其他字段保持不变。使用 `/tf_static` 核对 raw accel 与 merged IMU 的原点、轴及协方差合同。派生 bag 已包含这些值，P9 回放只转发。

发行适配器的 `imu_mode` 明确区分两个输入合同：

- `merged_linear`：驱动已以 `unite_imu_method=2` 发布线性合并 IMU，或输入是历史 converter 已重建的派生 IMU。适配器验证后原样转发，不再次插值。
- `raw_linear`：订阅 `gyro_topic` 和 `accel_topic`，在收到未来 raw accel 后输出相应 gyro 时间戳的线性插值样本。`max_accel_gap_sec=0.03`，不外推；队列有容量和时间限制。通过 `/tf_static` 验证 raw gyro/raw accel 与输出 IMU 同原点同轴，未知、非恒等或冲突的关系拒绝处理，不悄悄应用额外旋转。

`clock_offset_sec` 仅应用于 cloud header；`point_time_field` 和 `point_time_scale` 声明逐点时间。`mode=live` 检查消息年龄与未来时间（默认上限分别为 1 s 和 0.1 s）；`mode=replay` 不使用该 epoch 年龄检查。最新 cloud/IMU stamp 差超过 `clock_tolerance_sec=0.5` 会报告时钟诊断错误，这只是运行守卫，不是精密同步标定。完整参数和失败处理以发行代码为准。

在线 raw 插值会增加等待，merged 模式也可能包含驱动内部等待；两者均不增加实测加速度采样数量。raw 模式保留 preceding/exact raw accel 的协方差，不据插值公式自行缩小协方差，不能据此宣称重采样噪声已标定。当前在线接口与历史离线 converter 是不同实现，需要单独测试；没有依据历史派生 bag 的成功宣称在线真机已通过。

接受一组标定前，分别检查静止不同姿态下的比力、动态 gyro/acc 一致性、时间 holdout 残差、外参与杆臂效应。一次静止姿态接近 9.81 m/s² 不足以证明所有轴、符号和动态语义正确。
