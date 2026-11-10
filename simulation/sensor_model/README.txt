P9 navigation 仿真相机 IMU 模型

本目录随新发行包置于 /opt/p9/simulation/sensor_model。
使用上层 simulation/inside.sh 和 run_simulation.py 启动。
ROS_DOMAIN_ID=94，GZ_PARTITION=ssy-p9-nav-20261110。
prepare_runtime.py 会展开实际 /ws 安装中的机器人 URDF，验证安装矩阵、传感器位置、
新 overlay 和 p9nav-sim.yaml 一致，然后将证据保存在运行目录 runtime/。
不要把随附 model_manifest.json 的未展开状态当作运行验证通过。

本模型在现有D455挂载位置 trunk+[.28,0,.062] 新建实际Gazebo IMU link，并旋转到标准光学轴。
这只是明确的仿真安装定义；不是实测D435i/D455内部IMU芯片位置，也没有复用公开bag拟合外参。
启用相机RGBD不是前提；disable_d455=true仍可运行此传感器。没有相机图像输入。
新增link质量1e-6kg、惯量1e-9kg m²、无碰撞/可视几何，固定关节保留。
新增两个IMU传感器都直接测量camera_imu_optical_frame这个物理link：
  /p9_sim/gyro  sensor_msgs/Imu 200Hz；只取angular_velocity。
  /p9_sim/accel sensor_msgs/Imu 100Hz；只取linear_acceleration。
两sensor姿态字段均不进入Point-LIO；两个消息的其余未选测量也不混用。
噪声为选中gyro轴std=.0002rad/s、选中acc轴std=.017m/s²，无额外bias/drift模型。
所有时间使用Gazebo /clock，clock offset=0；禁止加真实bag的约2167秒偏移。

本目录不再添加gz-sim-imu-system：原机器人已含工作中的Imu系统，官方实现发现整个ECM中的
IMU实体。启动后仍须核验两topic各唯一源、频率200/100Hz、无重复header，以排除系统重复实例。
目标环境实际xacro/SDF转换与实时传感器尚由父代理验证；随附model_manifest.json记录
expanded_urdf_checked=false，不能拿它代替运行前model_validated.json。

manifest变换约定 parent_from_child: p_parent=R*p_child+t；四元数均xyzw。
body_from_imu、body_from_lidar、imu_from_lidar、imu_from_body均提供R/t/q/4x4矩阵。
原始L1点坐标保留，Point-LIO使用imu_from_lidar。滤除机身只在临时body坐标中计算mask。
GPU雷达整帧瞬时采样，所有点time=0为正确语义，不使用真实bag中必须time变化的校验规则。
rawgyro时间处线性插值rawacc需缓存双侧样本，不外推；精确命中直接取该样本。
输出合成camera IMU不能声称200Hz独立加速度测量；gyro保真、acc为100Hz测量之间的估计。

静止水平机身时，光学轴比力约[0,-9.81,0]m/s²，不能翻轴/去重力/灌入常量。
Point-LIO gravity_align=true依测得的初始重力建立Z向上世界，输出相机IMU原点位姿。
导航若需要机身姿态：T_map_body = T_map_imu * imu_from_body；因此aft_mapped->base_link
不能再发布恒等TF。registered_scan已在估计世界坐标，无需重复外参转换。
仿真GT若用于评价相机：T_world_imu = T_world_body * body_from_imu；GT严禁输入SLAM/规划控制。
完整运行须验收到达真实目标、避障和停止闭环；仅topic齐全或FAR reached信号不代表导航成功。
