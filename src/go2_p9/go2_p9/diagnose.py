"""Read-only DDS sensor inspection; no estimator, TF or control publisher."""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np


def run(seconds=10., cloud_topic='/utlidar/cloud', imu_topic='/camera/camera/imu', output=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, Imu
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError('seconds must be finite and positive')
    rclpy.init()
    node = Node('p9_readonly_diagnose')
    samples, details = {'cloud': [], 'imu': []}, {}
    def receive(key, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        samples[key].append((time.monotonic(), stamp, node.get_clock().now().nanoseconds*1e-9-stamp))
        detail = dict(frame=msg.header.frame_id)
        if key == 'cloud':
            detail.update(fields=[dict(name=f.name, datatype=f.datatype, offset=f.offset, count=f.count) for f in msg.fields],
                          points=msg.width*msg.height, point_step=msg.point_step)
        else:
            detail.update(acceleration_norm=float(np.linalg.norm([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])),
                          gyro_norm=float(np.linalg.norm([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])))
        details[key] = detail
    node.create_subscription(PointCloud2, cloud_topic, lambda m: receive('cloud', m), qos_profile_sensor_data)
    node.create_subscription(Imu, imu_topic, lambda m: receive('imu', m), qos_profile_sensor_data)
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    report = {'duration_sec': seconds, 'read_only': True, 'streams': {},
              'limits': 'Topic visibility does not verify physical calibration, gravity convention, driver unite_imu_method=2, or positioning accuracy.'}
    for key, topic in [('cloud', cloud_topic), ('imu', imu_topic)]:
        s = samples[key]
        report['streams'][key] = dict(topic=topic, count=len(s), latest=details.get(key),
            receipt_hz=(len(s)-1)/(s[-1][0]-s[0][0]) if len(s)>1 else None,
            median_age_sec=float(np.median([v[2] for v in s])) if s else None,
            header_regressions=sum(s[i][1] <= s[i-1][1] for i in range(1, len(s))),
            publishers=[dict(node=p.node_name, type=p.topic_type, reliability=str(p.qos_profile.reliability),
                             durability=str(p.qos_profile.durability)) for p in node.get_publishers_info_by_topic(topic)])
    report['last_cloud_minus_imu_sec'] = samples['cloud'][-1][1]-samples['imu'][-1][1] if all(samples.values()) else None
    text = json.dumps(report, indent=2, allow_nan=False)
    print(text)
    if output:
        Path(output).write_text(text+'\n')
    node.destroy_node()
    rclpy.shutdown()
    if not all(samples.values()):
        raise SystemExit(2)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--seconds', type=float, default=10.)
    p.add_argument('--cloud-topic', default='/utlidar/cloud')
    p.add_argument('--imu-topic', default='/camera/camera/imu')
    p.add_argument('--output')
    a = p.parse_args()
    run(a.seconds, a.cloud_topic, a.imu_topic, a.output)
