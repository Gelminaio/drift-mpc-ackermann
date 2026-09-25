import sys
import os
import pandas as pd
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# topics we care about for identification, and how to flatten each message
FIELDS = {
    '/joint_states': lambda m: {
        'wl': m.velocity[0] if len(m.velocity) > 0 else float('nan'),
        'wr': m.velocity[1] if len(m.velocity) > 1 else float('nan'),
        'pl': m.position[0] if len(m.position) > 0 else float('nan'),
        'pr': m.position[1] if len(m.position) > 1 else float('nan'),
    },
    '/steering_angle': lambda m: {'steer': m.data},
    '/imu/data_raw': lambda m: {
        'ax': m.linear_acceleration.x, 'ay': m.linear_acceleration.y, 'az': m.linear_acceleration.z,
        'gx': m.angular_velocity.x, 'gy': m.angular_velocity.y, 'gz': m.angular_velocity.z,
    },
    '/odom': lambda m: {
        'x': m.pose.pose.position.x, 'y': m.pose.pose.position.y,
        'vx': m.twist.twist.linear.x, 'wz': m.twist.twist.angular.z,
    },
    '/drive': lambda m: {'cmd_steer': m.steering_angle, 'cmd_speed': m.speed},
    '/cmd_vel': lambda m: {'cmd_vx': m.linear.x, 'cmd_wz': m.angular.z},
}


def extract(bag_path):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id='mcap'),
                ConverterOptions('', ''))

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    rows = {topic: [] for topic in FIELDS}

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic not in FIELDS:
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        row = {'t': t * 1e-9}
        row.update(FIELDS[topic](msg))
        rows[topic].append(row)

    base = bag_path.rstrip('/')
    name = os.path.basename(base)
    out_dir = os.path.join(os.path.dirname(base), name + '_parquet')
    os.makedirs(out_dir, exist_ok=True)

    for topic, data in rows.items():
        if not data:
            continue
        df = pd.DataFrame(data)
        fname = topic.strip('/').replace('/', '_') + '.parquet'
        df.to_parquet(os.path.join(out_dir, fname))
        print(f"{topic}: {len(df)} rows -> {fname}")

    print(f"written to {out_dir}")


if __name__ == '__main__':
    extract(sys.argv[1])