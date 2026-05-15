import math
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

storage_options = StorageOptions(
    uri='/home/althafsyed/ros2_ws/dwb_dynamic_run3/dwb_dynamic_run3_0.mcap',
    storage_id='mcap')
converter_options = ConverterOptions('', '')
reader = SequentialReader()
reader.open(storage_options, converter_options)

total_distance = 0.0
prev_x, prev_y = None, None
first_time = None
last_time = None
encounters = 0
last_encounter_time = None
CLOSE_DISTANCE = 1.0
COOLDOWN = 3.0

while reader.has_next():
    topic, data, timestamp = reader.read_next()
    if topic == '/odom':
        msg = deserialize_message(data, Odometry)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if prev_x is not None:
            total_distance += math.sqrt((x-prev_x)**2 + (y-prev_y)**2)
        prev_x, prev_y = x, y
        if first_time is None:
            first_time = timestamp
        last_time = timestamp
    if topic == '/scan':
        msg = deserialize_message(data, LaserScan)
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if not valid:
            continue
        min_range = min(valid)
        time_sec = timestamp / 1e9
        if min_range < CLOSE_DISTANCE:
            if last_encounter_time is None or time_sec - last_encounter_time > COOLDOWN:
                encounters += 1
                last_encounter_time = time_sec

travel_time = (last_time - first_time) / 1e9
print(f"DWB Dynamic Run 3 Metrics:")
print(f"Path distance: {total_distance:.3f} meters")
print(f"Travel time: {travel_time:.1f} seconds")
print(f"Human encounters: {encounters}")
