# ros2 bag record /camera/image/compressed
import os
import cv2
import numpy as np

from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import CompressedImage

# ===== config =====
bag_path = "rosbag2_2026_04_23-05_42_45"
output_dir = "frames"
topic_name = "/camera/image/compressed"
step_every = 1   # save every Nth frame (1 = all frames, 2 = every other frame, etc.)

os.makedirs(output_dir, exist_ok=True)

reader = SequentialReader()
storage_options = StorageOptions(uri=bag_path, storage_id="mcap")
converter_options = ConverterOptions("", "")
reader.open(storage_options, converter_options)

frame_id = 0
msg_index = 0

while reader.has_next():
    topic, data, t = reader.read_next()

    if topic != topic_name:
        continue

    if msg_index % step_every != 0:
        msg_index += 1
        continue

    msg = deserialize_message(data, CompressedImage)

    np_arr = np.frombuffer(msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    if img is not None:
        filename = os.path.join(output_dir, f"frame_{frame_id:05d}.jpg")
        cv2.imwrite(filename, img)
        frame_id += 1

    msg_index += 1

print(f"Saved {frame_id} frames to '{output_dir}'")