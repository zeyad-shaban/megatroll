import numpy as np
import cv2

def imgmsg_to_cv2(msg):
    """
    Converts a ROS 2 sensor_msgs/CompressedImage into an OpenCV BGR image matrix
    without using cv_bridge.
    """
    # 1. Convert the raw compressed byte buffer into a 1D NumPy array
    img_array = np.frombuffer(msg.data, dtype=np.uint8)
    
    # 2. Decompress the JPEG/PNG bytes directly into a standard BGR OpenCV matrix
    cv_image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    
    # 3. Quick safety check to ensure decoding didn't fail
    if cv_image is None:
        raise ValueError("Failed to decode CompressedImage data. Is the format corrupted?")
        
    return cv_image
    
def extract_largest_box(out_arr: np.ndarray, conf_thresh=0.25) -> tuple[dict, np.ndarray]:
    valid_cols = out_arr[:, out_arr[4, :] >= conf_thresh]
    if valid_cols.shape[1] == 0:
        return None, None
        
    valid_detections = valid_cols.T
    areas = valid_detections[:, 2] * valid_detections[:, 3]
    
    largest_idx = np.argmax(areas)
    row = valid_detections[largest_idx]
    
    return {
        "cx": float(row[0]),
        "cy": float(row[1]),
        "w": float(row[2]),
        "h": float(row[3]),
        "confidence": float(row[4])
    }, row

def clip_num(num: float, min_val, max_val):
    return max(min_val, min(num, max_val))