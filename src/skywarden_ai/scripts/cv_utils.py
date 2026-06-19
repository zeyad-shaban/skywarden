import numpy as np

def imgmsg_to_cv2(msg):
    if msg.encoding == "bgr8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    elif msg.encoding == "rgb8": # it's this
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    elif msg.encoding == "mono8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
    else:
        raise ValueError(f"Unsupported encoding: {msg.encoding}")