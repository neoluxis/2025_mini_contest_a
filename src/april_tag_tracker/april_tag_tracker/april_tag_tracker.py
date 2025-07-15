#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import cv2 as cv

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt16MultiArray


def ros2cv(RImg: CompressedImage):
    if RImg is None:
        return None
    np_arr = np.frombuffer(RImg.data, np.uint8)
    cv_image = cv.imdecode(np_arr, cv.IMREAD_COLOR)
    return cv_image


class OpenCVAprilTagNode(Node):
    def __init__(self):
        super().__init__('opencv_apriltag_node')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(UInt16MultiArray, '/apriltag/xy_pixels', 10)

        # 使用 OpenCV 的 AprilTag 字典
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_36h11)

    def image_callback(self, msg: CompressedImage):
        frame = ros2cv(msg)
        if frame is None:
            self.get_logger().warn('Failed to decode image')
            return

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # 检测 AprilTags
        corners, ids, _ = cv.aruco.detectMarkers(gray, self.dictionary)

        data = []
        if ids is not None:
            for i in range(len(ids)):
                pts = corners[i][0]  # 4个角点坐标
                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))
                data.extend([cx, cy])

        msg_out = UInt16MultiArray()
        msg_out.data = data
        self.publisher.publish(msg_out)

        self.get_logger().info(f"Published {len(data)//2} tag(s): {data}")


def main(args=None):
    rclpy.init(args=args)
    node = OpenCVAprilTagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
