#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import cv2 as cv

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt16MultiArray, String

from serial import Serial


def ros2cv(RImg: CompressedImage):
    if RImg is None:
        return None
    try:
        np_arr = np.frombuffer(RImg.data, np.uint8)
        cv_image = cv.imdecode(np_arr, cv.IMREAD_COLOR)
        return cv_image
    except Exception as e:
        return None


class OpenCVAprilTagNode(Node):
    
    class bytearr(bytearray):
        def __repr__(self):
            s= "[" 
            for x in self:
                s += f'{x:02X}, '
            s += ']'
            return s
        
        
        def __str__(self):
            return self.__repr__()
        
    def __init__(self):
        super().__init__('opencv_apriltag_node')

        # 订阅压缩图像
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.image_callback,
            10
        )
        self.subscription2 = self.create_subscription(
            CompressedImage,
            '/image',
            self.image_callback,
            10
        )

        # 发布检测到的 Tag 中心像素坐标
        self.publisher = self.create_publisher(
            UInt16MultiArray,
            '/apriltag/xy_pixels',
            10
        )

        # 初始化 OpenCV AprilTag 检测器
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_36h11)
        self.parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.get_logger().info('OpenCV AprilTag Node Initialized.')
        
        self.serial = Serial('/dev/ttyACM0', 115200)

    def image_callback(self, msg: CompressedImage):
        frame = ros2cv(msg)
        if frame is None:
            self.get_logger().warn('Failed to decode image.')
            return

        if frame.ndim != 3 or frame.shape[2] != 3:
            self.get_logger().warn('Invalid image format.')
            return

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        gray = cv.resize(gray, (0, 0), fx=0.5, fy=0.5)  # 缩小图像以提高处理速度

        # 检测 AprilTags
        corners, ids, _ = self.detector.detectMarkers(gray)

        data = []
        if ids is not None and len(ids) > 0:
            for i in range(len(ids)):
                pts = corners[i][0]  # shape: (4, 2)
                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))
                data.extend([cx, cy])

        if data:
            msg_out = UInt16MultiArray()
            msg_out.data = data
            self.publisher.publish(msg_out)

            # self.get_logger().info(f"Published {len(data)//2} tag(s): {data}")
            self.send(data)
            
    def send(self, data):
        cx = data[0]
        cxl8, cxh8 = cx>>8 & 0xFF, cx & 0xff
        sent = self.bytearr([0x31, cxl8, cxh8, 0x32])
        self.serial.write(sent)
        self.get_logger().info(f"Sent {sent}")

def main(args=None):
    rclpy.init(args=args)
    node = OpenCVAprilTagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
