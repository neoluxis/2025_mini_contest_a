import cv2 as cv
import threading
import time
import numpy as np
from datetime import datetime

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String


class Frame:
    def __init__(self, image, timestamp):
        self.image = image
        self.timestamp = timestamp


class ThreadCap:
    def __init__(self, camera_index=0, width=640, height=400, fps=240):
        self.cap = cv.VideoCapture(camera_index)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc(*"MJPG"))
        self.cap.set(cv.CAP_PROP_FPS, fps)

        self.frame = None
        self.last_frame = None
        self.lock = threading.Lock()
        self.stop_flag = False

        self.thread = threading.Thread(target=self._update_frame, daemon=True)
        self.thread.start()

    def _update_frame(self):
        while not self.stop_flag:
            ret, frame_gray = self.cap.read()
            if ret:
                # frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

                if not self.last_frame or not self._are_frames_similar(
                    frame_gray, self.last_frame.image
                ):
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                    frame_with_timestamp = self._add_timestamp(frame_gray, timestamp)

                    with self.lock:
                        self.frame = Frame(frame_with_timestamp, timestamp)
                        self.last_frame = Frame(frame_gray, timestamp)

    def _compute_mse(self, imageA, imageB):
        """计算两张图片的均方误差"""
        err = np.sum((imageA.astype("float") - imageB.astype("float")) ** 2)
        err /= float(imageA.shape[0] * imageA.shape[1])
        return err

    def _are_frames_similar(self, frame1, frame2, threshold=10):
        """判断两张图片是否相似"""
        mse = self._compute_mse(frame1, frame2)
        return mse < threshold

    def _add_timestamp(self, frame, timestamp):
        """在图像上添加时间戳"""
        font = cv.FONT_HERSHEY_SIMPLEX
        # cv.putText(frame, timestamp, (10, 30), font, 1, (255), 2, cv.LINE_AA)
        return frame

    def read(self):
        with self.lock:
            frame_copy = self.frame.image.copy() if self.frame is not None else None
            timestamp = self.frame.timestamp if self.frame is not None else None
        return (timestamp, frame_copy)

    def isOpened(self):
        return self.cap.isOpened()

    def release(self):
        self.stop_flag = True
        self.thread.join()
        self.cap.release()


def cv2ros(Img):
    rimg = cv.imencode(".jpeg", Img)[1].tobytes()
    return rimg


class VideoCap(Node):
    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter("cam_idx", "/dev/video0")
        self.declare_parameter("fps", 120)
        self.declare_parameter("img_width", 640)
        self.declare_parameter("img_height", 480)
        # self.declare_parameter("img_fourcc", cv.VideoWriter.fourcc(*"MJPG"))

        self.get_logger().info(f"VideoCap Node {name}")
        self.cam = ThreadCap(
            self.get_parameter("cam_idx").get_parameter_value().string_value,
            self.get_parameter("img_width").get_parameter_value().integer_value,
            self.get_parameter("img_height").get_parameter_value().integer_value,
            self.get_parameter("fps").get_parameter_value().integer_value,
        )
        self.get_logger().info(
            f'Using {self.get_parameter("cam_idx").get_parameter_value().integer_value} for QRCcode Scan'
        )

        # self.cam = cv.VideoCapture(
        #     self.get_parameter("cam_idx").get_parameter_value().integer_value
        # )
        # self.cam.set(
        #     cv.CAP_PROP_FRAME_WIDTH,
        #     self.get_parameter("img_width").get_parameter_value().integer_value,
        # )
        # self.cam.set(
        #     cv.CAP_PROP_FRAME_HEIGHT,
        #     self.get_parameter("img_height").get_parameter_value().integer_value,
        # )
        # self.cam.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc(*"MJPG"))
        # self.cam.set(
        #     cv.CAP_PROP_FPS,
        #     self.get_parameter("fps").get_parameter_value().integer_value,
        # )

        # self.qrc_image_pub = self.create_publisher(Image, "qrc_image", 10)
        self.qrc_image_pub = self.create_publisher(CompressedImage, "/camera/image/compressed", 10)

        timer_period = 0.01  # seconds
        self.qrc_image_pub_timer = self.create_timer(
            timer_period, self.qrc_image_pub_callback
        )
        self.msg = CompressedImage()

        self.frame_count = 0
        self.fps_timer = self.create_timer(1, self.fps_callback)

    def qrc_image_pub_callback(self):
        _, frame = self.cam.read()
        if frame is None:
            self.get_logger().info("Get None Pic")
            return
        # frame = cv.resize(frame, (0, 0), fx=0.35, fy=0.35)
        # cv.imshow("fr", frame)
        # cv.waitKey(1)
        self.msg.data = cv2ros(frame)
        # self.msg.width = frame.shape[1]
        # self.msg.height = frame.shape[0]
        # self.msg.encoding = "jpeg"
        # self.msg.step = frame.shape[1] * 2
        self.qrc_image_pub.publish(self.msg)
        self.frame_count += 1

    def fps_callback(self):
        self.get_logger().info(f"VideoCap: Pub FPS: {self.frame_count}")
        self.frame_count = 0



def main():
    rclpy.init()
    qrccam = VideoCap("qrc_cam")
    rclpy.spin(qrccam)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
