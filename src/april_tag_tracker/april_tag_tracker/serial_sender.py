import rclpy
from rclpy.node import Node

from serial import Serial

from std_msgs.msg import UInt16MultiArray, String

class Sender(Node):
    class bytearr(bytearray):
        def __repr__(self):
            s= "[" 
            for x in self:
                s += f'{x:02X}, '
            s += ']'
            return s
        
        
        def __str__(self):
            return self.__repr__()
    
    def __init__(self, handle='/dev/ttyS1', baud=115200):
        super().__init__('opencv_apriltag_node')

        self.subscription = self.create_subscription(
            UInt16MultiArray,
            '/apriltag/xy_pixels',
            self.scan_callback,
            10
        )
        self.string_publisher = self.create_publisher(String, '/serial/send_string', 10)
        
        self.alive_timer = self.create_timer(1, self.iamalive)

        
        self.serial = Serial(handle, baud)
        
    def scan_callback(self, msg):
        data = list(msg.data)
        self.get_logger().info(f"Data: {data}")
        self.send(data)
        
    def send(self, data):
        cx = data[0]
        cxl8, cxh8 = cx>>8 & 0xFF, cx & 0xff
        sent = self.bytearr([0x31, cxl8, cxh8, 0x32])
        self.serial.write(sent)
        self.get_logger().info(f"Sent {sent}")
        print(f"Sent {sent}")
        
        string_msg = String()
        string_msg.data = repr(sent)
        self.string_publisher.publish(string_msg)
        
    def iamalive(self):
        self.get_logger().info("I am alive")
        
        
def main(args=None):
    rclpy.init(args=args)
    node = Sender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
