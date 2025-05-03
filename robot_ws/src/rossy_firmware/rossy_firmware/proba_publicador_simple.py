#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class PublicadorSimpleSerial(Node):

    def __init__(self):
        super().__init__('publicador_simple_serial')
        self.publisher_ = self.create_publisher(String, 'arduino_topic', 10)
        timer_period = 0.5  # seconds
        
        self.declare_parameter("port","/dev/arduino")
        self.declare_parameter("baudrate",115200)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        
        self.arduino = serial.Serial(port=self.port,baudrate=self.baudrate)

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if rclpy.ok() and self.arduino.is_open:
            data = self.arduino.readline()
            try:
                data.decode("utf-8")
            except:
                return
            msg = String()
            msg.data = str(data)
            self.publisher_.publish(msg)
            self.get_logger().info('Recibido: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    publicador_simple_serial = PublicadorSimpleSerial()

    rclpy.spin(publicador_simple_serial)
    publicador_simple_serial.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()