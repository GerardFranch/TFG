#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from serial.serialutil import SerialException  

class SuscriptorSimpleSerial(Node):

    def __init__(self):
        super().__init__('suscriptor_simple_serial')

        # Declara los parámetros
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        
        # Obtiene los valores de los parámetros
        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        
        # Intenta abrir el puerto serial, pero maneja el error si no está disponible
        try:
            self.arduino = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.1)
            self.get_logger().info(f'Puerto serial {self.port} abierto correctamente')
        except SerialException:
            self.get_logger().warn(f'No se puede abrir el puerto serial {self.port}')
            self.arduino = None
        
        # Crea la suscripción
        self.subscription = self.create_subscription(
            String,
            'arduino_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('He recibido: "%s"' % msg.data)
        #self.arduino.write(msg.data.encode("utf-8"))


def main(args=None):
    rclpy.init(args=args)

    suscriptor_simple_serial = SuscriptorSimpleSerial()

    rclpy.spin(suscriptor_simple_serial)

    suscriptor_simple_serial.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()