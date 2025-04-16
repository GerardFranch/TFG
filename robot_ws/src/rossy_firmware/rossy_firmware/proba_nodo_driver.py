#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import sys
import tty
import termios
import select
import time
import threading

class ArduinoRobotController(Node):
    def __init__(self):
        super().__init__('arduino_robot_controller')
        
        # Parámetros de conexión serie
        self.serial_port = '/dev/ttyACM0'  # Puerto por defecto, modificar según sea necesario
        self.baud_rate = 115200
        
        # Velocidades predefinidas
        self.speed_low = 1.0
        self.speed_medium = 2.0
        self.speed_high = 3.0
        self.current_speed = self.speed_medium
        
        # Estado del robot
        self.left_wheel_speed = 0.0
        self.right_wheel_speed = 0.0
        
        # Intentar abrir la conexión serial
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Conectado al puerto {self.serial_port} a {self.baud_rate} baudios")
        except Exception as e:
            self.get_logger().error(f"Error al conectar con el puerto serie: {e}")
            sys.exit(1)
        
        # Iniciar los hilos para lectura de teclado y monitor de serial
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.read_keyboard)
        self.serial_monitor_thread = threading.Thread(target=self.monitor_serial)
        
        self.keyboard_thread.daemon = True
        self.serial_monitor_thread.daemon = True
        
        self.keyboard_thread.start()
        self.serial_monitor_thread.start()
        
        # Mostrar menú inicial
        self.show_menu()

    def show_menu(self):
        """Muestra el menú de control en la terminal"""
        self.get_logger().info("\n\n==== CONTROL DE ROBOT ====")
        self.get_logger().info("Usa las flechas del teclado para controlar el robot:")
        self.get_logger().info("↑ : Adelante")
        self.get_logger().info("↓ : Atrás")
        self.get_logger().info("← : Girar izquierda")
        self.get_logger().info("→ : Girar derecha")
        self.get_logger().info("Espacio : Detener")
        self.get_logger().info("1-3: Ajustar velocidad (1:baja, 2:media, 3:alta)")
        self.get_logger().info("q : Salir")
        self.get_logger().info("===========================")

    def getch(self):
        """Lee un solo carácter del teclado sin necesidad de Enter"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def read_keyboard(self):
        """Lee las teclas presionadas y envía comandos al Arduino"""
        while self.running:
            # Comprobar si hay una tecla disponible
            if select.select([sys.stdin], [], [], 0)[0]:
                key = self.getch()
                
                if key == '\x1b':  # Carácter de escape (inicio de secuencia de tecla de flecha)
                    # Leer los siguientes dos caracteres de la secuencia
                    next1, next2 = self.getch(), self.getch()
                    
                    # Procesar teclas de flecha
                    if next1 == '[':
                        if next2 == 'A':  # Flecha arriba
                            self.move_forward()
                        elif next2 == 'B':  # Flecha abajo
                            self.move_backward()
                        elif next2 == 'C':  # Flecha derecha
                            self.turn_right()
                        elif next2 == 'D':  # Flecha izquierda
                            self.turn_left()
                
                elif key == ' ':  # Espacio para detener
                    self.stop()
                elif key == '1':  # Velocidad baja
                    self.current_speed = self.speed_low
                    self.get_logger().info(f"Velocidad: Baja ({self.current_speed})")
                elif key == '2':  # Velocidad media
                    self.current_speed = self.speed_medium
                    self.get_logger().info(f"Velocidad: Media ({self.current_speed})")
                elif key == '3':  # Velocidad alta
                    self.current_speed = self.speed_high
                    self.get_logger().info(f"Velocidad: Alta ({self.current_speed})")
                elif key == 'q':  # Salir
                    self.running = False
                    self.get_logger().info("Saliendo...")
                    self.stop()  # Detener el robot antes de salir
                    break
            
            time.sleep(0.1)  # Pequeña pausa para no saturar la CPU

    def monitor_serial(self):
        """Lee y procesa los mensajes recibidos del Arduino"""
        while self.running:
            if self.serial_conn.in_waiting > 0:
                try:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    # Procesar datos de encoders (formato "rp0.00,ln0.00,")
                    if line.startswith('r') and ',' in line:
                        self.get_logger().debug(f"Recibido: {line}")
                except Exception as e:
                    self.get_logger().error(f"Error al leer datos: {e}")
            
            time.sleep(0.1)

    def send_command(self, motor, direction, speed):
        """Envía un comando al Arduino"""
        command = f"{motor}{direction}{speed:.2f},"
        self.get_logger().debug(f"Enviando: {command}")
        self.serial_conn.write(command.encode('utf-8'))

    def move_forward(self):
        """Mover el robot hacia adelante"""
        self.left_wheel_speed = self.current_speed
        self.right_wheel_speed = self.current_speed
        self.send_command('r', 'p', self.right_wheel_speed)
        self.send_command('l', 'p', self.left_wheel_speed)
        self.get_logger().info("Adelante")

    def move_backward(self):
        """Mover el robot hacia atrás"""
        self.left_wheel_speed = self.current_speed
        self.right_wheel_speed = self.current_speed
        self.send_command('r', 'n', self.right_wheel_speed)
        self.send_command('l', 'n', self.left_wheel_speed)
        self.get_logger().info("Atrás")

    def turn_left(self):
        """Girar el robot hacia la izquierda"""
        self.left_wheel_speed = self.current_speed / 2
        self.right_wheel_speed = self.current_speed
        self.send_command('r', 'p', self.right_wheel_speed)
        self.send_command('l', 'p', self.left_wheel_speed / 2)  # Rueda izquierda más lenta o en dirección opuesta
        self.get_logger().info("Izquierda")

    def turn_right(self):
        """Girar el robot hacia la derecha"""
        self.left_wheel_speed = self.current_speed
        self.right_wheel_speed = self.current_speed / 2
        self.send_command('r', 'p', self.right_wheel_speed / 2)  # Rueda derecha más lenta o en dirección opuesta
        self.send_command('l', 'p', self.left_wheel_speed)
        self.get_logger().info("Derecha")

    def stop(self):
        """Detener el robot"""
        self.left_wheel_speed = 0.0
        self.right_wheel_speed = 0.0
        self.send_command('r', 'p', 0.0)
        self.send_command('l', 'p', 0.0)
        self.get_logger().info("Detenido")


def main(args=None):
    rclpy.init(args=args)
    controller = ArduinoRobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.running = False
        controller.keyboard_thread.join(timeout=1.0)
        controller.serial_monitor_thread.join(timeout=1.0)
        controller.serial_conn.close()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()