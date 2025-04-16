#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MenuPublicador(Node):
    def __init__(self):
        # Inicializar nodo usando super()
        super().__init__('menu_publicador')
        
        # Crear publicador para el topic del Arduino
        self.publisher = self.create_publisher(String, 'arduino_topic', 10)
        
        # Mensaje de inicio usando logger del nodo
        self.get_logger().info("Nodo Menú Inicializado")
        
        # Velocidad actual como atributo de instancia
        self.velocidad_actual = 0

    def publish_motor_command(self, valor):
        """Publica un comando de motor como cadena"""
        try:
            # Convertir a cadena y validar rango
            valor_str = str(int(valor))
            if 0 <= int(valor) <= 255:
                msg = String()
                msg.data = valor_str
                self.publisher.publish(msg)
                self.get_logger().info(f"Enviado comando de motor: {valor_str}")
            else:
                self.get_logger().error("Valor fuera de rango (0-255)")
        except ValueError:
            self.get_logger().error("Entrada inválida")

    def mostrar_menu(self):
        """Mostrar menú de control de motor en terminal"""
        print("\n--- Control de Motor Arduino ROS 2 ---")
        print("1. Introducir valor de motor (0-255)")
        print("2. Aumentar velocidad")
        print("3. Disminuir velocidad")
        print("4. Detener motor")
        print("5. Salir")
        print("Seleccione una opción: ", end='', flush=True)

    def run(self):
        """Bucle principal de control interactivo"""
        try:
            while rclpy.ok():
                self.mostrar_menu()
                
                # Leer entrada de teclado
                opcion = self.getch()
                
                if opcion == '1':
                    print("\nIntroduzca valor (0-255): ", end='', flush=True)
                    nuevo_valor = self.getch_numero()
                    if nuevo_valor is not None:
                        self.velocidad_actual = nuevo_valor
                        self.publish_motor_command(self.velocidad_actual)
                
                elif opcion == '2':
                    if self.velocidad_actual < 255:
                        self.velocidad_actual = min(self.velocidad_actual + 25, 255)
                        self.publish_motor_command(self.velocidad_actual)
                    print(f"\nVelocidad aumentada a: {self.velocidad_actual}")
                
                elif opcion == '3':
                    if self.velocidad_actual > 0:
                        self.velocidad_actual = max(self.velocidad_actual - 25, 0)
                        self.publish_motor_command(self.velocidad_actual)
                    print(f"\nVelocidad reducida a: {self.velocidad_actual}")
                
                elif opcion == '4':
                    self.velocidad_actual = 0
                    self.publish_motor_command(self.velocidad_actual)
                    print("\nMotor detenido")
                
                elif opcion == '5':
                    print("\nSaliendo del programa...")
                    break
                
                else:
                    print("\nOpción inválida. Intente de nuevo.")

        except KeyboardInterrupt:
            self.get_logger().info('Programa interrumpido por el usuario')

    def getch(self):
        """Leer un carácter sin esperar Enter"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def getch_numero(self):
        """Leer número de 0-255"""
        valor = ""
        while True:
            ch = self.getch()
            if ch.isdigit():
                valor += ch
                print(ch, end='', flush=True)
                if len(valor) == 3 or (len(valor) > 0 and int(valor) > 255):
                    break
            elif ch == '\r' or ch == '\n':
                break
            elif ch == '\x7f':  # Tecla Backspace
                if valor:
                    valor = valor[:-1]
                    print('\b \b', end='', flush=True)
        
        try:
            return int(valor) if valor else None
        except ValueError:
            return None

def main(args=None):
    # Inicializar rclpy
    rclpy.init(args=args)
    
    menu = MenuPublicador()
    menu.run()
    menu.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()