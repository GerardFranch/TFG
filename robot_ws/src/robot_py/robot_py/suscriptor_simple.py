# Importaciones necesarias
import rclpy                    # Librería principal de ROS2 para Python
from rclpy.node import Node    # Clase base para crear nodos
from std_msgs.msg import String # Tipo de mensaje String

class SuscriptorSimple(Node):
    def __init__(self):
        # Llamamos al constructor de la clase padre (Node) 
        # con el nombre "suscriptor_simple"
        super().__init__("suscriptor_simple")
        
        # Creamos la suscripción con estos parámetros:
        # 1. String: tipo de mensaje que vamos a recibir
        # 2. "chatter": nombre del topic al que nos suscribimos
        # 3. self.msgCallback: función que se ejecutará cuando llegue un mensaje
        # 4. 10: tamaño de la cola de mensajes
        self.sub_ = self.create_subscription(String, 
                                           "chatter",
                                           self.msgCallback, 
                                           10)

    def msgCallback(self, msg):
        # Función que se ejecuta cada vez que llega un mensaje
        # msg: es el mensaje recibido, de tipo String
        # Imprimimos el contenido del mensaje usando el logger
        self.get_logger().info("He escuchado: %s" % msg.data)

def main():
    # Inicializamos ROS2
    rclpy.init()
    
    # Creamos una instancia de nuestro nodo suscriptor
    suscriptor_simple = SuscriptorSimple()
    
    # Mantenemos el nodo ejecutándose
    rclpy.spin(suscriptor_simple)
    
    # Limpieza al terminar
    suscriptor_simple.destroy_node()
    rclpy.shutdown()

# Este if asegura que el código solo se ejecute si el script
# se ejecuta directamente (no si se importa como módulo)
if __name__ == "__main__":
    main()