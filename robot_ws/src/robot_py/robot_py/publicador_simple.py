# Importamos la librería rclpy que contiene las funciones necesarias para interactuar con ROS 2 en Python
import rclpy
# Desde el módulo 'node' de rclpy, importamos la clase Node que será la base de nuestro nodo ROS 2
from rclpy.node import Node
# Importamos la definición del mensaje String desde la librería estándar de mensajes en ROS 2
from std_msgs.msg import String

# Definimos una nueva clase llamada SimplePublisher que hereda de la clase Node
class PublicadorSimple(Node):
    # Definimos el constructor, que se ejecuta automaticamente cuando se crea un nuevo objeto de la clase
    # Inicializa los atributos del objeto 
    def __init__(self):
        # Inicializamos la clase base (Node) usando el constructor del nodo y asignamos un nombre al nodo
        super().__init__('publicador_simple')

        # Creamos un objeto publisher que publicará mensajes del tipo String en el tópico llamado "chatter"
        # El tamaño del buffer se define como 10 para almacenar mensajes en caso de que el suscriptor no procese rápido
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Definimos una variable contador que llevará la cuenta de los mensajes publicados
        self.counter_ = 0
        # Definimos la frecuencia a la que queremos publicar mensajes (1 Hz en este caso)
        self.frequency_ = 1.0

        # Registramos un mensaje informativo en el logger indicando la frecuencia de publicación
        self.get_logger().info(f'Publishing at {self.frequency_} Hz')

        # Creamos un temporizador que ejecutará una función de callback (timer_callback) a intervalos regulares
        self.timer_ = self.create_timer(self.frequency_, self.timer_callback)

    # Esta función será llamada cada vez que el temporizador expire
    def timer_callback(self):
        # Creamos un nuevo mensaje del tipo String
        msg = String()
        # Asignamos el contenido del mensaje al atributo "data", incluyendo un contador de los mensajes enviados
        msg.data = f'Hello ROS 2: {self.counter_}'

        # Publicamos el mensaje creado en el tópico "chatter" usando el objeto publisher
        self.publisher_.publish(msg)

        # Mostramos un mensaje en el logger indicando que se ha publicado un mensaje
        self.get_logger().info(f'Published: "{msg.data}"')

        # Incrementamos el contador para reflejar que se ha publicado un nuevo mensaje
        self.counter_ += 1

# Punto de entrada principal del script
def main():
    # Inicializamos la interfaz de ROS 2
    rclpy.init()

    # Creamos una instancia del nodo SimplePublisher
    publicador_simple = PublicadorSimple()

    # Mantenemos el nodo activo usando el método spin, que lo hace funcional hasta que lo detengamos manualmente
    rclpy.spin(publicador_simple)
    publicador_simple.destroy_node()
    rclpy.shutdown()

# Este if hace que la función main() solo se ejecute cuando el script, sea el programa principal
# Si es el programa principal, __name__ tiene el valor de __main__
# Si el script es importado como un módulo, __name__ tendrá el valor del nombre del archivo
if __name__ == "__main__":
    main()
