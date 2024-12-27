// Incluimos las cabeceras necesarias
#include <rclcpp/rclcpp.hpp>// Librería principal de ROS2
#include <std_msgs/msg/string.hpp>// Para usar mensajes de tipo String
#include <chrono>// Para manejar tiempo

using namespace std::chrono_literals;// Para usar literales de tiempo como '1s'

// Definimos una clase que hereda de rclcpp::Node
class SimplePublisher : public rclcpp::Node {
public:
// Constructor de la clase
    SimplePublisher() : Node("simple_publisher"), counter_(0) {
// Creamos un publicador que enviará mensajes String en el topic "chatter"// El '10' es el tamaño de la cola de mensajes
        publisher_ = create_publisher<std_msgs::msg::String>("chatter", 10);

// Creamos un timer que llamará a timer_callback cada segundo
        timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timer_callback, this));

// Imprimimos un mensaje informativo usando el logger de ROS
        RCLCPP_INFO(get_logger(), "Publicando a 1 Hz");
    }

private:
// Variables miembro
    unsigned int counter_;// Contador para llevar la cuenta de mensajes
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;// Puntero al publicador
    rclcpp::TimerBase::SharedPtr timer_;// Puntero al timer// Función que se ejecuta cada vez que el timer se activa
    void timer_callback() {
// Creamos un nuevo mensaje
        auto message = std_msgs::msg::String();
// Añadimos datos al mensaje
        message.data = "Hola ROS2, contador: " + std::to_string(counter_++);
// Publicamos el mensaje
        publisher_->publish(message);
    }
};

// Función principal
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);// Inicializamos ROS2

// Creamos una instancia de nuestro nodo
    auto node = std::make_shared<SimplePublisher>();

// Mantenemos el nodo ejecutándose
    rclcpp::spin(node);

// Cerramos ROS2 limpiamente
    rclcpp::shutdown();
    return 0;
}

