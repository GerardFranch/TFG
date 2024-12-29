// Incluimos las cabeceras necesarias
#include <rclcpp/rclcpp.hpp>        // Librería principal de ROS2
#include <std_msgs/msg/string.hpp>   // Para usar mensajes de tipo String

using std::placeholders::_1;         // Para usar std::bind con un argumento

// Definimos una clase que hereda de rclcpp::Node
class SimpleSubscriber : public rclcpp::Node {
public:
    // Constructor de la clase
    SimpleSubscriber() : Node("simple_subscriber") {
        // Creamos la suscripción con estos parámetros:
        // 1. "chatter": nombre del topic
        // 2. 10: tamaño de la cola de mensajes
        // 3. std::bind: vincula la función callback con this (objeto actual) y _1 (el mensaje)
        sub_ = create_subscription<std_msgs::msg::String>(
            "chatter", 
            10, 
            std::bind(&SimpleSubscriber::msgCallback, this, _1)
        );
    }

private:
    // Declaramos el suscriptor como variable miembro privada
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    // Función callback que se ejecuta cuando llega un mensaje
    void msgCallback(const std_msgs::msg::String &msg) const {
        // Imprime el mensaje recibido usando el logger de ROS
        RCLCPP_INFO_STREAM(get_logger(), "I heard: " << msg.data.c_str());
    }
};

// Función principal
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);    // Inicializamos ROS2
    
    // Creamos una instancia compartida de nuestro nodo
    auto node = std::make_shared<SimpleSubscriber>();
    
    // Mantenemos el nodo ejecutándose
    rclcpp::spin(node);
    
    // Cerramos ROS2 limpiamente
    rclcpp::shutdown();
    return 0;
}