#include "rossy_firmware/arduino_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace rossy_firmware
{

ArduinoSerialInterface::ArduinoSerialInterface()
{
}

ArduinoSerialInterface::~ArduinoSerialInterface()
{

    if (arduino_.IsOpen())
    {
        try
        {
            arduino_.Close();    
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinoSerialInterface"), "Something went wrong while closing the connection with the port" << port_);
        }
            
    }
    
    
}

CallbackReturn ArduinoSerialInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    try 
    {
        port_ = info_.hardware_parameters.at("port");
    }
    catch(const std::out_of_range &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("ArduinoSerialInterface"), "No serial port provided, aborting");
        return CallbackReturn::FAILURE;
    }

    velocity_commands_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());
    velocity_states_.reserve(info_.joints.size());

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinoSerialInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, 
                hardware_interface::HW_IF_POSITION, 
                &position_states_[i]
            )
        );

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, 
                hardware_interface::HW_IF_VELOCITY, 
                &velocity_states_[i]
            )
        );
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArduinoSerialInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, 
                hardware_interface::HW_IF_VELOCITY, 
                &velocity_commands_[i]
            )
        );
    }

    return command_interfaces;
}

CallbackReturn ArduinoSerialInterface::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
    RCLCPP_INFO(rclcpp::get_logger("ArduinoSerialInterface"), "Starting robot hardware");

    velocity_commands_ = {0.0, 0.0};
    position_states_ = {0.0, 0.0};
    velocity_states_ = {0.0, 0.0};
    r_wheel_counts_ = 0;
    l_wheel_counts_ = 0;
    last_right_counts_ = 0; // Inicializar contador previo de la rueda derecha
    last_left_counts_ = 0;  // Inicializar contador previo de la rueda izquierda

    try
    {
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    catch(...)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinoSerialInterface"), "Something went wrong while interacting with port " << port_);
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("ArduinoSerialInterface"), "Hardware started, ready to take commands");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ArduinoSerialInterface::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
    RCLCPP_INFO(rclcpp::get_logger("ArduinoSerialInterface"), "Stopping robot hardware");

    if (arduino_.IsOpen())
    {
        try
        {
            arduino_.Close();    
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinoSerialInterface"), "Something went wrong while closing the port" << port_);
            return CallbackReturn::FAILURE;
        }
            
    }

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArduinoSerialInterface::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
    if (arduino_.IsDataAvailable())
    {
        // Leer el mensaje del Arduino
        std::string message;
        arduino_.ReadLine(message);

        // Dividir el mensaje en partes
        std::stringstream ss(message);
        std::string res;

        while (std::getline(ss, res, ','))
        {
            if (res[0] == 'r') // Rueda derecha
            {
                // Leer contador acumulado
                int32_t counts = std::stoi(res.substr(1));
                int32_t diff_counts = counts - r_wheel_counts_; // Diferencia de pulsos
                r_wheel_counts_ = counts; // Actualizar el contador acumulado

                // Calcular posición y velocidad
                position_states_[0] += diff_counts * RADS_PER_COUNT;

                // Calcular el tiempo transcurrido correctamente
                auto new_time = std::chrono::system_clock::now();
                std::chrono::duration<double> diff = new_time - last_time_;
                double delta_seconds = diff.count();
                velocity_states_[0] = (diff_counts * RADS_PER_COUNT) / delta_seconds;
            }
            else if (res[0] == 'l') // Rueda izquierda
            {
                // Leer contador acumulado
                int32_t counts = std::stoi(res.substr(1));
                int32_t diff_counts = counts - l_wheel_counts_; // Diferencia de pulsos
                l_wheel_counts_ = counts; // Actualizar el contador acumulado

                // Calcular posición y velocidad
                position_states_[1] += diff_counts * RADS_PER_COUNT;

                // Calcular el tiempo transcurrido correctamente
                auto new_time = std::chrono::system_clock::now();
                std::chrono::duration<double> diff = new_time - last_time_;
                double delta_seconds = diff.count();
                velocity_states_[1] = (diff_counts * RADS_PER_COUNT) / delta_seconds;
            }
        }

        // Actualizar el tiempo de la última lectura
        last_time_ = std::chrono::system_clock::now();
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinoSerialInterface::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
    std::stringstream message_stream;

    char right_wheel_sign = velocity_commands_[0] >= 0 ? 'p' : 'n';
    char left_wheel_sign = velocity_commands_[1] >= 0 ? 'p' : 'n';

    std::string compensate_zeros_right = "";
    std::string compensate_zeros_left = "";

    if (std::abs(velocity_commands_[0]) < 10)
    {
        compensate_zeros_right = "0";
    }
    else
    {
        compensate_zeros_right = "";
    }

    if (std::abs(velocity_commands_[1]) < 10)
    {
        compensate_zeros_left = "0";
    }
    else
    {
        compensate_zeros_left = "";
    }

    message_stream << std::fixed << std::setprecision(2) <<
        "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_[0]) << 
        ",l" << left_wheel_sign << compensate_zeros_left << std::abs(velocity_commands_[1]) << ",";

    try
    {
        arduino_.Write(message_stream.str());
    }
    catch(...)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("ArduinoSerialInterface"), "Something went wrong while sending the message " << 
            message_stream.str() << " on the port " << port_);
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rossy_firmware::ArduinoSerialInterface,
    hardware_interface::SystemInterface);