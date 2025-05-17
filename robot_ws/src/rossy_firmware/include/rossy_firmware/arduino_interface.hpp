#ifndef ARDUINO_INTERFACE_HPP
#define ARDUINO_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <vector>
#include <string>

namespace rossy_firmware
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class ArduinoSerialInterface: public hardware_interface::SystemInterface
    {
    public:
        ArduinoSerialInterface();
        virtual ~ArduinoSerialInterface();

        virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;

        virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    private:
        LibSerial::SerialPort arduino_;
        std::string port_;
        std::vector<double> velocity_commands_;
        std::vector<double> position_states_;
        std::vector<double> velocity_states_;

        // Constantes para el cálculo de posición
        const double COUNTS_PER_REVOLUTION = 824.0;
        const double RADS_PER_COUNT = 2.0 * M_PI / COUNTS_PER_REVOLUTION;

        // Variables para almacenar los contadores acumulados
        int32_t r_wheel_counts_ = 0;
        int32_t l_wheel_counts_ = 0;

        // Variables para almacenar los contadores previos
        int32_t last_right_counts_ = 0;
        int32_t last_left_counts_ = 0;

        // Tiempo de la última lectura
        std::chrono::time_point<std::chrono::system_clock> last_time_ = std::chrono::system_clock::now();
    };
}

#endif