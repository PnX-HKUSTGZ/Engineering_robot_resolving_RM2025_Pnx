#include "crc.hpp"
#include "packet.hpp"
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <serial_driver/serial_driver.hpp>
#include <iostream>
#include <thread>
#include <map>
#include <rclcpp/rclcpp.hpp>

#ifndef engineering_robot_hardware_interface
#define engineering_robot_hardware_interface

namespace Engineering_robot_RM2025_Pnx{

using hardware_interface::HardwareInfo;
using CallbackReturn=rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using hardware_interface::StateInterface;
using hardware_interface::CommandInterface;
using  hardware_interface::return_type;
using FlowControl = drivers::serial_driver::FlowControl;
using Parity = drivers::serial_driver::Parity;
using StopBits = drivers::serial_driver::StopBits;

class ERHardwareInterface : public hardware_interface::SystemInterface{

public:

// no content
void on_configure();
// no content
void on_cleanup();
// try reload
void on_shutdown();

// no content
void on_activate();
// no content
void on_deactivate();
// try reload
void on_error();

void reopen();
void open();

CallbackReturn on_init(const HardwareInfo & hardware_info) override;
std::vector<StateInterface> export_state_interfaces() override;
std::vector<CommandInterface> export_command_interfaces() override;
return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

rclcpp::Logger logger=rclcpp::get_logger("RHardwareInterface");
std::unique_ptr<IoContext> owned_ctx_;
std::string device_name_="/dev/pts/17";
// 顺序和串口包的顺序一样
std::vector<std::string> joint_name={"J1_pris_","J1_rotate_","J2_","J3_","J4_x_","J4_y_"};
std::vector<std::string> interface_name={"position","velocity"};
std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

// get from read
double state_data[6];
// get from read
float v[6];
// 位置
double WID_p[6];
// 速度
double WID_v[6];

uint32_t baud_rate=115200;
FlowControl fc = FlowControl::NONE;
Parity pt = Parity::NONE;
StopBits sb = StopBits::ONE;

};// ERHardwareInterface

}//Engineering_robot_RM2025_Pnx

#endif  //engineering_robot_hardware_interface