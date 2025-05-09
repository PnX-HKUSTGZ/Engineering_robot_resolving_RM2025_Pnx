#include "engineering_robot_controller/engineering_robot_controller.hpp"
namespace Engineering_robot_RM2025_Pnx{

void Engineering_robot_Controller::set_player_command(const PlayerCommandContent & input_command){
    std::lock_guard<std::mutex> ul(player_command_mutex);
    player_command=input_command;
}

void Engineering_robot_Controller::set_computer_state(const ComputerState & input_state){
    std::lock_guard<std::mutex> ul(player_command_mutex);
    computer_state=input_state;
}

void Engineering_robot_Controller::player_command_sub_callback(const command_interfaces::msg::PlayerCommand::ConstSharedPtr & msg){
    PlayerCommandContent input_command;
    input_command.command_time=msg->header.stamp;
    input_command.breakout=msg->breakout;
    input_command.is_finish=msg->is_finish;
    input_command.is_started=msg->is_started;
    input_command.is_attach=msg->is_attach;

    set_player_command(input_command);
}

void Engineering_robot_Controller::computer_state_pub_callback(){
    command_interfaces::msg::ComputerState msg;
    auto current_state=get_computer_state();
    msg.header.frame_id="/computer";
    msg.header.stamp=this->now();
    msg.current_state=current_state.current_state;
    msg.pos1_state=current_state.pos1_state;
    msg.pos2_state=current_state.pos2_state;
    msg.pos3_state=current_state.pos3_state;
    computer_state_pub_->publish(msg);
}

PlayerCommandContent Engineering_robot_Controller::get_player_command(){
    std::lock_guard<std::mutex> ul(player_command_mutex);
    return player_command;
}

ComputerState Engineering_robot_Controller::get_computer_state(){
    std::lock_guard<std::mutex> ul(computer_state_mutex);
    return computer_state;
}

}// namespace Engineering_robot_RM2025_Pnx