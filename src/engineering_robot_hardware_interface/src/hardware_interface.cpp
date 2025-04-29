#include "engineering_robot_hardware_interface/hardware_interface.hpp"

namespace Engineering_robot_RM2025_Pnx{

CallbackReturn ERHardwareInterface::on_init(const HardwareInfo & hardware_info){
    try{
        owned_ctx_=std::make_unique<drivers::common::IoContext>(2);
        serial_driver_=std::make_unique<drivers::serial_driver::SerialDriver>(*owned_ctx_);
    }
    catch(const std::exception & e){
        std::cerr<<"Load drivers::serial_driver::SerialDriver failed with "<<e.what()<<"\n";
        return CallbackReturn::ERROR;
    }

    bool noerror=0;
    while(!noerror){
        try{
            serial_driver_->init_port(device_name_, *device_config_);
            if (!serial_driver_->port()->is_open()){
                serial_driver_->port()->open();
            }
            noerror=1;
        }
        catch (const std::exception &ex){
            std::cerr<<"open Serial failed with "<<ex.what()<<", wait 200 ms.\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            noerror=0;
        }
    }

    std::map<std::string,bool> joiny_interface;
    for(auto & i : hardware_info.joints){
        joiny_interface[i.name]=1;
    }
    for(auto & i : joint_name){
        if(!joiny_interface[i]) return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

void ERHardwareInterface::on_configure(){
    std::cout<<"on_configure called!\n";
}

void ERHardwareInterface::on_cleanup(){
    std::cout<<"on_cleanup  called!\n";
}

std::vector<StateInterface> ERHardwareInterface::export_state_interfaces(){
    std::vector<StateInterface> state;

    for(int i=0;i<6;i++){
        state.push_back(StateInterface(joint_name[i],"position",&state_data[i]));
        std::cout<<"add state:"<<joint_name[i]<<"/"<<"position"<<std::endl;
    }

    return state;
}

std::vector<CommandInterface> ERHardwareInterface::export_command_interfaces(){
    std::vector<CommandInterface> inter;
    for(int i=0;i<6;i++){
        inter.push_back(CommandInterface(joint_name[i],"position",&WID_p[i]));
        inter.push_back(CommandInterface(joint_name[i],"velocity",&WID_v[i]));
        std::cout<<"add state:"<<joint_name[i]<<std::endl;
    }
    return inter;
}

bool ERHardwareInterface::read_from_serial(double target[6]){
    std::vector<uint8_t> header(1);
    std::vector<uint8_t> data;
    NowPosition packet;
    data.reserve(sizeof(NowPosition));
    try {
        serial_driver_->port()->receive(header);
        if (header[0] == 0x5A) {
            data.resize(sizeof(NowPosition) - 1);
            serial_driver_->port()->receive(data);

            data.insert(data.begin(), header[0]);
            fromVector(data,packet);

            bool crc_ok = Engineering_robot_RM2025_Pnx::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
            if (!crc_ok) {
                std::cerr<<"crc check fail!"<<std::endl;
                throw "crc check fail!";
                return 0;
            }
        }
    }
    catch(std::exception & e){
        std::cerr<<"read_from_serial fialed with "<<e.what()<<std::endl;
        throw e;
        return 0;
    }

    for(int i=0;i<6;i++){
        target[i]=packet.pos[i];
    }
    target[0]=-target[0];
    std::cout<<"read_from_serial ok!"<<std::endl;
    return 1;
}


return_type ERHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period){
    std::cout<<"call read at"<<time.seconds()<<","<<time.nanoseconds()<<std::endl;
    try{
        read_from_serial(state_data);
    }
    catch(const std::exception & e){
        std::cerr<<"read_from_serial failed with "<<e.what()<<std::endl;
        return return_type::ERROR;
    }
    return return_type::OK;
}

return_type ERHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period){
    SendDate packet;
    packet.reserved=0;
    std::cout<<"write called!"<<std::endl;
    for(int i=0;i<6;i++){
        packet.pos[i]=WID_p[i];
        packet.v[i]=WID_v[i];
        std::cout<<"add "<<WID_p[i]<<","<<WID_v[i]<<std::endl;
    }
    uint16_t crc16 = Get_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(SendDate), 0xFFFF);
    packet.crc16=crc16;
    std::cout<<"crc "<<crc16<<std::endl;
    std::vector<uint8_t> data = toVector(packet);

    try{
        serial_driver_->port()->send(data);
    }
    catch(std::exception & e){
        std::cerr<<"serial_driver_->port()->send() failed with "<<e.what()<<std::endl;
        return return_type::ERROR;
    }

    std::cout<<"write finish!"<<std::endl;
    return return_type::OK;

}

void ERHardwareInterface::on_activate(){

}

void ERHardwareInterface::on_deactivate(){

}

void ERHardwareInterface::on_error(){
    if(serial_driver_->port()->is_open()){
        serial_driver_->port()->close();
    }
    if(owned_ctx_){
        owned_ctx_->waitForExit();
    }
    try{
        owned_ctx_=std::make_unique<drivers::common::IoContext>(new IoContext(2));
        serial_driver_=std::make_unique<drivers::serial_driver::SerialDriver>(new drivers::serial_driver::SerialDriver(*owned_ctx_));
    }
    catch(const std::exception & e){
        std::cerr<<"Load drivers::serial_driver::SerialDriver failed with "<<e.what()<<"\n";
    }

    bool noerror=0;
    while(!noerror){
        try{
            serial_driver_->init_port(device_name_, *device_config_);
            if (!serial_driver_->port()->is_open()){
                serial_driver_->port()->open();
            }
            noerror=1;
        }
        catch (const std::exception &ex){
            std::cerr<<"open Serial failed with "<<ex.what()<<", wait 200 ms.\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            noerror=0;
        }
    }
}

}