#include "arx_x5_controller/arx_fsm.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "arx_control_node");
    ros::NodeHandle nh("~");
    
    ARXFSM fsm(nh);
    fsm.start();
    
    return 0;
}