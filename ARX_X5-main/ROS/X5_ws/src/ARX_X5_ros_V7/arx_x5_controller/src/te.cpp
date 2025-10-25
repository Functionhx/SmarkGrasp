#include <ros/ros.h>
#include "arm_control/PosCmd.h"
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ROS_INFO("Test node initialized successfully.");
    ros::Publisher pub = nh.advertise<arm_control::PosCmd>("/infer_cmd", 10);
    ros::Publisher fake_odom_pub = nh.advertise<arm_control::PosCmd>("/arm_status_ee", 10);
    arm_control::PosCmd msg, odom;
    int count = 0;
    while (ros::ok())
    {
        if (count >= 100)
        {
            msg.x = 0.4;
            msg.y = 0.3;
            msg.z = 0.2;
            msg.roll = 0.5;
            msg.pitch = 1.0;
            msg.yaw = 0.8;
            msg.gripper = 4;
            pub.publish(msg);
            count = 0;
        }
        count++;
        ros::spinOnce();
        ros::Duration(0.05).sleep(); // 每5秒发布一次
    }
    
    return 0;
}
