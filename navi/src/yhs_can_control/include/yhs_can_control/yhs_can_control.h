#ifndef __CANCONTROL_NODE_H__
#define __CANCONTROL_NODE_H__

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "yhs_can_msgs/ctrl_cmd.h"
#include "yhs_can_msgs/io_cmd.h"
#include "yhs_can_msgs/ctrl_fb.h"
#include "yhs_can_msgs/lr_wheel_fb.h"
#include "yhs_can_msgs/rr_wheel_fb.h"
#include "yhs_can_msgs/lf_wheel_fb.h"
#include "yhs_can_msgs/rf_wheel_fb.h"
#include "yhs_can_msgs/io_fb.h"
#include "yhs_can_msgs/steering_ctrl_cmd.h"
#include "yhs_can_msgs/bms_fb.h"
#include "yhs_can_msgs/bms_flag_fb.h"
#include "yhs_can_msgs/steering_ctrl_fb.h"
#include "yhs_can_msgs/front_angle_fb.h"
#include "yhs_can_msgs/rear_angle_fb.h"
#include "yhs_can_msgs/ultrasonic.h"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <string>
#include <sstream>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <mutex>

namespace yhs_tool
{
  class CanControl
  {
  public:
    CanControl();
    ~CanControl();

    void run();

  private:
    ros::NodeHandle nh_;

    ros::Publisher ctrl_fb_pub_;
    ros::Publisher lr_wheel_fb_pub_;
    ros::Publisher rr_wheel_fb_pub_;
    ros::Publisher io_fb_pub_;
    ros::Publisher rf_wheel_fb_pub_;
    ros::Publisher lf_wheel_fb_pub_;
    ros::Publisher bms_fb_pub_;
    ros::Publisher bms_flag_fb_pub_;
    ros::Publisher steering_ctrl_fb_pub_;
    ros::Publisher front_angle_fb_pub_;
    ros::Publisher rear_angle_fb_pub_;
    ros::Publisher ultrasonic_pub_;

    ros::Subscriber ctrl_cmd_sub_;
    ros::Subscriber io_cmd_sub_;
    ros::Subscriber steering_ctrl_cmd_sub_;

    std::mutex mutex_;

    unsigned char sendData_u_io_[8];
    unsigned char sendData_u_vel_[8];

    int dev_handler_;
    can_frame send_frames_;
    can_frame recv_frames_;

    std::vector<int> ultrasonic_number_;

    void io_cmdCallBack(const yhs_can_msgs::io_cmd msg);
    void ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd msg);
    void steering_ctrl_cmdCallBack(const yhs_can_msgs::steering_ctrl_cmd msg);

    void recvData();
    void sendData();
  };

}

#endif
