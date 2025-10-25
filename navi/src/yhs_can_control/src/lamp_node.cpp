// #include <ros/ros.h>
// #include "yhs_can_msgs/io_cmd.h" // ROS1消息头文件后缀是.h而不是.hpp
// #include <mutex>

// class HeadlightControlNode
// {
// public:
//   HeadlightControlNode() : count_1(0)
//   {
//     // 初始化ROS节点
//     ros::NodeHandle nh;
    
//     // 创建io_cmd话题的发布器 (ROS1版本)
//     io_cmd_publisher_ = nh.advertise<yhs_can_msgs::io_cmd>("io_cmd", 10);
    
//     // 创建定时器，每秒发布一次消息 (ROS1版本)
//     timer_ = nh.createTimer(ros::Duration(1.0), 
//                           &HeadlightControlNode::publish_io_cmd, this);
    
//     ROS_INFO("Headlight control node started. Lower beam headlight ON, all others OFF.");
//   }

// private:
//   void publish_io_cmd(const ros::TimerEvent&)
//   {
//     std::lock_guard<std::mutex> lock(mutex_);
    
//     yhs_can_msgs::io_cmd msg;
    
//     // 主灯控制 - 关闭 (对应bit0)
//     msg.io_cmd_lamp_ctrl = 1;
    
//     // 解锁信号 - 关闭 (对应bit1)
//     msg.io_cmd_unlock = false;
    
//     // 灯光设置 (字节1)
//     msg.io_cmd_lower_beam_headlamp = true;  // 近光灯开启 (bit0)
//     msg.io_cmd_upper_beam_headlamp = false; // (bit1)
//     msg.io_cmd_turn_lamp = 2;               // 转向灯关闭 (bit2-3)
//     msg.io_cmd_braking_lamp = false;        // (bit4)
//     msg.io_cmd_clearance_lamp = false;      // (bit5)
//     msg.io_cmd_fog_lamp = false;            // (bit6)
    
//     // 音频设备 (字节2)
//     msg.io_cmd_speaker = 0;
    
//     // 增加计数器 (字节6高4位)
//     count_1++;
//     if (count_1 == 16)
//       count_1 = 0;
    
//     // 发布消息
//     io_cmd_publisher_.publish(msg);
    
//     ROS_DEBUG("Published io_cmd message - Lower beam headlight ON");
//   }

//   ros::Publisher io_cmd_publisher_;
//   ros::Timer timer_;
//   unsigned char count_1;
//   std::mutex mutex_;
// };

// int main(int argc, char *argv[])
// {
//   ros::init(argc, argv, "lamp_node");
//   HeadlightControlNode node;
//   ros::spin();
//   return 0;
// }

// #include <ros/ros.h>
// #include "yhs_can_msgs/io_cmd.h" // ROS1消息头文件后缀是.h而不是.hpp
// #include "yhs_can_msgs/ctrl_cmd.h"
// #include <mutex>

// class HeadlightControlNode
// {
// public:
//   HeadlightControlNode() : count_1(0), count_2(0)
//   {
//     // 初始化ROS节点
//     ros::NodeHandle nh;
    
//     // 创建io_cmd和ctrl_cmd话题的发布器
//     io_cmd_publisher_ = nh.advertise<yhs_can_msgs::io_cmd>("io_cmd", 10);
//     ctrl_cmd_publisher_ = nh.advertise<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 10);
    
//     // 创建定时器，每秒发布一次消息
//     timer_ = nh.createTimer(ros::Duration(1.0), 
//                           &HeadlightControlNode::publish_cmds, this);
    
//     ROS_INFO("Headlight and motion control node started.");
//   }

// private:
//   void publish_cmds(const ros::TimerEvent&)
//   {
//     std::lock_guard<std::mutex> lock(mutex_);
    
//     // 发布io_cmd消息（灯光控制）
//     yhs_can_msgs::io_cmd io_msg;
//     io_msg.io_cmd_lamp_ctrl = 1;            // 主灯控制使能
//     io_msg.io_cmd_unlock = false;           // 解锁信号关闭
//     io_msg.io_cmd_lower_beam_headlamp = true;  // 近光灯开启
//     io_msg.io_cmd_upper_beam_headlamp = false; // 远光灯关闭
//     io_msg.io_cmd_turn_lamp = 2;            // 右转向灯开启
//     io_msg.io_cmd_braking_lamp = false;     // 制动灯关闭
//     io_msg.io_cmd_clearance_lamp = false;   // 示廓灯关闭
//     io_msg.io_cmd_fog_lamp = false;         // 雾灯关闭
//     io_msg.io_cmd_speaker = 0;              // 喇叭关闭
    
//     // 更新计数器
//     count_1++;
//     if (count_1 == 16) count_1 = 0;
    
//     io_cmd_publisher_.publish(io_msg);

//     // 发布ctrl_cmd消息（运动控制）
//     yhs_can_msgs::ctrl_cmd ctrl_msg;
//     ctrl_msg.ctrl_cmd_gear = 0x06;          // 4T4D档位
//     ctrl_msg.ctrl_cmd_x_linear = 0.001;     // x轴线速度0.001 m/s
//     ctrl_msg.ctrl_cmd_y_linear = 0.001;     // y轴线速度0.001 m/s
//     ctrl_msg.ctrl_cmd_z_angular = 0.01;     // z轴角速度0.01°/s
    
//     // 更新计数器
//     count_2++;
//     if (count_2 == 16) count_2 = 0;
    
//     ctrl_cmd_publisher_.publish(ctrl_msg);

//     ROS_DEBUG("Published control messages: lights and motion");
//   }

//   ros::Publisher io_cmd_publisher_;
//   ros::Publisher ctrl_cmd_publisher_;
//   ros::Timer timer_;
//   unsigned char count_1;  // io_cmd计数器
//   unsigned char count_2;  // ctrl_cmd计数器
//   std::mutex mutex_;
// };

// int main(int argc, char *argv[])
// {
//   ros::init(argc, argv, "control_node");
//   HeadlightControlNode node;
//   ros::spin();
//   return 0;
// }

#include <ros/ros.h>
#include "yhs_can_msgs/io_cmd.h"
#include "yhs_can_msgs/ctrl_cmd.h"
#include "yhs_can_msgs/steering_ctrl_cmd.h"
#include <mutex>

class HeadlightControlNode
{
public:
  HeadlightControlNode() : count_1(0), count_2(0), count_3(0)
  {
    // 初始化ROS节点
    ros::NodeHandle nh;
    
    // 创建话题发布器
    io_cmd_publisher_ = nh.advertise<yhs_can_msgs::io_cmd>("io_cmd", 10);
    ctrl_cmd_publisher_ = nh.advertise<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 10);
    // steering_ctrl_cmd_publisher_ = nh.advertise<yhs_can_msgs::steering_ctrl_cmd>("steering_ctrl_cmd", 10);
    
    // 创建定时器，每秒发布一次消息
    timer_ = nh.createTimer(ros::Duration(0.1), 
                          &HeadlightControlNode::publish_cmds, this);
    
    ROS_INFO("Control node started with light, motion and steering control.");
  }

private:
  void publish_cmds(const ros::TimerEvent&)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 1. 发布io_cmd消息（灯光控制）
    yhs_can_msgs::io_cmd io_msg;
    io_msg.io_cmd_lamp_ctrl = 1;            // 主灯控制使能
    io_msg.io_cmd_unlock = false;           // 解锁信号关闭
    io_msg.io_cmd_lower_beam_headlamp = true;  // 近光灯开启
    io_msg.io_cmd_upper_beam_headlamp = false; // 远光灯关闭
    io_msg.io_cmd_turn_lamp = 2;            // 右转向灯开启
    io_msg.io_cmd_braking_lamp = false;     // 制动灯关闭
    io_msg.io_cmd_clearance_lamp = false;   // 示廓灯关闭
    io_msg.io_cmd_fog_lamp = false;         // 雾灯关闭
    io_msg.io_cmd_speaker = 0;              // 喇叭关闭
    
    count_1++;
    if (count_1 == 16) count_1 = 0;
    io_cmd_publisher_.publish(io_msg);

    // 2. 发布ctrl_cmd消息（运动控制）
    yhs_can_msgs::ctrl_cmd ctrl_msg;
    ctrl_msg.ctrl_cmd_gear = 0x08;          // 4T4D档位
    ctrl_msg.ctrl_cmd_x_linear = 0.1;     // x轴线速度0.001 m/s
    ctrl_msg.ctrl_cmd_y_linear = 0.10;     // y轴线速度0.001 m/s
    ctrl_msg.ctrl_cmd_z_angular = 10.0;     // z轴角速度0.01°/s
    
    count_2++;
    if (count_2 == 16) count_2 = 0;
    ctrl_cmd_publisher_.publish(ctrl_msg);

    // // 3. 新增steering_ctrl_cmd消息（转向控制）
    // yhs_can_msgs::steering_ctrl_cmd steering_msg;
    // steering_msg.ctrl_cmd_gear = 0x07;               // 4T4D档位
    // steering_msg.steering_ctrl_cmd_velocity = 0; // 转向轮速度0.001 m/s
    // steering_msg.steering_ctrl_cmd_steering = 0;  // 转向角度50°
    
    // count_3++;
    // if (count_3 == 16) count_3 = 0;
    // steering_ctrl_cmd_publisher_.publish(steering_msg);

    ROS_DEBUG("Published control messages: lights, motion and steering");
  }

  ros::Publisher io_cmd_publisher_;
  ros::Publisher ctrl_cmd_publisher_;
  // ros::Publisher steering_ctrl_cmd_publisher_;
  ros::Timer timer_;
  unsigned char count_1;  // io_cmd计数器
  unsigned char count_2;  // ctrl_cmd计数器
  unsigned char count_3;  // steering_ctrl_cmd计数器
  std::mutex mutex_;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "integrated_control_node");
  HeadlightControlNode node;
  ros::spin();
  return 0;
}