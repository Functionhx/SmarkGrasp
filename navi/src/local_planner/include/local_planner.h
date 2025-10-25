#ifndef PID_POSITION_FOLLOW_H
#define PID_POSITION_FOLLOW_H

#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include "utility.h"
#include <tf2/utils.h>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "cubic_spline/cubic_spline_ros.h"
#include "utility.h"
#include <Eigen/Eigen>
#include <chrono>

#include <yhs_can_msgs/ctrl_cmd.h>

class RobotCtrl {
    public:
    RobotCtrl();
    ~RobotCtrl() = default;

    void GlobalPathCallback(const nav_msgs::PathConstPtr & msg);

    void FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                        const nav_msgs::Path& traj,
                        yhs_can_msgs::ctrl_cmd& cmd_vel);
    void FindNearstPose(geometry_msgs::PoseStamped& robot_pose,nav_msgs::Path& path, int& prune_index, double prune_ahead_dist);
    void Plan(const ros::TimerEvent& event);

private:
    ros::Publisher cmd_vel_pub_;
    ros::Publisher local_path_pub_;
    ros::Publisher nav_status_pub_;
    ros::Publisher infer_status_pub_;

    ros::Subscriber global_path_sub_;
    ros::Timer plan_timer_;

    ros::ServiceServer planner_server_;

    std::shared_ptr<tf::TransformListener> tf_listener_;
    tf::StampedTransform global2path_transform_;

    nav_msgs::Path global_path_;
    nav_msgs::Path local_path_;
    yhs_can_msgs::ctrl_cmd vel;

    bool plan_ = false;
    int prune_index_ = 0;

    double max_x_speed_;
    double max_y_speed_;
    double max_acc_;

    double set_yaw_speed_;

    double p_value_;
    double i_value_;
    double d_value_;

    int plan_freq_;
    double goal_dist_tolerance_;
    double prune_ahead_dist_;

    std::string global_frame_;

    double a_gimbal_yaw_position;  //
    double a_gimbal_pitch_position;  //

    double cur_gimbal_yaw_position;
    double cur_gimbal_pitch_position;

    double yaw_;  //机器人航向角
    
    uint8_t game_state_ = 4;
    int planner_state_ = 2;   //规划状态 0：静止  1：原地小陀螺  2：路径跟踪

};

//弧度制归一化
double normalizeRadian(const double angle)
{
   double n_angle = std::fmod(angle, 2 * M_PI);
   n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;
   return n_angle;
}

double ABS_limit(double value,double limit)
{
  if(value<limit && value>-limit)
  {
    return 0;
  }
  else
  {
    return value;
  }

}


#endif 
