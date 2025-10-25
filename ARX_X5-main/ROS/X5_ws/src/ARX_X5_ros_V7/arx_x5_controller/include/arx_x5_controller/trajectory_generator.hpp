#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include "arm_control/PosCmd.h"

class TrajectoryGenerator {
public:
    TrajectoryGenerator(ros::NodeHandle& nh);
    
    bool generateTrajectory(
        const arm_control::PosCmd& start_pose, 
        const arm_control::PosCmd& target_pose,
        bool is_resetting
    );
    
    double publishCommands(
        const ros::Time& start_time,
        const arm_control::PosCmd& target,
        bool is_resetting,
        ros::Publisher& cmd_pub
    );
    
private:
    // 线性插值与边界检查
    Eigen::VectorXd interpolate(double progress, const Eigen::VectorXd& start_vec, const Eigen::VectorXd& end_vec);
    bool checkBoundaries(const Eigen::VectorXd& position) const;
    
    // 轨迹参数
    double exp_velocity_;
    double move_duration_;
    
    // 安全边界
    Eigen::VectorXd min_boundaries_, max_boundaries_; // 6D vector for [x, y, z, roll, pitch, yaw]
    int min_gripper_, max_gripper_;
    
    // 轨迹状态
    Eigen::VectorXd start_vec_, end_vec_;
    Eigen::Vector3d start_pos_, end_pos_;
    int start_gripper_, target_gripper_;
    
    std::vector<Eigen::VectorXd> trajectory_points_;
    std::vector<Eigen::Vector3d> trajectory_orientations_;
};

#endif // TRAJECTORY_GENERATOR_H