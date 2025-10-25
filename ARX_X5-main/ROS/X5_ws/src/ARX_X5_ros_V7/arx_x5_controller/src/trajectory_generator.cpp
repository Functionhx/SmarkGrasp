#include "arx_x5_controller/trajectory_generator.hpp"

TrajectoryGenerator::TrajectoryGenerator(ros::NodeHandle& nh) 
    : exp_velocity_(0.2),
      trajectory_points_(6),
      min_boundaries_(6), max_boundaries_(6),
      start_vec_(6), end_vec_(6),
      min_gripper_(1), max_gripper_(5)
{
    nh.param("trajectory_generator/exp_velocity", exp_velocity_, 0.2);
    nh.param("trajectory_generator/min_x", min_boundaries_[0], 0.0);
    nh.param("trajectory_generator/max_x", max_boundaries_[0], 0.5);
    nh.param("trajectory_generator/min_y", min_boundaries_[1], -0.5);
    nh.param("trajectory_generator/max_y", max_boundaries_[1], 0.5);
    nh.param("trajectory_generator/min_z", min_boundaries_[2], -0.5);
    nh.param("trajectory_generator/max_z", max_boundaries_[2], 0.5);
    nh.param("trajectory_generator/min_roll", min_boundaries_[3], -2.1);
    nh.param("trajectory_generator/max_roll", max_boundaries_[3], 2.1);
    nh.param("trajectory_generator/min_pitch", min_boundaries_[4], -1.3);
    nh.param("trajectory_generator/max_pitch", max_boundaries_[4], 1.3); 
    nh.param("trajectory_generator/min_yaw", min_boundaries_[5], -1.3);
    nh.param("trajectory_generator/max_yaw", max_boundaries_[5], 1.3);
    nh.param("trajectory_generator/min_gripper", min_gripper_, 1);
    nh.param("trajectory_generator/max_gripper", max_gripper_, 5);
    
    ROS_INFO("Trajectory Generator initialized with max velocity: %.2f m/s", exp_velocity_);
    ROS_INFO("Safety boundaries set to: x[%.2f, %.2f], y[%.2f, %.2f], z[%.2f, %.2f]",
             min_boundaries_[0], max_boundaries_[0], min_boundaries_[1], max_boundaries_[1], min_boundaries_[2], max_boundaries_[2]);
    ROS_INFO("Orientation boundaries: roll [%.2f, %.2f], pitch [%.2f, %.2f], yaw [%.2f, %.2f]",
             min_boundaries_[3], max_boundaries_[3], min_boundaries_[4], max_boundaries_[4], min_boundaries_[5], max_boundaries_[5]);
    ROS_INFO("Gripper boundaries: [%d, %d]", min_gripper_, max_gripper_);
}

bool TrajectoryGenerator::generateTrajectory(
    const arm_control::PosCmd& start_pose, 
    const arm_control::PosCmd& target_pose,
    bool is_resetting
) {
    // 起点设置
    start_vec_ << start_pose.x,  start_pose.y,  start_pose.z, 
                  start_pose.roll,  start_pose.pitch,  start_pose.yaw;
    
    // 终点设置
    end_vec_ << target_pose.x, target_pose.y, target_pose.z, 
                target_pose.roll, target_pose.pitch, target_pose.yaw;
    
    start_gripper_ = start_pose.gripper;
    target_gripper_ = target_pose.gripper;
    
    // 安全边界检查
    if (!checkBoundaries(start_vec_)) {
        ROS_ERROR("Start position [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] exceeds safety boundaries!",
                  start_vec_[0], start_vec_[1], start_vec_[2], start_vec_[3], start_vec_[4], start_vec_[5]);
        return false;
    }
    
    if (!checkBoundaries(end_vec_)) {
        ROS_ERROR("Target position [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] exceeds safety boundaries!",
                    end_vec_[0], end_vec_[1], end_vec_[2], end_vec_[3], end_vec_[4], end_vec_[5]);
        return false;
    }
    
    start_pos_ = start_vec_.head<3>();
    end_pos_ = end_vec_.head<3>();
    // 计算轨迹时间和速度
    double distance = (end_pos_ - start_pos_).norm();
    move_duration_ = distance / exp_velocity_;
    move_duration_ = std::max(1.0, std::min(move_duration_, 10.0));
    
    // 生成轨迹点
    trajectory_points_.clear();
    int num_points = static_cast<int>(move_duration_ * 50);
    for (int i = 0; i <= num_points; i++) {
        double progress = static_cast<double>(i) / num_points;
        trajectory_points_.push_back(interpolate(progress, start_vec_, end_vec_));
    }
    
    std::cout << "--------------------" << std::endl;
    ROS_INFO("Generated %zu trajectory points from start to target.", trajectory_points_.size());
    std::cout << "Trajectory duration: " << move_duration_ << " seconds." << std::endl;
    std::cout << "--------------------" << std::endl;
    return true;
}

double TrajectoryGenerator::publishCommands(
    const ros::Time& start_time,
    const arm_control::PosCmd& target,
    bool is_resetting,
    ros::Publisher& cmd_pub) {

    ros::Duration elapsed = ros::Time::now() - start_time;
    double progress = std::min(elapsed.toSec() / move_duration_, 1.0);
    
    size_t index = static_cast<size_t>(progress * (trajectory_points_.size() - 1));
    index = std::min(index, trajectory_points_.size() - 1);
    Eigen::Vector3d current_pos = trajectory_points_[index].head<3>();
    Eigen::Vector3d orientation = trajectory_points_[index].tail<3>();
    
    arm_control::PosCmd cmd;
    cmd.x = current_pos.x();
    cmd.y = current_pos.y();
    cmd.z = current_pos.z();
    cmd.roll = orientation.x();
    cmd.pitch = orientation.y();
    cmd.yaw = orientation.z();
    
    // 根据状态设置抓取器
    if (is_resetting) {
        cmd.gripper = 0;
    } else {
        // 正常执行：接近终点时松开抓取器
        cmd.gripper = (progress >= 0.85) ? (target_gripper_ - 1) : target_gripper_;
    }
    
    cmd_pub.publish(cmd);

    return progress;
}

Eigen::VectorXd TrajectoryGenerator::interpolate(double progress, 
                                                const Eigen::VectorXd& start_vec, 
                                                const Eigen::VectorXd& end_vec) {
    return start_vec + (end_vec - start_vec) * progress;
}

bool TrajectoryGenerator::checkBoundaries(const Eigen::VectorXd& position) const {
    int size = position.size();
    for (int i = 0; i < size; i++){
        if (position[i] <= min_boundaries_[i] - 1e-2 || position[i] >= max_boundaries_[i] + 1e-2){
            return false;
        }
    }
    return true; 
}
