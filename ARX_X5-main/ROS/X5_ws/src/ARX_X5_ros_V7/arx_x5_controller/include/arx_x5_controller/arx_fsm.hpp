#ifndef ARX_FSM_H
#define ARX_FSM_H

#include <ros/ros.h>
#include <tf2/convert.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "arm_control/PosCmd.h"
#include "trajectory_generator.hpp"

class ARXFSM {
public:
    ARXFSM(ros::NodeHandle& nh);
    void start();
    
    // 状态枚举
    enum FSM_EXEC_STATE {
        INIT,
        IDLE,
        GEN_NEW_TRAJ,
        EXECUTING
    };

private:
    // 状态机核心方法
    void execFSMCallback(const ros::TimerEvent& event);
    void goalCallback(const arm_control::PosCmdConstPtr& msg);
    void eeStatusCallback(const arm_control::PosCmdConstPtr& msg);
    void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call);
    void printFSMExecState();
    void reset();
    
    // 状态变量
    FSM_EXEC_STATE exec_state_;
    bool has_current_pose_;
    bool is_executing_;
    bool has_target_;
    bool new_target_received_;
    bool is_resetting_;
    bool has_grasped_;
    double max_inactive_time_;
    ros::Time last_active_time_;
    ros::Time start_time_;
    
    // 当前状态
    arm_control::PosCmd current_ee_pose_;
    arm_control::PosCmd target_;
    arm_control::PosCmd new_target_;
    
    // 轨迹生成器
    TrajectoryGenerator trajectory_generator_;
    
    // ROS接口
    ros::NodeHandle nh_;
    ros::Subscriber goal_sub_;
    ros::Subscriber ee_status_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer state_timer_;
};

#endif // ARX_FSM_H