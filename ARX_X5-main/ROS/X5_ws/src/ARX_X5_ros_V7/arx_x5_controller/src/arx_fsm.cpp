#include "arx_x5_controller/arx_fsm.hpp"

ARXFSM::ARXFSM(ros::NodeHandle& nh) 
    : nh_(nh),
      exec_state_(INIT),
      has_current_pose_(false),
      is_executing_(false),
      has_target_(false),
      new_target_received_(false),
      is_resetting_(false),
      has_grasped_(false),
      max_inactive_time_(3.0),
      trajectory_generator_(nh)
{
    nh_.param("trajectory_generator/max_inactive_time", max_inactive_time_, 3.0);
    
    // ROS订阅发布
    goal_sub_ = nh_.subscribe("/infer_cmd", 10, &ARXFSM::goalCallback, this);
    ee_status_sub_ = nh_.subscribe("/arm_status_ee", 10, &ARXFSM::eeStatusCallback, this);
    cmd_pub_ = nh_.advertise<arm_control::PosCmd>("/arm_cmd", 10);
    
    // 状态机定时器 (100Hz)
    state_timer_ = nh_.createTimer(ros::Duration(0.01), &ARXFSM::execFSMCallback, this);
    last_active_time_ = ros::Time::now();

    ROS_INFO("ARX FSM initialized");
}

void ARXFSM::execFSMCallback(const ros::TimerEvent& event) {
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100) {
        printFSMExecState();
        fsm_num = 0;
    }

    switch (exec_state_) {
        case INIT: {
            if (!has_current_pose_) {
                return; // 等待当前位置信息
            } else {
                changeFSMExecState(IDLE, "ARX_FSM");
            }
            break;
        }
        case IDLE: {
            // 检测超时复位
            double inactive_time = (ros::Time::now() - last_active_time_).toSec();
            if (inactive_time > max_inactive_time_ && !is_resetting_) {
                if (fabs(current_ee_pose_.x) > 0.05 || fabs(current_ee_pose_.y) > 0.05 || fabs(current_ee_pose_.z) > 0.05){
                    reset(); // 触发复位
                }
            }
            
            // 处理新目标
            if (new_target_received_) {
                target_ = new_target_;
                new_target_received_ = false;
                has_target_ = true;
                is_resetting_ = false;
            }
            if (has_target_) {
                changeFSMExecState(GEN_NEW_TRAJ, "ARX_FSM");
            }
            break;
        }
        case GEN_NEW_TRAJ: {
            bool success = trajectory_generator_.generateTrajectory(
                current_ee_pose_, 
                target_,
                is_resetting_
            );
            
            if (success) {
                if (is_resetting_) {
                    changeFSMExecState(EXECUTING, "RESET_EXEC");
                } else {
                    changeFSMExecState(EXECUTING, "NORMAL_EXEC");
                }
                start_time_ = ros::Time::now();
            } else {
                changeFSMExecState(IDLE, "GEN_FAIL");
            }
            break;
        }
        case EXECUTING: {
            // 更新活动时间
            last_active_time_ = ros::Time::now();
            
            // 执行轨迹
            double progress = trajectory_generator_.publishCommands(
                start_time_,
                target_,
                is_resetting_,
                cmd_pub_
            );
            
            // TODO 需要加入current_pos和target的检测，这里还只是开环控制
            if (progress >= 1.0) {
                is_executing_ = false;
                has_target_ = false;
                
                if (is_resetting_) {
                    ROS_INFO("Reset completed");
                    is_resetting_ = false;
                } else {
                    has_grasped_ = true;
                    ROS_INFO("Trajectory execution completed");
                }   
                changeFSMExecState(IDLE, "EXEC_DONE");
            }
            break;
        }
    }
}

void ARXFSM::goalCallback(const arm_control::PosCmdConstPtr& msg) {
    if (!has_current_pose_)
    {
        return;
    }
    
    new_target_ = *msg;
    new_target_received_ = true;
    last_active_time_ = ros::Time::now();

    std::cout << "--------------------" << std::endl;
    ROS_INFO("\n New target received: \n Position:[%.2f, %.2f, %.2f] \n Orientation:[%.2f, %.2f, %.2f]", 
             new_target_.x, new_target_.y, new_target_.z, new_target_.roll, new_target_.pitch, new_target_.yaw);
    std::cout << "--------------------" << std::endl;

    if (!is_resetting_){
        if (exec_state_ == IDLE || exec_state_ == EXECUTING) {
            changeFSMExecState(GEN_NEW_TRAJ, "GOAL_RECEIVED");
        } 
    }
    

}

void ARXFSM::eeStatusCallback(const arm_control::PosCmdConstPtr& msg) {
    current_ee_pose_ = *msg;
    has_current_pose_ = true;
}

void ARXFSM::reset() {
    ROS_WARN("Initiating auto-reset to home position");
    has_target_ = true;
    is_resetting_ = true;
    
    // 设置复位目标位置
    target_.x = 0.0;
    target_.y = 0.0;
    target_.z = 0.0;
    target_.roll = 0.0;
    target_.pitch = 0.0;
    target_.yaw = 0.0;
    target_.gripper = (has_grasped_) ? (target_.gripper - 1) : 0;
    has_grasped_ = false;
    
    changeFSMExecState(GEN_NEW_TRAJ, "RESET_START");
    start_time_ = ros::Time::now();
}

void ARXFSM::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call) {
    std::string state_str[5] = { "INIT", "IDLE", "GEN_NEW_TRAJ", "EXECUTING"};
    int pre_s = static_cast<int>(exec_state_);
    exec_state_ = new_state;
    std::cout << "[" + pos_call + "]" + "State change:" + state_str[pre_s] + "->" +  state_str[static_cast<int>(new_state)] << std::endl;
}

void ARXFSM::printFSMExecState() {
    std::string state_str[5] = { "INIT", "IDLE", "GEN_NEW_TRAJ", "EXECUTING"};
    std::cout << "[FSM] State:" + state_str[static_cast<int>(exec_state_)] << std::endl;
}

void ARXFSM::start() {
    ROS_INFO("ARX FSM started");
    ros::spin();
}