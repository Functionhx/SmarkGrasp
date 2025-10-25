#include "local_planner.h"

RobotCtrl::RobotCtrl()
{
    ros::NodeHandle nh("~");
    nh.param<double>("max_x_speed", max_x_speed_, 1.0);
    nh.param<double>("max_y_speed", max_y_speed_, 1.0);

    nh.param<int>("plan_frequency", plan_freq_, 30);
    nh.param<double>("goal_dist_tolerance", goal_dist_tolerance_, 0.2);
    nh.param<double>("prune_ahead_distance", prune_ahead_dist_, 0.5);
    nh.param<std::string>("global_frame", global_frame_, "map");

    nh.param<double>("max_acc", max_acc_, 1.0);
    local_path_pub_ = nh.advertise<nav_msgs::Path>("local_path", 5);
    global_path_sub_ = nh.subscribe("/move_base/GlobalPlanner/plan", 5, &RobotCtrl::GlobalPathCallback, this);

    cmd_vel_pub_ = nh.advertise<yhs_can_msgs::ctrl_cmd>("/base_vel", 10);

    tf_listener_ = std::make_shared<tf::TransformListener>();

    plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_freq_), &RobotCtrl::Plan, this);

    infer_status_pub_ = nh.advertise<std_msgs::Bool>("/infer_status", 1);
}

void RobotCtrl::Plan(const ros::TimerEvent &event)
{

    if (plan_)
    {

        auto begin = std::chrono::steady_clock::now();
        auto start = ros::Time::now();
        // 1. Update the transform from global path frame to local planner frame
        UpdateTransform(tf_listener_, global_frame_,
                        global_path_.header.frame_id, global_path_.header.stamp,
                        global2path_transform_); // source_time needs decided

        // 2. Get current robot pose in global path frame
        geometry_msgs::PoseStamped robot_pose;
        GetGlobalRobotPose(tf_listener_, global_path_.header.frame_id, robot_pose);

        // 3. Check if robot has already arrived with given distance tolerance
        if (GetEuclideanDistance(robot_pose, global_path_.poses.back()) <= goal_dist_tolerance_ || prune_index_ == global_path_.poses.size() - 1)
        {
            plan_ = false;
            global_path_.poses.clear();
            
            yhs_can_msgs::ctrl_cmd cmd_vel;
            cmd_vel.ctrl_cmd_gear = 0X08;
            cmd_vel.ctrl_cmd_x_linear = 0.0;
            cmd_vel.ctrl_cmd_y_linear = 0.0;
            cmd_vel.ctrl_cmd_z_angular = 0.0;
            cmd_vel_pub_.publish(cmd_vel);

            std_msgs::Bool infer_status;
            infer_status.data = true;
            infer_status_pub_.publish(infer_status);

            ROS_INFO("Planning Success!");
            return;
        }

        // 4. Get prune index from given global path
        FindNearstPose(robot_pose, global_path_, prune_index_, prune_ahead_dist_); // TODO: double direct prune index is needed later!

        // 5. Generate the prune path and transform it into local planner frame
        nav_msgs::Path prune_path, local_path;

        local_path.header.frame_id = global_frame_;
        prune_path.header.frame_id = global_frame_;

        geometry_msgs::PoseStamped tmp_pose;
        tmp_pose.header.frame_id = global_frame_;

        TransformPose(global2path_transform_, robot_pose, tmp_pose);
        // std::cout << global2path_transform_.frame_id_ << std::endl;
        prune_path.poses.push_back(tmp_pose);

        int i = prune_index_;

        while (i < global_path_.poses.size() && i - prune_index_ < 50)
        {

            TransformPose(global2path_transform_, global_path_.poses[i], tmp_pose);
            prune_path.poses.push_back(tmp_pose);
            i++;
        }

        // 6. Generate the cubic spline trajectory from above prune path
        GenTraj(prune_path, local_path, 0.05f);
        local_path_ = local_path;
        local_path_pub_.publish(local_path);

        // 7. Follow the trajectory and calculate the velocity
        yhs_can_msgs::ctrl_cmd cmd_vel;
        FollowTraj(robot_pose, local_path, cmd_vel);
        cmd_vel_pub_.publish(cmd_vel);

        auto plan_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin);
    }
    else
    {
        yhs_can_msgs::ctrl_cmd cmd_vel;
            cmd_vel.ctrl_cmd_gear = 0X08; 
            cmd_vel.ctrl_cmd_x_linear = 0.0;
            cmd_vel.ctrl_cmd_y_linear = 0.0;
            cmd_vel_pub_.publish(cmd_vel);
    }
}

void RobotCtrl::FindNearstPose(geometry_msgs::PoseStamped &robot_pose, nav_msgs::Path &path, int &prune_index, double prune_ahead_dist)
{

    double dist_threshold = 10; // threshold is 10 meters (basically never over 10m i suppose)
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist;
    if (prune_index != 0)
    {
        sq_dist = GetEuclideanDistance(robot_pose, path.poses[prune_index - 1]);
    }
    else
    {
        sq_dist = 1e10;
    }

    double new_sq_dist = 0;
    while (prune_index < (int)path.poses.size())
    {
        new_sq_dist = GetEuclideanDistance(robot_pose, path.poses[prune_index]);
        if (new_sq_dist > sq_dist && sq_dist < sq_dist_threshold)
        {

            // Judge if it is in the same direction and sq_dist is further than 0.3 meters
            if ((path.poses[prune_index].pose.position.x - robot_pose.pose.position.x) *
                            (path.poses[prune_index - 1].pose.position.x - robot_pose.pose.position.x) +
                        (path.poses[prune_index].pose.position.y - robot_pose.pose.position.y) *
                            (path.poses[prune_index - 1].pose.position.y - robot_pose.pose.position.y) >
                    0 &&
                sq_dist > prune_ahead_dist)
            {
                prune_index--;
            }
            else
            {
                sq_dist = new_sq_dist;
            }

            break;
        }
        sq_dist = new_sq_dist;
        ++prune_index;
    }

    prune_index = std::min(prune_index, (int)(path.poses.size() - 1));
}

void RobotCtrl::FollowTraj(const geometry_msgs::PoseStamped &robot_pose,
                           const nav_msgs::Path &traj,
                           yhs_can_msgs::ctrl_cmd &cmd_vel)
{

    geometry_msgs::PoseStamped robot_pose_1;
    GetGlobalRobotPose(tf_listener_, global_path_.header.frame_id, robot_pose_1);
    // std::cout << "frame_id: " << global_path_.header.frame_id << std::endl;
    yaw_ = tf::getYaw(robot_pose_1.pose.orientation);

    double diff_yaw = 0;
    if (traj.poses.size() >= 5)
        diff_yaw = atan2((traj.poses[4].pose.position.y - robot_pose.pose.position.y),
                         (traj.poses[4].pose.position.x - robot_pose.pose.position.x));
    else
        diff_yaw = atan2((traj.poses[traj.poses.size() - 1].pose.position.y - robot_pose.pose.position.y),
                         (traj.poses[traj.poses.size() - 1].pose.position.x - robot_pose.pose.position.x));


    double vx_global = max_x_speed_ * cos(diff_yaw); //*diff_distance*p_value_;
    double vy_global = max_y_speed_ * sin(diff_yaw); //*diff_distance*p_value_;

    if (isnan(vy_global) == false && isnan(vx_global) == false)
    {
        double last_yaw;
        if (vel.ctrl_cmd_x_linear != 0 || vel.ctrl_cmd_y_linear != 0)
        {
            last_yaw = atan2(vel.ctrl_cmd_y_linear, vel.ctrl_cmd_x_linear);
            if (abs(last_yaw - diff_yaw) < M_PI / 36.)
            {
                cmd_vel = vel;
            }
            else
            {
                cmd_vel.ctrl_cmd_gear = 0X08;
                cmd_vel.ctrl_cmd_x_linear = vx_global;
                cmd_vel.ctrl_cmd_y_linear = vy_global;
                vel = cmd_vel;
            }
        }
        else
        {
            cmd_vel.ctrl_cmd_gear = 0X08;
            cmd_vel.ctrl_cmd_x_linear = vx_global;
            cmd_vel.ctrl_cmd_y_linear = vy_global;
            vel = cmd_vel;
        }
    }
    else
    {
        cmd_vel.ctrl_cmd_gear = 0X08;
        cmd_vel.ctrl_cmd_x_linear = 0.0;
        cmd_vel.ctrl_cmd_y_linear = 0.0;
        vel = cmd_vel;
    }
}

void RobotCtrl::GlobalPathCallback(const nav_msgs::PathConstPtr &msg)
{
    if (!msg->poses.empty())
    {
        nav_msgs::Path tmp_path = *msg;
        if (!global_path_.poses.empty())
        {
            double average_dis = GetPathsCost(tmp_path, global_path_, prune_index_);
            if (average_dis > 0.05)
            {
                global_path_ = tmp_path;
                prune_index_ = 0;
                // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!new_path!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            }
        }
        else
        {
            global_path_ = tmp_path;
            prune_index_ = 0;
        }
        plan_ = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_planner");
    RobotCtrl robotctrl;
    ros::spin();
    return 0;
}