#include "nav_service/infer_navigator.hpp"


void NavService::SearchGraspGoal(geometry_msgs::PointStamped &in){
    double rela_yaw = atan2(in.point.y - odom.pose.pose.position.y, in.point.x - odom.pose.pose.position.x);
    ROS_INFO("rela_yaw: %f", rela_yaw);
    in.point.x -= grasp_radius * cos(rela_yaw);
    in.point.y -= grasp_radius * sin(rela_yaw);
    in.point.z = 0.0;

}
void NavService::navRequestCB(const geometry_msgs::PoseStampedConstPtr &target){
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ROS_INFO("Origin goal: x = %f, y = %f", target->pose.position.x, target->pose.position.y);
    geometry_msgs::PointStamped in, out;
    in.header.frame_id = "body";
    in.header.stamp = odom.header.stamp;
    in.point.x = target->pose.position.x;
    in.point.y = target->pose.position.y;
    in.point.z = 0;
    try
    {
        lidar2base_buffer_.transform(in, out, "map", ros::Duration(0.3));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("Transform error: %s", ex.what());
        return;
    }
    SearchGraspGoal(out);
    goal.target_pose.pose.position.x = out.point.x;
    goal.target_pose.pose.position.y = out.point.y;
    goal.target_pose.pose.orientation.w = 1;
    ROS_INFO("Current goal: x = %f, y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    if (GetDistance(last_goal.target_pose, goal.target_pose) < grasp_radius)
    {
        ROS_INFO("The goal is too close to the last goal, skip this request.");
        return;
    }
    else if(is_reach_){
        ROS_INFO("Reach goal, ready for grasp.");
        is_reach_ = false;
        // movebase_cient->cancelAllGoals();
        return;
    }
    movebase_cient->sendGoal(goal);
    ROS_INFO("Send goal to move base");

    last_goal = goal;
}

NavService::NavService():
    lidar2base_buffer_(ros::Duration(3.0)),
    tf_listener_(lidar2base_buffer_)
    {
        lidar2base_buffer_.setUsingDedicatedThread(true);
        nav_cmd_sub = nh.subscribe<geometry_msgs::PoseStamped>("/infer_goal", 10, &NavService::navRequestCB, this);
        odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom_base_link", 10, &NavService::odomCallback, this);

        infer_sub = nh.subscribe<std_msgs::Bool>("/infer_status", 10, [&](const std_msgs::BoolConstPtr& msg){  
            is_reach_ = msg->data;
        });

        movebase_cient = std::make_unique<MoveBaseClient>("move_base", true);
        if (!movebase_cient->waitForServer(ros::Duration(5)))
        {
            ROS_INFO("Can't connected to move base server");
        }
        ROS_INFO("Connected to move base server");
    }


int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "NavService");

    NavService ns;
    ros::spin();
    return 0;
}