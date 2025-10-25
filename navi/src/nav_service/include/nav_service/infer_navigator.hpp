#include "utils.hpp"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <std_msgs/Bool.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NavService {
  private:
    ros::NodeHandle nh;
    ros::Subscriber nav_cmd_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber infer_sub;

    tf2_ros::TransformListener tf_listener_;
    tf2_ros::Buffer lidar2base_buffer_;
    std::unique_ptr<MoveBaseClient> movebase_cient;

    nav_msgs::Odometry odom;
    move_base_msgs::MoveBaseGoal last_goal;
    bool is_reach_ = false;
    
    double grasp_radius = 0.40;
    void navRequestCB(const geometry_msgs::PoseStampedConstPtr& target);
    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        odom = *msg;
    }
    void SearchGraspGoal(geometry_msgs::PointStamped& in);
  public:
    NavService();
    ~NavService(){}
};
