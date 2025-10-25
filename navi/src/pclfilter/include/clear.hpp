#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Eigen>
namespace pclfilter
{
    // TO delete start
    struct Point2D
    {
        double x;
        double y;
    };
    using Polygon2D = std::vector<Point2D>;
    // End!
    class clear
    {
    private:
        // ros
        ros::NodeHandle nh_;
        ros::Subscriber odom_sub_;
        ros::Subscriber cloud_sub_;
        ros::Subscriber basemap_cloud_sub_;
        ros::Publisher cloud_pub_;
        std::vector<Polygon2D> polygons_;
        // cube
        std::vector<Eigen::Vector4f> min_pts_;
        std::vector<Eigen::Vector4f> max_pts_;
        //
        double CTE; // 点云膨胀系数
        double x_;
        double y_;
        double yaw_;
        double pitch_;
        double roll_;
        bool if_clear_;
        int way_select;

    public:
        bool loadparam(const std::string &param_name);
        bool is_non_area(double x, double y);
        void odomCB(const nav_msgs::Odometry::ConstPtr &odom_msg);
        void cloudCB(const sensor_msgs::PointCloud2ConstPtr &input_msg);
        void basemapCloudCB(const sensor_msgs::PointCloud2ConstPtr &input_msg);
        void combineCloudCB(const sensor_msgs::PointCloud2ConstPtr &input_msg);
        bool loadcubeParam(const std::string &param_name);
        void init_basemap();
        void init_odom();
        void init_mapwithodom();
        void once_filter(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
            const Eigen::Vector4f &min_pt,
            const Eigen::Vector4f &max_pt,
            double cubeyaw);
        void once_filter(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
            const Eigen::Vector4f &min_pt,
            const Eigen::Vector4f &max_pt);
        clear();
        // test:PCD input 
        
    };

} // namespace pclfilter