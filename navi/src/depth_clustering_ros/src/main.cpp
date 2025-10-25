//
// Created by alex on 2020/4/12.
//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <depth_cluster.h>
#include <iostream>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

ros::Publisher point_pub;
ros::Publisher ground_pub;
ros::Publisher no_ground_pub;
DepthCluster depthCluster(2, 0.2, 48, 10);

std::shared_ptr<tf2_ros::Buffer> buffer;
std::shared_ptr<tf2_ros::TransformListener> listener;
// std::shared_ptr<tf_ros::buffer> buffer_ros;

void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn;
    laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_ptr, *laserCloudIn);
    //  cloud clear

    for (size_t i = 0; i < depthCluster.min_pts_.size(); ++i)
    {
        depthCluster.once_filter(laserCloudIn, depthCluster.min_pts_[i], depthCluster.max_pts_[i]);
    }
    
    // end！
    try
    {
        auto base_link_trans = buffer->lookupTransform("base_link", "camera_init", ros::Time(0));
        pcl_ros::transformPointCloud(*laserCloudIn, *laserCloudIn, base_link_trans.transform);
    }
    catch (tf2::TransformException &e)
    {
        std::cout << e.what() << std::endl;
    }

    depthCluster.setInputCloud(laserCloudIn);
    vector<vector<int>> clustersIndex = depthCluster.getClustersIndex();
    for (auto cluster_vec : clustersIndex)
    {
        int intensity = rand() % 255;
        for (int i = 0; i < cluster_vec.size(); ++i)
        {
            laserCloudIn->points[cluster_vec[i]].intensity = intensity;
        }
    }

    auto ground_index = depthCluster.getGroundCloudIndices();
    int intensity = 100;
    for (int j : ground_index)
    {
        laserCloudIn->points[j].intensity = intensity;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud(*laserCloudIn, ground_index, *ground_points);

    sensor_msgs::PointCloud2 laserCloudTemp;
    pcl::toROSMsg(*ground_points, laserCloudTemp);
    laserCloudTemp.header.stamp = cloud_ptr->header.stamp;
    laserCloudTemp.header.frame_id = cloud_ptr->header.frame_id;
    ground_pub.publish(laserCloudTemp);

    auto cluster_indices = depthCluster.getMergedClustersIndex();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_points(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud(*laserCloudIn, cluster_indices, *cluster_points);
    pcl::toROSMsg(*cluster_points, laserCloudTemp);
    laserCloudTemp.header.stamp = cloud_ptr->header.stamp;
    laserCloudTemp.header.frame_id = cloud_ptr->header.frame_id;
    point_pub.publish(laserCloudTemp);

    auto no_ground_index = depthCluster.getNoGroundCloudIndices();
    // std::cout << ground_index.size() << " ";
    // std::cout << laserCloudIn->points.size() << "  " << no_ground_index.size()  << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_points(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud(*laserCloudIn, no_ground_index, *no_ground_points);
    pcl::toROSMsg(*no_ground_points, laserCloudTemp);
    laserCloudTemp.header.frame_id = "base_link";
    no_ground_pub.publish(laserCloudTemp);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_cluster");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    buffer = std::make_shared<tf2_ros::Buffer>(ros::Duration(5));
    listener = std::make_shared<tf2_ros::TransformListener>(*(buffer.get()), nh);

    ros::Subscriber point_sub = nh.subscribe("/cloud_registered", 1, callback);
    point_pub = nh.advertise<sensor_msgs::PointCloud2>("cluster_result", 1);
    ground_pub = nh.advertise<sensor_msgs::PointCloud2>("ground_result", 1);
    no_ground_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_base_link_no_ground", 1);
    ros::spin();
}