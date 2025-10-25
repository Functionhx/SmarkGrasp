#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "../include/depth_cluster.hpp"
#include <iostream>
pclfilter::DepthCluster depthCluster(1, 0.2, 32, 20);
ros::Publisher point_pub;
ros::Publisher ground_pub;
void CloudCB(const sensor_msgs::PointCloud2ConstPtr &cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_ptr, *laserCloudIn);

    // 步骤2：深度聚类处理
    depthCluster.setInputCloud(laserCloudIn); // 输入点云到算法

    // 步骤3：获取聚类结果
    vector<vector<int>> clustersIndex = depthCluster.getClustersIndex();
    // 为每个聚类分配随机强度值（可视化区分）
    // for (auto cluster_vec : clustersIndex)
    // {
    //     int intensity = rand() % 255; // 随机强度值0-254
    //     for (int idx : cluster_vec)
    //     {
    //         laserCloudIn->points[idx].intensity = intensity; // 同簇同强度
    //     }
    // }

    // 步骤4：处理地面点云
    // auto ground_index = depthCluster.getGroundCloudIndices();
    // const int GROUND_INTENSITY = 100; // 地面点固定强度值
    // for (int idx : ground_index)
    // {
    //     laserCloudIn->points[idx].intensity = GROUND_INTENSITY;
    // }

    // 步骤5：发布地面点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::copyPointCloud(*laserCloudIn, ground_index, *ground_points);
    sensor_msgs::PointCloud2 ground_msg;
    pcl::toROSMsg(*ground_points, ground_msg);
    ground_msg.header = cloud_ptr->header;
    ground_pub.publish(ground_msg);

    // 步骤6：发布障碍物聚类结果
    auto cluster_indices = depthCluster.getMergedClustersIndex();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_points(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*laserCloudIn, cluster_indices, *cluster_points);
    sensor_msgs::PointCloud2 cluster_msg;
    pcl::toROSMsg(*cluster_points, cluster_msg);
    cluster_msg.header = cloud_ptr->header;
    point_pub.publish(cluster_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_cluster");
    ros::NodeHandle nh;
    ros::Subscriber point_sub = nh.subscribe("/cloud_registered", 1, CloudCB);
    point_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_filter_baselink", 1); // 障碍物
    ground_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_baselink", 1);       // 地面
    ros::spin();
    return 0;
}

// TODO:
// 点云坐标系转换
// 参数调整