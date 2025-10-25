//
// Created by alex on 2020/4/12.
//
#include "depth_cluster.h"
using namespace std;
using namespace pcl;
using namespace Eigen;

DepthCluster::DepthCluster(float vertcal_resolution, float horizontal_resolution, int lidar_lines, int cluster_size)
    : vertcal_resolution_(vertcal_resolution), horizontal_resolution_(horizontal_resolution), lidar_lines_(lidar_lines), cluster_size_(cluster_size)
{
    initParams();
}

void DepthCluster::initParams()
{
    // ground move algorithm parameters
    sorted_Pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    // cluster algorithm parameters init
    image_cols_ = int(360.0 / horizontal_resolution_);
    image_rows_ = lidar_lines_;
    //    vertical_angle_range_ = 30;
    vertical_angle_min_ = -27;
    vertical_angle_max_ = 72;
    sensor_height_ = 0.2;
    min_pts_ =  std::vector<Eigen::Vector4f>{
        Eigen::Vector4f(6.02,-3,-1,0),
        Eigen::Vector4f(12.37,3,-1,0),
        // Eigen::Vector4f(0,-5,-1,0),

    };
     max_pts_ =  std::vector<Eigen::Vector4f>{
        Eigen::Vector4f(8.18,-2,1,0),
        Eigen::Vector4f(14.1,3.8,1,0),
        // Eigen::Vector4f(3,-4,1,0)

    };
    // if (!loadcubeParam("cube"))
    // {
    //     ROS_ERROR("Failed to load cube-Yaml");
    // }
}

void DepthCluster::setInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &msg)
{
    // cout << "---------------- Algorithm Start ----------------" << endl;
    vector<vector<int>> label_image(image_rows_, vector<int>(image_cols_, -1));
    // move ground point cloud
    exactGroundPoints(msg, label_image);
    /* maybe can use OpenMP */
    vector<float> depth_vec;
    auto depth_image = generateDepthImage(msg);
    labelComponents(depth_image, label_image, msg);
}

vector<int> DepthCluster::exactGroundPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr &msg, vector<vector<int>> &label_image)
{
    // clock_t time_start = clock();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZI>); // shared ptr, dont need to be deleted by self
    pcl::copyPointCloud(*msg, *cloud_copy);                                               //
    vector<int> ground_indices;
    pcl::PointCloud<pcl::PointXYZI>::Ptr initial_ground_points(new pcl::PointCloud<pcl::PointXYZI>); //
    extractInitialGroundPoint(cloud_copy, initial_ground_points);
    Eigen::Vector4f ground_params = estimatePlaneParams(initial_ground_points);
    ground_indices = exactGroundCloudIndices(msg, label_image, ground_params);
    // clock_t time_end = clock();
    // auto time_used = 1000.0*double(time_end - time_start)/(double) CLOCKS_PER_SEC;
    // cout << "Label Ground used time: " << time_used << " ms" << endl;
    return ground_indices;
}

vector<int> DepthCluster::exactGroundCloudIndices(pcl::PointCloud<pcl::PointXYZI>::Ptr &msg, vector<vector<int>> &label_image, Eigen::Vector4f &ground_params)
{
    // 1.
    int clouds_size = msg->points.size();
    MatrixXf points(clouds_size, 3);
    int j = 0;
    for (int i = 0; i < clouds_size; ++i)
    {
        points.row(j++) << msg->points[i].x, msg->points[i].y, msg->points[i].z;
    }
    MatrixXf normal = ground_params.head<3>();
    VectorXf d_matrix = points * normal; //
    vector<int> ground_points_indeices;
    vector<int> no_ground_points_indeices;
    for (int i = 0; i < j; ++i)
    {
        if (d_matrix[i] < -(0.2 - ground_params(3, 0)))
        {
            int row_index, col_index;
            if (!calculateCoordinate(msg->points[i], row_index, col_index))
            {
                no_ground_points_indeices.push_back(i);
                continue;
            }
            if (row_index < label_image.size() && col_index < label_image[0].size())
            {
                label_image[row_index][col_index] = 0; // 0 means ground
                ground_points_indeices.push_back(i);
            }
        }
        else
        {
            no_ground_points_indeices.push_back(i);
        }
    }
    no_ground_points_indices_ = no_ground_points_indeices;
    ground_points_indices_ = ground_points_indeices;
    return ground_points_indeices;
}

void DepthCluster::extractInitialGroundPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &initial_ground_points)
{
    sort(cloud->points.begin(), cloud->points.end(), [](const pcl::PointXYZI &a, const pcl::PointXYZI &b)
         { return a.z < b.z; });
    auto it = cloud->points.begin();

    for (auto &point : cloud->points)
    {
        if (point.z < -1.5 * sensor_height_)
        {
            it++;
        }
        else
        {
            break;
        }
    }

    cloud->points.erase(cloud->points.begin(), it); // 把低于安装高度1.5倍的点全都删掉
    int initial_ground_points_counts = 50;
    if (initial_ground_points_counts > cloud->points.size())
    {
        cout << "Too few points" << endl;
        return;
    }

    float ground_height = 0;
    for (int i = 0; i < initial_ground_points_counts; ++i)
    {
        ground_height += cloud->points[i].z;
    }

    ground_height /= initial_ground_points_counts;
    initial_ground_points->clear();
    for (auto point : cloud->points)
    {
        if (point.z < ground_height + 0.2)
        {
            initial_ground_points->points.push_back(point);
        }
    }
}

Eigen::Vector4f DepthCluster::estimatePlaneParams(pcl::PointCloud<pcl::PointXYZI>::Ptr &initial_ground_points)
{
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*initial_ground_points, cov, pc_mean);
    JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    MatrixXf normal = (svd.matrixU().col(2));           // 第三列为地面平面的法向量。 3*1
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();     // 前三个数 也就是xyz的均值 取地面点的均值
    float d = -(normal.transpose() * seeds_mean)(0, 0); // d!=0
    Eigen::Vector4f plane_parameters;
    plane_parameters(3, 0) = d;
    plane_parameters.block<3, 1>(0, 0) = normal.block<3, 1>(0, 0); //
    return plane_parameters;
}

vector<vector<PointInfo>> DepthCluster::generateDepthImage(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_fused_ptr)
{
    vector<vector<PointInfo>> depth_image(image_rows_, vector<PointInfo>(image_cols_, PointInfo(-1, -1)));
    // clock_t time_start = clock();
    auto cloud_size = cloud_fused_ptr->points.size();
    for (int i = 0; i < cloud_size; i++)
    {
        const pcl::PointXYZI &point = cloud_fused_ptr->points[i];
        if (isnan(point.x) || isnan(point.y) || isnan(point.z))
        {
            continue;
        }
        int row_index, col_index;
        if (!calculateCoordinate(point, row_index, col_index))
        {
            continue;
        }
        float depth = sqrt(point.x * point.x + point.y * point.y + point.z * point.z); //[2,104]
        if (depth < 0.15 || depth > 110)
        { // delete too long or too short points
            continue;
        }
        if (row_index < depth_image.size() && col_index < depth_image[0].size())
        {
            depth_image[row_index][col_index].depth_ = depth;
            depth_image[row_index][col_index].index_ = i;
        }
        else
        {
            std::cout << "-----------------" << std::endl;
            std::cout << row_index << " " << col_index << std::endl;
            std::cout << depth_image.size() << " " << depth_image[0].size() << std::endl;
        }
    }

    // clock_t time_end = clock();
    // auto time_used = 1000.0*double(time_end - time_start)/(double) CLOCKS_PER_SEC;
    // cout << "Depth Image used time: " << time_used << " ms" << endl;
    return depth_image;
}

void DepthCluster::labelComponents(const vector<vector<PointInfo>> &depth_image, vector<vector<int>> &label_image, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_msg)
{
    // 四邻域
    //    vector<pair<int8_t, int8_t>> neighbor = {{1,  0},
    //                                             {-1, 0},
    //                                             {0,  1},
    //                                             {0,  -1}};
    // 八邻域
    //    vector<pair<int8_t, int8_t>> neighbor = {{1,  0},
    //                                             {1,  1},
    //                                             {1,  -1},
    //                                             {-1, 0},
    //                                             {-1, -1},
    //                                             {-1, 1},
    //                                             {0,  1},
    //                                             {0,  -1}};

    vector<pair<int8_t, int8_t>> neighbor = {{1, 0},
                                             {1, 1},
                                             {1, -1},
                                             {-1, 0},
                                             {-1, -1},
                                             {-1, 1},
                                             {0, 1},
                                             {0, -1}};

    int label_val = 1;
    int rows = image_rows_;
    int cols = image_cols_;
    const double eps = 1.0e-6; //

    vector<vector<int>> clusters_indices_vec;
    // clock_t time_start = clock();
    for (int row = 0; row < rows; row++)
    {
        for (int col = 0; col < cols; col++)
        {
            if (label_image[row][col] == -1 && depth_image[row][col].depth_ > 0)
            { // has been labeled or ground points
                queue<pair<int, int>> q;
                q.push(make_pair(row, col));
                label_image[row][col] = label_val;

                vector<int> cluster_indices_vec;
                cluster_indices_vec.push_back(depth_image[row][col].index_);

                while (!q.empty())
                {
                    auto target_point = q.front();
                    q.pop();
                    for (const auto &neigh : neighbor)
                    {
                        int x = target_point.first + neigh.first;
                        int y = target_point.second + neigh.second;
                        pair<int, int> neigh_point = make_pair(x, y);

                        if (warpPoint(neigh_point) && judgmentCondition(depth_image, target_point, neigh_point) &&
                            label_image[neigh_point.first][neigh_point.second] == -1 && depth_image[neigh_point.first][neigh_point.second].depth_ > 0)
                        { // valid point
                            q.push(neigh_point);
                            label_image[neigh_point.first][neigh_point.second] = label_val;
                            cluster_indices_vec.push_back(depth_image[neigh_point.first][neigh_point.second].index_);
                        }
                    }
                }
                label_val++;
                if (cluster_indices_vec.size() > cluster_size_)
                {
                    clusters_indices_vec.push_back(cluster_indices_vec);
                }
            }
        }
    }

    // clock_t time_end = clock();
    // auto time_used = 1000.0*double(time_end - time_start)/(double) CLOCKS_PER_SEC;
    // cout << "Cluster Algorithm used time: " << time_used << " ms" << endl;

    auto totle_clusters_size = clusters_indices_vec.size();
    clusters_indices_vec_ = clusters_indices_vec;
    //    cout << "Total cluster nums: " << totle_clusters_size << endl;
}

bool DepthCluster::judgmentCondition(const vector<vector<PointInfo>> &depth_image, const pair<int, int> &target_point, const pair<int, int> &neigh_point)
{
    float distance_sum = fabs(depth_image[target_point.first][target_point.second].depth_ - depth_image[neigh_point.first][neigh_point.second].depth_);
    return distance_sum < 0.4;
}

vector<vector<int>> DepthCluster::getClustersIndex()
{
    return clusters_indices_vec_;
}

vector<int> DepthCluster::getGroundCloudIndices()
{
    return ground_points_indices_;
}

vector<int> DepthCluster::getNoGroundCloudIndices()
{
    return no_ground_points_indices_;
}

vector<int> DepthCluster::getMergedClustersIndex()
{
    vector<int> res_resturn;
    for (auto i : clusters_indices_vec_)
    {
        res_resturn.insert(res_resturn.end(), i.begin(), i.end());
    }
    return res_resturn;
}
bool DepthCluster::warpPoint(pair<int, int> &pt)
{
    if (pt.first < 0 || pt.first >= image_rows_)
        return false;
    if (pt.second < 0)
        pt.second += image_cols_;
    if (pt.second >= image_cols_)
        pt.second -= image_cols_;
    return true;
}

void DepthCluster::paramsReset()
{
    sorted_Pointcloud_->clear();
}

bool DepthCluster::calculateCoordinate(const pcl::PointXYZI &point, int &row, int &col)
{
    // 1. calculate angle
    float vertical_angle = atan2(point.z, sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI; // 范围是[-15,15]
    float horizon_angle = atan2(point.x, point.y) * 180 / M_PI;                                      //[-180,180]
    // 2. calculate current points row col coordinate
    int row_index = int((vertical_angle - vertical_angle_min_) / vertcal_resolution_); // 0-15

    if (row_index < 0)
    {
        row_index = 0;
    }
    else if (row_index > vertical_angle_max_)
    {
        row_index = vertical_angle_max_;
    }

    int col_index = -round((horizon_angle) / horizontal_resolution_) + image_cols_ / 2.0;
    if (col_index >= image_cols_)
        col_index -= image_cols_;
    if (col_index < 0 || col_index >= image_cols_)
    {
        return false;
    }
    row = row_index;
    col = col_index;
    return true;
}

// bool DepthCluster::loadcubeParam(const std::string &param_name)
// {
//     XmlRpc::XmlRpcValue cuboid_list;
//     if (!nh_.getParam(param_name, cuboid_list))
//     {
//         ROS_ERROR("Failed to get param '%s'", param_name.c_str());
//         return false;
//     }
//     if (cuboid_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
//     {
//         ROS_ERROR("Param '%s' is not an array", param_name.c_str());
//         return false;
//     }

//     min_pts_.clear();
//     max_pts_.clear();

//     for (int i = 0; i < cuboid_list.size(); ++i)
//     {
//         if (!cuboid_list[i].hasMember("min") || !cuboid_list[i].hasMember("max"))
//         {
//             ROS_ERROR("cuboid element %d does not have min/max", i);
//             continue;
//         }

//         XmlRpc::XmlRpcValue &min_val = cuboid_list[i]["min"];
//         XmlRpc::XmlRpcValue &max_val = cuboid_list[i]["max"];

//         if (min_val.getType() != XmlRpc::XmlRpcValue::TypeArray || max_val.getType() != XmlRpc::XmlRpcValue::TypeArray ||
//             min_val.size() != 4 || max_val.size() != 4)
//         {
//             ROS_ERROR("cuboid element %d min/max not 4 elements", i);
//             continue;
//         }

//         Eigen::Vector4f min_pt, max_pt;
//         for (int j = 0; j < 4; ++j)
//         {
//             // min_val[j]
//             if (min_val[j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
//             {
//                 min_pt[j] = static_cast<float>(static_cast<double>(min_val[j]));
//             }
//             else if (min_val[j].getType() == XmlRpc::XmlRpcValue::TypeInt)
//             {
//                 min_pt[j] = static_cast<float>(static_cast<int>(min_val[j]));
//             }
//             else
//             {
//                 ROS_ERROR("cuboid element %d min_val[%d] has unexpected type", i, j);
//                 return false;
//             }

//             // max_val[j]
//             if (max_val[j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
//             {
//                 max_pt[j] = static_cast<float>(static_cast<double>(max_val[j]));
//             }
//             else if (max_val[j].getType() == XmlRpc::XmlRpcValue::TypeInt)
//             {
//                 max_pt[j] = static_cast<float>(static_cast<int>(max_val[j]));
//             }
//             else
//             {
//                 ROS_ERROR("cuboid element %d max_val[%d] has unexpected type", i, j);
//                 return false;
//             }
//         }
//         min_pts_.push_back(min_pt);
//         max_pts_.push_back(max_pt);
//     }
//     ROS_INFO("Loaded %lu cuboids from param '%s'", min_pts_.size(), param_name.c_str());
//     return true;
// }

void DepthCluster::once_filter(
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in,
    Eigen::Vector4f &min_pt,
    Eigen::Vector4f &max_pt)
{

    // Check if cloud has points
    if (cloud_in->empty())
    {
        ROS_WARN("Empty cloud passed to filter");
        return;
    }
    // Create a new cloud for output to verify points are removed
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    // Create CropBox filter
    pcl::CropBox<pcl::PointXYZI> crop_box_filter;
    crop_box_filter.setInputCloud(cloud_in);
    crop_box_filter.setMin(min_pt);
    crop_box_filter.setMax(max_pt);
    crop_box_filter.setNegative(true); // Remove points inside the box
    crop_box_filter.filter(*cloud_filtered);
    *cloud_in = *cloud_filtered;
}