#pragma once

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/PointCloud2.h>

#include <cmath>

#define PI 3.1415926

class Processor
{
  private:
    std::string sub_topic_;
    std::string pub_topic_;
    
    bool crop_view_mode_;
    bool crop_range_mode_;
    bool filter_mode_;
    bool downsample_mode_;
    bool show_points_size_;
    bool show_time_;
    
    float view_number_;
    float field_of_view_;

    float sensor_height_;
    float view_higher_limit_;
    float view_lower_limit_;
    float min_distance_;
    float max_distance_;
    
    float meank_;
    float stdmul_;

    float leafsize_;
    
    ros::Subscriber sub_;
    ros::Publisher pub_;
    
    //视场裁剪
    void crop_view(const pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr out);
    //距离裁剪
    void crop_range(const pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr out);
    //回调函数
    void callback(const sensor_msgs::PointCloud2ConstPtr& in);

  public:
    Processor(ros::NodeHandle &nh);
    ~Processor();
    void Spin();
};

