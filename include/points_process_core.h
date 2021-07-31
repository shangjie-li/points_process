#pragma once

#include <ros/ros.h>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/PointCloud2.h>

#define PI 3.1415926

class Processor
{
private:
    std::string sub_topic_;
    std::string pub_topic_;
    
    bool crop_view_mode_;
    bool crop_range_mode_;
    bool downsample_mode_;
    bool filter_mode_;
    
    bool show_points_size_;
    bool show_time_;
    
    float view_number_;
    float field_of_view_;

    float sensor_height_;
    float view_higher_limit_;
    float view_lower_limit_;
    float min_distance_;
    float max_distance_;
    
    float leafsize_;
    
    float meank_;
    float stdmul_;
    
    ros::Subscriber sub_;
    ros::Publisher pub_;
    
    void cropView(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr out);
    void cropRange(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr out);
    void callback(const sensor_msgs::PointCloud2ConstPtr in);

public:
    Processor(ros::NodeHandle& nh);
    ~Processor();
};

