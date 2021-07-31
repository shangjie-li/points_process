#include "points_process_core.h"

Processor::Processor(ros::NodeHandle& nh)
{
    nh.param<std::string>("sub_topic", sub_topic_, "/pandar_points");
    nh.param<std::string>("pub_topic", pub_topic_, "/pandar_points_processed");
    
    nh.param<bool>("crop_view_mode", crop_view_mode_, false);
    nh.param<bool>("crop_range_mode", crop_range_mode_, false);
    nh.param<bool>("downsample_mode", downsample_mode_, false);
    nh.param<bool>("filter_mode", filter_mode_, false);
    
    nh.param<bool>("show_points_size", show_points_size_, false);
    nh.param<bool>("show_time", show_time_, false);
    
    nh.param<float>("view_number", view_number_, 1);
    nh.param<float>("field_of_view", field_of_view_, 90);

    nh.param<float>("sensor_height", sensor_height_, 2.0);
    nh.param<float>("view_higher_limit", view_higher_limit_, 4.0);
    nh.param<float>("view_lower_limit", view_lower_limit_, -4.0);
    nh.param<float>("min_distance", min_distance_, 2.0);
    nh.param<float>("max_distance", max_distance_, 50.0);

    nh.param<float>("leafsize", leafsize_, 0.2);
    
    nh.param<float>("meank", meank_, 5);
    nh.param<float>("stdmul", stdmul_, 1);

    sub_ = nh.subscribe(sub_topic_, 1, &Processor::callback, this);
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(pub_topic_, 1);
    
    ros::spin();
}

Processor::~Processor()
{
}

void Processor::cropView(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> clipper;

    clipper.setInputCloud(in);
    pcl::PointIndices indices;
    if(view_number_ == 1)
    {
        float alpha = 90 - 0.5 * field_of_view_;
        float k = tan(alpha * PI / 180.0f);
        
        #pragma omp for
        for(size_t i = 0; i < in->points.size(); i++)
        {
            if(in->points[i].x > k * in->points[i].y && in->points[i].x > -k * in->points[i].y) {continue;}
            else {indices.indices.push_back(i);}
        }
    }
    else if(view_number_ == 2)
    {
        float alpha = 90 - 0.5 * field_of_view_;
        float k = tan(alpha * PI / 180.0f);
        
        #pragma omp for
        for(size_t i = 0; i < in->points.size(); i++)
        {
            if(-in->points[i].y > k * in->points[i].x && -in->points[i].y > -k * in->points[i].x) {continue;}
            else {indices.indices.push_back(i);}
        }
    }
    else if(view_number_ == 3)
    {
        float alpha = 90 - 0.5 * field_of_view_;
        float k = tan(alpha * PI / 180.0f);
        
        #pragma omp for
        for(size_t i = 0; i < in->points.size(); i++)
        {
            if(-in->points[i].x > k * in->points[i].y && -in->points[i].x > -k * in->points[i].y) {continue;}
            else {indices.indices.push_back(i);}
        }
    }
    else if(view_number_ == 4)
    {
        float alpha = 90 - 0.5 * field_of_view_;
        float k = tan(alpha * PI / 180.0f);
        
        #pragma omp for
        for(size_t i = 0; i < in->points.size(); i++)
        {
            if(in->points[i].y > k * in->points[i].x && in->points[i].y > -k * in->points[i].x) {continue;}
            else {indices.indices.push_back(i);}
        }
    }
    clipper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    clipper.setNegative(true);
    clipper.filter(*out);
}

void Processor::cropRange(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                          const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> clipper;

    clipper.setInputCloud(in);
    pcl::PointIndices indices;
    
    #pragma omp for
    for(size_t i = 0; i < in->points.size(); i++)
    {
        if(in->points[i].z < view_higher_limit_ - sensor_height_ &&
           in->points[i].z > view_lower_limit_ - sensor_height_ &&
           in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y > min_distance_ * min_distance_ &&
           in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y < max_distance_ * max_distance_)
        {
            continue;
        }
        indices.indices.push_back(i);
    }
    clipper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    clipper.setNegative(true);
    clipper.filter(*out);
}

void Processor::callback(const sensor_msgs::PointCloud2ConstPtr in)
{
    ros::Time time_start = ros::Time::now();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_current(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_cropped_view(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_cropped_range(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    
    pcl::fromROSMsg(*in, *pc_current);
    int size_in = pc_current->points.size();
    
    // 视场裁剪
    if(crop_view_mode_) {cropView(pc_current, pc_cropped_view);}
    else {pc_cropped_view = pc_current;}
    
    // 距离裁剪
    if(crop_range_mode_) {cropRange(pc_cropped_view, pc_cropped_range);}
    else {pc_cropped_range = pc_cropped_view;}
    
    // PCL提供的体素栅格缩减采样算法，该算法将点云分解成体素voxel，并用子云的中心点代替每个体素voxel中包含的所有点
    // 以Pandar40为例，原始点云数量约为每帧140000，经过(0.1,0.1,0.1)缩减采样后约为每帧60000，经过(0.2,0.2,0.2)缩减采样后约为每帧40000
    if(downsample_mode_)
    {
        pcl::VoxelGrid<pcl::PointXYZI> voxelSampler;
        voxelSampler.setInputCloud(pc_cropped_range);
        voxelSampler.setLeafSize(leafsize_, leafsize_, leafsize_); // 设置体素voxel的大小
        voxelSampler.filter(*pc_downsampled);
    }
    else
    {
        pc_downsampled = pc_cropped_range;
    }
    
    // PCL提供的统计离群值剔除算法，该算法限定处于平均值附近的一个范围，并剔除偏离平均值太多的点
    // 该算法较为耗时，应在裁剪点云后进行
    if(filter_mode_)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> statFilter;
        statFilter.setInputCloud(pc_downsampled);
        statFilter.setMeanK(meank_); // 设置用来计算平均值的相邻点的数目（偏离平均值太多的点将被剔除）
        statFilter.setStddevMulThresh(stdmul_); // 设置标准偏差阈值的乘值（偏离平均值μ ± σ·stdmul以上的点认为是离群点）
        statFilter.filter(*pc_filtered);
    }
    else
    {
        pc_filtered = pc_downsampled;
    }
    
    sensor_msgs::PointCloud2 out;
    pcl::toROSMsg(*pc_filtered, out);
    out.header = in->header;
    pub_.publish(out);
    
    int size_out = pc_filtered->points.size();
    ros::Time time_end = ros::Time::now();

    if(show_points_size_ || show_time_)
    {
        std::cout << "" << std::endl;
        std::cout << "[points_process]" << std::endl;
    }

    if(show_points_size_)
    {
        std::cout << "Size of input point clouds: " << size_in << std::endl;
        std::cout << "Size of output point clouds: " << size_out << std::endl;
    }

    if(show_time_)
    {
        std::cout << "Time cost per frame: " << time_end - time_start << "s" << std::endl;
    }
}

