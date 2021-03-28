#include "points_process_core.h"

Processor::Processor(ros::NodeHandle &nh)
{
    nh.param<std::string>("sub_topic", sub_topic_, "/pandar_points");
    nh.param<std::string>("pub_topic", pub_topic_, "/pandar_points_processed");
    
    nh.param<bool>("crop_view_mode", crop_view_mode_, false);
    nh.param<bool>("crop_range_mode", crop_range_mode_, false);
    nh.param<bool>("filter_mode", filter_mode_, false);
    nh.param<bool>("downsample_mode", downsample_mode_, false);
    nh.param<bool>("show_points_size", show_points_size_, false);
    nh.param<bool>("show_time", show_time_, false);
    
    //视场区域编号，1为x正向，2为y负向，3为x负向，4为y正向
    nh.param<float>("view_number", view_number_, 1);
    //水平视场角，单位度
    nh.param<float>("field_of_view", field_of_view_, 90);

    //以地面为基准，设置雷达高度、高度裁剪上限、高度裁剪下限
    nh.param<float>("sensor_height", sensor_height_, 2.0);
    nh.param<float>("view_higher_limit", view_higher_limit_, 4.0);
    nh.param<float>("view_lower_limit", view_lower_limit_, -4.0);
    //设置近处裁剪极限、远处裁剪极限
    nh.param<float>("min_distance", min_distance_, 2.0);
    nh.param<float>("max_distance", max_distance_, 50.0);

    //设置滤波算法中用来计算平均值的相邻点的数目、标准偏差阈值的乘值
    nh.param<float>("meank", meank_, 5);
    nh.param<float>("stdmul", stdmul_, 1);

    //设置缩减采样算法中体素voxel的大小
    nh.param<float>("leafsize", leafsize_, 0.2);
    
    sub_ = nh.subscribe(sub_topic_, 1, &Processor::callback, this);
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(pub_topic_, 1);
    
    ros::spin();
}

Processor::~Processor(){}

void Processor::Spin(){}

//视场裁剪
void Processor::crop_view(const pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZ> clipper;//创建ExtractIndices对象

    clipper.setInputCloud(in);//输入点云
    pcl::PointIndices indices;//创建索引
    if (view_number_ == 1)
    {
        float alpha = 90 - 0.5 * field_of_view_;
        float k = tan(alpha * PI / 180.0f);
        //pragma omp for语法是OpenMP的并行化语法，即希望通过OpenMP并行化执行这条语句后的for循环，从而起到加速效果
        #pragma omp for
        for (size_t i = 0; i < in->points.size(); i++)
        {
            if (in->points[i].x > k * in->points[i].y && in->points[i].x > -k * in->points[i].y)
            {
                continue;
            }
            else
            {
                indices.indices.push_back(i);//记录点的索引
            }
        }
    }
    else if (view_number_ == 2)
    {
        float alpha = 90 - 0.5 * field_of_view_;
        float k = tan(alpha * PI / 180.0f);
        //pragma omp for语法是OpenMP的并行化语法，即希望通过OpenMP并行化执行这条语句后的for循环，从而起到加速效果
        #pragma omp for
        for (size_t i = 0; i < in->points.size(); i++)
        {
            if (-in->points[i].y > k * in->points[i].x && -in->points[i].y > -k * in->points[i].x)
            {
                continue;
            }
            else
            {
                indices.indices.push_back(i);//记录点的索引
            }
        }
    }
    else if (view_number_ == 3)
    {
        float alpha = 90 - 0.5 * field_of_view_;
        float k = tan(alpha * PI / 180.0f);
        //pragma omp for语法是OpenMP的并行化语法，即希望通过OpenMP并行化执行这条语句后的for循环，从而起到加速效果
        #pragma omp for
        for (size_t i = 0; i < in->points.size(); i++)
        {
            if (-in->points[i].x > k * in->points[i].y && -in->points[i].x > -k * in->points[i].y)
            {
                continue;
            }
            else
            {
                indices.indices.push_back(i);//记录点的索引
            }
        }
    }
    else if (view_number_ == 4)
    {
        float alpha = 90 - 0.5 * field_of_view_;
        float k = tan(alpha * PI / 180.0f);
        //pragma omp for语法是OpenMP的并行化语法，即希望通过OpenMP并行化执行这条语句后的for循环，从而起到加速效果
        #pragma omp for
        for (size_t i = 0; i < in->points.size(); i++)
        {
            if (in->points[i].y > k * in->points[i].x && in->points[i].y > -k * in->points[i].x)
            {
                continue;
            }
            else
            {
                indices.indices.push_back(i);//记录点的索引
            }
        }
    }
    clipper.setIndices(boost::make_shared<pcl::PointIndices>(indices));//输入索引
    clipper.setNegative(true); //移除索引的点
    clipper.filter(*out);//输出点云
}

//距离裁剪
void Processor::crop_range(const pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZ> clipper;//创建ExtractIndices对象

    clipper.setInputCloud(in);//输入点云
    pcl::PointIndices indices;//创建索引
    //pragma omp for语法是OpenMP的并行化语法，即希望通过OpenMP并行化执行这条语句后的for循环，从而起到加速效果
    #pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        if (in->points[i].z < view_higher_limit_ - sensor_height_ &&
            in->points[i].z > view_lower_limit_ - sensor_height_ &&
            in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y > min_distance_ * min_distance_ &&
            in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y < max_distance_ * max_distance_)
        {
            continue;
        }
        indices.indices.push_back(i);//记录点的索引
    }
    clipper.setIndices(boost::make_shared<pcl::PointIndices>(indices));//输入索引
    clipper.setNegative(true); //移除索引的点
    clipper.filter(*out);//输出点云
}

//回调函数
void Processor::callback(const sensor_msgs::PointCloud2ConstPtr& in){
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_view_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_range_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromROSMsg(*in, *current_pc_ptr);
    
    ros::Time time_start = ros::Time::now();

    int size_in = current_pc_ptr->points.size();
    
    //视场裁剪
    if (crop_view_mode_)
    {
        crop_view(current_pc_ptr, cropped_view_pc_ptr);
    }
    else
    {
        cropped_view_pc_ptr = current_pc_ptr;
    }
    
    //距离裁剪
    if (crop_range_mode_)
    {
        crop_range(cropped_view_pc_ptr, cropped_range_pc_ptr);
    }
    else
    {
        cropped_range_pc_ptr = cropped_view_pc_ptr;
    }
    
    //PCL提供的统计离群值剔除算法，该算法限定处于平均值附近的一个范围，并剔除偏离平均值太多的点
    //该算法较为耗时，应在裁剪点云后进行
    if (filter_mode_)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;//定义滤波器statFilter
        statFilter.setInputCloud(cropped_range_pc_ptr);//输入点云
        statFilter.setMeanK(meank_);//设置用来计算平均值的相邻点的数目（偏离平均值太多的点将被剔除）
        statFilter.setStddevMulThresh(stdmul_);//设置标准偏差阈值的乘值（偏离平均值μ ± σ·stdmul以上的点认为是离群点）
        statFilter.filter(*filtered_pc_ptr);//输出点云
    }
    else
    {
        filtered_pc_ptr = cropped_range_pc_ptr;
    }
    
    //PCL提供的体素栅格缩减采样算法，该算法将点云分解成体素voxel，并用子云的中心点代替每个体素voxel中包含的所有点
    //以Pandar40为例，原始点云数量约为每帧140000，经过(0.1,0.1,0.1)缩减采样后约为每帧60000，经过(0.2,0.2,0.2)缩减采样后约为每帧40000
    if (downsample_mode_)
    {
        pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;//定义滤波器voxelSampler
        voxelSampler.setInputCloud(filtered_pc_ptr);//输入点云
        voxelSampler.setLeafSize(leafsize_, leafsize_, leafsize_);//设置体素voxel的大小
        voxelSampler.filter(*downsampled_pc_ptr);//输出点云
    }
    else
    {
        downsampled_pc_ptr = filtered_pc_ptr;
    }
    
    int size_out = downsampled_pc_ptr->points.size();

    ros::Time time_end = ros::Time::now();

    if (show_points_size_ || show_time_)
    {
        std::cout<<""<<std::endl;
    }

    if (show_points_size_)
    {
        std::cout<<"input size of point clouds:"<<size_in<<std::endl;
        std::cout<<"output size of point clouds:"<<size_out<<std::endl;
    }

    if (show_time_)
    {
        std::cout<<"cost time:"<<time_end - time_start<<"s"<<std::endl;
    }

    sensor_msgs::PointCloud2 out;
    pcl::toROSMsg(*downsampled_pc_ptr, out);
    
    out.header = in->header;
    pub_.publish(out);
}

