# points_process

ROS package for processing lidar points

## 安装
 - 建立工作空间并拷贝这个库
   ```Shell
   mkdir -p ros_ws/src
   cd ros_ws/src
   git clone https://github.com/shangjie-li/points_process.git
   cd ..
   catkin_make
   ```

## 参数配置
 - 修改`points_process/launch/points_process.launch`
   ```Shell
   <param name="sub_topic" value="/velodyne_points"/>
   <param name="pub_topic" value="/velodyne_points_processed"/>
        
   <param name="is_limit_mode" value="true"/>
   <param name="is_clip_mode" value="true"/>
   <param name="is_filter_mode" value="false"/>
   <param name="is_downsample_mode" value="true"/>
   <param name="is_show_points_size" value="true"/>
        
   <param name="the_view_number" value="1"/>
   <param name="the_field_of_view" value="100"/>

   <param name="the_sensor_height" value="1.0"/>
   <param name="the_view_higher_limit" value="2.0"/>
   <param name="the_view_lower_limit" value="0.0"/>
   <param name="the_min_distance" value="0.5"/>
   <param name="the_max_distance" value="100.0"/>
        
   <param name="the_meank" value="10"/>
   <param name="the_stdmul" value="1.0"/>
   <param name="the_leafsize" value="0.02"/>
   ```
    - `sub_topic`指明订阅的点云话题。
    - `pub_topic`指明发布的点云话题。
    - `the_view_number`为激光雷达视场区域编号，1为x正向，2为y负向，3为x负向，4为y正向。
    - `the_field_of_view`为水平视场角，单位度。
    - `the_sensor_height`指明传感器距地面高度，单位为米。
    - `the_view_higher_limit`和`the_view_lower_limit`指明期望的点云相对地面的限制高度，单位为米。
    - `the_min_distance`和`the_max_distance`指明期望的点云相对传感器的限制距离，单位为米。

## 运行
 - 启动`points_process`
   ```Shell
   roslauch points_process points_process.launch
   ```




