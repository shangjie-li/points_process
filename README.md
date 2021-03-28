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
   <param name="sub_topic" value="/pandar_points"/>
   <param name="pub_topic" value="/pandar_points_processed"/>

   <param name="crop_view_mode" value="true"/>
   <param name="crop_range_mode" value="true"/>
   <param name="filter_mode" value="false"/>
   <param name="downsample_mode" value="true"/>
   <param name="show_points_size" value="true"/>
   <param name="show_time" value="true"/>

   <param name="view_number" value="1"/>
   <param name="field_of_view" value="100"/>

   <param name="sensor_height" value="2.05"/>
   <param name="view_higher_limit" value="4.0"/>
   <param name="view_lower_limit" value="-4.0"/>
   <param name="min_distance" value="0.5"/>
   <param name="max_distance" value="50.0"/>

   <param name="meank" value="10"/>
   <param name="stdmul" value="1.0"/>

   <param name="leafsize" value="0.1"/>
   ```
    - `sub_topic`指明订阅的点云话题。
    - `pub_topic`指明发布的点云话题。
    - `view_number`为激光雷达视场区域编号，1为x正向，2为y负向，3为x负向，4为y正向。
    - `field_of_view`为水平视场角，单位度。
    - `sensor_height`指明传感器距地面高度，单位为米。
    - `view_higher_limit`和`view_lower_limit`指明期望的点云相对地面的限制高度，单位为米。
    - `min_distance`和`max_distance`指明期望的点云相对传感器的限制距离，单位为米。

## 运行
 - 启动`points_process`
   ```Shell
   roslaunch points_process points_process.launch
   ```




