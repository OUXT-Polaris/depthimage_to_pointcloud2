# depthimage_to_pointcloud2
A node that converts a depth image into a 3D point cloud

# subscribe
・depth_image : `sensor_msgs::msg::Image`  
・camera_info : `sensor_msgs::msg::CameraInfo`    
・rgb_image  (option) :　`sensor_msgs::msg::Image`

# publish
・point_cloud :　`sensor_msgs::msg::PointCloud2`

# parameters
|Name|Type|Description|
|---|---|---|---|
|image_type|string|image type(depth or rgbd)|
|depth_image_topic|string|depth img topic name|
|camera_info_topic|string|cam info topic name|
|rgb_image_topic|string|rgb image topic name|


# launch 
```
ros2 launch depthimage_to_pointcloud2 depth_to_pointcloud_launch.xml
```


