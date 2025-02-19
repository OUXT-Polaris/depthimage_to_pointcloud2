#ifndef DEPTH_IMAGE_2_POINT_CLOUD2_HPP
#define DEPTH_IMAGE_2_POINT_CLOUD2_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

namespace depthimage_to_pointcloud2
{
class Depthimage2Pointcloud2 : public rclcpp::Node
{
public:
    Depthimage2Pointcloud2(const rclcpp::NodeOptions & options);

private:
    void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr depth_image_msg);
    void rgbd_image_callback(
        const sensor_msgs::msg::Image::SharedPtr rgb_image,
        const sensor_msgs::msg::Image::SharedPtr depth_image
    );
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg);
    void projectDepthToPointCloud(const sensor_msgs::msg::Image::SharedPtr depth_image, const sensor_msgs::msg::Image::SharedPtr rgb_image);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sync_sub_;    
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sync_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_policy;
    message_filters::Synchronizer<approximate_policy> sync_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
};
}
#endif // DEPTH_IMAGE_2_POINT_CLOUD2_HPP 
