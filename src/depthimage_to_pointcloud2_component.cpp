#include <depthimage_to_pointcloud2/depthimage_to_pointcloud2_component.hpp>

namespace depthimage_to_pointcloud2
{
Depthimage2Pointcloud2::Depthimage2Pointcloud2(const rclcpp::NodeOptions & options)
: Node("depthimage_to_pointcloud2", options),
  sync_(approximate_policy(10),depth_sync_sub_,rgb_sync_sub_),
  buffer_(this->get_clock()),
  listener_(buffer_)
{
  declare_parameter<std::string>("image_type", "depth"); 
  declare_parameter<std::string>("depth_image_topic", "/camera/camera/depth/image_rect_raw");
  declare_parameter<std::string>("rgb_image_topic", "/camera/camera/color/image_raw");
  declare_parameter<std::string>("cam_info_topic", "/camera/camera/depth/camera_info");

  // Get parameters
  std::string image_type = get_parameter("image_type").as_string();
  std::string depth_image_topic = get_parameter("depth_image_topic").as_string();
  std::string rgb_image_topic = get_parameter("rgb_image_topic").as_string();
  std::string cam_info_topic = get_parameter("cam_info_topic").as_string();
    
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    cam_info_topic, 10, [this](const sensor_msgs::msg::CameraInfo::SharedPtr cam_info_msg) {
    camera_info_callback(cam_info_msg);
  });

  point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);

  if (image_type == "depth"){
    depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_image_topic, 10, [this](const sensor_msgs::msg::Image::SharedPtr depth_image_msg) {
      depth_image_callback(depth_image_msg);
    });
  }else if (image_type == "rgbd") {
    depth_sync_sub_.subscribe(this, depth_image_topic);
    rgb_sync_sub_.subscribe(this, rgb_image_topic); 
    sync_.registerCallback(&Depthimage2Pointcloud2::rgbd_image_callback, this);       
  }
}

void Depthimage2Pointcloud2::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr cam_info_msg
)
{ 
  cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*cam_info_msg);
}

void Depthimage2Pointcloud2::depth_image_callback(
  const sensor_msgs::msg::Image::SharedPtr depth_image_msg
)
{
  if (nullptr == cam_info_) {
    RCLCPP_WARN(this->get_logger(), "No camera info, skipping point cloud conversion");
    return;
  }
  projectDepthToPointCloud(depth_image_msg, nullptr);
}

void Depthimage2Pointcloud2::rgbd_image_callback(
  const sensor_msgs::msg::Image::SharedPtr depth_image,
  const sensor_msgs::msg::Image::SharedPtr rgb_image
){
  if (nullptr == cam_info_) {
    RCLCPP_WARN(this->get_logger(), "No camera info, skipping point cloud conversion");
    return;
  }
  projectDepthToPointCloud(depth_image, rgb_image);
}

void Depthimage2Pointcloud2::projectDepthToPointCloud(
  const sensor_msgs::msg::Image::SharedPtr depth_msg,
  const sensor_msgs::msg::Image::SharedPtr rgb_msg
)
{
  const auto fx = cam_info_->k[0];
  const auto fy = cam_info_->k[4];
  const auto cx = cam_info_->k[2];
  const auto cy = cam_info_->k[5];

  auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width = depth_msg->width;
  cloud_msg->is_dense = false;

  const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
  sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
  if (rgb_msg) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }
  modifier.resize(depth_msg->width * depth_msg->height);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

  std::optional<sensor_msgs::PointCloud2Iterator<uint8_t>> iter_r, iter_g, iter_b;
  if (rgb_msg) {
    auto rgb_cv_image = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding);
    cv::Mat rgb_mat = rgb_cv_image->image;

    iter_r = sensor_msgs::PointCloud2Iterator<uint8_t>(*cloud_msg, "r");
    iter_g = sensor_msgs::PointCloud2Iterator<uint8_t>(*cloud_msg, "g");
    iter_b = sensor_msgs::PointCloud2Iterator<uint8_t>(*cloud_msg, "b");

    for (size_t v = 0; v < depth_msg->height; ++v) {
      for (size_t u = 0; u < depth_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z) {
        uint16_t depth = depth_data[v * depth_msg->width + u];
        if (depth == 0) {
          *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
          *(*iter_r) = *(*iter_g) = *(*iter_b) = 0;
          ++(*iter_r);
          ++(*iter_g);
          ++(*iter_b);
          continue;
        }

        float z = static_cast<float>(depth) * 0.001f; // [mm] to [m]
        *iter_x = (static_cast<float>(u) - cx) * z / fx;
        *iter_y = (static_cast<float>(v) - cy) * z / fy;
        *iter_z = z;

        cv::Vec3b color = rgb_mat.at<cv::Vec3b>(v, u);
        *(*iter_r) = color[0];  // R
        *(*iter_g) = color[1];  // G
        *(*iter_b) = color[2];  // B

        ++(*iter_r);
        ++(*iter_g);
        ++(*iter_b);
      }
    }
  } else {
    for (size_t v = 0; v < depth_msg->height; ++v) {
      for (size_t u = 0; u < depth_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z) {
        uint16_t depth = depth_data[v * depth_msg->width + u];
        if (depth == 0) {
          *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
          continue;
        }

        float z = static_cast<float>(depth) * 0.001f; // [mm] to [m]
        *iter_x = (static_cast<float>(u) - cx) * z / fx;
        *iter_y = (static_cast<float>(v) - cy) * z / fy;
        *iter_z = z;
      }
    }
  }

  point_cloud_pub_->publish(*cloud_msg);
}
}
  
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(depthimage_to_pointcloud2::Depthimage2Pointcloud2)


