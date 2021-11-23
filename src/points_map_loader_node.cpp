#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

class PointsMapLoader : public rclcpp::Node
{
using PointType = pcl::PointXYZ;

public:
  PointsMapLoader(const rclcpp::NodeOptions & options) : Node("points_map_loader", options)
  {
    map_path_ = this->declare_parameter("map_path", "points_map.pcd");
    use_downsample_ = this->declare_parameter("use_downsample", false);
    leaf_size_ = this->declare_parameter("leaf_size", 0.2);
    period_ = this->declare_parameter("period", 1.0);

    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    rclcpp::QoS qos{1};
    qos.transient_local();
    qos.keep_last(1);
    points_map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_map", qos);

    points_map_ = loadPCD(map_path_);

    publish(points_map_);
  }
  ~PointsMapLoader() = default;

private:
  sensor_msgs::msg::PointCloud2 loadPCD(const std::string map_path)
  {
    pcl::PointCloud<PointType>::Ptr pcd(new pcl::PointCloud<PointType>);
    if(pcl::io::loadPCDFile(map_path, *pcd) == -1) {
      RCLCPP_ERROR(get_logger(), "map file is not found.");
      exit(-1);
    }

    if (use_downsample_) {
      pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>);
      pcl::VoxelGrid<PointType> voxel_grid;
      voxel_grid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
      voxel_grid.setInputCloud(pcd);

      voxel_grid.filter(*filtered);
      pcd = filtered;
    }

    geometry_msgs::msg::Vector3 vec;
    for(const auto &p : pcd->points) {
      vec.x += p.x;
      vec.y += p.y;
      vec.z += p.z;
    }
    vec.x /= pcd->points.size();
    vec.y /= pcd->points.size();
    vec.z /= pcd->points.size();
    publishMapTF(vec);

    sensor_msgs::msg::PointCloud2 points_map_msg;
    pcl::toROSMsg(*pcd, points_map_msg);
    return points_map_msg;
  }
  void publishMapTF(const geometry_msgs::msg::Vector3 vec)
  {
    geometry_msgs::msg::TransformStamped map_transform;

    map_transform.header.stamp = rclcpp::Clock().now();
    map_transform.header.frame_id = "map";
    map_transform.child_frame_id = "world";
    map_transform.transform.translation.x = vec.x;
    map_transform.transform.translation.y = vec.y;
    map_transform.transform.translation.z = vec.z;
    map_transform.transform.rotation.w = 1.0;
    map_transform.transform.rotation.x = 0.0;
    map_transform.transform.rotation.y = 0.0;
    map_transform.transform.rotation.z = 0.0;

    static_broadcaster_->sendTransform(map_transform);
  }
  void publish(sensor_msgs::msg::PointCloud2 &map)
  {
    map.header.frame_id = "map";
    points_map_publisher_->publish(map);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_map_publisher_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  sensor_msgs::msg::PointCloud2 points_map_;

  std::string map_path_;
  bool use_downsample_;
  double leaf_size_;
  double period_;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PointsMapLoader)
