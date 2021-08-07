#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

class PointsMapLoader
{
using PointType = pcl::PointXYZ;

public:
  PointsMapLoader()
  {
    pnh_.param<std::string>("map_path", map_path_, "points_map.pcd");
    pnh_.param<bool>("use_downsample", use_downsample_, false);
    pnh_.param<double>("leaf_size", leaf_size_, 0.2);
    pnh_.param<double>("period", period_, 1.0);

    points_map_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("points_map", 5, true);

    map_publish_timer_ =
      nh_.createWallTimer(ros::WallDuration(period_), &PointsMapLoader::publish, this, true, true);

    points_map_ = loadPCD(map_path_);
  }
  ~PointsMapLoader() = default;

private:
  sensor_msgs::PointCloud2 loadPCD(const std::string map_path)
  {
    pcl::PointCloud<PointType>::Ptr pcd(new pcl::PointCloud<PointType>);
    if(pcl::io::loadPCDFile(map_path, *pcd) == -1) {
      ROS_ERROR("map file is not found.");
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

    geometry_msgs::Vector3 vec;
    for(const auto &p : pcd->points) {
      vec.x += p.x;
      vec.y += p.y;
      vec.z += p.z;
    }
    vec.x /= pcd->points.size();
    vec.y /= pcd->points.size();
    vec.z /= pcd->points.size();
    publishMapTF(vec);

    sensor_msgs::PointCloud2 points_map_msg;
    pcl::toROSMsg(*pcd, points_map_msg);
    return points_map_msg;
  }
  void publishMapTF(const geometry_msgs::Vector3 vec)
  {
    static tf2_ros::StaticTransformBroadcaster map_tf_broadcaster;
    geometry_msgs::TransformStamped map_transform;

    map_transform.header.stamp = ros::Time::now();
    map_transform.header.frame_id = "map";
    map_transform.child_frame_id = "world";
    map_transform.transform.translation.x = vec.x;
    map_transform.transform.translation.y = vec.y;
    map_transform.transform.translation.z = vec.z;
    map_transform.transform.rotation.w = 1.0;
    map_transform.transform.rotation.x = 0.0;
    map_transform.transform.rotation.y = 0.0;
    map_transform.transform.rotation.z = 0.0;
    ROS_INFO("x: %f y: %f z: %f", vec.x, vec.y, vec.z);

    map_tf_broadcaster.sendTransform(map_transform);
  }
  void publish(const ros::WallTimerEvent & event)
  {
    points_map_.header.frame_id = "map";
    points_map_.header.stamp = ros::Time(0);
    points_map_publisher_.publish(points_map_);
  }

private:
  ros::NodeHandle nh_{};
  ros::NodeHandle pnh_{"~"};

  ros::WallTimer map_publish_timer_;
  ros::Publisher points_map_publisher_;

  std::string map_path_;

  sensor_msgs::PointCloud2 points_map_;

  bool use_downsample_;
  double leaf_size_;
  double period_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "points_map_loader");
  PointsMapLoader points_map_loader;
  ros::spin();
  return 0;
}
