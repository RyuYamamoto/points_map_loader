#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

class PointsMapLoader
{
public:
  PointsMapLoader()
  {
    pnh_.param<std::string>("pcd_files", pcd_files_, "points_map.pcd");
    pnh_.param<bool>("use_downsample", use_downsample_, false);
    pnh_.param<double>("leaf_size", leaf_size_, 0.2);
    pnh_.param<double>("period", period_, 1.0);

    points_map_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("points_map", 1);

    map_publish_timer_ =
      nh_.createWallTimer(ros::WallDuration(period_), &PointsMapLoader::publish, this, true, true);

    points_map_ = loadPCD(pcd_files_);
  }
  ~PointsMapLoader() = default;

private:
  sensor_msgs::PointCloud2 loadPCD(const std::string pcd_files)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcd_files, *pcd);

    if (use_downsample_) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
      voxel_grid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
      voxel_grid.setInputCloud(pcd);

      voxel_grid.filter(*filtered);
      pcd = filtered;
    }
    sensor_msgs::PointCloud2 points_map_msg;
    pcl::toROSMsg(*pcd, points_map_msg);
    return points_map_msg;
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

  std::string pcd_files_;

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
