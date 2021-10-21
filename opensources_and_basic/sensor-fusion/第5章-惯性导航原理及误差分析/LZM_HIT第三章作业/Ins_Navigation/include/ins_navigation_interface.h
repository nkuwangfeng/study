/*
 * @Description: INS 接口
 * @Author: Zm Liu
 * @Date: 
 */
#ifndef INS_NAVIGATION_INTERFACE_H_
#define INS_NAVIGATION_INTERFACE_H_
#include "ins_navigation.h"
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <memory>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include "boost/filesystem.hpp"

enum SolutionMode
{
  Offline = 0,
  Online = 1
};

struct ResultPose
{
  ResultPose()
  {
  }
  ResultPose(const double& time, const double& p_x, const double& p_y, const double& p_z, const double& q_x,
             const double& q_y, const double& q_z, const double& q_w)
  {
    time_stamp = time;
    x = p_x;
    y = p_y;
    z = p_z;
    qw = q_w;
    qx = q_x;
    qy = q_y;
    qz = q_z;
  }
  double time_stamp;
  double x, y, z, qw, qx, qy, qz;
};

class InsNavigationInterface
{
public:
  InsNavigationInterface(const YAML::Node& node);
  void saveResult();
  void run();

private:
  ros::NodeHandle nh_;
  ros::Subscriber gnss_sub_;
  ros::Subscriber imu_sub_;

  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_broad_cast_;

  std::shared_ptr<InsNavigation> ins_navigation_;
  std::vector<ImuData> imu_datas_;
  std::vector<double> init_state_;

private:
  void initialize(const YAML::Node& node);
  bool readFromFile(const std::string& folder);
  void importCsvData(const std::string& file_name, std::vector<std::vector<double>>& data);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
  void poseCallback(const InsPose& pose);
  std::ofstream odom_ofstream_;
  SolutionMode solution_mode_;
  bool initialized_;
  std::vector<ResultPose> ref_poses_;
  std::vector<ResultPose> ins_poses_;
  std::string result_folder_;
  bool use_fixed_angle_axis_;
};
#endif
