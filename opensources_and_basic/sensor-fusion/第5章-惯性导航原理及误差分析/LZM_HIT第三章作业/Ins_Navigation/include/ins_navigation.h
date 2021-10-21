/*
 * @Description: INS 具体实现
 * @Author: Zm Liu
 * @Date: 
 */
#ifndef INS_NAVIGATION_H_
#define INS_NAVIGATION_H_
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <functional>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "geom_function.h"
struct InsPose
{
  InsPose()
  {
  }
  InsPose(const Eigen::Vector3d& posi, const Eigen::Vector3d& velo, const Eigen::Quaterniond& rot)
  {
    position = posi;
    velocity = velo;
    rotation = rot;
  }

  double time_stamp;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d velocity_b;
  Eigen::Quaterniond rotation;
  Eigen::Vector3d accel;
  Eigen::Vector3d gyro;
};

struct ImuData
{
  ImuData()
  {
  }
  ImuData(const double& timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& omega)
  {
    time_stamp = timestamp;
    accel = acc;
    gyro = omega;
  }
  double time_stamp;
  Eigen::Vector3d accel;
  Eigen::Vector3d gyro;
};

struct AngleIncrement
{
  bool valid = false;
  Eigen::Vector3d increment;
  double begin_time;
  double end_time;
};

struct InitState
{
  Eigen::Vector3d lla;
  Eigen::Vector3d vel_b;
  Eigen::Vector3d ypr;
};

class InsNavigation
{
public:
  InsNavigation(int order = 1);
  void setInitialState(const InitState& init_state);
  void setInitialPose(Eigen::Vector3d position, Eigen::Vector3d velocity, Eigen::Quaterniond quaternion);
  void setInitialPose(Eigen::Vector3d position, Eigen::Vector3d velocity, Eigen::Vector3d rpy);

  void solution(const double& timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro,
                bool use_fixed_angle_axis = false);
  void registerPoseCallback(std::function<void(const InsPose& pose)> cb);

  static const Eigen::Vector3d gravity_;

private:
  void updateNavigation(const Eigen::Vector3d& phi, const ImuData& current_imu_data);
  void runCallback(const InsPose& pose);

private:
  int order_;
  Eigen::Vector3d init_position_;
  Eigen::Vector3d init_velocity_;
  Eigen::Quaterniond init_quaternion_;

  std::deque<AngleIncrement> angle_increments_;
  std::deque<InsPose> latest_navigation_positions_;

  std::vector<double> angle_param_;

  ImuData last_imu_data_;
  ImuData current_imu_data_;

  InitState init_state_;
  InsPose init_pose_;
  Eigen::Vector3d gn_ = Eigen::Vector3d::Zero();

private:
  std::vector<std::function<void(const InsPose)>> pose_cb_;
};
#endif
