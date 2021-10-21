/*
 * @Description: INS 具体实现
 * @Author: Zm Liu
 * @Date: 
 */
#include "ins_navigation.h"

InsNavigation::InsNavigation(int order)
{
  order_ = order;
  angle_param_.resize(order + 1);
}

void InsNavigation::registerPoseCallback(std::function<void(const InsPose& pose)> cb)
{
  pose_cb_.emplace_back(cb);
}

void InsNavigation::setInitialState(const InitState& init_state)
{
  init_state_ = init_state;
  std::cout << "init_state_.lla is: " << init_state_.lla.transpose()
            << " init_state_.vel_b is: " << init_state_.vel_b.transpose()
            << " init_state_.ypr is: " << init_state_.ypr.transpose() << std::endl;
  Eigen::Vector3d position = lla2ecef(init_state_.lla[0], init_state_.lla[1], init_state_.lla[2]);
  std::cout << "init position i###########s: " << std::setprecision(15) << position << std::endl;
  InsPose pose;
  pose.position = position;

  tf::Quaternion quaternion;
  quaternion.setEulerZYX(init_state_.ypr[0], init_state_.ypr[1], init_state_.ypr[2]);
  init_quaternion_.x() = quaternion.x();
  init_quaternion_.y() = quaternion.y();
  init_quaternion_.z() = quaternion.z();
  init_quaternion_.w() = quaternion.w();

  pose.rotation =
      Eigen::Quaterniond(init_quaternion_.w(), init_quaternion_.x(), init_quaternion_.y(), init_quaternion_.z());

  pose.velocity_b = init_state_.vel_b;

  auto c_bn = pose.rotation.matrix().transpose();
  std::cout << "c_bn is: " << std::endl;
  std::cout << c_bn << std::endl;

  pose.velocity = c_bn.transpose() * pose.velocity_b;

  latest_navigation_positions_.push_back(pose);

  auto earth_param = getEarthParameter(init_state_.lla[0], init_state_.lla[1], init_state_.lla[2]);
  gn_[2] = earth_param.g;
  pose.position -= init_pose_.position;
}

void InsNavigation::setInitialPose(Eigen::Vector3d position, Eigen::Vector3d velocity, Eigen::Quaterniond rotation)
{
}

void InsNavigation::setInitialPose(Eigen::Vector3d position, Eigen::Vector3d velocity, Eigen::Vector3d ypr)
{
  tf::Quaternion quaternion;
  quaternion.setEulerZYX(ypr[0], ypr[1], ypr[2]);
  init_quaternion_.x() = quaternion.x();
  init_quaternion_.y() = quaternion.y();
  init_quaternion_.z() = quaternion.z();
  init_quaternion_.w() = quaternion.w();

  init_position_ = position;
  init_velocity_ = velocity;

  InsPose pose(position, velocity, init_quaternion_);

  latest_navigation_positions_.push_back(pose);
  runCallback(pose);
}

void InsNavigation::solution(const double& timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro,
                             bool use_fixed_angle_axis)
{
  // std::cout << "current imu time is: " << timestamp << std::endl;
  static int imu_frame_idx = 0;
  current_imu_data_ = ImuData(timestamp, acc, gyro);
  if (imu_frame_idx == 0)
  {
    latest_navigation_positions_.front().time_stamp = timestamp;
    latest_navigation_positions_.front().accel = acc;
    latest_navigation_positions_.front().gyro = gyro;
    init_pose_ = latest_navigation_positions_.front();
    runCallback(latest_navigation_positions_.back());
  }
  else
  {
    if (use_fixed_angle_axis)
    {
      double dt = current_imu_data_.time_stamp - last_imu_data_.time_stamp;
      Eigen::Vector3d angle_increment = last_imu_data_.gyro * dt;

      auto last_navigation_info = latest_navigation_positions_.front();

      auto c_bn = last_navigation_info.rotation.matrix().transpose();
      Eigen::Vector3d vel_n =
          last_navigation_info.velocity + (c_bn.transpose() * last_navigation_info.accel + gn_) * dt;
      Eigen::Vector3d position = last_navigation_info.position + last_navigation_info.velocity * dt;

      InsPose pose;
      pose.time_stamp = current_imu_data_.time_stamp;
      pose.accel = current_imu_data_.accel;
      pose.gyro = current_imu_data_.gyro;
      pose.position = position;
      pose.velocity = vel_n;

      auto delta_phi = last_imu_data_.gyro * dt;
      auto abs_delta_phi = delta_phi.norm();
      auto q_w = cos(abs_delta_phi / 2.0);
      auto delta_q_xyz = delta_phi / abs_delta_phi * sin(abs_delta_phi / 2.0);
      Eigen::Quaterniond delta_quad(q_w, delta_q_xyz[0], delta_q_xyz[1], delta_q_xyz[2]);

      auto current_rotation = last_navigation_info.rotation * delta_quad;
      current_rotation.normalized();
      pose.rotation = current_rotation;
      pose.velocity_b = pose.rotation.matrix().transpose() * pose.velocity;

      latest_navigation_positions_.pop_front();
      latest_navigation_positions_.push_back(pose);
      runCallback(pose);
    }
    else
    {
      Eigen::Vector3d delta_phi;
      if (imu_frame_idx <= order_)
      {
        AngleIncrement angle_increment;
        angle_increment.valid = true;
        angle_increment.increment = (current_imu_data_.time_stamp - last_imu_data_.time_stamp) * 0.5 *
                                    (current_imu_data_.gyro + last_imu_data_.gyro);
        angle_increment.begin_time = last_imu_data_.time_stamp;
        angle_increment.end_time = current_imu_data_.time_stamp;
        angle_increments_.push_back(angle_increment);
        delta_phi = angle_increment.increment;
      }
      else
      {
        AngleIncrement current_angle_increment;
        current_angle_increment.valid = true;
        current_angle_increment.increment = (current_imu_data_.time_stamp - last_imu_data_.time_stamp) * 0.5 *
                                            (current_imu_data_.gyro + last_imu_data_.gyro);
        current_angle_increment.begin_time = last_imu_data_.time_stamp;
        current_angle_increment.end_time = current_imu_data_.time_stamp;

        AngleIncrement last_angle_increment = angle_increments_.front();
        angle_increments_.pop_front();

        angle_increments_.push_back(current_angle_increment);

        double T = current_angle_increment.end_time - last_angle_increment.begin_time;

        auto delta_1 = last_angle_increment.increment;
        auto delta_2 = current_angle_increment.increment;
        delta_phi = delta_1 + delta_2 + 2 / 3.0 * (delta_1.cross(delta_2));
        // std::cout << "phi is: " << phi.transpose() << std::endl;
      }
      updateNavigation(delta_phi, current_imu_data_);
    }
  }
  last_imu_data_ = current_imu_data_;

  imu_frame_idx++;
}

void InsNavigation::updateNavigation(const Eigen::Vector3d& delta_phi, const ImuData& current_imu_data)
{
  auto abs_delta_phi = delta_phi.norm();
  auto q_w = cos(abs_delta_phi / 2.0);
  auto delta_q_xyz = delta_phi / abs_delta_phi * sin(abs_delta_phi / 2.0);
  Eigen::Quaterniond delta_quad(q_w, delta_q_xyz[0], delta_q_xyz[1], delta_q_xyz[2]);

  InsPose pose;
  pose.time_stamp = current_imu_data.time_stamp;
  pose.accel = current_imu_data.accel;
  pose.gyro = current_imu_data.gyro;

  if (latest_navigation_positions_.size() < order_ + 1)
  {
    // update rotation
    auto last_navigation_info = latest_navigation_positions_.front();
    Eigen::Quaterniond current_quad = last_navigation_info.rotation * delta_quad;
    current_quad.normalized();
    pose.rotation = current_quad;

    auto dt = current_imu_data.time_stamp - last_navigation_info.time_stamp;

    // update velocity_n
    auto last_c_bn = last_navigation_info.rotation.matrix().transpose();
    auto current_c_bn = current_quad.matrix().transpose();
    Eigen::Vector3d vel_n = last_navigation_info.velocity +
                            dt * (0.5 * (last_c_bn.transpose() * last_navigation_info.accel +
                                         current_c_bn.transpose() * current_imu_data_.accel) +
                                  gn_);
    pose.velocity = vel_n;
    pose.velocity_b = current_c_bn * pose.velocity;

    // update position
    Eigen::Vector3d position = last_navigation_info.position + 0.5 * (last_navigation_info.velocity + vel_n) * dt;
    pose.position = position;
  }
  else
  {
    // update rotation
    auto front_navigation_info = latest_navigation_positions_.front();
    latest_navigation_positions_.pop_front();
    auto middile_navigation_info = latest_navigation_positions_.front();
    auto dt = current_imu_data.time_stamp - front_navigation_info.time_stamp;

    Eigen::Quaterniond current_quad = front_navigation_info.rotation * delta_quad;
    current_quad.normalized();
    pose.rotation = current_quad;

    // update velocity_n
    auto front_c_bn = front_navigation_info.rotation.matrix().transpose();
    auto middle_c_bn = middile_navigation_info.rotation.matrix().transpose();
    auto current_c_bn = current_quad.matrix().transpose();
    Eigen::Vector3d vel_n = front_navigation_info.velocity +
                            dt * (1.0 / 3 * (front_c_bn.transpose() * front_navigation_info.accel +
                                             middle_c_bn.transpose() * middile_navigation_info.accel +
                                             current_c_bn.transpose() * current_imu_data_.accel) +
                                  gn_);
    pose.velocity = vel_n;
    pose.velocity_b = current_c_bn * pose.velocity;

    // update position
    Eigen::Vector3d position =
        front_navigation_info.position +
        1.0 / 3 * (front_navigation_info.velocity + middile_navigation_info.velocity + vel_n) * dt;
    pose.position = position;
  }

  latest_navigation_positions_.push_back(pose);
  runCallback(pose);
}

void InsNavigation::runCallback(const InsPose& pose)
{
  for (auto cb : pose_cb_)
  {
    cb(pose);
  }
}
