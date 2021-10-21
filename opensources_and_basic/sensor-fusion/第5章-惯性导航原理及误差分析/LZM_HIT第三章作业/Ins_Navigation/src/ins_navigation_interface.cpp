/*
 * @Description: INS 接口
 * @Author: Zm Liu
 * @Date: 
 */
#include "ins_navigation_interface.h"

template <typename T>
void yamlRead(const YAML::Node& node, const std::string& name, T& value, const T& defaultValue)
{
  if (node[name])
  {
    value = node[name].as<T>();
  }
  else
  {
    value = defaultValue;
  }
}

InsNavigationInterface::InsNavigationInterface(const YAML::Node& node)
{
  ins_navigation_.reset(new InsNavigation(1));
  ins_navigation_->registerPoseCallback(std::bind(&InsNavigationInterface::poseCallback, this, std::placeholders::_1));
  initialize(node);
}

void InsNavigationInterface::initialize(const YAML::Node& node)
{
  initialized_ = false;
  int mode;
  yamlRead<int>(node, "solution_mode", mode, 0);
  switch (mode)
  {
    case 0:
      solution_mode_ = SolutionMode::Offline;
      break;
    case 1:
      solution_mode_ = SolutionMode::Online;
      break;
    default:
      solution_mode_ = SolutionMode::Offline;
      break;
  }

  if (solution_mode_ == SolutionMode::Offline)
  {
    std::string folder;
    yamlRead<std::string>(node, "data_folder", folder, "data");
    folder = std::string(PROJECT_PATH) + "/data/" + folder;
    result_folder_ = folder + "/result";

    if (!boost::filesystem::exists(folder) || !readFromFile(folder))
      return;
    InitState state;
    state.lla = Eigen::Vector3d(init_state_[0] * DEG2RAD, init_state_[1] * DEG2RAD, init_state_[2]);
    state.vel_b = Eigen::Vector3d(init_state_[3], init_state_[4], init_state_[5]);
    state.ypr = Eigen::Vector3d(init_state_[6], init_state_[7], init_state_[8]) * DEG2RAD;
    ins_navigation_->setInitialState(state);
  }
  else
  {
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/ins_odom", 10);
    ins_navigation_->setInitialPose(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu_raw", 100, &InsNavigationInterface::imuCallback, this);
  }

  initialized_ = true;

  yamlRead<bool>(node, "use_fixed_model", use_fixed_angle_axis_, false);
}

bool InsNavigationInterface::readFromFile(const std::string& folder)
{
  std::cout << "folder is: " << folder << std::endl;
  std::string acc_file = folder + "/accel-0.csv";
  std::string gyro_file = folder + "/gyro-0.csv";
  std::string time_file = folder + "/time.csv";
  std::string init_state_file = folder + "/init_state.csv";

  std::string ref_pos_file = folder + "/ref_pos.csv";
  std::string ref_quat_file = folder + "/ref_att_quat.csv";

  if (!boost::filesystem::exists(acc_file) || !boost::filesystem::exists(gyro_file) ||
      !boost::filesystem::exists(time_file) || !boost::filesystem::exists(init_state_file) ||
      !boost::filesystem::exists(ref_pos_file) || !boost::filesystem::exists(ref_quat_file))
  {
    std::cout << "some file is missing! please check the following file exist in folder: " << std::endl;
    std::cout << " acc_file: " << acc_file << std::endl;
    std::cout << " gyro_file: " << gyro_file << std::endl;
    std::cout << " time_file: " << time_file << std::endl;
    std::cout << " ref_pos_file: " << ref_pos_file << std::endl;
    std::cout << " ref_quat_file: " << ref_quat_file << std::endl;
    std::cout << " init_state_file: " << init_state_file << std::endl;
    return false;
  }
  std::vector<std::vector<double>> time_data;
  importCsvData(time_file, time_data);
  std::vector<std::vector<double>> acc_data;
  importCsvData(acc_file, acc_data);
  std::vector<std::vector<double>> gyro_data;
  importCsvData(gyro_file, gyro_data);
  std::vector<std::vector<double>> ref_pos;
  importCsvData(ref_pos_file, ref_pos);
  std::vector<std::vector<double>> ref_quat;
  importCsvData(ref_quat_file, ref_quat);

  for (int i = 0; i < time_data.size(); i++)
  {
    ImuData data(time_data[i][0], Eigen::Vector3d(acc_data[i][0], acc_data[i][1], acc_data[i][2]),
                 Eigen::Vector3d(gyro_data[i][0], gyro_data[i][1], gyro_data[i][2]) * DEG2RAD);
    // std::cout << i << " acc is: " << data.accel.transpose() << " gyro is: " << data.gyro.transpose() << std::endl;
    imu_datas_.emplace_back(data);
    ref_poses_.emplace_back(ResultPose(time_data[i][0], ref_pos[i][0], ref_pos[i][1], ref_pos[i][2], ref_quat[i][1],
                                       ref_quat[i][2], ref_quat[i][3], ref_quat[i][0]));
  }

  std::string line;
  std::ifstream infile;
  infile.open(init_state_file);
  for (int i = 0; i < 2; i++)
  {
    getline(infile, line);
    if (i == 1)
    {
      std::string data_str;
      std::istringstream sin(line);        //将整行字符串line读入到字符串流istringstream中
      while (getline(sin, data_str, ','))  //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
      {
        init_state_.push_back(atof(data_str.c_str()));
      }
    }
  }
  std::cout << "init_state_ is: " << std::endl;
  for (int i = 0; i < init_state_.size(); i++)
  {
    std::cout << init_state_[i] << " ";
  }
  std::cout << std::endl;
}

void InsNavigationInterface::importCsvData(const std::string& file_name, std::vector<std::vector<double>>& samples)
{
  samples.clear();

  std::string line;
  std::ifstream infile;

  infile.open(file_name);
  if (infile.is_open())
  {
    getline(infile, line);
    int l = 0;
    while (getline(infile, line))
    {
      std::istringstream sin(line);  //将整行字符串line读入到字符串流istringstream中
      std::vector<double> line_datas;

      std::string data_str;
      while (getline(sin, data_str, ','))  //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
      {
        line_datas.push_back(atof(data_str.c_str()));
      }
      samples.push_back(line_datas);
      l++;
    }
    infile.close();
  }
}

void InsNavigationInterface::run()
{
  if (!initialized_)
  {
    std::cout << "Initialize Error" << std::endl;
    return;
  }

  if (solution_mode_ == SolutionMode::Online)
  {
    ros::spin();
  }
  else
  {
    int i = 0;
    while (i < imu_datas_.size() && ros::ok())
    {
      auto imu_dat = imu_datas_[i];
      ins_navigation_->solution(imu_dat.time_stamp, imu_dat.accel, imu_dat.gyro, use_fixed_angle_axis_);
      usleep(1000);
      i++;
    }
  }
}

void InsNavigationInterface::saveResult()
{
  if (!boost::filesystem::exists(result_folder_))
  {
    boost::filesystem::create_directories(result_folder_);
  }

  std::cout << "saving ref_poses " << std::endl;
  std::string ref_file = result_folder_ + "/ref_poses.txt";
  std::ofstream f_file;
  f_file.open(ref_file);
  for (int i = 0; i < ref_poses_.size(); i++)
  {
    auto pose_data = ref_poses_[i];
    f_file << std::setprecision(15) << pose_data.time_stamp << " " << pose_data.x - ref_poses_[0].x << " "
           << pose_data.y - ref_poses_[0].y << " " << pose_data.z - ref_poses_[0].z << " " << pose_data.qx << " "
           << pose_data.qy << " " << pose_data.qz << " " << pose_data.qw << std::endl;
  }
  f_file.close();
  std::cout << "saving ref_poses ok!" << std::endl;

  std::cout << "saving ins_poses" << std::endl;
  std::string ins_file;
  if (use_fixed_angle_axis_)
    ins_file = result_folder_ + "/ins_poses_fixed_angle.txt";
  else
  {
    ins_file = result_folder_ + "/ins_poses.txt";
  }

  f_file.open(ins_file);
  for (int i = 0; i < ins_poses_.size(); i++)
  {
    auto pose_data = ins_poses_[i];
    f_file << std::setprecision(15) << pose_data.time_stamp << " " << pose_data.x - ins_poses_[0].x << " "
           << pose_data.y - ins_poses_[0].y << " " << pose_data.z - ins_poses_[0].z << " " << pose_data.qx << " "
           << pose_data.qy << " " << pose_data.qz << " " << pose_data.qw << std::endl;
  }
  f_file.close();
  std::cout << "saving ins_poses ok!" << std::endl;
}

void InsNavigationInterface::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  static double init_time = imu_msg->header.stamp.toSec();

  Eigen::Vector3d acc =
      Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
  Eigen::Vector3d gyro =
      Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);

  ins_navigation_->solution(imu_msg->header.stamp.toSec() - init_time, acc, gyro);
}

void InsNavigationInterface::poseCallback(const InsPose& pose)
{
  ins_poses_.emplace_back(ResultPose(pose.time_stamp, pose.position[0], pose.position[1], pose.position[2],
                                     pose.rotation.x(), pose.rotation.y(), pose.rotation.z(), pose.rotation.w()));
  std::cout << "====== position is: " << std::setprecision(15) << pose.position[0] << " " << pose.position[1] << " "
            << pose.position[2] << std::endl;
  if (solution_mode_ == SolutionMode::Online)
  {
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.stamp.fromSec(pose.time_stamp);
    odometry_msg.header.frame_id = "/map";
    odometry_msg.child_frame_id = "/imu_link";
    odometry_msg.pose.pose.position.x = pose.position[0];
    odometry_msg.pose.pose.position.y = pose.position[1];
    odometry_msg.pose.pose.position.z = pose.position[2];
    odometry_msg.pose.pose.orientation.x = pose.rotation.x();
    odometry_msg.pose.pose.orientation.y = pose.rotation.y();
    odometry_msg.pose.pose.orientation.z = pose.rotation.z();
    odometry_msg.pose.pose.orientation.w = pose.rotation.w();
    odom_pub_.publish(odometry_msg);
  }
}
