/*
 * @Description: INS 测试例子
 * @Author: Zm Liu
 * @Date: 
 */
#include "ins_navigation_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ins_navigation");
  std::string config_file = std::string(PROJECT_PATH) + "/config/config.yaml";

  YAML::Node config = YAML::LoadFile(config_file);
  InsNavigationInterface ins_interface(config);
  ins_interface.run();
  ins_interface.saveResult();
  return 0;
}
