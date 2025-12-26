#include "LIVMapper.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto mapper = std::make_shared<LIVMapper>();
  mapper->initializeSubscribersAndPublishers();
  mapper->run();
  rclcpp::shutdown();
  return 0;
}