#include "LIVMapper.h"
#include <iostream>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::Node::SharedPtr nh;
    image_transport::ImageTransport unused(nh);
    LIVMapper mapper(nh, "navigation_lio");
    mapper.initializeSubscribersAndPublishers(nh, unused);
    mapper.run(nh);
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
  } catch (const std::exception& e) {
    const bool interrupted = !rclcpp::ok();
    std::cerr << "Navigation LIO stopped: " << e.what() << std::endl;
    if (rclcpp::ok()) rclcpp::shutdown();
    return interrupted ? 0 : 1;
  }
}
