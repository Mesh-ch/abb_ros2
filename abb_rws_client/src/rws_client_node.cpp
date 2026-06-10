#include <rclcpp/rclcpp.hpp>

#include <abb_rws_client/rws_service_provider_ros.hpp>
#include <abb_rws_client/rws_state_publisher_ros.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr client_node = rclcpp::Node::make_shared("rws");

  client_node->declare_parameter("robot_nickname", std::string{});
  client_node->declare_parameter("no_connection_timeout", false);
  client_node->declare_parameter("rws_version", std::string{"rws1"});
  std::string robot_ip = client_node->declare_parameter<std::string>("robot_ip", "127.0.0.1");
  int robot_port = client_node->declare_parameter<int>("robot_port", 65535);

  client_node->get_parameter<std::string>("robot_ip", robot_ip);
  client_node->get_parameter<int>("robot_port", robot_port);

  std::string rws_version = client_node->get_parameter("rws_version").as_string();
  RCLCPP_INFO_STREAM(client_node->get_logger(), "Requested RWS protocol version: " << rws_version);
  if (rws_version != "rws1")
  {
    RCLCPP_WARN_STREAM(client_node->get_logger(),
                       "RWS2 backend support is not implemented yet in abb_rws_client; this parameter is a seam");
  }

  abb_rws_client::RWSServiceProviderROS srv_provider(client_node, robot_ip, robot_port, rws_version);
  abb_rws_client::RWSStatePublisherROS state_publisher(client_node, robot_ip, robot_port, rws_version);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(client_node);

  exec.spin();

  return 0;
}