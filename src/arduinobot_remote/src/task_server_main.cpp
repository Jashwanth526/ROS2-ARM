#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <class_loader/class_loader.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Load the TaskServer component
  class_loader::ClassLoader loader("libtask_server.so");
  auto node_factory = loader.createInstance<rclcpp_components::NodeFactory>(
    "rclcpp_components::NodeFactoryTemplate<arduinobot_remote::TaskServer>");
  
  rclcpp::NodeOptions options;
  auto node_wrapper = node_factory->create_node_instance(options);
  auto node = node_wrapper.get_node_base_interface();
  
  // Use MultiThreadedExecutor with 2 threads to process callbacks concurrently  
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  
  RCLCPP_INFO(rclcpp::get_logger("task_server_main"), 
              "Task Server starting with MultiThreadedExecutor (2 threads)");
  
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
