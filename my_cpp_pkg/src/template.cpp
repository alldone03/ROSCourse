#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class RobotNewsStationNode : public rclcpp::Node
{
public:
    RobotNewsStationNode() : Node("cpp_test")
    {
    }

private:
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsStationNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
