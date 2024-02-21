#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

#include <cmath>

class RobotControllerNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<my_robot_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_subscriber_;
    my_robot_interfaces::msg::Turtle::SharedPtr turtle_to_catch_;
    turtlesim::msg::Pose::SharedPtr pose_;
    bool catch_closest_turtle_first_ = true;

    void
    callback_turtle_pose(turtlesim::msg::Pose::SharedPtr msg)
    {
        this->pose_ = msg;
    }
    void callback_alives_turtles(my_robot_interfaces::msg::TurtleArray::SharedPtr msg)
    {

        if (msg->turtles.size() > 0)
        {
            my_robot_interfaces::msg::Turtle::SharedPtr closest_turtle = nullptr;
            _Float128 closest_turtle_distance = 0.0;
            if (catch_closest_turtle_first_ == true)
            {
                for (auto turtle : msg->turtles)
                {
                    _Float128 dist_x = turtle.x - this->pose_->x;
                    _Float128 dist_y = turtle.y - this->pose_->y;
                    _Float128 distance = sqrt(dist_x * dist_x + dist_y * dist_y);
                    if (closest_turtle != nullptr || distance < closest_turtle_distance)
                    {
                        closest_turtle = std::make_shared<my_robot_interfaces::msg::Turtle>(turtle);
                        closest_turtle_distance = distance;
                    }
                    this->turtle_to_catch_ = closest_turtle;
                }
            }
            else
            {
                this->turtle_to_catch_ = std::make_shared<my_robot_interfaces::msg::Turtle>(msg->turtles[0]);
            }
        }
    }
    void control_loop()
    {
        if (this->pose_ == nullptr || this->turtle_to_catch_ == nullptr)
        {
            return;
        }
        _Float128 dist_x = this->turtle_to_catch_->x - this->pose_->x;
        _Float128 dist_y = this->turtle_to_catch_->y - this->pose_->y;

        _Float128 distance = sqrt(dist_x * dist_x + dist_y * dist_y);

        auto msg = geometry_msgs::msg::Twist();
        if (distance > 0.5)
        {
            msg.linear.x = 2 * distance;

            _Float128 goal_theta = atan2(dist_y, dist_x);
            _Float128 diff = goal_theta - this->pose_->theta;
            if (diff > M_PI)
            {
                diff -= 2 * M_PI;
            }
            else if (diff < -M_PI)
            {
                diff += 2 * M_PI;
            }
            msg.angular.z = 6 * diff;
        }
        else
        {
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            call_catch_turtle_server(turtle_to_catch_->name);
            this->turtle_to_catch_ = nullptr;
        }

        cmd_vel_publisher_->publish(msg);
    }

    void call_catch_turtle_server(std::string turtle_name)
    {
        auto client = this->create_client<my_robot_interfaces::srv::CatchTurtle>("catch_turtle");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }
        auto request = std::make_shared<my_robot_interfaces::srv::CatchTurtle::Request>();
        request->name = turtle_name;
        auto future = client->async_send_request(request);
        try
        {
            auto response = future.get();
            if (!response->success)
            {

                RCLCPP_ERROR(this->get_logger(), "Turtle %s Could not be caught", turtle_name.c_str());
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        // auto status = future.wait_for(std::chrono::seconds(3));
        // if (status == std::future_status::ready)
        // {
        //     auto response = future.get();
        //     RCLCPP_INFO(this->get_logger(), "Srv response %d %s", response->success, turtle_name.c_str());
        // }
        // else
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        // }
    }

public:
    RobotControllerNode() : Node("turtle_controller")
    {

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        control_loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RobotControllerNode::control_loop, this));
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&RobotControllerNode::callback_turtle_pose, this, std::placeholders::_1));
        alive_turtles_subscriber_ = this->create_subscription<my_robot_interfaces::msg::TurtleArray>("alive_turtles", 10, std::bind(&RobotControllerNode::callback_alives_turtles, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Robot News Station has been started.");
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
