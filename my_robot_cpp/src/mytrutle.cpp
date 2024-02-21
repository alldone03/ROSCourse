#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"
#include <cmath>

class TurtleSimControllerNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<my_robot_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_subscriber_;
    my_robot_interfaces::msg::Turtle::SharedPtr turtle_to_catch_;
    turtlesim::msg::Pose::SharedPtr pose_;
    bool catch_closest_turtle_first_;

public:
    TurtleSimControllerNode() : Node("turtle_controller")
    {
        this->declare_parameter("catch_closest_turtle_first", true);
        this->get_parameter("catch_closest_turtle_first", catch_closest_turtle_first_);

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&TurtleSimControllerNode::callback_turtle_pose, this, std::placeholders::_1));
        alive_turtles_subscriber_ = this->create_subscription<my_robot_interfaces::msg::TurtleArray>("alive_turtles", 10, std::bind(&TurtleSimControllerNode::callback_alives_turtles, this, std::placeholders::_1));
        control_loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TurtleSimControllerNode::control_loop, this));
    }

    void callback_catch_turtle(std::string turtle_name)
    {
        auto client = this->create_client<my_robot_interfaces::srv::CatchTurtle>("catch_turtle");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<my_robot_interfaces::srv::CatchTurtle::Request>();
        request->name = turtle_name;

        auto future = client->async_send_request(request);
        auto status = future.wait_for(std::chrono::seconds(3));

        if (status == std::future_status::ready)
        {
            auto response = future.get();
            if (!response->success)
            {
                RCLCPP_ERROR(this->get_logger(), "Turtle %s could not be caught", turtle_name.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

    void callback_call_catch_turtle(rclcpp::Client<my_robot_interfaces::srv::CatchTurtle>::SharedFuture future, std::string turtle_name)
    {
        try
        {
            auto response = future.get();
            if (!response->success)
            {
                RCLCPP_ERROR(this->get_logger(), "Turtle %s could not be caught", turtle_name.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

    void callback_turtle_pose(turtlesim::msg::Pose::SharedPtr msg)
    {
        pose_ = msg;
    }

    void callback_alives_turtles(my_robot_interfaces::msg::TurtleArray::SharedPtr msg)
    {
        if (msg->turtles.size() > 0)
        {
            my_robot_interfaces::msg::Turtle::SharedPtr closest_turtle = nullptr;
            _Float32 closest_turtle_distance = 0.0;

            if (catch_closest_turtle_first_)
            {
                for (auto turtle : msg->turtles)
                {
                    _Float32 dist_x = turtle.x - pose_->x;
                    _Float32 dist_y = turtle.y - pose_->y;
                    _Float32 distance = sqrt(dist_x * dist_x + dist_y * dist_y);

                    if (!closest_turtle || distance < closest_turtle_distance)
                    {
                        closest_turtle = std::make_shared<my_robot_interfaces::msg::Turtle>(turtle);
                        closest_turtle_distance = distance;
                    }
                }

                turtle_to_catch_ = closest_turtle;
            }
            else
            {
                turtle_to_catch_ = std::make_shared<my_robot_interfaces::msg::Turtle>(msg->turtles[0]);
            }
        }
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
    }

    void control_loop()
    {
        RCLCPP_INFO(get_logger(), "Control loop started");
        // RCLCPP_INFO(get_logger(), "Pose value: x=%f, y=%f, z=%f", pose_->x, pose_->y);

        if (pose_ == nullptr || turtle_to_catch_ == nullptr)
        {
            RCLCPP_INFO(get_logger(), "Pose or turtle_to_catch_ is nullptr, returning");
            return;
        }

        _Float32 dist_x = turtle_to_catch_->x - pose_->x;
        _Float32 dist_y = turtle_to_catch_->y - pose_->y;
        _Float32 distance = sqrt(dist_x * dist_x + dist_y * dist_y);
        auto msg = geometry_msgs::msg::Twist();

        // some stuff

        if (distance > 0.5)
        {
            // Position
            msg.linear.x = 2 * distance;

            // Orientation
            _Float32 goal_theta = atan2(dist_y, dist_x);
            _Float32 diff = goal_theta - pose_->theta;
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
            // target is reached
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            call_catch_turtle_server(turtle_to_catch_->name);
            turtle_to_catch_ = nullptr;
            pose_ = nullptr;
            RCLCPP_INFO(get_logger(), "Target turtle reached, resetting pose_ to nullptr");
        }

        cmd_vel_publisher_->publish(msg);
        RCLCPP_INFO(get_logger(), "Control loop completed");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSimControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}