#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class Talker : public rclcpp::Node
{
    public:
        Talker()
        : Node("talker")
        {
            this->declare_parameter<int>("v", 0);
            this->declare_parameter<int>("d", 0);

            this->get_parameter("v", v_);
            this->get_parameter("d", d_);

            RCLCPP_INFO(this->get_logger(), "v: %d, d: %d", v_, d_);

            publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&Talker::publish, this)
            );
        }
    private:

        void publish(){
            auto msg = ackermann_msgs::msg::AckermannDriveStamped();
            msg.header.stamp = this->now();
            msg.header.frame_id = "base_link";

            msg.drive.speed = v_;
            msg.drive.acceleration = 0.0;
            msg.drive.jerk = 0.0;
            msg.drive.steering_angle = d_;
            msg.drive.steering_angle_velocity = 0.0;

            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published AckermannDriveStamped");
        }
        
        int v_, d_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();
    return 0;
}