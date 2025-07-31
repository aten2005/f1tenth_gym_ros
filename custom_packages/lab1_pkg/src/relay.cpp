#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;

class Relay : public rclcpp::Node
{
    public:
        Relay()
        : Node("relay")
        {
            subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10, std::bind(&Relay::drive_callback, this, _1));
            publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10);
        }

    private:

        void drive_callback(ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) const {
            msg->header.stamp = this->now();
            msg->drive.speed *= 3;
            msg->drive.steering_angle *= 3;

            publisher_->publish(*msg);
            RCLCPP_INFO(this->get_logger(), "Relayed a message!");

        }

        rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Relay>());
    rclcpp::shutdown();
    return 0;
}