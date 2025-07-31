

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class Safety : public rclcpp::Node
{
    public:
        Safety()
        : Node("safety") 
        {
            scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Safety::set_scan, this, _1));
            odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, std::bind(&Safety::set_odom, this, _1));

            drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&Safety::calculate_ttc, this)
            );
        }
    
    private:
        void set_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            last_scan_ = *msg;
        }

        void set_odom(const nav_msgs::msg::Odometry::SharedPtr msg){
            odom_ = *msg;
        }

        void emergency_brake(){
            auto msg = ackermann_msgs::msg::AckermannDriveStamped();
            msg.header.stamp = this-> now();
            msg.header.frame_id = "base_link";

            msg.drive.speed = 0;

            drive_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Braking!!!");
        }

        void calculate_ttc(){
            RCLCPP_INFO(this->get_logger(), "%d", last_scan_.ranges.size());
            for(int i = 0; i < last_scan_.ranges.size(); i++){
                double angle = last_scan_.angle_min + i*last_scan_.angle_increment;
                double vel = odom_.twist.twist.linear.x*cos(angle);
                if (vel < .35) continue;
                double ttc = last_scan_.ranges[i]/std::max((double)0, vel);
                // RCLCPP_INFO(this->get_logger(), "%lf -> %lf", vel, ttc);
                if(ttc <= 1.2){
                    emergency_brake();
                }
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        sensor_msgs::msg::LaserScan last_scan_;
        nav_msgs::msg::Odometry odom_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();

    return 0;
}