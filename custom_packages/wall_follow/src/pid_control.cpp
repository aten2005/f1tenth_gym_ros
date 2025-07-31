#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

class PidControl : public rclcpp::Node
{
public:
    PidControl()
    : Node("pid_control")
    {
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&PidControl::set_scan, this, _1));
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, std::bind(&PidControl::set_odom, this, _1));

        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PidControl::drive, this)
        );
    }

private:
    const float THETA = 45*3.14/180; //Offset angle (0-70);
    const float L = 1.2; //Look ahead
    const float hold_distance = 1.2;
    const float Kp = .8, Ki = 0, Kd = .4;

    void set_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan_ = *msg;
    }
    void set_odom(const nav_msgs::msg::Odometry::SharedPtr msg){
        odom_ = *msg;
    }

    void calculate_wall_distance(){
        int index = last_scan_.ranges.size()/2 + (3.14/2 - THETA)/last_scan_.angle_increment;
        float offset_distance = last_scan_.ranges[index];
        float perp_distance = last_scan_.ranges[last_scan_.ranges.size()/2 + (int)((3.14/2)/last_scan_.angle_increment)];
        float alpha = atan((offset_distance*cos(THETA) - perp_distance)/(offset_distance*sin(THETA)));
        float wall_distance = perp_distance*cos(alpha);
        prev_error_= cur_error_;
        est_wall_distance_ = wall_distance + L*sin(alpha);
        cur_error_ = hold_distance - est_wall_distance_;
        cum_error_ += cur_error_;
    }

    void calculate_steering(){
        steering_angle_ = Kp*(cur_error_) + Kd*(cur_error_ - prev_error_) + Ki*cum_error_ ;
        steering_angle_ = std::min(steering_angle_, (float)(40*3.14/180));
        steering_angle_ = std::max(steering_angle_, (float)(-40*3.14/180));
        steering_angle_ *= -1;
    }

    void calculate_speed(){
        if(abs(steering_angle_)*180/3.14 < 15) {
            drive_speed_ = 1.5;
        } else if(abs(steering_angle_)*180/3.14 < 30) {
            drive_speed_ = 1.0;
        } else {
            drive_speed_ = 0.5;
        }
    }

    void drive(){
        calculate_wall_distance();
        calculate_steering();
        calculate_speed();

        RCLCPP_INFO(this->get_logger(), "%f %f %f", cur_error_, cum_error_, steering_angle_);


        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        msg.header.stamp = this-> now();
        msg.header.frame_id = "base_link";

        msg.drive.speed = drive_speed_;
        msg.drive.steering_angle = steering_angle_;
        drive_publisher_->publish(msg);

    }




    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan last_scan_;
    float est_wall_distance_;
    float prev_error_, cur_error_, cum_error_ = 0;
    float steering_angle_;
    float drive_speed_;

    nav_msgs::msg::Odometry odom_;
};

int main(int argc, char ** argv)
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PidControl>());
    rclcpp::shutdown();
    return 0;
}
