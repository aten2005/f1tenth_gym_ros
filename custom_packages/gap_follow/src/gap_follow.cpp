#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <algorithm>

using std::placeholders::_1;

class GapFollow : public rclcpp::Node {
    public:
        GapFollow()
        : Node("gap_follow"){
            scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&GapFollow::set_scan, this, _1));
    
            drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

            //DEBUG
            marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/marker", 10);
    
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&GapFollow::drive, this)
            );
        }
    
        private:

            const float bot_radius = 0.4;
            const float pad_radius = 0.5;
            const float disparity_thresh = 1;

            void set_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                last_scan_ = *msg;
            }

            void find_gap(){
                std::vector<float> gaps = last_scan_.ranges;

                //Find disparities
                for(int i = 1; i < (int)gaps.size()-1; i++){
                    if(abs(gaps[i]-gaps[i+1]) > disparity_thresh || abs(gaps[i] - gaps[i-1]) > disparity_thresh){
                        float dist = std::min(std::min(gaps[i], gaps[i-1]), gaps[i+1]);
                        int ext_range = (pad_radius/(dist))/last_scan_.angle_increment;
                        int st_index = std::max(0, i-ext_range);
                        int end_index = std::min((int)gaps.size(), i+ext_range);
                        std::fill(gaps.begin()+st_index, gaps.begin()+end_index, dist);
                        i = end_index + 1;
                    }
                }

                std::vector<float>::iterator closest_ray = std::min_element(gaps.begin(), gaps.end());
                int bubble_range = (bot_radius/(*closest_ray))/last_scan_.angle_increment;

                //DEBUG
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = last_scan_.header.frame_id;
                marker.header.stamp = this->now();
                marker.ns = "fan_sector";
                marker.id = 0;
                marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.scale.x = 1.0; // Scale must be 1.0 for TRIANGLE_LIST
                marker.scale.y = 1.0;
                marker.scale.z = 1.0;
                marker.color.r = 1.0f;
                marker.color.g = 0.5f;
                marker.color.b = 0.0f;
                marker.color.a = 0.5f; 

                geometry_msgs::msg::Point center;
                center.x = 0.0;
                center.y = 0.0;
                center.z = 0.0;

                // Find points along the arc
                std::vector<geometry_msgs::msg::Point> arc_points;

                for(auto it = (closest_ray-bubble_range); it != closest_ray+bubble_range; it++){
                    geometry_msgs::msg::Point p;
                    float angle = last_scan_.angle_min + (it - gaps.begin())*last_scan_.angle_increment;
                    if(std::isfinite(*it)){
                        p.x = (*it)*cos(angle);
                        p.y = (*it)*sin(angle);
                        p.z = 0.0;
                        arc_points.push_back(p);
                    }

                }

                for(auto it = arc_points.begin(); it != arc_points.end()-1; it++){
                    marker.points.push_back(center);
                    marker.points.push_back(*it);
                    marker.points.push_back(*(it+1));
                }

                marker_publisher_->publish(marker);

                int closest_index = closest_ray - gaps.begin();
                int rstart_index = std::max(0, closest_index-bubble_range);
                int rend_index = std::min((int)gaps.size(), closest_index+bubble_range);
                
                std::fill(gaps.begin() + rstart_index, gaps.begin()+rend_index, 0);

                size_t max_start = 0, max_len = 0;
                size_t curr_start = 0, curr_len = 0;
            
                for (size_t i = 0; i < gaps.size(); ++i) {
                    if (gaps[i] != 0.0f) {
                        if (curr_len == 0) {
                            curr_start = i;
                        }
                        ++curr_len;
            
                        if (curr_len > max_len) {
                            max_len = curr_len;
                            max_start = curr_start;
                        }
                    } else {
                        curr_len = 0;
                    }
                }

                std::vector<float>::iterator gs = gaps.begin() + max_start;
                int ml = max_len;

                std::vector<float>::iterator farthest_ray = std::max_element(gs, gs+ml);
                steering_angle_ = last_scan_.angle_min + (farthest_ray - gaps.begin())*last_scan_.angle_increment;

                RCLCPP_INFO(this->get_logger(), "%f %f", steering_angle_, *farthest_ray);

                if(abs(*farthest_ray) < .5) {
                    drive_speed_ = .5;
                } else if(*farthest_ray < 5) {
                    drive_speed_ = 2*(*farthest_ray)/4;
                } else {
                    drive_speed_ = 3;
                }

                if(abs(steering_angle_) > 3.14/6){
                    drive_speed_*= .25;
                }
            }

            void drive(){
                find_gap();

                auto msg = ackermann_msgs::msg::AckermannDriveStamped();
                msg.header.stamp = this-> now();
                msg.header.frame_id = "base_link";

                RCLCPP_INFO(this->get_logger(), "%f", drive_speed_);
        
                msg.drive.speed = drive_speed_;
                msg.drive.steering_angle = steering_angle_;
                drive_publisher_->publish(msg);
            }            


            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
            rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
            rclcpp::TimerBase::SharedPtr timer_;
            sensor_msgs::msg::LaserScan last_scan_;
            float steering_angle_;
            float drive_speed_;

};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapFollow>());
    rclcpp::shutdown();
}