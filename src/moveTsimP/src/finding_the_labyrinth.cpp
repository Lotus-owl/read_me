#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"


#include <chrono>
#include <iostream>

using namespace std;
using namespace std::chrono_literals;

class LaserSensor : public rclcpp::Node
{
public:
    LaserSensor()
        : Node("laser_sensorr")
    {
        auto qos_profile = rclcpp::SensorDataQoS();
        auto qos_profile2 = rclcpp::QoS(rclcpp::KeepLast(10));
        _sub = create_subscription<sensor_msgs::msg::LaserScan>("/scan", qos_profile, std::bind(&LaserSensor::sub_laser, this, std::placeholders::_1));
        _twist_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos_profile2);
        _pub_timer = create_wall_timer(30ms, std::bind(&LaserSensor::pub_twist, this));
        _update_timer = create_wall_timer(10ms, std::bind(&LaserSensor::update, this));

        _sub2 = create_subscription<sensor_msgs::msg::CompressedImage>("/image_raw/compressed", qos_profile2, std::bind(&LaserSensor::sub_img, this, std::placeholders::_1));
    }

private:
    bool _is_wall;
    float _laser_data[360];
    sensor_msgs::msg::LaserScan _laser_data2;
    geometry_msgs::msg::Twist _twist;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _twist_pub;
    rclcpp::TimerBase::SharedPtr _pub_timer;
    rclcpp::TimerBase::SharedPtr _update_timer;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _sub;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr _sub2;
    void sub_img(const sensor_msgs::msg::CompressedImage msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr->image;
        //cv::Mat canny;
        //cv::Canny(img, canny, 100, 200);
        cv::imshow("img", img);
        cv::waitKey(30);
    }

    void pub_twist()
    {
        //restrain();
        _twist_pub->publish(_twist);
    }

    void sub_laser(const sensor_msgs::msg::LaserScan msg)
    {
        _laser_data2 = msg;
        // 내부변수 업데이트.
        for (long unsigned int i=0; i < _laser_data2.ranges.size();i++)
        {
            auto degree = (_laser_data2.angle_min+ _laser_data2.angle_increment * i)*180/3.1415926535;
            auto int_degree = (int)degree;
            // RCLCPP_INFO(this->get_logger(), "0:: %d", int_degree);

            _laser_data[int_degree] = _laser_data2.ranges[i];
        }
    }

    void update()
    {

        if (_laser_data2.ranges.size() == 0)
        {
            RCLCPP_INFO(this->get_logger(), "NO SIZE");
            return;
        }

        if (_is_wall == false){
            _twist.linear.x = 0.2;
            if (_laser_data[0] < 0.3)
            {
                _is_wall = true;
                RCLCPP_INFO(this->get_logger(), "wa");
            }
        }else{
            // 로봇이 전방이나 양 옆에서 벽을 감지할 때
            if (_laser_data[0] < 0.3 || _laser_data[45] < 0.3 || _laser_data[315] < 0.3)
            {
                RCLCPP_INFO(this->get_logger(), "오른쪽 회전:: ");
                RCLCPP_INFO(this->get_logger(), "0:: %f", _laser_data[0]);
                RCLCPP_INFO(this->get_logger(), "45:: %f", _laser_data[45]);
                RCLCPP_INFO(this->get_logger(), "315:: %f", _laser_data[315]);
                // 정지 후 오른쪽으로 회전
                _twist.linear.x = 0.0;
                _twist.angular.z = -0.2;
            }
            else if (_laser_data[90] > 0.3) // 오른쪽 벽과 거리가 충분히 멀다면
            {
                RCLCPP_INFO(this->get_logger(), "직진 왼쪽 회전:: ");
                RCLCPP_INFO(this->get_logger(), "0:: %f", _laser_data[90]);
                // 약간 왼쪽으로 돌면서 전진
                _twist.linear.x = 0.05;
                _twist.angular.z = 0.1;
            }
            else if (_laser_data[90] < 0.2) // 오른쪽 벽과 너무 가깝다면
            {
                RCLCPP_INFO(this->get_logger(), "직진 오른쪽 회전:: ");
                RCLCPP_INFO(this->get_logger(), "0:: %f", _laser_data[90]);
                // 약간 오른쪽으로 돌면서 전진
                _twist.linear.x = 0.05;
                _twist.angular.z = -0.1;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "직진 :: ");
                RCLCPP_INFO(this->get_logger(), "0:: %f", _laser_data[90]);
                // 벽을 따라 직진
                _twist.linear.x = 0.1;
                _twist.angular.z = 0.0;
            }
        }
    }

    void restrain()
    {
        if (_twist.angular.z > 1.84)
            _twist.angular.z = 1.84;
        else if (_twist.angular.z < -1.84)
            _twist.angular.z = -1.84;
        if (_twist.linear.x > 0.20)
            _twist.linear.x = 0.20;
        else if (_twist.linear.x < -0.20)
            _twist.linear.x = -0.20;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserSensor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}