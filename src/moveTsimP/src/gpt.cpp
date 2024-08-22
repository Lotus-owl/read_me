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
        cv::imshow("img", img);
        cv::waitKey(30);
    }

    // 새로운 보정된 각도를 위한 변수
    int angle_offset = 0; 

    void pub_twist()
    {
        //restrain();
        _twist_pub->publish(_twist);
    }

    void sub_laser(const sensor_msgs::msg::LaserScan msg)
    {
        _laser_data2 = msg;
        // 내부변수 업데이트.
        
        _laser_data2.ranges.size();


    // sensor_msgs::LaserScan scan;
    // scan.header.stamp = scan_time;
    // scan.header.frame_id = "laser_frame";
    // scan.angle_min = -1.57;
    // scan.angle_max = 1.57;
    // scan.angle_increment = 3.14 / num_readings;
    // scan.time_increment = (1 / laser_frequency) / (num_readings);
    // scan.range_min = 0.0;
    // scan.range_max = 100.0;
    //for (long unsigned int i=0; i < msg.ranges.size();i++){
        RCLCPP_INFO(this->get_logger(), "msg.angle_increment:: %f\n", msg.angle_increment);
RCLCPP_INFO(this->get_logger(), "min:: %f\n", msg.angle_min);
RCLCPP_INFO(this->get_logger(), "max:: %f\n", msg.angle_max);
    //}

        RCLCPP_INFO(this->get_logger(), "_laser_data2.ranges.size():: %lu", _laser_data2.ranges.size());
        for (long unsigned int i=0; i < _laser_data2.ranges.size();i++)
        {
            if(_laser_data2.ranges[i] == 0.0f){
                continue;
            }
            if(std::isnan(_laser_data2.ranges[i])){
                _laser_data2.ranges[i] = 3.5; 
            }
            auto degree = (_laser_data2.angle_min+ _laser_data2.angle_increment * i)*180/3.1415926535;
            auto int_degree = (int)degree;

            RCLCPP_INFO(this->get_logger(), "degree:: %d\n", int_degree);

           // int corrected_degree = (int_degree + angle_offset + 360) % 360;  // 각도를 보정하고 0~359도로 제한

            //RCLCPP_INFO(this->get_logger(), "corrected_degree:: %d\n",sb corrected_degree);

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
            _twist.linear.x = 0.1;
            _twist.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "false로 돌아옴");
            for (int i = 355; i <= 364; ++i) {
                int value = (i > 359) ? (i - 359) : i;
                if (_laser_data[value] < 0.2 && _laser_data[value] > 0.0f)
                {
                    _is_wall = true;
                    RCLCPP_INFO(this->get_logger(), "wa");
                    RCLCPP_INFO(this->get_logger(), "0:: %d",value);
                    RCLCPP_INFO(this->get_logger(), "0:: %f", _laser_data[value]);
                    break;
                }
            }
        }else{
            for (int i = 355; i <= 364; ++i) {
                int value = (i > 359) ? (i - 359) : i;
                if (_laser_data[value] < 0.2 && _laser_data[value] > 0.0f)
                {
                    _twist.linear.x = 0.0;
                    _twist.angular.z = 0.0;
                    // || _laser_data[315] < 0.3
                    RCLCPP_INFO(this->get_logger(), "오른쪽 회전:: ");
                    RCLCPP_INFO(this->get_logger(), "0:: %f", _laser_data[0]);
                    RCLCPP_INFO(this->get_logger(), "45:: %f", _laser_data[45]);
                    RCLCPP_INFO(this->get_logger(), "315:: %f", _laser_data[315]);

                    rotate_90_degrees();  // 벽을 만나면 90도 회전
                    break;
                }
                std::cout << value << std::endl;
            }
        }
    }

    void rotate_90_degrees()
    {
        RCLCPP_INFO(this->get_logger(), "90도 로직");
        // 90도 회전 각속도 설정
        double angular_speed = -0.2; // 각속도 (rad/s)
        double target_angle = -M_PI / 2; // 목표 회전 각도 (90도 = π/2 rad)
        double current_angle = 0.0;

        // 시간 간격 설정
        rclcpp::Rate loop_rate(100); // 100Hz 주기
        auto start_time = this->now();
        RCLCPP_INFO(this->get_logger(), "회전 : %f", current_angle);
        RCLCPP_INFO(this->get_logger(), "회전 : %f", current_angle);
        // 회전 시작
        while (rclcpp::ok() && current_angle > target_angle)
        {
            _twist.linear.x = 0.0;
            _twist.angular.z = angular_speed;

            _twist_pub->publish(_twist);

            // 시간 경과 계산
            auto current_time = this->now();
            auto elapsed_time = (current_time - start_time).seconds();
            
            // 현재 회전한 각도 계산
            current_angle = angular_speed * elapsed_time;
            RCLCPP_INFO(this->get_logger(), "회전 : %f", current_angle);

            loop_rate.sleep();
        }



        // 회전 완료 후 정지
        _twist.angular.z = 0.0;
        _is_wall = false;
        
        _twist_pub->publish(_twist);
        RCLCPP_INFO(this->get_logger(), "회전완료");

        // 정면을 0도로 보정
        angle_offset = (angle_offset + 90) % 360;
        RCLCPP_INFO(this->get_logger(), "angle_offset : %d", angle_offset);
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