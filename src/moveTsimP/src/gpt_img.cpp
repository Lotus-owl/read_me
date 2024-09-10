#include "rclcpp/rclcpp.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <chrono>
#include <iostream>
#include <filesystem>

using namespace std;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

class LaserSensor : public rclcpp::Node
{
public:
    LaserSensor()
        : Node("laser_sensorr")
    {
        auto qos_profile2 = rclcpp::QoS(rclcpp::KeepLast(10));
        _sub2 = create_subscription<sensor_msgs::msg::CompressedImage>("/image_raw/compressed", qos_profile2, std::bind(&LaserSensor::sub_img, this, std::placeholders::_1));
    
        // 15초마다 이미지를 저장하는 타이머 추가
        _image_save_timer = create_wall_timer(15s, std::bind(&LaserSensor::save_image, this));
    
        // 이미지 저장 디렉토리 확인 및 생성
        save_directory = "/home/leejieun/namee";
        if (!fs::exists(save_directory)) {
            fs::create_directories(save_directory);
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr _sub2;
    rclcpp::TimerBase::SharedPtr _image_save_timer; // 이미지 저장 타이머

    cv::Mat _current_image; // 현재 이미지 저장
    std::string save_directory; // 이미지 저장 디렉토리

    void sub_img(const sensor_msgs::msg::CompressedImage msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        _current_image = cv_ptr->image; // 현재 이미지를 저장
        cv::imshow("img",  _current_image);
        cv::waitKey(30);
    }

    void save_image() 
    {
        if (!_current_image.empty()) 
        {
            static int image_count = 0; // 이미지 카운트
            std::string filename = save_directory + "img" + std::to_string(image_count++) + ".jpg";
            cv::imwrite(filename, _current_image); // 이미지 저장
            RCLCPP_INFO(this->get_logger(), "Saved image: %s", filename.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "No image to save.");
        }
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