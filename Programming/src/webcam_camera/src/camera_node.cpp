#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <thread>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode()
        : Node("camera_node")
    {
        // Publisher untuk gambar dan koordinat objek
        coord_object_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("coord_object", 10);
        coord_finish_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("coord_finish", 10);

        RCLCPP_INFO(this->get_logger(), "Trying to open camera...");
        cap_.open(0); 
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open camera!");
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Camera opened successfully.");
        }

        capture_once();
    }

private:
    void capture_once()
    {
        RCLCPP_INFO(this->get_logger(), "Entering capture_once...");

        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Received empty frame from camera!");
            return;
        }

        // Deteksi warna hijau
        cv::Point green_target = detect_green(frame);
        if (green_target.x >= 0 && green_target.y >= 0) {
            RCLCPP_INFO(this->get_logger(), "Green target detected at: [%d, %d]", green_target.x, green_target.y);

            // Publikasikan koordinat area hijau
            geometry_msgs::msg::Point green_msg;
            green_msg.x = green_target.x;
            green_msg.y = green_target.y;
            green_msg.z = 0.0;
            coord_finish_publisher_->publish(green_msg);
        }

        // Deteksi warna merah
        cv::Point red_target = detect_red(frame);
        if (red_target.x >= 0 && red_target.y >= 0) {
            RCLCPP_INFO(this->get_logger(), "Red target detected at: [%d, %d]", red_target.x, red_target.y);

            // Publikasikan koordinat area merah
            geometry_msgs::msg::Point red_msg;
            red_msg.x = red_target.x;
            red_msg.y = red_target.y;
            red_msg.z = 0.0;
            coord_object_publisher_->publish(red_msg);
        }

        // Menampilkan gambar di jendela OpenCV
        cv::imshow("Detected Image", frame);
        cv::waitKey(1); // Tunggu sampai user menutup jendela gambar
    }

    cv::Point detect_green(cv::Mat &frame)
    {
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Rentang warna hijau
        cv::Scalar lower_green(35, 100, 100);
        cv::Scalar upper_green(85, 255, 255);
        cv::inRange(hsv, lower_green, upper_green, mask);

        return find_largest_contour_center(frame, mask);
    }

    cv::Point detect_red(cv::Mat &frame)
    {
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Rentang warna merah
        cv::Scalar lower_red_1(0, 120, 70); // Rentang merah pertama
        cv::Scalar upper_red_1(10, 255, 255);
        cv::Scalar lower_red_2(170, 120, 70); // Rentang merah kedua
        cv::Scalar upper_red_2(180, 255, 255);

        cv::Mat mask1, mask2;
        cv::inRange(hsv, lower_red_1, upper_red_1, mask1);
        cv::inRange(hsv, lower_red_2, upper_red_2, mask2);

        mask = mask1 | mask2;

        return find_largest_contour_center(frame, mask);
    }

    cv::Point find_largest_contour_center(cv::Mat &frame, const cv::Mat &mask)
    {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Point target(-1, -1);
        double max_area = 0;

        for (const auto &contour : contours) {
            double area = cv::contourArea(contour);
            if (area > max_area) {
                max_area = area;
                cv::Moments m = cv::moments(contour);
                target = cv::Point(m.m10 / m.m00, m.m01 / m.m00);
            }
        }

        cv::drawContours(frame, contours, -1, cv::Scalar(0, 255, 0), 2);
        if (target.x >= 0 && target.y >= 0) {
            cv::circle(frame, target, 5, cv::Scalar(0, 0, 255), -1);
        }

        return target;
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr coord_object_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr coord_finish_publisher_;
    cv::VideoCapture cap_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
