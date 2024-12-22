#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <serial/serial.h>  
#include <cmath>
#include <string>

class StepperNode : public rclcpp::Node
{
public:
    StepperNode()
        : Node("stepper_node")
    {
        object_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "coord_object", 10, std::bind(&StepperNode::object_callback, this, std::placeholders::_1));

        green_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "coord_finish", 10, std::bind(&StepperNode::green_callback, this, std::placeholders::_1));

        // Inisialisasi komunikasi serial
        serial_port_ = "/dev/ttyUSB0"; 
        baud_rate_ = 9600;

        // Membuat objek Timeout terlebih dahulu
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);

        try {
            serial_.setPort(serial_port_);
            serial_.setBaudrate(baud_rate_);
            serial_.setTimeout(timeout); 
            serial_.open();
            if (!serial_.isOpen()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

private:
    void object_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        // Mendapatkan koordinat objek merah
        double x_camera = msg->x;
        double y_camera = msg->y;

        RCLCPP_INFO(this->get_logger(), "Received red object coordinates: [%f, %f]", x_camera, y_camera);

        const double offset_x = 26.0; 
        const double offset_y = 55.0;

        // Konversi koordinat kamera ke koordinat dunia nyata
        double scale_factor = 0.3; 
        double x_world = x_camera * scale_factor + offset_x;
        double y_world = y_camera * scale_factor + offset_y;

        RCLCPP_INFO(this->get_logger(), "Moving to object at world coordinates: [%f, %f]", x_world, y_world);

        // Kirim perintah ke stepper motor untuk bergerak ke lokasi objek
        move_to_position(x_world, y_world);
        grab_object();
    }

    void green_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        // Mendapatkan koordinat target hijau dari kamera
        double x_camera = msg->x;
        double y_camera = msg->y;

        RCLCPP_INFO(this->get_logger(), "Received green target coordinates: [%f, %f]", x_camera, y_camera);

        const double offset_x = 26.0; 
        const double offset_y = 55.0;

        // Konversi koordinat kamera ke koordinat dunia nyata
        double scale_factor = 0.3; 
        double x_world = x_camera * scale_factor + offset_x;
        double y_world = y_camera * scale_factor + offset_y;

        RCLCPP_INFO(this->get_logger(), "Moving to green target at world coordinates: [%f, %f]", x_world, y_world);

        // Kirim perintah ke stepper motor untuk bergerak ke lokasi hijau
        move_to_position(x_world, y_world);
        release_object();
    }

    void move_to_position(double x, double y)
    {
        // Fungsi untuk menggerakkan stepper motor ke posisi yang diberikan
        std::string command = "x" + std::to_string(x) + ",y" + std::to_string(y);
        serial_.write(command);
        RCLCPP_INFO(this->get_logger(), "Moving to position: %s", command.c_str());
    }

    void grab_object()
    {
        // Perintah untuk mengambil objek
        for (int i=0; i < 8; i++){
            serial_.write("zt");
        }
        serial_.write("cj");
        for (int i=0; i < 8; i++){
            serial_.write("zn");
        }
        RCLCPP_INFO(this->get_logger(), "Grabbing object...");
    }

    void release_object()
    {
        // Perintah untuk melepas objek
        for (int i=0; i < 8; i++){
            serial_.write("zt");
        }
        serial_.write("cb");
        for (int i=0; i < 8; i++){
            serial_.write("zn");
        }
        RCLCPP_INFO(this->get_logger(), "Releasing object...");
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr object_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr green_subscriber_;
    serial::Serial serial_;
    std::string serial_port_;
    int baud_rate_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StepperNode>());
    rclcpp::shutdown();
    return 0;
}
