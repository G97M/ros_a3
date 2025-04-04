#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class LightStatusNode : public rclcpp::Node
{
public:

/**
     * Constructor for the LightStatusNode.
     *        Initializes the node name, subscribes to "/image" topic,
     *        and advertises the "light_status" topic.
     */



    LightStatusNode() : Node("light_status_node"), threshold_(100.0) // Fixed brightness threshold
    {   // Create a subscriber to the "/image" topic (queue size 10)
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", 10,
            std::bind(&LightStatusNode::image_callback, this, std::placeholders::_1));
            // Create a publisher for the "/light_status" topic (queue size 10
        publisher_ = this->create_publisher<std_msgs::msg::String>("light_status", 10);

        RCLCPP_INFO(this->get_logger(), "Light Status Node started with threshold: %.2f", threshold_);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convert the ROS Image message to an OpenCV Mat in BGR format.
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image; 
            cv::Mat gray; // Convert the color image to grayscale for brightness calculation.
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            // Compute the mean brightness of the grayscale image
            double avg_brightness = cv::mean(gray)[0];

             // Create a String message to publish the status
            std_msgs::msg::String status_msg;
            status_msg.data = (avg_brightness > threshold_) ? "Light is ON" : "Light is OFF";
            publisher_->publish(status_msg);

  // Debug-level logging for brightness and status. Only visible if log level is DEBUG or lower.
            RCLCPP_DEBUG(this->get_logger(), "Brightness: %.2f - Status: %s",
                        avg_brightness, status_msg.data.c_str());

          // Log an error if the image conversion fails              
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge error: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    const double threshold_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LightStatusNode>());
    rclcpp::shutdown();
    return 0;
}