
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <mutex>

class BrightnessDetector : public rclcpp::Node
{
public:
    BrightnessDetector() : Node("brightness_detector")
    {
        // Parameter declaration with descriptor
        rcl_interfaces::msg::ParameterDescriptor threshold_descriptor;
        threshold_descriptor.name = "threshold";
        threshold_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        threshold_descriptor.description = "Brightness threshold (0-255)";
        
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = 255.0;
        range.step = 1.0;
        threshold_descriptor.floating_point_range = {range};

        this->declare_parameter("threshold", 100.0, threshold_descriptor);
        threshold_ = this->get_parameter("threshold").as_double();

        // Parameter change callback
        param_handler_ = this->add_on_set_parameters_callback(
            std::bind(&BrightnessDetector::parameter_callback, this, std::placeholders::_1));

        // Subscriber and Publisher
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", 10,
            std::bind(&BrightnessDetector::image_callback, this, std::placeholders::_1));
            
        publisher_ = this->create_publisher<std_msgs::msg::String>("light_status", 10);

        RCLCPP_INFO(this->get_logger(), "Node started with initial threshold: %.2f", threshold_);
    }

private:
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        for (const auto &param : parameters) {
            if (param.get_name() == "threshold") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                    double new_threshold = param.as_double();
                    if (new_threshold >= 0 && new_threshold <= 255) {
                        std::lock_guard<std::mutex> lock(threshold_mutex_);
                        threshold_ = new_threshold;
                        RCLCPP_INFO(this->get_logger(), "Updated threshold to: %.2f", threshold_);
                    } else {
                        result.successful = false;
                        result.reason = "Threshold must be between 0 and 255";
                    }
                } else {
                    result.successful = false;
                    result.reason = "Threshold must be a double value";
                }
            }
        }
        return result;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            double avg_brightness = cv::mean(gray)[0];
            
            std_msgs::msg::String status_msg;
            {
                std::lock_guard<std::mutex> lock(threshold_mutex_);
                status_msg.data = (avg_brightness > threshold_) ? "Light is ON" : "Light is OFF";
            }
            
            publisher_->publish(status_msg);

            RCLCPP_DEBUG(this->get_logger(), "Brightness: %.2f - Threshold: %.2f - Status: %s",
                        avg_brightness, threshold_, status_msg.data.c_str());
                        
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge error: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handler_;
    
    double threshold_;
    std::mutex threshold_mutex_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrightnessDetector>());
    rclcpp::shutdown();
    return 0;
}