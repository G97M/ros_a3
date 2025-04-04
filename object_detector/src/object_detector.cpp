#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <mutex>

class ObjectDetector : public rclcpp::Node
{
public:
    ObjectDetector() : Node("object_detector")
    {
        // Parameter configuration
        configure_parameters();
        
        // Setup parameter callback
        param_handler_ = this->add_on_set_parameters_callback(
            std::bind(&ObjectDetector::parameter_callback, this, std::placeholders::_1));

        // Subscriber and Publisher
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image", 10,
            std::bind(&ObjectDetector::image_callback, this, std::placeholders::_1));
            
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("object_position", 10);

        RCLCPP_INFO(this->get_logger(), "Object detector initialized");
    }

private:
    void configure_parameters()
    {
        // Threshold parameter
        rcl_interfaces::msg::ParameterDescriptor thresh_desc;
        thresh_desc.name = "threshold";
        thresh_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
        thresh_desc.description = "Brightness threshold (0-255)";
        thresh_desc.integer_range.resize(1);
        thresh_desc.integer_range[0].from_value = 0;
        thresh_desc.integer_range[0].to_value = 255;
        thresh_desc.integer_range[0].step = 1;
        this->declare_parameter("threshold", 200, thresh_desc);

        // Minimum area parameter
        rcl_interfaces::msg::ParameterDescriptor area_desc;
        area_desc.name = "min_area";
        area_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
        area_desc.description = "Minimum detection area in pixels";
        area_desc.integer_range.resize(1);
        area_desc.integer_range[0].from_value = 1;
        area_desc.integer_range[0].to_value = 10000;
        this->declare_parameter("min_area", 100, area_desc);

        // Get initial values
        threshold_ = this->get_parameter("threshold").as_int();
        min_area_ = this->get_parameter("min_area").as_int();
    }

    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        std::lock_guard<std::mutex> lock(param_mutex_);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters) {
            if (param.get_name() == "threshold") {
                if (param.as_int() >= 0 && param.as_int() <= 255) {
                    threshold_ = param.as_int();
                    RCLCPP_INFO(this->get_logger(), "Updated threshold to: %d", threshold_);
                } else {
                    result.successful = false;
                    result.reason = "Threshold must be between 0-255";
                }
            }
            else if (param.get_name() == "min_area") {
                if (param.as_int() > 0) {
                    min_area_ = param.as_int();
                    RCLCPP_INFO(this->get_logger(), "Updated min area to: %d", min_area_);
                } else {
                    result.successful = false;
                    result.reason = "Minimum area must be positive";
                }
            }
        }
        return result;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(param_mutex_);
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            
            // Convert to grayscale and threshold
            cv::Mat gray, binary;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::threshold(gray, binary, threshold_, 255, cv::THRESH_BINARY);

            // Calculate moments
            auto output_msg = geometry_msgs::msg::PointStamped();
            output_msg.header = msg->header;
            
            cv::Moments m = cv::moments(binary, true);
            if (m.m00 > min_area_) {
                output_msg.point.x = m.m10 / m.m00;
                output_msg.point.y = m.m01 / m.m00;
                output_msg.point.z = m.m00;  // Store area in z coordinate indication of the object’s size
            } else {
                output_msg.point.x = NAN;
                output_msg.point.y = NAN;
                output_msg.point.z = 0;
            }

            publisher_->publish(output_msg);

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge error: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handler_;
    
    int threshold_;
    int min_area_;
    std::mutex param_mutex_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetector>());
    rclcpp::shutdown();
    return 0;
}