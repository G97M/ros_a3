#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>

class ColorObjectDetector : public rclcpp::Node
{
public:
    ColorObjectDetector() : Node("color_object_detector")
    {
        // Parameter declaration with default values for green detection
        this->declare_parameter("lower_hsv", std::vector<int64_t>{35, 50, 50});  // H:35-85, S:50-255, V:50-255
        this->declare_parameter("upper_hsv", std::vector<int64_t>{85, 255, 255});
        this->declare_parameter("min_area", 100);

        // Get initial values
        update_parameters();

        // Parameter callback
        param_handler_ = this->add_on_set_parameters_callback(
            std::bind(&ColorObjectDetector::parameter_callback, this, std::placeholders::_1));

        // Subscriber and Publisher
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/output/moving_camera", 10,
            std::bind(&ColorObjectDetector::image_callback, this, std::placeholders::_1));
            
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("object_position", 10);

        RCLCPP_INFO(this->get_logger(), "Color detector initialized for green (H:%d-%d S:%d-%d V:%d-%d)",
                   lower_hsv_[0], upper_hsv_[0], lower_hsv_[1], upper_hsv_[1], lower_hsv_[2], upper_hsv_[2]);
    }

private:
    void update_parameters()
    {
        std::lock_guard<std::mutex> lock(param_mutex_);
        auto lower = this->get_parameter("lower_hsv").as_integer_array();
        auto upper = this->get_parameter("upper_hsv").as_integer_array();
        
        lower_hsv_ = {static_cast<int>(lower[0]), static_cast<int>(lower[1]), static_cast<int>(lower[2])};
        upper_hsv_ = {static_cast<int>(upper[0]), static_cast<int>(upper[1]), static_cast<int>(upper[2])};
        min_area_ = this->get_parameter("min_area").as_int();
    }

    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        std::lock_guard<std::mutex> lock(param_mutex_);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters) {
            if (param.get_name() == "lower_hsv" || param.get_name() == "upper_hsv") {
                auto values = param.as_integer_array();
                if (values.size() != 3) {
                    result.successful = false;
                    result.reason = param.get_name() + " must be a 3-element array";
                }
                else if (param.get_name() == "lower_hsv") {
                    lower_hsv_ = {static_cast<int>(values[0]), static_cast<int>(values[1]), static_cast<int>(values[2])};
                }
                else {
                    upper_hsv_ = {static_cast<int>(values[0]), static_cast<int>(values[1]), static_cast<int>(values[2])};
                }
            }
            else if (param.get_name() == "min_area") {
                if (param.as_int() > 0) {
                    min_area_ = param.as_int();
                } else {
                    result.successful = false;
                    result.reason = "Minimum area must be positive";
                }
            }
        }
        
        if (result.successful) {
            RCLCPP_INFO(this->get_logger(), "Updated parameters: H:%d-%d S:%d-%d V:%d-%d Area:%d",
                       lower_hsv_[0], upper_hsv_[0], lower_hsv_[1], upper_hsv_[1], 
                       lower_hsv_[2], upper_hsv_[2], min_area_);
        }
        return result;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(param_mutex_);
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            
            // Convert to HSV and threshold
            cv::Mat hsv, mask;
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, 
                       cv::Scalar(lower_hsv_[0], lower_hsv_[1], lower_hsv_[2]),
                       cv::Scalar(upper_hsv_[0], upper_hsv_[1], upper_hsv_[2]),
                       mask);

            // Calculate moments
            auto output_msg = geometry_msgs::msg::PointStamped();
            output_msg.header = msg->header;
            
            cv::Moments m = cv::moments(mask, true);
            if (m.m00 > min_area_) {
                output_msg.point.x = m.m10 / m.m00;  // X coordinate
                output_msg.point.y = m.m01 / m.m00;  // Y coordinate
                output_msg.point.z = m.m00;          // Area
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
    
    std::vector<int> lower_hsv_;
    std::vector<int> upper_hsv_;
    int min_area_;
    std::mutex param_mutex_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorObjectDetector>());
    rclcpp::shutdown();
    return 0;
}
//use this code if we want to launch the provided sequence controller beacuse it use point instead of point stamped
/*#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>  // Changed header
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>

class ColorObjectDetector : public rclcpp::Node
{
public:
    ColorObjectDetector() : Node("color_object_detector")
    {
        // Parameter declaration with default values for green detection
        this->declare_parameter("lower_hsv", std::vector<int64_t>{35, 50, 50});
        this->declare_parameter("upper_hsv", std::vector<int64_t>{85, 255, 255});
        this->declare_parameter("min_area", 100);

        // Get initial values
        update_parameters();

        // Parameter callback
        param_handler_ = this->add_on_set_parameters_callback(
            std::bind(&ColorObjectDetector::parameter_callback, this, std::placeholders::_1));

        // Subscriber and Publisher
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/output/moving_camera", 10,
            std::bind(&ColorObjectDetector::image_callback, this, std::placeholders::_1));
            
        // Changed to Point publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("object_position", 10);

        RCLCPP_INFO(this->get_logger(), "Color detector initialized for green (H:%d-%d S:%d-%d V:%d-%d)",
                   lower_hsv_[0], upper_hsv_[0], lower_hsv_[1], upper_hsv_[1], lower_hsv_[2], upper_hsv_[2]);
    }

private:
    void update_parameters()
    {
        std::lock_guard<std::mutex> lock(param_mutex_);
        auto lower = this->get_parameter("lower_hsv").as_integer_array();
        auto upper = this->get_parameter("upper_hsv").as_integer_array();
        
        lower_hsv_ = {static_cast<int>(lower[0]), static_cast<int>(lower[1]), static_cast<int>(lower[2])};
        upper_hsv_ = {static_cast<int>(upper[0]), static_cast<int>(upper[1]), static_cast<int>(upper[2])};
        min_area_ = this->get_parameter("min_area").as_int();
    }

    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        std::lock_guard<std::mutex> lock(param_mutex_);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters) {
            if (param.get_name() == "lower_hsv" || param.get_name() == "upper_hsv") {
                auto values = param.as_integer_array();
                if (values.size() != 3) {
                    result.successful = false;
                    result.reason = param.get_name() + " must be a 3-element array";
                }
                else if (param.get_name() == "lower_hsv") {
                    lower_hsv_ = {static_cast<int>(values[0]), static_cast<int>(values[1]), static_cast<int>(values[2])};
                }
                else {
                    upper_hsv_ = {static_cast<int>(values[0]), static_cast<int>(values[1]), static_cast<int>(values[2])};
                }
            }
            else if (param.get_name() == "min_area") {
                if (param.as_int() > 0) {
                    min_area_ = param.as_int();
                } else {
                    result.successful = false;
                    result.reason = "Minimum area must be positive";
                }
            }
        }
        
        if (result.successful) {
            RCLCPP_INFO(this->get_logger(), "Updated parameters: H:%d-%d S:%d-%d V:%d-%d Area:%d",
                       lower_hsv_[0], upper_hsv_[0], lower_hsv_[1], upper_hsv_[1], 
                       lower_hsv_[2], upper_hsv_[2], min_area_);
        }
        return result;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(param_mutex_);
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            
            // Convert to HSV and threshold
            cv::Mat hsv, mask;
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, 
                       cv::Scalar(lower_hsv_[0], lower_hsv_[1], lower_hsv_[2]),
                       cv::Scalar(upper_hsv_[0], upper_hsv_[1], upper_hsv_[2]),
                       mask);

            // Changed to Point message
            auto output_msg = geometry_msgs::msg::Point();
            
            cv::Moments m = cv::moments(mask, true);
            if (m.m00 > min_area_) {
                output_msg.x = m.m10 / m.m00;  // Direct field access
                output_msg.y = m.m01 / m.m00;
                output_msg.z = m.m00;
            } else {
                output_msg.x = NAN;
                output_msg.y = NAN;
                output_msg.z = 0;
            }

            publisher_->publish(output_msg);

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge error: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;  // Changed type
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handler_;
    
    std::vector<int> lower_hsv_;
    std::vector<int> upper_hsv_;
    int min_area_;
    std::mutex param_mutex_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorObjectDetector>());
    rclcpp::shutdown();
    return 0;
}*/