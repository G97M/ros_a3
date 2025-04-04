#include <chrono>
#include <memory>
#include <cmath>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;

class ExtendedSequenceController : public rclcpp::Node {
public:
    ExtendedSequenceController()
    : Node("extended_sequence_controller"),
      seq_substate_(INITIAL_DELAY),
      seq_counter_(0)
    {
        // -----------------------------
        // 1) Declare runtime parameters
        // -----------------------------
        this->declare_parameter<std::string>("control_mode", "follow");  // "sequence" or "follow"

        this->declare_parameter<double>("min_area", 2000.0);
        this->declare_parameter<double>("max_area", 4000.0);
        this->declare_parameter<double>("fwd_speed", 5.5);
        this->declare_parameter<double>("turn_threshold", 5.0);
        this->declare_parameter<double>("turn_speed", 1.5);

        // Retrieve initial values
        control_mode_   = this->get_parameter("control_mode").as_string();
        min_area_       = this->get_parameter("min_area").as_double();
        max_area_       = this->get_parameter("max_area").as_double();
        fwd_speed_      = this->get_parameter("fwd_speed").as_double();
        turn_threshold_ = this->get_parameter("turn_threshold").as_double();
        turn_speed_     = this->get_parameter("turn_speed").as_double();

        // -----------------------------
        // 2) Create param callback
        // -----------------------------
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ExtendedSequenceController::on_parameter_change, this, std::placeholders::_1)
        );

        // Publishers for wheel velocities
        left_pub_ = this->create_publisher<example_interfaces::msg::Float64>(
            "/input/left_motor/setpoint_vel", 10);
        right_pub_ = this->create_publisher<example_interfaces::msg::Float64>(
            "/input/right_motor/setpoint_vel", 10);

        // Subscribe to object position for follow mode
        object_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/object_position", 10,
            std::bind(&ExtendedSequenceController::object_callback, this, std::placeholders::_1));

        // Timer to run at 50 Hz
        timer_ = this->create_wall_timer(20ms, [this]() { this->control_loop(); });

        RCLCPP_INFO(get_logger(), "ExtendedSequenceController initialized (mode=%s)", control_mode_.c_str());
    }

private:
    // States for the old forward/backward sequence
    enum SequenceState { INITIAL_DELAY, FORWARD, STOP1, BACKWARD, STOP2 };

    // -----------------------------
    // 3) Parameter change callback
    // -----------------------------
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> & params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (auto & param : params) {
            if (param.get_name() == "control_mode") {
                control_mode_ = param.as_string();
                RCLCPP_INFO(get_logger(), "Updated control_mode to '%s'", control_mode_.c_str());
            }
            else if (param.get_name() == "min_area") {
                min_area_ = param.as_double();
                RCLCPP_INFO(get_logger(), "Updated min_area to %.2f", min_area_);
            }
            else if (param.get_name() == "max_area") {
                max_area_ = param.as_double();
                RCLCPP_INFO(get_logger(), "Updated max_area to %.2f", max_area_);
            }
            else if (param.get_name() == "fwd_speed") {
                fwd_speed_ = param.as_double();
                RCLCPP_INFO(get_logger(), "Updated fwd_speed to %.2f", fwd_speed_);
            }
            else if (param.get_name() == "turn_threshold") {
                turn_threshold_ = param.as_double();
                RCLCPP_INFO(get_logger(), "Updated turn_threshold to %.2f", turn_threshold_);
            }
            else if (param.get_name() == "turn_speed") {
                turn_speed_ = param.as_double();
                RCLCPP_INFO(get_logger(), "Updated turn_speed to %.2f", turn_speed_);
            }
        }
        return result;
    }

    // Main control loop called at 50 Hz
    void control_loop() {
        if (control_mode_ == "sequence") {
            run_sequence_mode();
        } else {
            run_follow_mode();
        }
    }

    // 1) Old forward/backward sequence
    void run_sequence_mode() {
        example_interfaces::msg::Float64 left_msg, right_msg;
        left_msg.data = 0.0;
        right_msg.data = 0.0;

        switch (seq_substate_) {
            case INITIAL_DELAY:
                if (++seq_counter_ >= 250) { // 5s
                    seq_substate_ = FORWARD;
                    seq_counter_ = 0;
                    RCLCPP_INFO(get_logger(), "SEQ: Starting forward motion");
                }
                break;
            case FORWARD:
                left_msg.data = -1.0;
                right_msg.data =  1.0;
                if (++seq_counter_ >= 250) {
                    seq_substate_ = STOP1;
                    seq_counter_ = 0;
                    RCLCPP_INFO(get_logger(), "SEQ: Stopping");
                }
                break;
            case STOP1:
                left_msg.data = 0.0;
                right_msg.data = 0.0;
                if (++seq_counter_ >= 150) {
                    seq_substate_ = BACKWARD;
                    seq_counter_ = 0;
                    RCLCPP_INFO(get_logger(), "SEQ: Backward motion");
                }
                break;
            case BACKWARD:
                left_msg.data = 1.0;
                right_msg.data = -1.0;
                if (++seq_counter_ >= 250) {
                    seq_substate_ = STOP2;
                    seq_counter_ = 0;
                    RCLCPP_INFO(get_logger(), "SEQ: Stopping again");
                }
                break;
            case STOP2:
                left_msg.data = 0.0;
                right_msg.data = 0.0;
                if (++seq_counter_ >= 150) {
                    seq_substate_ = FORWARD;
                    seq_counter_ = 0;
                    RCLCPP_INFO(get_logger(), "SEQ: Restart cycle");
                }
                break;
        }

        left_pub_->publish(left_msg);
        right_pub_->publish(right_msg);
    }

    // 2) Simple follow mode with dynamic parameters
    void run_follow_mode() {
        example_interfaces::msg::Float64 left_msg, right_msg;
        left_msg.data = 0.0;
        right_msg.data = 0.0;

        // Lock the object data while we read it
        {
            std::lock_guard<std::mutex> lock(object_mutex_);
            if (std::isnan(object_x_)) {
                // No valid object -> stop
                left_msg.data = 0.0;
                right_msg.data = 0.0;
            } else {
                // Basic logic: turn if object is left or right of center
                const double IMAGE_CENTER = 180.0; // e.g. if your camera is 640 wide
                double error_x = object_x_ - IMAGE_CENTER;

                if (error_x < -turn_threshold_) {
                    // object is left of center
                    left_msg.data  = turn_speed_;
                    right_msg.data = turn_speed_;
                }
                else if (error_x > turn_threshold_) {
                    // object is right of center
                    left_msg.data  = -turn_speed_;
                    right_msg.data = -turn_speed_;
                }

                // Move forward/backward based on area
                if (object_area_ < min_area_) {
                    // object is too small/far -> move forward
                    left_msg.data  += -fwd_speed_;
                    right_msg.data +=  fwd_speed_;
                }
                else if (object_area_ > max_area_) {
                    // object is too large/close -> move backward
                    left_msg.data  +=  fwd_speed_;
                    right_msg.data += -fwd_speed_;
                }
            }
        }

        left_pub_->publish(left_msg);
        right_pub_->publish(right_msg);
    }

    // Callback for /object_position
    void object_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(object_mutex_);
        object_x_ = msg->point.x;
        object_y_ = msg->point.y;
        object_area_ = msg->point.z;
    }

    // Publishers, Subscribers, Timers
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_pub_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr object_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Sequence state
    enum SequenceState seq_substate_;
    int seq_counter_;

    // Control mode and param callback
    std::string control_mode_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Shared object data
    std::mutex object_mutex_;
    double object_x_{NAN};
    double object_y_{NAN};
    double object_area_{0.0};

    // Dynamic parameters
    double min_area_;
    double max_area_;
    double fwd_speed_;
    double turn_threshold_;
    double turn_speed_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExtendedSequenceController>());
    rclcpp::shutdown();
    return 0;
}
