#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64.hpp"

using namespace std::chrono_literals;


/**
 *        This node generates a sequence of velocity setpoints for the left and right motors
 *        of a differential-drive robot. It cycles through an initial delay, a forward motion,
 *        a stop, a backward motion, and another stop, repeating this pattern indefinitely.
 */

class SetpointGenerator : public rclcpp::Node {
public:
    SetpointGenerator() : Node("setpoint_generator"), state_(0), state_counter_(0) {
        left_pub_ = create_publisher<example_interfaces::msg::Float64>(
            "/input/left_motor/setpoint_vel", 10);
        right_pub_ = create_publisher<example_interfaces::msg::Float64>(
            "/input/right_motor/setpoint_vel", 10);

        timer_ = create_wall_timer(20ms, [this]() { timer_callback(); });
        RCLCPP_INFO(get_logger(), "Setpoint generator initialized");
    }

private:
    enum State {INITIAL_DELAY, FORWARD, STOP1, BACKWARD, STOP2};
    
    void timer_callback() {
        // Create messages for left and right wheel velocities.
        example_interfaces::msg::Float64 left_msg;
        example_interfaces::msg::Float64 right_msg;
        bool publish = false;

        switch(state_) {
            case INITIAL_DELAY:
                if(++state_counter_ >= 250) { // 5s delay
                    state_ = FORWARD;
                    state_counter_ = 0;
                    RCLCPP_INFO(get_logger(), "Starting forward motion");
                }
                break;

            case FORWARD:
                left_msg.data = -1.0;
                right_msg.data = 1.0;
                publish = true;
                if(++state_counter_ >= 250) { // 5s forward
                    state_ = STOP1;
                    state_counter_ = 0;
                    RCLCPP_INFO(get_logger(), "Stopping");
                }
                break;

            case STOP1:
                left_msg.data = 0.0;
                right_msg.data = 0.0;
                publish = true;
                if(++state_counter_ >= 150) { // 3s stop
                    state_ = BACKWARD;
                    state_counter_ = 0;
                    RCLCPP_INFO(get_logger(), "Starting backward motion");
                }
                break;

            case BACKWARD:
                left_msg.data = 1.0;
                right_msg.data = -1.0;
                publish = true;
                if(++state_counter_ >= 250) { // 5s backward
                    state_ = STOP2;
                    state_counter_ = 0;
                    RCLCPP_INFO(get_logger(), "Stopping");
                }
                break;

            case STOP2:
                left_msg.data = 0.0;
                right_msg.data = 0.0;
                publish = true;
                if(++state_counter_ >= 150) { // 3s stop
                    state_ = FORWARD;
                    state_counter_ = 0;
                    RCLCPP_INFO(get_logger(), "Restarting sequence");
                }
                break;
        }

        if(publish) {
            left_pub_->publish(left_msg);
            right_pub_->publish(right_msg);
        }
    }
    
    // Timer that triggers 'timer_callback' periodically.
    rclcpp::TimerBase::SharedPtr timer_; 
    // Publishers for left and right wheel velocity setpoints
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_pub_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_pub_;
    int state_;
    int state_counter_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetpointGenerator>());
    rclcpp::shutdown();
    return 0;
}