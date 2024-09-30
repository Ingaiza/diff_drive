#include <rclcpp/rclcpp.hpp>
#include <lgpio.h>
#include <vector>
#include <algorithm>

// Motor pin definitions
const int Motor_A_EN = 4;
const int Motor_B_EN = 17;
const int Motor_A_Pin1 = 14;
const int Motor_A_Pin2 = 15;
const int Motor_B_Pin1 = 27;
const int Motor_B_Pin2 = 18;

class GPIOFreeNode : public rclcpp::Node
{
public:
    GPIOFreeNode() : Node("gpio_free_node"), h(-1)
    {
        initialize_gpio();
        if (h < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize GPIO. Cannot proceed.");
            return;
        }

        free_pins();
        lgGpiochipClose(h);
        RCLCPP_INFO(this->get_logger(), "GPIO pins freed and chip closed.");
    }

private:
    void initialize_gpio()
    {
        h = lgGpiochipOpen(0); // Try opening chip 0
        if (h < 0) {
            h = lgGpiochipOpen(1); // If chip 0 fails, try chip 1
        }
        if (h < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize lgpio");
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully initialized lgpio");
        }
    }

    void free_pins()
    {
        std::vector<int> pins = {Motor_A_EN, Motor_B_EN, Motor_A_Pin1, Motor_A_Pin2, Motor_B_Pin1, Motor_B_Pin2};
        for (int pin : pins) {
            if (lgGpioFree(h, pin) == LG_OKAY) {
                RCLCPP_INFO(this->get_logger(), "Successfully freed pin %d", pin);
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to free pin %d", pin);
            }
        }
    }

    int h;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPIOFreeNode>();
    rclcpp::shutdown();
    return 0;
}