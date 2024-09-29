#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <gpiod.hpp>
#include <thread>

using namespace std::chrono_literals;

class MotionTest : public rclcpp::Node
{
public:
    MotionTest()
    : Node("motion_test"), state_("forward")
    {
        // Motor pin definitions
        Motor_A_EN = 4;
        Motor_B_EN = 17;
        Motor_A_Pin1 = 14;
        Motor_A_Pin2 = 15;
        Motor_B_Pin1 = 27;
        Motor_B_Pin2 = 18;

        // Initialize GPIO
        initialize_gpio();

        // Create timers
        timer_ = this->create_wall_timer(100ms, std::bind(&MotionTest::motor_function, this));
        pwm_timer_a_ = this->create_wall_timer(10ms, std::bind(&MotionTest::pwm_function_a, this));
        pwm_timer_b_ = this->create_wall_timer(10ms, std::bind(&MotionTest::pwm_function_b, this));

        state_time_ = this->now();
        duty_cycle_a_ = 0;
        duty_cycle_b_ = 0;
        pwm_counter_a_ = 0;
        pwm_counter_b_ = 0;
    }

    ~MotionTest()
    {
        motorStop();
        if (chip_) {
            chip_->reset();
        }
    }

private:
    void initialize_gpio()
    {
        for (int chip_num = 0; chip_num < 2; ++chip_num) {
            try {
                chip_ = std::make_unique<gpiod::chip>(std::to_string(chip_num));
                RCLCPP_INFO(this->get_logger(), "Successfully opened GPIO chip %d", chip_num);
                break;
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to open GPIO chip %d: %s", chip_num, e.what());
            }
        }

        if (!chip_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open any GPIO chip");
            return;
        }

        try {
            lines_ = chip_->get_lines({Motor_A_EN, Motor_B_EN, Motor_A_Pin1, Motor_A_Pin2, Motor_B_Pin1, Motor_B_Pin2});
            lines_.request({"motion_test", gpiod::line_request::DIRECTION_OUTPUT, 0});
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize GPIO lines: %s", e.what());
        }
    }

    void safe_gpio_write(int pin_index, int value)
    {
        if (pin_index < 0 || pin_index >= static_cast<int>(lines_.size())) {
            RCLCPP_WARN(this->get_logger(), "Invalid pin index: %d", pin_index);
            return;
        }

        try {
            lines_[pin_index].set_value(value);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to write to pin %d: %s", pin_index, e.what());
        }
    }

    void pwm_function_a()
    {
        pwm_counter_a_++;
        if (pwm_counter_a_ >= 100) {
            pwm_counter_a_ = 0;
        }
        safe_gpio_write(0, pwm_counter_a_ < duty_cycle_a_ ? 1 : 0);
    }

    void pwm_function_b()
    {
        pwm_counter_b_++;
        if (pwm_counter_b_ >= 100) {
            pwm_counter_b_ = 0;
        }
        safe_gpio_write(1, pwm_counter_b_ < duty_cycle_b_ ? 1 : 0);
    }

    void motorStop()
    {
        safe_gpio_write(2, 0);
        safe_gpio_write(3, 0);
        safe_gpio_write(4, 0);
        safe_gpio_write(5, 0);
        duty_cycle_a_ = 0;
        duty_cycle_b_ = 0;
    }

    void motor_A(int status, int speed)
    {
        if (status == 0) {
            safe_gpio_write(2, 0);
            safe_gpio_write(3, 0);
            duty_cycle_a_ = 0;
        } else if (status == 1) {
            safe_gpio_write(2, 1);
            safe_gpio_write(3, 0);
            duty_cycle_a_ = speed;
        } else if (status == -1) {
            safe_gpio_write(2, 0);
            safe_gpio_write(3, 1);
            duty_cycle_a_ = speed;
        }
    }

    void motor_B(int status, int speed)
    {
        if (status == 0) {
            safe_gpio_write(4, 0);
            safe_gpio_write(5, 0);
            duty_cycle_b_ = 0;
        } else if (status == 1) {
            safe_gpio_write(4, 1);
            safe_gpio_write(5, 0);
            duty_cycle_b_ = speed;
        } else if (status == -1) {
            safe_gpio_write(4, 0);
            safe_gpio_write(5, 1);
            duty_cycle_b_ = speed;
        }
    }

    void move(int speed, const std::string& direction)
    {
        if (direction == "forward") {
            motor_A(1, speed);
            motor_B(1, speed);
        } else if (direction == "backward") {
            motor_A(-1, speed);
            motor_B(-1, speed);
        } else if (direction == "stop") {
            motor_A(0, speed);
            motor_B(0, speed);
        }
    }

    void motor_function()
    {
        auto current_time = this->now();
        if ((current_time - state_time_).seconds() > 3.0) {
            if (state_ == "forward") {
                motorStop();
                std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                move(60, "backward");
                state_ = "backward";
            } else if (state_ == "backward") {
                motorStop();
                std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                move(60, "forward");
                state_ = "forward";
            }
            state_time_ = current_time;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr pwm_timer_a_;
    rclcpp::TimerBase::SharedPtr pwm_timer_b_;
    rclcpp::Time state_time_;
    std::string state_;
    int duty_cycle_a_;
    int duty_cycle_b_;
    int pwm_counter_a_;
    int pwm_counter_b_;
    std::unique_ptr<gpiod::chip> chip_;
    gpiod::line_bulk lines_;

    int Motor_A_EN;
    int Motor_B_EN;
    int Motor_A_Pin1;
    int Motor_A_Pin2;
    int Motor_B_Pin1;
    int Motor_B_Pin2;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}