#include <rclcpp/rclcpp.hpp>
#include <lgpio.h>
#include <chrono>
#include <thread>
#include <vector>

// Motor pin definitions
const int Motor_A_EN = 4;
const int Motor_B_EN = 17;
const int Motor_A_Pin1 = 14;
const int Motor_A_Pin2 = 15;
const int Motor_B_Pin1 = 27;
const int Motor_B_Pin2 = 18;

class MotionTest : public rclcpp::Node
{
public:
    MotionTest() : Node("motion_test"), h(-1), state("forward")
    {
        initialize_gpio();
        if (h < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize GPIO. Cannot proceed.");
            return;
        }

        available_pins = setup();
        if (available_pins.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No GPIO pins available. Cannot proceed.");
            return;
        }

        timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MotionTest::motor_function, this));
        pwm_timer_a = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MotionTest::pwm_function_a, this));
        pwm_timer_b = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MotionTest::pwm_function_b, this));

        state_time = this->now();
        duty_cycle_a = 0;
        duty_cycle_b = 0;
        pwm_counter_a = 0;
        pwm_counter_b = 0;
    }

    ~MotionTest()
    {
        if (h >= 0) {
            motorStop();
            lgpio_stop(h);
        }
    }

private:
    void initialize_gpio()
    {
        h = lgpio_start();
        if (h < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize lgpio");
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully initialized lgpio");
        }
    }

    std::vector<int> setup()
    {
        std::vector<int> pins = {Motor_A_EN, Motor_B_EN, Motor_A_Pin1, Motor_A_Pin2, Motor_B_Pin1, Motor_B_Pin2};
        std::vector<int> available_pins;
        for (int pin : pins) {
            if (lgpio_claim_output(h, 0, pin, 0) == 0) {
                available_pins.push_back(pin);
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to set up pin %d", pin);
            }
        }
        return available_pins;
    }

    void safe_gpio_write(int pin, int value)
    {
        if (std::find(available_pins.begin(), available_pins.end(), pin) != available_pins.end()) {
            if (lgpio_gpio_write(h, pin, value) != 0) {
                RCLCPP_WARN(this->get_logger(), "Failed to write to pin %d", pin);
            }
        }
    }

    void pwm_function_a()
    {
        pwm_counter_a++;
        if (pwm_counter_a >= 100) {
            pwm_counter_a = 0;
        }
        safe_gpio_write(Motor_A_EN, pwm_counter_a < duty_cycle_a ? 1 : 0);
    }

    void pwm_function_b()
    {
        pwm_counter_b++;
        if (pwm_counter_b >= 100) {
            pwm_counter_b = 0;
        }
        safe_gpio_write(Motor_B_EN, pwm_counter_b < duty_cycle_b ? 1 : 0);
    }

    void motorStop()
    {
        safe_gpio_write(Motor_A_Pin1, 0);
        safe_gpio_write(Motor_A_Pin2, 0);
        safe_gpio_write(Motor_B_Pin1, 0);
        safe_gpio_write(Motor_B_Pin2, 0);
        duty_cycle_a = 0;
        duty_cycle_b = 0;
    }

    void motor_A(int status, int speed)
    {
        if (status == 0) {  // stop
            safe_gpio_write(Motor_A_Pin1, 0);
            safe_gpio_write(Motor_A_Pin2, 0);
            duty_cycle_a = 0;
        } else if (status == 1) {  // positive rotation
            safe_gpio_write(Motor_A_Pin1, 1);
            safe_gpio_write(Motor_A_Pin2, 0);
            duty_cycle_a = speed;
        } else if (status == -1) {  // negative rotation
            safe_gpio_write(Motor_A_Pin1, 0);
            safe_gpio_write(Motor_A_Pin2, 1);
            duty_cycle_a = speed;
        }
    }

    void motor_B(int status, int speed)
    {
        if (status == 0) {  // stop
            safe_gpio_write(Motor_B_Pin1, 0);
            safe_gpio_write(Motor_B_Pin2, 0);
            duty_cycle_b = 0;
        } else if (status == 1) {  // positive rotation
            safe_gpio_write(Motor_B_Pin1, 1);
            safe_gpio_write(Motor_B_Pin2, 0);
            duty_cycle_b = speed;
        } else if (status == -1) {  // negative rotation
            safe_gpio_write(Motor_B_Pin1, 0);
            safe_gpio_write(Motor_B_Pin2, 1);
            duty_cycle_b = speed;
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
        rclcpp::Time current_time = this->now();
        if ((current_time - state_time).seconds() > 3.0) {
            if (state == "forward") {
                motorStop();
                std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                move(60, "backward");
                state = "backward";
            } else if (state == "backward") {
                motorStop();
                std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                move(60, "forward");
                state = "forward";
            }
            state_time = current_time;
        }
    }

    int h;
    std::vector<int> available_pins;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr pwm_timer_a;
    rclcpp::TimerBase::SharedPtr pwm_timer_b;
    rclcpp::Time state_time;
    std::string state;
    int duty_cycle_a;
    int duty_cycle_b;
    int pwm_counter_a;
    int pwm_counter_b;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}