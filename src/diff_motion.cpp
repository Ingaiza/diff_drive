#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64.hpp"
#include <cmath>
#include <queue>
#include <lgpio.h>


#ifndef M_PI
#define M_PI 3.14159265358979323846;
#endif

// Motor pin definitions
const int Motor_A_EN = 4;
const int Motor_B_EN = 17;
const int Motor_A_Pin1 = 14;
const int Motor_A_Pin2 = 15;
const int Motor_B_Pin1 = 27;
const int Motor_B_Pin2 = 18;

class motionnode : public rclcpp::Node 
{
public:
    motionnode() : Node("diff_motion")
    {
        rightduty_callbackgroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        leftduty_callbackgroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions right_duty_,left_duty_;
        right_duty_.callback_group = rightduty_callbackgroup_;
        left_duty_.callback_group = leftduty_callbackgroup_;

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

        pwm_timer_a_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&motionnode::pwm_a_callback,this)
        );

        pwm_timer_b_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&motionnode::pwm_b_callback,this)
        );

        //KPP
        right_duty_subscription_ = this->create_subscription<example_interfaces::msg::Float64>(
            "right_duty",
            100,
            std::bind(&motionnode::right_duty_subcallback,this,std::placeholders::_1),
            right_duty_);

        left_duty_subscription_ = this->create_subscription<example_interfaces::msg::Float64>(
            "left_duty",
            100,
            std::bind(&motionnode::left_duty_subcallback,this,std::placeholders::_1),
            left_duty_);
        //KPP
        
        RCLCPP_INFO(this->get_logger(), "Motion Subscription is Active");

        pwm_counter_a = 0;
        pwm_counter_b = 0;

    }

    ~motionnode()
    {
        if (h >= 0) {
            motorStop();
            lgGpiochipClose(h);
        }
    }

private:

    void right_duty_subcallback(const example_interfaces::msg::Float64::SharedPtr msg)
    {
        //static_cast<void>(msg);
        {
            std::lock_guard<std::mutex> lock(right_duty_queue_mutex_);
            right_duty_queue_.push(msg);
            // RCLCPP_WARN(this->get_logger(),"PUSHED RIGHT DUTY MSG TO QUEUE");
        
        }
        
    }

    void left_duty_subcallback(const example_interfaces::msg::Float64::SharedPtr msg)
    {
        //static_cast<void>(msg); 
        {
            std::lock_guard<std::mutex> lock(left_duty_queue_mutex_);
            left_duty_queue_.push(msg);
            // RCLCPP_WARN(this->get_logger(),"PUSHED LEFT DUTY MSG TO QUEUE");
        
        }   
    
    }

    void initialize_gpio()
    {
        h = lgGpiochipOpen(0);  // Try opening chip 0
        if (h < 0) {
            h = lgGpiochipOpen(1);  // If chip 0 fails, try chip 1
        }
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
            if (lgGpioClaimOutput(h, 0, pin, LG_LOW) == LG_OKAY) {
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
            if (lgGpioWrite(h, pin, value) != LG_OKAY) {
                RCLCPP_WARN(this->get_logger(), "Failed to write to pin %d", pin);
            }
        }
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

    void pwm_a_callback()
    {
        example_interfaces::msg::Float64::SharedPtr duty;
        if(!right_duty_queue_.empty())
        {
            {
                std::lock_guard<std::mutex> lock(right_duty_queue_mutex_);
                duty = right_duty_queue_.front();
                right_duty_queue_.pop();

            }          
        }

        pwm_counter_a++;
        if (pwm_counter_a >= 100) 
        {
            pwm_counter_a = 0;
        }

        if(duty)
        {
            duty_a = duty->data;
            duty_cycle_a = (duty_a) * 100;
        }

        safe_gpio_write(Motor_A_EN, pwm_counter_a < duty_cycle_a ? 1 : 0);          
        // RCLCPP_WARN(this->get_logger(),"MOTOR_A_EN PWM IS ACTIVE");

        //safe_gpio_write(Motor_A_EN, 0); //disable pwm
        // RCLCPP_WARN(this->get_logger(),"RIGHT QUEUE IS EMPTY"); 
        
    }

    void pwm_b_callback()
    {   
        example_interfaces::msg::Float64::SharedPtr duty;
        if(!left_duty_queue_.empty())
        {
            {
                std::lock_guard<std::mutex> lock(left_duty_queue_mutex_);        
                duty = left_duty_queue_.front();
                left_duty_queue_.pop();
                
            }
        }

         pwm_counter_b++;
        if (pwm_counter_b >= 100) 
        {
            pwm_counter_b = 0;
        }

        if(duty)
        {
            duty_b = duty->data;
            duty_cycle_b = duty_b * 100;
        }
        
        safe_gpio_write(Motor_B_EN, pwm_counter_b < duty_cycle_b ? 1 : 0);         
        // RCLCPP_WARN(this->get_logger(),"MOTOR_B_EN PWM IS ACTIVE");   
    
        //safe_gpio_write(Motor_B_EN, 0); //disable pwm 
        // RCLCPP_WARN(this->get_logger(),"LEFT QUEUE IS EMPTY");  
        
    }

    void motor_A()
    {   
        try
        {
            safe_gpio_write(Motor_A_Pin1, 0);
            safe_gpio_write(Motor_A_Pin2, 1);
            // RCLCPP_INFO(this->get_logger(),"MOTOR_B IN1 & IN2  PINS ACTIVE");
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "FAILED TO WRITE TO MOTOR A PINS");
        }
        
       
    }

    void motor_B()
    {
        try
        {
            safe_gpio_write(Motor_B_Pin1, 0);
            safe_gpio_write(Motor_B_Pin2, 1);
            // RCLCPP_INFO(this->get_logger(),"MOTOR_B IN1 & IN2  PINS ACTIVE");
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "FAILED TO WRITE TO MOTOR B PINS");   
        }
        
    }

    void motor_callback()
    {
        motor_A();
        motor_B();
        /*
        if(!(right_duty_queue_.empty() ||  left_duty_queue_.empty()))
        {
            motor_A();
            motor_B();
        }
        else
        {   
            RCLCPP_ERROR(this->get_logger(),"MOTOR STOP FUNCTION IS ACTIVE");
            motorStop();
        }
        */
    }

    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr right_duty_subscription_;
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr left_duty_subscription_;    
    rclcpp::TimerBase::SharedPtr pwm_timer_a_;
    rclcpp::TimerBase::SharedPtr pwm_timer_b_;
    rclcpp::CallbackGroup::SharedPtr rightduty_callbackgroup_;
    rclcpp::CallbackGroup::SharedPtr leftduty_callbackgroup_;
    std::queue<example_interfaces::msg::Float64::SharedPtr> right_duty_queue_;
    std::queue<example_interfaces::msg::Float64::SharedPtr> left_duty_queue_;
    std::mutex right_duty_queue_mutex_;
    std::mutex left_duty_queue_mutex_;
    double duty_a;
    double duty_b;
    int h; //gpio handle
    std::vector<int> available_pins;
    int duty_cycle_a = 0;
    int duty_cycle_b = 0;
    int pwm_counter_a;
    int pwm_counter_b;   

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motionnode>(); 
    rclcpp::executors::MultiThreadedExecutor executor_;
    executor_.add_node(node);
    executor_.spin();
    rclcpp::shutdown();
    return 0;
}
