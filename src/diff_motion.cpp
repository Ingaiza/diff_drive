#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "example_interfaces/msg/float64.hpp"
#include <cmath>
#include <queue>
#include <lgpio.h>


#ifndef M_PI
#define M_PI = 3.14159265358979323846;
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
        motionsub_callbackgroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        twist2vel_callbackgroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rightduty_callbackgroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        leftduty_callbackgroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions motion_,right_duty_,left_duty_;
        motion_.callback_group = motionsub_callbackgroup_;
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


        motion_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            1 , 
            std::bind(&motionnode::motion_callback,this,std::placeholders::_1),
            motion_);

        twist_to_vel_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&motionnode::twist_to_vel_callback,this),
            twist2vel_callbackgroup_);

        pwm_timer_a_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&motionnode::pwm_a_callback,this)
        );

        pwm_timer_b_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&motionnode::pwm_b_callback,this)
        );
        motor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&motionnode::motor_callback,this)
        );

        right_duty_subscription_ = this->create_subscription<example_interfaces::msg::Float64>(
            "right_duty",
            1,
            std::bind(&motionnode::right_duty_subcallback,this,std::placeholders::_1),
            right_duty_);

        left_duty_subscription_ = this->create_subscription<example_interfaces::msg::Float64>(
            "left_duty",
            1,
            std::bind(&motionnode::left_duty_subcallback,this,std::placeholders::_1),
            left_duty_);

        right_duty_publisher_ = this->create_publisher<example_interfaces::msg::Float64>("right_duty", 1);
        left_duty_publisher_ = this->create_publisher<example_interfaces::msg::Float64>("left_duty", 1);
        
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

    void motion_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {   
        RCLCPP_WARN(this->get_logger(),"MOTION CALLBACK IS ACTIVE");

        {
            std::lock_guard<std::mutex> lock(motionqueue_mutex_);
            motion_queue_.push(msg);
            //RCLCPP_WARN(this->get_logger(),"PUSHED MOTION MSG TO QUEUE");
        
        }

    }
    void twist_to_vel_callback()
    {
         //RCLCPP_INFO(this->get_logger(),"Started vel timer callback");

            geometry_msgs::msg::Twist::SharedPtr msg;
            {
                std::lock_guard<std::mutex> lock(motionqueue_mutex_);
                if (!motion_queue_.empty())
                {
                    msg = motion_queue_.front();
                    motion_queue_.pop();
                }
            }

            if(msg)
            {
                //RCLCPP_INFO(this->get_logger(),"Linear Velocity: %f, Angular Velocity: %f",msg->linear.x,msg->angular.z);

                //Find angular velocities for each wheel
                right_wheel_angular_vel = this->right_diff_calculator(msg);
                left_wheel_angular_vel = this->left_diff_calculator(msg);

                //RCLCPP_INFO(this->get_logger(),"Right Wheel Angular Velocity: %f, Left Wheel Angular Velocity: %f", right_wheel_angular_vel,left_wheel_angular_vel);
                
                //Find voltages for each wheel 
                auto right_wheel_voltage = this->pwm_voltage_calc(right_wheel_angular_vel);
                auto left_wheel_voltage = this->pwm_voltage_calc(left_wheel_angular_vel);
                
                //RCLCPP_INFO(this->get_logger(),"Right Wheel Voltage: %f, Left Wheel Voltage: %f", right_wheel_voltage,left_wheel_voltage);

                /* The max voltage measured in the rasptank was is 8.0V so if the required value exceeds 
                max value set it to the max value(8.0V) */
                if(right_wheel_voltage > 8.0)
                {
                    right_wheel_voltage = 8.0;
                }
                else if(left_wheel_voltage > 8.0)
                {
                    left_wheel_voltage = 8.0;
                }
                //Find PWM duty cycles required for each wheel
                example_interfaces::msg::Float64 right_duty_cycle;
                right_duty_cycle.data = this->duty_cycle(right_wheel_voltage);
                example_interfaces::msg::Float64 left_duty_cycle;
                left_duty_cycle.data = this->duty_cycle(left_wheel_voltage);

                //RCLCPP_WARN(this->get_logger(),"RIGHT DUTY CYCLE: %f, LEFT DUTY CYCLE: %f",right_duty_cycle.data,left_duty_cycle.data);
                
                right_duty_publisher_->publish(right_duty_cycle);
                left_duty_publisher_-> publish(left_duty_cycle);


            }
            else
            {
                //RCLCPP_WARN(this->get_logger(),"NO MSG");
                //std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }


    }

    void right_duty_subcallback(const example_interfaces::msg::Float64::SharedPtr msg)
    {
        //static_cast<void>(msg);
        {
            std::lock_guard<std::mutex> lock(right_duty_queue_mutex_);
            right_duty_queue_.push(msg);
            RCLCPP_WARN(this->get_logger(),"PUSHED RIGHT DUTY MSG TO QUEUE");
        
        }
        
    }

    void left_duty_subcallback(const example_interfaces::msg::Float64::SharedPtr msg)
    {
        //static_cast<void>(msg); 
        {
            std::lock_guard<std::mutex> lock(left_duty_queue_mutex_);
            left_duty_queue_.push(msg);
            RCLCPP_WARN(this->get_logger(),"PUSHED LEFT DUTY MSG TO QUEUE");
        
        }   
    
    }

    double right_diff_calculator(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double linear_vel = msg->linear.x;
        double angular_vel = msg->angular.z;
        double vel_right = (2*linear_vel + angular_vel*wheel_offset)/(2*wheel_radius);

        return vel_right;

    }

    double left_diff_calculator(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double linear_vel = msg->linear.x;
        double angular_vel = msg->angular.z;
        double vel_left = (2*linear_vel - angular_vel*wheel_offset)/(2*wheel_radius);

        return vel_left;

    }

    double pwm_voltage_calc(const double vel)
    {
        auto voltage = vel * 6/dcmotor_angular_vel; /* The GA12-N20 has a linear relationship between 
        Voltage applied and the RPM so we use the datasheet value of 6V which should give
        80RPM when loaded to find the desired voltage for a specific angular velocity (vel) */

        return voltage;
    }

    double duty_cycle(const double voltage)
    {
        auto duty = voltage/peak_voltage; 
        return duty;
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
        if(!right_duty_queue_.empty())
        {   
            example_interfaces::msg::Float64::SharedPtr duty;
            {
                std::lock_guard<std::mutex> lock(right_duty_queue_mutex_);
                duty = right_duty_queue_.front();
                right_duty_queue_.pop();

            }
            if(duty == NULL)
            {
                RCLCPP_WARN(this->get_logger(),"DUTY A IS NULL"); 
            }
            duty_cycle_a = (duty->data) * 100;
            pwm_counter_a++;
            if (pwm_counter_a >= 100) 
            {
                pwm_counter_a = 0;
            }
            safe_gpio_write(Motor_A_EN, pwm_counter_a < duty_cycle_a ? 1 : 0);
            RCLCPP_WARN(this->get_logger(),"MOTOR_A_EN PWM IS ACTIVE");
        }
        else
        {
            safe_gpio_write(Motor_A_EN, 0); //disable pwm
            RCLCPP_WARN(this->get_logger(),"PWM DISABLED"); 
        }
    }

    void pwm_b_callback()
    {
        if(!left_duty_queue_.empty())
        {
            example_interfaces::msg::Float64::SharedPtr duty;
            {
                std::lock_guard<std::mutex> lock(left_duty_queue_mutex_);
                if (!left_duty_queue_.empty())
                {
                    duty = left_duty_queue_.front();
                    left_duty_queue_.pop();
                }
            }
            if(duty == NULL)
            {
                RCLCPP_WARN(this->get_logger(),"DUTY B IS NULL"); 
            }
            duty_cycle_b = (duty->data) * 100;
            pwm_counter_b++;
            if (pwm_counter_b >= 100) 
            {
                pwm_counter_b = 0;
            }
            safe_gpio_write(Motor_B_EN, pwm_counter_b < duty_cycle_b ? 1 : 0);
            RCLCPP_WARN(this->get_logger(),"MOTOR_B_EN PWM IS ACTIVE");   
        }
        else
        {
            safe_gpio_write(Motor_B_EN, 0); //disable pwm 
            RCLCPP_WARN(this->get_logger(),"PWM DISABLED");  
        }
    }

    void motor_A()
    {   
        try
        {
            safe_gpio_write(Motor_A_Pin1, 0);
            safe_gpio_write(Motor_A_Pin2, 1);
            RCLCPP_INFO(this->get_logger(),"MOTOR_B IN1 & IN2  PINS ACTIVE");
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
            RCLCPP_INFO(this->get_logger(),"MOTOR_B IN1 & IN2  PINS ACTIVE");
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

    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_duty_publisher_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_duty_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_subscription_;
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr right_duty_subscription_;
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr left_duty_subscription_;    
    rclcpp::TimerBase::SharedPtr twist_to_vel_;
    rclcpp::TimerBase::SharedPtr pwm_timer_a_;
    rclcpp::TimerBase::SharedPtr pwm_timer_b_;
    rclcpp::TimerBase::SharedPtr motor_timer_;
    rclcpp::CallbackGroup::SharedPtr motionsub_callbackgroup_;
    rclcpp::CallbackGroup::SharedPtr twist2vel_callbackgroup_;
    rclcpp::CallbackGroup::SharedPtr rightduty_callbackgroup_;
    rclcpp::CallbackGroup::SharedPtr leftduty_callbackgroup_;
    std::queue<geometry_msgs::msg::Twist::SharedPtr> motion_queue_;
    std::queue<example_interfaces::msg::Float64::SharedPtr> right_duty_queue_;
    std::queue<example_interfaces::msg::Float64::SharedPtr> left_duty_queue_;
    std::mutex motionqueue_mutex_;
    std::mutex right_duty_queue_mutex_;
    std::mutex left_duty_queue_mutex_;
    double wheel_radius = 0.0225; // radius of the wheels
    double wheel_offset = 0.105; // distance between the center of the left and right wheel
    double right_wheel_angular_vel; // right wheel angular vel calculated from cmd_vel command
    double left_wheel_angular_vel; // left wheel angular vel calculated from cmd_vel command
    double dcmotor_angular_vel = 80 * (2*M_PI)/60; /*This is the rated angular velocity for the GA12-N20 
    dc motor used in adeept rasp tank when loaded at 6 Volts, the unloaded value is 100RPM */
    double peak_voltage = 8; //Approximate Peak Voltage in the rasptank 
    int h; //gpio handle
    std::vector<int> available_pins;
    int duty_cycle_a;
    int duty_cycle_b;
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
