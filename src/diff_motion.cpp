#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "example_interfaces/msg/float64.hpp"
#include <cmath>
#include <queue>

#ifndef M_PI
#define M_PI = 3.14159265358979323846;
#endif

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


        motion_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            1 , 
            std::bind(&motionnode::motion_callback,this,std::placeholders::_1),
            motion_);

        twist_to_vel_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&motionnode::twist_to_vel_callback,this),
            twist2vel_callbackgroup_);

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
                RCLCPP_INFO(this->get_logger(),"Linear Velocity: %f, Angular Velocity: %f",msg->linear.x,msg->angular.z);

                //Find angular velocities for each wheel
                right_wheel_angular_vel = this->right_diff_calculator(msg);
                left_wheel_angular_vel = this->left_diff_calculator(msg);

                RCLCPP_INFO(this->get_logger(),"Right Wheel Angular Velocity: %f, Left Wheel Angular Velocity: %f", right_wheel_angular_vel,left_wheel_angular_vel);
                
                //Find voltages for each wheel 
                auto right_wheel_voltage = this->pwm_voltage_calc(right_wheel_angular_vel);
                auto left_wheel_voltage = this->pwm_voltage_calc(left_wheel_angular_vel);
                
                RCLCPP_INFO(this->get_logger(),"Right Wheel Voltage: %f, Left Wheel Voltage: %f", right_wheel_voltage,left_wheel_voltage);

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

                RCLCPP_WARN(this->get_logger(),"RIGHT DUTY CYCLE: %f, LEFT DUTY CYCLE: %f",right_duty_cycle.data,left_duty_cycle.data);
                
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
        static_cast<void>(msg);
        
    }

    void left_duty_subcallback(const example_interfaces::msg::Float64::SharedPtr msg)
    {
        static_cast<void>(msg);    
    
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

    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_duty_publisher_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_duty_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_subscription_;
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr right_duty_subscription_;
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr left_duty_subscription_;    
    rclcpp::TimerBase::SharedPtr twist_to_vel_;
    rclcpp::CallbackGroup::SharedPtr motionsub_callbackgroup_;
    rclcpp::CallbackGroup::SharedPtr twist2vel_callbackgroup_;
    rclcpp::CallbackGroup::SharedPtr rightduty_callbackgroup_;
    rclcpp::CallbackGroup::SharedPtr leftduty_callbackgroup_;
    std::queue<geometry_msgs::msg::Twist::SharedPtr> motion_queue_;
    std::mutex motionqueue_mutex_;
    double wheel_radius = 0.0225; // radius of the wheels
    double wheel_offset = 0.105; // distance between the center of the left and right wheel
    double right_wheel_angular_vel; // right wheel angular vel calculated from cmd_vel command
    double left_wheel_angular_vel; // left wheel angular vel calculated from cmd_vel command
    double dcmotor_angular_vel = 80 * (2*M_PI)/60; /*This is the rated angular velocity for the GA12-N20 
    dc motor used in adeept rasp tank when loaded at 6 Volts, the unloaded value is 100RPM */
    double peak_voltage = 8; //Approximate Peak Voltage in the rasptank    

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
