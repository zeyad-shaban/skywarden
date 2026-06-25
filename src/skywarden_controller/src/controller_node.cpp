#include "rclcpp/rclcpp.hpp"
#include "skywarden_interfaces/msg/target_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

enum class SystemState
{
    IDLE,
    LOCKING,
    SHOOT,
};

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode() : Node("controller_node")
    {
        RCLCPP_INFO(this->get_logger(), "controller_node started!");

        this->declare_parameter("Kyaw", 0.2);
        this->declare_parameter("Kpitch", 0.2);

        this->get_parameter("Kyaw", Kyaw);
        this->get_parameter("Kpitch", Kpitch);

        target_state_sub_ = this->create_subscription<skywarden_interfaces::msg::TargetState>("/target_state", 10, std::bind(&ControllerNode::target_state_callback, this, _1));

        this->joint_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_trajectory_controller/commands", 10);

        this->main_timer_ = this->create_wall_timer(100ms, std::bind(&ControllerNode::main_loop, this));
        this->state = SystemState::IDLE;
    }

private:
    void main_loop()
    {
        if (state == SystemState::IDLE)
            this->idle();
        else if (state == SystemState::LOCKING)
            this->locking();
    }

    void idle()
    {
        if (this->target_state_msg == nullptr || !this->target_state_msg->is_target_visible)
            return;

        RCLCPP_INFO(this->get_logger(), "Switching to locking state");
        this->state = SystemState::LOCKING;
    }

    void locking()
    {
        if ((this->now() - last_msg_time_).seconds() > 0.2)
        {
            RCLCPP_WARN(this->get_logger(), "Target signal lost! Returning to IDLE.");
            this->state = SystemState::IDLE;
            this->target_state_msg = nullptr;

            auto msg = std_msgs::msg::Float64MultiArray();
            msg.data = {0.0, 0.0};
            this->joint_vel_publisher_->publish(msg);
            return;
        }
        
        float err_x = target_state_msg->norm_err_x;
        float err_y = target_state_msg->norm_err_y;

        float cmd_yaw = this->Kyaw * -err_x;
        float cmd_pitch = this->Kpitch * err_y;

        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {cmd_yaw, cmd_pitch};
        this->joint_vel_publisher_->publish(msg);
    }

    void target_state_callback(const skywarden_interfaces::msg::TargetState::SharedPtr msg)
    {
        this->last_msg_time_ = this->now();
        this->target_state_msg = msg;
    }

    rclcpp::TimerBase::SharedPtr main_timer_;
    rclcpp::Subscription<skywarden_interfaces::msg::TargetState>::SharedPtr target_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_vel_publisher_;

    skywarden_interfaces::msg::TargetState::SharedPtr target_state_msg;
    SystemState state;

    float Kyaw;
    float Kpitch;
    rclcpp::Time last_msg_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}