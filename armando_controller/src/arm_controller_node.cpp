#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <vector>
#include <string>
#include <chrono>
#include "rclcpp/parameter.hpp"

using std::placeholders::_1;

class ArmandoController : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    ArmandoController() : Node("arm_controller_node")
    {
        // 4.b Subscriber per gli stati dei giunti
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&ArmandoController::joint_state_callback, this, _1));


        RCLCPP_INFO(this->get_logger(), "Armando Controller Node started.");


        // 4.c Publisher per i comandi di posizione
        position_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "position_controller/commands", 10);

        // 4.d Publisher per i comandi di traiettoria
        trajectory_command_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "joint_trajectory_controller/joint_trajectory", 10);

        //Definizione di 4 pose distinte per il braccio
        position_commands_ = {
            {0.0, 0.0, 0.0, 0.0},
            {0.1, 0.2, 0.0, -0.1},
            {-0.1, -0.2, 0.3, 0.2},
            {0.5, 0.0, -0.5, 0.1}
        };
        cmd_index_ = 0;

        // Definizione delle velocità corrispondenti per ogni posa
        velocity_commands_ = {
            {0.0, 0.0, 0.0, 0.0},
            {0.7, 0.7, 1.0, -0.5},
            {-0.95, -0.45, 0.1, 0.1},
            {0.2, 0.0, -0.2, 0.65}
        };

        // Dichiarazione e lettura del parametro "publisher_type"
        this->declare_parameter<std::string>("publisher_type", "position");
        publisher_type_ = this->get_parameter("publisher_type").as_string();

        RCLCPP_INFO(this->get_logger(), "Using %s publisher", publisher_type_.c_str());


        // Action per i comandi di traiettoria
        trajectory_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "joint_trajectory_controller/follow_joint_trajectory");
        
        // Timer per la pubblicazione dei comandi ogni 5 secondi
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5000),
            std::bind(&ArmandoController::publish_commands, this));

    }

private:

    // Log degli joint states 
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "--- Joint States ---");
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (i < msg->position.size()) {
                RCLCPP_INFO(this->get_logger(), "Joint %s: Position = %f", 
                            msg->name[i].c_str(), msg->position[i]);
            }
        }
    }

    // Pubblicazione dei comandi di posizione
    void publish_joint_commands()
    {
        if (!position_command_publisher_) {
            return;
        }

        const std::vector<double> &cmd = position_commands_[cmd_index_];

        std_msgs::msg::Float64MultiArray msg;
        msg.data = cmd;

        position_command_publisher_->publish(msg);

        cmd_index_ = (cmd_index_ + 1) % position_commands_.size();
    }

    // Pubblicazione dei comandi di traiettoria
    void publish_joint_trajectory()
    {
        if (!trajectory_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Trajectory action server not available!");
            return;
        }

        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names = {"j0", "j1", "j2", "j3"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = position_commands_[cmd_index_];
        point.velocities = velocity_commands_[cmd_index_];
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);
        goal_msg.trajectory.points.push_back(point);

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.result_callback = [](const GoalHandleFollowJointTrajectory::WrappedResult & result) {
           
        };

        trajectory_action_client_->async_send_goal(goal_msg, send_goal_options);

        cmd_index_ = (cmd_index_ + 1) % position_commands_.size();
    }

    // Selezione del tipo di controllo in base al parametro
    void publish_commands()
    {
        if (publisher_type_ == "position") {
            publish_joint_commands();
        } else if (publisher_type_ == "trajectory") {
            publish_joint_trajectory();
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown publisher type: %s", publisher_type_.c_str());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_command_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_action_client_;

    std::vector<std::vector<double>> position_commands_;
    std::vector<std::vector<double>> velocity_commands_;
    size_t cmd_index_;
    std::string publisher_type_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmandoController>());
    rclcpp::shutdown();
    return 0;
}