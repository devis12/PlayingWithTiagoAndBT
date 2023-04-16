#include <ros/console.h>

// CUSTOM NODES INCLUDES
#include "tiago_basic_bt/bt_actions/tiago_move_circle.hpp"

#define ODOMETRY_TOPIC_NAME "/mobile_base_controller/odom"
#define CMD_VEL_TOPIC_NAME "mobile_base_controller/cmd_vel"

#define VELOCITY_PORT_NAME "velocity"
#define ROUNDS_PORT_NAME "rounds"

#define EPS 0.1


TiagoMoveCircle::TiagoMoveCircle(const std::string& name, const BT::NodeConfiguration& config)
: StatefulActionNode(name, config)
{}

void TiagoMoveCircle::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_pose_ = msg->pose.pose;
}

void TiagoMoveCircle::rosInit(std::shared_ptr<ros::NodeHandle> n)
{
    n_ptr_ = n;
    odometry_sub_ = n_ptr_->subscribe(ODOMETRY_TOPIC_NAME, 100, &TiagoMoveCircle::odometryCallback, this);
}

BT::PortsList TiagoMoveCircle::providedPorts()
{
    return{ BT::InputPort<int>(VELOCITY_PORT_NAME), BT::InputPort<int>(ROUNDS_PORT_NAME)};
}
    
BT::NodeStatus TiagoMoveCircle::onStart()
{
    if(!n_ptr_) return BT::NodeStatus::FAILURE;

    const auto rounds = getInput<int>(ROUNDS_PORT_NAME);
    const auto velocity = getInput<int>(VELOCITY_PORT_NAME);
    if(!rounds || !velocity) return BT::NodeStatus::FAILURE;

    rounds_ = rounds.value();
    velocity_ = velocity.value();

    init_pose_ = current_pose_;
    rounds_performed_ = 0;
    wait_eps_dist_ = true;

    cmd_vel_msg_.linear.x = velocity_;
    cmd_vel_msg_.angular.z = velocity_;

    cmd_vel_pub_ = n_ptr_->advertise<geometry_msgs::Twist>(CMD_VEL_TOPIC_NAME, 100);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TiagoMoveCircle::onRunning()
{   
    cmd_vel_pub_.publish(cmd_vel_msg_);
    if(!wait_eps_dist_)
    {
        if(std::abs(current_pose_.position.x - init_pose_.position.x) < EPS && std::abs(current_pose_.position.y - init_pose_.position.y) < EPS)
        {
            rounds_performed_++;
            ROS_INFO("Perform %d rounds ", rounds_performed_);
            if(rounds_performed_ == rounds_)
                return BT::NodeStatus::SUCCESS;
            wait_eps_dist_ = true;
        }
    }
    else
    {
        if(std::abs(current_pose_.position.x - init_pose_.position.x) > EPS || std::abs(current_pose_.position.y - init_pose_.position.y) > EPS)
        {
            wait_eps_dist_ = false;
        }
    }

    return BT::NodeStatus::RUNNING;
}

void TiagoMoveCircle::onHalted()
{
    rounds_ = velocity_ = 0;
    cmd_vel_pub_.shutdown();
}