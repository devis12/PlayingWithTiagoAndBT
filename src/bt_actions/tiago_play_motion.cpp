#include <ros/console.h>
// CUSTOM NODES INCLUDES
#include "tiago_basic_bt/bt_actions/tiago_play_motion.hpp"

#define MOTION_PORT_NAME "motion_name"

#define PLAY_MOTION_TOPIC "play_motion"


TiagoPlayMotion::TiagoPlayMotion(const std::string& name, const BT::NodeConfiguration& config)
: StatefulActionNode(name, config), client_(PLAY_MOTION_TOPIC, true)
{}


BT::PortsList TiagoPlayMotion::providedPorts()
{
    return{ BT::InputPort<std::string>(MOTION_PORT_NAME) };
}
    

BT::NodeStatus TiagoPlayMotion::onStart()
{
    waiting_server_connect_ = true;
    waiting_final_result_ = false;
    return BT::NodeStatus::RUNNING;
}


BT::NodeStatus TiagoPlayMotion::onRunning()
{
    ROS_INFO("TiagoPlayMotion::onRunning()");

    // wait for the action server to start
    if(waiting_server_connect_ && !client_.waitForServer(ros::Duration(0.5)))
    {
        ROS_INFO("Waiting for action %s server to start.", PLAY_MOTION_TOPIC);
    }
    else if(waiting_server_connect_ && client_.isServerConnected())
        waiting_server_connect_ = false;// not in waiting status anymore

    if(!waiting_server_connect_ && !waiting_final_result_)
    {
        const auto motion_name = getInput<std::string>(MOTION_PORT_NAME);
        if(!motion_name) return BT::NodeStatus::FAILURE;

        ROS_INFO_STREAM("Playing motion " << motion_name.value());
        // Connected to server, but still has to make the first request
        play_motion_msgs::PlayMotionActionGoal goal;
        goal.goal.motion_name = motion_name.value();
        client_.sendGoal(goal.goal);

        // make the first request, now let's just wait for the result
        waiting_final_result_ = true;
    }

    if(waiting_final_result_ && client_.getState().isDone())
    {
        auto result_ptr = client_.getResult();
        if(result_ptr->error_code == result_ptr->SUCCEEDED)
            return BT::NodeStatus::SUCCESS;
        else
            return BT::NodeStatus::FAILURE;

    }

    return BT::NodeStatus::RUNNING;
}

void TiagoPlayMotion::onHalted()
{
    client_.cancelAllGoals();
    waiting_server_connect_ = false;
    waiting_final_result_ = false;
}
