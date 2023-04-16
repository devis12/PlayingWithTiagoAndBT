#ifndef TIAGO_PLAY_MOTION_ACTION
#define TIAGO_PLAY_MOTION_ACTION

// C++ standard headers
#include <cstdlib>
#include <string>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// PAL includes
#include <play_motion_msgs/PlayMotionAction.h>


/**/
#include "behaviortree_cpp_v3/action_node.h"

class TiagoPlayMotion : public BT::StatefulActionNode
{

public:

     // Any TreeNode with ports must have a constructor with this signature
    TiagoPlayMotion(const std::string& name, const BT::NodeConfiguration& config);

     // It is mandatory to define this static method.
    static BT::PortsList providedPorts();
    
    /// method to be called at the beginning.
    /// If it returns RUNNING, this becomes an asychronous node.
    BT::NodeStatus onStart() override;

    /// method invoked by a RUNNING action.
    BT::NodeStatus onRunning() override;

    /// when the method halt() is called and the action is RUNNING, this method is invoked.
    /// This is a convenient place todo a cleanup, if needed.
    void onHalted() override;

private:
    // Actionlib stuff.
    actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction>           client_;

    // If true, waiting for the server to be up
    bool waiting_server_connect_;
    // If true, waiting for the action goal final result
    bool waiting_final_result_;
};
#endif // __PLAN_MOTION_H_INCLUDED__