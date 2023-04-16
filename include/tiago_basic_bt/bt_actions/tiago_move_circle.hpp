#ifndef TIAGO_MOVE_CIRCLE_ACTION
#define TIAGO_MOVE_CIRCLE_ACTION

// C++ standard headers
#include <cstdlib>
#include <string>
#include <memory>

// ROS headers
#include <ros/ros.h>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"

/**/
#include "behaviortree_cpp_v3/action_node.h"

class TiagoMoveCircle : public BT::StatefulActionNode
{

public:

     // Any TreeNode with ports must have a constructor with this signature
    TiagoMoveCircle(const std::string& name, const BT::NodeConfiguration& config);

    void rosInit(std::shared_ptr<ros::NodeHandle> n);

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
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // ROS stuff.
    std::shared_ptr<ros::NodeHandle> n_ptr_;
    ros::Subscriber odometry_sub_;
    ros::Publisher cmd_vel_pub_;

    // Amount of iteration rounds performed by the robot before concluding the action
    int rounds_;
    // Linear velocity x and angular velocity z of the robot running in circle like a drunk motherfucker
    bool velocity_;

    int rounds_performed_;
    bool wait_eps_dist_;

    geometry_msgs::Twist cmd_vel_msg_;
    geometry_msgs::Pose current_pose_;
    geometry_msgs::Pose init_pose_;
};
#endif // __TIAGO_MOVE_CIRCLE_ACTION_INCLUDED__