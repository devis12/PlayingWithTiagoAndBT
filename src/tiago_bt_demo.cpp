#include <memory>

// BT INCLUDES
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// ROS INCLUDES
#include "ros/ros.h"
#include "ros/package.h"

// STD INCLUDES
#include <vector>
#include <string>

// IOSTREAM INCLUDES
#include <fstream>

// CUSTOM NODES INCLUDES
#include "tiago_basic_bt/bt_actions/tiago_move_circle.hpp"
#include "tiago_basic_bt/bt_actions/tiago_play_motion.hpp"

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "tiago_bt_demo");
    std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();

    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Initialize resources

    // Load xml BT file
    std::string default_path = ros::package::getPath("tiago_basic_bt"); // retrieve fs path for tiago_basic_bt package
    std::string default_local_path = "/trees/tiago_basic.xml"; // relative path to the tree xml

    default_path += default_local_path;
    std::string path = nh->param("bt_xml_path", default_path); //If param ok, the path is completely defined (package+local path), if not default path is calculated
    ROS_INFO_STREAM("BT XML File path: " << path);
    std::ifstream ifs(path);
    if (!ifs.is_open())
    {
        ROS_ERROR("File not open");
        return 0;
    }
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    ifs.close();
    std::string xml_text = buffer.str();


    // We use the BehaviorTreeFactory to register our custom nodes
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<TiagoMoveCircle>("MoveCircle");
    factory.registerNodeType<TiagoPlayMotion>("PlayMotion");
    std::cout << "BT_CONTROL: All nodes registered" << std::endl;

    // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
    // The currently supported format is XML.
    // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed
    auto tree = factory.createTreeFromText(xml_text);
    
    // visitor will initialize the instances of 
    auto visitor = [nh](BT::TreeNode* node)
    {
        if (auto mv_circle_node = dynamic_cast<TiagoMoveCircle*>(node))
        {
            mv_circle_node->rosInit(nh);
        }
    };

    // Apply the visitor to ALL the nodes of the tree
    applyRecursiveVisitor(tree.rootNode(), visitor);
    
    // This logger prints state changes on console
    BT::StdCoutLogger logger_cout(tree);

    // This logger stores the execution time of each node
    BT::MinitraceLogger logger_minitrace(tree, "tiago_basic_demo.json");

    // This logger saves state changes on file
    BT::FileLogger logger_file(tree, "tiago_basic_demo.fbl");

    // This logger publish status changes using ZeroMQ. Used by Groot
    BT::PublisherZMQ publisher_zmq(tree);
    std::cout << "BT COMMUNICATION: ZMQ found" << std::endl;

    BT::NodeStatus status =  tree.tickRootWhileRunning(std::chrono::milliseconds{256}); // tick the root each 256ms: because Devis said to do so

    return 0;
}