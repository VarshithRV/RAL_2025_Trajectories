// cpp example for moveit2
/*
1. IK+path planning for tool position target
2. Joint state goal movement
3. IK solvers
4. Path planning and trajectory retiming
5. Scene Management
6. Servoing
*/

#include <memory>
#include <functional>
#include <string>
#include <chrono>
#include <cstdlib>
#include <thread>
#include "moveit/move_group_interface/move_group_interface.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;


int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("moveit_example");
    RCLCPP_INFO(node->get_logger(),"Started the tutorials node");
    auto move_group_interface = MoveGroupInterface(node,"ur_manipulator");
    auto const target_pose = []{
        geometry_msgs::msg::Pose pose;
        pose.orientation.x=0;
        pose.orientation.y=1;
        pose.orientation.z=0;
        pose.orientation.w=0;
        pose.position.x=-0.370;
        pose.position.y=-0.200;
        pose.position.z=0.409;
        return pose;
    }();

    move_group_interface.setPoseTarget(target_pose);
    auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok,msg);
    }();

    if(success)
        move_group_interface.execute(plan);
    else
        RCLCPP_ERROR(node->get_logger(),"Planning Failed");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
}