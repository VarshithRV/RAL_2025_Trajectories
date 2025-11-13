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

// class Moveit2Tutorial : public rclcpp::Node{

//     public:
//         Moveit2Tutorial(): Node("moveit_client"){
//             RCLCPP_INFO(this->get_logger(),"starting moveit2 examples");
//         }

//         void move_to_pose(){
//             move_group_interface_ = MoveGroupInterface(shared_from_this(),"ur_manipulator");
//             auto const target_pose = []{
//                 geometry_msgs::msg::Pose pose;
//                 pose.orientation.w=1;
//                 pose.position.x=0.2;
//                 pose.position.y=-0.3;
//                 pose.position.z=0.1;
//                 return pose;
//             }();
//             move_group_interface_.setPoseTarget(target_pose);
//             auto const [success, plan] = [this]{
//                 moveit::planning_interface::MoveGroupInterface::Plan msg;
//                 auto const ok = static_cast<bool>(this->move_group_interface_.plan(msg));
//                 return std::make_pair(ok,msg);
//             }();

//             if(success){
//                 move_group_interface_.execute(plan);
//             }
//             else
//                 RCLCPP_ERROR(this->get_logger(),"PLANNING FAILED");
//         }
    
//     private:
//         MoveGroupInterface move_group_interface_;

// };

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("moveit_example");
    RCLCPP_INFO(node->get_logger(),"Started the tutorials node");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
}