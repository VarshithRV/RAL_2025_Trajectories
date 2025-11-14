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

class MoveitExamples{
    public: 
        MoveitExamples(rclcpp::Node::SharedPtr &node){
            node_ = node;
            move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur_manipulator");
        }

        void move_to_pose(const geometry_msgs::msg::Pose &pose){
            move_group_interface_->setPoseTarget(pose);
            auto const [success, plan] = [this]{
                moveit::planning_interface::MoveGroupInterface::Plan msg;
                auto const ok=static_cast<bool>(this->move_group_interface_->plan(msg));
                return std::make_pair(ok,msg);
            }();
            if(success)
                move_group_interface_->execute(plan);
            else
                RCLCPP_ERROR(node_->get_logger(),"Planning Failed");
        }

    private:
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
        rclcpp::Node::SharedPtr node_;
};

int main(int argc, char* argv[]){

    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("moveit_example");
    auto moveit_example = std::make_shared<MoveitExamples>(node);
    RCLCPP_INFO(node->get_logger(),"Started the tutorials node");

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

    moveit_example->move_to_pose(target_pose);

    rclcpp::spin(node);
    rclcpp::shutdown();
}