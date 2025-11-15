// cpp example for moveit2
/*
- [x] IK+path planning for tool position target
- [x] Joint state goal movement
- [ ] IK solvers
- [ ] Path planning and trajectory retiming
- [ ] Scene Management
- [ ] Servoing
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
#include "example_interfaces/srv/trigger.hpp"

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

class MoveitExamples{
    public: 
        MoveitExamples(rclcpp::Node::SharedPtr &node){
            node_ = node;
            move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur_manipulator");
            bool ok = move_group_interface_->startStateMonitor(5.0);
            if (!ok)
                RCLCPP_WARN(node_->get_logger(), "State monitor did not receive joint states within 5 seconds");
            print_state_server_ = node_->create_service<example_interfaces::srv::Trigger>("print_robot_state",std::bind(&MoveitExamples::print_state,this,std::placeholders::_1,std::placeholders::_2));
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
            move_group_interface_->clearPoseTargets();
        }

        void move_to_joint_state(const std::vector<double> group_variable_values){
            move_group_interface_->setJointValueTarget(group_variable_values);
            auto const [success, plan] = [this]{
                moveit::planning_interface::MoveGroupInterface::Plan msg;
                auto const ok=static_cast<bool>(this->move_group_interface_->plan(msg));
                return std::make_pair(ok,msg);
            }();
            if(success){
                move_group_interface_->execute(plan);
                RCLCPP_INFO(node_->get_logger(),"Finished execution");
            }
            else
                RCLCPP_ERROR(node_->get_logger(),"Planning Failed");
        }

        void print_state(const example_interfaces::srv::Trigger_Request::SharedPtr request, example_interfaces::srv::Trigger_Response::SharedPtr response){ // not working, stupid timer issue
            auto current_state = move_group_interface_->getCurrentState();
            auto current_pose = move_group_interface_->getCurrentPose();
            auto current_joint_values = move_group_interface_->getCurrentJointValues();
            auto print_pose = [this,current_state, current_joint_values, current_pose](){
                double x = current_pose.pose.position.x;
                double y = current_pose.pose.position.y;
                double z = current_pose.pose.position.z;
                double qx = current_pose.pose.orientation.x;
                double qy = current_pose.pose.orientation.y;
                double qz = current_pose.pose.orientation.z;
                double qw = current_pose.pose.orientation.w;
                RCLCPP_INFO(this->node_->get_logger(),"X : %f",x);
                RCLCPP_INFO(this->node_->get_logger(),"Y : %f",y);
                RCLCPP_INFO(this->node_->get_logger(),"Z : %f",z);
                RCLCPP_INFO(this->node_->get_logger(),"Qx : %f",qx);
                RCLCPP_INFO(this->node_->get_logger(),"Qy : %f",qy);
                RCLCPP_INFO(this->node_->get_logger(),"Qz : %f",qz);
                RCLCPP_INFO(this->node_->get_logger(),"Qw : %f",qw);
                std::string message;
                for(std::size_t i=0; i<current_joint_values.size(); i++)
                    message = "Joint " + std::to_string(i) + ": " + std::to_string(current_joint_values[i]);
                message += "     X : " + std::to_string(x) + " Y : " + std::to_string(y) + " Z : " + std::to_string(z);
                return message;
            };
            response->message = print_pose();
            response->success = true;
        }

    private:
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr print_state_server_;
};

int main(int argc, char* argv[]){

    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("moveit_example");
    RCLCPP_INFO(node->get_logger(),"Started the tutorials node");
    
    // auto const target_pose = []{
        //     geometry_msgs::msg::Pose pose;
        //     pose.orientation.x=0;
        //     pose.orientation.y=1;
        //     pose.orientation.z=0;
        //     pose.orientation.w=0;
        //     pose.position.x=-0.370;
        //     pose.position.y=-0.200;
        //     pose.position.z=0.409;
        //     return pose;
        // }();
        
        auto moveit_example = std::make_shared<MoveitExamples>(node);
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        std::thread spinner([&executor]() { executor.spin();});
        
        // rclcpp::executors::SingleThreadedExecutor executor;
        // executor.add_node(node);
        // std::thread executor_thread([&executor](){
        //     executor.spin();
        // });

    rclcpp::shutdown();
}