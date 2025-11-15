#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node=rclcpp::Node::make_shared("kinematics_and_inverse_kinematics",node_options);
    const auto& LOGGER = node->get_logger();
    // need to under stand the & operator better
    
    //robotmodelloader will look up the robot description
    robot_model_loader::RobotModelLoader robot_model_loader(node);
    // kinematic_model will contain the symbolic model of the robot in the program ig
    const moveit::core::RobotModelPtr& kinematic_model=robot_model_loader.getModel();
    RCLCPP_INFO(LOGGER,"Model frame : %s", kinematic_model->getModelFrame().c_str());

    // what is kinematic state? what is robot model ptr and robot state ptr? what is model vs state
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    // kinematic_state->setToDefaultValues();
    // Joint model Group?
    const moveit::core::JointModelGroup* joint_model_group=kinematic_model->getJointModelGroup("ur_manipulator");
    // from joint_model_group, get the joint names ig?
    const std::vector<std::string>& joint_names=joint_model_group->getVariableNames();
    // get joint values for the joint group
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group,joint_values);
    for(std::size_t i=0;i<joint_names.size();i++){
        RCLCPP_INFO(LOGGER,"Joint %s: %f", joint_names[i].c_str(),joint_values[i]);
    }

    // robot_model_loader -> robot_model_ptr (kinematic_model) -> robot_state_ptr (kinematic_state)
    // (superset of all groups) ->  joint_model_group(specific to each joint group)
    // -> get joint values, etc
    
    kinematic_state->setToRandomPositions(joint_model_group);
    // forward kinematics
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("wrist_3_link");

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(0.001, 0.291, 0.791);
    T.linear() = Eigen::Quaterniond(0.707, -0.707, 0.001, 0.001).toRotationMatrix();

    RCLCPP_INFO_STREAM(LOGGER,"Translation : \n" << T.translation()<<'\n');
    RCLCPP_INFO_STREAM(LOGGER,"Rotation: \n"<< T.rotation() <<"\n");

    // Inverse Kinematics
    double timeout=0.1;
    for(std::size_t i=0; i<100; i++){
        bool found_ik=kinematic_state->setFromIK(joint_model_group,end_effector_state,timeout);
        // print IK solution? solutions?
        if(found_ik){
            kinematic_state->copyJointGroupPositions(joint_model_group,joint_values);
            for(std::size_t i=0; i<joint_names.size();i++){
                RCLCPP_INFO(LOGGER,"Joint %s : %f", joint_names[i].c_str(),joint_values[i]);
            }
        }
        else{
            RCLCPP_INFO(LOGGER,"Did not find IK solution");
        }
        RCLCPP_INFO(LOGGER,"###################");
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}