#include "../include/circle.hpp"



int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "trajectory");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;

    // Create PlanningOptions
    MoveitPlanning::PlanningOptions planning_options =
        MoveitPlanning::PlanningOptions();
    planning_options.num_attempts = 10;
    planning_options.allow_replanning = true;
    planning_options.set_planning_time = 30.0;
    planning_options.goal_position_tolerance = 0.01;
    planning_options.goal_orientation_tolerance = 0.01;
    planning_options.goal_joint_tolerance = 0.01;
    planning_options.velocity_scaling_factor = 0.1;
    planning_options.acceleration_scaling_factor = 0.1;

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

    geometry_msgs::Pose pose1;
    pose1.position.x = 0;
    pose1.position.y = 0.3;
    pose1.position.z = 1.45;
    pose1.orientation.x = -0.707;
    pose1.orientation.y = 0.0;
    pose1.orientation.z = 0.0;
    pose1.orientation.w = 0.707;

    std::string reference_frame = "base_link_inertia";

    // Create instance of Plan
    moveit::planning_interface::MoveGroupInterface::Plan plan_pose1;
    std::string end_effector_name = "moveit";
    bool plan_success;
    plan_success = ArmController::planToPoseTarget(planning_options,arm_move_group,pose1,reference_frame,plan_pose1,end_effector_name);

    int flag = 0;

    if(plan_success){
        ROS_INFO("pose 1 plan succeeded");
        ros::Duration(1.0).sleep();
        flag = 1;

        arm_move_group.execute(plan_pose1);
    }
    else{
        ROS_INFO("plan_failed");
        flag = 0;
    }

    if(flag){
        
        const double PI = std::atan(1.0)*4;
        std::vector< geometry_msgs::Pose > vect;
        double radius = 0.1;

        double Xc = 0;
        double Zc = 1.35;

        for(double angle = 0; angle <= 2 * PI; angle+=0.1){
            geometry_msgs::Pose poses = arm_move_group.getCurrentPose().pose;
            poses.position.x = Xc + radius * sin(angle);
            poses.position.z = Zc + radius * cos(angle);
            vect.push_back(poses);
        }

        while (!vect.empty()) {
            geometry_msgs::Pose start_pose1 = arm_move_group.getCurrentPose().pose;
            geometry_msgs::Pose nextpose = vect.back();
            vect.pop_back();
            ArmController::planCartesianPath(start_pose1, nextpose, arm_move_group);
        }

    } 
    
}