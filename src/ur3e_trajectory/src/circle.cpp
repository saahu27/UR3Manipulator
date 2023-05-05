#include "../include/circle.hpp"



int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "trajectory");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;

    double x_c;
    double y_c;
    double z_c;
    double radius;
    std::string plane;
    n.getParam("/circle/radius", radius);
    n.getParam("/circle/x_c", x_c);
    n.getParam("/circle/y_c", y_c);
    n.getParam("/circle/z_c", z_c);
    n.getParam("/circle/plane", plane);

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

    int flag = 0;

    geometry_msgs::Pose pose1;
    if (plane == "xy" || plane == "yx"){
        pose1.position.x = x_c;
        pose1.position.y = y_c + radius;
        pose1.position.z = z_c;
        if (z_c <= 1.3){
            pose1.orientation.x = 0.707;
            pose1.orientation.y = 0.707;
            pose1.orientation.z = 0.0;
            pose1.orientation.w = 0.0;
        }
        else{
            pose1.orientation.x = 0.0;
            pose1.orientation.y = 0.0;
            pose1.orientation.z = 0.0;
            pose1.orientation.w = 1.0;
        }
        
    }
    else if (plane == "xz" || plane == "zx"){
        pose1.position.x = x_c;
        pose1.position.y = y_c;
        pose1.position.z = z_c + radius;
        if (y_c >= 0.0){
            pose1.orientation.x = -0.707;
            pose1.orientation.y = 0.0;
            pose1.orientation.z = 0.0;
            pose1.orientation.w = 0.707;
        }
        else{
            pose1.orientation.x = 0.0;
            pose1.orientation.y = 0.707;
            pose1.orientation.z = -0.707;
            pose1.orientation.w = 0.0;
        }
    }
    else if (plane == "yz" || plane == "zy"){
        pose1.position.x = x_c;
        pose1.position.y = y_c;
        pose1.position.z = z_c + radius;
        if (x_c >= 0.0){
            pose1.orientation.x = -0.5;
            pose1.orientation.y = 0.5;
            pose1.orientation.z = -0.5;
            pose1.orientation.w = 0.5;
        }
        else{
            pose1.orientation.x = -0.5;
            pose1.orientation.y = -0.5;
            pose1.orientation.z = 0.5;
            pose1.orientation.w = 0.5;
        }
    }

    // Create instance of Plan
    moveit::planning_interface::MoveGroupInterface::Plan plan_pose1;

    std::string reference_frame = "base_link";

    bool plan_success;
    plan_success = ArmController::planToPoseTarget(planning_options,arm_move_group,pose1,reference_frame,plan_pose1);

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
        std::vector< geometry_msgs::Pose > waypoints;

        geometry_msgs::Pose start_pose = arm_move_group.getCurrentPose().pose;

        if (plane == "xy" || plane == "yx"){
            for(double angle = 0; angle <= 2 * PI; angle+=0.1){
                geometry_msgs::Pose next_pose = start_pose;
                next_pose.position.x = x_c + radius * sin(angle);
                next_pose.position.y = y_c + radius * cos(angle);
                waypoints.push_back(next_pose);
            }
        }
        else if (plane == "xz" || plane == "zx"){
            for(double angle = 0; angle <= 2 * PI; angle+=0.1){
                geometry_msgs::Pose next_pose = start_pose;
                next_pose.position.x = x_c + radius * sin(angle);
                next_pose.position.z = z_c + radius * cos(angle);
                waypoints.push_back(next_pose);
            }
        }
        else if (plane == "yz" || plane == "zy"){
            for(double angle = 0; angle <= 2 * PI; angle+=0.1){
                geometry_msgs::Pose next_pose = start_pose;
                next_pose.position.y = y_c + radius * sin(angle);
                next_pose.position.z = z_c + radius * cos(angle);
                waypoints.push_back(next_pose);
            }
        }

        moveit_msgs::RobotTrajectory trajectory;
        trajectory = ArmController::planCartesianPath(start_pose, waypoints, reference_frame, arm_move_group);

        arm_move_group.execute(trajectory);

    } 
    
}