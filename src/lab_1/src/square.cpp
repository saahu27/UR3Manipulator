#include "../include/square.hpp"

int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "square");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;

    double x_c;
    double y_c;
    double z_c;
    double side;
    std::string plane;
    n.getParam("/square/side", side);
    n.getParam("/square/x_c", x_c);
    n.getParam("/square/y_c", y_c);
    n.getParam("/square/z_c", z_c);
    n.getParam("/square/plane", plane);


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
    std::vector<double> pos_vect;

    geometry_msgs::Pose pose1;
    if (plane == "xy" || plane == "yx"){
        pose1.position.x = x_c - side/2;
        pose1.position.y = y_c + side/2;
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
        pos_vect.push_back(z_c);
        pos_vect.push_back(y_c + side/2);
        pos_vect.push_back(x_c - side/2);
        pos_vect.push_back(z_c);
        pos_vect.push_back(y_c - side/2);
        pos_vect.push_back(x_c - side/2);
        pos_vect.push_back(z_c);
        pos_vect.push_back(y_c - side/2);
        pos_vect.push_back(x_c + side/2);
        pos_vect.push_back(z_c);
        pos_vect.push_back(y_c + side/2);
        pos_vect.push_back(x_c + side/2);
        
    }
    else if (plane == "xz" || plane == "zx"){
        pose1.position.x = x_c - side/2;
        pose1.position.y = y_c;
        pose1.position.z = z_c + side/2;
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
        pos_vect.push_back(z_c + side/2);
        pos_vect.push_back(y_c);
        pos_vect.push_back(x_c - side/2);
        pos_vect.push_back(z_c - side/2);
        pos_vect.push_back(y_c);
        pos_vect.push_back(x_c - side/2);
        pos_vect.push_back(z_c - side/2);
        pos_vect.push_back(y_c);
        pos_vect.push_back(x_c + side/2);
        pos_vect.push_back(z_c + side/2);
        pos_vect.push_back(y_c);
        pos_vect.push_back(x_c + side/2);
    }
    else if (plane == "yz" || plane == "zy"){
        pose1.position.x = x_c;
        pose1.position.y = y_c + side/2;
        pose1.position.z = z_c + side/2;
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
        pos_vect.push_back(z_c + side/2);
        pos_vect.push_back(y_c + side/2);
        pos_vect.push_back(x_c);
        pos_vect.push_back(z_c - side/2);
        pos_vect.push_back(y_c + side/2);
        pos_vect.push_back(x_c);
        pos_vect.push_back(z_c - side/2);
        pos_vect.push_back(y_c - side/2);
        pos_vect.push_back(x_c);
        pos_vect.push_back(z_c + side/2);
        pos_vect.push_back(y_c - side/2);
        pos_vect.push_back(x_c);
    }


    // pose1.position.x = 0.15;
    // pose1.position.y = 0.3;
    // pose1.position.z = 1.1;
    // pose1.orientation.x = 0.707;
    // pose1.orientation.y = 0.707;
    // pose1.orientation.z = 0.0;
    // pose1.orientation.w = 0.0;
    
    std::string reference_frame = "base_link_inertia";

    // Create instance of Plan
    moveit::planning_interface::MoveGroupInterface::Plan plan_pose1;
    std::string end_effector_name = "moveit";
    bool plan_success;
    plan_success = ArmController::planToPoseTarget(planning_options,arm_move_group,pose1,reference_frame,plan_pose1,end_effector_name);

    if(plan_success){
        ROS_INFO("pose 1 plan succeeded");
        ros::Duration(1.0).sleep();
        flag = 1;

        arm_move_group.execute(plan_pose1);
        geometry_msgs::Pose posep = arm_move_group.getCurrentPose().pose;
        ROS_INFO("plan_pose1 done %f %f %f", posep.position.x,posep.position.x,posep.position.z);

    }
    else{
        ROS_INFO("plan_failed");
        flag = 0;
    }

    if(flag){
        
        geometry_msgs::Pose start_pose1 = arm_move_group.getCurrentPose().pose;
        geometry_msgs::Pose end_pose1 = start_pose1;
        // end_pose1.position.x = 0.35;
        // end_pose1.position.y = 0.3;
        // end_pose1.position.z = 1.15;
        end_pose1.position.x = pos_vect.back();
        pos_vect.pop_back();
        end_pose1.position.y = pos_vect.back();
        pos_vect.pop_back();
        end_pose1.position.z = pos_vect.back();
        pos_vect.pop_back();


        ArmController::planCartesianPath(start_pose1, end_pose1, arm_move_group);

        geometry_msgs::Pose start_pose2 = arm_move_group.getCurrentPose().pose;
        geometry_msgs::Pose end_pose2 = start_pose2;
        // end_pose2.position.x = 0.35;
        // end_pose2.position.y = 0.1;
        // end_pose2.position.z = 1.15;
        end_pose2.position.x = pos_vect.back();
        pos_vect.pop_back();
        end_pose2.position.y = pos_vect.back();
        pos_vect.pop_back();
        end_pose2.position.z = pos_vect.back();
        pos_vect.pop_back();

        ArmController::planCartesianPath(start_pose2, end_pose2, arm_move_group);
        
        geometry_msgs::Pose start_pose3 = arm_move_group.getCurrentPose().pose;
        geometry_msgs::Pose end_pose3 = start_pose3;
        // end_pose3.position.x = 0.15;
        // end_pose3.position.y = 0.1;
        // end_pose3.position.z = 1.15;
        end_pose3.position.x = pos_vect.back();
        pos_vect.pop_back();
        end_pose3.position.y = pos_vect.back();
        pos_vect.pop_back();
        end_pose3.position.z = pos_vect.back();
        pos_vect.pop_back();

        ArmController::planCartesianPath(start_pose3, end_pose3, arm_move_group);

        geometry_msgs::Pose start_pose4 = arm_move_group.getCurrentPose().pose;
        geometry_msgs::Pose end_pose4 = start_pose4;
        // end_pose4.position.x = 0.15;
        // end_pose4.position.y = 0.3;
        // end_pose4.position.z = 1.15;
        end_pose4.position.x = pos_vect.back();
        pos_vect.pop_back();
        end_pose4.position.y = pos_vect.back();
        pos_vect.pop_back();
        end_pose4.position.z = pos_vect.back();
        pos_vect.pop_back();
        
        ArmController::planCartesianPath(start_pose4, end_pose4, arm_move_group);

    } 
    
}