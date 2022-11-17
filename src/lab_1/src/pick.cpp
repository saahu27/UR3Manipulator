#include "../include/square.hpp"

int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "square");
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

    int flag = 0;

    std::string planning_frame = arm_move_group.getPlanningFrame();

    geometry_msgs::Pose pose1;

    pose1.position.x = 0.3;
    pose1.position.y = 0.3;
    pose1.position.z = 1.1;
    pose1.orientation.x = 0.707;
    pose1.orientation.y = 0.707;
    pose1.orientation.z = 0.0;
    pose1.orientation.w = 0.0;

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
    ArmController::open_gripper(&n);


    geometry_msgs::Pose pose2;
    pose2.position.x = 0.3;
    pose2.position.y = 0.3;
    pose2.position.z = 0.805;
    pose2.orientation.x = 0.707;
    pose2.orientation.y = 0.707;
    pose2.orientation.z = 0.0;
    pose2.orientation.w = 0.0;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(pose2);
    geometry_msgs::Pose start_pose = arm_move_group.getCurrentPose().pose;
    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(start_pose, waypoints, reference_frame, arm_move_group);
    arm_move_group.execute(trajectory);
    ArmController::close_gripper(&n);
    arm_move_group.attachObject("cube");


    geometry_msgs::Pose pose3;

    pose3.position.x = 0.0;
    pose3.position.y = 0.0;
    pose3.position.z = 0.85;
    pose3.orientation.x = 0.707;
    pose3.orientation.y = 0.707;
    pose3.orientation.z = 0.0;
    pose3.orientation.w = 0.0;

    // Create instance of Plan
    moveit::planning_interface::MoveGroupInterface::Plan plan_pose3;

    bool plan_success3;
    plan_success3 = ArmController::planToPoseTarget(planning_options,arm_move_group,pose3,reference_frame,plan_pose3);

    if(plan_success3){
        ROS_INFO("pose 1 plan succeeded");
        ros::Duration(1.0).sleep();
        flag = 1;

        arm_move_group.execute(plan_pose3);

    }
    else{
        ROS_INFO("plan_failed");
        flag = 0;
    }
    ArmController::open_gripper(&n);
}