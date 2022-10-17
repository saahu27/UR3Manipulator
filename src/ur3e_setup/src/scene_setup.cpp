#include "../include/scene_setup.hpp"

int main(int argc, char **argv)
{
    // Setup ROS node
    ros::init(argc, argv, "scene_setup");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;

    double distance;
    double width;
    
    // Getting values from the launch file
    n.getParam("/scene_setup/distance", distance);
    n.getParam("/scene_setup/width", width);

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

    std::string planning_frame = arm_move_group.getPlanningFrame();

    // Table
    moveit_msgs::CollisionObject robot_table_obj;
    std::string table_id = "table";
    shape_msgs::SolidPrimitive table_primitive;
    std::vector<double> table_dim = {1.12, 0.805, 0.766};
    std::string table_type = "BOX";
    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.0;
    table_pose.position.y = 0.3125;
    table_pose.position.z = 0.383;
    ArmController::createCollisionObject(robot_table_obj, table_id, table_primitive, table_pose, table_type, table_dim, planning_frame);

    // // Wall Left
    // moveit_msgs::CollisionObject wall_left_obj;
    // std::string wall_left_id = "wall_left";
    // shape_msgs::SolidPrimitive wall_left_primitive;
    // std::vector<double> wall_left_dim = {0.1, width, 1.80};
    // std::string wall_left_type = "BOX";
    // geometry_msgs::Pose wall_left_pose;
    // wall_left_pose.orientation.w = 1.0;
    // wall_left_pose.position.x = -distance; //-0.5;
    // wall_left_pose.position.y = 0.0;
    // wall_left_pose.position.z = 0.9;
    // ArmController::createCollisionObject(wall_left_obj, wall_left_id, wall_left_primitive, wall_left_pose, wall_left_type , wall_left_dim, planning_frame);

    // moveit_msgs::CollisionObject wall_front_obj;
    // std::string wall_front_id = "wall_front";
    // shape_msgs::SolidPrimitive wall_front_primitive;
    // std::vector<double> wall_front_dim = {width, 0.1, 1.80};
    // std::string wall_front_type = "BOX";
    // geometry_msgs::Pose wall_front_pose;
    // wall_front_pose.orientation.w = 1.0;
    // wall_front_pose.position.x = 0.0;
    // wall_front_pose.position.y = distance; // 0.45;
    // wall_front_pose.position.z = 0.9;
    // ArmController::createCollisionObject(wall_front_obj, wall_front_id, wall_front_primitive, wall_front_pose, wall_front_type, wall_front_dim, planning_frame);
    
    // moveit_msgs::CollisionObject wall_right_obj;
    // std::string wall_right_id = "wall_right";
    // shape_msgs::SolidPrimitive wall_right_primitive;
    // std::vector<double> wall_right_dim = {0.1, width, 1.80};
    // std::string wall_right_type = "BOX";
    // geometry_msgs::Pose wall_right_pose;
    // wall_right_pose.orientation.w = 1.0;
    // wall_right_pose.position.x = distance; // 0.5;
    // wall_right_pose.position.y = 0.0;
    // wall_right_pose.position.z = 0.9;
    // ArmController::createCollisionObject(wall_right_obj, wall_right_id, wall_right_primitive, wall_right_pose, wall_right_type, wall_right_dim, planning_frame);

    // Actual publishing
    ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    moveit_msgs::PlanningScene planning_scene_msg;
    ArmController::addCollisionObjectToScene(robot_table_obj, planning_scene_msg);
    // ArmController::addCollisionObjectToScene(wall_left_obj, planning_scene_msg);
    // ArmController::addCollisionObjectToScene(wall_front_obj, planning_scene_msg);
    // ArmController::addCollisionObjectToScene(wall_right_obj, planning_scene_msg);

    planning_scene_msg.is_diff = true;
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        planning_scene_diff_publisher.publish(planning_scene_msg);
        ROS_INFO("published");
        loop_rate.sleep();
    }
}