#include "../include/scene.hpp"

int main(int argc, char **argv)
{
    // Setup ROS node
    ros::init(argc, argv, "scene");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

    std::string planning_frame = arm_move_group.getPlanningFrame();

    // Table
    moveit_msgs::CollisionObject robot_table_obj;
    std::string table_id = "table";
    shape_msgs::SolidPrimitive table_primitive;
    std::vector<double> table_dim = {0.47, 0.55, 0.80};
    std::string table_type = "BOX";
    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.0;
    table_pose.position.y = -0.18;
    table_pose.position.z = 0.40;
    ArmController::createCollisionObject(robot_table_obj, table_id, table_primitive, table_pose, table_type, table_dim, planning_frame);

    // Table 2
    moveit_msgs::CollisionObject robot_table2_obj;
    std::string table2_id = "table2";
    shape_msgs::SolidPrimitive table2_primitive;
    std::vector<double> table2_dim = {0.77, 0.77, 0.74};
    std::string table2_type = "BOX";
    geometry_msgs::Pose table2_pose;
    table2_pose.orientation.w = 1.0;
    table2_pose.position.x = -0.615;
    table2_pose.position.y = -0.18;
    table2_pose.position.z = 0.37;
    ArmController::createCollisionObject(robot_table2_obj, table2_id, table2_primitive, table2_pose, table2_type, table2_dim, planning_frame);

    // Actual publishing
    ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    moveit_msgs::PlanningScene planning_scene_msg;
    ArmController::addCollisionObjectToScene(robot_table_obj, planning_scene_msg);
    ArmController::addCollisionObjectToScene(robot_table2_obj, planning_scene_msg);

    planning_scene_msg.is_diff = true;
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        planning_scene_diff_publisher.publish(planning_scene_msg);
        ROS_INFO("published");
        loop_rate.sleep();
    }
}