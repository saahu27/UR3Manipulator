#include "../include/pick_place.hpp"

int main(int argc, char **argv) {

    ros::init(argc, argv, "ppscene");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle n;

    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

    std::string planning_frame = arm_move_group.getPlanningFrame();

    // Box
    moveit_msgs::CollisionObject robot_box_obj;
    std::string box_id = "box";
    shape_msgs::SolidPrimitive box_primitive;
    std::vector<double> box_dim = {0.04, 0.04, 0.04};
    std::string box_type = "BOX";
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.40;
    box_pose.position.z = 0.786;
    ArmController::createCollisionObject(robot_box_obj, box_id, box_primitive, box_pose, box_type, box_dim, planning_frame);

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

    ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    moveit_msgs::PlanningScene planning_scene_msg;
    ArmController::addCollisionObjectToScene(robot_box_obj, planning_scene_msg);
    ArmController::addCollisionObjectToScene(robot_table_obj, planning_scene_msg);

    planning_scene_msg.is_diff = true;
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        planning_scene_diff_publisher.publish(planning_scene_msg);
        ROS_INFO("published");
        loop_rate.sleep();
    }
}