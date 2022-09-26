#include "../include/pick_place.hpp"



PickPlace::PickPlace(ros::NodeHandle *nodehandle): PP{*nodehandle} {
    this->pick_detected = false;
    fiducial_sub = PP.subscribe("/logitech_webcam/fiducial_transforms", 1, &PickPlace::Fiducial_Callback, this);
    pick_sub = PP.subscribe("/aruco_tf/picktopic", 1, &PickPlace::Pick_Callback, this);
}

void PickPlace::Pick_Callback(const geometry_msgs::Pose::ConstPtr& pick) {
  if(this->pick_detected){
    this->Pick.position.x = pick->position.x;
    this->Pick.position.y = pick->position.y;
    this->Pick.position.z = pick->position.z;
    this->Pick.orientation.x = pick->orientation.x;
    this->Pick.orientation.y = pick->orientation.y;
    this->Pick.orientation.z = pick->orientation.z;
    this->Pick.orientation.w = pick->orientation.w;
  }
}

void PickPlace::Fiducial_Callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
  
  if(!msg->transforms.empty()){
    if(msg->transforms.size() == 1){
      if(msg->transforms[0].fiducial_id == 5){
        this->pick_detected = true;
      }
      else{
        this->pick_detected = false;
      }
    }
  }
}

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "pickplace");
  ros::AsyncSpinner spinner(4);
  spinner.start();

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

  ros::NodeHandle nh;
  PickPlace pickplace(&nh);
  ros::Rate r(10);

  while(ros::ok()){
    ros::spinOnce();
    // ROS_INFO("Looking for Marker");
    if(pickplace.pick_detected) break;
    r.sleep();
  }

  moveit::planning_interface::MoveGroupInterface::Plan pick_plan;

  int flag = 0;
  bool pick_plan_success;
  std::string reference_frame = "base_link_inertia";

  ROS_INFO("Planning to");
  ROS_INFO("x: %f",pickplace.Pick.position.x);
  ROS_INFO("y: %f",pickplace.Pick.position.y);
  ROS_INFO("z: %f",pickplace.Pick.position.z);

  pick_plan_success = ArmController::planToPoseTarget(planning_options,arm_move_group,pickplace.Pick,reference_frame,pick_plan);

  if(pick_plan_success){
        ROS_INFO("pick plan succeeded");
        ros::Duration(1.0).sleep();
        flag = 1;

        arm_move_group.execute(pick_plan);

    }
    else{
        ROS_INFO("pick plan failed");
        flag = 0;
    }

  
}


