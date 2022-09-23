#include <moveit_wrapper/arm_controller.hpp>


geometry_msgs::Pose ArmController::getRandomPose(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    std::string &frame_name) {
  geometry_msgs::PoseStamped random_pose;
  move_group_interface.setPoseReferenceFrame(frame_name);
  random_pose = move_group_interface.getRandomPose("tool0");

  return random_pose.pose;
}

bool ArmController::planToNamedTarget(
    MoveitPlanning::PlanningOptions &options,
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    std::string &target_name,
    moveit::planning_interface::MoveGroupInterface::Plan &plan) {
  move_group_interface.clearPoseTargets();
  move_group_interface.setStartState(*move_group_interface.getCurrentState());
  move_group_interface.setPlanningTime(options.set_planning_time);
  move_group_interface.allowReplanning(options.allow_replanning);
  move_group_interface.setNumPlanningAttempts(options.num_attempts);
  move_group_interface.setNamedTarget(target_name);
  bool plan_success = false;

  plan_success = (move_group_interface.plan(plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Return success or failure
  if (plan_success) {
    ROS_INFO("Plan Succeeded");
  } else {
    ROS_ERROR("Plan Failed");
  }
  ros::Duration(2.0).sleep();

  return plan_success;
}

bool ArmController::planToPoseTarget(
    MoveitPlanning::PlanningOptions &options,
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    geometry_msgs::Pose &target_pose, std::string &reference_frame, moveit::planning_interface::MoveGroupInterface::Plan &plan) {
  move_group_interface.clearPoseTargets();
  move_group_interface.setPlanningTime(options.set_planning_time);
  move_group_interface.allowReplanning(options.allow_replanning);
  move_group_interface.setNumPlanningAttempts(options.num_attempts);
  move_group_interface.setMaxAccelerationScalingFactor(
      options.acceleration_scaling_factor);
  move_group_interface.setMaxVelocityScalingFactor(
      options.velocity_scaling_factor);
  move_group_interface.setStartState(*move_group_interface.getCurrentState());
  move_group_interface.setPoseTarget(target_pose);
  move_group_interface.setPlannerId("RRTConnect");
  move_group_interface.setPoseReferenceFrame(reference_frame);
  ROS_INFO("Planning for the Pose Target");

  // Do planning for entire group
  bool plan_success = false;
  plan_success = (move_group_interface.plan(plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Return success or failure
  if (plan_success) {
    ROS_INFO("Plan Succeeded");
  } else {
    ROS_ERROR("Plan Failed");
  }
  ros::Duration(2.0).sleep();

  return plan_success;
}

bool ArmController::planToJointTargets(
    MoveitPlanning::PlanningOptions &options,
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    moveit::planning_interface::MoveGroupInterface::Plan &plan,
    std::map<std::string, double> &joint_targets) {
  move_group_interface.clearPoseTargets();
  move_group_interface.setPlanningTime(options.set_planning_time);
  move_group_interface.allowReplanning(options.allow_replanning);
  move_group_interface.setNumPlanningAttempts(options.num_attempts);
  move_group_interface.setGoalPositionTolerance(
      options.goal_position_tolerance);
  move_group_interface.setGoalOrientationTolerance(
      options.goal_orientation_tolerance);
  move_group_interface.setGoalJointTolerance(options.goal_joint_tolerance);
  move_group_interface.setStartState(*move_group_interface.getCurrentState());
  move_group_interface.setPlannerId("RRTConnect");

  move_group_interface.setJointValueTarget(joint_targets);
  ROS_INFO("Planning for the Joint Target");

  // Do planning for entire group
  bool plan_success = false;
  plan_success = (move_group_interface.plan(plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Return success or failure
  if (plan_success) {
    ROS_INFO("Plan Succeeded");
  } else {
    ROS_ERROR("Plan Failed");
  }
  ros::Duration(2.0).sleep();

  return plan_success;
}

void ArmController::createCollisionObject(
    moveit_msgs::CollisionObject &collision_obj, std::string primitive_id,
    shape_msgs::SolidPrimitive &primitive, geometry_msgs::Pose &obj_pose,
    std::string primitive_type,
    std::vector<double> &primitive_dim,
    std::string planning_frame) {
  try{
    ROS_INFO("Creating primitive object: %s", primitive_id.c_str());

    collision_obj.id = primitive_id;
    collision_obj.header.frame_id = planning_frame;
    
    if(primitive_type == "BOX"){
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
    }
    else if(primitive_type == "SPHERE"){
      primitive.type = primitive.SPHERE;
      primitive.dimensions.resize(1);
    }
    else if(primitive_type == "CYLINDER"){
      primitive.type = primitive.CYLINDER;
      primitive.dimensions.resize(2);
    }
    else if(primitive_type == "CONE"){
      primitive.type = primitive.CONE;
      primitive.dimensions.resize(2);
    }

    for(std::size_t i = 0; i < primitive_dim.size(); i++){
      primitive.dimensions[i] = primitive_dim[i];
    }

    collision_obj.primitives.clear();
    collision_obj.primitives.push_back(primitive);
    collision_obj.primitive_poses.clear();
    collision_obj.primitive_poses.push_back(obj_pose);
  }catch(const std::exception &e){
    std::cout << e.what() << std::endl;
  }
}

void ArmController::addCollisionObjectToScene(
    moveit_msgs::CollisionObject &collision_obj, moveit_msgs::PlanningScene &planning_scene_msg){
  ROS_INFO("Adding object to scene: %s", collision_obj.id.c_str());

  collision_obj.operation = collision_obj.ADD;
  // planning_scene_msg.world.collision_objects.clear();
  planning_scene_msg.world.collision_objects.push_back(collision_obj);
}

void ArmController::planCartesianPath(geometry_msgs::Pose start_pose, std::vector<geometry_msgs::Pose> waypoints,
  std::string &reference_frame, moveit::planning_interface::MoveGroupInterface &move_group_interface){
  moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
  const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup(move_group_interface.getName());
  start_state.setFromIK(joint_model_group, start_pose);
  move_group_interface.setStartState(start_state);

  // std::vector<geometry_msgs::Pose> waypoints;
  // waypoints.push_back(end_pose);

  move_group_interface.allowReplanning(true);
  move_group_interface.setMaxVelocityScalingFactor(0.1);
  move_group_interface.setMaxAccelerationScalingFactor(0.1);
  move_group_interface.setPoseReferenceFrame(reference_frame);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 5.0;
  const double eef_step = 0.005;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  ROS_INFO("Planned %.2f%% of path...", (fraction * 100));

  std::vector<double> joint_values;
  std::vector<std::vector<double>> eef_values;

  for(int i = 0; i < trajectory.joint_trajectory.points.size(); i++){
    for(int j = 0; j < trajectory.joint_trajectory.points[i].positions.size(); j++){
      ROS_INFO("%f",trajectory.joint_trajectory.points[i].positions[j]);
      joint_values.push_back(trajectory.joint_trajectory.points[i].positions[j]);
    }
    ROS_INFO("position %d printed", i);
    start_state.setJointGroupPositions("manipulator", joint_values);
    const Eigen::Affine3d& link_pose = start_state.getGlobalLinkTransform("tool0");
    Eigen::Vector3d cartesian_position = link_pose.translation();
    ROS_INFO("Printing eef pos");
    for(int k = 0; k < cartesian_position.size(); k++){
      ROS_INFO("%f", cartesian_position[k]);
    }

    joint_values.clear();
  }

  if(fraction >= 0.0) move_group_interface.execute(trajectory);
  else ROS_INFO("plan_failed");
}

void get_eef_positions(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::string in_path, std::string out_path){
  moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
  move_group_interface.setStartState(start_state);

  std::vector<double> joint_val;
  std::fstream joint_file;
  joint_file.open(in_path,std::ios::in);
  std::ofstream eef_file;
  eef_file.open(out_path);
  if(joint_file.is_open()){
    std::string tp;
    while(getline(joint_file,tp)){
      std::stringstream ss(tp);
      while(ss){
        double num;
        ss >> num;
        joint_val.push_back(num);
      }
      start_state.setJointGroupPositions("manipulator", joint_val);
      const Eigen::Affine3d& link_pose = start_state.getGlobalLinkTransform("tool0");
      Eigen::Vector3d cartesian_position = link_pose.translation();
      for(int i = 0; i < cartesian_position.size(); i++){
        if(i == cartesian_position.size()-1){
          eef_file << cartesian_position[i] << std::endl;
        }
        else{
          eef_file << cartesian_position[i] << ",";
        }
      }
      joint_val.clear();
    }
  }
  joint_file.close();
  eef_file.close();
}