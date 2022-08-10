#include "../include/aruco_tf.hpp"

/**
 * @brief Save calibration data to file
 */
void ArucoTF::saveCalibToFile(const Eigen::Quaternionf &save_rot,
                              const Eigen::Vector3f &save_trans) {
  if (!ArucoTF::calib) {
    ROS_INFO_STREAM("Saving calibration to file");
    std::string calib_path = ros::package::getPath("lab_2");
    calib_path += "/calibration/camera/logitech_extrinsics.json";

    std::vector<float> rot, trans;

    // Convert quaternion (w,x,y,z) from Eigen to Vector
    rot.push_back(save_rot.w());
    rot.push_back(save_rot.x());
    rot.push_back(save_rot.y());
    rot.push_back(save_rot.z());

    // Convert translation from Eigen to Vector
    trans.push_back(save_trans(0));
    trans.push_back(save_trans(1));
    trans.push_back(save_trans(2));

    // Open existing calibration
    std::ifstream calib_file_in;
    calib_file_in.open(calib_path);

    std::stringstream ss;
    if (calib_file_in.is_open()) {
      // Convert to string
      ss << calib_file_in.rdbuf();
    } else {
      std::cout << "Unable to open file" << std::endl;
    }
    calib_file_in.close();

    // Parse calibration text to json object
    nlohmann::json calib_data;
    try {
      calib_data = nlohmann::json::parse(ss);
    } catch (nlohmann::json::parse_error &e) {
      ROS_WARN_STREAM("JSON Parse failed: " << e.what());
    }

    // Find corresponding camera data in json
    std::string calib_section = "logitech_webcam";
    auto cam_section_itr = calib_data.find(calib_section);
    if (cam_section_itr != calib_data.end()) {
      std::cout << "FOUND " << calib_section << " in calibration file."
                << std::endl;
    } else {
      std::cout << "NOT FOUND " << calib_section << " in calibration file."
                << std::endl;
    }

    // Add rotation and translation to json object
    calib_data[calib_section]["rot"] = rot;
    calib_data[calib_section]["trans"] = trans;

    std::ofstream calib_file_out(calib_path,
                                 std::fstream::out | std::ofstream::trunc);
    calib_file_out << std::setprecision(16) << calib_data;
    calib_file_out.close();
  }
}

/**
 * @brief Load calibration data from file
 */
void ArucoTF::loadCalibFromFile() {
  if (!ArucoTF::calib) {
    ROS_INFO_STREAM("Loading calibration from file");
    std::string calib_path = ros::package::getPath("lab_2");
    calib_path += "/calibration/camera/logitech_extrinsics.json";

    // Open existing calibration
    std::ifstream calib_file_in;
    calib_file_in.open(calib_path);

    std::stringstream ss;
    if (calib_file_in.is_open()) {
      // Convert to string
      ss << calib_file_in.rdbuf();
    } else {
      std::cout << "Unable to open file" << std::endl;
    }
    calib_file_in.close();

    // Parse calibration text to json object
    nlohmann::json calib_data;
    try {
      calib_data = nlohmann::json::parse(ss);
    } catch (nlohmann::json::parse_error &e) {
      ROS_WARN_STREAM("JSON Parse failed: " << e.what());
    }

    // Find corresponding camera data in json
    std::string calib_section = "logitech_webcam";
    auto cam_section_itr = calib_data.find(calib_section);
    if (cam_section_itr != calib_data.end()) {
      std::cout << "FOUND " << calib_section << " in calibration file."
                << std::endl;
      // Get translation and rotation data from json
      std::vector<float> trans_json = calib_data[calib_section]["trans"];
      std::vector<float> rot_json = calib_data[calib_section]["rot"];

      // Vector in (x, y, z) format
      Eigen::Vector3f trans;
      trans(0) = trans_json[0];
      trans(1) = trans_json[1];
      trans(2) = trans_json[2];
      // Quaternion in (w,x,y,z) format
      Eigen::Quaternionf quat;
      quat.w() = rot_json[0];
      quat.x() = rot_json[1];
      quat.y() = rot_json[2];
      quat.z() = rot_json[3];

      std::cout << trans_json << std::endl;
      std::cout << rot_json << std::endl;

      // Apply calibration data to camera
      ArucoTF::setTFCamToWorld(quat, trans);
      // Set calibrated
      ArucoTF::calib = true;
    } else {
      std::cout << "NOT FOUND " << calib_section << " in calibration file."
                << std::endl;
      ArucoTF::calib = false;
    }
  }
}

void ArucoTF::setTFCamToWorld(tf2::Quaternion &quat, tf2::Vector3 &trans) {
  // Convert to tf2
  ArucoTF::tf_camToWorld.setRotation(quat);
  ArucoTF::tf_camToWorld.setOrigin(trans);
}

void ArucoTF::setTFCamToWorld(Eigen::Quaternionf &quat,
                              Eigen::Vector3f &trans) {
  // Get rotation in tf2
  tf2::Quaternion rot_quat_camToWorld(quat.x(), quat.y(), quat.z(), quat.w());
  rot_quat_camToWorld.normalize();
  // Get translation in tf2
  tf2::Vector3 trans_camToWorld(trans(0), trans(1), trans(2));

  // Convert to tf2
  ArucoTF::tf_camToWorld.setRotation(rot_quat_camToWorld);
  ArucoTF::tf_camToWorld.setOrigin(trans_camToWorld);
}

/**
 * @brief Get transform from source to destination given 3D point
 * correspondences.
 */

void ArucoTF::estimateTransformPointToPoint() {
  // Compute transform
  Eigen::Matrix4f tf_srcToDst = Eigen::Matrix4f::Zero();
  ROS_INFO_STREAM("Calibrating camera to world");
  tf_srcToDst = Eigen::umeyama(ArucoTF::samples_camToMarker,
                               ArucoTF::samples_markerToWorld);

  // Get rotation
  Eigen::Matrix3f rot = tf_srcToDst.topLeftCorner(3, 3);
  Eigen::Quaternionf rot_quat(rot);
  tf2::Quaternion rot_quat_camToWorld(rot_quat.x(), rot_quat.y(), rot_quat.z(),
                                      rot_quat.w());

  // Get translation
  Eigen::Vector3f trans = tf_srcToDst.topRightCorner(3, 1);
  tf2::Vector3 trans_camToWorld(trans(0), trans(1), trans(2));

  // Convert to tf2
  ArucoTF::setTFCamToWorld(rot_quat_camToWorld, trans_camToWorld);

  // Save data to file
  ArucoTF::saveCalibToFile(rot_quat, trans);

  // Set calibrated
  ArucoTF::calib = true;
}

/**
 * @brief Take samples of camera to marker and marker to world poses from robot
 */
void ArucoTF::takeCalibrationSamples() {
  if (ArucoTF::calib) {
    ROS_INFO_STREAM("Already calibrated, exiting.");
    return;
  };

  int sample_cnt = 0;
  ROS_INFO_STREAM("Move robot to pose...");
  ROS_INFO_STREAM("Press ENTER to record sample.");
<<<<<<< HEAD
  
  if(ArucoTF::recordCalibPoints){
    std::string calib_points_path = ros::package::getPath("lab_2");
    calib_points_path += "/calib_points.txt";
    ArucoTF::file.open(calib_points_path);
  }
  
=======
>>>>>>> 9e3fc93 (yes)

  while (sample_cnt < num_samples) {
    ROS_INFO_STREAM("Pose: " << sample_cnt + 1 << "/"
                        << ArucoTF::num_samples);
    char c = getchar();

    ArucoTF::lookup_camToMarker();
    ArucoTF::samples_camToMarker.col(sample_cnt) =
        Eigen::Vector3f(ArucoTF::tform_camToMarker.translation.x,
                        ArucoTF::tform_camToMarker.translation.y,
                        ArucoTF::tform_camToMarker.translation.z)
            .transpose();

    ArucoTF::lookup_markerToWorld();
    ArucoTF::samples_markerToWorld.col(sample_cnt) =
        Eigen::Vector3f(ArucoTF::tform_markerToWorld.transform.translation.x,
                        ArucoTF::tform_markerToWorld.transform.translation.y,
                        ArucoTF::tform_markerToWorld.transform.translation.z)
            .transpose();

<<<<<<< HEAD
    if(ArucoTF::recordCalibPoints){
      ArucoTF::file << ArucoTF::tform_camToMarker.translation.x << " " 
        << ArucoTF::tform_camToMarker.translation.y << " " 
        << ArucoTF::tform_camToMarker.translation.z << " ";        
    
      ArucoTF::file << ArucoTF::tform_markerToWorld.transform.translation.x << " " 
          << ArucoTF::tform_markerToWorld.transform.translation.y << " " 
          << ArucoTF::tform_markerToWorld.transform.translation.z << "\n";
    }
    

    sample_cnt++;
  }
  ROS_INFO_ONCE("Calibration samples gathered");
  if(ArucoTF::recordCalibPoints) ArucoTF::file.close();
=======
    sample_cnt++;
  }
  ROS_INFO_ONCE("Calibration samples gathered");
>>>>>>> 9e3fc93 (yes)
}

/**
 * @brief Callback function to get marker pose in camera coordinates
 * Sets class variable with returned value
 */
void ArucoTF::lookup_camToMarker() {
  ROS_INFO_STREAM("Getting aruco transform");

  fiducial_msgs::FiducialTransformArray::ConstPtr fiducial_msg =
      ros::topic::waitForMessage<fiducial_msgs::FiducialTransformArray>(
          ArucoTF::aruco_transform_topic);

  for (fiducial_msgs::FiducialTransform marker : fiducial_msg->transforms) {
    if (marker.fiducial_id == aruco_calib_target) {
      ArucoTF::tform_camToMarker = marker.transform;
    }
  }
}

/**
 * @brief Callback function to get marker pose in camera coordinates
 * Overloaded to accept marker ID
 * Returns obtained transform
 */
geometry_msgs::Transform ArucoTF::lookup_camToMarker(const int &marker_id) {
  ROS_INFO_STREAM("Getting aruco transform for Marker " << marker_id);

  fiducial_msgs::FiducialTransformArray::ConstPtr fiducial_msg =
      ros::topic::waitForMessage<fiducial_msgs::FiducialTransformArray>(
          ArucoTF::aruco_transform_topic);

  for (fiducial_msgs::FiducialTransform marker : fiducial_msg->transforms) {
    if (marker.fiducial_id == marker_id) {
      geometry_msgs::Transform tform = marker.transform;
      return tform;
    }
  }

<<<<<<< HEAD
  // throw(ArucoTF::NoTransformException());
=======
  throw(ArucoTF::NoTransformException());
>>>>>>> 9e3fc93 (yes)
}

/**
 * @brief Get transform from tool0 to world frame
 * The marker is placed at the tool0 location on the robot
 * lookupTransform goes to target frame from source
 */
void ArucoTF::lookup_markerToWorld() {
  // TF2 listener for marker to world
  try {
<<<<<<< HEAD
    if (ArucoTF::tfBuffer.canTransform("base_link_inertia", "tool0", ros::Time(0))) {
      ROS_INFO_STREAM("Getting tool0 to world");
      ArucoTF::tform_markerToWorld =
          tfBuffer.lookupTransform("base_link_inertia", "tool0", ros::Time(0));
=======
    if (ArucoTF::tfBuffer.canTransform("base_link", "tool0", ros::Time(0))) {
      ROS_INFO_STREAM("Getting tool0 to world");
      ArucoTF::tform_markerToWorld =
          tfBuffer.lookupTransform("base_link", "tool0", ros::Time(0));
>>>>>>> 9e3fc93 (yes)
    } else {
      ArucoTF::tform_markerToWorld = geometry_msgs::TransformStamped();
      ROS_INFO_STREAM("Could not find transform from world to tool0");
    }
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

/**
 * @brief Broadcast camera pose with respect to world
 */
void ArucoTF::broadcast_camToWorld() {
  ROS_INFO_STREAM_THROTTLE(10, "Broadcasting camera to world");
  ArucoTF::tform_camToWorld.header.stamp = ros::Time::now();
<<<<<<< HEAD
  ArucoTF::tform_camToWorld.header.frame_id = "base_link_inertia";
=======
  ArucoTF::tform_camToWorld.header.frame_id = "base_link";
>>>>>>> 9e3fc93 (yes)
  ArucoTF::tform_camToWorld.child_frame_id = "logitech_webcam";
  ArucoTF::tform_camToWorld.transform = tf2::toMsg(ArucoTF::tf_camToWorld);
  ArucoTF::br_camToWorld.sendTransform(ArucoTF::tform_camToWorld);
}

/**
 * @brief Broadcast marker pose from camera frame to world frame
 */
void ArucoTF::broadcast_allMarkersToWorld() {
  for (auto i : ArucoTF::aruco_track_targets) {
    ROS_INFO_STREAM_THROTTLE(10, "Broadcasting marker_" << i
                                    << " to world");
    // Get marker_i to cam
    geometry_msgs::Transform tform_camToNewMarker =
        ArucoTF::lookup_camToMarker(i);
    tf2::Transform tf_camToNewMarker;
    tf2::fromMsg(tform_camToNewMarker, tf_camToNewMarker);

    // Transform to world frame
    tf2::Transform tf_newMarkerToWorld =
        ArucoTF::tf_camToWorld * tf_camToNewMarker;

    // Convert back to geometry_msgs::TransformStamped
    geometry_msgs::TransformStamped tform_newMarkerToWorld;
    tform_newMarkerToWorld.header.stamp = ros::Time::now();
<<<<<<< HEAD
    tform_newMarkerToWorld.header.frame_id = "base_link_inertia";
=======
    tform_newMarkerToWorld.header.frame_id = "base_link";
>>>>>>> 9e3fc93 (yes)
    tform_newMarkerToWorld.child_frame_id = "marker_" + std::to_string(i);
    tform_newMarkerToWorld.transform = tf2::toMsg(tf_newMarkerToWorld);

    ArucoTF::br_markersToWorld.sendTransform(tform_newMarkerToWorld);
  }
}

/**
 * @brief Compute marker pose from camera frame to world frame
 */
void ArucoTF::lookup_allMarkersToWorld(const int &marker_id,
                                       tf2::Transform &tf_newMarkerToWorld) {
  try {
    // Get marker_id to cam
    geometry_msgs::Transform tform_camToNewMarker =
        ArucoTF::lookup_camToMarker(marker_id);
    tf2::Transform tf_camToNewMarker;
    tf2::fromMsg(tform_camToNewMarker, tf_camToNewMarker);

    // Transform to world frame
    tf_newMarkerToWorld = ArucoTF::tf_camToWorld * tf_camToNewMarker;
    tf_newMarkerToWorld.getRotation().normalize();

  } catch (const ArucoTF::NoTransformException &e) {
    ROS_WARN("%s", e.what());
    ros::Duration(1.0).sleep();
  }
}

<<<<<<< HEAD
void ArucoTF::getCalibratedPoints(){
  std::string calib_points_path = ros::package::getPath("lab_2");
  calib_points_path += "/calib_points.txt";
  ArucoTF::myfile.open(calib_points_path);

  float x;
  float y;
  float z;
  int count = 0;
  while(ArucoTF::myfile){
    // std::getline(ArucoTF::myfile,myline);
    if(count < ArucoTF::num_samples){
      ArucoTF::myfile >> x;
      ArucoTF::myfile >> y;
      ArucoTF::myfile >> z;
      std::cout << x << " " << y << " " << z << std::endl;
      ArucoTF::calib_points_camToMarker.push_back({x,y,z});
      ArucoTF::myfile >> x;
      ArucoTF::myfile >> y;
      ArucoTF::myfile >> z;
      std::cout << x << " " << y << " " << z << std::endl;
      ArucoTF::calib_points_markerToWorld.push_back({x,y,z});
      count++;
    }
    else break;
  }
  ArucoTF::myfile.close();

  std::string calib_points2_path = ros::package::getPath("lab_2");
  calib_points2_path += "/calib_points2.txt";
  ArucoTF::file.open(calib_points2_path);

  for(int i = 0; i < ArucoTF::calib_points_camToMarker.size(); i++){
    geometry_msgs::Transform tform_camToNewMarker;
    tform_camToNewMarker.translation.x = ArucoTF::calib_points_camToMarker[i][0];
    tform_camToNewMarker.translation.y = ArucoTF::calib_points_camToMarker[i][1];
    tform_camToNewMarker.translation.z = ArucoTF::calib_points_camToMarker[i][2];
    
    tf2::Transform tf_camToNewMarker;
    tf2::fromMsg(tform_camToNewMarker, tf_camToNewMarker);

    // Transform to world frame
    tf2::Transform tf_newMarkerToWorld;
    tf_newMarkerToWorld = ArucoTF::tf_camToWorld * tf_camToNewMarker;
    tf_newMarkerToWorld.getRotation().normalize();

    ArucoTF::file << tf_newMarkerToWorld.getOrigin()[0] << " " 
        << tf_newMarkerToWorld.getOrigin()[1] << " " 
        << tf_newMarkerToWorld.getOrigin()[2] << " ";

    ArucoTF::file << ArucoTF::calib_points_markerToWorld[i][0] << " " 
        << ArucoTF::calib_points_markerToWorld[i][1] << " " 
        << ArucoTF::calib_points_markerToWorld[i][2] << "\n";
  }

  ArucoTF::file.close();



}

=======
>>>>>>> 9e3fc93 (yes)
/**
 * @brief Compare calibration marker to ideal transformation
 *
 * @param marker_id
 */
void ArucoTF::verifyCalibration(const int &marker_id) {
  // Get marker to world
  tf2::Transform tf_calibMarkerToWorld;
  ArucoTF::lookup_allMarkersToWorld(ArucoTF::aruco_calib_target,
                                    tf_calibMarkerToWorld);
  // Get tool0 TF
  tf2::Stamped<tf2::Transform> tf_toolToWorld;
  ArucoTF::lookup_markerToWorld();
  tf2::fromMsg(ArucoTF::tform_markerToWorld, tf_toolToWorld);

  // Check errors between the two
  float translation_error = ArucoTF::euclidean<float>(
      tf_calibMarkerToWorld.getOrigin(), tf_toolToWorld.getOrigin());
  float quaternion_error =
      tf_calibMarkerToWorld.getRotation().dot(tf_toolToWorld.getRotation());
  std::cout << "TranslationErr: " << translation_error
            << " RotationErr: " << quaternion_error << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "aruco_tf");
  ros::NodeHandle n("~");
  ros::Rate rate(10.0);
  ROS_INFO("Started node");

  bool load_calib;
  bool verify_calib;
  int num_poses;
<<<<<<< HEAD
  bool record;
  // Load external calibration file
  n.param<bool>("load_calibration", load_calib, true);
=======
  // Load external calibration file
  n.param<bool>("load_calibration", load_calib, false);
>>>>>>> 9e3fc93 (yes)
  // Verify calibration
  n.param<bool>("verify_calibration", verify_calib, false);
  // Number of poses to use for calibration
  n.param<int>("num_poses", num_poses, 2);

<<<<<<< HEAD
  n.param<bool>("record_calib_points", record, false);

  ROS_INFO("----------------------------------------------------------");
  // Set number of poses to capture for calibration
  ArucoTF calibrate_cam(load_calib, num_poses, record);
  // Camera calibration
  if (!calibrate_cam.load_calib) {
    calibrate_cam.takeCalibrationSamples();
    if(!calibrate_cam.recordCalibPoints) calibrate_cam.estimateTransformPointToPoint();
  } else {
    calibrate_cam.loadCalibFromFile();
    calibrate_cam.getCalibratedPoints();
=======
  ROS_INFO("----------------------------------------------------------");
  // Set number of poses to capture for calibration
  ArucoTF calibrate_cam(load_calib, num_poses);
  // Camera calibration
  if (!calibrate_cam.load_calib) {
    calibrate_cam.takeCalibrationSamples();
    calibrate_cam.estimateTransformPointToPoint();
  } else {
    calibrate_cam.loadCalibFromFile();
>>>>>>> 9e3fc93 (yes)
  }
  ROS_INFO("----------------------------------------------------------");
  if (verify_calib) {
    calibrate_cam.verifyCalibration(1);
  }

<<<<<<< HEAD
  tf2::Transform tf_pickMarkerToWorld;
  // tf2::Transform tf_placeMarkerToWorld;

  ros::Publisher pick_pub = n.advertise<geometry_msgs::Pose>("picktopic", 1);
  // ros::Publisher place_pub = n.advertise<geometry_msgs::Pose>("placetopic", 1000);

  geometry_msgs::Pose pick;
  // geometry_msgs::Pose place;

  if(!calibrate_cam.recordCalibPoints){
    while (n.ok()) {
      calibrate_cam.broadcast_camToWorld();
      calibrate_cam.broadcast_allMarkersToWorld();
      calibrate_cam.lookup_allMarkersToWorld(5, tf_pickMarkerToWorld);
      // calibrate_cam.lookup_allMarkersToWorld(1, tf_placeMarkerToWorld); 
      pick.position.x = tf_pickMarkerToWorld.getOrigin()[0];
      pick.position.y = tf_pickMarkerToWorld.getOrigin()[1];
      pick.position.z = tf_pickMarkerToWorld.getOrigin()[2];
      pick.orientation.x = tf_pickMarkerToWorld.getRotation()[0];
      pick.orientation.y = tf_pickMarkerToWorld.getRotation()[1];
      pick.orientation.z = tf_pickMarkerToWorld.getRotation()[2];
      pick.orientation.w = tf_pickMarkerToWorld.getRotation()[3]; 

      // place.position.x = tf_placeMarkerToWorld.getOrigin()[0];
      // place.position.y = tf_placeMarkerToWorld.getOrigin()[1];
      // place.position.z = tf_placeMarkerToWorld.getOrigin()[2];
      // place.orientation.x = tf_placeMarkerToWorld.getRotation()[0];
      // place.orientation.y = tf_placeMarkerToWorld.getRotation()[1];
      // place.orientation.z = tf_placeMarkerToWorld.getRotation()[2];
      // place.orientation.w = tf_placeMarkerToWorld.getRotation()[3];

      // ROS_INFO("x: %f",pick.position.x);
      // ROS_INFO("y: %f",pick.position.y);
      // ROS_INFO("z: %f",pick.position.z);

      pick_pub.publish(pick);
  
      // place_pub.publish(place);
      ros::spinOnce();
      rate.sleep();
    }
  }



=======
  while (n.ok()) {
    calibrate_cam.broadcast_camToWorld();
    ros::spinOnce();
    rate.sleep();
  }
>>>>>>> 9e3fc93 (yes)
}