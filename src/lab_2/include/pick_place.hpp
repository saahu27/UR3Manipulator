#pragma once

#include <moveit_wrapper/arm_controller.hpp>
#include "../include/aruco_tf.hpp"



class PickPlace {
    public:
    /*
     */
    PickPlace(ros::NodeHandle *nodehandle);

    void Pick_Callback(const geometry_msgs::Pose::ConstPtr& pick);
    void Fiducial_Callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);

    ros::NodeHandle PP;
    ros::Subscriber pick_sub;
    ros::Subscriber fiducial_sub;
    bool pick_detected;
    geometry_msgs::Pose Pick;

};


