// include header file for empty planner class 
#include "EmptyLocalPlanner.h"
// include header for plugin registration
#include <pluginlib/class_list_macros.h>

// Regsiter empty planner class as plugin -- make available to ROS 

PLUGINLIB_EXPORT_CLASS(empty_local_planner::EmptyLocalPlanner, nav_core::BaseLocalPlanner)

namespace empty_local_planner{

// initialse memeber variables fpr EmptyLocalPlanner  
EmptyLocalPlanner::EmptyLocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

// init planner with ROS objects 
void EmptyLocalPlanner::initialize(std::string, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
    //check if already init 
    if(!initialized_){
        // store the provided ROS objects for later use 
        tf_ = tf;
        //set inital flag to true
        initialized_ = true;
  }
}
bool EmptyLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& /*plan*/) {
    return true; // always true, no velocity commands will be generated. 
}

// check if goal is reached -- return false
bool EmptyLocalPlanner::isGoalReached(){
    return false;
}

}

