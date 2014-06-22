#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
//#include <ee_cart_imped_action/ee_cart_imped_arm.h>
//#include <ee_cart_imped_msgs/EECartImpedGoal.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/robot_plan_w_keyframes_t.hpp"

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;



class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_l_;
  TrajClient* traj_client_r_;

  // Listen to LCM
  boost::shared_ptr<lcm::LCM> lcm_;

  void stateHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string &channel, const drc::robot_plan_w_keyframes_t* msg);
public:
  //! Initialize the action client and wait for action server to come up
  RobotArm(boost::shared_ptr<lcm::LCM> &lcm_): lcm_(lcm_)
  {
    // tell the action client that we want to spin a thread by default
    traj_client_l_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);
    traj_client_r_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while(!traj_client_l_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
    
    while(!traj_client_r_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
    
    lcm_->subscribe("CANDIDATE_MANIP_PLAN",&RobotArm::stateHandler,this);
  }

  //! Clean up the action client
  ~RobotArm()
  {
    delete traj_client_l_;
    delete traj_client_r_;
  }

  pr2_controllers_msgs::JointTrajectoryGoal drivearm_Plan(const drc::robot_plan_w_keyframes_t* msg, bool isleft);
  
  //! Sends the command to start a given trajectory
  void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goalL, pr2_controllers_msgs::JointTrajectoryGoal goalR)
  {
    // When to start the trajectory: 1s from now
    goalL.trajectory.header.stamp = ros::Time::now();
    goalR.trajectory.header.stamp = ros::Time::now();
    traj_client_l_->sendGoal(goalL);
    traj_client_r_->sendGoal(goalR);
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory(bool isleft)
  {
    //our goal variable.
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    if (isleft) {
        goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
        goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
        goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
        goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
        goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
    }
    else {
        goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
        goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
        goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
        goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
        goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
    }

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    // First trajectory point
    // Positions
    int ind = 0;
    int njoints = 7;
    goal.trajectory.points[ind].positions.resize(njoints);
    for (size_t j = 0; j < njoints; ++j)
    {
      goal.trajectory.points[ind].positions[j] = 0;
    }
    // Velocities
    goal.trajectory.points[ind].velocities.resize(njoints);
    for (size_t j = 0; j < njoints; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState(bool isleft)
  {
    if(isleft)
       return traj_client_l_->getState();
    else
       return traj_client_r_->getState();
  }
};

pr2_controllers_msgs::JointTrajectoryGoal RobotArm::drivearm_Plan(const drc::robot_plan_w_keyframes_t* msg, bool isleft) {
  int nstates = msg->num_states;
  
//our goal variable
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  // First, the joint names, which apply to all waypoints
  if(isleft){
      goal.trajectory.joint_names.push_back("l_shoulder_pan_joint"); //22-6 = 16
      goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
      goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
      goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
      goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
      goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
      goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
  }
  else {
      goal.trajectory.joint_names.push_back("r_shoulder_pan_joint"); //22-6 = 16
      goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
      goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
      goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
      goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
      goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
      goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
  }
  // We will have two waypoints in this goal trajectory
  goal.trajectory.points.resize(nstates-1);
  
  for(int ind=1; ind<nstates; ind++){
    int i = ind-1;
    // First trajectory point
    // Positions
    goal.trajectory.points[i].positions.resize(7);
    if(isleft){
        for (size_t j = 0; j < 7; ++j)
        {
          goal.trajectory.points[i].positions[j] = msg->plan[ind].joint_position[29+j];
        }
        printf("%dL POS J=%.3f %.3f %.3f %.3f %.3f %.3f %.3f, T=%f\n", 
          ind,
          msg->plan[ind].joint_position[29],
          msg->plan[ind].joint_position[30],
          msg->plan[ind].joint_position[31],
          msg->plan[ind].joint_position[32],
          msg->plan[ind].joint_position[33],
          msg->plan[ind].joint_position[34],
          msg->plan[ind].joint_position[35],
          (msg->plan[ind].utime)/1e6);
    }
    else{
        for (size_t j = 0; j < 7; ++j)
        {
          goal.trajectory.points[i].positions[j] = msg->plan[ind].joint_position[16+j];
        }
        printf("%dR POS J=%.3f %.3f %.3f %.3f %.3f %.3f %.3f, T=%f\n", 
          ind,
          msg->plan[ind].joint_position[16],
          msg->plan[ind].joint_position[17],
          msg->plan[ind].joint_position[18],
          msg->plan[ind].joint_position[19],
          msg->plan[ind].joint_position[20],
          msg->plan[ind].joint_position[21],
          msg->plan[ind].joint_position[22],
          (msg->plan[ind].utime)/1e6);
    }
      
    // Velocities
    goal.trajectory.points[i].velocities.resize(7);
    if(isleft){
        for (size_t j = 0; j < 7; ++j)
        {
          goal.trajectory.points[i].velocities[j] = msg->plan[ind].joint_velocity[29+j];
        }
        printf("%dL VEL J=%.3f %.3f %.3f %.3f %.3f %.3f %.3f, T=%f\n", 
			  ind,
			  msg->plan[ind].joint_velocity[29],
			  msg->plan[ind].joint_velocity[30],
			  msg->plan[ind].joint_velocity[31],
			  msg->plan[ind].joint_velocity[32],
			  msg->plan[ind].joint_velocity[33],
			  msg->plan[ind].joint_velocity[34],
			  msg->plan[ind].joint_velocity[35],
			  (msg->plan[ind].utime)/1e6);

    }
    else {
        for (size_t j = 0; j < 7; ++j)
        {
          goal.trajectory.points[i].velocities[j] = msg->plan[ind].joint_velocity[16+j];
        }
        printf("%dR VEL J=%.3f %.3f %.3f %.3f %.3f %.3f %.3f, T=%f\n", 
			  ind,
			  msg->plan[ind].joint_velocity[16],
			  msg->plan[ind].joint_velocity[17],
			  msg->plan[ind].joint_velocity[18],
			  msg->plan[ind].joint_velocity[19],
			  msg->plan[ind].joint_velocity[20],
			  msg->plan[ind].joint_velocity[21],
			  msg->plan[ind].joint_velocity[22],
			  (msg->plan[ind].utime)/1e6);
    }
    goal.trajectory.points[i].time_from_start = ros::Duration((msg->plan[ind].utime)/1e6);
    
  }
  
  return goal;
    
}


void RobotArm::stateHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const drc::robot_plan_w_keyframes_t* msg) {
  int nstates = msg->num_states;
  int njoints = msg->plan.size()>0 ? msg->plan[0].joint_position.size() : 0;
  
  printf("received plan with nstates=%d with njoints=%d\n", nstates, njoints);
  startTrajectory(drivearm_Plan(msg, true), drivearm_Plan(msg, false));
}


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "robot_arm_driver");
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }  

  RobotArm arm(lcm);
  //Start the trajectory
  /*arm.startTrajectory(arm.armExtensionTrajectory(true),arm.armExtensionTrajectory(false));
  // Wait for trajectory completion
  printf("startTrajectory\n");
  while(!arm.getState(true).isDone() && !arm.getState(false).isDone()&& ros::ok())
  {
    usleep(50000);
  }*/
  
  printf("drive_arm_lcm ready.\n");
  
  while(0 == lcm->handle());
  
  return 0;
}

