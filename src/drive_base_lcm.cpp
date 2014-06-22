#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>


#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/robot_plan_w_keyframes_t.hpp"

class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  //! We will be listening to TF transforms as well
  tf::TransformListener listener_;
  
  // Listen to LCM
  boost::shared_ptr<lcm::LCM> lcm_;

  void stateHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string &channel, const drc::robot_plan_w_keyframes_t* msg);

  bool drivebase_Plan(const drc::robot_plan_w_keyframes_t* msg);
     
public:
  //! ROS node initialization
  RobotDriver(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_): lcm_(lcm_),nh_(nh_) 
  {
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    lcm_->subscribe("CANDIDATE_MANIP_PLAN",&RobotDriver::stateHandler,this);
  }


  //! Drive forward a specified distance based on odometry information
  bool driveForwardOdom(double distance)
  {
    //wait for the listener to get the first message
    listener_.waitForTransform("base_footprint", "odom_combined", 
                               ros::Time(0), ros::Duration(1000.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("base_footprint", "odom_combined", 
                              ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to go forward at 0.25 m/s
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = 0.25;
    
    ros::Rate rate(10.0);
    bool done = false;
    while (!done && nh_.ok())
    {
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_footprint", "odom_combined", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how far we've traveled
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      double dist_moved = relative_transform.getOrigin().length();

      if(dist_moved > distance) done = true;
    }
    if (done) return true;
    return false;
  }
  //! Drive forward a specified distance based on odometry information
  bool testdrive()
  {
    geometry_msgs::Twist base_cmd;
    //the command will be to go forward at 0.25 m/s
    base_cmd.linear.y = 1;
    base_cmd.angular.z = 0;
    base_cmd.linear.x = 0;
    
    ros::Rate rate(10.0);
    bool done = false;
    for(int i=0;i<100 && nh_.ok();i++)
    {
      printf("%d",i);
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
    }
    return true;
  }
  bool turnOdom(bool clockwise, double radians)
  {
    while(radians < 0) radians += 2*M_PI;
    while(radians > 2*M_PI) radians -= 2*M_PI;

    //wait for the listener to get the first message
    listener_.waitForTransform("base_footprint", "odom_combined", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("base_footprint", "odom_combined", 
                              ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to turn at 0.75 rad/s
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = 0.75;
    if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;
    
    //the axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0,0,1);
    if (!clockwise) desired_turn_axis = -desired_turn_axis;
    
    ros::Rate rate(10.0);
    bool done = false;
    while (!done && nh_.ok())
    {
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_footprint", "odom_combined", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      tf::Vector3 actual_turn_axis = 
        relative_transform.getRotation().getAxis();
      double angle_turned = relative_transform.getRotation().getAngle();
      if ( fabs(angle_turned) < 1.0e-2) continue;

      if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
        angle_turned = 2 * M_PI - angle_turned;

      if (angle_turned > radians) done = true;
    }
    if (done) return true;
    return false;
  }
};


bool RobotDriver::drivebase_Plan(const drc::robot_plan_w_keyframes_t* msg) {
  int nstates = msg->num_states;
  
    
    //wait for the listener to get the first message
    listener_.waitForTransform("base_footprint", "odom_combined", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("base_footprint", "odom_combined", 
                              ros::Time(0), start_transform);
    
    
    //the axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0,0,1);
    
    ros::Rate rate(0.4);
  
  
  for(int i=0; i<nstates; i++){
    bool done = false;
    //msg->plan[i];
    printf("i=%d utime=%ld\n", i,msg->utime);
    
    printf("pose.translation(%.2f,%.2f,%.2f)\n",
      msg->plan[i].pose.translation.x,
      msg->plan[i].pose.translation.y,
      msg->plan[i].pose.translation.z);
    printf("pose.rotation(%.2f,%.2f,%.2f,%.2f)\n",
      msg->plan[i].pose.rotation.w,
      msg->plan[i].pose.rotation.x,
      msg->plan[i].pose.rotation.y,
      msg->plan[i].pose.rotation.z);
    printf("twist.linear_velocity(%.2f,%.2f,%.2f)\n",
      msg->plan[i].twist.linear_velocity.x,
      msg->plan[i].twist.linear_velocity.y,
      msg->plan[i].twist.linear_velocity.z);
    printf("twist.angular_velocity(%.2f,%.2f,%.2f)\n",
      msg->plan[i].twist.angular_velocity.x,
      msg->plan[i].twist.angular_velocity.y,
      msg->plan[i].twist.angular_velocity.z);
      
    
    
    
    
    while (!done && nh_.ok())
    {
      geometry_msgs::Twist base_cmd;
      //the command will be to turn at 0.75 rad/s
      //base_cmd.linear.x = msg->plan[i].twist.linear_velocity.x;
      //base_cmd.linear.y = msg->plan[i].twist.linear_velocity.y;
      //base_cmd.angular.z = msg->plan[i].twist.angular_velocity.x;
      base_cmd.linear.y = 1;
      
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_footprint", "odom_combined", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      /*
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      tf::Vector3 actual_turn_axis = 
        relative_transform.getRotation().getAxis();
      double angle_turned = relative_transform.getRotation().getAngle();
      if ( fabs(angle_turned) < 1.0e-2) continue;

      if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
        angle_turned = 2 * M_PI - angle_turned;

      if (angle_turned > radians) done = true;*/
      done = true;
    }
  }
    
    
}

void RobotDriver::stateHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const drc::robot_plan_w_keyframes_t* msg) {
  int nstates = msg->num_states;
  printf("received plan with nstates=%d\n", nstates);
  drivebase_Plan(msg);
}


int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }  
  ros::NodeHandle nh;


  RobotDriver driver(lcm,nh);
  //driver.driveForwardOdom();
  driver.testdrive();
  //driver.turnOdom(true,1.732);
  printf("drive_base_lcm ready\n");
  while(0 == lcm->handle());
  return 0;
}

