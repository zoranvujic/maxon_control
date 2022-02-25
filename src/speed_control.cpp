#include "maxon_control/speed_control.hpp"

namespace maxon_control {

  //Constructor
  SpeedControl::SpeedControl(ros::NodeHandle& nodeHandle) :
    nodeHandle_(nodeHandle)
  {
    if(!readParameters())
    {
      ROS_ERROR("Could not read parameters. Shutting down...");
      ros::requestShutdown();
    }
    else
    {
      ROS_INFO("Succesful parameter load.");
    }


    subscriber_=nodeHandle_.subscribe(subscriber_topic_,2,&SpeedControl::topicCallback,this);
    pub_pwm_=nodeHandle_.advertise<std_msgs::UInt8>("/pwm_value",1);
    pub_direction_=nodeHandle_.advertise<std_msgs::UInt8>("/motor_direction",1);

    
    pub_debug_init();

  }

  SpeedControl::~SpeedControl()
  {
  }



  bool SpeedControl::readParameters()
  {
    if(nodeHandle_.getParam("subscriber_topic", subscriber_topic_) && nodeHandle_.getParam("period",T_) && 
        nodeHandle_.getParam("imp_per_rot",imp_per_rot_)) return true;
    return false;
  }

  void SpeedControl::topicCallback(const std_msgs::Int16& msg)
  {
    int counter = msg.data;
    std_msgs::Int16 err_msg;
    std_msgs::Float64 speed_msg,rps_msg,rpm_msg,speed_err_msg,rps_err_msg,rpm_err_msg;
    std_msgs::UInt8 pwm_msg;
    double speed, rps, rpm, prev_speed, prev_rps, prev_rpm;


    err_=counter-prev_counter_;
    speed = err_ / T_;
    prev_speed = prev_err_/T_;
    
    rps = speed/imp_per_rot_;
    prev_rps = prev_speed/imp_per_rot_;

    rpm = rps*60;
    prev_rpm = prev_rps*60;

    err_msg.data = err_;
    speed_msg.data=speed;
    rps_msg.data=rps;
    rpm_msg.data=rpm;

    speed_err_msg.data = speed-prev_speed;
    rps_err_msg.data=rps-prev_rps;
    rpm_err_msg.data=rpm-prev_rpm;
    
    /** test slanja pwm-a **/
    pwm_msg.data = speed_;
    pub_pwm_.publish(pwm_msg);
    /**********************/
    
    pub_err_.publish(err_msg);
    pub_speed_.publish(speed_msg);
    pub_rps_.publish(rps_msg);
    pub_rpm_.publish(rpm_msg);
    
    pub_speed_err_.publish(speed_err_msg);
    pub_rps_err_.publish(rps_err_msg);
    pub_rpm_err_.publish(rpm_err_msg);

    prev_counter_=counter;
    p_prev_err_=prev_err_;
    prev_err_=err_;
  }


  void SpeedControl::callback_pid_reconfigure(maxon_control::ParametersConfig &config, uint32_t level)
  {
    speed_=config.speed;

    kp_=config.KP;
    ki_=config.KI;
    kd_=config.KD;
    rotation_ = config.cw_rotation;
    ROS_INFO("Reconfigure Request: %d %f %f %f %s", 
              speed_, kp_, ki_, kd_,rotation_?"True":"False"
              );
  }



  void SpeedControl::pub_debug_init()
  {
    pub_err_=nodeHandle_.advertise<std_msgs::Int16>("/err_value",1);
    pub_speed_=nodeHandle_.advertise<std_msgs::Float64>("/speed_value",1);
    pub_rps_=nodeHandle_.advertise<std_msgs::Float64>("/rps_value",1);
    pub_rpm_=nodeHandle_.advertise<std_msgs::Float64>("/rpm_value",1);

    pub_speed_err_=nodeHandle_.advertise<std_msgs::Float64>("/speed_err_value",1);
    pub_rps_err_=nodeHandle_.advertise<std_msgs::Float64>("/rps_err_value",1);
    pub_rpm_err_=nodeHandle_.advertise<std_msgs::Float64>("/rpm_err_value",1);
  }

}
