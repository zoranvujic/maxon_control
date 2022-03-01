#include "maxon_control/speed_control.hpp"

#define DOUBLE_TO_INT(f) ((int)(f >= 0.0 ? (f + 0.5) : (f - 0.5)))

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
        nodeHandle_.getParam("imp_per_rot",imp_per_rot_) && 
        nodeHandle_.getParam("i_limit",i_limit_)) return true;
    return false;
  }

void SpeedControl::topicCallback(const std_msgs::Int16& msg)
  {
    int counter = msg.data;
    int pwm=0;
    std_msgs::Int16 err_msg;
    std_msgs::Float64 rpm_msg,rpm_err_msg,u_msg;
    std_msgs::UInt8 pwm_msg;
    double speed, rps, rpm, prev_speed, prev_rps, prev_rpm, err_rpm, ref_rpm;

    ref_rpm = speed_;

    err_=counter-prev_counter_;
    speed = err_ / T_;
    prev_speed = prev_err_/T_;
    rps = speed/imp_per_rot_;
    prev_rps = prev_speed/imp_per_rot_;

    rpm = rps*60;
    prev_rpm = prev_rps*60;

    err_rpm = ref_rpm-rpm;

    if (ref_rpm < 0.1)
    {
        u_ = 0;
        pwm_msg.data = 0;
        pub_pwm_.publish(pwm_msg);
        dUi_ = 0.0;
    }else if (ref_rpm<120){
      if (abs(err_rpm) > 5)
      {
      
        pwm=pid_regulator(err_rpm);
        pwm_msg.data = pwm;
        pub_pwm_.publish(pwm_msg);
      }
    }else{
      if (abs(err_rpm) > 2.5)
      {
      
        pwm=pid_regulator(err_rpm);
        pwm_msg.data = pwm;
        pub_pwm_.publish(pwm_msg);
      }
    }

    p_prev_err_rpm_=prev_err_rpm_;
    prev_err_rpm_ = err_rpm;

    u_msg.data = u_;

    rpm_msg.data=rpm;
    
    rpm_err_msg.data=rpm-prev_rpm;
    
    pub_rpm_.publish(rpm_msg);
    
    pub_rpm_err_.publish(rpm_err_msg);

    pub_u_.publish(u_msg);

    prev_counter_=counter;
    p_prev_err_=prev_err_;
    prev_err_=err_;
  }

  int SpeedControl::pid_regulator(double err_rpm)
  {
    double dUp=0, dUi=0, dUd=0;
    int pwm;

    /**** PID incremental******
    dUp=kp_*(err_rpm-prev_err_rpm);
    dUi=ki_*T_*err_rpm;
    dUd=kd_/T_*(err_rpm - 2*prev_err_rpm + p_prev_err_rpm_);

    u_=u_ + dUp + dUi + dUd;
    

    ROS_INFO("%f %f %f %f %f %f %f",u_, dUp, dUi, dUd, err_rpm, prev_err_rpm,p_prev_err_rpm_);
    *************************/

    /*****PID********/
    dUp = kp_*err_rpm;
    dUi = dUi_ + ki_*err_rpm;
    dUd = kd_ * (err_rpm-prev_err_rpm_);

    if (dUi >= i_limit_)
    {
      dUi = i_limit_;
    }else if( dUi <= -i_limit_)
    {
      dUi=-i_limit_;
    }

    u_=u_+ dUp + dUi + dUd;

    ROS_INFO("%f %f %f %f %f",u_, dUp, dUi,dUd, err_rpm);
    dUi_=dUi; 
    /*****************/
    dUi_=dUi;

    if (u_ > 255.0)
    {
      u_=255;
    }else if(u_<0.0)
    {
      u_=0;
    }

    pwm = DOUBLE_TO_INT(u_);
    
    return pwm;
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
    pub_u_=nodeHandle_.advertise<std_msgs::Float64>("/u_value",1);
    pub_rpm_err_=nodeHandle_.advertise<std_msgs::Float64>("/rpm_err_value",1);
  }

}
