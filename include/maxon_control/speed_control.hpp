#pragma once

#include <ros/ros.h>
#include <cmath> 

#include <dynamic_reconfigure/server.h>
#include <maxon_control/ParametersConfig.h>

#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>



namespace maxon_control {

    class SpeedControl{
        public:
            SpeedControl(ros::NodeHandle& nodeHandle);

            ~SpeedControl();

            void callback_pid_reconfigure(maxon_control::ParametersConfig &config, uint32_t level);
        private:
            bool readParameters();
            void topicCallback(const std_msgs::Int16& msg);
	    void speed_ref_callback(const std_msgs::Int16& msg);
            

            int pid_regulator(double err_rpm);
            void pub_debug_init();


            ros::NodeHandle nodeHandle_;
            ros::Publisher pub_direction_;
            ros::Publisher pub_pwm_;
            
            ros::Subscriber subscriber_;
	    ros::Subscriber subscriber_speed_;
            //debug 
            ros::Publisher pub_err_;
            ros::Publisher pub_speed_;
            ros::Publisher pub_rps_;
            ros::Publisher pub_rpm_;

            ros::Publisher pub_speed_err_;
            ros::Publisher pub_rpm_err_;
            ros::Publisher pub_u_;
            

            

            std::string subscriber_topic_;

            int speed_=0;
            signed short int err_=0; 
            signed short int prev_err_=0;
            int p_prev_err_=0;
            int prev_counter_=0; 
            double u_=0.0;
            double kp_=0.0, ki_=0.0, kd_=0.0;
            double prev_err_rpm_=0.0, p_prev_err_rpm_=0.0;
            double dUi_=0.0;

            double T_; //50ms
            int imp_per_rot_ ;
            double i_limit_;
            bool rotation_ ;
    };

}
