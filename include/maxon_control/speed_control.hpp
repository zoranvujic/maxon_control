#pragma once

#include <ros/ros.h>

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

            

            void incremental_pid();
            void pub_debug_init();


            ros::NodeHandle nodeHandle_;
            ros::Publisher pub_direction_;
            ros::Publisher pub_pwm_;
            
            ros::Subscriber subscriber_;

            /**********DEBUG************/ 
            ros::Publisher pub_err_;
            ros::Publisher pub_speed_;
            ros::Publisher pub_rps_;
            ros::Publisher pub_rpm_;

            ros::Publisher pub_speed_err_;
            ros::Publisher pub_rps_err_;
            ros::Publisher pub_rpm_err_;
            /****************************/
            

            std::string subscriber_topic_;

            int speed_=0;
            signed short int err_=0; 
            signed short int prev_err_=0;
            int p_prev_err_=0;
            int prev_counter_=0; 
            double u_=0.0;
            double kp_=0.0, ki_=0.0, kd_=0.0;

            double T_; //50ms
            int imp_per_rot_ ;
            bool rotation_ ;
    };
}
