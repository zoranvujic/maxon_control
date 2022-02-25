#pragma once

#include <ros/ros.h>
#include <std_msgs/Int16.h>

namespace maxon_control {

    class SpeedControl{
        public:
            SpeedControl(ros::NodeHandle& nodeHandle);

            ~SpeedControl();

        private:
            ros::NodeHandle nodeHandle_;
            ros::Publisher publisher_;
            ros::Subscriber subscriber_;
            void topicCallback(const std_msgs::Int16& msg);
            
           

    };

}
