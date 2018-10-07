#ifndef RC_CONTROLLER_RC_OFFBOARD_CONTROLLER_H
#define RC_CONTROLLER_RC_RC_OFFBOARD_CONTROLLER_H

#include <math.h>

#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>

#include <tf/tf.h>
#include <tf2/buffer_core.h>
#include <tf2/LinearMath/Transform.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>

#include <algorithm>
#include <memory>

namespace rc_controller
{
    class RCOffboardController
    {
        public:
            RCOffboardController();
            ~RCOffboardController() = default;

               // public methods
            bool init(ros::NodeHandle& nh);
            bool arm();
            void spin();

            // subcriptions callback
            void stateCallback(const mavros_msgs::State::ConstPtr &msg);
   
        private:
            float smoothFilter(float cur, float prev, float factor) const;

        private:
            // Ros parameters
            float spin_rate_         = 20.0;
            float smooth_factor_     = 0.15f;
            float wait_for_services_ = 30.0;
            float wait_for_service_  = 5.0;
            // Saved states messages 
            mavros_msgs::State current_state_;
            geometry_msgs::PoseStamped current_pose_;
            // Ros services 
            ros::ServiceClient arming_service_;
            ros::ServiceClient setmode_service_;
            // Ros subscribers
            ros::Subscriber state_sub_;
            // Ros publishers
            ros::Publisher local_pose_pub_;

    };
}
#endif