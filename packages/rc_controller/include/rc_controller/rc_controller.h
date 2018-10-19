/**
    rc_controller ROS node. 

    Authors/maintainers: Pedro Fillastre
*/

#ifndef RC_CONTROLLER_RC_CONTROLLER_H
#define RC_CONTROLLER_RC_CONTROLLER_H

#include <math.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>

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
    class RCController
    {
        public:
            RCController();
            ~RCController() = default;

            // public methods
            bool init(ros::NodeHandle& nh);
            bool arm();
            void spin();

            // subcriptions callback
            void stateCallback(const mavros_msgs::State::ConstPtr &msg);
            void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    
        private:
            class RobotModel
            {
                public:
                    ~RobotModel() = default;
                    std::string getName()             { return "Tencendur"; }
                    std::string getOffboardModeName() { return "OFFBOARD"; }
                    bool init(ros::NodeHandle& nh);
                private:
                    // Ros publishers
                    ros::Publisher rc_pub_;
                    // RC parameters
                    // Steering
                    int rc_steer_trim_ = 1500;
                    int rc_steer_dz_   = 30;
                    int rc_steer_min_  = 1100;
                    int rc_steer_max_  = 1900;
                    // Throttle
                    int rc_throttle_trim_ = 1500;
                    int rc_throttle_dz_   = 30;
                    int rc_throttle_min_  = 1100;
                    int rc_throttle_max_  = 1900;
            };

        private:
            float smoothFilter(float cur, float prev, float factor) const;
            geometry_msgs::Point computeNextPosePosition(
                geometry_msgs::PoseStamped& current_pose,
                float linear_control_val,
                float angular_control_val,
                float linear_speed ) const;

        private:
            // Ros parameters
            float spin_rate_         = 20.0;
            float smooth_factor_     = 0.15f;
            float wait_for_services_ = 30.0;
            float wait_for_service_  = 5.0;
            // Internal objects
            std::unique_ptr<RobotModel> robot_model_;
            // Saved states messages 
            mavros_msgs::State current_state_;
            geometry_msgs::PoseStamped current_pose_;
            // Ros services 
            ros::ServiceClient arming_service_;
            ros::ServiceClient setmode_service_;
            // Ros subscribers
            ros::Subscriber state_sub_;
            ros::Subscriber local_pose_sub_;
            // Ros publishers
            ros::Publisher local_pose_pub_;
    };
}
#endif