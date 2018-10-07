#include "rc_controller/rc_offboard_controller.h"

namespace rc_controller
{

    RCOffboardController::RCOffboardController()
    {
    }


    bool RCOffboardController::init(ros::NodeHandle& nh)
    {
        const int QUEUE_SIZE = 10;
        
        arming_service_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        setmode_service_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        
        state_sub_ = nh.subscribe<mavros_msgs::State>("/mavros/state", QUEUE_SIZE, &RCOffboardController::stateCallback, this);
        if(!state_sub_)
        {
            ROS_INFO("Could not subscribe to /mavros/state");
            return false;
        }

        local_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", QUEUE_SIZE);
        if(!local_pose_pub_)
        {
            ROS_INFO("Could not advertise to /mavros/setpoint_position/local");
            return false;
        }
        return true;
    }

    bool RCOffboardController::arm()
    {
        //the setpoint publishing rate MUST be faster than 2Hz
        ros::Rate rate(20.0);

        // wait for FCU connection
        while(ros::ok() && !current_state_.connected){
            ros::spinOnce();
            rate.sleep();
        }

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2;

        //send a few setpoints before starting
        for(int i = 100; ros::ok() && i > 0; --i){
            local_pose_pub_.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        ros::Time last_request = ros::Time::now();

        while(ros::ok()){
            if( current_state_.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( setmode_service_.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if( !current_state_.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( arming_service_.call(arm_cmd) &&
                        arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                        return true;
                    }
                    last_request = ros::Time::now();
                }
            }

            local_pose_pub_.publish(pose);

            ros::spinOnce();
            rate.sleep();
        }

        return false;

    }

    void RCOffboardController::spin()
    {
        ros::Rate rate(spin_rate_);
        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
        
    }

    void RCOffboardController::stateCallback(const mavros_msgs::State::ConstPtr &msg)
    {
        current_state_ = *msg;
        ROS_INFO("%s/n", current_state_.mode.c_str());
    }

    inline float RCOffboardController::smoothFilter(float cur, float prev, float factor) const
    {
        return factor * cur + (1.0f - factor) * prev;
    }

}

