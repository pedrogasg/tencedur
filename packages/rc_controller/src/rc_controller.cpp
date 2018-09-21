#include "rc_controller/rc_controller.h"

namespace rc_controller
{

    RCController::RCController()
    {
    }

    bool RCController::RobotModel::init(ros::NodeHandle& nh)
    {
        const int QUEUE_SIZE = 1;
        robot_model_ = std::make_unique<RobotModel>();
        rc_pub_ = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", QUEUE_SIZE);
        if(!rc_pub_)
        {
            ROS_INFO("Could not advertise to /mavros/rc/override");
            return false;
        }
        robot_model_ = std::make_unique<RobotModel>();
        arming_service_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        setmode_service_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        
        state_sub_ = nh.subscribe<mavros_msgs::State>("/mavros/state", QUEUE_SIZE, &RCController::stateCallback, this);
        if(!state_sub_)
        {
            ROS_INFO("Could not subscribe to /mavros/state");
            return false;
        }

        local_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", QUEUE_SIZE, &RCController::poseCallback, this);
        if(!local_pose_sub_)
        {
            ROS_INFO("Could not subscribe to /mavros/local_position/pose");
            return false;
        }
        
        local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", QUEUE_SIZE);
        if(!local_pose_pub_)
        {
            ROS_INFO("Could not advertise to /mavros/setpoint_position/local");
            return false;
        }
        return true;
    }

    bool RCController::arm()
    {
        ros::Rate rate(spin_rate_);

        ROS_INFO("Waiting for FCU board...");

        while (ros::ok() && current_state_.connected)
        {
            ros::spinOnce();
            rate.sleep();
        }

        ROS_INFO("Connected to FCU board.");


        geometry_msgs::PoseStamped pose;

        pose.header.stamp    = ros::Time::now();
        pose.header.frame_id = 1;
        pose.header.seq      = 0;

        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;

        ROS_INFO("Stabilize current pose before warmup...");

        for (int i = 100; ros::ok() && i > 0; --i)
        {
            ros::spinOnce();
            rate.sleep();
            geometry_msgs::PoseStamped current_pose = current_pose_; 
            pose.pose.position.x = smoothFilter(current_pose.pose.position.x, pose.pose.position.x, smooth_factor_);
            pose.pose.position.y = smoothFilter(current_pose.pose.position.y, pose.pose.position.y, smooth_factor_);
            pose.pose.position.z = smoothFilter(current_pose.pose.position.z, pose.pose.position.z, smooth_factor_);
        }

        ROS_INFO("Sending warmup messages...");

        for (int i = 100; ros::ok() && i > 0; --i)
        {
            pose.header.stamp    = ros::Time::now();
            pose.header.frame_id = 1;
            local_pose_pub_.publish(pose);

            ros::spinOnce();
            rate.sleep();
        }

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        ros::Time last_request = ros::Time::now();
        ros::Time init_start   = ros::Time::now();

        while(ros::ok() && (ros::Time::now() - init_start < ros::Duration(wait_for_services_)))
        {
            if( current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(wait_for_service_)))
            {
                if( setmode_service_.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
            } 
            else 
            {
                if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(wait_for_service_)))
                {
                    if( arming_service_.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle armed");
                    }
                }
            }
            last_request = ros::Time::now();
            if( current_state_.mode != "OFFBOARD" || !current_state.armed)
            {
                pose.pose.position.x = smoothFilter(current_pose.pose.position.x, pose.pose.position.x, smooth_factor_);
                pose.pose.position.y = smoothFilter(current_pose.pose.position.y, pose.pose.position.y, smooth_factor_);
                pose.pose.position.z = smoothFilter(current_pose.pose.position.z, pose.pose.position.z, smooth_factor_);
            }
            else
            {
                return true;
            }
            pose.header.stamp    = ros::Time::now();
            pose.header.frame_id = 1;
            local_pos_pub.publish(pose);

            ros::spinOnce();
            rate.sleep();
        }

        return false;

    }

    void RCController::spin()
    {
        ros::Rate rate(spin_rate_);
        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
        
    }

    void RCController::stateCallback(const mavros_msgs::State::ConstPtr &msg)
    {
        current_state_ = *msg;
    }

    void RCController::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        current_pose_ = *msg;
    }

    inline float RCController::smoothFilter(float cur, float prev, float factor) const
    {
        return factor * cur + (1.0f - factor) * prev;
    }

}

