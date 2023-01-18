#include <iostream>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

// VehicleState for the planner
class VehicleState
{
private:
    ros::NodeHandle nh;
    ros::Subscriber odomSub, stateSub;
    ros::ServiceClient armingClient, setmodeClient;
    double takeoff_land_speed = 0.5;

public:
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;

    Eigen::Quaterniond att;
    double yaw;

    bool initialized = false;

    bool connected = false;
    bool armed = false;
    bool manual_input = false;
    std::string mode = "NONE";

public:
    VehicleState(ros::NodeHandle &nh_)
        : nh(nh_)
    {

        odomSub = nh.subscribe("/mavros/local_position/odom", 1, &VehicleState::odomCallback, this,
                               ros::TransportHints().tcpNoDelay());
        stateSub = nh.subscribe("/mavros/state", 1, &VehicleState::stateCallback, this,
                                ros::TransportHints().tcpNoDelay());
        armingClient = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        setmodeClient = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    }

    void Update(Eigen::Vector3d position, Eigen::Vector3d velocity, Eigen::Quaterniond attitude)
    {
        pos = position;
        vel = velocity;
        att = attitude;
        initialized = true;
    }

    void Update(Eigen::Vector3d position, Eigen::Vector3d velocity, Eigen::Vector4d attitude)
    {
        Update(position, velocity, Eigen::Quaterniond(attitude(0), attitude(1), attitude(2), attitude(3)));
    }

    void Update(Eigen::Vector3d position, Eigen::Vector3d velocity)
    {
        Update(position, velocity, Eigen::Quaterniond(0, 0, 0, 1));
    }

    Eigen::Vector3d QuaterniontoEuler(Eigen::Quaterniond quat)
    {
        // roll (x-axis rotation)
        double t0 = +2.0 * (quat.w() * quat.x() + quat.y() * quat.z());
        double t1 = +1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y());
        double roll = std::atan2(t0, t1);

        // pitch (y-axis rotation)
        double t2 = +2.0 * (quat.w() * quat.y() - quat.z() * quat.x());
        t2 = t2 > 1.0 ? 1.0 : t2;
        t2 = t2 < -1.0 ? -1.0 : t2;
        double pitch = std::asin(t2);

        // yaw (z-axis rotation)
        double t3 = +2.0 * (quat.w() * quat.z() + quat.x() * quat.y());
        double t4 = +1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z());
        double yaw = std::atan2(t3, t4);

        return Eigen::Vector3d(roll, pitch, yaw);
    }

    void odomCallback(const nav_msgs::OdometryConstPtr &odom)
    {
        Eigen::Vector3d position, velocity;
        Eigen::Quaterniond attitude;
        position.x() = odom->pose.pose.position.x;
        position.y() = odom->pose.pose.position.y;
        position.z() = odom->pose.pose.position.z;

        velocity.x() = odom->twist.twist.linear.x;
        velocity.y() = odom->twist.twist.linear.y;
        velocity.z() = odom->twist.twist.linear.z;

        attitude = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                      odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

        velocity = attitude * velocity; // fix for PX4
        yaw = QuaterniontoEuler(attitude).z();

        Update(position, velocity, attitude);
    }

    void stateCallback(const mavros_msgs::State &state)
    {
        connected = state.connected;
        armed = state.armed;
        manual_input = state.manual_input;
        mode = state.mode;
    }

    // true -> arming, false -> disarming
    bool commandArm(bool arm)
    {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = arm;

        if (armingClient.call(arm_cmd) && arm_cmd.response.success)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool setMode(std::string mode)
    {
        mavros_msgs::SetMode set_mode;
        set_mode.request.custom_mode = mode;

        if (setmodeClient.call(set_mode) && set_mode.response.mode_sent)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};
