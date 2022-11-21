#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <mavros_msgs/PositionTarget.h>

#include "misc/vehicle_state.hpp"

struct Config
{
    std::string mapTopic;
    std::string targetTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double minThrust;
    double maxThrust;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;
    double timeFoward;

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("MapTopic", mapTopic);
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("DilateRadius", dilateRadius);
        nh_priv.getParam("VoxelWidth", voxelWidth);
        nh_priv.getParam("MapBound", mapBound);
        nh_priv.getParam("TimeoutRRT", timeoutRRT);
        nh_priv.getParam("MaxVelMag", maxVelMag);
        nh_priv.getParam("MaxBdrMag", maxBdrMag);
        nh_priv.getParam("MaxTiltAngle", maxTiltAngle);
        nh_priv.getParam("MinThrust", minThrust);
        nh_priv.getParam("MaxThrust", maxThrust);
        nh_priv.getParam("VehicleMass", vehicleMass);
        nh_priv.getParam("GravAcc", gravAcc);
        nh_priv.getParam("HorizDrag", horizDrag);
        nh_priv.getParam("VertDrag", vertDrag);
        nh_priv.getParam("ParasDrag", parasDrag);
        nh_priv.getParam("SpeedEps", speedEps);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("ChiVec", chiVec);
        nh_priv.getParam("SmoothingEps", smoothingEps);
        nh_priv.getParam("IntegralIntervs", integralIntervs);
        nh_priv.getParam("RelCostTol", relCostTol);
        nh_priv.getParam("TimeFoward", timeFoward);
    }
};

class GlobalPlanner
{
private:
    Config config;

    ros::NodeHandle nh;
    ros::Subscriber mapSub;
    ros::Subscriber targetSub;

    ros::Timer mainTimer, visTimer, collisionCheckTimer;

    bool mapInitialized;
    voxel_map::VoxelMap voxelMap;
    Visualizer visualizer;
    std::vector<Eigen::Vector3d> startGoal;
    Eigen::Vector3d commandGoal;

    Trajectory<5> traj;
    double trajStamp;

    VehicleState state;
    ros::Publisher map_pub, offboard_cmd_pub;

    Eigen::Vector3d cmdPosition;
    double cmdYaw;

public:
    GlobalPlanner(const Config &conf,
                  ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          mapInitialized(false),
          visualizer(nh),
          state(nh)
    {
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

        // external grid_map // TODO: integrate to voxel_map
        mapSub = nh.subscribe(config.mapTopic, 1, &GlobalPlanner::mapCallBack, this,
                              ros::TransportHints().tcpNoDelay());

        targetSub = nh.subscribe(config.targetTopic, 1, &GlobalPlanner::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());

        mainTimer = nh.createTimer(ros::Duration(0.01), &GlobalPlanner::mainLoop , this);
        visTimer = nh.createTimer(ros::Duration(0.05), &GlobalPlanner::visCallback, this);
        collisionCheckTimer = nh.createTimer(ros::Duration(0.25), &GlobalPlanner::collisionCheckCallback, this);

        map_pub = nh.advertise<sensor_msgs::PointCloud2>("/voxel_map/occupancy", 10);
        offboard_cmd_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    }

    inline void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        voxelMap.resetVoxel();

        size_t cur = 0;
        const size_t total = msg->data.size() / msg->point_step;
        float *fdata = (float *)(&msg->data[0]);

        // calc min/max bound
        Eigen::Vector3d min3d(voxelMap.getCorner());
        Eigen::Vector3d max3d(voxelMap.getOrigin());

        // set occupied
        for (size_t i = 0; i < total; i++)
        {
            cur = msg->point_step / sizeof(float) * i;

            if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
            {
                continue;
            }

            voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                 fdata[cur + 1],
                                                 fdata[cur + 2]));

            if (min3d(0) > fdata[cur + 0]) min3d(0) = fdata[cur + 0];
            if (min3d(1) > fdata[cur + 1]) min3d(1) = fdata[cur + 1];
            if (min3d(2) > fdata[cur + 2]) min3d(2) = fdata[cur + 2];
            if (max3d(0) < fdata[cur + 0]) max3d(0) = fdata[cur + 0];
            if (max3d(1) < fdata[cur + 1]) max3d(1) = fdata[cur + 1];
            if (max3d(2) < fdata[cur + 2]) max3d(2) = fdata[cur + 2];
        }
        
        int margin = 30;
        min3d = min3d.array() - margin * voxelMap.getScale();
        max3d = max3d.array() + margin * voxelMap.getScale();

        voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()), min3d, max3d);

        mapInitialized = true;
    }

    inline std::vector<Eigen::Vector3d> shrinkPath(const std::vector<Eigen::Vector3d> &path_in)
    {
        if (path_in.size() <= 2) return path_in;

        std::vector<Eigen::Vector3d> path_out;
        path_out.push_back(path_in.front());

        double eps = 1.0e-6;
        Eigen::Vector3d dir1, dir2;
        for (size_t i = 2; i < path_in.size(); i++)
        {
            dir1 = (path_in[i - 1] - path_out.back()).normalized();
            dir2 = (path_in[i] - path_in[i - 1]).normalized();
            if ((dir2 - dir1).norm() > eps)
            {
                path_out.push_back(path_in[i-1]);
            }
        }
        path_out.push_back(path_in.back());

        return path_out;
    }

    inline void plan()
    {
        if (startGoal.size() == 2)
        {
            ros::Time t[10];
            t[0] = ros::Time::now();

            std::vector<Eigen::Vector3d> route;
            sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],
                                                   startGoal[1],
                                                   voxelMap.getOrigin(),
                                                   voxelMap.getCorner(),
                                                   &voxelMap, 0.03,
                                                   route);
            route = shrinkPath(route);

            std::vector<Eigen::MatrixX4d> hPolys;
            std::vector<Eigen::Vector3d> pc;
            voxelMap.getSurf(pc);

            sfc_gen::convexCover(route,
                                 pc,
                                 voxelMap.getOrigin(),
                                 voxelMap.getCorner(),
                                 7.0,
                                 3.0,
                                 hPolys);
            sfc_gen::shortCut(hPolys);

            if (route.size() > 1)
            {
                visualizer.visualizePolytope(hPolys);

                Eigen::Matrix3d iniState;
                Eigen::Matrix3d finState;
                if (!state.initialized) {
                    iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                } else {
                    iniState << route.front(), state.vel, Eigen::Vector3d::Zero();
                }
                if (startGoal.back() != commandGoal) {
                    double finVelmag = std::min(config.maxVelMag, (commandGoal - startGoal.back()).norm());
                    Eigen::Vector3d finVel = (commandGoal - startGoal.back()).normalized() * finVelmag;
                    finState << route.back(), finVel, Eigen::Vector3d::Zero();
                } else {
                    finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                }

                gcopter::GCOPTER_PolytopeSFC gcopter;

                // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
                // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
                // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
                //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
                // initialize some constraint parameters
                Eigen::VectorXd magnitudeBounds(5);
                Eigen::VectorXd penaltyWeights(5);
                Eigen::VectorXd physicalParams(6);
                magnitudeBounds(0) = config.maxVelMag;
                magnitudeBounds(1) = config.maxBdrMag;
                magnitudeBounds(2) = config.maxTiltAngle;
                magnitudeBounds(3) = config.minThrust;
                magnitudeBounds(4) = config.maxThrust;
                penaltyWeights(0) = (config.chiVec)[0];
                penaltyWeights(1) = (config.chiVec)[1];
                penaltyWeights(2) = (config.chiVec)[2];
                penaltyWeights(3) = (config.chiVec)[3];
                penaltyWeights(4) = (config.chiVec)[4];
                physicalParams(0) = config.vehicleMass;
                physicalParams(1) = config.gravAcc;
                physicalParams(2) = config.horizDrag;
                physicalParams(3) = config.vertDrag;
                physicalParams(4) = config.parasDrag;
                physicalParams(5) = config.speedEps;
                const int quadratureRes = config.integralIntervs;

                traj.clear();

                if (!gcopter.setup(config.weightT,
                                   iniState, finState,
                                   hPolys, INFINITY,
                                   config.smoothingEps,
                                   quadratureRes,
                                   magnitudeBounds,
                                   penaltyWeights,
                                   physicalParams))
                {
                    return;
                }

                if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
                {
                    return;
                }

                if (traj.getPieceNum() > 0)
                {
                    trajStamp = ros::Time::now().toSec() - config.timeFoward;
                    visualizer.visualize(traj, route);
                }

                t[1] = ros::Time::now();
                std::cout << "[compute time] plan : " << (t[1] - t[0]).toSec() << " s" << std::endl;
            }
        }
    }

    inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (mapInitialized)
        {
            const double zGoal = state.initialized ? state.pos.z() : config.mapBound[4] + config.dilateRadius +
                                 fabs(msg->pose.orientation.z) *
                                     (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius);
            Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);
            if (voxelMap.query(goal) == 0)
            {
                if (startGoal.size() != 1)
                {
                    if (voxelMap.query(state.pos) != 0) {
                        ROS_WARN("Current position is infeasible!");
                        return;
                    } else if (state.initialized) {
                        startGoal.clear();
                        startGoal.emplace_back(state.pos);
                    }
                }
                commandGoal = goal;
                visualizer.visualizeStartGoal(goal, 0.5, 1);

                Eigen::Vector3d sub_vector = goal - startGoal.front();
                double max_dist = 20.0;
                if (sub_vector.norm() > max_dist)
                {
                    goal = startGoal.front() + sub_vector.normalized() * max_dist;
                }
                startGoal.emplace_back(goal);

                plan();
            }
            else
            {
                ROS_WARN("Infeasible Position Selected !!!\n");
            }
        }
        return;
    }

    inline void visCallback(const ros::TimerEvent & /*event*/)
    {
        publishMap();
    }

    inline void publishMap()
    {
        if (map_pub.getNumSubscribers() <= 0)
            return;

        pcl::PointCloud<pcl::PointXYZ> cloud;

        voxelMap.iterateVoxelPosVal([&](Eigen::Vector3d pos, uint8_t val) {
            if (val != 0) {
                cloud.points.emplace_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
            }
        });

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = "map";

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        map_pub.publish(cloud_msg);
    }

    inline void mainLoop(const ros::TimerEvent & /*event*/)
    {
        // process();
        movingVoxel();
        offboardCommand();
    }

    inline void process()
    {
        Eigen::VectorXd physicalParams(6);
        physicalParams(0) = config.vehicleMass;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;

        flatness::FlatnessMap flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                      physicalParams(3), physicalParams(4), physicalParams(5));

        if (traj.getPieceNum() > 0)
        {
            const double delta = ros::Time::now().toSec() - trajStamp;
            if (delta > 0.0 && delta < traj.getTotalDuration())
            {
                double thr;
                Eigen::Vector4d quat;
                Eigen::Vector3d omg;

                flatmap.forward(traj.getVel(delta),
                                traj.getAcc(delta),
                                traj.getJer(delta),
                                0.0, 0.0,
                                thr, quat, omg);
                double speed = traj.getVel(delta).norm();
                double bodyratemag = omg.norm();
                double tiltangle = acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)));
                std_msgs::Float64 speedMsg, thrMsg, tiltMsg, bdrMsg;
                speedMsg.data = speed;
                thrMsg.data = thr;
                tiltMsg.data = tiltangle;
                bdrMsg.data = bodyratemag;
                visualizer.speedPub.publish(speedMsg);
                visualizer.thrPub.publish(thrMsg);
                visualizer.tiltPub.publish(tiltMsg);
                visualizer.bdrPub.publish(bdrMsg);

                visualizer.visualizeSphere(traj.getPos(delta),
                                           config.dilateRadius);

                state.Update(traj.getPos(delta), traj.getVel(delta), quat);
            }
        }
    }

    inline void movingVoxel()
    {
        Eigen::Vector3d change_unit(10, 10, 2.5);
        Eigen::Vector3d test_orig = voxelMap.posI2D(voxelMap.getSize() / 2);
        Eigen::Vector3d test_new = state.pos;
        Eigen::Vector3i test_res((int)round((test_new.x() - test_orig.x()) / change_unit.x()),
                                 (int)round((test_new.y() - test_orig.y()) / change_unit.y()),
                                 (int)round((test_new.z() - test_orig.z()) / change_unit.z()));

        if (abs(test_new.x() - test_orig.x()) > change_unit.x() * 2.0 / 3.0) {
            std::cout << "changeOrigin - X direction" << std::endl;
            voxelMap.changeOrigin(Eigen::Vector3d((double)test_res.x() * change_unit.x(), 0, 0));
        } else if (abs(test_new.y() - test_orig.y()) > change_unit.y() * 2.0 / 3.0) {
            std::cout << "changeOrigin - Y direction" << std::endl;
            voxelMap.changeOrigin(Eigen::Vector3d(0, (double)test_res.y() * change_unit.y(), 0));
        } else if (abs(test_new.z() - test_orig.z()) > change_unit.z() * 2.0 / 3.0) {
            std::cout << "changeOrigin - Z direction" << std::endl;
            voxelMap.changeOrigin(Eigen::Vector3d(0, 0, (double)test_res.z() * change_unit.z()));
        }
    }
    
    inline void offboardCommand()
    {
        mavros_msgs::PositionTarget msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.coordinate_frame = 1;

        // ignore velocity, acceleration, force, and yawrate reference
        msg.type_mask |= msg.IGNORE_VX | msg.IGNORE_VY | msg.IGNORE_VZ;
        msg.type_mask |= msg.IGNORE_AFX | msg.IGNORE_AFY | msg.IGNORE_AFZ;
        msg.type_mask |= msg.FORCE;
        msg.type_mask |= msg.IGNORE_YAW_RATE;

        if (traj.getPieceNum() > 0)
        {
            const double delta = ros::Time::now().toSec() - trajStamp;
            if (delta > 0.0 && delta < traj.getTotalDuration())
            {
                float cmdYaw_temp = cmdYaw;
                if (traj.getVel(delta).norm() > 0.1)
                {
                    cmdYaw = atan2(traj.getVel(delta).y(), traj.getVel(delta).x());
                }

                if (cos(cmdYaw - cmdYaw_temp) > 0.5)
                {
                    cmdPosition.x() = traj.getPos(delta).x();
                    cmdPosition.y() = traj.getPos(delta).y();
                    cmdPosition.z() = traj.getPos(delta).z();
                } 
            }
        }
        else if (state.initialized)
        {
            cmdPosition = state.pos;
            cmdYaw = state.yaw;
        }

        msg.position.x = cmdPosition.x();
        msg.position.y = cmdPosition.y();
        msg.position.z = cmdPosition.z();
        msg.yaw = cmdYaw;

        offboard_cmd_pub.publish(msg);
        visualizer.visualizeSphere(cmdPosition, 0.4);
    }

    inline void collisionCheckCallback(const ros::TimerEvent & /*event*/)
    {
        if (startGoal.size() < 2) return;
        if (voxelMap.query(startGoal.front()) != 0) return;
        if (voxelMap.query(startGoal.back()) != 0)
        {
            ROS_WARN("Goal position is infeasible!");
            return;
        }
        if (!state.initialized) return;

        Eigen::Vector3d last_safety_pos = startGoal.front();
        bool need_replan = false;
        if (traj.getPieceNum() > 0)
        {
            const double delta = ros::Time::now().toSec() - trajStamp;
            if (delta + config.timeFoward > traj.getTotalDuration() && commandGoal != startGoal.back())
            {
                need_replan = true;
            }
            else
            {
                const double dt = 0.05;
                Eigen::Vector3i pos_p = voxelMap.posD2I(traj.getPos(0));

                for (double t = dt; t < traj.getTotalDuration(); t += dt)
                {
                    Eigen::Vector3i pos_n = voxelMap.posD2I(traj.getPos(t));
                    if (pos_p != pos_n)
                    {
                        if (voxelMap.query(pos_n) != 0)
                        {
                            need_replan = true;
                            break;
                        }
                        last_safety_pos = voxelMap.posI2D(pos_p);
                        pos_p = pos_n;
                    }
                }
            }
        }

        if (need_replan)
        {
            ROS_WARN("Replan!");
            if (voxelMap.query(state.pos) == 0)
            {
                startGoal.front() = state.pos;
            }
            else
            {
                ROS_WARN("use last_safety_pos.");
                startGoal.front() = last_safety_pos;
            }

            Eigen::Vector3d sub_vector = commandGoal - startGoal.front();
            double max_dist = 20.0;
            if (sub_vector.norm() > max_dist)
            {
                startGoal.back() = startGoal.front() + sub_vector.normalized() * max_dist;
            }
            else
            {
                startGoal.back() = commandGoal;
            }
            visualizer.visualizeStartGoal(commandGoal, 0.5, 1);
            plan();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gcopter_realtime_node");
    ros::NodeHandle nh("~");

    GlobalPlanner global_planner(Config(nh), nh);

    ros::spin();

    return 0;
}
