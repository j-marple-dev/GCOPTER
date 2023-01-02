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
#include <std_msgs/String.h>

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
#include <mavros_msgs/RCIn.h>

#include "misc/vehicle_state.hpp"
#include "path_searching/dyn_a_star.h"

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
    double timeFoward = 0;

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
    
    AStar::Ptr a_star_;

    ros::Subscriber rcdemoSub, rcinSub;
    double rc_input[4] = {0}; // normalized values;
    ros::Time last_rc_time;
    ros::Timer calcWPTimer;
    enum ControlState {
        Hover,
        Manual,
        Assistance,
        RVizInput
    } control_state;
    double cruiseVel;

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

        rcdemoSub = nh.subscribe("/rc_demo", 10, &GlobalPlanner::rcdemoCallback, this, ros::TransportHints().tcpNoDelay());
        rcinSub = nh.subscribe("/mavros/rc/in", 10, &GlobalPlanner::rcinCallback, this, ros::TransportHints().tcpNoDelay());

        mainTimer = nh.createTimer(ros::Duration(0.01), &GlobalPlanner::mainLoop , this);
        visTimer = nh.createTimer(ros::Duration(0.05), &GlobalPlanner::visCallback, this);
        // collisionCheckTimer = nh.createTimer(ros::Duration(0.25), &GlobalPlanner::collisionCheckCallback, this);
        calcWPTimer = nh.createTimer(ros::Duration(0.5), &GlobalPlanner::calcWaypointCallback, this);

        map_pub = nh.advertise<sensor_msgs::PointCloud2>("/voxel_map/occupancy", 10);
        offboard_cmd_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
        
        a_star_.reset(new AStar);
        a_star_->initGridMap(voxel_map::VoxelMap::Ptr(&voxelMap), Eigen::Vector3i(256, 256, 256));

        control_state = ControlState::Hover;
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

    inline std::vector<Eigen::Vector3d> shrinkPath(const std::vector<Eigen::Vector3d> &path_in, const bool enable_diag = false)
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

        if (path_out.size() > 2 && enable_diag)
        {
            std::vector<Eigen::Vector3d> path_diag;
            path_diag.push_back(path_out.front());

            auto ray_cast = [&](const Eigen::Vector3d &start, const Eigen::Vector3d &end, const double dt) {
                const double eps = 1.0e-6;
                Eigen::Vector3d sub = (end - start);
                const double length = sub.norm();
                const Eigen::Vector3d dir = sub.normalized();
                Eigen::Vector3i pos_p = voxelMap.posD2I(start);

                for (double t = dt; t < length + eps; t += dt)
                {
                    Eigen::Vector3i pos_n = voxelMap.posD2I(start + dir * t);
                    if (pos_p != pos_n)
                    {
                        if (voxelMap.query(pos_n) != 0)
                        {
                            return voxelMap.posI2D(pos_n);
                        }
                        pos_p = pos_n;
                    }
                }

                return end;
            };

            for (size_t i = 2; i < path_out.size(); i++)
            {
                if (path_out[i] != ray_cast(path_diag.back(), path_out[i], 0.05))
                {
                    path_diag.push_back(path_out[i-1]);
                }
            }
            path_diag.push_back(path_out.back());

            return path_diag;
        }
        else
        {
            return path_out;
        }
    }

    inline void plan()
    {
        if (startGoal.size() == 2)
        {
            ros::Time t[10];
            t[0] = ros::Time::now();

            Eigen::Vector3d iniVel = Eigen::Vector3d::Zero();
            Eigen::Vector3d iniAcc = Eigen::Vector3d::Zero();
            if (traj.getPieceNum() > 0)
            {
                const double delta = ros::Time::now().toSec() - trajStamp;
                if (delta > 0.0 && delta < traj.getTotalDuration())
                {
                    if (voxelMap.query(traj.getPos(delta)) == 0)
                    {
                        startGoal[0] = traj.getPos(delta);
                        iniVel = traj.getVel(delta);
                        iniAcc = traj.getAcc(delta);
                    }
                }
            }

            std::vector<Eigen::Vector3d> route;
            if (a_star_->AstarSearch(voxelMap.getScale(), startGoal[0], startGoal[1])) {
                std::vector<Eigen::Vector3d> a_star_path = a_star_->getPath();
                route = shrinkPath(a_star_path, true);
            }
            else
            {
                ROS_WARN("a_star fail! Path deleted!");
                t[1] = ros::Time::now();
                std::cout << "[compute time] plan : " << (t[1] - t[0]).toSec() << " s" << std::endl;
                startGoal.clear();
                traj.clear();
                visualizer.deletePlans();
                return;
            }

            if (route.size() > 1)
            {
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

                visualizer.visualizePolytope(hPolys);

                Eigen::Matrix3d iniState;
                Eigen::Matrix3d finState;
                if (!state.initialized) {
                    iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                } else {
                    iniState << startGoal[0], iniVel, iniAcc;
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
                magnitudeBounds(0) = (control_state == ControlState::Assistance) ? cruiseVel : config.maxVelMag;
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
            else
            {
                traj.clear();
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
                else if (sub_vector.norm() < voxelMap.getScale())
                {
                    ROS_WARN("Goal position is too close to Start !!!\n");
                    return;
                }
                startGoal.emplace_back(goal);

                plan();
                control_state = ControlState::RVizInput;
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
        if (state.initialized)
        {
            visualizer.visualizePath(state.pos, 200);
        }
    }

    inline void publishMap()
    {
        if (map_pub.getNumSubscribers() <= 0)
            return;

        pcl::PointCloud<pcl::PointXYZ> cloud;

        std::vector<Eigen::Vector3d> pc;
        voxelMap.getSurf(pc);
        for (auto p : pc) cloud.points.emplace_back(p(0), p(1), p(2));

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
        movingVoxel();
        offboardCommand();
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
        if (!state.initialized) return;

        mavros_msgs::PositionTarget msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.coordinate_frame = 1;

        // ignore velocity, acceleration, force, and yawrate reference
        msg.type_mask |= msg.IGNORE_VX | msg.IGNORE_VY | msg.IGNORE_VZ;
        msg.type_mask |= msg.IGNORE_AFX | msg.IGNORE_AFY | msg.IGNORE_AFZ;
        msg.type_mask |= msg.FORCE;
        msg.type_mask |= msg.IGNORE_YAW_RATE;

        if (control_state == ControlState::Hover)
        {
            if ((cmdPosition - state.pos).norm() > 0.5)
                cmdPosition = state.pos;
            if (cos(cmdYaw - state.yaw) < 0.8)
                cmdYaw = state.yaw;
        }
        else if (control_state == ControlState::Manual)
        {
            if ((ros::Time::now() - last_rc_time).toSec() > 1.0)
            {
                control_state = ControlState::Hover;
                return;
            }

            cmdPosition = state.pos;
            cmdYaw = state.yaw;

            Eigen::Vector3d control(rc_input[0] * 2.0, rc_input[1] * 2.0, rc_input[2]);
            cmdPosition += Eigen::AngleAxisd(state.yaw, Eigen::Vector3d::UnitZ()) * control;
            cmdYaw += rc_input[3] * 0.5;
        }
        else if (control_state == ControlState::Assistance)
        {
            if ((ros::Time::now() - last_rc_time).toSec() > 1.0)
            {
                control_state = ControlState::Hover;
                return;
            }

            cmdPosition = state.pos;
            cmdYaw = state.yaw;

            if (traj.getPieceNum() > 0)
            {
                const double delta = ros::Time::now().toSec() - trajStamp;
                if (delta > 0.0 && delta < traj.getTotalDuration())
                {
                    cmdPosition.x() = traj.getPos(delta).x();
                    cmdPosition.y() = traj.getPos(delta).y();
                    cmdPosition.z() = traj.getPos(delta).z();
                }

                cmdYaw += rc_input[3] * 0.5;
            }
            else
            {
                cmdPosition.z() += rc_input[2] * 1.0;
                cmdYaw += rc_input[3] * 0.5;
            }
        }
        else if (control_state == ControlState::RVizInput)
        {
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
            else
            {
                control_state = ControlState::Hover;
            }
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
            startGoal.clear();
            traj.clear();
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
            else if (delta > traj.getTotalDuration())
            {
                ROS_WARN("Since vehicle has arrived at goal, remove traj.");
                startGoal.clear();
                traj.clear();
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

            if (voxelMap.query(startGoal.back()) != 0)
            {
                ROS_WARN("Goal position is infeasible!");
                startGoal.clear();
                traj.clear();
                return;
            }

            visualizer.visualizeStartGoal(commandGoal, 0.5, 1);
            plan();
        }
    }

    std::vector<std::string> split(std::string str)
    {
        std::string temp = "";
        std::vector<std::string> ret;
        for (auto x : str) {
            if (x == ',') {
                ret.push_back(temp);
                temp = "";
            }
            else {
                temp = temp + x;
            }
        }

        ret.push_back(temp);
        return ret;
    }

    void rcdemoCallback(const std_msgs::String& msg)
    {
        auto splited = split(msg.data);
        if (splited.size() == 0) return;

        if (splited.front() == "c")
        {
            rc_input[0] = std::stod(splited[2]);
            rc_input[1] = std::stod(splited[1]);
            rc_input[2] = std::stod(splited[4]);
            rc_input[3] = std::stod(splited[3]);
            last_rc_time = ros::Time::now();
            control_state = ControlState::Manual;

            if (traj.getPieceNum() > 0) traj.clear();
        }
        else if (splited.front() == "w")
        {
            rc_input[0] = std::stod(splited[2]);
            rc_input[1] = std::stod(splited[1]);
            rc_input[2] = std::stod(splited[4]);
            rc_input[3] = std::stod(splited[3]);
            last_rc_time = ros::Time::now();

            if (rc_input[0] > 0)
            {
                control_state = ControlState::Assistance;
            }
            else
            {
                control_state = ControlState::Manual;
                if (traj.getPieceNum() > 0) traj.clear();
            }
        }
        else if (splited.front() == "d")
        {
            startGoal.clear();
            traj.clear();
            ROS_WARN("Path deleted!");
            visualizer.deletePlans();
            control_state = ControlState::Hover;
        }
        else if (splited.front() == "p")
        {
            setAstarBound();
            std::cout << "test" << std::endl;
        }
    }

    void rcinCallback(const mavros_msgs::RCIn& msg)
    {
        double chan_normalized[4];
        double padding = 100;

        for (int i = 0; i < 4; i++)
        {
            chan_normalized[i] = (double)(msg.channels[i] - 1500) / (1000.0 - padding);
            chan_normalized[i] = std::min(1.0, std::max(-1.0, chan_normalized[i]));
        }

        rc_input[0] =  chan_normalized[1]; // pitch -> x
        rc_input[1] = -chan_normalized[0]; // roll -> y
        rc_input[2] =  chan_normalized[3]; // throttle -> z
        rc_input[3] = -chan_normalized[2]; // yaw -> yawrate 

        last_rc_time = ros::Time::now();

        if (rc_input[0] > 0)
        {
            control_state = ControlState::Assistance;
        }
        else
        {
            control_state = ControlState::Manual;
            if (traj.getPieceNum() > 0) traj.clear();
        }
    }

    void setAstarBound()
    {
        double hFov = 56.0 / 2.0 * 0.01745329;
        double vFov = 40.0 / 2.0 * 0.01745329;

        Eigen::Vector3d foward = Eigen::Vector3d(13.0, 0, 0);
        Eigen::Vector3d back = Eigen::Vector3d(-0.5, 0, 0);
        Eigen::Vector3d focal = Eigen::AngleAxisd(state.yaw, Eigen::Vector3d::UnitZ()) * back + state.pos;
        Eigen::Vector3d max_range_center = Eigen::AngleAxisd(state.yaw, Eigen::Vector3d::UnitZ()) * foward + focal;

        Eigen::MatrixX4d Poly(6, 4);
        Poly.leftCols<3>().row(0) <<  cos(state.yaw), sin(state.yaw), 0; // front
        Poly.leftCols<3>().row(1) << -cos(state.yaw),-sin(state.yaw), 0; // back
        Poly.leftCols<3>().row(2) << -sin(hFov + state.yaw), cos(hFov + state.yaw), 0; // left
        Poly.leftCols<3>().row(3) << -sin(hFov - state.yaw),-cos(hFov - state.yaw), 0; // right
        Poly.leftCols<3>().row(4) << -sin(vFov)*cos(state.yaw),-sin(vFov)*sin(state.yaw), cos(vFov); // up
        Poly.leftCols<3>().row(5) << -sin(vFov)*cos(state.yaw),-sin(vFov)*sin(state.yaw),-cos(vFov); // bottom

        Poly.rightCols<1>().row(0) << -Poly.leftCols<3>().row(0).dot(max_range_center);
        for (int i = 1; i < Poly.rows(); i++)
            Poly.rightCols<1>().row(i) << -Poly.leftCols<3>().row(i).dot(focal);

        a_star_->setSearchBound(Poly);

        // visualize A* search bound
        std::vector<Eigen::MatrixX4d> hPolys;
        hPolys.push_back(Poly);
        visualizer.visualizePolytope(hPolys, 1);
    }

    void calcWaypointCallback(const ros::TimerEvent & /*event*/)
    {
        if (control_state != ControlState::Assistance || (ros::Time::now() - last_rc_time).toSec() > 3.0)
        {
            return;
        }

        Eigen::Vector3d waypoint = calcWaypoint();
        if ((waypoint - state.pos).norm() > voxelMap.getScale())
        {
            if (voxelMap.query(waypoint) != 0) return;
            setAstarBound();
            if (!a_star_->checkSearchBound(waypoint)) return;

            startGoal.clear();
            startGoal.emplace_back(state.pos);
            startGoal.emplace_back(waypoint);
            commandGoal = waypoint;
            plan();
            control_state = ControlState::Assistance;

            visualizer.visualizeStartGoal(waypoint, 0.5, 1);
        }
        else if (traj.getPieceNum() > 0)
        {
            startGoal.clear();
            traj.clear();
            ROS_WARN("Path deleted!");
            visualizer.deletePlans();
        }
    }

    Eigen::Vector3d calcWaypoint()
    {
        const double control_mag_max = 10.0;
        const double control_mag_min = 3.0;

        Eigen::Vector3d control(rc_input[0], rc_input[1], 0);
        if (control.norm() < 0.1)
        {
            return state.pos;
        }
        else
        {
            cruiseVel = control.norm() * config.maxVelMag;
        }

        auto ray_cast = [&](const Eigen::Vector3d &start, const Eigen::Vector3d &end, const double dt) {
            const double eps = 1.0e-6;
            Eigen::Vector3d sub = (end - start);
            const double length = sub.norm();
            const Eigen::Vector3d dir = sub.normalized();
            Eigen::Vector3i pos_p = voxelMap.posD2I(start);

            for (double t = dt; t < length + eps; t += dt)
            {
                Eigen::Vector3i pos_n = voxelMap.posD2I(start + dir * t);
                if (pos_p != pos_n)
                {
                    if (voxelMap.query(pos_n) != 0)
                    {
                        return voxelMap.posI2D(pos_n);
                    }
                    pos_p = pos_n;
                }
            }

            return end;
        };

        Eigen::Vector3d test_control = control.normalized() * control_mag_max;
        Eigen::Vector3d test_wp = state.pos + Eigen::AngleAxisd(state.yaw, Eigen::Vector3d::UnitZ()) * test_control;
        double control_mag = (ray_cast(state.pos, test_wp, 0.05) - state.pos).norm() * 0.9;
        control_mag = control_mag < control_mag_min ? control_mag_max : control_mag;

        control = control.normalized() * control_mag;
        control.z() = control.norm() * rc_input[2] * 0.4;

        Eigen::Vector3d waypoint = state.pos + Eigen::AngleAxisd(state.yaw + rc_input[3] * 0.7, Eigen::Vector3d::UnitZ()) * control;

        return waypoint;
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
