/**
 * @file test_ssc_with_mpdm.cc
 * @author HKUST Aerial Robotics Group (lzhangbz@ust.hk)
 * @brief
 * @version 0.1
 * @date 2020-09-21
 *
 * @copyright Copyright (c) 2020
 *
 */
#include <ros/ros.h>
#include <stdlib.h>

#include <chrono>
#include <iostream>

#include "behavior_planner/behavior_server_ros.h"
#include "semantic_map_manager/data_renderer.h"
#include "semantic_map_manager/ros_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
#include "ssc_planner/ssc_server_ros.h"

DECLARE_BACKWARD;
double ssc_planner_work_rate = 20.0;
double bp_work_rate = 20.0;

planning::SscPlannerServer* p_ssc_server_{nullptr};
planning::BehaviorPlannerServer* p_bp_server_{nullptr};

// 使用bind是为了将BehaviorPlannerServer内部的对象smm传至类外的p_ssc_server_
int BehaviorUpdateCallback(const semantic_map_manager::SemanticMapManager& smm) {
    if (p_ssc_server_)
        p_ssc_server_->PushSemanticMap(smm); // Behavior planning后的SemanticMap交给ssc planner使用
    return 0;
}
// 使用bind是为了将RosAdapter内部的对象*p_smm_传至类外的p_bp_server_
int SemanticMapUpdateCallback(const semantic_map_manager::SemanticMapManager& smm) {
    if (p_bp_server_)
        p_bp_server_->PushSemanticMap(smm); // ros adapter的SemanticMap交给behavior planner使用
    return 0;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "~");
    ros::NodeHandle nh("~");

    int ego_id;
    if (!nh.getParam("ego_id", ego_id)) {
        ROS_ERROR("Failed to get param %d", ego_id);
        assert(false);
    }

    std::string agent_config_path;
    // 读取智能体的配置(Json格式), 在SemanticMapManager构造函数内解析
    if (!nh.getParam("agent_config_path", agent_config_path)) {
        ROS_ERROR("Failed to get param %s", agent_config_path.c_str());
        assert(false);
    }

    std::string ssc_config_path;
    // 读取规划的配置
    if (!nh.getParam("ssc_config_path", ssc_config_path)) {
        ROS_ERROR("Failed to get param ssc_config_path %s", ssc_config_path.c_str());
        assert(false);
    }

    semantic_map_manager::SemanticMapManager semantic_map_manager(ego_id, agent_config_path);
    semantic_map_manager::RosAdapter smm_ros_adapter(nh, &semantic_map_manager);
    smm_ros_adapter.BindMapUpdateCallback(SemanticMapUpdateCallback);

    double desired_vel;
    nh.param("desired_vel", desired_vel, 6.0); // 20.0
    // Declare BehaviorPlanner
    p_bp_server_ = new planning::BehaviorPlannerServer(nh, bp_work_rate, ego_id);
    p_bp_server_->set_user_desired_velocity(desired_vel);
    p_bp_server_->BindBehaviorUpdateCallback(BehaviorUpdateCallback);
    p_bp_server_->set_autonomous_level(3);
    p_bp_server_->enable_hmi_interface(); // true -> 使能手柄进行人机交互

    p_ssc_server_ = new planning::SscPlannerServer(nh, ssc_planner_work_rate, ego_id);

    p_ssc_server_->Init(ssc_config_path); // 1.初始化规划的参数  2.发布topic: 控制信号, ssc地图, 轨迹可视化
    p_bp_server_->Init(); // 1.构建RoutePlanner  2.订阅手柄数据  3.可视化
    smm_ros_adapter.Init(); // 1.订阅静态环境信息  2.订阅动态环境信息

    p_bp_server_->Start(); // 1.设定地图接口  2.开启behavior planner线程
    p_ssc_server_->Start(); // 1.设定地图接口  2.开启ssc planner线程

    // TicToc timer;
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
