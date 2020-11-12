/**
 * @file test_ssc_with_eudm.cc
 * @author HKUST Aerial Robotics Group
 * @brief test ssc planner with eudm behavior planner
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#include <ros/ros.h>
#include <stdlib.h>

#include <chrono>
#include <iostream>

#include "eudm_planner/eudm_server_ros.h"
#include "semantic_map_manager/data_renderer.h"
#include "semantic_map_manager/ros_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
#include "ssc_planner/ssc_server_ros.h"

DECLARE_BACKWARD;
double ssc_planner_work_rate = 20.0;
double bp_work_rate = 20.0;

planning::SscPlannerServer* p_ssc_server_{nullptr};
planning::EudmPlannerServer* p_bp_server_{nullptr};

int BehaviorUpdateCallback(const semantic_map_manager::SemanticMapManager& smm) {
    if (p_ssc_server_)
        p_ssc_server_->PushSemanticMap(smm);
    return 0;
}

int SemanticMapUpdateCallback(const semantic_map_manager::SemanticMapManager& smm) {
    if (p_bp_server_)
        p_bp_server_->PushSemanticMap(smm);
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
    if (!nh.getParam("agent_config_path", agent_config_path)) {
        ROS_ERROR("Failed to get param agent_config_path %s", agent_config_path.c_str());
        assert(false);
    }

    std::string bp_config_path;
    if (!nh.getParam("bp_config_path", bp_config_path)) {
        ROS_ERROR("Failed to get param bp_config_path %s", bp_config_path.c_str());
        assert(false);
    }

    std::string ssc_config_path;
    if (!nh.getParam("ssc_config_path", ssc_config_path)) {
        ROS_ERROR("Failed to get param ssc_config_path %s", ssc_config_path.c_str());
        assert(false);
    }

    semantic_map_manager::SemanticMapManager semantic_map_manager(ego_id, agent_config_path);
    semantic_map_manager::RosAdapter smm_ros_adapter(nh, &semantic_map_manager);
    smm_ros_adapter.BindMapUpdateCallback(SemanticMapUpdateCallback);

    double desired_vel;
    nh.param("desired_vel", desired_vel, 6.0);
    // Declare bp
    p_bp_server_ = new planning::EudmPlannerServer(nh, bp_work_rate, ego_id);
    p_bp_server_->set_user_desired_velocity(desired_vel);
    p_bp_server_->BindBehaviorUpdateCallback(BehaviorUpdateCallback);

    p_ssc_server_ = new planning::SscPlannerServer(nh, ssc_planner_work_rate, ego_id);

    p_bp_server_->Init(bp_config_path);
    p_ssc_server_->Init(ssc_config_path);
    smm_ros_adapter.Init();

    p_bp_server_->Start();
    p_ssc_server_->Start();

    // TicToc timer;
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
