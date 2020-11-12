#include "semantic_map_manager/config_loader.h"

namespace semantic_map_manager {

using Json = nlohmann::json;

// 读取Agent的Json格式信息
ErrorType ConfigLoader::ParseAgentConfig(AgentConfigInfo* p_agent_config) {
    printf("\n[ConfigLoader] Loading vehicle set\n");

    std::fstream fs(agent_config_path_);
    Json root;
    fs >> root;

    Json agent_config_json = root["agent_config"];
    int num = static_cast<int>(agent_config_json["info"].size());
    for (int i = 0; i < num; ++i) {
        Json agent = agent_config_json["info"][i];
        if (agent["id"].get<int>() != ego_id_) // 只读取主车的信息, ego_id_ = 0
            continue;
        // common::GridMapMetaInfo构造函数中计算了w_metric和h_metric
        p_agent_config->obstacle_map_meta_info =
            common::GridMapMetaInfo(agent["obstacle_map_meta_info"]["width"].get<double>(),           // 1000, 索引数
                                    agent["obstacle_map_meta_info"]["height"].get<double>(),          // 1000, 索引数
                                    agent["obstacle_map_meta_info"]["resolution"].get<double>());     // 0.2
        p_agent_config->surrounding_search_radius = agent["surrounding_search_radius"].get<double>(); // 150.0
        p_agent_config->enable_openloop_prediction = agent["enable_openloop_prediction"].get<bool>(); // true
        if (agent.count("enable_tracking_noise")) // false
        {
            p_agent_config->enable_tracking_noise = agent["enable_tracking_noise"].get<bool>();
        }
        if (agent.count("enable_log")) // false
        {
            p_agent_config->enable_log = agent["enable_log"].get<bool>();
            p_agent_config->log_file = agent["log_file"].get<std::string>();
        }
    }

    p_agent_config->PrintInfo();
    fs.close();
    return kSuccess;
}

} // namespace semantic_map_manager