#include "semantic_map_manager/data_renderer.h"

#include <algorithm>
#include <iterator>
#include <random>

#include "roguelike_ray_casting/roguelike_ray_casting.h"

namespace semantic_map_manager {

DataRenderer::DataRenderer(SemanticMapManager* smm_ptr) : p_semantic_map_manager_(smm_ptr) {
    ego_id_ = p_semantic_map_manager_->ego_id();
    obstacle_map_info_ = p_semantic_map_manager_->agent_config_info().obstacle_map_meta_info;
    surrounding_search_radius_ = p_semantic_map_manager_->agent_config_info().surrounding_search_radius;

    std::array<int, 2> map_size = {{obstacle_map_info_.height, obstacle_map_info_.width}}; // 1000, 1000
    std::array<decimal_t, 2> map_resl = {{obstacle_map_info_.resolution, obstacle_map_info_.resolution}}; // 0.2, 0.2
    std::array<std::string, 2> map_name = {{"height", "width"}};

    // 构造全局GridMap, 存储结构为std::vector<T> data_, 初始化为0
    p_obstacle_grid_ = new common::GridMapND<ObstacleMapType, 2>(map_size, map_resl, map_name);

    printf("[DataRenderer] Initialization finished\n");
}

ErrorType DataRenderer::Render(const double& time_stamp, const common::LaneNet& lane_net,
                               const common::VehicleSet& vehicle_set, const common::ObstacleSet& obstacle_set) {
    time_stamp_ = time_stamp;
    GetEgoVehicle(vehicle_set); // ~ Must update ego vehicle first
    GetObstacleMap(obstacle_set);
    GetWholeLaneNet(lane_net);
    GetSurroundingLaneNet(lane_net);
    GetSurroundingVehicles(vehicle_set);

    if (p_semantic_map_manager_->agent_config_info().enable_tracking_noise) { // default: false
        InjectObservationNoise();
        p_semantic_map_manager_->set_uncertain_vehicle_ids(uncertain_vehicle_ids_);
    }

    TicToc timer;
    FakeMapper();
    // printf("[RayCasting]Time cost: %lf ms\n", timer.toc());

    p_semantic_map_manager_->UpdateSemanticMap(time_stamp_, ego_vehicle_, whole_lane_net_, surrounding_lane_net_,
                                               *p_obstacle_grid_, obs_grids_, surrounding_vehicles_);

    return kSuccess;
}

ErrorType DataRenderer::InjectObservationNoise() {
    // * update uncertain vehicle ids if necessary
    if (ego_id_ != 0)
        return kSuccess;
    cnt_random_++;

    if (cnt_random_ == 10) { // 每十个周期施加一次高斯噪声
        uncertain_vehicle_ids_.clear();
        const decimal_t angle_noise_std = 0.22;
        std::normal_distribution<double> lat_pos_dist(0.0, 0.2); // <均值, 方差>
        std::normal_distribution<double> long_pos_dist(0.0, 0.7);
        std::normal_distribution<double> angle_dist(0.0, angle_noise_std);

        // * sample vehicle
        std::vector<int> surrounding_ids;
        for (const auto& v : surrounding_vehicles_.vehicles) {
            surrounding_ids.push_back(v.first); // push_back vehicle id
        }
        std::shuffle(surrounding_ids.begin(), surrounding_ids.end(),
                     random_engine_); // std::shuffle洗牌算法: https://www.cnblogs.com/lx-hhxxttxs/p/5908213.html

        std::vector<int> sampled_ids;
        for (int i = 0; i < 3 && i < surrounding_ids.size(); i++) { // 最多给三辆车加入噪声
            sampled_ids.push_back(surrounding_ids[i]);
        }

        // * inject noise
        for (auto& v : surrounding_vehicles_.vehicles) {
            if (std::find(sampled_ids.begin(), sampled_ids.end(), v.first) == sampled_ids.end())
                continue;

            // 按照高斯分布生成随机数
            decimal_t lateral_position_noise = lat_pos_dist(random_engine_);
            decimal_t long_position_noise = long_pos_dist(random_engine_);
            decimal_t angle_noise = angle_dist(random_engine_);

            common::State original_state = v.second.state();
            Vec2f original_position = original_state.vec_position;
            decimal_t angle = original_state.angle;
            Vec2f augmented_position =
                Vec2f(original_position.x() + lateral_position_noise * sin(angle) + long_position_noise * cos(angle),
                      original_position.y() - lateral_position_noise * cos(angle)
                          + long_position_noise * sin(angle)); // 错了吧???
            original_state.vec_position = augmented_position;
            original_state.angle = normalize_angle(original_state.angle + angle_noise);

            if (fabs(angle_noise) > 1.5 * angle_noise_std && v.second.type().compare("brokencar") != 0)
                uncertain_vehicle_ids_.push_back(v.first); // 若方差过大, 则标记为uncertain_vehicle

            v.second.set_state(original_state); // 将车辆状态修改为加入噪声后的状态
        }
        cnt_random_ = 0;
    }
    return kSuccess;
}

ErrorType DataRenderer::GetEgoVehicle(const common::VehicleSet& vehicle_set) {
    ego_vehicle_ = vehicle_set.vehicles.at(ego_id_);
    ego_param_ = ego_vehicle_.param();
    ego_state_ = ego_vehicle_.state();
    return kSuccess;
}

ErrorType DataRenderer::GetObstacleMap(const common::ObstacleSet& obstacle_set) {
    // ~ NOTICE:
    // ~ Origin of OccupancyGrid is at left-bottom corner,
    // ~ when x -> right, y -> up

    // 以当前车辆为中心, 左下角为原点构建局部地图: 200m * 200m
    decimal_t x = ego_state_.vec_position(0) - obstacle_map_info_.h_metric / 2.0;
    decimal_t y = ego_state_.vec_position(1) - obstacle_map_info_.w_metric / 2.0;
    // Aligning to global coordinate to improve consistency
    decimal_t x_r = std::round(x); // std::round四舍五入
    decimal_t y_r = std::round(y);

    // Use gridmap nd
    p_obstacle_grid_->fill_data(GridMap2D::UNKNOWN);
    std::array<decimal_t, 2> origin = {{x_r, y_r}}; // 设定局部障碍物坐标系的原点
    p_obstacle_grid_->set_origin(origin);

    cv::Mat grid_mat = cv::Mat(obstacle_map_info_.height, obstacle_map_info_.width,
                               CV_MAKETYPE(cv::DataType<ObstacleMapType>::type, 1), p_obstacle_grid_->get_data_ptr());

    // Fill circle using opencv
    for (const auto& obs : obstacle_set.obs_circle) {
        std::array<decimal_t, 2> center_w = {{obs.second.circle.center.x, obs.second.circle.center.y}};
        auto center_coord = p_obstacle_grid_->GetCoordUsingGlobalPosition(center_w); // 障碍物在局部坐标系下的坐标
        cv::Point2i center(center_coord[0], center_coord[1]);
        int radius = obs.second.circle.radius / obstacle_map_info_.resolution;
        cv::circle(grid_mat, center, radius, cv::Scalar(GridMap2D::OCCUPIED), -1);
    }
    // Fill polygon using opencv
    std::vector<std::vector<cv::Point>> polys;
    for (const auto& obs : obstacle_set.obs_polygon) {
        std::vector<cv::Point> poly;
        for (const auto& pt : obs.second.polygon.points) {
            std::array<decimal_t, 2> pt_w = {{pt.x, pt.y}};
            auto coord = p_obstacle_grid_->GetCoordUsingGlobalPosition(pt_w);
            cv::Point2i coord_cv(coord[0], coord[1]);
            poly.push_back(coord_cv);
        }
        polys.push_back(poly);
    }
    cv::fillPoly(grid_mat, polys, cv::Scalar(GridMap2D::OCCUPIED));

    return kSuccess;
}

ErrorType DataRenderer::FakeMapper() {
    decimal_t dist_thres = obstacle_map_info_.h_metric / 2.0 * 0.8; // 80.0m
    // obs_grids_.clear();
    RayCastingOnObstacleMap();

    decimal_t ego_pos_x = ego_state_.vec_position(0);
    decimal_t ego_pos_y = ego_state_.vec_position(1);

    // Fill past obstacles and remove far ones
    for (auto it = obs_grids_.begin(); it != obs_grids_.end();) {
        decimal_t dx = abs((*it)[0] - ego_pos_x);
        decimal_t dy = abs((*it)[1] - ego_pos_y);
        if (dx >= dist_thres || dy >= dist_thres) { // 滤掉距离过远的障碍物
            it = obs_grids_.erase(it);
            continue;
        } else {
            // SCANNED_OCCUPIED表示raycasting得到的障碍物
            p_obstacle_grid_->SetValueUsingGlobalPosition(*it, GridMap2D::SCANNED_OCCUPIED);
            ++it;
        }
    }
    return kSuccess;
}

ErrorType DataRenderer::RayCastingOnObstacleMap() {
    Vec3f ray_casting_origin;
    ego_vehicle_.Ret3DofStateAtGeometryCenter(&ray_casting_origin); // 计算车辆几何中心的状态

    std::array<decimal_t, 2> origin = {{ray_casting_origin(0), ray_casting_origin(1)}};
    auto coord = p_obstacle_grid_->GetCoordUsingGlobalPosition(origin); // 车辆中心在局部坐标系的索引
    int r_idx_max = p_obstacle_grid_->dims_size(0) * 3.0 / 4.0; // ray_casting的最大范围(索引)

    std::vector<uint8_t> render_mat(p_obstacle_grid_->data_size(), 0);
    for (int i = 0; i < 8; ++i) {
        std::set<std::array<int, 2>> obs_coord_set;
        roguelike_ray_casting::RasterizeFOVOctant(coord[0], coord[1], r_idx_max, p_obstacle_grid_->dims_size(0),
                                                  p_obstacle_grid_->dims_size(1), i, p_obstacle_grid_->data_ptr(),
                                                  render_mat.data(), &obs_coord_set);
        for (const auto coord : obs_coord_set) {
            std::array<decimal_t, 2> p_w;
            p_obstacle_grid_->GetGlobalPositionUsingCoordinate(coord, &p_w); // 局部坐标系的索引 -> 全局坐标系的位置
            obs_grids_.insert(p_w); // ray_casting的障碍物在全局坐标系下的位置
        }
    }
    for (int i = 0; i < p_obstacle_grid_->data_size(); ++i) {
        render_mat[i] = std::max(render_mat[i], p_obstacle_grid_->data(i)); // 将ray_casting的结果和原始障碍物地图融合
    }

    p_obstacle_grid_->set_data(render_mat);

    return kSuccess;
}

ErrorType DataRenderer::GetWholeLaneNet(const common::LaneNet& lane_net) {
    whole_lane_net_ = lane_net;
    return kSuccess;
}

ErrorType DataRenderer::GetSurroundingLaneNet(const common::LaneNet& lane_net) {
    surrounding_lane_net_.clear();
    // TODO(lu.zhang): Use on lane distance instead
    // 使用KD-Tree查找范围内的车道线
    lane_net_pts_.pts.clear(); // common::PointVecForKdTree
    for (auto iter = lane_net.lane_set.begin(); iter != lane_net.lane_set.end(); ++iter) { // loop each lane
        for (int i = 0; i < static_cast<int>(iter->second.lane_points.size()); ++i) { // loop each point
            common::PointWithValue<int> p;
            int id = iter->second.id;
            p.pt.x = iter->second.lane_points[i](0);
            p.pt.y = iter->second.lane_points[i](1);
            p.values.push_back(id); // 车道线id
            p.values.push_back(i); // 第几个点
            lane_net_pts_.pts.push_back(p);
        }
    }
    kdtree_lane_net_ =
        std::make_shared<KdTreeFor2dPointVec>(2, lane_net_pts_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    kdtree_lane_net_->buildIndex();

    const decimal_t query_pt[2] = {ego_vehicle_.state().vec_position(0), ego_vehicle_.state().vec_position(1)};
    const decimal_t search_radius =
        surrounding_search_radius_ * surrounding_search_radius_ * 4; // surrounding_search_radius_ = 150.0
    std::vector<std::pair<size_t, decimal_t>>
        ret_matches; // std::pair<IndexType, DistanceType>, IndexType为索引, DistanceType为距离
    nanoflann::SearchParams params;

    kdtree_lane_net_->radiusSearch(&query_pt[0], search_radius, ret_matches, params);

    std::set<size_t> matched_lane_id_set; // set不允许有重复元素, 刚好滤除重复的id
    for (const auto e : ret_matches) {
        matched_lane_id_set.insert(lane_net_pts_.pts[e.first].values[0]); // insert id
    }

    for (const auto id : matched_lane_id_set) {
        surrounding_lane_net_.lane_set.insert(
            std::pair<int, common::LaneRaw>(id, lane_net.lane_set.at(id))); // 将周围的车道线插入到surrounding_lane_net_
    }
    return kSuccess;
}

ErrorType DataRenderer::GetSurroundingVehicles(const common::VehicleSet& vehicle_set) {
    surrounding_vehicles_.vehicles.clear();

    for (const auto& v : vehicle_set.vehicles) {
        if (v.second.id() == ego_id_) // 跳过ego_car
            continue;
        // 计算距ego_car的距离
        double dx = v.second.state().vec_position(0) - ego_vehicle_.state().vec_position(0);
        double dy = v.second.state().vec_position(1) - ego_vehicle_.state().vec_position(1);
        double dist =
            std::hypot(dx, dy); // std::hypot计算两点距离: https://blog.csdn.net/dally2/article/details/78169355
        if (dist < surrounding_search_radius_) { // surrounding_search_radius_ = 150.0
            surrounding_vehicles_.vehicles.insert(v); // v: pair<const int, common::Vehicle>
        }
    }

    return kSuccess;
}

} // namespace semantic_map_manager
