#include "common/state/state_transformer.h"

namespace common {

ErrorType StateTransformer::GetStateFromFrenetState(const FrenetState& fs, State* s) const {
    if (!lane_.IsValid()) {
        printf("[StateFromFrenetState]Err: lane not valid.\n");
        return kIllegalInput;
    }
    if (!fs.is_ds_usable) {
        // ~ You may come from high speed traj but vs = 0.
        return kIllegalInput;
    }

    if (LaneDim != 2) {
        printf("[StateFromFrenetState]Err: cannot support non-plane now.\n");
        return kIllegalInput;
    }

    decimal_t curvature, curvature_derivative;
    if (lane_.GetCurvatureByArcLength(fs.vec_s[0], &curvature, &curvature_derivative) != kSuccess) {
        return kWrongStatus;
    }

    decimal_t one_minus_curd = 1 - curvature * fs.vec_ds[0];
    if (one_minus_curd < kEPS) {
        // ~ the violation is typically caused by lateral dependent trajectories
        // ~ with very small s (overshotting)
        return kWrongStatus;
    }

    Vecf<LaneDim> lane_pos;
    if (lane_.GetPositionByArcLength(fs.vec_s[0], &lane_pos) != kSuccess) {
        return kWrongStatus;
    }

    Vecf<LaneDim> vec_tangent;
    if (lane_.GetTangentVectorByArcLength(fs.vec_s[0], &vec_tangent) != kSuccess) {
        return kWrongStatus;
    }

    decimal_t lane_orientation = vec2d_to_angle(vec_tangent);

    Vecf<LaneDim> vec_normal(-vec_tangent[1], vec_tangent[0]);
    decimal_t tan_delta_theta = fs.vec_ds[1] / one_minus_curd;
    decimal_t delta_theta = atan2(fs.vec_ds[1], one_minus_curd);
    decimal_t cn_delta_theta = cos(delta_theta);

    s->vec_position = vec_normal * fs.vec_ds[0] + lane_pos;
    s->velocity = fs.vec_s[1] * one_minus_curd / cn_delta_theta;
    s->angle = normalize_angle(delta_theta + lane_orientation);

    decimal_t lhs = (fs.vec_ds[2] + (curvature_derivative * fs.vec_ds[0] + curvature * fs.vec_ds[1]) * tan_delta_theta)
                    * cn_delta_theta * cn_delta_theta / one_minus_curd;
    s->curvature = (lhs + curvature) * cn_delta_theta / one_minus_curd;
    decimal_t delta_theta_derivative = 1.0 / (1 + tan_delta_theta * tan_delta_theta)
                                       * (fs.vec_ds[2] * one_minus_curd + curvature * fs.vec_ds[1] * fs.vec_ds[1])
                                       / pow(one_minus_curd, 2);
    s->acceleration = fs.vec_s[2] * one_minus_curd / cn_delta_theta
                      + fs.vec_s[1] * fs.vec_s[1] / cn_delta_theta
                            * (one_minus_curd * tan_delta_theta * delta_theta_derivative
                               - (curvature_derivative * fs.vec_ds[0] + curvature * fs.vec_ds[1]));
    s->time_stamp = fs.time_stamp; // pass the time stamp
    return kSuccess;
}

ErrorType StateTransformer::GetFrenetStateVectorFromStates(const vec_E<State> state_vec,
                                                           vec_E<FrenetState>* fs_vec) const {
    fs_vec->clear();
    fs_vec->reserve(state_vec.size());
    FrenetState fs;
    for (const auto& state : state_vec) {
        if (GetFrenetStateFromState(state, &fs) == kSuccess) {
            fs_vec->push_back(fs);
        } else {
            return kWrongStatus;
        }
    }
    return kSuccess;
}

ErrorType StateTransformer::GetStateVectorFromFrenetStates(const vec_E<FrenetState>& fs_vec,
                                                           vec_E<State>* state_vec) const {
    State s;
    state_vec->clear();
    state_vec->reserve(fs_vec.size());
    for (auto& fs : fs_vec) {
        if (GetStateFromFrenetState(fs, &s) == kSuccess) {
            state_vec->push_back(s);
        } else {
            // TODO (@denny.ding): confirm this logic is correct
            return kWrongStatus;
        }
    }
    return kSuccess;
}

ErrorType StateTransformer::GetFrenetStateFromState(const State& s, FrenetState* fs) const {
    if (!lane_.IsValid()) {
        return kIllegalInput;
    }

    // 给定lane外一点, 计算该点在lane上的弧长(通过优化的方法计算在lane上到该点的最近点(近似垂直))
    decimal_t arc_length; // s
    if (lane_.GetArcLengthByVecPosition(s.vec_position, &arc_length) != kSuccess) {
        return kWrongStatus;
    }

    // 给定车道线的弧长, 计算车道线上该点的曲率和曲率变化率
    decimal_t curvature, curvature_derivative; // kr, kr'
    if (lane_.GetCurvatureByArcLength(arc_length, &curvature, &curvature_derivative) != kSuccess) {
        return kWrongStatus;
    }

    // 给定车道线的弧长, 计算车道线上该点的坐标
    Vecf<2> lane_position; // (xr, yr)
    if (lane_.GetPositionByArcLength(arc_length, &lane_position) != kSuccess) {
        return kWrongStatus;
    }

    // 给定车道线的弧长, 计算该点的切线单位向量
    Vecf<2> lane_tangent_vec; // T
    if (lane_.GetTangentVectorByArcLength(arc_length, &lane_tangent_vec) != kSuccess) {
        return kWrongStatus;
    }
    // 车道线方向
    decimal_t lane_orientation = vec2d_to_angle(lane_tangent_vec);
    // 车道线切线的法向量(lane_normal_vec * lane_tangent_vec = 0)
    Vecf<2> lane_normal_vec = Vecf<2>(-lane_tangent_vec[1], lane_tangent_vec[0]); // N

    const decimal_t step_tolerance = 0.5;
    if (fabs((s.vec_position - lane_position).dot(lane_tangent_vec)) > step_tolerance) {
        // 当前车辆位置和车道线方向差距过大或者距离太远
        // ~ projection deviates
        // ~ @(denny.ding) check whether this condition is good?
        return kWrongStatus;
    }

    // 推导步骤: https://blog.csdn.net/davidhopper/article/details/79162385
    decimal_t d = (s.vec_position - lane_position).dot(lane_normal_vec); // l
    decimal_t one_minus_curd = 1 - curvature * d; // 1 - l * kr
    if (one_minus_curd < kEPS) {
        // printf("[StateFromFrenetState]d not valid for transform.\n");
        return kWrongStatus;
    }
    decimal_t delta_theta = normalize_angle(s.angle - lane_orientation); // 车辆和车道线切线的夹角, theta_x - theta_r

    decimal_t cn_delta_theta = cos(delta_theta);
    decimal_t tan_delta_theta = tan(delta_theta);
    decimal_t ds = one_minus_curd * tan_delta_theta; // l'(dl/ds) = (1 - lk) * tan_theta
    decimal_t dss = // l''(dl'/ds)
        -(curvature_derivative * d + curvature * ds) * tan_delta_theta
        + one_minus_curd / pow(cn_delta_theta, 2) * (s.curvature * one_minus_curd / cn_delta_theta - curvature);
    decimal_t sp = s.velocity * cn_delta_theta / one_minus_curd; // s'(ds/dt) = vx * cos_theta / (1 - lk)

    decimal_t delta_theta_derivative = // delta_theta'
        1 / (1 + pow(tan_delta_theta, 2)) * (dss * one_minus_curd + curvature * pow(ds, 2)) / pow(one_minus_curd, 2);

    decimal_t spp = (s.acceleration // s''(ds'/dt)
                     - pow(sp, 2) / cn_delta_theta
                           * (one_minus_curd * tan_delta_theta * delta_theta_derivative
                              - (curvature_derivative * d + curvature * ds)))
                    * cn_delta_theta / one_minus_curd;

    fs->Load(Vecf<3>(arc_length, sp, spp), Vecf<3>(d, ds, dss), FrenetState::kInitWithDs);
    fs->time_stamp = s.time_stamp;
    return kSuccess;
}

ErrorType StateTransformer::GetFrenetPointFromPoint(const Vec2f& s, Vec2f* fs) const {
    if (!lane_.IsValid())
        return kIllegalInput;
    decimal_t arc_length;
    if (lane_.GetArcLengthByVecPosition(s, &arc_length) != kSuccess) {
        return kWrongStatus;
    }

    Vecf<2> lane_position;
    if (lane_.GetPositionByArcLength(arc_length, &lane_position) != kSuccess) {
        return kWrongStatus;
    }

    Vecf<2> lane_tangent_vec;
    if (lane_.GetTangentVectorByArcLength(arc_length, &lane_tangent_vec) != kSuccess) {
        return kWrongStatus;
    }
    Vecf<2> lane_normal_vec = Vecf<2>(-lane_tangent_vec[1], lane_tangent_vec[0]);

    decimal_t d = (s - lane_position).dot(lane_normal_vec);

    (*fs)(0) = arc_length;
    (*fs)(1) = d;

    return kSuccess;
}

ErrorType StateTransformer::GetFrenetPointVectorFromPoints(const vec_E<Vec2f>& pts, vec_E<Vec2f>* fps) const {
    fps->clear();
    fps->reserve(pts.size());
    Vec2f fp;
    for (const auto& pt : pts) {
        if (GetFrenetPointFromPoint(pt, &fp) == kSuccess) {
            fps->push_back(fp);
        } else {
            return kWrongStatus;
        }
    }
    return kSuccess;
}

} // namespace common