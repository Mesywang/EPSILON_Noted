#include "common/lane/lane_generator.h"

#include "common/basics/config.h"
#include "common/spline/spline_generator.h"

namespace common {

ErrorType LaneGenerator::GetLaneBySampleInterpolation(const vec_Vecf<LaneDim>& samples,
                                                      const std::vector<decimal_t>& para, Lane* lane) {
    Spline<LaneDegree, LaneDim> spline; // 定义的五次样条, 实际求解出来的是三次样条, 后两项系数为0
    SplineGenerator<LaneDegree, LaneDim> spline_generator;

    // 三次样条插值
    if (spline_generator.GetCubicSplineBySampleInterpolation(samples, para, &spline) != kSuccess) {
        // printf("[LaneBySampleInterpolation]Cannot get lane by interpolation.\n");
        return kWrongStatus;
    }
    lane->set_position_spline(spline);
    if (!lane->IsValid())
        return kWrongStatus;
    return kSuccess;
}

ErrorType LaneGenerator::GetLaneBySamplePoints(const vec_Vecf<LaneDim>& samples, Lane* lane) {
    std::vector<decimal_t> para;
    double d = 0;
    para.push_back(d);
    for (int i = 1; i < (int)samples.size(); ++i) {
        double dx = samples[i](0) - samples[i - 1](0);
        double dy = samples[i](1) - samples[i - 1](1);
        d += std::hypot(dx, dy);
        para.push_back(d); // 从起点沿着轨迹到每个采样点的距离(近似表示弧长)
    }
    if (common::LaneGenerator::GetLaneBySampleInterpolation(samples, para, lane) != kSuccess) {
        // printf("Cannot get lane.\n");
        return kWrongStatus;
    }
    return kSuccess;
}

ErrorType LaneGenerator::GetLaneBySampleFitting(const vec_Vecf<LaneDim>& samples, const std::vector<decimal_t>& para,
                                                const Eigen::ArrayXf& breaks, const decimal_t regulator, Lane* lane) {
    Spline<LaneDegree, LaneDim> spline;
    SplineGenerator<LaneDegree, LaneDim> spline_generator;

    // TODO: to be read
    if (spline_generator.GetQuinticSplineBySampleFitting(samples, para, breaks, regulator, &spline) != kSuccess) {
        // printf("[LaneBySampleInterpolation]Cannot get lane by fitting.\n");
        return kWrongStatus;
    }
    lane->set_position_spline(spline);
    if (!lane->IsValid())
        return kWrongStatus;
    return kSuccess;
}

}  // namespace common
