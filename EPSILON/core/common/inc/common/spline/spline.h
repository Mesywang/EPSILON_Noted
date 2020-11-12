#ifndef _CORE_COMMON_INC_COMMON_SPLINE_SPLINE_H__
#define _CORE_COMMON_INC_COMMON_SPLINE_SPLINE_H__

#include <assert.h>

#include <vector>

#include "common/spline/polynomial.h"

namespace common {

template <int N_DEG, int N_DIM> class Spline {
public:
    typedef PolynomialND<N_DEG, N_DIM> PolynomialType;

    Spline() {}

    /**
     * @brief Set the vector domain
     * @param vec_domain: input vec domain
     */
    void set_vec_domain(const std::vector<decimal_t>& vec_domain) {
        assert(vec_domain.size() > 1);
        vec_domain_ = vec_domain;
        poly_.resize(vec_domain_.size() - 1);
    }

    /**
     * @brief Get the number of segments of the spline
     * @param n number of the segments
     */
    int num_segments() const {
        return static_cast<int>(poly_.size());
    }

    /**
     * @brief Get the vec domain of the spline
     * @param n number of the segments
     */
    std::vector<decimal_t> vec_domain() const {
        return vec_domain_;
    }

    /**
     * @brief Get the begin of the parameterization
     */
    decimal_t begin() const {
        if (vec_domain_.size() < 1)
            return 0.0;
        return vec_domain_.front();
    }

    /**
     * @brief Get the end of the parameterization
     */
    decimal_t end() const {
        if (vec_domain_.size() < 1)
            return 0.0;
        return vec_domain_.back();
    }

    /**
     * @brief Return a reference to a certain 1D polynomial
     * @param n index of the segment
     * @param j index of the dimension
     */
    Polynomial<N_DEG>& operator()(int n, int j) {
        assert(n < this->num_segments());
        assert(j < N_DIM);
        return poly_[n][j];
    }

    /**
     * @brief Return evaluation of spline at given parameterization
     * @param s parameterization, s should be in the vector domain
     * @param d derivate to take
     * @note  the function will automatically do extrapolation if out of domain
     */
    ErrorType evaluate(const decimal_t s, int d, Vecf<N_DIM>* ret) const {
        int num_pts = vec_domain_.size();
        if (num_pts < 1)
            return kIllegalInput;

        auto it = std::lower_bound(vec_domain_.begin(), vec_domain_.end(), s);
        int idx = std::max(static_cast<int>(it - vec_domain_.begin()) - 1, 0);
        decimal_t h = s - vec_domain_[idx];

        if (s < vec_domain_[0]) {
            poly_[0].evaluate(h, d, ret);
            // return kIllegalInput;
        } else if (s > vec_domain_[num_pts - 1]) {
            h = s - vec_domain_[idx - 1];
            poly_[num_pts - 2].evaluate(h, d, ret);
            // return kIllegalInput;
        } else {
            poly_[idx].evaluate(h, d, ret);
        }
        return kSuccess;
    }

    ErrorType evaluate(const decimal_t s, Vecf<N_DIM>* ret) const {
        int num_pts = vec_domain_.size();
        if (num_pts < 1)
            return kIllegalInput;

        // https://www.cnblogs.com/Tang-tangt/p/9291018.html
        auto it = std::lower_bound(vec_domain_.begin(), vec_domain_.end(), s); // 二分查找: lower_bound(起始地址，结束地址，要查找的值), 返回的是大于或等于val的第一个元素的位置
        int idx = std::max(static_cast<int>(it - vec_domain_.begin()) - 1, 0); // 计算当前s处对应的segment index
        decimal_t h = s - vec_domain_[idx];

        if (s < vec_domain_[0]) {
            poly_[0].evaluate(h, ret);
            // return kIllegalInput;
        } else if (s > vec_domain_[num_pts - 1]) {
            h = s - vec_domain_[idx - 1];
            poly_[num_pts - 2].evaluate(h, ret);
            // return kIllegalInput;
        } else {
            poly_[idx].evaluate(h, ret);
        }
        return kSuccess;
    }

    void print() const {
        int num_polys = static_cast<int>(poly_.size());
        for (int i = 0; i < num_polys; i++) {
            printf("vec domain (%lf, %lf).\n", vec_domain_[i], vec_domain_[i + 1]);
            poly_[i].print();
            Vecf<2> v;
            poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 0, &v);
            printf("end pos: (%lf, %lf).\n", v[0], v[1]);
            poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 1, &v);
            printf("end vel: (%lf, %lf).\n", v[0], v[1]);
            poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 2, &v);
            printf("end acc: (%lf, %lf).\n", v[0], v[1]);
            poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 3, &v);
            printf("end jerk: (%lf, %lf).\n", v[0], v[1]);
        }
    }

private:
    vec_E<PolynomialType> poly_; // 分段多项式
    std::vector<decimal_t> vec_domain_; // 每个采样点到起点的递增距离(样条的自变量, 是弧长的近似?)
};

}  // namespace common

#endif