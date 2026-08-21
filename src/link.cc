//
// Created by guan on 2026/8/17.
//

#include "robotics/link.h"

#include "util.h"

namespace robotics {
    //统一使用弧度制

    [[nodiscard]] math::matrix<4, 4> Link::T(float q) const {
        float theta, d;

        if (type_ == Revolute) {
            theta = internal::limit(q, qmin_, qmax_);
            theta += offset_;
            d = d_;
        } else {
            theta = theta0_;
            float q_eff = q + offset_;
            d = internal::limit(q_eff, qmin_, qmax_);
        }
        return mdh_trans(a_, alpha_, d, theta);
    }

}