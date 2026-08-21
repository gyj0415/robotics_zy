//
// Created by guan on 2026/8/17.
//
#pragma once

#include <cmath>

#include "math/matrix.h"

namespace robotics::internal {
    //统一使用弧度制

    // 获取Position
    inline math::matrix<3, 1> pos_from_t(const math::matrix<4, 4> &T) {
        return math::matrix<3, 1>({T(0, 3), T(1, 3), T(2, 3)});
    }

    // 获取Z轴方向
    inline math::matrix<3, 1> z_from_t(const math::matrix<4, 4> &T) {
        return math::matrix<3, 1>({T(0, 2), T(1, 2), T(2, 2)});
    }

    //TODO : 未检查临时 matrix 函数的正确性

    // 三维叉乘 a × b
    inline math::matrix<3, 1> cross(const math::matrix<3, 1> &a, const math::matrix<3, 1> &b) {
        return math::matrix<3, 1>({
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        });
    }

    // 齐次变换逆 T = [R, p; 0, 1] 的逆为 T^-1 = [R^T, -R^T * p; 0, 1] 为旋转矩阵特制
    inline math::matrix<4, 4> inv_t(const math::matrix<4, 4> &T) {
        auto inv = math::matrix<4, 4>::eye();

        for (uint8_t r = 0; r < 3; ++r) {
            for (uint8_t c = 0; c < 3; ++c) {
                inv(r, c) = T(c, r);
            }
            inv(r, 3) = -(inv(r, 0) * T(0, 3) +
                            inv(r, 1) * T(1, 3) +
                            inv(r, 2) * T(2, 3));
        }
        return inv;
    }

    inline float limit(float val, const float& min, const float& max) {
        if (min > max)
            return val;
        else if (val < min)
            val = min;
        else if (val > max)
            val = max;
        return val;
    }

}
