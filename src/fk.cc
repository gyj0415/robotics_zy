//
// Created by guan on 2026/8/17.
//

#include "robotics/arm_link.h"

#include "util.h"

namespace robotics {
    //统一使用弧度制

    static void set_col(math::matrix<6, 6>& J, uint8_t col,
                const math::matrix<3, 1>& lin, const math::matrix<3, 1>& ang) {
        J(0, col) = lin[0]; J(1, col) = lin[1];
        J(2, col) = lin[2]; J(3, col) = ang[0];
        J(4, col) = ang[1]; J(5, col) = ang[2];
    }

    void Arm_link::forward(const math::matrix<6, 1> &tem_q) {
        cur_q = tem_q;
        T_ = math::matrix<4, 4>::eye();

        for (uint8_t i = 0; i < k_dof; ++i) {
            T_joint[i] = links[i].T(cur_q[i]);
            T_ = T_ * T_joint[i];
        }

        T_tool = Base * T_ * Tool;
    }

    void Arm_link::jacobi_clc() {
        Jacobi = math::matrix<6, 6>::zeros();

        const auto p_tcp = internal::pos_from_t(T_tool);
        auto T_base = Base;
        for (uint8_t i = 0; i < k_dof; ++i) {
            T_base = T_base * T_joint[i];
            const auto zi = internal::z_from_t(T_base);
            const auto pi = internal::pos_from_t(T_base);
            set_col(Jacobi, i, internal::cross(zi, p_tcp - pi), zi);
        }
    }

}




