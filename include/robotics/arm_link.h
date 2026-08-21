//
// Created by guan on 2026/8/17.
//
#pragma once

#include "robotics/link.h"

namespace robotics {
    //统一使用弧度制

    struct IkResult {
        math::matrix<8, 6> raw_data{};
        math::matrix<8, 6> valid_data{};
        uint8_t valid_count = 0;
        bool reachable = true;
    };

    class Arm_link {
    public:
        Arm_link() = default;
        Arm_link(const Link links_[k_dof], const math::matrix<4, 4>& base_, const math::matrix<4, 4>& tool_)
            :Base(base_), Tool(tool_) {
            BSP_ASSERT(links != nullptr);

            for (uint8_t i = 0; i < k_dof; i++) {
                links[i] = links_[i];   // 逐个拷贝
            }
            a2 = links[2].a(); a3 = links[3].a();
            d2 = links[1].d(); d4 = links[3].d();
            // arm_theta.cur_angle = arm_theta.raw_data = math::matrix<8, 6>::zeros();
            Base_inv = internal::inv_t(Base), Tool_inv = internal::inv_t(Tool);
            forward(math::matrix<6, 1>::zeros());
            jacobi_clc();
        }

        // 正运动学解算 根据当前关节角更新末端 T。
        void forward(const math::matrix<6, 1> &q);

        // 雅可比矩阵计算 更新为末端的雅可比矩阵，加入 tool 偏移。
        void jacobi_clc();

        /**
         * Newton–Euler 逆动力学：τ = M(q)q̈ + C(q,q̇)q̇ + G(q)
         * @param q   关节位置 [rad]
         * @param qd  关节速度 [rad/s]
         * @param qdd 关节加速度 [rad/s²]
         * 逆动力学解算 可用于重力补偿关节力矩( 默认末端不携带物品，若长期携带物品请更改惯性张量 )。
         * 默认基坐标系 z 轴向上，基座静止
         */
        void rne(const math::matrix<6, 1>& q,
            const math::matrix<6, 1>& qd, const math::matrix<6, 1>& qdd);

    private:

        float a2, a3, d2, d4;
        Link links[6];
        // uint8_t validCount = 0;
        math::matrix<4, 4> Base, Tool;
        math::matrix<4, 4> Base_inv, Tool_inv;
        math::matrix<4, 4> T_joint[k_dof];
        math::matrix<4, 4> T_, T_tool;
        math::matrix<6, 6> Jacobi;
        math::matrix<6, 1> tau_t;
        math::matrix<6, 1> cur_q;
        math::matrix<3, 1> gravity_{0.0f, 0.0f, k_gravity};
        // app_Arm_data_t arm_theta;
    };
}
