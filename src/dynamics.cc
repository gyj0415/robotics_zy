//
// Created by guan on 2026/8/18.
//

#include "robotics/arm_link.h"

#include "util.h"

namespace robotics {
    //统一使用弧度制

    void Arm_link::rne(const math::matrix<6, 1>& q,
        const math::matrix<6, 1>& qd, const math::matrix<6, 1>& qdd) {

        //对应基坐标系 z 轴指向正上方
        math::matrix<3, 1> z;
        z[2] = 1.0f;

        math::matrix<4, 4> T_ne[k_dof];
        math::matrix<3, 1> p_ne[k_dof];
        for (uint8_t i = 0; i < k_dof; i++) {
            T_ne[i] = links[i].T(q[i]);
            p_ne[i] = internal::pos_from_t(T_ne[i]);
        }

        math::matrix<3, 7> w = math::matrix<3, 7>::zeros(), wd = math::matrix<3, 7>::zeros(), vd = math::matrix<3, 7>::zeros();
        vd(0, 0) = gravity_[0], vd(1, 0) = gravity_[1], vd(2, 0) = gravity_[2];
        math::matrix<3, 6> F = math::matrix<3, 6>::zeros(), N = math::matrix<3, 6>::zeros();

        // 内推：连杆 1→6
        for (uint8_t i = 0; i < k_dof; i++) {  // 内推 1→6
            math::matrix<3, 3> R_i_im1;
            for (uint8_t r = 0; r < 3; r++) {
                for (uint8_t c = 0; c < 3; c++) {
                    R_i_im1(r, c) = T_ne[i](c, r);
                }
            }
            const auto w_im1 = w.col(i), wd_im1 = wd.col(i), vd_im1 = vd.col(i);

            const auto R_w_im1 = R_i_im1 * w_im1;
            const auto w_i = R_w_im1 + qd[i] * z;
            const auto wd_i = R_i_im1 * wd_im1 + internal::cross(R_w_im1, qd[i] * z) + qdd[i] * z;
            const auto vd_i = R_i_im1 * (internal::cross(wd_im1, p_ne[i]) +
                internal::cross(w_im1, internal::cross(w_im1, p_ne[i])) + vd_im1);
            const auto pc_i = links[i].rc();
            const auto vcd_i = internal::cross(wd_i, pc_i) +
                internal::cross(w_i, internal::cross(w_i, pc_i)) + vd_i;

            const auto F_i = links[i].m() * vcd_i;
            const auto N_i = links[i].I() * wd_i + internal::cross(w_i, links[i].I() * w_i);

            for (uint8_t r = 0; r < 3; r++) {
                w(r, i + 1) = w_i[r];
                wd(r, i + 1) = wd_i[r];
                vd(r, i + 1) = vd_i[r];
                F(r, i) = F_i[r];
                N(r, i) = N_i[r];
            }
        }

        // 外推：连杆 6→1
        auto f = F.col(5);
        auto n = N.col(5) + internal::cross(links[5].rc(), F.col(5));
        tau_t[5] = (n.T() * z)[0];

        for (int8_t i = 4; i >= 0; i--) {  // 外推 6→1
            const auto R_i1_i = T_ne[i + 1].block<3, 3>(0, 0);
            const auto Rf = R_i1_i * f;
            const auto f_next = Rf + F.col(i);
            n = N.col(i) + R_i1_i * n + internal::cross(links[i].rc(), F.col(i)) +
                internal::cross(p_ne[i + 1], Rf);
            tau_t[i] = (n.T() * z)[0];
            f = f_next;
        }
    }

}