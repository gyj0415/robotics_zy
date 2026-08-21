//
// Created by guan on 2026/8/17.
//
#pragma once

#include <cstdint>

#include "math/matrix.h"
#include "util.h"

namespace robotics {
    //统一使用弧度制

    //TODO : 这里的 k_dof 和 k_gravity 可以考虑放到 #define 里，或者放到 arm_link 里
    inline constexpr uint8_t k_dof = 6;              // 关节数，原代码写死 6 轴
    inline constexpr float k_gravity = 9.80665f;     // 原 Arm_G，重力加速度 [m/s^2]

    // 角度归一化到 [-π, π]
    inline float wrapPi(float x) {
        return atan2f(sinf(x), cosf(x));
    }

    // 改进 DH 旋转矩阵
    // 参数：a - 连杆长度, alpha - 连杆扭角, d - 连杆偏移, theta - 关节角度
    inline math::matrix<4, 4> mdh_trans(float a, float alpha, float d, float theta) {
        const float ct = cosf(theta);
        const float st = sinf(theta);
        const float ca = cosf(alpha);
        const float sa = sinf(alpha);
        return math::matrix<4, 4>{
            ct,      -st,     0.0f,  a,
            st * ca,  ct * ca, -sa,  -d * sa,
            st * sa,  ct * sa,  ca,   d * ca,
            0.0f,     0.0f,    0.0f,  1.0f,
        };
    }

    enum JointType {
        Revolute, Prismatic
    };

    // (MDH) + (m, rc：质心坐标, I：惯量张量)
    class Link {
    public:
        Link() = default;
        Link(const float a, const float alpha, const float d, const float theta0, const JointType type = Revolute,
             const float offset = 0.0f,  const float qmin = 0.0f, const float qmax = 0.0f, const float m = 1.0f,
             const math::matrix<3, 1> rc = math::matrix<3, 1>::zeros(), const math::matrix<3, 3> I = math::matrix<3, 3>::zeros())
            : a_(a), alpha_(alpha), d_(d), theta0_(theta0), type_(type), offset_(offset),
            qmin_(qmin), qmax_(qmax), m_(m), rc_(rc), I_(I) {}

        Link(const Link& link)
            : a_(link.a_), alpha_(link.alpha_), d_(link.d_), theta0_(link.theta0_),
            type_(link.type_), offset_(link.offset_), qmin_(link.qmin_), qmax_(link.qmax_),
            m_(link.m_), rc_(link.rc_), I_(link.I_) {}

        Link& operator = (Link link) {
            a_ = link.a_;
            alpha_ = link.alpha_;
            d_ = link.d_;
            theta0_ = link.theta0_;
            type_ = link.type_;
            offset_ = link.offset_;
            qmin_ = link.qmin_;
            qmax_ = link.qmax_;
            m_ = link.m_;
            rc_ = link.rc_;
            I_ = link.I_;
            return *this;
        }

        [[nodiscard]] math::matrix<4, 4> T(float q) const {
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

        float a() const { return a_; }                        // 连杆长度 [m]
        float alpha() const { return alpha_; }                // 连杆扭角 [rad]
        float d() const { return d_; }                        // 连杆偏距 [m]
        float theta0() const { return theta0_; }              // 移动副时的固定关节角
        float offset() const { return offset_; }              // DH 零位偏置，如 J2 的 π/2
        float qmin() const { return qmin_; }                  // 关节下限 [rad] 或 [m]
        float qmax() const { return qmax_; }                  // 关节上限；与 qmin 相等表示不限位
        float m() const { return m_; }                        // 连杆质量 [kg]
        const math::matrix<3, 1>& rc() const { return rc_; }  // 质心在连杆系坐标 [m]
        const math::matrix<3, 3>& I() const { return I_; }    // 惯量张量 [kg·m^2]
        JointType type() const { return type_; }              // 连杆类型

    private:
        float a_, alpha_, d_, theta0_;
        JointType type_{Revolute};
        // bool use_ang_lim_{false};
        float offset_{0.0f}, qmin_{0.0f}, qmax_{0.0f};
        float m_{1.0f};
        math::matrix<3, 1> rc_{};
        math::matrix<3, 3> I_{};
    };

}