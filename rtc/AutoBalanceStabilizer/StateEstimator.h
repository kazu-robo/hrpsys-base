// -*- mode: C++ -*-

/**
 * @file  StateEstimator.h
 * @brief
 * @date  $Date$
 */

#ifndef STATEESTIMATOR_H
#define STATEESTIMATOR_H

#include <cmath>
#include <mutex>
#include <Eigen/Geometry>
#include <hrpUtil/EigenTypes.h>
#include <hrpModel/Body.h>
#include "LinkConstraint.h"
#include "../TorqueFilter/IIRFilter.h"

namespace hrp {

struct stateInputData
{
    hrp::dvector q_current;
    hrp::Vector3 rpy;
    std::vector<hrp::dvector6> wrenches;
    hrp::ConstraintsWithCount constraints;
    double zmp_z; // 常にreferenceの値を用いる
    size_t cur_const_idx;
};

struct limbParam
{
    hrp::Vector3 contact_cop_info = hrp::Vector3::Zero();
    double prev_act_force_z = 0;
    bool contact_states = false;
    Eigen::Isometry3d foot_frame_ee_coord = Eigen::Isometry3d::Identity();
};

class StateEstimator
{
  private:
    hrp::BodyPtr m_robot;
    std::mutex& m_mutex; // This is the reference to the mutex of AutoBalanceStabilizer class
    const std::string comp_name;
    const double dt;
    static constexpr double g_acc = 9.80665; // [m/s^2]

    // world frame
    hrp::Vector3 cog = hrp::Vector3::Zero();
    hrp::Vector3 zmp = hrp::Vector3::Zero();
    hrp::Vector3 base_rpy = hrp::Vector3::Zero();

    // base-link frame
    hrp::Vector3 base_frame_zmp = hrp::Vector3::Zero();

    // foot-origion frame
    hrp::Vector3 foot_frame_cog = hrp::Vector3::Zero();
    hrp::Vector3 prev_foot_frame_cog = hrp::Vector3::Zero();
    hrp::Vector3 foot_frame_zmp = hrp::Vector3::Zero();
    hrp::Vector3 foot_frame_cogvel = hrp::Vector3::Zero();

    Eigen::Isometry3d foot_origin_coord = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d prev_foot_origin_coord = Eigen::Isometry3d::Identity();

    std::map<int, limbParam> limb_param;
    bool on_ground = false;
    bool prev_on_ground = false;
    double contact_decision_threshold = 25; // [N]

    size_t prev_const_idx = 0;
    size_t jump_time_count = 0;
    double jump_initial_velocity_z = 0;
    std::unique_ptr<FirstOrderLowPassFilter<hrp::Vector3>> cogvel_filter;

  public:
    StateEstimator(const hrp::BodyPtr& _robot, const std::string& _comp_name, const double _dt, std::mutex& _mutex, const std::vector<int>& link_indices);

    void calcStates(const stateInputData& input_data);
    bool calcZMP(hrp::Vector3& ret_zmp, const hrp::ConstraintsWithCount& constraints, const double zmp_z);
    inline bool isContact(const int idx) { return limb_param[idx].prev_act_force_z > contact_decision_threshold; };

    // hrp::Vector3 calcCOPFromRobotState(const hrp::BodyPtr& act_robot,
    //                                    const std::vector<LinkConstraint>& constraints,
    //                                    const LinkConstraint::ConstraintType type_thre = LinkConstraint::FLOAT);

    // hrp::Matrix33 calcCOPRotationFromRobotState(const hrp::BodyPtr& act_robot,
    //                                             const std::vector<LinkConstraint>& constraints,
    //                                             const LinkConstraint::ConstraintType type_thre = LinkConstraint::FLOAT);

    // inline hrp::Vector3 calcCP(const hrp::Vector3& cog, const hrp::Vector3& cog_vel, const double zmp_z,
    //                            const double g_acc = 9.80665)
    // {
    //     return cog + cog_vel / std::sqrt(g_acc / (cog[2] - zmp_z));
    // }

    // bool calcIsOnGround();

};

}

#endif // STATEESTIMATOR_H
