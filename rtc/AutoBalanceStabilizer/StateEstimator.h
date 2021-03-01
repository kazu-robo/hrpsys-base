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

namespace hrp {

struct stateInputData
{
    hrp::dvector q_current;
    hrp::Vector3 rpy;
    std::vector<hrp::dvector6> wrenches;
    hrp::ConstraintsWithCount constraints;
    double zmp_z; // 常にreferenceの値を用いる
};

struct limbParam
{
    hrp::Vector3 contact_cop_info = hrp::Vector3::Zero();
    double prev_act_force_z = 0;
};

class StateEstimator
{
  private:
    hrp::BodyPtr m_robot;
    std::mutex& m_mutex; // This is the reference to the mutex of AutoBalanceStabilizer class
    const std::string comp_name;
    const double dt;

    hrp::Vector3 cog = hrp::Vector3::Zero();;
    hrp::Vector3 zmp = hrp::Vector3::Zero();;
    hrp::Vector3 base_rpy = hrp::Vector3::Zero();;

    Eigen::Isometry3d foot_origin_coord;

    std::map<int, limbParam> limb_param;
    bool on_ground = false;
    double contact_decision_threshold = 50; // [N]

  public:
    StateEstimator(const hrp::BodyPtr& _robot, const std::string& _comp_name, const double _dt, std::mutex& _mutex, const std::vector<int>& link_indices);

    void calcStates(const stateInputData& input_data);
    bool calcZMP(hrp::Vector3& ret_zmp, const hrp::ConstraintsWithCount& constraints, const double zmp_z);

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
