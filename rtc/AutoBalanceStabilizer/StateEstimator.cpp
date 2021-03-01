// -*- mode: C++ -*-

/**
 * @file  StateEstimator.cpp
 * @brief
 * @date  $Date$
 */

#include <hrpModel/Sensor.h>
#include <hrpModel/Link.h>
#include "StateEstimator.h"

namespace hrp {

StateEstimator::StateEstimator(const hrp::BodyPtr& _robot, const std::string& _comp_name, const double _dt, std::mutex& _mutex, const std::vector<int>& link_indices)
    : m_robot(_robot),
      comp_name(_comp_name),
      dt(_dt),
      m_mutex(_mutex)
{
    for (const auto& id : link_indices) {
        limb_param.emplace(id, limbParam());
    }
}

void StateEstimator::calcStates(const stateInputData& input_data)
{
}

hrp::Vector3 calcActZMP(const hrp::BodyPtr& act_robot,
                        const std::vector<LinkConstraint>& constraints,
                        const double zmp_z)
{
    for (const auto& id : link_indices) {
        limb_param.emplace(id, limbParam());
    }

    cogvel_filter = std::make_unique<FirstOrderLowPassFilter<hrp::Vector3>>(4.0, dt, hrp::Vector3::Zero()); // 4.0 Hz
}

void StateEstimator::calcStates(const stateInputData& input_data)
{
    // world frame =>
    base_rpy = hrp::rpyFromRot(m_robot->rootLink()->R);
    foot_origin_coord = input_data.constraints.calcFootOriginCoord(m_robot);

    // cog
    cog = m_robot->calcCM();
    // zmp
    on_ground = calcZMP(zmp, input_data.constraints, input_data.zmp_z);
    // set actual contact states
    for (const auto& constraint : input_data.constraints.constraints) {
        const int link_id = constraint.getLinkId();
        limb_param[link_id].contact_states = isContact(link_id);
    }
    // <= world frame

    // convert absolute -> base-link relative
    base_frame_zmp = m_robot->rootLink()->R.transpose() * (zmp - m_robot->rootLink()->p);

    // foot_origin frame =>
    foot_frame_cog = foot_origin_coord.inverse() * cog;
    foot_frame_zmp = foot_origin_coord.inverse() * zmp;

    if (input_data.cur_const_idx != prev_const_idx) { // 接触状態が変わった
        foot_frame_cogvel = (foot_origin_coord.linear().transpose() * prev_foot_origin_coord.linear()) * foot_frame_cogvel;
    } else {
        if (on_ground) { // on ground
            foot_frame_cogvel = (foot_frame_cog - prev_foot_frame_cog) / dt;
        } else {
            if (prev_on_ground) { // take off
                jump_time_count = 1;
                jump_initial_velocity_z = foot_frame_cogvel(2);
            } else { // jumping
                ++jump_time_count;
            }
            foot_frame_cogvel(2) = jump_initial_velocity_z - g_acc * jump_time_count * dt;
        }
    }
    foot_frame_cogvel = cogvel_filter->passFilter(foot_frame_cogvel);
    foot_frame_cp = foot_frame_cog + foot_frame_cogvel / std::sqrt(g_acc / (foot_frame_cog - foot_frame_zmp)(2));
    base_frame_cp = m_robot->rootLink()->R.transpose() * (static_cast<hrp::Vector3>(foot_origin_coord * foot_frame_cp) - m_robot->rootLink()->p); // 以前は高さをzmp(2)にして地面上に投影していたが，3次元CPとして扱うことにした

    for (const auto& constraint : input_data.constraints.constraints) {
        const int link_id = constraint.getLinkId();
        const hrp::Link* const target = dynamic_cast<hrp::Link*>(m_robot->link(link_id));
        limb_param[link_id].foot_frame_ee_coord.translation() = foot_origin_coord.inverse() * constraint.calcActualTargetPosFromLinkState(target->p, target->R);
        limb_param[link_id].foot_frame_ee_coord.linear() = foot_origin_coord.linear().transpose() * constraint.calcActualTargetRotFromLinkState(target->R);
    }
    // <= foot_origin frame

    // set prev values
    prev_const_idx = input_data.cur_const_idx;
    prev_foot_origin_coord = foot_origin_coord;
    prev_foot_frame_cog = foot_frame_cog;
    prev_on_ground = on_ground;
}

// hrp::Vector3 StateEstimator::calcCOPFromRobotState(const hrp::BodyPtr& act_robot,
//                                    const std::vector<LinkConstraint>& constraints,
//                                    const LinkConstraint::ConstraintType type_thre)
// {
//     hrp::Vector3 cop_pos = hrp::Vector3::Zero();
//     double sum_weight = 0;

//     for (const LinkConstraint& constraint : constraints) {
//         if (constraint.getConstraintType() >= type_thre || !constraint.isZmpCalcTarget()) continue;
//         const double weight = constraint.getCOPWeight();
//         const hrp::Link* const target = act_robot->link(constraint.getLinkId());
//         cop_pos += constraint.calcActualTargetPosFromLinkState(target->p, target->R) * weight;
//         sum_weight += weight;
//     }
//     if (sum_weight > 0) cop_pos /= sum_weight;

//     return cop_pos;
// }

// hrp::Matrix33 StateEstimator::calcCOPRotationFromRobotState(const hrp::BodyPtr& act_robot,
//                                             const std::vector<LinkConstraint>& constraints,
//                                             const LinkConstraint::ConstraintType type_thre)
// {
//     Eigen::Quaternion<double> cop_quat = Eigen::Quaternion<double>::Identity();
//     double sum_weight = 0;
//     constexpr double EPS = 1e-6;

//     for (const LinkConstraint& constraint : constraints) {
//         const double weight = constraint.getCOPWeight();
//         if (constraint.getConstraintType() >= type_thre || !constraint.isZmpCalcTarget() ||
//             weight < EPS /* to avoid zero division */) continue;
//         sum_weight += weight;

//         const hrp::Link* const target = act_robot->link(constraint.getLinkId());
//         const Eigen::Quaternion<double> contact_quat(constraint.calcActualTargetRotFromLinkState(target->R));
//         cop_quat = cop_quat.slerp(weight / sum_weight, contact_quat);
//     }

//     return cop_quat.toRotationMatrix();
// }

}
