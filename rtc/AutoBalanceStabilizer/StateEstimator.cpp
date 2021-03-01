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
    // TODO: rename
    double tmpzmpx = 0;
    double tmpzmpy = 0;
    double tmpfz = 0;
    // double tmpfz2 = 0.0;

    for (const LinkConstraint& constraint : constraints) {
        if (!constraint.isZmpCalcTarget()) continue;
        if (act_robot->link(constraint.getLinkId())->sensors.size() == 0) continue; // TODO: 無い場合トルク推定?

        const hrp::ForceSensor* const sensor = dynamic_cast<hrp::ForceSensor*>(act_robot->link(constraint.getLinkId())->sensors[0]);
        if (!sensor) continue;

        const hrp::Matrix33 sensor_R = sensor->link->R * sensor->localR;
        // rats::rotm3times(sensor_R, sensor->link->R, sensor->localR);
        const hrp::Vector3 nf = sensor_R * sensor->f;
        const hrp::Vector3 nm = sensor_R * sensor->tau;
        const hrp::Vector3 sensor_p = sensor->link->p + sensor->link->R * sensor->localPos;
        tmpzmpx += nf(2) * sensor_p(0) - (sensor_p(2) - zmp_z) * nf(0) - nm(1);
        tmpzmpy += nf(2) * sensor_p(1) - (sensor_p(2) - zmp_z) * nf(1) + nm(0);
        tmpfz += nf(2);

        // calc ee-local COP
        // const hrp::Link* target = m_robot->link(stikp[i].target_name);
        // const hrp::Matrix33 eeR = target->R * stikp[i].localR;
        // const hrp::Vector3 ee_fsp = eeR.transpose() * (fsp - (target->p + target->R * stikp[i].localp)); // ee-local force sensor pos
        // nf = eeR.transpose() * nf;
        // nm = eeR.transpose() * nm;
        // // ee-local total moment and total force at ee position
        // const double tmp_cop_mx = nf(2) * ee_fsp(1) - nf(1) * ee_fsp(2) + nm(0);
        // const double tmp_cop_my = nf(2) * ee_fsp(0) - nf(0) * ee_fsp(2) - nm(1);
        // const double tmp_cop_fz = nf(2);
        // contact_cop_info[i][0] = tmp_cop_mx;
        // contact_cop_info[i][1] = tmp_cop_my;
        // contact_cop_info[i][2] = tmp_cop_fz;
        // prev_act_force_z[i] = 0.85 * prev_act_force_z[i] + 0.15 * nf(2); // filter, cut off 5[Hz]
        // tmpfz2 += prev_act_force_z[i];
    }

    return hrp::Vector3(tmpzmpx / tmpfz, tmpzmpy / tmpfz, zmp_z);

    // if (tmpfz2 < contact_decision_threshold) {
    //     ret_zmp = act_zmp;
    //     return false; // in the air
    // } else {
    //     ret_zmp = hrp::Vector3(tmpzmpx / tmpfz, tmpzmpy / tmpfz, zmp_z);
    //     return true; // on ground
    // }
}

hrp::Vector3 calcCOPFromRobotState(const hrp::BodyPtr& act_robot,
                                   const std::vector<LinkConstraint>& constraints,
                                   const LinkConstraint::ConstraintType type_thre)
{
    hrp::Vector3 cop_pos = hrp::Vector3::Zero();
    double sum_weight = 0;

    for (const LinkConstraint& constraint : constraints) {
        if (constraint.getConstraintType() >= type_thre || !constraint.isZmpCalcTarget()) continue;
        const double weight = constraint.getCOPWeight();
        const hrp::Link* const target = act_robot->link(constraint.getLinkId());
        cop_pos += constraint.calcActualTargetPosFromLinkState(target->p, target->R) * weight;
        sum_weight += weight;
    }
    if (sum_weight > 0) cop_pos /= sum_weight;

    return cop_pos;
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
