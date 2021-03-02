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

bool StateEstimator::calcZMP(hrp::Vector3& ret_zmp, const hrp::ConstraintsWithCount& constraints, const double zmp_z)
{
    // TODO: rename
    double tmp_zmpx = 0;
    double tmp_zmpy = 0;
    double tmp_fz = 0;
    double tmp_filterd_fz = 0.0;

    for (const auto& constraint : constraints.constraints) {
        const int link_id = constraint.getLinkId();
        if (!constraint.isZmpCalcTarget()) continue;
        if (m_robot->link(link_id)->sensors.size() == 0) continue; // TODO: 無い場合トルク推定?
        const hrp::ForceSensor* const sensor = dynamic_cast<hrp::ForceSensor*>(m_robot->link(link_id)->sensors[0]);
        if (!sensor) continue;

        const hrp::Matrix33 sensor_R = sensor->link->R * sensor->localR;
        hrp::Vector3 nf = sensor_R * sensor->f;
        hrp::Vector3 nm = sensor_R * sensor->tau;
        const hrp::Vector3 sensor_p = sensor->link->p + sensor->link->R * sensor->localPos;
        tmp_zmpx += nf(2) * sensor_p(0) - (sensor_p(2) - zmp_z) * nf(0) - nm(1);
        tmp_zmpy += nf(2) * sensor_p(1) - (sensor_p(2) - zmp_z) * nf(1) + nm(0);
        tmp_fz += nf(2);

        // calc ee-local COP
        const hrp::Link* const target = dynamic_cast<hrp::Link*>(m_robot->link(link_id));
        const hrp::Matrix33 ee_R =  constraint.calcActualTargetRotFromLinkState(target->R);
        const hrp::Vector3 ee_frame_senspor_p = ee_R.transpose() * (sensor_p - constraint.calcActualTargetPosFromLinkState(target->p, target->R)); // ee-local force sensor pos
        nf = ee_R.transpose() * nf;
        nm = ee_R.transpose() * nm;
        // ee-local total moment and total force at ee position
        const double tmp_cop_mx = nf(2) * ee_frame_senspor_p(1) - nf(1) * ee_frame_senspor_p(2) + nm(0);
        const double tmp_cop_my = nf(2) * ee_frame_senspor_p(0) - nf(0) * ee_frame_senspor_p(2) - nm(1);
        const double tmp_cop_fz = nf(2);
        limb_param[link_id].contact_cop_info[0] = tmp_cop_mx;
        limb_param[link_id].contact_cop_info[1] = tmp_cop_my;
        limb_param[link_id].contact_cop_info[2] = tmp_cop_fz;

        limb_param[link_id].prev_act_force_z = 0.85 * limb_param[link_id].prev_act_force_z + 0.15 * nf(2); // filter, cut off 5[Hz]
        tmp_filterd_fz += limb_param[link_id].prev_act_force_z;
    }

    if (tmp_filterd_fz < contact_decision_threshold) {
        ret_zmp = cog; // 昔はact_zmpだったけど空中はcogと一致しているほうが都合がいい？
        return false; // in the air
    } else {
        ret_zmp = hrp::Vector3(tmp_zmpx / tmp_fz, tmp_zmpy / tmp_fz, zmp_z);
        return true; // on ground
    }
}

}
