// -*- C++ -*-
/*!
 * @file  Stabilizer.cpp
 * @brief stabilizer filter
 * @date  $Date$
 *
 * $Id$
 */

#include <boost/make_shared.hpp>
#include <rtm/RTC.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include "EigenUtil.h"
#include "Stabilizer.h"

// Utility functions
using hrp::deg2rad;
using hrp::rad2deg;
using hrp::calcInteriorPoint;
using hrp::clamp;
using hrp::copyJointAnglesToRobotModel;
using hrp::copyJointAnglesFromRobotModel;

namespace {
constexpr unsigned int DEBUG_LEVEL = 0;
inline bool DEBUGP(unsigned int loop) { return (DEBUG_LEVEL == 1 && loop % 200 == 0) || DEBUG_LEVEL > 1; }
}

namespace hrp {

Stabilizer::Stabilizer(const hrp::BodyPtr& _robot, const std::string& _comp_name, const double _dt, std::mutex& _mutex)
    : m_robot(boost::make_shared<hrp::Body>(*_robot)),
      comp_name(_comp_name),
      dt(_dt),
      m_mutex(_mutex)
{
    limb_stretch_avoidance_vlimit[0] = -100 * 1e-3 * dt; // lower limit
    limb_stretch_avoidance_vlimit[1] = 50 * 1e-3 * dt; // upper limit
    total_mass = m_robot->totalMass();

    eefm_swing_damping_force_thre.resize(3, 300);
    eefm_swing_damping_moment_thre.resize(3, 15);
    cp_check_margin.resize(4, 30 * 1e-3); // [m]
    tilt_margin.resize(2, hrp::deg2rad(30));

    const size_t num_joints = m_robot->numJoints();
    transition_joint_q = hrp::dvector::Zero(num_joints);
    qorg               = hrp::dvector::Zero(num_joints);
    qrefv              = hrp::dvector::Zero(num_joints);

    act_cogvel_filter = std::make_unique<FirstOrderLowPassFilter<hrp::Vector3>>(4.0, dt, hrp::Vector3::Zero()); // 4.0 Hz ?

    servo_pgains.resize(num_joints, 100);
    servo_dgains.resize(num_joints, 100);
    servo_tqpgains.resize(num_joints, 0);
    // servo_tqdgains.resize(num_joints, 0);
    gains_transition_times.resize(num_joints, 2.0);
    szd = std::make_unique<SimpleZMPDistributor>(dt);
}

void Stabilizer::initStabilizer(const RTC::Properties& prop, const size_t ee_num)
{
    // Check is legged robot or not
    is_legged_robot = false;
    for (size_t i = 0; i < stikp.size(); i++) {
        if (stikp[i].ee_name.find("leg") == std::string::npos) continue;
        hrp::Sensor* sen= m_robot->sensor<hrp::ForceSensor>(stikp[i].sensor_name);
        if (sen != NULL) is_legged_robot = true;
    }

    target_ee_p               .resize(ee_num, hrp::Vector3::Zero());
    target_ee_R               .resize(ee_num, hrp::Matrix33::Identity());
    act_ee_p                  .resize(ee_num, hrp::Vector3::Zero());
    act_ee_R                  .resize(ee_num, hrp::Matrix33::Identity());
    projected_normal          .resize(ee_num, hrp::Vector3::Zero());
    act_force                 .resize(ee_num, hrp::Vector3::Zero());
    ref_force                 .resize(ee_num, hrp::Vector3::Zero());
    ref_moment                .resize(ee_num, hrp::Vector3::Zero());
    prev_act_force_z          .resize(ee_num, 0.0);
    ref_contact_states        .resize(ee_num, true);
    prev_ref_contact_states   .resize(ee_num, true);
    act_contact_states        .resize(ee_num, false);
    prev_act_contact_states   .resize(ee_num, false);
    toe_heel_ratio            .resize(ee_num, 1.0);
    contact_cop_info          .resize(ee_num, hrp::Vector3::Zero());
    wrenches                  .resize(ee_num, hrp::dvector6::Zero());
    control_swing_support_time.resize(ee_num, 1.0);

    is_ik_enable              .reserve(ee_num);
    is_feedback_control_enable.reserve(ee_num);
    is_zmp_calc_enable        .reserve(ee_num);
    jpe_v                     .reserve(ee_num);
    for (size_t i = 0; i < ee_num; i++) {
        const bool is_ee_leg = (stikp[i].ee_name.find("leg") != std::string::npos);
        // Hands ik => disabled, feet ik => enabled, by default
        is_ik_enable.push_back(is_ee_leg);
        // Hands feedback control => disabled, feet feedback control => enabled, by default
        is_feedback_control_enable.push_back(is_ee_leg);
        // To zmp calculation, hands are disabled and feet are enabled, by default
        is_zmp_calc_enable.push_back(is_ee_leg);

        const auto jpe = std::make_shared<hrp::JointPathEx>(m_robot,
                                                            m_robot->link(stikp[i].ee_base),
                                                            m_robot->link(stikp[i].target_name),
                                                            dt, false, comp_name);

        stikp[i].support_pgain = hrp::dvector::Constant(jpe->numJoints(), 100);
        stikp[i].support_dgain = hrp::dvector::Constant(jpe->numJoints(), 100);
        stikp[i].landing_pgain = hrp::dvector::Constant(jpe->numJoints(), 100);
        stikp[i].landing_dgain = hrp::dvector::Constant(jpe->numJoints(), 100);

        // Fix for toe joint
        const size_t num_joints = jpe->numJoints();
        if (is_ee_leg && num_joints == 7) { // leg and has 7dof joint (6dof leg +1dof toe)
            std::vector<double> optw;
            optw.resize(num_joints, 1.0);
            optw.back() = 0.0;
            jpe->setOptionalWeightVector(optw);
        }

        contact_states_index_map.emplace(stikp[i].ee_name, i);
        jpe_v.push_back(jpe);
    }

    {
        std::vector<std::pair<hrp::Link*, hrp::Link*>> interlocking_joints;
        readInterlockingJointsParamFromProperties(interlocking_joints, m_robot, prop["interlocking_joints"], comp_name);
        if (interlocking_joints.size() > 0) {
            for (size_t i = 0; i < jpe_v.size(); i++) {
                std::cerr << "[" << comp_name << "] Interlocking Joints for [" << stikp[i].ee_name << "]" << std::endl;
                jpe_v[i]->setInterlockingJointPairIndices(interlocking_joints, comp_name);
            }
        }
    }

    std::vector<std::vector<Eigen::Vector2d>> support_polygon_vec;
    for (size_t i = 0; i < stikp.size(); i++) {
        support_polygon_vec.push_back(std::vector<Eigen::Vector2d>(1, Eigen::Vector2d::Zero()));
    }
    szd->set_vertices(support_polygon_vec);

    // なぜreserve, push_back, clear?
    rel_ee_pos.reserve(ee_num);
    rel_ee_rot.reserve(ee_num);

    if (!m_robot->sensor<hrp::RateGyroSensor>("gyrometer")) {
        std::cerr << "[" << comp_name << "] WARNING! This robot model has no GyroSensor named 'gyrometer'! " << std::endl;
    }
}

// TODO: 右辺値？
void Stabilizer::execStabilizer(const paramsFromAutoBalancer& abc_param,
                                const paramsFromSensors& sensor_param)
{
    loop++; // TODO: remove?
    if (!is_legged_robot) return;

    if (DEBUGP(loop)) {
        std::cerr << "[" << comp_name << "] Parameters from autobalancer and sensors\n";
        std::cerr << "[" << comp_name << "]   zmp_ref:        " << abc_param.zmp_ref.transpose() << "\n";
        std::cerr << "[" << comp_name << "]   base_pos_ref:   " << abc_param.base_pos_ref.transpose() << "\n";
        std::cerr << "[" << comp_name << "]   base_rpy_ref:   " << abc_param.base_rpy_ref.transpose() << "\n";
        std::cerr << "[" << comp_name << "]   sbp_cog_offset: " << abc_param.sbp_cog_offset.transpose() << "\n";
        std::cerr << "[" << comp_name << "]   wrenches_ref:\n";
        for (const hrp::dvector6& ref_wrench : abc_param.wrenches_ref) {
            std::cerr << "[" << comp_name << "]                   " << ref_wrench.transpose() << "\n";
        }

        std::cerr << "[" << comp_name << "]   sensor_rpy:     " << sensor_param.rpy.transpose() << "\n";
        std::cerr << "[" << comp_name << "]   act wrenches:\n";
        for (const hrp::dvector6& wrench : sensor_param.wrenches) {
            std::cerr << "[" << comp_name << "]                   " << wrench.transpose() << "\n";
        }
        std::cerr << std::endl;
    }

    calcTargetParameters(abc_param);
    calcActualParameters(sensor_param);
    calcStateForEmergencySignal();

    switch (control_mode) {
      case MODE_IDLE:
          break;
      case MODE_AIR:
          if (transition_count == 0 && on_ground) syncToSt();
          break;
      case MODE_ST:
          if (st_algorithm != OpenHRP::AutoBalanceStabilizerService::TPCC) {
              calcEEForceMomentControl();
          } else {
              calcTPCC();
          }
          if (transition_count == 0 && !on_ground) {
              if (is_air_counter < detection_count_to_mode_air) ++is_air_counter;
              else control_mode = MODE_SYNC_TO_AIR;
          } else is_air_counter = 0;
          break;
      case MODE_SYNC_TO_IDLE:
          syncToIdle();
          control_mode = MODE_IDLE;
          break;
      case MODE_SYNC_TO_AIR:
          syncToIdle();
          control_mode = MODE_AIR;
          break;
    }

    storeCurrentStates();
}

void Stabilizer::calcTargetParameters(const paramsFromAutoBalancer& abc_param)
{
    // Ref: qRef, zmpRef, baseRpy, basePos, control_swing_support_time, ref_wrenches
    // m_COPInfo ?
    // Act: rpy, wrenches,

    // 1. Copy variables from AutoBalancer
    // 2. transition
    // 3.

    ref_contact_states = abc_param.ref_contact_states;
    // toe_heel_ratio = abc_param.toe_heel_ratio;
    control_swing_support_time = abc_param.control_swing_support_time;
    is_walking = abc_param.is_walking;
    sbp_cog_offset = abc_param.sbp_cog_offset;

    // Reference world frame =>
    // update internal robot model

    if (transition_count == 0) {
        transition_smooth_gain = 1.0;
        qrefv = abc_param.q_ref;
    } else {
        const double max_transition_count = calcMaxTransitionCount();
        transition_smooth_gain = 1 / (1 + exp(-9.19 * (((max_transition_count - std::fabs(transition_count)) / max_transition_count) - 0.5))); // TODO: 意味がわからない

        if (transition_count > 0) {
            qrefv = calcInteriorPoint(transition_joint_q, abc_param.q_ref, transition_smooth_gain);

            if (transition_count == 1) {
                std::cerr << "[" << comp_name << "] Move to MODE_IDLE" << std::endl;
                reset_emergency_flag = true;
            }
            --transition_count;
        } else {
            ++transition_count;
            qrefv = abc_param.q_ref;
        }
    }

    copyJointAnglesToRobotModel(m_robot, qrefv);

    target_root_p = abc_param.base_pos_ref;
    target_root_R = hrp::rotFromRpy(abc_param.base_rpy_ref);
    m_robot->rootLink()->p = target_root_p;
    m_robot->rootLink()->R = target_root_R;
    m_robot->calcForwardKinematics();

    ref_zmp = target_root_R * abc_param.zmp_ref + target_root_p; // base frame -> world frame

    hrp::Vector3 foot_origin_pos;
    hrp::Matrix33 foot_origin_rot;
    calcFootOriginCoords(foot_origin_pos, foot_origin_rot);

    // ref_cog = m_robot->calcCM(); // TODO: done: 下に移動した
    ref_total_force = hrp::Vector3::Zero();
    ref_total_moment = hrp::Vector3::Zero(); // Total moment around reference ZMP tmp
    ref_total_foot_origin_moment = hrp::Vector3::Zero();
    // std::cerr << "ref_force_z: ";
    for (size_t i = 0; i < stikp.size(); ++i) {
        // TODO: なぜかyは0
        // const hrp::Vector3 limb_cop_offset(abc_param.limb_cop_offsets[i][0], 0, abc_param.limb_cop_offsets[i][2]);
        const hrp::Vector3 limb_cop_offset = hrp::Vector3(0.05, 0, 0); // TODO: tmp
        stikp[i].localCOPPos = stikp[i].localp + stikp[i].localR * limb_cop_offset;
        const hrp::Link* target = m_robot->link(stikp[i].target_name);
        target_ee_p[i] = target->p + target->R * stikp[i].localp;
        target_ee_R[i] = target->R * stikp[i].localR;
        ref_force[i]  = abc_param.wrenches_ref[i].head<3>();
        // std::cerr << ref_force[i](2) << ", ";
        ref_moment[i] = abc_param.wrenches_ref[i].tail<3>();
        ref_total_force += ref_force[i];
        ref_total_moment += (target_ee_p[i] - ref_zmp).cross(ref_force[i]);
#ifndef FORCE_MOMENT_DIFF_CONTROL
        // Force/moment control
        ref_total_moment += ref_moment[i];
#endif

        if (is_feedback_control_enable[i]) {
            ref_total_foot_origin_moment += (target_ee_p[i] - foot_origin_pos).cross(ref_force[i]) + ref_moment[i];
        }
    }
    // std::cerr << std::endl;

    // todo: commit [Stabilizer] use reference zmp without first-order lead element by inverse system when calculating ref_total_moment
    if (st_algorithm != OpenHRP::AutoBalanceStabilizerService::TPCC) {
    // if (false) {
        // apply inverse system
        const hrp::Vector3 modified_ref_zmp = ref_zmp + eefm_zmp_delay_time_const[0] * (ref_zmp - prev_ref_zmp) / dt;
        prev_ref_zmp = ref_zmp;
        ref_zmp = modified_ref_zmp;
    }
    // <= Reference world frame

    ref_cog = m_robot->calcCM();

    // TODO: prev_ref_cog と ref_cogの座標系が異なるためref_cogvelが過大になる．現状はsmooth_gainで強引に吹っ飛ばなくしている
    // Reset prev_ref_cog for transition (MODE_IDLE=>MODE_ST) because the coordinates for ref_cog differs among st algorithms.
    if (transition_count == (-calcMaxTransitionCount() + 1)) { // max transition count. In MODE_IDLE => MODE_ST, transition_count is < 0 and upcounter. "+ 1" is upcount at the beginning of this function.
        prev_ref_cog = ref_cog;
        std::cerr << "[" << comp_name
                  << "]   Reset prev_ref_cog for transition (MODE_IDLE=>MODE_ST)."
                  << std::endl;
    }

    if (st_algorithm == OpenHRP::AutoBalanceStabilizerService::TPCC) {
        ref_cogvel = (ref_cog - prev_ref_cog) / dt;
    } else {
        // Reference foot_origin frame =>
        // initialize for new_refzmp
        new_refzmp = ref_zmp;
        rel_cog = m_robot->rootLink()->R.transpose() * (ref_cog - m_robot->rootLink()->p);
        // convert world (current-tmp) => local (foot_origin)
        zmp_origin_off = ref_zmp(2) - foot_origin_pos(2);
        ref_zmp = foot_origin_rot.transpose() * (ref_zmp - foot_origin_pos);
        ref_cog = foot_origin_rot.transpose() * (ref_cog - foot_origin_pos);
        new_refzmp = foot_origin_rot.transpose() * (new_refzmp - foot_origin_pos);

        if (ref_contact_states != prev_ref_contact_states) {
            ref_cogvel = (foot_origin_rot.transpose() * prev_ref_foot_origin_rot) * ref_cogvel;
        } else {
            ref_cogvel = (ref_cog - prev_ref_cog)/dt;
        }
        prev_ref_foot_origin_rot = ref_foot_origin_rot = foot_origin_rot;

        // std::cerr << "ref_force_z: ";
        for (size_t i = 0; i < stikp.size(); i++) {
            stikp[i].target_ee_diff_p = foot_origin_rot.transpose() * (target_ee_p[i] - foot_origin_pos);
            stikp[i].target_ee_diff_r = foot_origin_rot.transpose() * target_ee_R[i];
            ref_force[i] = foot_origin_rot.transpose() * ref_force[i];
            // std::cerr << ref_force[i](2) << ", ";
            ref_moment[i] = foot_origin_rot.transpose() * ref_moment[i];
        }
        // std::cerr << std::endl;

        ref_total_foot_origin_moment = foot_origin_rot.transpose() * ref_total_foot_origin_moment;
        ref_total_force = foot_origin_rot.transpose() * ref_total_force;
        ref_total_moment = foot_origin_rot.transpose() * ref_total_moment;
        target_foot_origin_rot = foot_origin_rot;

        // capture point
        ref_cp = ref_cog + ref_cogvel / std::sqrt(g_acc / (ref_cog(2) - ref_zmp(2)));
        rel_ref_cp = hrp::Vector3(ref_cp(0), ref_cp(1), ref_zmp(2));
        rel_ref_cp = m_robot->rootLink()->R.transpose() * ((foot_origin_pos + foot_origin_rot * rel_ref_cp) - m_robot->rootLink()->p);
        sbp_cog_offset = foot_origin_rot.transpose() * sbp_cog_offset;
        // <= Reference foot_origin frame
    }

    if (transition_count == (-calcMaxTransitionCount() + 1)) { // max transition count. In MODE_IDLE => MODE_ST, transition_count is < 0 and upcounter. "+ 1" is upcount at the beginning of this function.
        prev_ref_cogvel = ref_cogvel;
        std::cerr << "[" << comp_name
                  << "]   Reset prev_ref_cog for transition (MODE_IDLE=>MODE_ST)."
                  << std::endl;
    }

    ref_cogacc = (ref_cogvel - prev_ref_cogvel) / dt;

    prev_ref_cog = ref_cog;
    prev_ref_cogvel = ref_cogvel;

    // Calc swing support limb gain param
    calcSwingSupportLimbGain();
}

// TODO: rename
void Stabilizer::calcActualParameters(const paramsFromSensors& sensor_param)
{
    // Actual world frame =>
    hrp::Vector3 foot_origin_pos;
    hrp::Matrix33 foot_origin_rot;
    if (st_algorithm != OpenHRP::AutoBalanceStabilizerService::TPCC) {
        copyJointAnglesToRobotModel(m_robot, sensor_param.q_current);

        // tempolary
        m_robot->rootLink()->p = hrp::Vector3::Zero();
        m_robot->calcForwardKinematics(); // TODO: いらなさそう

        const hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
        const hrp::Matrix33 senR = sen->link->R * sen->localR;
        act_Rs = hrp::rotFromRpy(sensor_param.rpy);
        m_robot->rootLink()->R = act_Rs * (senR.transpose() * m_robot->rootLink()->R);
        m_robot->calcForwardKinematics();

        act_base_rpy = hrp::rpyFromRot(m_robot->rootLink()->R);
        calcFootOriginCoords(foot_origin_pos, foot_origin_rot);
        // std::cerr << "calcActualParameters foot_origin_pos: " << foot_origin_pos.transpose() << std::endl;
    } else {
        // TODO: restore関数, current_root_pという名前も微妙かも
        copyJointAnglesToRobotModel(m_robot, qorg);
        m_robot->rootLink()->p = current_root_p;
        m_robot->rootLink()->R = current_root_R;
        m_robot->calcForwardKinematics();
    }

    std::copy(sensor_param.wrenches.begin(), sensor_param.wrenches.end(), wrenches.begin());

    // cog
    act_cog = m_robot->calcCM();
    // zmp
    if (st_algorithm != OpenHRP::AutoBalanceStabilizerService::TPCC) {
        on_ground = calcZMP(act_zmp, zmp_origin_off + foot_origin_pos(2));
    } else {
        on_ground = calcZMP(act_zmp, ref_zmp(2));
    }

    // set actual contact states
    for (size_t i = 0; i < stikp.size(); i++) {
        const size_t idx = contact_states_index_map[stikp[i].ee_name];
        act_contact_states[idx] = isContact(idx);
    }
    // <= Actual world frame

    // convert absolute (in st) -> root-link relative
    rel_act_zmp = m_robot->rootLink()->R.transpose() * (act_zmp - m_robot->rootLink()->p);
    if (st_algorithm != OpenHRP::AutoBalanceStabilizerService::TPCC) {
        // Actual foot_origin frame =>
        act_zmp = foot_origin_rot.transpose() * (act_zmp - foot_origin_pos);
        act_cog = foot_origin_rot.transpose() * (act_cog - foot_origin_pos);
        foot_origin_pos_ = foot_origin_pos;
        //act_cogvel = foot_origin_rot.transpose() * act_cogvel;

        if (ref_contact_states != prev_ref_contact_states) {
            act_cogvel = (foot_origin_rot.transpose() * prev_act_foot_origin_rot) * act_cogvel;
        } else {
            if (act_contact_states[contact_states_index_map["rleg"]] || act_contact_states[contact_states_index_map["lleg"]]) { // on ground
                act_cogvel = (act_cog - prev_act_cog) / dt;
            } else if (prev_act_contact_states[contact_states_index_map["rleg"]] || prev_act_contact_states[contact_states_index_map["lleg"]]) { // take off
                jump_time_count = 1;
                jump_initial_velocity = act_cogvel(2);
                act_cogvel(2) = jump_initial_velocity - g_acc * jump_time_count * dt;
            } else { // jumping
                ++jump_time_count;
                act_cogvel(2) = jump_initial_velocity - g_acc * jump_time_count * dt;
            }
        }

        prev_act_foot_origin_rot = foot_origin_rot;
        act_cogvel = act_cogvel_filter->passFilter(act_cogvel);
        prev_act_cog = act_cog;
        //act_root_rot = m_robot->rootLink()->R;
        for (size_t i = 0; i < stikp.size(); i++) {
            const hrp::Link* target = m_robot->link(stikp[i].target_name);
            act_ee_p[i] = target->p + target->R * stikp[i].localp;
            act_ee_p[i] = foot_origin_rot.transpose() * (act_ee_p[i] - foot_origin_pos);
            act_ee_R[i] = foot_origin_rot.transpose() * (target->R * stikp[i].localR);
        }

        // capture point
        act_cp = act_cog + act_cogvel / std::sqrt(g_acc / (act_cog - act_zmp)(2));
        // Debug
        // std::cerr << "act_cog: " << act_cog.transpose() << ", act_cogvel: " << act_cogvel.transpose() << ", act_cp: " << act_cp.transpose() << ", act_zmp: " << act_zmp.transpose() << ", g_acc: " << g_acc << ", : " << g_acc / (act_cog - act_zmp)(2) << std::endl;
        rel_act_cp = hrp::Vector3(act_cp(0), act_cp(1), act_zmp(2));
        rel_act_cp = m_robot->rootLink()->R.transpose() * ((foot_origin_pos + foot_origin_rot * rel_act_cp) - m_robot->rootLink()->p);
        // <= Actual foot_origin frame

        // Actual world frame =>
        // new ZMP calculation
        // Kajita's feedback law
        //   Basically Equation (26) in the paper [1].
        const hrp::Vector3 dcog    = foot_origin_rot * (ref_cog - act_cog);
        const hrp::Vector3 dcogvel = foot_origin_rot * (ref_cogvel - act_cogvel);
        const hrp::Vector3 dzmp    = foot_origin_rot * (ref_zmp - act_zmp);
        new_refzmp = foot_origin_rot * new_refzmp + foot_origin_pos;
        for (size_t i = 0; i < 2; i++) {
            new_refzmp(i) += eefm_k1[i] * transition_smooth_gain * dcog(i) + eefm_k2[i] * transition_smooth_gain * dcogvel(i) + eefm_k3[i] * transition_smooth_gain * dzmp(i) + ref_zmp_aux(i);
        }

        // std::cerr << "act_cogvel = " << hrp::Vector3(act_cogvel*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;

        if (DEBUGP(loop)) {
            // All state variables are foot_origin coords relative
            std::cerr << "[" << comp_name << "] state values" << std::endl;
            std::cerr << "[" << comp_name << "]   "
                      << "ref_cog    = "
                      << hrp::Vector3(ref_cog*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                      << ", act_cog    = "
                      << hrp::Vector3(act_cog*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                      << "[mm]" << std::endl;
            std::cerr << "[" << comp_name << "]   "
                      << "ref_cogvel = " << hrp::Vector3(ref_cogvel*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                      << ", act_cogvel = " << hrp::Vector3(act_cogvel*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                      << "[mm/s]" << std::endl;
            std::cerr << "[" << comp_name << "]   "
                      << "ref_zmp    = " << hrp::Vector3(ref_zmp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                      << ", act_zmp    = " << hrp::Vector3(act_zmp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                      << "[mm]" << std::endl;
            const hrp::Vector3 tmpnew_refzmp = foot_origin_rot.transpose() * (new_refzmp-foot_origin_pos); // Actual world -> foot origin relative
            std::cerr << "[" << comp_name << "]   "
                      << "new_zmp    = " << hrp::Vector3(tmpnew_refzmp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                      << ", dif_zmp    = " << hrp::Vector3((tmpnew_refzmp-ref_zmp)*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                      << "[mm]" << std::endl;
        }

        if (control_mode == MODE_ST) {
            // distribute new ZMP into foot force & moment
            std::vector<std::string>   ee_name;
            std::vector<hrp::Vector3>  tmp_ref_force;
            std::vector<hrp::Vector3>  tmp_ref_moment;
            std::vector<double>        limb_gains;
            std::vector<hrp::dvector6> ee_forcemoment_distribution_weight;
            std::vector<double>        tmp_toe_heel_ratio;
            std::vector<hrp::Vector3>  ee_pos;
            std::vector<hrp::Vector3>  cop_pos;
            std::vector<hrp::Matrix33> ee_rot;
            std::vector<bool>          is_contact_list; // TODO: delete
            double                     total_ref_fz = 0;

            const size_t stikp_size = stikp.size();
            ee_name                           .reserve(stikp_size);
            tmp_ref_force                     .reserve(stikp_size);
            tmp_ref_moment                    .reserve(stikp_size);
            limb_gains                        .reserve(stikp_size);
            ee_forcemoment_distribution_weight.reserve(stikp_size);
            tmp_toe_heel_ratio                .reserve(stikp_size);
            ee_pos                            .reserve(stikp_size);
            cop_pos                           .reserve(stikp_size);
            ee_rot                            .reserve(stikp_size);
            is_contact_list                   .reserve(stikp_size);

            for (size_t i = 0; i < stikp_size; i++) {
                if (!is_feedback_control_enable[i]) continue;

                const STIKParam& ikp = stikp[i];
                const hrp::Link* const target = m_robot->link(ikp.target_name);
                ee_name                           .push_back(ikp.ee_name);
                ee_pos                            .push_back(target->p + target->R * ikp.localp);
                cop_pos                           .push_back(target->p + target->R * ikp.localCOPPos);
                ee_rot                            .push_back(target->R * ikp.localR);
                limb_gains                        .push_back(ikp.swing_support_gain);
                tmp_ref_force                     .push_back(foot_origin_rot * ref_force[i]);
                tmp_ref_moment                    .push_back(foot_origin_rot * ref_moment[i]);
                rel_ee_pos                        .push_back(foot_origin_rot.transpose() * (ee_pos.back() - foot_origin_pos));
                rel_ee_rot                        .push_back(foot_origin_rot.transpose() * ee_rot.back());
                is_contact_list                   .push_back(act_contact_states[i]);
                ee_forcemoment_distribution_weight.push_back(ikp.eefm_ee_forcemoment_distribution_weight);
                tmp_toe_heel_ratio                .push_back(toe_heel_ratio[i]);

                total_ref_fz += tmp_ref_force.back()(2);
                // TODO add acc to fz 目標加速度項
            }

            total_ref_fz += total_mass * ref_cogacc.z();

            // All state variables are foot_origin coords relative
            if (DEBUGP(loop)) {
            // if (true) {
                std::cerr << "[" << comp_name << "] ee values" << std::endl;
                hrp::Vector3 tmpp;
                for (size_t i = 0; i < ee_name.size(); i++) {
                    tmpp = foot_origin_rot.transpose()*(ee_pos[i]-foot_origin_pos);
                    std::cerr << "[" << comp_name << "]   "
                              << "ee_pos (" << ee_name[i] << ")    = "
                              << hrp::Vector3(tmpp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"));
                    tmpp = foot_origin_rot.transpose()*(cop_pos[i]-foot_origin_pos);
                    std::cerr << ", cop_pos (" << ee_name[i] << ")    = "
                              << hrp::Vector3(tmpp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                              << "[mm]" << std::endl;
                }
            }

            // truncate ZMP
            // if (use_zmp_truncation) {
            if (false) {
                Eigen::Vector2d new_refzmp_xy(new_refzmp.head<2>());
                szd->get_vertices(support_polygon_vertices);
                szd->calc_convex_hull(support_polygon_vertices, ref_contact_states, ee_pos, ee_rot);
                if (!szd->is_inside_support_polygon(new_refzmp_xy, hrp::Vector3::Zero(), true, comp_name)) {
                    new_refzmp.head<2>() = new_refzmp_xy;
                }
            }

            const auto printVec = [](const std::string& name, const hrp::Vector3& vec) {
                std::cerr << name << ": " << vec.transpose() << std::endl;
            };

            // for (size_t i = 0; i < stikp_size; ++i) {
            //     printVec("cop_pos[i]", cop_pos[i]);
            // }
            // printVec("new_refzmp", new_refzmp);
            // printVec("refzmp", hrp::Vector3(foot_origin_rot * ref_zmp + foot_origin_pos));

            // Distribute ZMP into each EE force/moment at each COP
            if (on_ground) {
                if (st_algorithm == OpenHRP::AutoBalanceStabilizerService::EEFM) {
                    // Modified version of distribution in Equation (4)-(6) and (10)-(13) in the paper [1].
                    szd->distributeZMPToForceMoments(tmp_ref_force, tmp_ref_moment,
                                                     ee_pos, cop_pos, ee_rot, ee_name, limb_gains,
                                                     new_refzmp, hrp::Vector3(foot_origin_rot * ref_zmp + foot_origin_pos),
                                                     total_ref_fz, dt,
                                                     DEBUGP(loop), comp_name);
                } else if (st_algorithm == OpenHRP::AutoBalanceStabilizerService::EEFMQP) {
                    szd->distributeZMPToForceMomentsQP(tmp_ref_force, tmp_ref_moment,
                                                       ee_pos, cop_pos, ee_rot, ee_name, limb_gains,
                                                       new_refzmp, hrp::Vector3(foot_origin_rot * ref_zmp + foot_origin_pos),
                                                       total_ref_fz, dt,
                                                       DEBUGP(loop), comp_name,
                                                       (st_algorithm == OpenHRP::AutoBalanceStabilizerService::EEFMQPCOP));
                } else if (st_algorithm == OpenHRP::AutoBalanceStabilizerService::EEFMQPCOP) {
                    szd->distributeZMPToForceMomentsPseudoInverse(tmp_ref_force, tmp_ref_moment,
                                                                  ee_pos, cop_pos, ee_rot, ee_name, limb_gains, tmp_toe_heel_ratio,
                                                                  new_refzmp, hrp::Vector3(foot_origin_rot * ref_zmp + foot_origin_pos),
                                                                  total_ref_fz, dt,
                                                                  DEBUGP(loop), comp_name,
                                                                  (st_algorithm == OpenHRP::AutoBalanceStabilizerService::EEFMQPCOP), is_contact_list);
                } else if (st_algorithm == OpenHRP::AutoBalanceStabilizerService::EEFMQPCOP2) {
                    szd->distributeZMPToForceMomentsPseudoInverse2(tmp_ref_force, tmp_ref_moment,
                                                                   ee_pos, cop_pos, ee_rot, ee_name, limb_gains,
                                                                   new_refzmp, hrp::Vector3(foot_origin_rot * ref_zmp + foot_origin_pos),
                                                                   foot_origin_rot * ref_total_force, foot_origin_rot * ref_total_moment,
                                                                   ee_forcemoment_distribution_weight,
                                                                   total_ref_fz, dt,
                                                                   DEBUGP(loop), comp_name);
                }
            }

            // for debug output
            new_refzmp = foot_origin_rot.transpose() * (new_refzmp - foot_origin_pos);

            // std::cerr << "tmp_ref_force z: ";
            // for (const auto& force : tmp_ref_force) std::cerr << force(2) << ", ";
            // std::cerr << std::endl;

            // foot modif
            hrp::Vector3 f_diff = hrp::Vector3::Zero();
            std::vector<bool> large_swing_f_diff(3, false);
            // moment control
            act_total_foot_origin_moment = hrp::Vector3::Zero();

            for (size_t i = 0; i < stikp_size; i++) {
                if (!is_feedback_control_enable[i]) continue;

                STIKParam& ikp = stikp[i];
                std::vector<bool> large_swing_m_diff(3, false);
                const hrp::Sensor* const sensor = m_robot->sensor<hrp::ForceSensor>(ikp.sensor_name);
                const hrp::Link* const target = m_robot->link(ikp.target_name);

                // Convert moment at COP => moment at ee
                const size_t idx = contact_states_index_map[ikp.ee_name];
                ikp.ref_moment = tmp_ref_moment[idx] + ((target->R * ikp.localCOPPos + target->p) - (target->R * ikp.localp + target->p)).cross(tmp_ref_force[idx]);
                ikp.ref_force = tmp_ref_force[idx];
                if (!act_contact_states[i]) { // TODO isFlightPhase gg導入？
                    ikp.ref_moment = hrp::Vector3::Zero();
                    ikp.ref_force = hrp::Vector3::Zero();
                }

                // Actual world frame =>
                hrp::Vector3 sensor_force = (sensor->link->R * sensor->localR) * wrenches[i].head<3>();
                const hrp::Vector3 sensor_moment = (sensor->link->R * sensor->localR) * wrenches[i].tail<3>();
                //hrp::Vector3 ee_moment = ((sensor->link->R * sensor->localPos + sensor->link->p) - (target->R * ikp.localCOPPos + target->p)).cross(sensor_force) + sensor_moment;
                hrp::Vector3 ee_moment = ((sensor->link->R * sensor->localPos + sensor->link->p) - (target->R * ikp.localp + target->p)).cross(sensor_force) + sensor_moment;
                // <= Actual world frame

                // Convert force & moment as foot origin coords relative
                ikp.ref_moment = foot_origin_rot.transpose() * ikp.ref_moment;
                ikp.ref_force = foot_origin_rot.transpose() * ikp.ref_force;
                sensor_force = foot_origin_rot.transpose() * sensor_force;
                ee_moment = foot_origin_rot.transpose() * ee_moment;

                // std::cerr << "ref_force: " << ikp.ref_force.transpose() << std::endl;
                // std::cerr << "act force: " << sensor_force.transpose() << std::endl;

                if (i == 0) f_diff -= sensor_force;
                else f_diff += sensor_force;
                for (size_t j = 0; j < 3; ++j) {
                    if ((!ref_contact_states[i] || !act_contact_states[i]) && fabs(ikp.ref_force(j) - sensor_force(j)) > eefm_swing_damping_force_thre[j]) large_swing_f_diff[j] = true;
                    if ((!ref_contact_states[i] || !act_contact_states[i]) && fabs(ikp.ref_moment(j) - ee_moment(j)) > eefm_swing_damping_moment_thre[j]) large_swing_m_diff[j] = true;
                }

                // Moment limitation
                const hrp::Matrix33 ee_R(target->R * ikp.localR);
                ikp.ref_moment = ee_R * clamp((ee_R.transpose() * ikp.ref_moment), ikp.eefm_ee_moment_limit);
                // calcDampingControl
                // d_foot_rpy and d_foot_pos is (actual) foot origin coords relative value because these use foot origin coords relative force & moment
                { // Rot
                    //   Basically Equation (16) and (17) in the paper [1]
                    hrp::Vector3 rot_damping_gain; // TODO: rename
                    for (size_t j = 0; j < 3; ++j) {
                        rot_damping_gain(j) = (!eefm_use_swing_damping || !large_swing_m_diff[j]) ? ikp.eefm_rot_damping_gain(j) : eefm_swing_rot_damping_gain(j);
                    }
                    rot_damping_gain = (1 - transition_smooth_gain) * rot_damping_gain * 10 + transition_smooth_gain * rot_damping_gain;

                    if (!ref_contact_states[i] && !act_contact_states[i]) {
                        ikp.d_foot_rpy = calcDampingControl(ikp.d_foot_rpy, ikp.eefm_rot_time_const);
                    } else {
                        ikp.d_foot_rpy = calcDampingControl(ikp.ref_moment, ee_moment, ikp.d_foot_rpy, rot_damping_gain, ikp.eefm_rot_time_const);
                    }

                    ikp.d_foot_rpy = clamp(ikp.d_foot_rpy, ikp.eefm_rot_compensation_limit);
                }

                if (!eefm_use_force_difference_control) { // Pos
                    const hrp::Vector3 pos_damping_gain = (1 - transition_smooth_gain) * ikp.eefm_pos_damping_gain * 10 + transition_smooth_gain * ikp.eefm_pos_damping_gain; // TODO: rename

                    if (!ref_contact_states[i] && !act_contact_states[i]) {
                        ikp.d_foot_pos = calcDampingControl(ikp.d_foot_pos, ikp.eefm_pos_time_const_support);
                    } else {
                        ikp.d_foot_pos = calcDampingControl(ikp.ref_force, sensor_force, ikp.d_foot_pos, pos_damping_gain, ikp.eefm_pos_time_const_support);
                    }

                    ikp.d_foot_pos = clamp(ikp.d_foot_pos, ikp.eefm_pos_compensation_limit);
                }
                if ( joint_control_mode == OpenHRP::AutoBalanceStabilizerService::JOINT_TORQUE ) {
                    ikp.d_foot_rpy = hrp::Vector3::Zero();
                    ikp.d_foot_pos = hrp::Vector3::Zero();
                }
                // Actual ee frame =>
                ikp.ee_d_foot_rpy = ee_R.transpose() * (foot_origin_rot * ikp.d_foot_rpy);
                // tilt Check : only flat plane is supported
                {
                    const hrp::Vector3 plane_x = target_ee_R[i].col(0);
                    const hrp::Vector3 plane_y = target_ee_R[i].col(1);
                    const hrp::Matrix33 act_ee_R_world = target->R * stikp[i].localR;
                    const hrp::Vector3 normal_vector = act_ee_R_world.col(2);
                    /* projected_normal = c1 * plane_x + c2 * plane_y : c1 = plane_x.dot(normal_vector), c2 = plane_y.dot(normal_vector) because (normal-vector - projected_normal) is orthogonal to plane */
                    projected_normal.at(i) = plane_x.dot(normal_vector) * plane_x + plane_y.dot(normal_vector) * plane_y;
                    act_force.at(i) = sensor_force;
                }
                act_total_foot_origin_moment += (target->R * ikp.localp + target->p - foot_origin_pos).cross(sensor_force) + ee_moment;
            }
            act_total_foot_origin_moment = foot_origin_rot.transpose() * act_total_foot_origin_moment;

            if (eefm_use_force_difference_control) {
                // fxyz control
                // foot force difference control version
                //   Basically Equation (18) in the paper [1]
                const hrp::Vector3 ref_f_diff = (stikp[1].ref_force - stikp[0].ref_force);
                // std::cerr << "ref force z 0: " << stikp[0].ref_force(2) << ", ref force z 1: " << stikp[1].ref_force(2) << std::endl;

                if (eefm_use_swing_damping) {
                    hrp::Vector3 damping_gain;
                    for (size_t i = 0; i < 3; ++i) {
                        if (!large_swing_f_diff[i]) damping_gain(i) = (1 - transition_smooth_gain) * stikp[0].eefm_pos_damping_gain(i) * 10 +
                                                        transition_smooth_gain * stikp[0].eefm_pos_damping_gain(i);
                        else damping_gain(i) = (1 - transition_smooth_gain) * eefm_swing_pos_damping_gain(i) * 10 +
                                 transition_smooth_gain * eefm_swing_pos_damping_gain(i);
                    }
                    pos_ctrl = calcDampingControl (ref_f_diff, f_diff, pos_ctrl,
                                                   damping_gain, stikp[0].eefm_pos_time_const_support);
                } else {
                    if ((ref_contact_states[contact_states_index_map["rleg"]] && ref_contact_states[contact_states_index_map["lleg"]]) // Reference : double support phase
                         || (act_contact_states[0] && act_contact_states[1])) { // Actual : double support phase
                        // Temporarily use first pos damping gain (stikp[0])
                        hrp::Vector3 damping_gain = (1 - transition_smooth_gain) * stikp[0].eefm_pos_damping_gain * 10 + transition_smooth_gain * stikp[0].eefm_pos_damping_gain;
                        pos_ctrl = calcDampingControl (ref_f_diff, f_diff, pos_ctrl,
                                                       damping_gain, stikp[0].eefm_pos_time_const_support);
                    } else {
                        double remain_swing_time;
                        if ( !ref_contact_states[contact_states_index_map["rleg"]] ) { // rleg swing
                            remain_swing_time = control_swing_support_time[contact_states_index_map["rleg"]];
                        } else { // lleg swing
                            remain_swing_time = control_swing_support_time[contact_states_index_map["lleg"]];
                        }
                        const double swing_ratio = hrp::clamp(1.0 - (remain_swing_time - eefm_pos_margin_time) / eefm_pos_transition_time, 0.0, 1.0); // 0=>1
                        // Temporarily use first pos damping gain (stikp[0])
                        const hrp::Vector3 damping_gain = (1 - transition_smooth_gain) * stikp[0].eefm_pos_damping_gain * 10 + transition_smooth_gain * stikp[0].eefm_pos_damping_gain;
                        const hrp::Vector3 tmp_time_const = (1 - swing_ratio) * eefm_pos_time_const_swing*hrp::Vector3::Ones() + swing_ratio*stikp[0].eefm_pos_time_const_support;
                        pos_ctrl = calcDampingControl (swing_ratio * ref_f_diff, swing_ratio * f_diff, pos_ctrl, damping_gain, tmp_time_const);
                    }
                }

                // Temporarily use first pos compensation limit (stikp[0])
                pos_ctrl = clamp(pos_ctrl, -1 * stikp[0].eefm_pos_compensation_limit * 2, stikp[0].eefm_pos_compensation_limit * 2);
                stikp[0].d_foot_pos = -0.5 * pos_ctrl; // TODO: use contact index map ?
                stikp[1].d_foot_pos = 0.5 * pos_ctrl;
            }

            if (DEBUGP(loop)) {
            // if (true) {
                std::cerr << "[" << comp_name << "] Control values" << std::endl;
                if (eefm_use_force_difference_control) {
                    std::cerr << "[" << comp_name << "]   "
                              << "pos_ctrl    = [" << pos_ctrl(0)*1e3 << " " << pos_ctrl(1)*1e3 << " "<< pos_ctrl(2)*1e3 << "] [mm]" << std::endl;
                }
                for (size_t i = 0; i < ee_name.size(); i++) {
                    std::cerr << "[" << comp_name << "]   "
                              << "d_foot_pos (" << ee_name[i] << ")  = [" << stikp[i].d_foot_pos(0)*1e3 << " " << stikp[i].d_foot_pos(1)*1e3 << " " << stikp[i].d_foot_pos(2)*1e3 << "] [mm], "
                              << "d_foot_rpy (" << ee_name[i] << ")  = [" << rad2deg(stikp[i].d_foot_rpy(0)) << " " << rad2deg(stikp[i].d_foot_rpy(1)) << " " << rad2deg(stikp[i].d_foot_rpy(2)) << "] [deg]" << std::endl;
                }
            }
        }
    } // st_algorithm != OpenHRP::AutoBalanceStabilizerService::TPCC

    if (joint_control_mode == OpenHRP::AutoBalanceStabilizerService::JOINT_TORQUE && control_mode == MODE_ST) setSwingSupportJointServoGains();
    calcExternalForce(foot_origin_rot * act_cog + foot_origin_pos, foot_origin_rot * new_refzmp + foot_origin_pos, foot_origin_rot); // foot origin relative => Actual world fraem
    calcTorque(foot_origin_rot);

    copyJointAnglesToRobotModel(m_robot, qrefv);
    m_robot->rootLink()->p = target_root_p;
    m_robot->rootLink()->R = target_root_R;
    if (!(control_mode == MODE_IDLE || control_mode == MODE_AIR)) {
        for (size_t i = 0; i < jpe_v.size(); i++) {
            if (!is_ik_enable[i]) continue;
            for ( int j = 0; j < jpe_v[i]->numJoints(); j++ ){
                int idx = jpe_v[i]->joint(j)->jointId;
                m_robot->joint(idx)->q = qorg[idx];
            }
        }
        m_robot->rootLink()->p(0) = current_root_p(0);
        m_robot->rootLink()->p(1) = current_root_p(1);
        m_robot->rootLink()->R = current_root_R;
        m_robot->calcForwardKinematics();
    }
    prev_ref_contact_states = ref_contact_states;
    prev_act_contact_states = act_contact_states;
    if (control_mode != MODE_ST) d_pos_z_root = 0.0;
}

void Stabilizer::storeCurrentStates()
{
    hrp::dvector cur_q;
    copyJointAnglesFromRobotModel(cur_q, m_robot);
    const hrp::Vector3  prev_root_p = current_root_p;
    const hrp::Matrix33 prev_root_R = current_root_R;
    const hrp::Vector3  prev_root_v = m_robot->rootLink()->v;
    const hrp::Vector3  prev_root_w = m_robot->rootLink()->w;
    current_root_p = m_robot->rootLink()->p;
    current_root_R = m_robot->rootLink()->R;

    const size_t num_joints = m_robot->numJoints();
    for (size_t i = 0; i < num_joints; ++i) {
        const double prev_dq = m_robot->joint(i)->dq;
        // m_robot->joint(i)->dq  = (cur_q[i] - qorg[i]) / dt;
        // m_robot->joint(i)->ddq = (m_robot->joint(i)->dq - prev_dq) / dt;
    }

    m_robot->rootLink()->v = (current_root_p - prev_root_p) / dt;
    const Eigen::AngleAxis<double> dR_aa(prev_root_R.transpose() * current_root_R);
    m_robot->rootLink()->w = (std::fabs(dR_aa.angle()) < 1e-6) ? hrp::Vector3::Zero() :
        hrp::Vector3(prev_root_R * ((dR_aa.angle() / dt) * dR_aa.axis()));
    m_robot->rootLink()->dv = hrp::Vector3(0, 0, g_acc) + (m_robot->rootLink()->v - prev_root_v) / dt;
    m_robot->rootLink()->dw = (m_robot->rootLink()->w - prev_root_w) / dt;

    m_robot->rootLink()->vo = m_robot->rootLink()->v - m_robot->rootLink()->w.cross(m_robot->rootLink()->p);
    m_robot->rootLink()->dvo = m_robot->rootLink()->dv - m_robot->rootLink()->dw.cross(m_robot->rootLink()->p) - m_robot->rootLink()->w.cross(m_robot->rootLink()->v);

    qorg = cur_q;
}

void Stabilizer::calcFootOriginCoords(hrp::Vector3& foot_origin_pos, hrp::Matrix33& foot_origin_rot)
{
    // member variable : ref_contact_states, m_robot, stikp
    rats::coordinates leg_c[2];
    const hrp::Vector3 ez = hrp::Vector3::UnitZ();
    const hrp::Vector3 ex = hrp::Vector3::UnitX();

    for (size_t i = 0; i < stikp.size(); i++) {
        if (stikp[i].ee_name.find("leg") == std::string::npos) continue;

        const hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(stikp[i].sensor_name)->link;
        leg_c[i].pos = target->p + target->R * foot_origin_offset[i];

        hrp::Vector3 xv1(target->R * ex);
        xv1(2)=0.0;
        xv1.normalize();
        const hrp::Vector3 yv1(ez.cross(xv1));
        // TODO: 整理
        leg_c[i].rot(0,0) = xv1(0); leg_c[i].rot(1,0) = xv1(1); leg_c[i].rot(2,0) = xv1(2);
        leg_c[i].rot(0,1) = yv1(0); leg_c[i].rot(1,1) = yv1(1); leg_c[i].rot(2,1) = yv1(2);
        leg_c[i].rot(0,2) = ez(0);  leg_c[i].rot(1,2) = ez(1);  leg_c[i].rot(2,2) = ez(2);
    }

    if (ref_contact_states[contact_states_index_map["rleg"]] &&
        ref_contact_states[contact_states_index_map["lleg"]]) {
        rats::coordinates mid_coords;
        rats::mid_coords(mid_coords, 0.5, leg_c[0], leg_c[1]);
        foot_origin_pos = mid_coords.pos;
        foot_origin_rot = mid_coords.rot;
    } else if (ref_contact_states[contact_states_index_map["rleg"]]) {
        foot_origin_pos = leg_c[contact_states_index_map["rleg"]].pos;
        foot_origin_rot = leg_c[contact_states_index_map["rleg"]].rot;
    } else {
        foot_origin_pos = leg_c[contact_states_index_map["lleg"]].pos;
        foot_origin_rot = leg_c[contact_states_index_map["lleg"]].rot;
    }
}

bool Stabilizer::calcZMP(hrp::Vector3& ret_zmp, const double zmp_z)
{
    // member variable : m_robot, stikp, is_zmp_calc_enable
    double tmpzmpx = 0;
    double tmpzmpy = 0;
    double tmpfz = 0, tmpfz2 = 0.0; // TODO: rename

    for (size_t i = 0; i < stikp.size(); i++) {
        if (!is_zmp_calc_enable[i]) continue;

        const hrp::ForceSensor* const sensor = m_robot->sensor<hrp::ForceSensor>(stikp[i].sensor_name);
        const hrp::Matrix33 sensor_R = normalizeMatProduct(sensor->link->R, sensor->localR);
        hrp::Vector3 nf = sensor_R * wrenches[i].head<3>();
        hrp::Vector3 nm = sensor_R * wrenches[i].tail<3>();
        const hrp::Vector3 fsp = sensor->link->p + sensor->link->R * sensor->localPos;
        tmpzmpx += nf(2) * fsp(0) - (fsp(2) - zmp_z) * nf(0) - nm(1);
        tmpzmpy += nf(2) * fsp(1) - (fsp(2) - zmp_z) * nf(1) + nm(0);
        tmpfz += nf(2);

        // calc ee-local COP
        const hrp::Link* const target = m_robot->link(stikp[i].target_name);
        const hrp::Matrix33 eeR = target->R * stikp[i].localR;
        const hrp::Vector3 ee_fsp = eeR.transpose() * (fsp - (target->p + target->R * stikp[i].localp)); // ee-local force sensor pos
        nf = eeR.transpose() * nf;
        nm = eeR.transpose() * nm;
        // ee-local total moment and total force at ee position
        const double tmp_cop_mx = nf(2) * ee_fsp(1) - nf(1) * ee_fsp(2) + nm(0);
        const double tmp_cop_my = nf(2) * ee_fsp(0) - nf(0) * ee_fsp(2) - nm(1);
        const double tmp_cop_fz = nf(2);
        contact_cop_info[i][0] = tmp_cop_mx;
        contact_cop_info[i][1] = tmp_cop_my;
        contact_cop_info[i][2] = tmp_cop_fz;
        prev_act_force_z[i] = 0.85 * prev_act_force_z[i] + 0.15 * nf(2); // filter, cut off 5[Hz]
        tmpfz2 += prev_act_force_z[i];
    }

    if (tmpfz2 < contact_decision_threshold) {
        ret_zmp = act_zmp;
        return false; // in the air
    } else {
        ret_zmp = hrp::Vector3(tmpzmpx / tmpfz, tmpzmpy / tmpfz, zmp_z);
        return true; // on ground
    }
}

bool Stabilizer::calcIfCOPisOutside()
{
    bool is_cop_outside = false;
    if (on_ground && transition_count == 0 && control_mode == MODE_ST) {
        if (DEBUGP(loop)) std::cerr << "[" << comp_name << "] COP check" << std::endl;

        for (size_t i = 0; i < stikp.size(); i++) {
            if (stikp[i].ee_name.find("leg") == std::string::npos) continue;
            // check COP inside
            // TODO: if のネストが多い?
            if (contact_cop_info[i][2] > 20.0) {
                const hrp::Vector3 tmp_cop(contact_cop_info[i][1] / contact_cop_info[i][2],
                                           contact_cop_info[i][0] / contact_cop_info[i][2],
                                           0);
                is_cop_outside = is_cop_outside ||
                    (!szd->is_inside_foot(tmp_cop, stikp[i].ee_name=="lleg", cop_check_margin) ||
                     szd->is_front_of_foot(tmp_cop, cop_check_margin) ||
                     szd->is_rear_of_foot(tmp_cop, cop_check_margin));

                if (DEBUGP(loop)) {
                    std::cerr << "[" << comp_name << "]   [" << stikp[i].ee_name << "] "
                              << "outside(" << !szd->is_inside_foot(tmp_cop, stikp[i].ee_name=="lleg", cop_check_margin) << ") "
                              << "front(" << szd->is_front_of_foot(tmp_cop, cop_check_margin) << ") "
                              << "rear(" << szd->is_rear_of_foot(tmp_cop, cop_check_margin) << ")" << std::endl;
                }
            } else {
                is_cop_outside = true;
            }
        }
    }

    return is_cop_outside;
}

bool Stabilizer::calcIfCPisOutside()
{
    bool is_cp_outside = false;
    if (on_ground && transition_count == 0 && control_mode == MODE_ST) {
        Eigen::Vector2d current_cp = act_cp.head<2>();
        szd->get_margined_vertices(margined_support_polygon_vertices);
        szd->calc_convex_hull(margined_support_polygon_vertices, act_contact_states, rel_ee_pos, rel_ee_rot);
        if (!is_walking || is_estop_while_walking) is_cp_outside = !szd->is_inside_support_polygon(current_cp, -sbp_cog_offset);

        if (DEBUGP(loop)) {
            std::cerr << "[" << comp_name << "] CP value " << "[" << act_cp(0) << "," << act_cp(1) << "] [m], "
                      << "sbp cog offset [" << sbp_cog_offset(0) << " " << sbp_cog_offset(1) << "], outside ? "
                      << (is_cp_outside ? "Outside" : "Inside")
                      << std::endl;
        }

        if (is_cp_outside) {
            if (initial_cp_too_large_error || loop % static_cast<int>(0.2/dt) == 0 ) { // once per 0.2[s]
                std::cerr << "[" << comp_name << "] "
                          << "CP too large error " << "[" << act_cp(0) << "," << act_cp(1) << "] [m]" << std::endl;
            }
            initial_cp_too_large_error = false;
        } else {
            initial_cp_too_large_error = true;
        }
    }

    return is_cp_outside;
}

bool Stabilizer::calcFalling() // TODO: rename
{
    // TODO: 整理
    bool is_falling = false;
    bool will_fall = false;
    hrp::Vector3 fall_direction = hrp::Vector3::Zero();
    double total_force = 0.0;

    // TODO: ifが多い
    for (size_t i = 0; i < stikp.size(); i++) {
        if (is_zmp_calc_enable[i]) {
            if (is_walking) {
                if (projected_normal.at(i).norm() > sin(tilt_margin[0])) {
                    will_fall = true;
                    if (m_will_fall_counter[i] % static_cast <int>(1.0/dt) == 0 ) { // once per 1.0[s]
                        std::cerr << "[" << comp_name << "] " << stikp[i].ee_name << " cannot support total weight, "
                                  << "swgsuptime : " << control_swing_support_time[i] << ", state : " << ref_contact_states[i]
                                  << ", otherwise robot will fall down toward " << "(" << projected_normal.at(i)(0) << "," << projected_normal.at(i)(1) << ") direction" << std::endl;
                    }
                    m_will_fall_counter[i]++;
                } else {
                    m_will_fall_counter[i] = 0;
                }
            }
            fall_direction += projected_normal.at(i) * act_force.at(i).norm();
            total_force += act_force.at(i).norm();
        }
    }

    if (on_ground && transition_count == 0 && control_mode == MODE_ST) {
        fall_direction = fall_direction / total_force;
    } else {
        fall_direction = hrp::Vector3::Zero();
    }

    if (fall_direction.norm() > sin(tilt_margin[1])) {
        is_falling = true;
        if (m_is_falling_counter % static_cast <int>(0.2/dt) == 0) { // once per 0.2[s]
            std::cerr << "[" << comp_name << "] robot is falling down toward ("
                      << fall_direction(0) << ", " << fall_direction(1) << ") direction"
                      << std::endl;
        }
        m_is_falling_counter++;
    } else {
        m_is_falling_counter = 0;
    }

    return is_falling || will_fall;
}

void Stabilizer::calcStateForEmergencySignal()
{
    if (DEBUGP(loop)) {
        std::cerr << "[" << comp_name << "] Check Emergency State (seq = " << (is_seq_interpolating ? "interpolating" : "empty") << ")" << std::endl;
    }

    // TODO: 組み合わせにする？ CP かつ TILTみたいな
    // Total check for emergency signal
    switch (emergency_check_mode) {
      case OpenHRP::AutoBalanceStabilizerService::NO_CHECK:
          is_emergency = false;
          break;
      case OpenHRP::AutoBalanceStabilizerService::COP:
          is_emergency = calcIfCOPisOutside() && is_seq_interpolating;
          break;
      case OpenHRP::AutoBalanceStabilizerService::CP:
          is_emergency = calcIfCPisOutside();
          break;
      case OpenHRP::AutoBalanceStabilizerService::TILT:
          is_emergency = calcFalling();
          break;
      default:
          break;
    }

    // TODO: TILTがない
    if (DEBUGP(loop)) {
        std::cerr << "[" << comp_name << "] EmergencyCheck ("
                  << (emergency_check_mode == OpenHRP::AutoBalanceStabilizerService::NO_CHECK ? "NO_CHECK":
                      (emergency_check_mode == OpenHRP::AutoBalanceStabilizerService::COP ? "COP" :
                       "CP"))
                  << ") " << (is_emergency ? "emergency" : "non-emergency") << std::endl;
    }

    whether_send_emergency_signal = false;
    if (reset_emergency_flag) {
        emergency_signal = 0;
        reset_emergency_flag = false;
        whether_send_emergency_signal = true;
    } else if (is_emergency) {
        emergency_signal = 1;
        whether_send_emergency_signal = true;
    }

    rel_ee_pos.clear();
    rel_ee_rot.clear();
}

void Stabilizer::moveBasePosRotForBodyRPYControl ()
{
    // member variable : transition_smooth_gain, eefm_body_attitude_control_gain, eefm_body_attitude_control_time_const, d_rpy, root_rot_compensation_limit, current_root_R, target_root_R, target_root_p, rel_cog, current_base_rpy, current_base_pos

    // Body rpy control
    //   Basically Equation (1) and (2) in the paper [1]
    const hrp::Vector3 ref_root_rpy = hrp::rpyFromRot(target_root_R);
    bool is_root_rot_limit = false;

    for (size_t i = 0; i < 2; i++) {
        d_rpy[i] = transition_smooth_gain * (eefm_body_attitude_control_gain[i] * (ref_root_rpy(i) - act_base_rpy(i)) - 1 / eefm_body_attitude_control_time_const[i] * d_rpy[i]) * dt + d_rpy[i];
        d_rpy[i] = clamp(d_rpy[i], -root_rot_compensation_limit[i], root_rot_compensation_limit[i]);
        is_root_rot_limit = is_root_rot_limit || (std::fabs(std::fabs(d_rpy[i]) - root_rot_compensation_limit[i]) < 1e-5); // near the limit
    }

    current_root_R = normalizeMatProduct(target_root_R, hrp::rotFromRpy(d_rpy[0], d_rpy[1], 0));
    m_robot->rootLink()->R = current_root_R;
    m_robot->rootLink()->p = target_root_p + target_root_R * rel_cog - current_root_R * rel_cog;
    m_robot->calcForwardKinematics();

    if (DEBUGP(loop) || (is_root_rot_limit && loop % 200 == 0)) {
        std::cerr << "[" << comp_name << "] Root rot control" << std::endl;
        if (is_root_rot_limit) std::cerr << "[" << comp_name << "]   Root rot limit reached!!" << std::endl;
        const hrp::Vector3 current_base_rpy = hrp::rpyFromRot(m_robot->rootLink()->R);

        std::cerr << "[" << comp_name << "]   ref = [" << rad2deg(ref_root_rpy(0)) << " " << rad2deg(ref_root_rpy(1)) << "], "
                  << "act = [" << rad2deg(act_base_rpy(0)) << " " << rad2deg(act_base_rpy(1)) << "], "
                  << "cur = [" << rad2deg(current_base_rpy(0)) << " " << rad2deg(current_base_rpy(1)) << "], "
                  << "limit = [" << rad2deg(root_rot_compensation_limit[0]) << " " << rad2deg(root_rot_compensation_limit[1])
                  << "][deg]" << std::endl;
    }
}

void Stabilizer::calcSwingSupportLimbGain ()
{
    for (size_t i = 0; i < stikp.size(); i++) {
        STIKParam& ikp = stikp[i];
        if (ref_contact_states[i]) { // Support
            // Limit too large support time increment. Max time is 3600.0[s] = 1[h], this assumes that robot's one step time is smaller than 1[h].
            ikp.support_time = std::min(3600.0, ikp.support_time + dt);
            // In some PC, does not work because the first line is optimized out.
            // ikp.support_time += dt;
            // ikp.support_time = std::min(3600.0, ikp.support_time);
            if (ikp.support_time > eefm_pos_transition_time) {
                ikp.swing_support_gain = (control_swing_support_time[i] / eefm_pos_transition_time);
            } else {
                ikp.swing_support_gain = (ikp.support_time / eefm_pos_transition_time);
            }
            ikp.swing_support_gain = clamp(ikp.swing_support_gain, 0.0, 1.0);
        } else { // Swing
            ikp.swing_support_gain = 0.0;
            ikp.support_time = 0.0;
        }
    }

    if (DEBUGP(loop)) {
        const size_t stikp_size = stikp.size();

        std::cerr << "[" << comp_name << "] SwingSupportLimbGain = [";
        for (size_t i = 0; i < stikp_size; i++) std::cerr << stikp[i].swing_support_gain << " ";

        std::cerr << "], ref_contact_states = [";
        for (size_t i = 0; i < stikp_size; i++) std::cerr << ref_contact_states[i] << " ";

        std::cerr << "], sstime = [";
        for (size_t i = 0; i < stikp_size; i++) std::cerr << control_swing_support_time[i] << " ";

        std::cerr << "], toe_heel_ratio = [";
        for (size_t i = 0; i < stikp_size; i++) std::cerr << toe_heel_ratio[i] << " ";

        std::cerr << "], support_time = [";
        for (size_t i = 0; i < stikp_size; i++) std::cerr << stikp[i].support_time << " ";

        std::cerr << "]" << std::endl;
    }
}

void Stabilizer::calcTPCC()
{
    // stabilizer loop
    // Choi's feedback law
    const hrp::Vector3 cog = m_robot->calcCM();
    const hrp::Vector3 dcog(ref_cog - act_cog);
    const hrp::Vector3 dzmp(ref_zmp - act_zmp);
    hrp::Vector3 newcog = hrp::Vector3::Zero();

    for (size_t i = 0; i < 2; i++) {
        double uu = ref_cogvel(i)
            - k_tpcc_p[i] * transition_smooth_gain * dzmp(i)
            + k_tpcc_x[i] * transition_smooth_gain * dcog(i);
        newcog(i) = uu * dt + cog(i);
    }

    moveBasePosRotForBodyRPYControl();

    // target at ee => target at link-origin
    hrp::Vector3 target_link_p[stikp.size()];
    hrp::Matrix33 target_link_R[stikp.size()];
    for (size_t i = 0; i < stikp.size(); i++) {
        target_link_R[i] = normalizeMatProduct(target_ee_R[i], stikp[i].localR.transpose());
        target_link_p[i] = target_ee_p[i] - target_ee_R[i] * stikp[i].localCOPPos;
    }

    // TODO: IKはここじゃない気がする
    // solveIK
    //   IK target is link origin pos and rot, not ee pos and rot.
    size_t max_ik_loop_count = 0;
    for (size_t i = 0; i < stikp.size(); i++) {
        max_ik_loop_count = std::max(max_ik_loop_count, stikp[i].ik_loop_count);
    }

    constexpr double MOVE_COG_RATIO = 0.9;
    for (size_t _ = 0; _ < max_ik_loop_count; ++_) {
        m_robot->rootLink()->p += MOVE_COG_RATIO * (newcog - m_robot->calcCM());
        m_robot->calcForwardKinematics();

        for (size_t i = 0; i < stikp.size(); i++) {
            if (!is_ik_enable[i]) continue;
            jpe_v[i]->calcInverseKinematics2Loop(target_link_p[i], target_link_R[i], 1.0, stikp[i].avoid_gain, stikp[i].reference_gain, &qrefv, transition_smooth_gain);
        }
    }
}

void Stabilizer::calcEEForceMomentControl()
{
    // TODO: この復元ここ？
    // stabilizer loop
    // return to reference
    copyJointAnglesToRobotModel(m_robot, qrefv);

    for (size_t i = 0; i < jpe_v.size(); i++) {
        if (!is_ik_enable[i]) continue;
        for (size_t j = 0; j < jpe_v[i]->numJoints(); j++){
            const int idx = jpe_v[i]->joint(j)->jointId;
            m_robot->joint(idx)->q = qorg[idx];
        }

        // Fix for toe joint
        if (jpe_v[i]->numJoints() == 7) {
            const int toe_idx = jpe_v[i]->joint(jpe_v[i]->numJoints() - 1)->jointId;
            m_robot->joint(toe_idx)->q = qrefv[toe_idx];
        }
    }

    const size_t stikp_size = stikp.size();

    // State calculation for swing ee compensation
    //   joint angle : current control output
    //   root pos : target root p
    //   root rot : actual root rot
    {
        // Calc status
        m_robot->rootLink()->R = target_root_R;
        m_robot->rootLink()->p = target_root_p;
        m_robot->calcForwardKinematics(); // TODO: これいる？
        const hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
        const hrp::Matrix33 senR = sen->link->R * sen->localR;
        m_robot->rootLink()->R = act_Rs * (senR.transpose() * m_robot->rootLink()->R);
        m_robot->calcForwardKinematics();

        // Calculate foot_origin_coords-relative ee pos and rot
        hrp::Vector3 foot_origin_pos;
        hrp::Matrix33 foot_origin_rot;
        calcFootOriginCoords(foot_origin_pos, foot_origin_rot);

        // Subtract them from target_ee_diff_xx
        for (size_t i = 0; i < stikp_size; i++) {
            const hrp::Link* target = m_robot->link(stikp[i].target_name);
            stikp[i].target_ee_diff_p -= foot_origin_rot.transpose() * (target->p + target->R * stikp[i].localp - foot_origin_pos);
            stikp[i].target_ee_diff_r  = (foot_origin_rot.transpose() * target->R * stikp[i].localR).transpose() * stikp[i].target_ee_diff_r;
        }
    }

    // State calculation for control : calculate "current" state
    //   joint angle : current control output
    //   root pos : target + keep COG against rpy control
    //   root rot : target + rpy control
    moveBasePosRotForBodyRPYControl();

    // Convert d_foot_pos in foot origin frame => "current" world frame
    hrp::Vector3 foot_origin_pos;
    hrp::Matrix33 foot_origin_rot;
    calcFootOriginCoords(foot_origin_pos, foot_origin_rot);
    // std::cerr << "calcEE foot_origin_pos: " << foot_origin_pos.transpose() << std::endl;
    std::vector<hrp::Vector3> current_d_foot_pos(stikp_size);
    for (size_t i = 0; i < stikp_size; i++) {
        current_d_foot_pos[i] = foot_origin_rot * stikp[i].d_foot_pos;
    }

    // Swing ee compensation.
    calcSwingEEModification();

    // TODO: IKの位置はここじゃない？ or stikpのループを1つにする
    // solveIK
    //   IK target is link origin pos and rot, not ee pos and rot.
    std::vector<hrp::Vector3>  ref_ee_p(stikp_size); // TODO: should be target_ee_p ?
    std::vector<hrp::Matrix33> ref_ee_R(stikp_size);
    double tmp_d_pos_z_root = 0.0;
    for (size_t i = 0; i < stikp_size; i++) {
        if (!is_ik_enable[i]) continue;

        // Add damping_control compensation to target value
        if (is_feedback_control_enable[i]) {
            ref_ee_R[i] = normalizeMatProduct(target_ee_R[i], hrp::rotFromRpy(-1 * stikp[i].ee_d_foot_rpy));
            // foot force difference control version
            // total_target_foot_p[i](2) = target_foot_p[i](2) + (i==0?0.5:-0.5)*zctrl;
            // foot force independent damping control
            ref_ee_p[i] = target_ee_p[i] - current_d_foot_pos[i];
        } else {
            ref_ee_p[i] = target_ee_p[i];
            ref_ee_R[i] = target_ee_R[i];
        }
        // Add swing ee compensation
        ref_ee_R[i] = normalizeMatProduct(ref_ee_R[i], hrp::rotFromRpy(stikp[i].d_rpy_swing));
        ref_ee_p[i] = ref_ee_p[i] + foot_origin_rot * stikp[i].d_pos_swing;
    }

    for (size_t i = 0; i < stikp_size; i++) {
        if (!is_ik_enable[i]) continue;
        for (size_t _ = 0; _ < stikp[i].ik_loop_count; ++_) {
            jpe_v[i]->calcInverseKinematics2Loop(ref_ee_p[i], ref_ee_R[i], 1.0, 0.001, 0.01,
                                                 &qrefv, transition_smooth_gain,
                                                 //stikp[i].localCOPPos;
                                                 stikp[i].localp, stikp[i].localR);
        }
    }
}

// Swing ee compensation.
//   Calculate compensation values to minimize the difference between "current" foot-origin-coords-relative pos and rot and "target" foot-origin-coords-relative pos and rot for swing ee.
//   Input  : target_ee_diff_p, target_ee_diff_r
//   Output : d_pos_swing, d_rpy_swing
void Stabilizer::calcSwingEEModification ()
{
    for (size_t i = 0; i < stikp.size(); i++) {
        // Calc compensation values
        constexpr double limit_pos = 30 * 1e-3; // 30[mm] limit
        constexpr double limit_rot = deg2rad(10); // 10[deg] limit
        if (ref_contact_states[contact_states_index_map[stikp[i].ee_name]] ||
            act_contact_states[contact_states_index_map[stikp[i].ee_name]]) {
            // If actual contact or target contact is ON, do not use swing ee compensation. Exponential zero retrieving.
            stikp[i].d_rpy_swing = calcDampingControl(stikp[i].d_rpy_swing, stikp[i].eefm_swing_rot_time_const);
            stikp[i].d_pos_swing = calcDampingControl(stikp[i].d_pos_swing, stikp[i].eefm_swing_pos_time_const);
            stikp[i].target_ee_diff_p_filter->reset(stikp[i].d_pos_swing);
            stikp[i].target_ee_diff_r_filter->reset(stikp[i].d_rpy_swing);
        } else {
            /* position */
            {
                const hrp::Vector3 diff_p = stikp[i].eefm_swing_pos_spring_gain.cwiseProduct(stikp[i].target_ee_diff_p_filter->passFilter(stikp[i].target_ee_diff_p));
                const double lvlimit = -50 * 1e-3 * dt;
                const double uvlimit =  50 * 1e-3 * dt; // 50 [mm/s]
                const hrp::Vector3 limit_by_lvlimit = stikp[i].prev_d_pos_swing + lvlimit * hrp::Vector3::Ones();
                const hrp::Vector3 limit_by_uvlimit = stikp[i].prev_d_pos_swing + uvlimit * hrp::Vector3::Ones();
                stikp[i].d_pos_swing = clamp(clamp(diff_p, -limit_pos, limit_pos), limit_by_lvlimit, limit_by_uvlimit);
            }
            /* rotation */
            {
                const hrp::Vector3 diff_r = stikp[i].eefm_swing_rot_spring_gain.cwiseProduct(stikp[i].target_ee_diff_r_filter->passFilter(hrp::rpyFromRot(stikp[i].target_ee_diff_r)));
                const double lvlimit = deg2rad(-20.0 * dt);
                const double uvlimit = deg2rad(20.0 * dt); // 20 [deg/s]
                const hrp::Vector3 limit_by_lvlimit = stikp[i].prev_d_rpy_swing + lvlimit * hrp::Vector3::Ones();
                const hrp::Vector3 limit_by_uvlimit = stikp[i].prev_d_rpy_swing + uvlimit * hrp::Vector3::Ones();
                stikp[i].d_rpy_swing = clamp(clamp(diff_r, -limit_rot, limit_rot), limit_by_lvlimit, limit_by_uvlimit);
            }
        }
        stikp[i].prev_d_pos_swing = stikp[i].d_pos_swing;
        stikp[i].prev_d_rpy_swing = stikp[i].d_rpy_swing;
    }

    if (DEBUGP(loop)) {
        std::cerr << "[" << comp_name << "] Swing foot control" << std::endl;
        for (size_t i = 0; i < stikp.size(); i++) {
            std::cerr << "[" << comp_name << "]   "
                      << "d_rpy_swing (" << stikp[i].ee_name << ")  = "
                      << (stikp[i].d_rpy_swing / M_PI * 180.0).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                      << "[deg], "
                      << "d_pos_swing (" << stikp[i].ee_name << ")  = "
                      << (stikp[i].d_pos_swing * 1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                      << "[mm]" << std::endl;
        }
    }
}

// Damping control functions
//   Basically Equation (14) in the paper [1]
double Stabilizer::calcDampingControl(const double tau_d, const double tau, const double prev_d,
                                       const double DD, const double TT)
{
    return (1 / DD * (tau_d - tau) - 1 / TT * prev_d) * dt + prev_d;
}

hrp::Vector3 Stabilizer::calcDampingControl(const hrp::Vector3& tau_d, const hrp::Vector3& tau, const hrp::Vector3& prev_d,
                                             const hrp::Vector3& DD, const hrp::Vector3& TT)
{
    return ((tau_d - tau).cwiseQuotient(DD) - prev_d.cwiseQuotient(TT)) * dt + prev_d;
}

// Retrieving only
double Stabilizer::calcDampingControl(const double prev_d, const double TT)
{
    return -1 / TT * prev_d * dt + prev_d;
}

// Retrieving only
hrp::Vector3 Stabilizer::calcDampingControl(const hrp::Vector3& prev_d, const hrp::Vector3& TT)
{
    return (-prev_d.cwiseQuotient(TT)) * dt + prev_d;
}

void Stabilizer::setSwingSupportJointServoGains()
{
    static double tmp_landing2support_transition_time = landing2support_transition_time;

    const size_t stikp_size = stikp.size();
    for (size_t i = 0; i < stikp_size; i++) {
        STIKParam& ikp = stikp[i];
        const auto& jpe = jpe_v[i];
        if (ikp.contact_phase == SWING_PHASE && !ref_contact_states[i] && control_swing_support_time[i] < swing2landing_transition_time + landing_phase_time) { // SWING -> LANDING
            change_servo_gains = true;
            ikp.contact_phase = LANDING_PHASE;
            ikp.phase_time = 0;
            for (size_t j = 0, joint_num = ikp.support_pgain.size(); j < joint_num; ++j) {
                const int joint_id = jpe->joint(j)->jointId;
                servo_pgains[joint_id] = ikp.landing_pgain(j);
                servo_dgains[joint_id] = ikp.landing_dgain(j);
                gains_transition_times[joint_id] = swing2landing_transition_time;
            }
        }

        if (ikp.contact_phase == LANDING_PHASE && act_contact_states[i] && ref_contact_states[i] && ikp.phase_time > swing2landing_transition_time) { // LANDING -> SUPPORT
            change_servo_gains = true;
            ikp.contact_phase = SUPPORT_PHASE;
            ikp.phase_time = 0;
            tmp_landing2support_transition_time = std::min(landing2support_transition_time, control_swing_support_time[i]);
            for (size_t j = 0, joint_num = ikp.support_pgain.size(); j < joint_num; ++j) {
                const int joint_id = jpe->joint(j)->jointId;
                servo_pgains[joint_id] = ikp.support_pgain(j);
                servo_dgains[joint_id] = ikp.support_dgain(j);
                gains_transition_times[joint_id] = tmp_landing2support_transition_time;
            }
        }

        // if (ikp.contact_phase == SUPPORT_PHASE && !act_contact_states[i] && ikp.phase_time > tmp_landing2support_transition_time) { // SUPPORT -> SWING
        if (ikp.contact_phase == SUPPORT_PHASE && !act_contact_states[i] && ikp.phase_time > tmp_landing2support_transition_time
            // && ( (ref_contact_states[i] && control_swing_support_time[i] < 0.2) || !ref_contact_states[i] )) { // SUPPORT -> SWING
            && !ref_contact_states[i] ) { // SUPPORT -> SWING
            ikp.contact_phase = SWING_PHASE;
            ikp.phase_time = 0;
        }
        ikp.phase_time += dt;
    }
}

void Stabilizer::calcExternalForce(const hrp::Vector3& cog, const hrp::Vector3& zmp, const hrp::Matrix33& rot)
{
    // cog and zmp must be in the same coords with stikp.ref_forece
    hrp::Vector3 total_force = hrp::Vector3::Zero();
    for (size_t j = 0; j < stikp.size(); ++j) {
        total_force(2) += stikp[j].ref_force(2); // only fz
    }

    total_force.head<2>() = (cog.head<2>() - zmp.head<2>()) * total_force(2) / (cog(2) - zmp(2)); // overwrite fxy
    constexpr double EPS = 1e-6;
    if (total_force(2) < EPS) return;

    for (STIKParam& ikp : stikp) {
        ikp.ref_force.head<2>() += (rot.transpose() * total_force).head<2>() * ikp.ref_force(2) / total_force(2);

        constexpr double rate_during_landing = 0.1;
        constexpr double wrench_transition_time = 0.15;
        if (ikp.contact_phase == LANDING_PHASE) ikp.ref_moment *= rate_during_landing;
        else if (ikp.contact_phase == SUPPORT_PHASE) {
            ikp.ref_moment *= rate_during_landing + (1 - rate_during_landing) * std::min(1.0, ikp.phase_time / wrench_transition_time);
        }
    }
}

void Stabilizer::calcTorque(const hrp::Matrix33& rot)
{
    m_robot->calcForwardKinematics(true, true);

    hrp::Vector3 root_w_x_v;
    hrp::Vector3 g(0, 0, 9.80665);
    root_w_x_v = m_robot->rootLink()->w.cross(m_robot->rootLink()->vo + m_robot->rootLink()->w.cross(m_robot->rootLink()->p));
    m_robot->rootLink()->dvo = g - root_w_x_v; // dv = g, dw = 0
    m_robot->rootLink()->dw.setZero();

    hrp::Vector3 root_f;
    hrp::Vector3 root_tau;
    m_robot->calcInverseDynamics(m_robot->rootLink(), root_f, root_tau);

    if (control_mode == MODE_ST) {
        for (const STIKParam& ikp : stikp) {
            hrp::Link* target = m_robot->link(ikp.target_name);
            const hrp::JointPath jm(m_robot->rootLink(), target);
            const size_t jm_num_joints = jm.numJoints();
            hrp::dmatrix JJ(6, jm_num_joints);
            jm.calcJacobian(JJ);

            hrp::dvector6 ft;
            ft << rot * ikp.ref_force, rot * ikp.ref_moment;
            ft.tail(3) += (- target->R * ikp.localp).cross(ft.head<3>());
            const hrp::dvector tq_from_extft = JJ.transpose() * ft; // size: jm.numJoints()

            for (size_t i = 0; i < jm_num_joints; i++) jm.joint(i)->u -= tq_from_extft(i);
        }
    }

    const size_t num_joints = m_robot->numJoints();
    for (size_t i = 0; i < num_joints; ++i) {
        m_robot->joint(i)->u *= transition_smooth_gain;
        // prev_dqv[i] = m_robot->joint(i)->dq;
    }
}

void Stabilizer::syncToSt()
{
    std::cerr << "[" << comp_name << "] Sync IDLE => ST"  << std::endl;
    // TODO: この辺の初期化をまとめたい
    d_rpy[0] = d_rpy[1] = 0;
    pos_ctrl = hrp::Vector3::Zero();
    for (STIKParam& ikp : stikp) {
        ikp.target_ee_diff_p = hrp::Vector3::Zero();
        ikp.target_ee_diff_r = hrp::Matrix33::Identity();

        ikp.d_pos_swing      = hrp::Vector3::Zero();
        ikp.prev_d_pos_swing = hrp::Vector3::Zero();
        ikp.d_rpy_swing      = hrp::Vector3::Zero();
        ikp.prev_d_rpy_swing = hrp::Vector3::Zero();

        ikp.d_foot_pos    = hrp::Vector3::Zero();
        ikp.ee_d_foot_pos = hrp::Vector3::Zero();
        ikp.d_foot_rpy    = hrp::Vector3::Zero();
        ikp.ee_d_foot_rpy = hrp::Vector3::Zero();

        ikp.target_ee_diff_p_filter->reset(hrp::Vector3::Zero());
        ikp.target_ee_diff_r_filter->reset(hrp::Vector3::Zero());
    }

    if (on_ground) {
        transition_count = -calcMaxTransitionCount();
        control_mode = MODE_ST;
    } else {
        transition_count = 0;
        control_mode = MODE_AIR;
    }
}

void Stabilizer::syncToIdle()
{
    std::cerr << "[" << comp_name << "] Sync ST => IDLE"  << std::endl;
    transition_count = calcMaxTransitionCount();
    for (size_t i = 0; i < m_robot->numJoints(); i++) {
        transition_joint_q[i] = m_robot->joint(i)->q;
    }
}

void Stabilizer::startStabilizer()
{
    // Wait until all transition has finished
    waitSTTransition();
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (control_mode == MODE_IDLE) {
            std::cerr << "[" << comp_name << "] " << "Start ST"  << std::endl;
            syncToSt();
        }
    }
    waitSTTransition(); // TODO: 後ろ？

    if (joint_control_mode == OpenHRP::AutoBalanceStabilizerService::JOINT_TORQUE) {
        std::cerr << "[" << comp_name << "] " << "Moved to ST command pose and sync to TORQUE mode"  << std::endl;
        constexpr double DEFAULT_TRANSITION_TIME = 2.0;

        std::lock_guard<std::mutex> lock(m_mutex);
        for (size_t i = 0; i < m_robot->numJoints(); ++i) {
            servo_pgains[i] = 100;
            servo_dgains[i] = 100;
            servo_tqpgains[i] = 100;
            // servo_tqdgains[i] = 100;
            gains_transition_times[i] = DEFAULT_TRANSITION_TIME;
        }

        // m_robotHardwareService0->setServoGainPercentage("all", 100);
        // m_robotHardwareService0->setServoTorqueGainPercentage("all", 100);
        for (size_t i = 0; i < stikp.size(); ++i) {
            const STIKParam& ikp = stikp[i];
            const auto& jpe = jpe_v[i];
            for (size_t j = 0; j < ikp.support_pgain.size(); ++j) {
                const int joint_id = jpe->joint(j)->jointId;
                servo_pgains[joint_id] = ikp.support_pgain(j);
                servo_dgains[joint_id] = ikp.support_dgain(j);
                gains_transition_times[joint_id] = DEFAULT_TRANSITION_TIME;
            }
        }

        change_servo_gains = true;
        // for (size_t j = 0; j < ikp.support_pgain.size(); j++) {
        //     m_robotHardwareService0->setServoPGainPercentageWithTime(jpe->joint(j)->name.c_str(), ikp.support_pgain(j), 3);
        //     m_robotHardwareService0->setServoDGainPercentageWithTime(jpe->joint(j)->name.c_str(), ikp.support_dgain(j), 3);
        // }
    }

    std::cerr << "[" << comp_name << "] " << "Start ST DONE"  << std::endl;
}

void Stabilizer::stopStabilizer()
{
    waitSTTransition(); // Wait until all transition has finished
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if ((control_mode == MODE_ST || control_mode == MODE_AIR)) {
            std::cerr << "[" << comp_name << "] " << "Stop ST"  << std::endl;
            control_mode = (control_mode == MODE_ST) ? MODE_SYNC_TO_IDLE : MODE_IDLE;
        }
    }
    waitSTTransition();

    if (joint_control_mode == OpenHRP::AutoBalanceStabilizerService::JOINT_TORQUE) {
        std::cerr << "[" << comp_name << "] " << "Sync to Position mode"  << std::endl;
        constexpr double DEFAULT_TRANSITION_TIME = 2.0;
        for (size_t i = 0; i < m_robot->numJoints(); ++i) {
            servo_pgains[i] = 100;
            servo_dgains[i] = 100;
            servo_tqpgains[i] = 0;
            // servo_tqdgains[i] = 0;
            gains_transition_times[i] = DEFAULT_TRANSITION_TIME;
        }
        servo_pgains[0] = servo_pgains[6] = 5;
        servo_dgains[0] = servo_dgains[6] = 30;
        change_servo_gains = true;
    }

    std::cerr << "[" << comp_name << "] " << "Stop ST DONE"  << std::endl;
}

std::string Stabilizer::getStabilizerAlgorithmString(const OpenHRP::AutoBalanceStabilizerService::STAlgorithm _st_algorithm)
{
    switch (_st_algorithm) {
    case OpenHRP::AutoBalanceStabilizerService::TPCC:
        return "TPCC";
    case OpenHRP::AutoBalanceStabilizerService::EEFM:
        return "EEFM";
    case OpenHRP::AutoBalanceStabilizerService::EEFMQP:
        return "EEFMQP";
    case OpenHRP::AutoBalanceStabilizerService::EEFMQPCOP:
        return "EEFMQPCOP";
    case OpenHRP::AutoBalanceStabilizerService::EEFMQPCOP2:
        return "EEFMQPCOP2";
    default:
        return "";
    }
}

void Stabilizer::setBoolSequenceParam(std::vector<bool>& st_bool_values,
                                      const OpenHRP::AutoBalanceStabilizerService::BoolSequence& output_bool_values,
                                      const std::string& prop_name)
{
    const std::vector<bool> prev_values(st_bool_values);

    if (st_bool_values.size() != output_bool_values.length()) {
        std::cerr << "[" << comp_name << "]   " << prop_name
                  << " cannot be set. Length " << st_bool_values.size()
                  << " != " << output_bool_values.length() << std::endl;
    } else if (control_mode != MODE_IDLE) {
        std::cerr << "[" << comp_name << "]   " << prop_name
                  << " cannot be set. Current control_mode is "
                  << control_mode << std::endl;
    } else {
        for (size_t i = 0; i < st_bool_values.size(); i++) {
            st_bool_values[i] = output_bool_values[i];
        }
    }

    std::cerr << "[" << comp_name << "]   " << prop_name << " is ";
    for (size_t i = 0; i < st_bool_values.size(); i++) {
        std::cerr <<"[" << st_bool_values[i] << "]";
    }
    std::cerr << "(set = ";
    for (size_t i = 0; i < output_bool_values.length(); i++) {
        std::cerr <<"[" << output_bool_values[i] << "]";
    }
    std::cerr << ", prev = ";
    for (size_t i = 0; i < prev_values.size(); i++) {
        std::cerr <<"[" << prev_values[i] << "]";
    }
    std::cerr << ")" << std::endl;
}

// TODO: 上のを合わせられない?
void Stabilizer::setBoolSequenceParamWithCheckContact(std::vector<bool>& st_bool_values,
                                                      const OpenHRP::AutoBalanceStabilizerService::BoolSequence& output_bool_values,
                                                      const std::string& prop_name)
{
    std::vector<bool> prev_values(st_bool_values);

    if (st_bool_values.size() != output_bool_values.length()) {
        std::cerr << "[" << comp_name << "]   " << prop_name
                  << " cannot be set. Length " << st_bool_values.size()
                  << " != " << output_bool_values.length() << std::endl;
    } else if ( control_mode == MODE_IDLE ) {
        for (size_t i = 0; i < st_bool_values.size(); i++) {
            st_bool_values[i] = output_bool_values[i];
        }
    } else {
        std::vector<size_t> failed_indices;
        for (size_t i = 0; i < st_bool_values.size(); i++) {
            if (st_bool_values[i] != output_bool_values[i]) { // If mode change
                // reference contact_states should be OFF
                if (!ref_contact_states[i]) st_bool_values[i] = output_bool_values[i];
                else failed_indices.push_back(i);
            }
        }
        if (failed_indices.size() > 0) {
            std::cerr << "[" << comp_name << "]   " << prop_name << " cannot be set partially. failed_indices is [";
            for (size_t i = 0; i < failed_indices.size(); i++) {
                std::cerr << failed_indices[i] << " ";
            }
            std::cerr << "]" << std::endl;
        }
    }

    std::cerr << "[" << comp_name << "]   " << prop_name << " is ";
    for (size_t i = 0; i < st_bool_values.size(); i++) {
        std::cerr <<"[" << st_bool_values[i] << "]";
    }
    std::cerr << "(set = ";
    for (size_t i = 0; i < output_bool_values.length(); i++) {
        std::cerr <<"[" << output_bool_values[i] << "]";
    }
    std::cerr << ", prev = ";
    for (size_t i = 0; i < prev_values.size(); i++) {
        std::cerr <<"[" << prev_values[i] << "]";
    }
    std::cerr << ")" << std::endl;
}

void Stabilizer::waitSTTransition()
{
    // Wait condition
    //   1. Check transition_count : Wait until transition is finished
    //   2. Check control_mode : Once control_mode is SYNC mode, wait until control_mode moves to the next mode (MODE_AIR or MODE_IDLE)
    bool flag = (control_mode == MODE_SYNC_TO_AIR || control_mode == MODE_SYNC_TO_IDLE);
    while (transition_count != 0 ||
           (flag ? !(control_mode == MODE_IDLE || control_mode == MODE_AIR) : false)) {
        usleep(10);
        flag = (control_mode == MODE_SYNC_TO_AIR || control_mode == MODE_SYNC_TO_IDLE);
    }
    usleep(10);
}

void Stabilizer::getStabilizerParam(OpenHRP::AutoBalanceStabilizerService::StabilizerParam& i_stp)
{
    std::cerr << "[" << comp_name << "] getStabilizerParam" << std::endl;

    for (size_t i = 0; i < 2; i++) {
        i_stp.k_tpcc_p[i] = k_tpcc_p[i];
        i_stp.k_tpcc_x[i] = k_tpcc_x[i];
        i_stp.k_brot_p[i] = k_brot_p[i];
        i_stp.k_brot_tc[i] = k_brot_tc[i];
    }

    for (size_t i = 0; i < 2; i++) {
        i_stp.eefm_k1[i] = eefm_k1[i];
        i_stp.eefm_k2[i] = eefm_k2[i];
        i_stp.eefm_k3[i] = eefm_k3[i];
        i_stp.eefm_zmp_delay_time_const[i] = eefm_zmp_delay_time_const[i];
        i_stp.eefm_ref_zmp_aux[i] = ref_zmp_aux(i);
        i_stp.eefm_body_attitude_control_time_const[i] = eefm_body_attitude_control_time_const[i];
        i_stp.eefm_body_attitude_control_gain[i] = eefm_body_attitude_control_gain[i];
        i_stp.ref_capture_point[i] = ref_cp(i);
        i_stp.act_capture_point[i] = act_cp(i);
        i_stp.cp_offset[i] = cp_offset(i);
    }

    const size_t stikp_size = stikp.size();
    i_stp.eefm_pos_time_const_support.length(stikp_size);
    i_stp.eefm_pos_damping_gain.length(stikp_size);
    i_stp.eefm_pos_compensation_limit.length(stikp_size);
    i_stp.eefm_swing_pos_spring_gain.length(stikp_size);
    i_stp.eefm_swing_pos_time_const.length(stikp_size);
    i_stp.eefm_rot_time_const.length(stikp_size);
    i_stp.eefm_rot_damping_gain.length(stikp_size);
    i_stp.eefm_rot_compensation_limit.length(stikp_size);
    i_stp.eefm_swing_rot_spring_gain.length(stikp_size);
    i_stp.eefm_swing_rot_time_const.length(stikp_size);
    i_stp.eefm_ee_moment_limit.length(stikp_size);
    i_stp.eefm_ee_forcemoment_distribution_weight.length(stikp_size);
    for (size_t j = 0; j < stikp_size; j++) {
        i_stp.eefm_pos_damping_gain[j].length(3);
        i_stp.eefm_pos_time_const_support[j].length(3);
        i_stp.eefm_swing_pos_spring_gain[j].length(3);
        i_stp.eefm_swing_pos_time_const[j].length(3);
        i_stp.eefm_rot_damping_gain[j].length(3);
        i_stp.eefm_rot_time_const[j].length(3);
        i_stp.eefm_swing_rot_spring_gain[j].length(3);
        i_stp.eefm_swing_rot_time_const[j].length(3);
        i_stp.eefm_ee_moment_limit[j].length(3);
        i_stp.eefm_ee_forcemoment_distribution_weight[j].length(6);
        for (size_t i = 0; i < 3; i++) {
            i_stp.eefm_pos_damping_gain[j][i] = stikp[j].eefm_pos_damping_gain(i);
            i_stp.eefm_pos_time_const_support[j][i] = stikp[j].eefm_pos_time_const_support(i);
            i_stp.eefm_swing_pos_spring_gain[j][i] = stikp[j].eefm_swing_pos_spring_gain(i);
            i_stp.eefm_swing_pos_time_const[j][i] = stikp[j].eefm_swing_pos_time_const(i);
            i_stp.eefm_rot_damping_gain[j][i] = stikp[j].eefm_rot_damping_gain(i);
            i_stp.eefm_rot_time_const[j][i] = stikp[j].eefm_rot_time_const(i);
            i_stp.eefm_swing_rot_spring_gain[j][i] = stikp[j].eefm_swing_rot_spring_gain(i);
            i_stp.eefm_swing_rot_time_const[j][i] = stikp[j].eefm_swing_rot_time_const(i);
            i_stp.eefm_ee_moment_limit[j][i] = stikp[j].eefm_ee_moment_limit(i);
            i_stp.eefm_ee_forcemoment_distribution_weight[j][i] = stikp[j].eefm_ee_forcemoment_distribution_weight(i);
            i_stp.eefm_ee_forcemoment_distribution_weight[j][i+3] = stikp[j].eefm_ee_forcemoment_distribution_weight(i+3);
        }
        i_stp.eefm_pos_compensation_limit[j] = stikp[j].eefm_pos_compensation_limit;
        i_stp.eefm_rot_compensation_limit[j] = stikp[j].eefm_rot_compensation_limit;
    }

    for (size_t i = 0; i < 3; i++) {
        i_stp.eefm_swing_pos_damping_gain[i] = eefm_swing_pos_damping_gain(i);
        i_stp.eefm_swing_rot_damping_gain[i] = eefm_swing_rot_damping_gain(i);
    }

    i_stp.eefm_pos_time_const_swing = eefm_pos_time_const_swing;
    i_stp.eefm_pos_transition_time = eefm_pos_transition_time;
    i_stp.eefm_pos_margin_time = eefm_pos_margin_time;
    i_stp.eefm_leg_inside_margin = szd->get_leg_inside_margin();
    i_stp.eefm_leg_outside_margin = szd->get_leg_outside_margin();
    i_stp.eefm_leg_front_margin = szd->get_leg_front_margin();
    i_stp.eefm_leg_rear_margin = szd->get_leg_rear_margin();

    std::vector<std::vector<Eigen::Vector2d> > support_polygon_vec;
    szd->get_vertices(support_polygon_vec);
    i_stp.eefm_support_polygon_vertices_sequence.length(support_polygon_vec.size());
    for (size_t ee_idx = 0; ee_idx < support_polygon_vec.size(); ee_idx++) {
        i_stp.eefm_support_polygon_vertices_sequence[ee_idx].vertices.length(support_polygon_vec[ee_idx].size());
        for (size_t v_idx = 0; v_idx < support_polygon_vec[ee_idx].size(); v_idx++) {
            i_stp.eefm_support_polygon_vertices_sequence[ee_idx].vertices[v_idx].pos[0] = support_polygon_vec[ee_idx][v_idx](0);
            i_stp.eefm_support_polygon_vertices_sequence[ee_idx].vertices[v_idx].pos[1] = support_polygon_vec[ee_idx][v_idx](1);
        }
    }

    i_stp.eefm_cogvel_cutoff_freq = act_cogvel_filter->getCutOffFreq();
    i_stp.eefm_wrench_alpha_blending = szd->get_wrench_alpha_blending();
    i_stp.eefm_alpha_cutoff_freq = szd->get_alpha_cutoff_freq();
    i_stp.gravitational_acceleration = g_acc;
    i_stp.eefm_ee_error_cutoff_freq = stikp[0].target_ee_diff_p_filter->getCutOffFreq();
    i_stp.eefm_use_force_difference_control = eefm_use_force_difference_control;
    i_stp.eefm_use_swing_damping = eefm_use_swing_damping;

    for (size_t i = 0; i < 3; ++i) {
        i_stp.eefm_swing_damping_force_thre[i] = eefm_swing_damping_force_thre[i];
        i_stp.eefm_swing_damping_moment_thre[i] = eefm_swing_damping_moment_thre[i];
    }
    i_stp.is_ik_enable.length(is_ik_enable.size());
    for (size_t i = 0; i < is_ik_enable.size(); i++) {
        i_stp.is_ik_enable[i] = is_ik_enable[i];
    }
    i_stp.is_feedback_control_enable.length(is_feedback_control_enable.size());
    for (size_t i = 0; i < is_feedback_control_enable.size(); i++) {
        i_stp.is_feedback_control_enable[i] = is_feedback_control_enable[i];
    }
    i_stp.is_zmp_calc_enable.length(is_zmp_calc_enable.size());
    for (size_t i = 0; i < is_zmp_calc_enable.size(); i++) {
        i_stp.is_zmp_calc_enable[i] = is_zmp_calc_enable[i];
    }

    i_stp.foot_origin_offset.length(2);
    for (size_t i = 0; i < i_stp.foot_origin_offset.length(); i++) {
        i_stp.foot_origin_offset[i].length(3);
        i_stp.foot_origin_offset[i][0] = foot_origin_offset[i](0);
        i_stp.foot_origin_offset[i][1] = foot_origin_offset[i](1);
        i_stp.foot_origin_offset[i][2] = foot_origin_offset[i](2);
    }

    i_stp.st_algorithm = st_algorithm;
    i_stp.transition_time = transition_time;
    i_stp.cop_check_margin = cop_check_margin;
    for (size_t i = 0; i < cp_check_margin.size(); i++) {
        i_stp.cp_check_margin[i] = cp_check_margin[i];
    }
    for (size_t i = 0; i < tilt_margin.size(); i++) {
        i_stp.tilt_margin[i] = tilt_margin[i];
    }
    i_stp.contact_decision_threshold = contact_decision_threshold;
    i_stp.is_estop_while_walking = is_estop_while_walking;
    switch(control_mode) {
      case MODE_IDLE: i_stp.controller_mode = OpenHRP::AutoBalanceStabilizerService::MODE_IDLE; break;
      case MODE_AIR: i_stp.controller_mode = OpenHRP::AutoBalanceStabilizerService::MODE_AIR; break;
      case MODE_ST: i_stp.controller_mode = OpenHRP::AutoBalanceStabilizerService::MODE_ST; break;
      case MODE_SYNC_TO_IDLE: i_stp.controller_mode = OpenHRP::AutoBalanceStabilizerService::MODE_SYNC_TO_IDLE; break;
      case MODE_SYNC_TO_AIR: i_stp.controller_mode = OpenHRP::AutoBalanceStabilizerService::MODE_SYNC_TO_AIR; break;
      default: break;
    }
    i_stp.emergency_check_mode = emergency_check_mode;
    i_stp.end_effector_list.length(stikp_size);
    i_stp.use_limb_stretch_avoidance = use_limb_stretch_avoidance;
    i_stp.use_zmp_truncation = use_zmp_truncation;
    i_stp.limb_stretch_avoidance_time_const = limb_stretch_avoidance_time_const;
    i_stp.limb_length_margin.length(stikp_size);
    i_stp.detection_time_to_air = detection_count_to_mode_air * dt;
    i_stp.detection_time_to_air = 1.0; // tmp
    for (size_t i = 0; i < 2; i++) {
        i_stp.limb_stretch_avoidance_vlimit[i] = limb_stretch_avoidance_vlimit[i];
        i_stp.root_rot_compensation_limit[i] = root_rot_compensation_limit[i];
    }
    for (size_t i = 0; i < stikp_size; i++) {
        const rats::coordinates cur_ee = rats::coordinates(stikp.at(i).localp, stikp.at(i).localR);
        OpenHRP::AutoBalanceStabilizerService::Footstep ret_ee;
        // position
        memcpy(ret_ee.pos, cur_ee.pos.data(), sizeof(double)*3);
        // rotation
        Eigen::Quaternion<double> qt(cur_ee.rot);
        ret_ee.rot[0] = qt.w();
        ret_ee.rot[1] = qt.x();
        ret_ee.rot[2] = qt.y();
        ret_ee.rot[3] = qt.z();
        // name
        ret_ee.leg = stikp.at(i).ee_name.c_str();
        // set
        i_stp.end_effector_list[i] = ret_ee;
        i_stp.limb_length_margin[i] = stikp[i].limb_length_margin;
    }
    i_stp.ik_limb_parameters.length(jpe_v.size());
    for (size_t i = 0; i < jpe_v.size(); i++) {
        OpenHRP::AutoBalanceStabilizerService::IKLimbParameters& ilp = i_stp.ik_limb_parameters[i];
        ilp.ik_optional_weight_vector.length(jpe_v[i]->numJoints());
        std::vector<double> ov;
        ov.resize(jpe_v[i]->numJoints());
        jpe_v[i]->getOptionalWeightVector(ov);
        for (size_t j = 0; j < jpe_v[i]->numJoints(); j++) {
            ilp.ik_optional_weight_vector[j] = ov[j];
        }
        ilp.sr_gain = jpe_v[i]->getSRGain();
        ilp.avoid_gain = stikp[i].avoid_gain;
        ilp.reference_gain = stikp[i].reference_gain;
        ilp.manipulability_limit = jpe_v[i]->getManipulabilityLimit();
        ilp.ik_loop_count = stikp[i].ik_loop_count; // size_t -> unsigned short, value may change, but ik_loop_count is small value and value not change
    }
}

// TODO: move to Service_impl ?
void Stabilizer::setStabilizerParam(const OpenHRP::AutoBalanceStabilizerService::StabilizerParam& i_stp)
{
    std::cerr << "[" << comp_name << "] setStabilizerParam" << std::endl;
    std::lock_guard<std::mutex> lock(m_mutex);

    for (size_t i = 0; i < 2; i++) {
        k_tpcc_p[i] = i_stp.k_tpcc_p[i];
        k_tpcc_x[i] = i_stp.k_tpcc_x[i];
        k_brot_p[i] = i_stp.k_brot_p[i];
        k_brot_tc[i] = i_stp.k_brot_tc[i];
    }

    std::cerr << "[" << comp_name << "]  TPCC" << std::endl;
    std::cerr << "[" << comp_name
              << "]   k_tpcc_p  = [" << k_tpcc_p[0] << ", " << k_tpcc_p[1]
              << "], k_tpcc_x  = [" << k_tpcc_x[0] << ", " << k_tpcc_x[1]
              << "], k_brot_p  = [" << k_brot_p[0] << ", " << k_brot_p[1]
              << "], k_brot_tc = [" << k_brot_tc[0] << ", " << k_brot_tc[1]
              << "]" << std::endl;

    std::cerr << "[" << comp_name << "]  EEFM" << std::endl;
    for (size_t i = 0; i < 2; i++) {
        eefm_k1[i] = i_stp.eefm_k1[i];
        eefm_k2[i] = i_stp.eefm_k2[i];
        eefm_k3[i] = i_stp.eefm_k3[i];
        eefm_zmp_delay_time_const[i] = i_stp.eefm_zmp_delay_time_const[i];
        ref_zmp_aux(i) = i_stp.eefm_ref_zmp_aux[i];
        eefm_body_attitude_control_gain[i] = i_stp.eefm_body_attitude_control_gain[i];
        eefm_body_attitude_control_time_const[i] = i_stp.eefm_body_attitude_control_time_const[i];
        ref_cp(i) = i_stp.ref_capture_point[i];
        act_cp(i) = i_stp.act_capture_point[i];
        cp_offset(i) = i_stp.cp_offset[i];
    }

    const size_t stikp_size = stikp.size();
    bool is_damping_parameter_ok = false;
    if (i_stp.eefm_pos_damping_gain.length()                   == stikp_size &&
        i_stp.eefm_pos_time_const_support.length()             == stikp_size &&
        i_stp.eefm_pos_compensation_limit.length()             == stikp_size &&
        i_stp.eefm_swing_pos_spring_gain.length()              == stikp_size &&
        i_stp.eefm_swing_pos_time_const.length()               == stikp_size &&
        i_stp.eefm_rot_damping_gain.length()                   == stikp_size &&
        i_stp.eefm_rot_time_const.length()                     == stikp_size &&
        i_stp.eefm_rot_compensation_limit.length()             == stikp_size &&
        i_stp.eefm_swing_rot_spring_gain.length()              == stikp_size &&
        i_stp.eefm_swing_rot_time_const.length()               == stikp_size &&
        i_stp.eefm_ee_moment_limit.length()                    == stikp_size &&
        i_stp.eefm_ee_forcemoment_distribution_weight.length() == stikp_size) {

        is_damping_parameter_ok = true;
        for (size_t j = 0; j < stikp_size; j++) {
            for (size_t i = 0; i < 3; i++) {
                stikp[j].eefm_pos_damping_gain(i) = i_stp.eefm_pos_damping_gain[j][i];
                stikp[j].eefm_pos_time_const_support(i) = i_stp.eefm_pos_time_const_support[j][i];
                stikp[j].eefm_swing_pos_spring_gain(i) = i_stp.eefm_swing_pos_spring_gain[j][i];
                stikp[j].eefm_swing_pos_time_const(i) = i_stp.eefm_swing_pos_time_const[j][i];
                stikp[j].eefm_rot_damping_gain(i) = i_stp.eefm_rot_damping_gain[j][i];
                stikp[j].eefm_rot_time_const(i) = i_stp.eefm_rot_time_const[j][i];
                stikp[j].eefm_swing_rot_spring_gain(i) = i_stp.eefm_swing_rot_spring_gain[j][i];
                stikp[j].eefm_swing_rot_time_const(i) = i_stp.eefm_swing_rot_time_const[j][i];
                stikp[j].eefm_ee_moment_limit(i) = i_stp.eefm_ee_moment_limit[j][i];
                stikp[j].eefm_ee_forcemoment_distribution_weight(i) = i_stp.eefm_ee_forcemoment_distribution_weight[j][i];
                stikp[j].eefm_ee_forcemoment_distribution_weight(i+3) = i_stp.eefm_ee_forcemoment_distribution_weight[j][i+3];
            }
            stikp[j].eefm_pos_compensation_limit = i_stp.eefm_pos_compensation_limit[j];
            stikp[j].eefm_rot_compensation_limit = i_stp.eefm_rot_compensation_limit[j];
        }
    }

    for (size_t i = 0; i < 3; i++) {
        eefm_swing_pos_damping_gain(i) = i_stp.eefm_swing_pos_damping_gain[i];
        eefm_swing_rot_damping_gain(i) = i_stp.eefm_swing_rot_damping_gain[i];
    }
    eefm_pos_time_const_swing = i_stp.eefm_pos_time_const_swing;
    eefm_pos_transition_time = i_stp.eefm_pos_transition_time;
    eefm_pos_margin_time = i_stp.eefm_pos_margin_time;
    szd->set_leg_inside_margin(i_stp.eefm_leg_inside_margin);
    szd->set_leg_outside_margin(i_stp.eefm_leg_outside_margin);
    szd->set_leg_front_margin(i_stp.eefm_leg_front_margin);
    szd->set_leg_rear_margin(i_stp.eefm_leg_rear_margin);
    szd->set_vertices_from_margin_params();

    if (i_stp.eefm_support_polygon_vertices_sequence.length() != stikp_size) {
        std::cerr << "[" << comp_name << "]   eefm_support_polygon_vertices_sequence cannot be set. Length "
                  << i_stp.eefm_support_polygon_vertices_sequence.length()
                  << " != " << stikp_size << std::endl;
    } else {
        std::cerr << "[" << comp_name << "]   eefm_support_polygon_vertices_sequence set" << std::endl;
        std::vector<std::vector<Eigen::Vector2d> > support_polygon_vec;
        for (size_t ee_idx = 0; ee_idx < i_stp.eefm_support_polygon_vertices_sequence.length(); ee_idx++) {
            std::vector<Eigen::Vector2d> tvec; // TODO: サイズ分からない？ rename
            for (size_t v_idx = 0; v_idx < i_stp.eefm_support_polygon_vertices_sequence[ee_idx].vertices.length(); v_idx++) {
                tvec.push_back(Eigen::Vector2d(i_stp.eefm_support_polygon_vertices_sequence[ee_idx].vertices[v_idx].pos[0],
                                               i_stp.eefm_support_polygon_vertices_sequence[ee_idx].vertices[v_idx].pos[1]));
            }
            support_polygon_vec.push_back(tvec);
        }
        szd->set_vertices(support_polygon_vec);
        szd->print_vertices(comp_name);
    }

    eefm_use_force_difference_control = i_stp.eefm_use_force_difference_control;
    eefm_use_swing_damping = i_stp.eefm_use_swing_damping;
    for (size_t i = 0; i < 3; ++i) {
        eefm_swing_damping_force_thre[i] = i_stp.eefm_swing_damping_force_thre[i];
        eefm_swing_damping_moment_thre[i] = i_stp.eefm_swing_damping_moment_thre[i];
    }
    act_cogvel_filter->setCutOffFreq(i_stp.eefm_cogvel_cutoff_freq);
    szd->set_wrench_alpha_blending(i_stp.eefm_wrench_alpha_blending);
    szd->set_alpha_cutoff_freq(i_stp.eefm_alpha_cutoff_freq);
    g_acc = i_stp.gravitational_acceleration;
    for (size_t i = 0; i < stikp_size; i++) {
        stikp[i].target_ee_diff_p_filter->setCutOffFreq(i_stp.eefm_ee_error_cutoff_freq);
        stikp[i].target_ee_diff_r_filter->setCutOffFreq(i_stp.eefm_ee_error_cutoff_freq);
        stikp[i].limb_length_margin = i_stp.limb_length_margin[i];
    }

    setBoolSequenceParam(is_ik_enable, i_stp.is_ik_enable, std::string("is_ik_enable"));
    setBoolSequenceParamWithCheckContact(is_feedback_control_enable, i_stp.is_feedback_control_enable, std::string("is_feedback_control_enable"));
    setBoolSequenceParam(is_zmp_calc_enable, i_stp.is_zmp_calc_enable, std::string("is_zmp_calc_enable"));

    emergency_check_mode = i_stp.emergency_check_mode;
    transition_time = i_stp.transition_time;
    cop_check_margin = i_stp.cop_check_margin;
    for (size_t i = 0; i < cp_check_margin.size(); i++) {
        cp_check_margin[i] = i_stp.cp_check_margin[i];
    }
    szd->set_vertices_from_margin_params(cp_check_margin);
    for (size_t i = 0; i < tilt_margin.size(); i++) {
        tilt_margin[i] = i_stp.tilt_margin[i];
    }
    contact_decision_threshold = i_stp.contact_decision_threshold;
    is_estop_while_walking = i_stp.is_estop_while_walking;
    use_limb_stretch_avoidance = i_stp.use_limb_stretch_avoidance;
    use_zmp_truncation = i_stp.use_zmp_truncation;
    limb_stretch_avoidance_time_const = i_stp.limb_stretch_avoidance_time_const;
    for (size_t i = 0; i < 2; i++) {
        limb_stretch_avoidance_vlimit[i] = i_stp.limb_stretch_avoidance_vlimit[i];
        root_rot_compensation_limit[i] = i_stp.root_rot_compensation_limit[i];
    }
    detection_count_to_mode_air = static_cast<int>(i_stp.detection_time_to_air / dt);

    if (control_mode == MODE_IDLE) {
        for (size_t i = 0; i < i_stp.end_effector_list.length(); i++) {
            auto ikp_itr = std::find_if(stikp.begin(), stikp.end(), [&i_stp, i](STIKParam& ikp) { return ikp.ee_name == std::string(i_stp.end_effector_list[i].leg); });
            memcpy(ikp_itr->localp.data(), i_stp.end_effector_list[i].pos, sizeof(double)*3);
            ikp_itr->localR = (Eigen::Quaternion<double>(i_stp.end_effector_list[i].rot[0], i_stp.end_effector_list[i].rot[1], i_stp.end_effector_list[i].rot[2], i_stp.end_effector_list[i].rot[3])).normalized().toRotationMatrix();
        }
    } else {
        std::cerr << "[" << comp_name << "] cannot change end-effectors except during MODE_IDLE" << std::endl;
    }

    for (std::vector<STIKParam>::const_iterator it = stikp.begin(); it != stikp.end(); it++) {
        std::cerr << "[" << comp_name << "]  End Effector [" << it->ee_name << "]" << std::endl;
        std::cerr << "[" << comp_name << "]   localpos = " << it->localp.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
        std::cerr << "[" << comp_name << "]   localR = " << it->localR.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", "    [", "]")) << std::endl;
    }

    if (i_stp.foot_origin_offset.length () != 2) {
        std::cerr << "[" << comp_name << "]   foot_origin_offset cannot be set. Length " << i_stp.foot_origin_offset.length() << " != " << 2 << std::endl;
    } else if (control_mode != MODE_IDLE) {
        std::cerr << "[" << comp_name << "]   foot_origin_offset cannot be set. Current control_mode is " << control_mode << std::endl;
    } else {
        for (size_t i = 0; i < i_stp.foot_origin_offset.length(); i++) {
            foot_origin_offset[i](0) = i_stp.foot_origin_offset[i][0];
            foot_origin_offset[i](1) = i_stp.foot_origin_offset[i][1];
            foot_origin_offset[i](2) = i_stp.foot_origin_offset[i][2];
        }
    }

    std::cerr << "[" << comp_name << "]   foot_origin_offset is ";
    for (size_t i = 0; i < 2; i++) {
        std::cerr << foot_origin_offset[i].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"));
    }
    std::cerr << "[m]" << std::endl;
    std::cerr << "[" << comp_name << "]   eefm_k1  = [" << eefm_k1[0] << ", " << eefm_k1[1] << "], eefm_k2  = [" << eefm_k2[0] << ", " << eefm_k2[1] << "], eefm_k3  = [" << eefm_k3[0] << ", " << eefm_k3[1] << "]" << std::endl;
    std::cerr << "[" << comp_name << "]   eefm_zmp_delay_time_const  = [" << eefm_zmp_delay_time_const[0] << ", " << eefm_zmp_delay_time_const[1] << "][s], eefm_ref_zmp_aux  = [" << ref_zmp_aux(0) << ", " << ref_zmp_aux(1) << "][m]" << std::endl;
    std::cerr << "[" << comp_name << "]   eefm_body_attitude_control_gain  = [" << eefm_body_attitude_control_gain[0] << ", " << eefm_body_attitude_control_gain[1] << "], eefm_body_attitude_control_time_const  = [" << eefm_body_attitude_control_time_const[0] << ", " << eefm_body_attitude_control_time_const[1] << "][s]" << std::endl;

    if (is_damping_parameter_ok) {
        for (size_t j = 0; j < stikp_size; j++) {
            std::cerr << "[" << comp_name << "]   [" << stikp[j].ee_name << "] eefm_rot_damping_gain = "
                      << stikp[j].eefm_rot_damping_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                      << ", eefm_rot_time_const = "
                      << stikp[j].eefm_rot_time_const.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                      << "[s]" << std::endl;
            std::cerr << "[" << comp_name << "]   [" << stikp[j].ee_name << "] eefm_pos_damping_gain = "
                      << stikp[j].eefm_pos_damping_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                      << ", eefm_pos_time_const_support = "
                      << stikp[j].eefm_pos_time_const_support.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                      << "[s]" << std::endl;
            std::cerr << "[" << comp_name << "]   [" << stikp[j].ee_name << "] "
                      << "eefm_pos_compensation_limit = " << stikp[j].eefm_pos_compensation_limit << "[m], "
                      << "eefm_rot_compensation_limit = " << stikp[j].eefm_rot_compensation_limit << "[rad], "
                      << "eefm_ee_moment_limit = " << stikp[j].eefm_ee_moment_limit.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[Nm]" << std::endl;
            std::cerr << "[" << comp_name << "]   [" << stikp[j].ee_name << "] "
                      << "eefm_swing_pos_spring_gain = " << stikp[j].eefm_swing_pos_spring_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << ", "
                      << "eefm_swing_pos_time_const = " << stikp[j].eefm_swing_pos_time_const.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << ", "
                      << "eefm_swing_rot_spring_gain = " << stikp[j].eefm_swing_rot_spring_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << ", "
                      << "eefm_swing_pos_time_const = " << stikp[j].eefm_swing_pos_time_const.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << ", "
                      << std::endl;
            std::cerr << "[" << comp_name << "]   [" << stikp[j].ee_name << "] "
                      << "eefm_ee_forcemoment_distribution_weight = " << stikp[j].eefm_ee_forcemoment_distribution_weight.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "" << std::endl;
        }
    } else {
        std::cerr << "[" << comp_name << "]   eefm damping parameters cannot be set because of invalid param." << std::endl;
    }

    std::cerr << "[" << comp_name << "]   eefm_pos_transition_time = " << eefm_pos_transition_time << "[s], eefm_pos_margin_time = " << eefm_pos_margin_time << "[s] eefm_pos_time_const_swing = " << eefm_pos_time_const_swing << "[s]" << std::endl;
    std::cerr << "[" << comp_name << "]   cogvel_cutoff_freq = " << act_cogvel_filter->getCutOffFreq() << "[Hz]" << std::endl;
    szd->print_params(std::string(comp_name));
    std::cerr << "[" << comp_name << "]   g_acc = " << g_acc << "[m/s^2], eefm_use_force_difference_control = " << (eefm_use_force_difference_control? "true":"false") << ", eefm_use_swing_damping = " << (eefm_use_swing_damping? "true":"false") << std::endl;
    std::cerr << "[" << comp_name << "]   eefm_ee_error_cutoff_freq = " << stikp[0].target_ee_diff_p_filter->getCutOffFreq() << "[Hz]" << std::endl;
    std::cerr << "[" << comp_name << "]  COMMON" << std::endl;
    if (control_mode == MODE_IDLE) {
        st_algorithm = i_stp.st_algorithm;
        std::cerr << "[" << comp_name << "]   st_algorithm changed to [" << getStabilizerAlgorithmString(st_algorithm) << "]" << std::endl;
    } else {
        std::cerr << "[" << comp_name << "]   st_algorithm cannot be changed to [" << getStabilizerAlgorithmString(st_algorithm) << "] during MODE_AIR or MODE_ST." << std::endl;
    }
    std::cerr << "[" << comp_name << "]   emergency_check_mode changed to [" << (emergency_check_mode == OpenHRP::AutoBalanceStabilizerService::NO_CHECK?"NO_CHECK": (emergency_check_mode == OpenHRP::AutoBalanceStabilizerService::COP?"COP":"CP") ) << "]" << std::endl;
    std::cerr << "[" << comp_name << "]   transition_time = " << transition_time << "[s]" << std::endl;
    std::cerr << "[" << comp_name << "]   cop_check_margin = " << cop_check_margin << "[m], "
              << "cp_check_margin = [" << cp_check_margin[0] << ", " << cp_check_margin[1] << ", " << cp_check_margin[2] << ", " << cp_check_margin[3] << "] [m], "
              << "tilt_margin = [" << tilt_margin[0] << ", " << tilt_margin[1] << "] [rad]" << std::endl;
    std::cerr << "[" << comp_name << "]   contact_decision_threshold = " << contact_decision_threshold << "[N], detection_time_to_air = " << detection_count_to_mode_air * dt << "[s]" << std::endl;
    std::cerr << "[" << comp_name << "]   root_rot_compensation_limit = [" << root_rot_compensation_limit[0] << " " << root_rot_compensation_limit[1] << "][rad]" << std::endl;
    // IK limb parameters
    std::cerr << "[" << comp_name << "]  IK limb parameters" << std::endl;

    bool is_ik_limb_parameter_valid_length = true;
    if (i_stp.ik_limb_parameters.length() != jpe_v.size()) {
        is_ik_limb_parameter_valid_length = false;
        std::cerr << "[" << comp_name << "]   ik_limb_parameters invalid length! Cannot be set. (input = " << i_stp.ik_limb_parameters.length() << ", desired = " << jpe_v.size() << ")" << std::endl;
    } else {
        for (size_t i = 0; i < jpe_v.size(); i++) {
            if (jpe_v[i]->numJoints() != i_stp.ik_limb_parameters[i].ik_optional_weight_vector.length())
                is_ik_limb_parameter_valid_length = false;
        }
        if (is_ik_limb_parameter_valid_length) {
            for (size_t i = 0; i < jpe_v.size(); i++) {
                const OpenHRP::AutoBalanceStabilizerService::IKLimbParameters& ilp = i_stp.ik_limb_parameters[i];
                std::vector<double> ov;
                ov.resize(jpe_v[i]->numJoints());
                for (size_t j = 0; j < jpe_v[i]->numJoints(); j++) {
                    ov[j] = ilp.ik_optional_weight_vector[j];
                }
                jpe_v[i]->setOptionalWeightVector(ov);
                jpe_v[i]->setSRGain(ilp.sr_gain);
                stikp[i].avoid_gain = ilp.avoid_gain;
                stikp[i].reference_gain = ilp.reference_gain;
                jpe_v[i]->setManipulabilityLimit(ilp.manipulability_limit);
                stikp[i].ik_loop_count = ilp.ik_loop_count; // unsigned short -> size_t, value not change
            }
        } else {
            std::cerr << "[" << comp_name << "]   ik_optional_weight_vector invalid length! Cannot be set. (input = [";
            for (size_t i = 0; i < jpe_v.size(); i++) {
                std::cerr << i_stp.ik_limb_parameters[i].ik_optional_weight_vector.length() << ", ";
            }
            std::cerr << "], desired = [";
            for (size_t i = 0; i < jpe_v.size(); i++) {
                std::cerr << jpe_v[i]->numJoints() << ", ";
            }
            std::cerr << "])" << std::endl;
        }
    }

    if (is_ik_limb_parameter_valid_length) {
        std::cerr << "[" << comp_name << "]   ik_optional_weight_vectors = ";
        for (size_t i = 0; i < jpe_v.size(); i++) {
            std::vector<double> ov;
            ov.resize(jpe_v[i]->numJoints());
            jpe_v[i]->getOptionalWeightVector(ov);
            std::cerr << "[";
            for (size_t j = 0; j < jpe_v[i]->numJoints(); j++) {
                std::cerr << ov[j] << " ";
            }
            std::cerr << "]";
        }
        std::cerr << std::endl;
        std::cerr << "[" << comp_name << "]   sr_gains = [";
        for (size_t i = 0; i < jpe_v.size(); i++) {
            std::cerr << jpe_v[i]->getSRGain() << ", ";
        }
        std::cerr << "]" << std::endl;
        std::cerr << "[" << comp_name << "]   avoid_gains = [";
        for (size_t i = 0; i < stikp_size; i++) {
            std::cerr << stikp[i].avoid_gain << ", ";
        }
        std::cerr << "]" << std::endl;
        std::cerr << "[" << comp_name << "]   reference_gains = [";
        for (size_t i = 0; i < stikp_size; i++) {
            std::cerr << stikp[i].reference_gain << ", ";
        }
        std::cerr << "]" << std::endl;
        std::cerr << "[" << comp_name << "]   manipulability_limits = [";
        for (size_t i = 0; i < jpe_v.size(); i++) {
            std::cerr << jpe_v[i]->getManipulabilityLimit() << ", ";
        }
        std::cerr << "]" << std::endl;
        std::cerr << "[" << comp_name << "]   ik_loop_count = [";
        for (size_t i = 0; i < stikp_size; i++) {
            std::cerr << stikp[i].ik_loop_count << ", ";
        }
        std::cerr << "]" << std::endl;
    }

    // TODO: control_modeでまとめる。整理
    // joint servo control parameters
    std::cerr << "[" << comp_name << "]  joint servo control parameters" << std::endl;
    if (control_mode == MODE_IDLE) {
        // !TORQUE -> TORQUE
        if (joint_control_mode != OpenHRP::AutoBalanceStabilizerService::JOINT_TORQUE && i_stp.joint_control_mode == OpenHRP::AutoBalanceStabilizerService::JOINT_TORQUE) {
            for (STIKParam& ikp : stikp) {
                ikp.eefm_pos_damping_gain *= 1000;
                ikp.eefm_rot_damping_gain *= 1000;
            }
            eefm_swing_pos_damping_gain *= 1000;
            eefm_swing_rot_damping_gain *= 1000;
        } else if (joint_control_mode == OpenHRP::AutoBalanceStabilizerService::JOINT_TORQUE && i_stp.joint_control_mode != OpenHRP::AutoBalanceStabilizerService::JOINT_TORQUE) {
            // TORQUE -> !TORQUE
            for (STIKParam& ikp : stikp) {
                ikp.eefm_pos_damping_gain /= 1000;
                ikp.eefm_rot_damping_gain /= 1000;
            }
            eefm_swing_pos_damping_gain /= 1000;
            eefm_swing_rot_damping_gain /= 1000;
        }

        joint_control_mode = i_stp.joint_control_mode;

        std::cerr << "[" << comp_name << "]   joint_control_mode changed to mode " << joint_control_mode << std::endl;
    } else {
        std::cerr << "[" << comp_name << "]   joint_control_mode cannot be changed during MODE_AIR or MODE_ST." << std::endl;
    }

    swing2landing_transition_time = i_stp.swing2landing_transition_time;
    landing_phase_time = i_stp.landing_phase_time;
    landing2support_transition_time = i_stp.landing2support_transition_time;

    bool is_joint_servo_control_parameter_valid_length = true;
    if (i_stp.joint_servo_control_parameters.length() == stikp_size) {
        // TODO: true / false判定を先にしておくべきか
        for (size_t i = 0; i < stikp_size; i++) {
            const OpenHRP::AutoBalanceStabilizerService::JointServoControlParameter& jscp = i_stp.joint_servo_control_parameters[i];
            if (stikp[i].support_pgain.size() == jscp.support_pgain.length() &&
                stikp[i].support_dgain.size() == jscp.support_dgain.length() &&
                stikp[i].landing_pgain.size() == jscp.landing_pgain.length() &&
                stikp[i].landing_dgain.size() == jscp.landing_dgain.length()) {
                for (size_t j = 0; j < stikp[i].support_pgain.size(); j++) {
                    stikp[i].support_pgain(j) = jscp.support_pgain[j];
                    stikp[i].support_dgain(j) = jscp.support_dgain[j];
                    stikp[i].landing_pgain(j) = jscp.landing_pgain[j];
                    stikp[i].landing_dgain(j) = jscp.landing_dgain[j];
                }
            } else {
                std::cerr << stikp[i].support_pgain.size() << ", " << jscp.support_pgain.length() << std::endl;
                std::cerr << stikp[i].support_dgain.size() << ", " << jscp.support_dgain.length() << std::endl;
                std::cerr << stikp[i].landing_pgain.size() << ", " << jscp.landing_pgain.length() << std::endl;
                std::cerr << stikp[i].landing_dgain.size() << ", " << jscp.landing_dgain.length() << std::endl;
                is_joint_servo_control_parameter_valid_length = false;
                break;
            }
        }
    } else {
        std::cerr << "i_stp.joint_servo_control_parameters.length() != stikp.size()" << std::endl;
        is_joint_servo_control_parameter_valid_length = false;
    }

    if (is_joint_servo_control_parameter_valid_length) {
        std::cerr << "[" << comp_name << "]   support_pgain = [";
        for (size_t i = 0; i < stikp_size; i++) std::cerr << "[" << stikp[i].support_pgain.transpose() << "],";
        std::cerr << "]" << std::endl;
        std::cerr << "[" << comp_name << "]   support_dgain = [";
        for (size_t i = 0; i < stikp_size; i++) std::cerr << "[" << stikp[i].support_dgain.transpose() << "],";
        std::cerr << "]" << std::endl;
        std::cerr << "[" << comp_name << "]   landing_pgain = [";
        for (size_t i = 0; i < stikp_size; i++) std::cerr << "[" << stikp[i].landing_pgain.transpose() << "],";
        std::cerr << "]" << std::endl;
        std::cerr << "[" << comp_name << "]   landing_dgain = [";
        for (size_t i = 0; i < stikp_size; i++) std::cerr << "[" << stikp[i].landing_dgain.transpose() << "],";
        std::cerr << "]" << std::endl;
    } else {
        std::cerr << "[" << comp_name << "]   Servo gain parameters cannot be set because of invalid param." << std::endl;
    }
}

}
