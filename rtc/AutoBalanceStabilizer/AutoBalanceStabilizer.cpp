// -*- C++ -*-
/*!
 * @file  AutoBalanceStabilizer.cpp
 * @brief autobalancestabilizer component
 * @date  $Date$
 *
 * $Id$
 */

#include <boost/make_shared.hpp>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Sensor.h>
#include "interpolator.h"
#include "../TorqueFilter/IIRFilter.h"
#include "EigenUtil.h"
#include "AutoBalanceStabilizer.h"

using Guard = std::lock_guard<std::mutex>;
using paramsToStabilizer = hrp::paramsFromAutoBalancer;
// Utility functions
using hrp::deg2rad;
using hrp::rad2deg;
using hrp::calcInteriorPoint;
using hrp::copyJointAnglesToRobotModel;
using hrp::copyJointAnglesFromRobotModel;

namespace
{

// Module specification
// <rtc-template block="module_spec">
const char* autobalancestabilizer_spec[] =
{
    "implementation_id", "AutoBalanceStabilizer",
    "type_name",         "AutoBalanceStabilizer",
    "description",       "autobalancestabilizer component",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debugLevel", "0",
    ""
};
// </rtc-template>

std::ostream& operator<<(std::ostream& os, const struct RTC::Time &tm)
{
    int pre = os.precision();
    os.setf(std::ios::fixed);
    os << std::setprecision(6)
       << (tm.sec + tm.nsec/1e9)
       << std::setprecision(pre);
    os.unsetf(std::ios::fixed);
    return os;
}

}

AutoBalanceStabilizer::AutoBalanceStabilizer(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qActIn("qAct", m_qAct),
      m_qRefIn("qRef", m_qRef),
      m_actImuRpyIn("actImuRpy", m_actImuRpy), // from KF
      m_comBasePosIn("comBasePos", m_comBasePos),
      m_comBaseRpyIn("comBaseRpy", m_comBaseRpy),
      m_comZmpIn("comZmp", m_comZmp),
      m_comOptionalDataIn("comOptionalData", m_comOptionalData),

      m_qOut("q", m_qRef),
      m_genBasePosOut("genBasePos", m_genBasePos),
      m_genBaseRpyOut("genBaseRpy", m_genBaseRpy),
      m_genBasePoseOut("genBasePose", m_genBasePose),
      m_ikImuAccOut("ikImuAcc", m_ikImuAcc),
      m_emergencySignalOut("emergencySignal", m_emergencySignal),

      // Out port for debugging
      m_genBaseTformOut("genBaseTform", m_genBaseTform),
      m_genZmpOut("genZmp", m_genZmp), // global
      m_nominalZmpOut("nominalZmp", m_nominalZmp), // global
      m_genEndCpOut("genEndCp", m_genEndCp), // global
      m_genCpOut("genCp", m_genCp), // global
      m_genRemainTimeOut("genRemainTime", m_genRemainTime),
      m_genCmpOut("genCmp", m_genCmp),
      m_baseFrameGenZmpOut("baseFrameGenZmp", m_baseFrameGenZmp),
      m_genCogOut("genCog", m_genCog),
      m_genCogVelOut("genCogVel", m_genCogVel),
      m_genCogAccOut("genCogAcc", m_genCogAcc),
      m_genAngularMomentumRpyOut("genAngularMomentumRpy", m_genAngularMomentumRpy),
      m_genSbpCogOffsetOut("genSbpCogOffset", m_genSbpCogOffset),
      m_genContactStatesOut("genContactStates", m_genContactStates),

      // Data from Stabilizer
      m_modTauOut("modTau", m_modTau),
      m_actBaseRpyOut("actBaseRpy", m_actBaseRpy),
      m_baseFrameActZmpOut("baseFrameActZmp", m_baseFrameActZmp),
      m_footFrameGenZmpOut("footFrameGenZmp", m_footFrameGenZmp),
      m_footFrameModZmpOut("footFrameModZmp", m_footFrameModZmp),
      m_footFrameActZmpOut("footFrameActZmp", m_footFrameActZmp),
      m_footFrameGenCogOut("footFrameGenCog", m_footFrameGenCog),
      m_footFrameActCogOut("footFrameActCog", m_footFrameActCog),
      m_actContactStatesOut("actContactStates", m_actContactStates),
      m_modWrenchesOut("modWrenches", m_modWrenches),
      m_baseFrameGenCpOut("baseFrameGenCp", m_baseFrameGenCp),
      m_baseFrameActCpOut("baseFrameActCp", m_baseFrameActCp),
      m_eeOriginCopInfoOut("eeOriginCopInfo", m_eeOriginCopInfo),
      m_modPgainOut("modPgain", m_modPgain),
      m_modDgainOut("modDgain", m_modDgain),
      m_modTqPgainOut("modTqPgain", m_modTqPgain),
      // m_modTqDgainOut("modTqDgain", m_modTqDgain),
      m_modGainTransitionTimeOut("modGainTransitionTime", m_modGainTransitionTime),

      m_AutoBalanceStabilizerServicePort("AutoBalanceStabilizerService")
      // </rtc-template>
{
    m_service0.autobalancestabilizer(this);
}

// AutoBalanceStabilizer::~AutoBalanceStabilizer()
// {
// }


RTC::ReturnCode_t AutoBalanceStabilizer::onInitialize()
{
    std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
    bindParameter("debugLevel", m_debugLevel, "0");
    setupBasicPort();

    const RTC::Properties& prop = getProperties();
    coil::stringTo(m_dt, prop["dt"].c_str());

    if (!loadModel(m_robot, prop["model"])) {
        std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
        return RTC::RTC_ERROR;
    }

    fik = std::make_unique<hrp::FullbodyInverseKinematicsSolver>(m_robot, std::string(m_profile.instance_name), m_dt);
    st = std::make_unique<hrp::Stabilizer>(m_robot, std::string(m_profile.instance_name) + "_ST", m_dt, m_mutex);
    std::vector<int> contacts_link_indices;
    {
        std::vector<hrp::LinkConstraint> init_constraints = readContactPointsFromProps(prop, contacts_link_indices);

        // addBodyConstraint(init_constraints, m_robot);

        constexpr double PREVIEW_TIME = 1.6;
        if (init_constraints.size() > 0) {
            setupIKConstraints(m_robot, init_constraints);

            m_robot->calcForwardKinematics();
            // TODO: FixLegToCoords ?
            gg = std::make_unique<hrp::GaitGenerator>(m_robot, m_mutex, m_dt, std::move(init_constraints), PREVIEW_TIME);
        } else {
            std::cerr << "[" << m_profile.instance_name << "] failed to read contact points. GaitGenerator cannot be created." << std::endl;
        }
    }

    // setting from conf file
    // rleg,TARGET_LINK,BASE_LINK
    const coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
    const unsigned int num_pfsensors = m_robot->numSensors(hrp::Sensor::FORCE);
    constexpr size_t ee_prop_num = 10;

    if (end_effectors_str.size() > 0) {
        const size_t ee_num = end_effectors_str.size() / ee_prop_num;

        for (size_t i = 0; i < ee_num; i++) {
            const std::string ee_name(end_effectors_str[i*ee_prop_num]);
            const std::string ee_target(end_effectors_str[i*ee_prop_num + 1]);
            const std::string ee_base(end_effectors_str[i*ee_prop_num + 2]);

            ee_vec.push_back(ee_name);

            // For Stabilizer IK TODO: IKを統合して消す
            {
                bool is_ee_exists = false;
                std::string ikp_sensor_name = "";
                for (size_t j = 0; j < num_pfsensors; ++j) {
                    hrp::Sensor* fsensor = m_robot->sensor(hrp::Sensor::FORCE, j);
                    hrp::Link* alink = m_robot->link(ee_target);
                    while (alink != NULL && alink->name != ee_base && !is_ee_exists) {
                        if (alink->name == fsensor->link->name) {
                            is_ee_exists = true;
                            ikp_sensor_name = fsensor->name;
                        }
                        alink = alink->parent;
                    }
                }

                // TODO: tmp, delete
                hrp::Vector3 local_pos;
                for (size_t j = 0; j < 3; j++) {
                    coil::stringTo(local_pos(j), end_effectors_str[i*ee_prop_num+3+j].c_str());
                }

                double axis_angle[4];
                for (int j = 0; j < 4; j++ ) {
                    coil::stringTo(axis_angle[j], end_effectors_str[i*ee_prop_num+6+j].c_str());
                }
                const hrp::Matrix33 local_rot = Eigen::AngleAxis<double>(axis_angle[3], hrp::Vector3(axis_angle[0], axis_angle[1], axis_angle[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle

                st->addSTIKParam(ee_name, ee_target, ee_base, ikp_sensor_name, local_pos, local_rot);
            }
        }

        m_actContactStates.data.length(ee_num);
        m_modWrenches.data.length(ee_num * 6);
        m_eeOriginCopInfo.data.length(ee_num * 3);
        for (size_t i = 0; i < ee_num; i++) {
            m_actContactStates.data[i] = false;

            for (size_t j = 0; j < 6; ++j) {
                m_modWrenches.data[6*i+j] = 0.0;
            }

            for (size_t j = 0; j < 3; ++j) {
                m_eeOriginCopInfo.data[i * 3 + j] = 0.0;
            }
        }

        st->initStabilizer(prop, ee_num);
    }

    // {
    //     std::vector<std::pair<hrp::Link*, hrp::Link*> > interlocking_joints;
    //     readInterlockingJointsParamFromProperties(interlocking_joints, m_robot, prop["interlocking_joints"], std::string(m_profile.instance_name));
    //     if (interlocking_joints.size() > 0) {
    //         fik->initializeInterlockingJoints(interlocking_joints);
    //     }
    // }

    transition_interpolator = std::make_unique<interpolator>(1, m_dt, interpolator::HOFFARBIB, 1);
    transition_interpolator->setName(std::string(m_profile.instance_name) + " transition_interpolator");

    // load virtual force sensors
    readVirtualForceSensorParamFromProperties(m_vfs, m_robot, prop["virtual_force_sensor"], std::string(m_profile.instance_name));
    // ref force port
    const unsigned int num_vfsensors = m_vfs.size();
    const unsigned int num_fsensors  = num_pfsensors + num_vfsensors;
    // check number of force sensors
    if (num_fsensors < (end_effectors_str.size() / ee_prop_num)) {
        std::cerr << "[" << m_profile.instance_name
                  << "] WARNING! This robot model has less force sensors("
                  << num_fsensors
                  << ") than end-effector settings("
                  << ref_contact_states.size()
                  << ") !"
                  << std::endl;
    }

    m_ref_force.resize(num_fsensors);
    m_ref_forceIn.resize(num_fsensors);
    m_wrenches.resize(num_fsensors);
    m_wrenchesIn.resize(num_fsensors);

    for (unsigned int i = 0; i < num_pfsensors; i++) {
        sensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
    }
    for (unsigned int i = 0; i < num_vfsensors; i++) {
        for (std::map<std::string, hrp::VirtualForceSensorParam>::iterator it = m_vfs.begin(); it != m_vfs.end(); ++it) {
            if (it->second.id == static_cast<int>(i)) sensor_names.push_back(it->first);
        }
    }

    // set force port
    std::cerr << "[" << m_profile.instance_name << "] force sensor ports (" << num_fsensors << ")" << std::endl;
    for (unsigned int i = 0; i < num_fsensors; i++) {
        const std::string port_name("ref_" + sensor_names[i]);
        m_ref_forceIn[i] = new InPort<TimedDoubleSeq>(port_name.c_str(), m_ref_force[i]);
        m_ref_force[i].data.length(6);
        registerInPort(port_name.c_str(), *m_ref_forceIn[i]);
        std::cerr << "[" << m_profile.instance_name << "]   name = " << port_name << std::endl;
        ref_forces.push_back(hrp::Vector3::Zero());
        ref_moments.push_back(hrp::Vector3::Zero());

        m_wrenchesIn[i] = new InPort<TimedDoubleSeq>(sensor_names[i].c_str(), m_wrenches[i]);
        m_wrenches[i].data.length(6);
        registerInPort(sensor_names[i].c_str(), *m_wrenchesIn[i]);
    }

    m_ikImuAcc.data.ax = m_ikImuAcc.data.ay = m_ikImuAcc.data.az = 0.0;

    {
        const hrp::Sensor* const gyro = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
        if (!gyro) {
            std::cerr << "[" << m_profile.instance_name
                      << "] WARNING! This robot model has no GyroSensor named 'gyrometer'! "
                      << std::endl;
        }
    }

    // allocate memory
    {
        const size_t num_joints = m_robot->numJoints();
        m_qAct.data.length(num_joints);
        m_qRef.data.length(num_joints);
        m_modTau.data.length(num_joints);
        m_genBaseTform.data.length(12);
        m_genRemainTime.data.length(2);

        q_prev_ik = hrp::dvector::Zero(num_joints);
        q_act = hrp::dvector::Zero(num_joints);
        q_ref = hrp::dvector::Zero(num_joints);
        ref_wrenches.resize(num_fsensors, hrp::dvector6::Zero());
        ref_wrenches_for_st.resize(num_fsensors, hrp::dvector6::Zero());
        act_wrenches.resize(num_fsensors, hrp::dvector6::Zero());

        m_modPgain.data.length(num_joints);
        m_modDgain.data.length(num_joints);
        m_modTqPgain.data.length(num_joints);
        // m_modTqDgain.data.length(num_joints);
        m_modGainTransitionTime.data.length(num_joints);

        // Debug
        m_genContactStates.data.length(2);
    }

    storeRobotStatesForIK();

    additional_force_applied_link = m_robot->rootLink();
    additional_force_applied_point_offset = hrp::Vector3::Zero();

    m_act_robot = boost::make_shared<hrp::Body>(*m_robot);
    act_se = std::make_unique<hrp::StateEstimator>(m_act_robot, std::string(m_profile.instance_name) + "_SE", m_dt, m_mutex, contacts_link_indices);

    return RTC::RTC_OK;
}


RTC::ReturnCode_t AutoBalanceStabilizer::onFinalize()
{
    return RTC::RTC_OK;
}

RTC::ReturnCode_t AutoBalanceStabilizer::onActivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
    return RTC::RTC_OK;
}

RTC::ReturnCode_t AutoBalanceStabilizer::onDeactivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
    Guard guard(m_mutex);
    if (control_mode == MODE_ABC) {
        control_mode = MODE_SYNC_TO_IDLE;
        constexpr double tmp_ratio = 0.0;
        transition_interpolator->setGoal(&tmp_ratio, m_dt, true); // sync in one controller loop
    }
    return RTC::RTC_OK;
}

RTC::ReturnCode_t AutoBalanceStabilizer::onExecute(RTC::UniqueId ec_id)
{
    // std::cerr << "AutoBalanceStabilizer::onExecute(" << ec_id << ")" << std::endl;
    Guard guard(m_mutex); // TODO: スコープ確認

    ++loop;
    gg->setCurrentLoop(loop);
    readInportData();
    updateBodyParams();
    if(!gg_is_walking) gg->setConstraintToFootCoord(m_robot);
    act_se->calcStates(hrp::stateInputData{q_act, act_rpy, act_wrenches, gg->getCurrentConstraints(loop), ref_zmp(2)});

    gg->setDebugLevel(m_debugLevel);

    if (control_mode != MODE_IDLE) {
        // adjustCOPCoordToTarget();

        // 脚軌道, COP, RootLink計算
        gg->forwardTimeStep(loop);
        gg->calcCogAndLimbTrajectory(loop, m_dt);
        gg_is_walking = gg->getWalkingState();
        ref_zmp = gg->getRefZMP();

        // TODO: rootlink計算
        // TODO: IKを後ろに回す
        setIKConstraintsTarget();

        // // reduce upper body weight
        // fik->q_ref_constraint_weight.fill(2e-6);
        // fik->q_ref_constraint_weight.segment(fik->ikp["rleg"].group_indices.back(), fik->ikp["rleg"].group_indices.size()).fill(0);
        // fik->q_ref_constraint_weight.segment(fik->ikp["lleg"].group_indices.back(), fik->ikp["lleg"].group_indices.size()).fill(0);

        // reduce chest joint movements
        if(m_robot->link("CHEST_JOINT0") != NULL) fik->dq_weight_all(m_robot->link("CHEST_JOINT0")->jointId) = 1000;
        if(m_robot->link("CHEST_JOINT1") != NULL) fik->dq_weight_all(m_robot->link("CHEST_JOINT1")->jointId) = 1000;
        if(m_robot->link("CHEST_JOINT2") != NULL) fik->dq_weight_all(m_robot->link("CHEST_JOINT2")->jointId) = 1000;
        // reduce head joint movements
        if(m_robot->link("HEAD_JOINT0") != NULL) fik->dq_weight_all(m_robot->link("HEAD_JOINT0")->jointId) = 1000;
        if(m_robot->link("HEAD_JOINT1") != NULL) fik->dq_weight_all(m_robot->link("HEAD_JOINT1")->jointId) = 1000;
        fik->solveFullbodyIKLoop(ik_constraints, max_ik_iteration);
        // std::cerr << "moment: " << fik->getComMomentum().transpose() << std::endl;
    }

    calcOutputRefForcesFromRefZmp();
    storeRobotStatesForIK();

    // - ST

    // - IK

    // Get transition ratio
    const bool is_transition_interpolator_empty = transition_interpolator->isEmpty();
    if (!is_transition_interpolator_empty) {
        transition_interpolator->get(&transition_interpolator_ratio, true);
    } else {
        transition_interpolator_ratio = (control_mode == MODE_IDLE) ? 0.0 : 1.0;
    }

    hrp::Vector3 ref_zmp_base_frame;
    if (control_mode == MODE_IDLE) {
        ref_zmp_base_frame = input_ref_zmp;
        // fik->d_root_height = 0.0;
    } else {
        // TODO: 座標変換をライブラリ使うか関数実装する transform(vec, cur_coord, new_coord)
        ref_zmp_base_frame = m_robot->rootLink()->R.transpose() * (ref_zmp - m_robot->rootLink()->p);
    }

    hrp::Vector3  ref_basePos = m_robot->rootLink()->p;
    hrp::Matrix33 ref_baseRot = m_robot->rootLink()->R;

    // Transition
    if (!is_transition_interpolator_empty) {
        // transition_interpolator_ratio 0=>1 : IDLE => ABC
        // transition_interpolator_ratio 1=>0 : ABC => IDLE
        ref_basePos = calcInteriorPoint(ref_base_pos, m_robot->rootLink()->p, transition_interpolator_ratio);
        ref_zmp_base_frame = calcInteriorPoint(input_ref_zmp, ref_zmp_base_frame, transition_interpolator_ratio);
        ref_baseRot = hrp::slerpMat(ref_baseRot, m_robot->rootLink()->R, transition_interpolator_ratio);

        for (size_t i = 0, num_joints = m_robot->numJoints(); i < num_joints; ++i) {
            m_robot->joint(i)->q = calcInteriorPoint(q_ref[i], m_robot->joint(i)->q, transition_interpolator_ratio);
        }

        for (size_t i = 0, wrenches_size = ref_wrenches.size(); i < wrenches_size; ++i) {
            ref_wrenches_for_st[i] = calcInteriorPoint(ref_wrenches_for_st[i], ref_wrenches[i], transition_interpolator_ratio);
        }
    }

    // mode change for sync
    if (control_mode == MODE_SYNC_TO_ABC) {
        control_mode = MODE_ABC;
    } else if (control_mode == MODE_SYNC_TO_IDLE && transition_interpolator->isEmpty()) {
        std::cerr << "[" << m_profile.instance_name << "] [" << m_qRef.tm
                  << "] Finished cleanup" << std::endl;
        control_mode = MODE_IDLE;
    }

    // Set reference acceleration for kalman filter before executing stabilizer
    hrp::Vector3 kf_acc_ref = hrp::Vector3::Zero();
    {
        const hrp::Sensor* const sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
        if (sen) {
            const hrp::Vector3 imu_sensor_pos = sen->link->p + sen->link->R * sen->localPos;
            const hrp::Vector3 imu_sensor_vel = (imu_sensor_pos - prev_imu_sensor_pos) / m_dt;
            // convert to imu sensor local acceleration
            kf_acc_ref = (sen->link->R * sen->localR).transpose() * (imu_sensor_vel - prev_imu_sensor_vel) / m_dt;
            prev_imu_sensor_pos = imu_sensor_pos;
            prev_imu_sensor_vel = imu_sensor_vel;
        }
    }

    // Stabilizer
    hrp::dvector abc_qref(m_robot->numJoints()); // TODO: IKなしで大丈夫なのか，必要なのか
    copyJointAnglesFromRobotModel(abc_qref, m_robot);

    // TODO: StabilizerのLinkConstraints対応をしたら消す
    const auto& cs = gg->getCurrentConstraints(loop);
    const size_t cs_size = cs.constraints.size();
    ref_contact_states.resize(cs_size);
    control_swing_support_time.resize(cs_size);
    for (size_t i = 0; i < cs_size; ++i) {
        ref_contact_states[i] = (cs.constraints[i].getConstraintType() == hrp::LinkConstraint::FIX);

        if (cs.constraints[i].getConstraintType() != hrp::LinkConstraint::FLOAT) {
            control_swing_support_time[i] = 1.0;
        } else {
            const std::vector<hrp::ConstraintsWithCount>& constraints = gg->getConstraintsList();
            size_t index = hrp::getConstraintIndexFromCount(constraints, loop);
            for (; index < constraints.size(); ++index) {
                if (constraints[index].constraints[i].getConstraintType() == hrp::LinkConstraint::FIX) break;
            }
            if (index == constraints.size()) control_swing_support_time[i] = 1.0;
            else control_swing_support_time[i] = (constraints[index].start_count - cs.start_count) * m_dt;
        }
    }

    // TODO: Rotをわざわざrpyにして渡すのはNG
    // TODO: STでtarget_root_pとikpのtarget_p0, target_r0を更新
    //       ikpを共通化して、STに渡す (ぐずぐずになるのはいまいちな気もするが)
    st->execStabilizer(paramsToStabilizer{abc_qref, ref_zmp_base_frame, ref_basePos,
                                          hrp::rpyFromRot(ref_baseRot), gg_is_walking, ref_contact_states, // toe_heel_ratio,
                                          control_swing_support_time, ref_wrenches_for_st, // limb_cop_offsets,
                                          sbp_cog_offset},
                       hrp::paramsFromSensors{q_act, act_rpy, act_wrenches});

    writeOutPortData(ref_basePos, ref_baseRot, ref_zmp, gg->getNominalZMP(), gg->getRefEndCP(), gg->getNewRefCP(),
                     gg->getStepRemainTime(), gg->getConstRemainTime(), ref_zmp_base_frame,
                     gg->getCog(), gg->getCogVel(), gg->getCogAcc(),
                     ref_angular_momentum, sbp_cog_offset,
                     kf_acc_ref, st->getStabilizerPortData());

    m_act_robot->rootLink()->R = ref_baseRot;

    return RTC::RTC_OK;
}

bool AutoBalanceStabilizer::loadModel(hrp::BodyPtr body, const string& model_path)
{
    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0) {
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
    return loadBodyFromModelLoader(body, model_path.c_str(),
                                   CosNaming::NamingContext::_duplicate(naming.getRootContext()));
}

void AutoBalanceStabilizer::setupBasicPort()
{
    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qAct", m_qActIn);
    addInPort("qRef", m_qRefIn);
    addInPort("actImuRpy", m_actImuRpyIn);
    addInPort("comBasePos", m_comBasePosIn);
    addInPort("comBaseRpy", m_comBaseRpyIn);
    addInPort("comZmp", m_comZmpIn);
    addInPort("comOptionalData", m_comOptionalDataIn);

    // Set OutPort buffer
    addOutPort("q", m_qOut);
    addOutPort("genBasePos", m_genBasePosOut);
    addOutPort("genBaseRpy", m_genBaseRpyOut);
    addOutPort("genBasePose", m_genBasePoseOut);
    addOutPort("ikImuAcc", m_ikImuAccOut);
    addOutPort("emergencySignal", m_emergencySignalOut);

    // Out port for debugging
    addOutPort("genZmp", m_genZmpOut);
    addOutPort("nominalZmp", m_nominalZmpOut);
    addOutPort("genEndCp", m_genEndCpOut);
    addOutPort("genCp", m_genCpOut);
    addOutPort("genRemainTime", m_genRemainTimeOut);
    addOutPort("genCmp", m_genCmpOut);
    addOutPort("baseFrameGenZmp", m_baseFrameGenZmpOut);
    addOutPort("genBaseTform", m_genBaseTformOut);
    addOutPort("genCog", m_genCogOut);
    addOutPort("genCogVel", m_genCogVelOut);
    addOutPort("genCogAcc", m_genCogAccOut);
    addOutPort("genAngularMomentumRpy", m_genAngularMomentumRpyOut);
    addOutPort("genSbpCogOffset", m_genSbpCogOffsetOut);
    addOutPort("baseFrameGenCp", m_baseFrameGenCpOut);
    addOutPort("genContactStates", m_genContactStatesOut);
    // Data from Stabilizer
    addOutPort("modTau", m_modTauOut);
    addOutPort("actBaseRpy", m_actBaseRpyOut);
    addOutPort("baseFrameActZmp", m_baseFrameActZmpOut);
    addOutPort("footFrameGenZmp", m_footFrameGenZmpOut);
    addOutPort("footFrameModZmp", m_footFrameModZmpOut);
    addOutPort("footFrameActZmp", m_footFrameActZmpOut);
    addOutPort("footFrameGenCog", m_footFrameGenCogOut);
    addOutPort("footFrameActCog", m_footFrameActCogOut);
    addOutPort("actContactStates", m_actContactStatesOut);
    addOutPort("modWrenches", m_modWrenchesOut);
    addOutPort("baseFrameActCp", m_baseFrameActCpOut);
    addOutPort("eeOriginCopInfo", m_eeOriginCopInfoOut);
    addOutPort("modPgain", m_modPgainOut);
    addOutPort("modDgain", m_modDgainOut);
    addOutPort("modTqPgain", m_modTqPgainOut);
    // addOutPort("modTqDgain", m_modTqDgainOut);
    addOutPort("modGainTransitionTime", m_modGainTransitionTimeOut);

    // Set service provider to Ports
    m_AutoBalanceStabilizerServicePort.registerProvider("service0", "AutoBalanceStabilizerService", m_service0);

    // Set service consumers to Ports
    // Set CORBA Service Ports
    addPort(m_AutoBalanceStabilizerServicePort);

    // </rtc-template>
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable

    // </rtc-template>
}

void AutoBalanceStabilizer::readInportData()
{
    if (m_qActIn.isNew()) {
        m_qActIn.read();
        for (size_t i = 0; i < m_robot->numJoints(); ++i) {
            q_act[i] = m_qAct.data[i];
        }
    }

    if (m_qRefIn.isNew()) {
        m_qRefIn.read();
        for (size_t i = 0; i < m_robot->numJoints(); ++i) {
            q_ref[i] = m_qRef.data[i];
        }
    }

    if (m_actImuRpyIn.isNew()) {
        m_actImuRpyIn.read();
        act_rpy[0] = m_actImuRpy.data.r;
        act_rpy[1] = m_actImuRpy.data.p;
        act_rpy[2] = m_actImuRpy.data.y;
    }

    if (m_comBasePosIn.isNew()) {
        m_comBasePosIn.read();
        ref_base_pos(0) = m_comBasePos.data.x;
        ref_base_pos(1) = m_comBasePos.data.y;
        ref_base_pos(2) = m_comBasePos.data.z;
    }

    if (m_comBaseRpyIn.isNew()) {
        m_comBaseRpyIn.read();
        ref_base_rot = hrp::rotFromRpy(m_comBaseRpy.data.r, m_comBaseRpy.data.p, m_comBaseRpy.data.y);
    }

    if (m_comZmpIn.isNew()) {
        m_comZmpIn.read();
        input_ref_zmp(0) = m_comZmp.data.x;
        input_ref_zmp(1) = m_comZmp.data.y;
        input_ref_zmp(2) = m_comZmp.data.z;
    }

    const size_t num_fsensors = m_wrenchesIn.size();
    for (size_t i = 0; i < num_fsensors; i++) {
        if (m_ref_forceIn[i]->isNew()) {
            m_ref_forceIn[i]->read();
            for (size_t j = 0; j < 6; ++j) {
                ref_wrenches[i][j] = m_ref_force[i].data[j];
            }
            hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_names[i]);
            for (size_t j = 0; j < 3; ++j) {
                sensor->f[j]   = m_ref_force[i].data[j];
                sensor->tau[j] = m_ref_force[i].data[j + 3];
            }
        }

        if (m_wrenchesIn[i]->isNew()) {
            m_wrenchesIn[i]->read();
            for (size_t j = 0; j < 6; ++j) {
                act_wrenches[i][j] = m_wrenches[i].data[j];
            }
            hrp::ForceSensor* sensor = m_act_robot->sensor<hrp::ForceSensor>(sensor_names[i]);
            for (size_t j = 0; j < 3; ++j) {
                sensor->f[j]   = m_wrenches[i].data[j];
                sensor->tau[j] = m_wrenches[i].data[j + 3];
            }
        }
    }

    if (m_comOptionalDataIn.isNew()) m_comOptionalDataIn.read(); // TODO
}

void AutoBalanceStabilizer::writeOutPortData(const hrp::Vector3& base_pos,
                                             const hrp::Matrix33& base_rot,
                                             const hrp::Vector3& ref_zmp_global,
                                             const hrp::Vector3& nominal_zmp_global,
                                             const hrp::Vector3& ref_end_cp_global,
                                             const hrp::Vector3& new_ref_cp_global,
                                             const double& step_remain_time,
                                             const double& const_remain_time,
                                             const hrp::Vector3& ref_zmp_base_frame,
                                             // const hrp::Vector3& ref_cmp,
                                             const hrp::Vector3& ref_cog,
                                             const hrp::Vector3& ref_cog_vel,
                                             const hrp::Vector3& ref_cog_acc,
                                             const hrp::Vector3& ref_momentum,
                                             const hrp::Vector3& sbp_cog_offset,
                                             const hrp::Vector3& acc_ref,
                                             const hrp::stabilizerPortData& st_port_data)
{
    const unsigned int qRef_length = m_qRef.data.length();
    for (unsigned int i = 0; i < qRef_length; i++) {
        m_qRef.data[i] = st_port_data.joint_angles(i);
        m_modTau.data[i]  = st_port_data.joint_torques(i);
    }
    m_modTau.tm = m_qRef.tm;
    if (qRef_length != 0) {
        m_qOut.write();
        m_modTauOut.write();
    }

    m_genBasePos.tm     = m_qRef.tm;
    m_genBasePos.data.x = base_pos(0);
    m_genBasePos.data.y = base_pos(1);
    m_genBasePos.data.z = base_pos(2);
    m_genBasePosOut.write();

    const hrp::Vector3 base_rpy = hrp::rpyFromRot(base_rot);
    m_genBaseRpy.tm     = m_qRef.tm;
    m_genBaseRpy.data.r = base_rpy(0);
    m_genBaseRpy.data.p = base_rpy(1);
    m_genBaseRpy.data.y = base_rpy(2);
    m_genBaseRpyOut.write();

    double *tform_arr = m_genBaseTform.data.get_buffer();
    m_genBaseTform.tm    = m_qRef.tm;
    tform_arr[0]      = m_genBasePos.data.x;
    tform_arr[1]      = m_genBasePos.data.y;
    tform_arr[2]      = m_genBasePos.data.z;
    hrp::setMatrix33ToRowMajorArray(base_rot, tform_arr, 3);
    m_genBaseTformOut.write();

    m_genBasePose.tm                 = m_qRef.tm;
    m_genBasePose.data.position.x    = m_genBasePos.data.x;
    m_genBasePose.data.position.y    = m_genBasePos.data.y;
    m_genBasePose.data.position.z    = m_genBasePos.data.z;
    m_genBasePose.data.orientation.r = m_genBaseRpy.data.r;
    m_genBasePose.data.orientation.p = m_genBaseRpy.data.p;
    m_genBasePose.data.orientation.y = m_genBaseRpy.data.y;
    m_genBasePoseOut.write();

    m_genZmp.tm     = m_qRef.tm;
    m_genZmp.data.x = ref_zmp_global(0);
    m_genZmp.data.y = ref_zmp_global(1);
    m_genZmp.data.z = ref_zmp_global(2);
    m_genZmpOut.write();

    m_nominalZmp.tm     = m_qRef.tm;
    m_nominalZmp.data.x = nominal_zmp_global(0);
    m_nominalZmp.data.y = nominal_zmp_global(1);
    m_nominalZmp.data.z = nominal_zmp_global(2);
    m_nominalZmpOut.write();

    m_genEndCp.tm     = m_qRef.tm;
    m_genEndCp.data.x = ref_end_cp_global(0);
    m_genEndCp.data.y = ref_end_cp_global(1);
    m_genEndCp.data.z = ref_end_cp_global(2);
    m_genEndCpOut.write();

    m_genCp.tm     = m_qRef.tm;
    m_genCp.data.x = new_ref_cp_global(0);
    m_genCp.data.y = new_ref_cp_global(1);
    m_genCp.data.z = new_ref_cp_global(2);
    m_genCpOut.write();

    m_genRemainTime.tm   = m_qRef.tm;
    m_genRemainTime.data[0] = step_remain_time;
    m_genRemainTime.data[1] = const_remain_time;
    m_genRemainTimeOut.write();

    m_baseFrameGenZmp.tm     = m_qRef.tm;
    m_baseFrameGenZmp.data.x = ref_zmp_base_frame(0);
    m_baseFrameGenZmp.data.y = ref_zmp_base_frame(1);
    m_baseFrameGenZmp.data.z = ref_zmp_base_frame(2);
    m_baseFrameGenZmpOut.write();

    // m_genCmp.tm     = m_qRef.tm;
    // m_genCmp.data.x = ref_cmp(0);
    // m_genCmp.data.y = ref_cmp(1);
    // m_genCmp.data.z = ref_cmp(2);
    // m_genCmpOut.write();

    m_genCog.tm     = m_qRef.tm;
    m_genCog.data.x = ref_cog(0);
    m_genCog.data.y = ref_cog(1);
    m_genCog.data.z = ref_cog(2);
    m_genCogOut.write();

    m_genCogVel.tm     = m_qRef.tm;
    m_genCogVel.data.x = ref_cog_vel(0);
    m_genCogVel.data.y = ref_cog_vel(1);
    m_genCogVel.data.z = ref_cog_vel(2);
    m_genCogVelOut.write();

    m_genCogAcc.tm     = m_qRef.tm;
    m_genCogAcc.data.x = ref_cog_acc(0);
    m_genCogAcc.data.y = ref_cog_acc(1);
    m_genCogAcc.data.z = ref_cog_acc(2);
    m_genCogAccOut.write();

    {
        m_genAngularMomentumRpy.tm = m_qRef.tm;
        const hrp::Vector3 ref_momentum_rpy = hrp::rpyFromRot(hrp::rotationMatrixFromOmega(ref_momentum));
        m_genAngularMomentumRpy.data.x = ref_momentum_rpy(0);
        m_genAngularMomentumRpy.data.y = ref_momentum_rpy(1);
        m_genAngularMomentumRpy.data.z = ref_momentum_rpy(2);
        m_genAngularMomentumRpyOut.write();
    }

    m_genSbpCogOffset.tm = m_qRef.tm;
    m_genSbpCogOffset.data.x = sbp_cog_offset(0);
    m_genSbpCogOffset.data.y = sbp_cog_offset(1);
    m_genSbpCogOffset.data.z = sbp_cog_offset(2);
    m_genSbpCogOffsetOut.write();

    m_ikImuAcc.data.ax = acc_ref(0);
    m_ikImuAcc.data.ay = acc_ref(1);
    m_ikImuAcc.data.az = acc_ref(2);
    m_ikImuAccOut.write();

    // control parameters
    {
        m_genContactStates.tm = m_qRef.tm;
        const auto& cs = gg->getCurrentConstraints(loop);
        for (size_t i = 0; i < m_genContactStates.data.length(); ++i) {
            m_genContactStates.data[i] = cs.constraints[i].getConstraintType();
            // m_genContactStates.data[i] = (cs.constraints[i].getConstraintType() == hrp::LinkConstraint::FIX);
        }
        m_genContactStatesOut.write();
    }

    // Data from Stabilizer
    m_actBaseRpy.tm     = m_qRef.tm;
    m_actBaseRpy.data.r = st_port_data.act_base_rpy(0);
    m_actBaseRpy.data.p = st_port_data.act_base_rpy(1);
    m_actBaseRpy.data.y = st_port_data.act_base_rpy(2);
    m_actBaseRpyOut.write();

    m_baseFrameActZmp.tm = m_qRef.tm;
    m_baseFrameActZmp.data.x = st_port_data.rel_act_zmp(0);
    m_baseFrameActZmp.data.y = st_port_data.rel_act_zmp(1);
    m_baseFrameActZmp.data.z = st_port_data.rel_act_zmp(2);
    m_baseFrameActZmpOut.write();

    m_footFrameGenZmp.tm = m_qRef.tm;
    m_footFrameGenZmp.data.x = st_port_data.ref_zmp(0);
    m_footFrameGenZmp.data.y = st_port_data.ref_zmp(1);
    m_footFrameGenZmp.data.z = st_port_data.ref_zmp(2);
    m_footFrameGenZmpOut.write();

    m_footFrameModZmp.tm = m_qRef.tm;
    m_footFrameModZmp.data.x = st_port_data.new_ref_zmp(0);
    m_footFrameModZmp.data.y = st_port_data.new_ref_zmp(1);
    m_footFrameModZmp.data.z = st_port_data.new_ref_zmp(2);
    m_footFrameModZmpOut.write();

    m_footFrameActZmp.tm = m_qRef.tm;
    m_footFrameActZmp.data.x = st_port_data.act_zmp(0);
    m_footFrameActZmp.data.y = st_port_data.act_zmp(1);
    m_footFrameActZmp.data.z = st_port_data.act_zmp(2);
    m_footFrameActZmpOut.write();

    m_footFrameGenCog.tm = m_qRef.tm;
    m_footFrameGenCog.data.x = st_port_data.origin_ref_cog(0);
    m_footFrameGenCog.data.y = st_port_data.origin_ref_cog(1);
    m_footFrameGenCog.data.z = st_port_data.origin_ref_cog(2);
    m_footFrameGenCogOut.write();

    m_footFrameActCog.tm = m_qRef.tm;
    m_footFrameActCog.data.x = st_port_data.origin_act_cog(0);
    m_footFrameActCog.data.y = st_port_data.origin_act_cog(1);
    m_footFrameActCog.data.z = st_port_data.origin_act_cog(2);
    m_footFrameActCogOut.write();

    {
        m_actContactStates.tm = m_qRef.tm;
        const std::vector<bool> act_contact_states = st->getActContactStates();
        const size_t contact_size = act_contact_states.size();
        for (size_t i = 0; i < contact_size; ++i) {
            m_actContactStates.data[i] = act_contact_states[i];
        }
        m_actContactStatesOut.write();
    }

    {
        m_modWrenches.tm = m_qRef.tm;
        const size_t ref_wrenches_size = st_port_data.ref_wrenches.size();
        for (size_t i = 0; i < ref_wrenches_size; ++i) {
            m_modWrenches.data[i] = st_port_data.ref_wrenches[i];
        }
        m_modWrenchesOut.write();
    }

    m_emergencySignal.tm = m_qRef.tm;
    std::pair<bool, int> emergency_signal_pair = st->getEmergencySignal();
    if (emergency_signal_pair.first) {
        m_emergencySignal.data = emergency_signal_pair.second;
        m_emergencySignalOut.write();
    }

    m_baseFrameGenCp.tm = m_qRef.tm;
    const hrp::Vector3 rel_ref_cp = st->getOriginRefCP();
    m_baseFrameGenCp.data.x = rel_ref_cp(0);
    m_baseFrameGenCp.data.y = rel_ref_cp(1);
    m_baseFrameGenCp.data.z = rel_ref_cp(2);
    m_baseFrameGenCpOut.write();

    m_baseFrameActCp.tm = m_qRef.tm;
    const hrp::Vector3 rel_act_cp = st->getOriginActCP();
    m_baseFrameActCp.data.x = rel_act_cp(0);
    m_baseFrameActCp.data.y = rel_act_cp(1);
    m_baseFrameActCp.data.z = rel_act_cp(2);
    m_baseFrameActCpOut.write();

    {
        m_eeOriginCopInfo.tm = m_qRef.tm;
        const std::vector<hrp::Vector3> contact_cop_info = st->getContactCOPInfo();
        const size_t contact_size = contact_cop_info.size();
        for (size_t i = 0; i < contact_size; ++i) {
            for (size_t j = 0; j < 3; ++j) {
                m_eeOriginCopInfo.data[i * 3 + j] = contact_cop_info[i][j];
            }
        }
        m_eeOriginCopInfoOut.write();
    }

    if (st->getIfChangeServoGains()) {
        std::cerr << "[" << m_profile.instance_name << "] Change servo gains" << std::endl;
        st->setIfChangeServoGains(false);

        m_modPgain.tm = m_qRef.tm;
        m_modDgain.tm = m_qRef.tm;
        m_modTqPgain.tm = m_qRef.tm;
        // m_modTqDgain.tm = m_qRef.tm;
        m_modGainTransitionTime.tm = m_qRef.tm;

        const size_t num_joints = m_modPgain.data.length();
        for (size_t i = 0; i < num_joints; ++i) {
            m_modPgain.data[i] = st_port_data.servo_pgains[i];
            m_modDgain.data[i] = st_port_data.servo_dgains[i];
            m_modTqPgain.data[i] = st_port_data.servo_tqpgains[i];
            // m_modTqDgain.data[i] = st_port_data.servo_tqdgains[i];
            m_modGainTransitionTime.data[i] = st_port_data.gains_transition_times[i];
        }

        m_modPgainOut.write();
        m_modDgainOut.write();
        m_modTqPgainOut.write();
        // m_modTqDgainOut.write();
        m_modGainTransitionTimeOut.write();
    }
}

void AutoBalanceStabilizer::updateBodyParams()
{
    copyJointAnglesToRobotModel(m_robot, q_ref);
    m_robot->rootLink()->p = ref_base_pos;
    m_robot->rootLink()->R = ref_base_rot;
    m_robot->calcForwardKinematics();

    copyJointAnglesToRobotModel(m_act_robot, q_act);
    const hrp::Sensor* const gyro = m_act_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    const hrp::Matrix33 gyro_R = gyro->link->R * gyro->localR;
    m_act_robot->rootLink()->p = ref_base_pos; // actの原点位置は0でなくrefと同じになった
    m_act_robot->rootLink()->R = hrp::rotFromRpy(act_rpy) * (gyro_R.transpose() * m_act_robot->rootLink()->R);
    m_act_robot->calcForwardKinematics();
}

std::vector<hrp::LinkConstraint> AutoBalanceStabilizer::readContactPointsFromProps(const RTC::Properties& prop, std::vector<int>& contacts_link_indices)
{
    std::vector<hrp::LinkConstraint> init_constraints;

    // Read property
    // link_id, num_contact, contact_point (3 dim) * num_contact, local_rot (axis + angle)
    const coil::vstring contact_str = coil::split(prop["contact_points"], ",");
    init_constraints.reserve(contact_str.size());
    contacts_link_indices.reserve(contact_str.size());
    for (size_t i = 0; i < contact_str.size();) {
        std::cerr << contact_str[i] << " Index: " << m_robot->link(contact_str[i])->index << std::endl;
        contacts_link_indices.push_back(m_robot->link(contact_str[i])->index);
        hrp::LinkConstraint constraint(m_robot->link(contact_str[i++])->index);

        const std::string constraint_type_str = contact_str[i++];
        hrp::LinkConstraint::ConstraintType constraint_type = hrp::LinkConstraint::ConstraintType::FIX;
        if      (constraint_type_str == "FIX")    constraint_type = hrp::LinkConstraint::ConstraintType::FIX;
        else if (constraint_type_str == "ROTATE") constraint_type = hrp::LinkConstraint::ConstraintType::ROTATE;
        else if (constraint_type_str == "SLIDE")  constraint_type = hrp::LinkConstraint::ConstraintType::SLIDE;
        else if (constraint_type_str == "FLOAT")  constraint_type = hrp::LinkConstraint::ConstraintType::FLOAT;
        else if (constraint_type_str == "FREE")   constraint_type = hrp::LinkConstraint::ConstraintType::FREE;
        else {
            std::cerr << "[" << m_profile.instance_name << "] Constraint type " << constraint_type_str << " is not a proper type. Set FIX." << std::endl;
        }
        constraint.setConstraintType(constraint_type);

        // tmp
        const auto getToeContactPoints = [&](const std::vector<hrp::Vector3>& contact_points) {
            std::vector<hrp::Vector3> sorted_contact_points = contact_points;
            std::sort(sorted_contact_points.begin(), sorted_contact_points.end(),
                      [](hrp::Vector3& a, hrp::Vector3& b) { return a[0] > b[0]; });
            sorted_contact_points.resize(2);
            for (const auto& point : sorted_contact_points) std::cerr << "toe point: " << point.transpose() << std::endl;
            return sorted_contact_points;
        };
        const auto getHeelContactPoints = [&](const std::vector<hrp::Vector3>& contact_points) {
            std::vector<hrp::Vector3> sorted_contact_points = contact_points;
            std::sort(sorted_contact_points.begin(), sorted_contact_points.end(),
                      [](hrp::Vector3& a, hrp::Vector3& b) { return a[0] < b[0]; });
            sorted_contact_points.resize(2);
            for (const auto& point : sorted_contact_points) std::cerr << "heel point: " << point.transpose() << std::endl;
            return sorted_contact_points;
        };

        const size_t num_contact_points = std::stoi(contact_str[i++]);
        std::vector<hrp::Vector3> contact_points(num_contact_points);
        for (size_t j = 0; j < num_contact_points; ++j) {
            for (size_t k = 0; k < 3; ++k) {
                contact_points[j][k] = std::stod(contact_str[i++]);
            }
        }
        constraint.setLinkContactPoints(contact_points);
        constraint.setDefaultContactPoints(contact_points);
        constraint.setToeContactPoints(getToeContactPoints(contact_points));
        constraint.setHeelContactPoints(getHeelContactPoints(contact_points));
        constraint.calcLinkLocalPos();

        const hrp::Link* const link = m_robot->link(constraint.getLinkId());
        hrp::Vector3 cop_offset;
        if      (link->name == "RLEG_JOINT5") cop_offset = hrp::Vector3(0,0.03,0);
        else if (link->name == "LLEG_JOINT5") cop_offset = hrp::Vector3(0,-0.03,0);
        else                                  cop_offset = hrp::Vector3::Zero();
        constraint.setCOPOffset(cop_offset);
        constraint.setStartCOPOffset(cop_offset);

        std::array<double, 4> local_rot;
        for (size_t j = 0; j < 4; ++j) {
            local_rot[j] = std::stod(contact_str[i++]);
        }
        constraint.localRot() = Eigen::AngleAxisd(local_rot[3], Eigen::Vector3d(local_rot[0], local_rot[1], local_rot[2])).toRotationMatrix();
        init_constraints.push_back(constraint);
    }

    return init_constraints;
}

void AutoBalanceStabilizer::setupIKConstraints(const hrp::BodyPtr& _robot,
                                               const std::vector<hrp::LinkConstraint>& constraints)
{
    const size_t num_constraints = constraints.size();
    ik_constraints.resize(num_constraints + 2); // LinkConstraints + Body + COM constraint

    q_weights_interpolator = std::make_unique<interpolator>(_robot->numJoints(), m_dt, interpolator::CUBICSPLINE);
    q_weights_interpolator->setName(std::string(m_profile.instance_name) + " q_weights_interpolator");

    momentum_weights_interpolator = std::make_unique<interpolator>(3, m_dt, interpolator::CUBICSPLINE);
    momentum_weights_interpolator->setName(std::string(m_profile.instance_name) + " momentum_weights_interpolator");

    // Joint
    constexpr double DEFAULT_Q_WEIGHT = 2e-6; // default q weight
    default_q_weights = hrp::dvector::Constant(_robot->numJoints(), DEFAULT_Q_WEIGHT);
    flywheel_q_weights = hrp::dvector::Constant(_robot->numJoints(), DEFAULT_Q_WEIGHT);

    // default_q_weights <<
    //     1e-3, 1e-3, 1e-3, // chest
    //     W, W,    // HEAD
    //     1e-3, 1e-2, 2e-5, 2e-5, W, W, W, // LARM
    //     1e-3, 1e-2, 2e-5, 2e-5, W, W, W, // RARM
    //     W, W, W, W, W, W, // LLEG
    //     W, W, W, W, W, W; // RLEG

    // flywheel_q_weights <<
    //     W, W, W, // chest
    //     W, W,    // HEAD
    //     0, 0, 0, 0, 0, 0, 0, // LARM
    //     0, 0, 0, 0, 0, 0, 0, // RARM
    //     W, W, W, W, W, W, // LLEG
    //     W, W, W, W, W, W; // RLEG

    q_weights_interpolator->set(default_q_weights.data());
    flywheel_q_weights = default_q_weights;
    fik->q_ref_constraint_weight = default_q_weights;

    size_t i;
    for (i = 0; i < num_constraints; ++i) {
        ik_constraints[i].target_link_name = _robot->link(constraints[i].getLinkId())->name;
        ik_constraints[i].localPos = constraints[i].localPos();
        ik_constraints[i].localR = constraints[i].localRot();
        hrp::dvector6 const_weight;
        const_weight << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        ik_constraints[i].constraint_weight = const_weight;
    }

    // Body
    ik_constraints[i].target_link_name = _robot->rootLink()->name;
    hrp::dvector6 root_weight;
    // root_weight << 1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3;
    // root_weight << 1e-2, 1e-2, 1e-3, 1e-3, 1e-3, 1e-3;
    // root_weight << 1e-8, 1e-8, 1e-8, 1e-3, 1e-3, 1e-3;
    root_weight << 0, 0, 0, 1e-2, 1e-2, 1e-4;
    ik_constraints[i].constraint_weight = root_weight;
    fik->rootlink_rpy_llimit = hrp::Vector3(deg2rad(-15), deg2rad(-30), deg2rad(-180));
    fik->rootlink_rpy_ulimit = hrp::Vector3(deg2rad(15), deg2rad(45), deg2rad(180));
    ++i;

    // COM
    ik_constraints[i].target_link_name = "COM";
    // com_weight << 1, 1, 1e-1, 1e-2, 1e-2, 1e-4;
    // com_weight << 1, 1, 1e-1, 0, 0, 0;
    // com_weight << 1, 1, 1e-1, 1e-3, 1e-3, 1e-4;
    // com_weight << 1, 1, 1, 1e-2, 1e-2, 1e-4;
    // com_weight << 1, 1, 1e-1, 0, 0, 0;
    // com_weight << 1e-1, 1e-1, 1e-2, 0, 0, 0;
    // com_weight << 1e-3, 1e-3, 1e-4, 0, 0, 0;
    // default_momentum_weights = hrp::Vector3(1e-6, 1e-6, 1e-2);
    default_momentum_weights = hrp::Vector3(1, 1, 1e-2);
    flywheel_momentum_weights = hrp::Vector3(1, 1, 1e-2);
    momentum_weights_interpolator->set(default_momentum_weights.data());
    const hrp::Vector3 com_weight = hrp::Vector3(1, 1, 1);
    ik_constraints[i].constraint_weight.head<3>() = com_weight;
    ik_constraints[i].constraint_weight.tail<3>() = default_momentum_weights;
}

void AutoBalanceStabilizer::setIKConstraintsTarget()
{
    const std::vector<hrp::LinkConstraint>& cur_constraints = gg->getCurrentConstraints(loop).constraints;

    size_t i;
    for (i = 0; i < cur_constraints.size(); ++i) {
        if (cur_constraints[i].getConstraintType() == hrp::LinkConstraint::ConstraintType::FREE) {
            ik_constraints[i].constraint_weight.setZero();
            continue;
        }
        ik_constraints[i].targetPos   = cur_constraints[i].targetPos();
        ik_constraints[i].localPos    = cur_constraints[i].localPos();
        ik_constraints[i].localR      = cur_constraints[i].localRot();
        ik_constraints[i].targetOmega = hrp::omegaFromRot(cur_constraints[i].targetRot());
        ik_constraints[i].constraint_weight.setOnes();
        // std::cerr << "omega: " << ik_constraints[i].targetOmega.transpose() << std::endl;
    }

    // Body
    ik_constraints[i].targetPos = gg->rootPos();;
    ik_constraints[i].targetOmega = hrp::omegaFromRot(gg->rootRot());
    ++i;

    // Momentum
    constexpr double INTERPOLATE_TIME = 0.2; // [s]
    static hrp::Vector3 hoge = hrp::Vector3::Zero();
    if (gg->getCogMoment().squaredNorm() < 1e-6) {
        ref_angular_momentum.setZero();

        // if (interpolating_to_flywheel) {
        //     interpolating_to_flywheel = false;
        //     q_weights_interpolator->setGoal(default_q_weights.data(), nullptr, INTERPOLATE_TIME, true);
        //     momentum_weights_interpolator->setGoal(default_momentum_weights.data(), nullptr, INTERPOLATE_TIME, true);
        // }
    } else {
        ref_angular_momentum = hoge + gg->getCogMoment() * m_dt;
        ref_angular_momentum[2] = 0; // tmp

        // if (!interpolating_to_flywheel) {
        //     interpolating_to_flywheel = true;
        //     q_weights_interpolator->setGoal(flywheel_q_weights.data(), nullptr, INTERPOLATE_TIME, true);
        //     momentum_weights_interpolator->setGoal(flywheel_momentum_weights.data(), nullptr, INTERPOLATE_TIME, true);
        // }
    }

    q_weights_interpolator->get(fik->q_ref_constraint_weight.data(), true);
    momentum_weights_interpolator->get(ik_constraints[i].constraint_weight.tail<3>().data(), true);

    // COM
    ik_constraints[i].targetPos = gg->getCog();
    ik_constraints[i].targetOmega = ref_angular_momentum;
    hoge = hoge * 0.85 + ref_angular_momentum * 0.15;
    // ik_constraints[i].targetOmega.setZero(); // tmp

    ik_constraints[i].constraint_weight.tail(3) = hrp::Vector3(1e-6, 1e-6, 1e-6);
    // ik_constraints[i].targetOmega.setZero();
    // ik_constraints[i].targetOmega = hrp::clamp(ik_constraints[i].targetOmega, hrp::Vector3(-150, -150, -100), hrp::Vector3(150, 150, 100));
    // ik_constraints[i].targetOmega = hrp::Vector3::Zero();
    // std::cerr << "moment: " << ik_constraints[i].targetOmega.transpose() << std::endl;
    // std::cerr << "ref com:  " << ik_constraints[i].targetPos.transpose() << std::endl;
    // std::cerr << "ref com_vel:  " << gg->getCogVel().transpose() << std::endl;
    // std::cerr << "ref com_acc:  " << gg->getCogAcc().transpose() << std::endl;

    restoreRobotStatesForIK();
    // qref
    fik->setReferenceRobotStatesFromBody(m_robot);
}

// void AutoBalanceStabilizer::rotateRefForcesForFixCoords (coordinates& tmp_fix_coords)
// {
//     /* update ref_forces ;; StateHolder's absolute -> AutoBalanceStabilizer's absolute */
//     for (size_t i = 0; i < ref_wrenches.size(); ++i) {
//       // hrp::Matrix33 eeR;
//       // hrp::Link* parentlink;
//       // hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_names[i]);
//       // if (sensor) parentlink = sensor->link;
//       // else parentlink = m_vfs[sensor_names[i]].link;
//       // for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
//       //     if (it->second.target_link->name == parentlink->name) eeR = parentlink->R * it->second.localR;
//       // }
//       // End effector frame
//       //ref_forces[i] = eeR * ref_wrenches[i].head<3>();
//       // world frame
//         ref_forces[i]  = tmp_fix_coords.rot * ref_wrenches[i].head<3>();
//         ref_moments[i] = tmp_fix_coords.rot * ref_wrenches[i].tail<3>();
//     }
//     sbp_offset = tmp_fix_coords.rot * hrp::Vector3(sbp_offset);
// }

// TODO: Move to GaitGenerator
void AutoBalanceStabilizer::calcOutputRefForcesFromRefZmp()
{
    // TODO : need to be updated for multicontact and other walking
    const hrp::ConstraintsWithCount& cur_constraints = gg->getCurrentConstraints(loop);
    double alpha = (ref_zmp - cur_constraints.constraints[1].targetPos()).norm() / ((cur_constraints.constraints[0].targetPos() - ref_zmp).norm() + (cur_constraints.constraints[1].targetPos() - ref_zmp).norm());
    alpha = hrp::clamp(alpha, 0.0, 1.0);

    // if (DEBUGP) {
    //     std::cerr << "[" << m_profile.instance_name << "] alpha: " << alpha << std::endl;
    // }

    const double mg = m_robot->totalMass() * g_acc;
    ref_wrenches_for_st[0](2) = alpha * mg;
    ref_wrenches_for_st[1](2) = (1 - alpha) * mg;

    // if (use_force == MODE_REF_FORCE_WITH_FOOT || use_force == MODE_REF_FORCE_RFU_EXT_MOMENT) { // TODO : use other use_force mode. This should be depends on Stabilizer distribution mode.
    //     distributeReferenceZMPToWrenches(ref_zmp);
    // }

    // prev_ref_zmp = ref_zmp; // TODO: ここじゃない
}

// void AutoBalanceStabilizer::distributeReferenceZMPToWrenches (const hrp::Vector3& _ref_zmp)
// {
//     // apply inverse system
//     // TODO : fix 0.055 (zmp delay)
//     const hrp::Vector3 modified_ref_zmp = _ref_zmp + 0.055 * (_ref_zmp - prev_ref_zmp) / m_dt;

//     // TODO: Assume that constraints[0] and constraints[1] are legs

//     // std::vector<hrp::Vector3> cop_pos;
//     // std::vector<double> limb_gains;
//     // for (size_t i = 0 ; i < leg_names.size(); i++) {
//     //     ABCIKparam& tmpikp = ikp[leg_names[i]];
//     //     cop_pos.push_back(tmpikp.target_p0 + tmpikp.target_r0 * tmpikp.localR * default_zmp_offsets[i]);
//     //     limb_gains.push_back(m_contactStates.data[contact_states_index_map[leg_names[i]]] ? 1.0 : 0.0);
//     // }
//     const size_t ee_num = 2; // TODO
//     const size_t state_dim = 6 * ee_num;
//     constexpr size_t total_wrench_dim = 5;

//     // size_t total_fz = m_robot->totalMass() * gg->get_gravitational_acceleration();
//     size_t total_fz = m_ref_force[0].data[2]+m_ref_force[1].data[2];
//     //size_t total_wrench_dim = 3;
//     hrp::dmatrix Wmat = hrp::dmatrix::Identity(state_dim/2, state_dim/2);
//     hrp::dmatrix Gmat = hrp::dmatrix::Zero(total_wrench_dim, state_dim/2);
//     // Set Gmat
//     //   Fill Fz
//     for (size_t j = 0; j < ee_num; j++) {
//         if (total_wrench_dim == 3) {
//             Gmat(0,3*j+2) = 1.0;
//         } else {
//             for (size_t k = 0; k < 3; k++) Gmat(k,3*j+k) = 1.0;
//         }
//     }
//     //   Fill Nx and Ny
//     for (size_t i = 0; i < total_wrench_dim; i++) {
//         for (size_t j = 0; j < ee_num; j++) {
//             if ( i == total_wrench_dim-2 ) { // Nx
//                 Gmat(i,3*j+1) = -(cop_pos[j](2) - modified_ref_zmp(2));
//                 Gmat(i,3*j+2) = (cop_pos[j](1) - modified_ref_zmp(1));
//             } else if ( i == total_wrench_dim-1 ) { // Ny
//                 Gmat(i,3*j) = (cop_pos[j](2) - modified_ref_zmp(2));
//                 Gmat(i,3*j+2) = -(cop_pos[j](0) - modified_ref_zmp(0));
//             }
//         }
//     }
//     // Set Wmat
//     for (size_t j = 0; j < ee_num; j++) {
//         for (size_t i = 0; i < 3; i++) {
//             if (ee_num == 2)
//                 Wmat(i+j*3, i+j*3) = Wmat(i+j*3, i+j*3) * limb_gains[j] * (i==2? 1.0 : 0.01);
//             else
//                 Wmat(i+j*3, i+j*3) = Wmat(i+j*3, i+j*3) * limb_gains[j];
//         }
//     }
//     // Ret is wrench around cop_pos
//     //   f_cop = f_ee
//     //   n_ee = (cop_pos - ee_pos) x f_cop + n_cop
//     hrp::dvector ret(state_dim/2);
//     hrp::dvector total_wrench = hrp::dvector::Zero(total_wrench_dim);
//     total_wrench(total_wrench_dim-5) = m_ref_force[0].data[0]+m_ref_force[1].data[0];
//     total_wrench(total_wrench_dim-4) = m_ref_force[0].data[1]+m_ref_force[1].data[1];
//     total_wrench(total_wrench_dim-3) = total_fz;
//     calcWeightedLinearEquation(ret, Gmat, Wmat, total_wrench);
//     if (DEBUGP) {
//         std::cerr << "[" << m_profile.instance_name << "] distributeReferenceZMPToWrenches" << std::endl;
//     }
//     for (size_t i = 0 ; i < leg_names.size(); i++) {
//         size_t fidx = contact_states_index_map[leg_names[i]];
//         ABCIKparam& tmpikp = ikp[leg_names[i]];
//         hrp::Vector3 f_ee(ret(3*i), ret(3*i+1), ret(3*i+2));
//         //hrp::Vector3 tmp_ee_pos = tmpikp.target_p0 + tmpikp.target_r0 * tmpikp.localPos;
//         hrp::Vector3 tmp_ee_pos = tmpikp.target_p0;
//         hrp::Vector3 n_ee = (cop_pos[i]-tmp_ee_pos).cross(f_ee); // n_cop = 0
//         m_force[fidx].data[0] = f_ee(0);
//         m_force[fidx].data[1] = f_ee(1);
//         m_force[fidx].data[2] = f_ee(2);
//         m_force[fidx].data[3] = n_ee(0);
//         m_force[fidx].data[4] = n_ee(1);
//         m_force[fidx].data[5] = n_ee(2);
//         if (DEBUGP) {
//             std::cerr << "[" << m_profile.instance_name << "]   "
//                       << "ref_force  [" << leg_names[i] << "] " << f_ee.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N], "
//                       << "ref_moment [" << leg_names[i] << "] " << n_ee.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
//         }
//     }
//     if (DEBUGP) {
//         std::cerr << "[" << m_profile.instance_name << "]   Gmat = " << Gmat.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N,Nm]" << std::endl;
//         std::cerr << "[" << m_profile.instance_name << "]   total_wrench = " << total_wrench.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N,Nm]" << std::endl;
//         hrp::dvector tmp(total_wrench.size());
//         tmp = Gmat*ret;
//         std::cerr << "[" << m_profile.instance_name << "]   Gmat*ret = " << tmp.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N,Nm]" << std::endl;
//         std::cerr << "[" << m_profile.instance_name << "]   (Gmat*ret-total_wrench) = " << (tmp-total_wrench).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N,Nm]" << std::endl;
//         std::cerr << "[" << m_profile.instance_name << "]   ret = " << ret.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N,Nm]" << std::endl;
//         std::cerr << "[" << m_profile.instance_name << "]   Wmat(diag) = [";
//         for (size_t j = 0; j < ee_num; j++) {
//             for (size_t i = 0; i < 3; i++) {
//                 std::cerr << Wmat(i+j*3, i+j*3) << " ";
//             }
//         }
//         std::cerr << "]" << std::endl;
//     }
// }

void AutoBalanceStabilizer::adjustCOPCoordToTarget()
{
    gg->adjustCOPCoordToTarget(m_robot, loop);
    m_robot->rootLink()->p = gg->rootPos();
    m_robot->rootLink()->R = gg->rootRot();
    m_robot->calcForwardKinematics();
}

// TODO: delete limbs, この関数もいらなさそう
void AutoBalanceStabilizer::startABCparam(const OpenHRP::AutoBalanceStabilizerService::StrSequence& limbs)
{
    std::cerr << "[" << m_profile.instance_name << "] start auto balancer mode" << std::endl;
    Guard guard(m_mutex); // TODO: control_modeの切り替えのみmutexをかけるべき？
    double tmp_ratio = 0.0;
    transition_interpolator->clear();
    transition_interpolator->set(&tmp_ratio);
    tmp_ratio = 1.0;
    transition_interpolator->setGoal(&tmp_ratio, transition_time, true);
    prev_ref_zmp = ref_zmp;

    // TODO: mutexのためSTが終わった段階で呼ばれるので，m_robotが想定外のところにあるのでその修正．いずれ無くしたい
    restoreRobotStatesForIK();
    adjustCOPCoordToTarget();
    gg->resetGaitGenerator(m_robot, loop, m_dt);
    control_mode = MODE_SYNC_TO_ABC;
}

void AutoBalanceStabilizer::stopABCparam()
{
    std::cerr << "[" << m_profile.instance_name << "] stop auto balancer mode" << std::endl;
    //Guard guard(m_mutex);
    double tmp_ratio = 1.0;
    transition_interpolator->clear();
    transition_interpolator->set(&tmp_ratio);
    tmp_ratio = 0.0;
    transition_interpolator->setGoal(&tmp_ratio, transition_time, true);
    control_mode = MODE_SYNC_TO_IDLE;
}

bool AutoBalanceStabilizer::startWalking()
{
    if ( control_mode != MODE_ABC ) {
        std::cerr << "[" << m_profile.instance_name << "] Cannot start walking without MODE_ABC. Please startAutoBalancer." << std::endl;
        return false;
    }
    return true;
}

void AutoBalanceStabilizer::stopWalking ()
{
    gg->setWalkingState(false);
}

bool AutoBalanceStabilizer::startAutoBalancer(const OpenHRP::AutoBalanceStabilizerService::StrSequence& limbs)
{
    if (control_mode == MODE_IDLE) {
        startABCparam(limbs);
        waitABCTransition();
        return true;
    } else {
        return false;
    }
}

bool AutoBalanceStabilizer::stopAutoBalancer ()
{
  if (control_mode == MODE_ABC) {
    stopABCparam();
    waitABCTransition();
    return true;
  } else {
    return false;
  }
}

void AutoBalanceStabilizer::getStabilizerParam(OpenHRP::AutoBalanceStabilizerService::StabilizerParam& i_param)
{
    st->getStabilizerParam(i_param);
}

void AutoBalanceStabilizer::setStabilizerParam(const OpenHRP::AutoBalanceStabilizerService::StabilizerParam& i_param)
{
    st->setStabilizerParam(i_param);
}

void AutoBalanceStabilizer::startStabilizer(void)
{
    st->startStabilizer();
}

void AutoBalanceStabilizer::stopStabilizer(void)
{
    st->stopStabilizer();
}

void AutoBalanceStabilizer::testMotion(const int test_number)
{
    switch(test_number) {
      case 0:
          gg->startRunning(m_dt);
          break;
      case 1:
          gg->startJumping(m_dt);
          break;
      case 2:
          gg->startRunJumpDemo(m_dt);
          break;
      default:
          std::cerr << "[" << m_profile.instance_name << "] Unused number" << std::endl;
    }
}

void AutoBalanceStabilizer::waitABCTransition()
{
    while (!transition_interpolator->isEmpty()) usleep(1000);
    usleep(1000);
}

bool AutoBalanceStabilizer::goPos(const double x, const double y, const double th)
{
    if (is_stop_mode) {
        std::cerr << "[" << m_profile.instance_name << "] Cannot goPos while stopping mode." << std::endl;
        return false;
    }

    if (control_mode != MODE_ABC) {
        std::cerr << "[" << m_profile.instance_name << "] Cannot goPos if not MODE_ABC." << std::endl;
        return false;
    }

    const hrp::ConstraintsWithCount& cur_constraints = gg->getCurrentConstraints(loop);
    Eigen::Isometry3d target = cur_constraints.calcCOPCoord();
    target.translation() += target.linear() * hrp::Vector3(x, y, 0);
    target.linear() = target.linear() * Eigen::AngleAxisd(deg2rad(th), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    std::cerr << "gopos target: " << target.translation().transpose() << std::endl;

    // TODO: confファイルからcycleをよむ
    // CHIDORI
    // std::vector<int> support_link_cycle{13, 7};
    // std::vector<int> swing_link_cycle{7, 13};

    // Blue
    std::vector<int> support_link_cycle{25, 31};
    std::vector<int> swing_link_cycle{31, 25};
    if (!gg->goPos(target, support_link_cycle, swing_link_cycle)) return false;

    Guard guard(m_mutex);
    gg->setWalkingState(true);

    return true;
}

bool AutoBalanceStabilizer::setFootSteps(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss)
{
    if (is_stop_mode) {
        std::cerr << "[" << m_profile.instance_name << "] Cannot setFootSteps while stopping mode." << std::endl;
        return false;
    }

    if (control_mode != MODE_ABC) {
        std::cerr << "[" << m_profile.instance_name << "] Cannot setFootSteps if not MODE_ABC." << std::endl;
        return false;
    }

    const hrp::ConstraintsWithCount& cur_constraints = gg->getCurrentConstraints(loop);
    Eigen::Isometry3d start_coord = cur_constraints.calcCOPCoord();
    // target.translation() += target.linear() * hrp::Vector3(x, y, 0);
    // target.linear() = target.linear() * Eigen::AngleAxisd(deg2rad(th), Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // TODO: confファイルからcycleをよむ
    // CHIDORI
    // std::vector<int> support_link_cycle{13, 7};
    // std::vector<int> swing_link_cycle{7, 13};

    // Blue
    std::vector<int> support_link_cycle{25, 31};
    std::vector<int> swing_link_cycle{31, 25};
    int length = fss.length();
    int fs_side[length];        // 0->右 1->左
    Eigen::Isometry3d footstep_coord;
    hrp::Vector3 footstep_pos;
    Eigen::Quaterniond footstep_q;
    hrp::Vector3 footsteps_pos[length];
    Eigen::Quaterniond footsteps_rot[length];
    std::vector<std::string> side(length);

    for (size_t i = 0; i < length; ++i) {  // TODO footstepsに複数footstepが入る場合
        side[i] = fss[i].fs[0].leg;
        if (side[i] == "rleg") {
            fs_side[i] = 0;
        }
        else if (side[i] == "lleg") {
            fs_side[i] = 1;
        }
        else {
            std::cerr << "[" << m_profile.instance_name << "] footstep leg is not lleg or rleg" << std::endl; // biped only
            return false;
        }

        footstep_pos.x() = fss[i].fs[0].pos[0];
        footstep_pos.y() = fss[i].fs[0].pos[1];
        footstep_pos.z() = fss[i].fs[0].pos[2];
        footstep_q.w() = fss[i].fs[0].rot[0];
        footstep_q.x() = fss[i].fs[0].rot[1];
        footstep_q.y() = fss[i].fs[0].rot[2];
        footstep_q.z() = fss[i].fs[0].rot[3];

        footstep_coord.translation() = start_coord.translation() + start_coord.linear() * footstep_pos; // zがあるとき怪しい？start_coordでyaw軸以外の回転があった場合も
        footstep_coord.linear() = start_coord.linear() * footstep_q.normalized().toRotationMatrix();

        footsteps_pos[i] = footstep_coord.translation();
        footsteps_rot[i] = footstep_coord.linear();
    }

    if (!gg->setFootSteps(support_link_cycle, swing_link_cycle, footsteps_pos, footsteps_rot, fs_side, length)) return false;

    Guard guard(m_mutex);
    gg->setWalkingState(true);

    return true;
}

bool AutoBalanceStabilizer::setRunningFootSteps(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss)
{
    if (is_stop_mode) {
        std::cerr << "[" << m_profile.instance_name << "] Cannot setRunningFootSteps while stopping mode." << std::endl;
        return false;
    }

    if (control_mode != MODE_ABC) {
        std::cerr << "[" << m_profile.instance_name << "] Cannot setRunningFootSteps if not MODE_ABC." << std::endl;
        return false;
    }

    const hrp::ConstraintsWithCount& cur_constraints = gg->getCurrentConstraints(loop);
    Eigen::Isometry3d start_coord = cur_constraints.calcCOPCoord();

    // TODO: confファイルからcycleをよむ
    // CHIDORI
    // std::vector<int> support_link_cycle{13, 7};
    // std::vector<int> swing_link_cycle{7, 13};

    // Blue
    std::vector<int> support_link_cycle{25, 31}; // なんで要らなくなった？
    std::vector<int> swing_link_cycle{31, 25};

    int length = fss.length();
    int fs_side[length];        // 0->右 1->左
    Eigen::Isometry3d footstep_coord;
    hrp::Vector3 footstep_pos;
    Eigen::Quaterniond footstep_q;
    hrp::Vector3 footsteps_pos[length];
    Eigen::Quaterniond footsteps_rot[length];
    std::vector<std::string> side(length);

    for (size_t i = 0; i < length; ++i) {  // TODO footstepsに複数footstepが入る場合
        side[i]=fss[i].fs[0].leg;
        if (side[i] == "rleg") {
            fs_side[i] = 0;
        }
        else if (side[i] == "lleg") {
            fs_side[i] = 1;
        }
        else {
            std::cerr << "[" << m_profile.instance_name << "] footstep leg is not lleg or rleg" << std::endl; // biped only
            return false;
        }

        footstep_pos.x() = fss[i].fs[0].pos[0];
        footstep_pos.y() = fss[i].fs[0].pos[1];
        footstep_pos.z() = fss[i].fs[0].pos[2];
        footstep_q.w() = fss[i].fs[0].rot[0];
        footstep_q.x() = fss[i].fs[0].rot[1];
        footstep_q.y() = fss[i].fs[0].rot[2];
        footstep_q.z() = fss[i].fs[0].rot[3];

        footstep_coord.translation() = start_coord.translation() + start_coord.linear() * footstep_pos; // zがあるとき怪しい？start_coordでyaw軸以外の回転があった場合も
        footstep_coord.linear() = start_coord.linear() * footstep_q.normalized().toRotationMatrix();

        footsteps_pos[i] = footstep_coord.translation();
        footsteps_rot[i] = footstep_coord.linear();
    }

    // if (!gg->setRunningFootSteps(support_link_cycle, swing_link_cycle, footsteps_pos, footsteps_rot, fs_side, length, m_dt)) return false;
    if (!gg->setRunningFootSteps(support_link_cycle, swing_link_cycle, footsteps_pos, footsteps_rot, fs_side, length, m_dt)) return false;

    Guard guard(m_mutex);
    gg->setWalkingState(true);

    return true;
}

bool AutoBalanceStabilizer::setJumpingFootSteps(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss)
{
    if (is_stop_mode) {
        std::cerr << "[" << m_profile.instance_name << "] Cannot setJumpingFootSteps while stopping mode." << std::endl;
        return false;
    }

    if (control_mode != MODE_ABC) {
        std::cerr << "[" << m_profile.instance_name << "] Cannot setJumpingFootSteps if not MODE_ABC." << std::endl;
        return false;
    }

    // TODO biped only

    const hrp::ConstraintsWithCount& cur_constraints = gg->getCurrentConstraints(loop);
    Eigen::Isometry3d start_coord = cur_constraints.calcCOPCoord();

    // TODO: confファイルからcycleをよむ
    // CHIDORI
    // std::vector<int> support_link_cycle{13, 7};
    // std::vector<int> swing_link_cycle{7, 13};

    // Blue
    std::vector<int> support_link_cycle{25, 31};
    std::vector<int> swing_link_cycle{31, 25};

    int length = fss.length();
    std::vector<int> fs_num(length);
    for (size_t i = 0; i < length; ++i) {
        fs_num[i] = fss[i].fs.length();
        std::cerr << "fs_num[" << i << "] : " << fs_num[i] << std::endl;
    }

    Eigen::Isometry3d footstep_coord;
    hrp::Vector3 footstep_pos;
    Eigen::Quaterniond footstep_q;
    std::vector<std::vector<hrp::Vector3> > footsteps_pos(length);
    std::vector<std::vector<Eigen::Quaterniond> > footsteps_rot(length);
    std::vector<std::vector<int> > fs_side(length);        // 0->右 1->左
    std::string side;

    for (size_t i = 0; i < length; ++i){ // todo fsに足の数以上のfootstepが入ってたらfalseにする
        for (size_t j = 0; j < fs_num[i]; ++j){
            side = fss[i].fs[j].leg;
            if (side == "rleg") {
                fs_side[i].push_back(0);
            }
            else if (side == "lleg") {
                fs_side[i].push_back(1);
            }
            else {
                std::cerr << "[" << m_profile.instance_name << "] footstep leg is not lleg or rleg" << std::endl; // biped only
                return false;
            }
            footstep_pos.x() = fss[i].fs[j].pos[0];
            footstep_pos.y() = fss[i].fs[j].pos[1];
            footstep_pos.z() = fss[i].fs[j].pos[2];
            footstep_q.w() = fss[i].fs[j].rot[0];
            footstep_q.x() = fss[i].fs[j].rot[1];
            footstep_q.y() = fss[i].fs[j].rot[2];
            footstep_q.z() = fss[i].fs[j].rot[3]; // この辺まででなんかやらかしてそう

            footstep_coord.translation() = start_coord.translation() + start_coord.linear() * footstep_pos; // zがあるとき怪しい？start_coordでyaw軸以外の回転があった場合も
            footstep_coord.linear() = start_coord.linear() * footstep_q.normalized().toRotationMatrix();
            footstep_q = footstep_coord.linear();

            footsteps_pos[i].push_back(footstep_coord.translation());
            footsteps_rot[i].push_back(footstep_q);
        }
    }

    if (!gg->setJumpingFootSteps(m_dt, footsteps_pos, footsteps_rot, fs_side)) return false;
    // if (!gg->startJumping(m_dt)) return false;

    return true;
}

bool AutoBalanceStabilizer::goStop ()
{
    std::cerr << "[" << m_profile.instance_name << "] goStop" << std::endl;
    Eigen::Isometry3d modif = Eigen::Isometry3d::Identity();
    modif.translation() = hrp::Vector3(0.05, 0.01, 0);
    Guard guard(m_mutex);
    gg->modifyConstraintsTarget(loop, 0, 0, modif, 250, m_dt);
    return true;
}

bool AutoBalanceStabilizer::releaseEmergencyStop ()
{
  if (is_stop_mode) {
      std::cerr << "[" << m_profile.instance_name << "] releaseEmergencyStop" << std::endl;
      is_stop_mode = false;
  }
  return true;
}

void AutoBalanceStabilizer::waitFootSteps()
{
    while (gg_is_walking || !transition_interpolator->isEmpty()) usleep(1000);
    // gg->set_offset_velocity_param(0,0,0);
}

bool AutoBalanceStabilizer::setGaitGeneratorParam(const OpenHRP::AutoBalanceStabilizerService::GaitGeneratorParam& i_param)
{
    return true;
}

bool AutoBalanceStabilizer::getGaitGeneratorParam(OpenHRP::AutoBalanceStabilizerService::GaitGeneratorParam& i_param)
{
    return true;
}

bool AutoBalanceStabilizer::setAutoBalancerParam(const OpenHRP::AutoBalanceStabilizerService::AutoBalancerParam& i_param)
{
    return true;
}

bool AutoBalanceStabilizer::getAutoBalancerParam(OpenHRP::AutoBalanceStabilizerService::AutoBalancerParam& i_param)
{
    return true;
}

bool AutoBalanceStabilizer::getFootstepParam(OpenHRP::AutoBalanceStabilizerService::FootstepParam& i_param)
{
    return true;
}

// void AutoBalanceStabilizer::static_balance_point_proc_one(hrp::Vector3& tmp_input_sbp, const double ref_com_height)
// {
//     hrp::Vector3 target_sbp = hrp::Vector3(0, 0, 0);
//     hrp::Vector3 tmpcog = m_robot->calcCM();
//     if ( use_force == MODE_NO_FORCE ) {
//         tmp_input_sbp = tmpcog + sbp_cog_offset;
//     } else {
//         calc_static_balance_point_from_forces(target_sbp, tmpcog, ref_com_height);
//         tmp_input_sbp = target_sbp - sbp_offset;
//         sbp_cog_offset = tmp_input_sbp - tmpcog;
//     }
// }

// void AutoBalanceStabilizer::calc_static_balance_point_from_forces(hrp::Vector3& sb_point, const hrp::Vector3& tmpcog, const double ref_com_height)
// {
//     hrp::Vector3 denom, nume;
//     /* sb_point[m] = nume[kg * m/s^2 * m] / denom[kg * m/s^2] */
//     double mass = m_robot->totalMass();
//     double mg = mass * gg->get_gravitational_acceleration();
//     hrp::Vector3 total_sensor_ref_force = hrp::Vector3::Zero();
//     for (size_t i = 0; i < ref_forces.size(); i++) {
//         total_sensor_ref_force += ref_forces[i];
//     }
//     hrp::Vector3 total_nosensor_ref_force = mg * hrp::Vector3::UnitZ() - total_sensor_ref_force; // total ref force at the point without sensors, such as torso
//     hrp::Vector3 tmp_ext_moment = fix_leg_coords2.pos.cross(total_nosensor_ref_force) + fix_leg_coords2.rot * hrp::Vector3(m_refFootOriginExtMoment.data.x, m_refFootOriginExtMoment.data.y, m_refFootOriginExtMoment.data.z);
//     // For MODE_REF_FORCE_RFU_EXT_MOMENT, store previous root position to calculate influence from tmp_ext_moment while walking (basically, root link moves while walking).
//     //   Calculate values via fix_leg_coords2 relative/world values.
//     static hrp::Vector3 prev_additional_force_applied_pos = fix_leg_coords2.rot.transpose() * (additional_force_applied_link->p - fix_leg_coords2.pos);
//     //   If not is_hold_value (not hold value), update prev_additional_force_applied_pos
//     if (!is_ref_foot_origin_ext_moment_hold_value) {
//         prev_additional_force_applied_pos = fix_leg_coords2.rot.transpose() * (additional_force_applied_link->p - fix_leg_coords2.pos);
//     }
//     const hrp::Vector3 tmp_prev_additional_force_applied_pos = fix_leg_coords2.rot * prev_additional_force_applied_pos + fix_leg_coords2.pos;
//     // Calculate SBP
//     for (size_t j = 0; j < 2; j++) {
//         nume(j) = mg * tmpcog(j);
//         denom(j) = mg;
//         if ( use_force == MODE_REF_FORCE_RFU_EXT_MOMENT ) {
//             //nume(j) += (j==0 ? tmp_ext_moment(1):-tmp_ext_moment(0));
//             nume(j) += (tmp_prev_additional_force_applied_pos(j) - additional_force_applied_link->p(j)) * total_nosensor_ref_force(2) + (j == 0 ? tmp_ext_moment(1) : -tmp_ext_moment(0));
//             denom(j) -= total_nosensor_ref_force(2);
//         } else {
//             for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
//                 // Check leg_names. leg_names is assumed to be support limb for locomotion, cannot be used for manipulation. If it->first is not included in leg_names, use it for manipulation and static balance point calculation.
//                 if (std::find(leg_names.begin(), leg_names.end(), it->first) == leg_names.end()) {
//                     size_t idx = contact_states_index_map[it->first];
//                     // Force applied point is assumed as end effector
//                     hrp::Vector3 fpos = it->second.target_link->p + it->second.target_link->R * it->second.localPos;
//                     nume(j) += ( (fpos(2) - ref_com_height) * ref_forces[idx](j) - fpos(j) * ref_forces[idx](2) );
//                     nume(j) += (j==0 ? ref_moments[idx](1):-ref_moments[idx](0));
//                     denom(j) -= ref_forces[idx](2);
//                 }
//             }
//             if ( use_force == MODE_REF_FORCE_WITH_FOOT ) {
//                 hrp::Vector3 fpos(additional_force_applied_link->p+additional_force_applied_point_offset);
//                 nume(j) += ( (fpos(2) - ref_com_height) * total_nosensor_ref_force(j) - fpos(j) * total_nosensor_ref_force(2) );
//                 denom(j) -= total_nosensor_ref_force(2);
//             }
//         }
//         sb_point(j) = nume(j) / denom(j);
//     }
//     sb_point(2) = ref_com_height;
// }

std::string AutoBalanceStabilizer::getUseForceModeString ()
{
    switch (use_force) {
    case OpenHRP::AutoBalanceStabilizerService::MODE_NO_FORCE:
        return "MODE_NO_FORCE";
    case OpenHRP::AutoBalanceStabilizerService::MODE_REF_FORCE:
        return "MODE_REF_FORCE";
    case OpenHRP::AutoBalanceStabilizerService::MODE_REF_FORCE_WITH_FOOT:
        return "MODE_REF_FORCE_WITH_FOOT";
    case OpenHRP::AutoBalanceStabilizerService::MODE_REF_FORCE_RFU_EXT_MOMENT:
        return "MODE_REF_FORCE_RFU_EXT_MOMENT";
    default:
        return "";
    }
};

//
extern "C"
{
void AutoBalanceStabilizerInit(RTC::Manager* manager)
{
    RTC::Properties profile(autobalancestabilizer_spec);
    manager->registerFactory(profile,
                             RTC::Create<AutoBalanceStabilizer>,
                             RTC::Delete<AutoBalanceStabilizer>);
}
};
