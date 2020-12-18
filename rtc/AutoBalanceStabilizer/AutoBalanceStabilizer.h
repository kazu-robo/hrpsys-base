// -*- C++ -*-
/*!
 * @file  AutoBalanceStabilizer.h
 * @brief autobalancestabilizer component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef AUTOBALANCESTABILIZER_H
#define AUTOBALANCESTABILIZER_H

#include <memory>
#include <mutex>
#include <boost/make_shared.hpp>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/CorbaNaming.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>

#include <hrpModel/Link.h>
#include <hrpModel/Body.h>

#include "Utility.h"
#include "FullbodyInverseKinematicsSolver.h"
#include "GaitGenerator.h"
#include "Stabilizer.h"
// Service implementation headers
// <rtc-template block="service_impl_h">
#include "AutoBalanceStabilizerService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class interpolator;
class IIRFilter;

class AutoBalanceStabilizer : public RTC::DataFlowComponentBase
{
  public:
    AutoBalanceStabilizer(RTC::Manager* manager);
    // virtual ~AutoBalanceStabilizer();

    // The initialize action (on CREATED->ALIVE transition)
    // formaer rtc_init_entry()
    RTC::ReturnCode_t onInitialize() override;

    // The finalize action (on ALIVE->END transition)
    // formaer rtc_exiting_entry()
    RTC::ReturnCode_t onFinalize() override;

    // The activated action (Active state entry action)
    // former rtc_active_entry()
    RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id) override;

    // The deactivated action (Active state exit action)
    // former rtc_active_exit()
    RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id) override;

    // The execution action that is invoked periodically
    // former rtc_active_do()
    RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id) override;

    bool startAutoBalancer(const ::OpenHRP::AutoBalanceStabilizerService::StrSequence& limbs);
    bool stopAutoBalancer();

    std::vector<int> getConstraintLinkIds() { return gg->getConstraintLinkIds(); }
    /* Service for GaitGenerator */
    void printConstraintLinkIds()
    {
        for (const int id : gg->getConstraintLinkIds()) std::cerr << id << " ";
        std::cerr << std::endl;
    }
    void setDefaultSingleSupportTime(const double time, const double dt);
    void setDefaultDoubleSupportTime(const double time, const double dt);
    void setDefaultToeSupportTime(const double time, const double dt);
    void setDefaultHeelSupportTime(const double time, const double dt);
    void setDefaultStepHeight(const double height)   { gg->setDefaultStepHeight(height); }
    void setMaxStride(const double stride)           { gg->setMaxStride(stride); }
    void setMaxRotAngle(const double angle_rad)      { gg->setMaxRotAngle(angle_rad); }
    void setUseToeHeel(const bool use_toe_heel)      { gg->setUseToeHeel(use_toe_heel); }
    void setToeKickAngle(const double angle_rad)     { gg->setToeKickAngle(angle_rad); }
    void setHeelContactAngle(const double angle_rad) { gg->setHeelContactAngle(angle_rad); }
    bool setToeContactPoints(const int link_id, const std::vector<hrp::Vector3>& contact_points)
    {
        return gg->setToeContactPoints(link_id, contact_points);
    }
    bool setHeelContactPoints(const int link_id, const std::vector<hrp::Vector3>& contact_points)
    {
        return gg->setHeelContactPoints(link_id, contact_points);
    }

    void setWalkingMode(const OpenHRP::AutoBalanceStabilizerService::WalkingMode mode) {
        // check abc mode
        switch (mode) {
        case OpenHRP::AutoBalanceStabilizerService::PREVIEW_CONTROL:
            gg->setWalkingMode(hrp::GaitGenerator::PREVIEW_CONTROL);
            break;
        case OpenHRP::AutoBalanceStabilizerService::FOOT_GUIDED_WALK:
            gg->setWalkingMode(hrp::GaitGenerator::FOOT_GUIDED_WALK);
            break;
        default:
            break;
        }
    }

    bool goPos(const double x, const double y, const double th);
    bool goVelocity(const double& vx, const double& vy, const double& vth);
    bool goStop();
    bool emergencyStop ();
    bool setIKJointWeight(const hrp::dvector q_weights)
    {
        if (fik->q_ref_constraint_weight.size() != q_weights.size()) return false;
        fik->q_ref_constraint_weight = q_weights;
        return true;
    }
    bool setFootSteps(const OpenHRP::AutoBalanceStabilizerService::FootstepSequence& fs, CORBA::Long overwrite_fs_idx);
    bool setFootSteps(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss);
    // bool setFootSteps(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx);
    bool setFootStepsWithParam(const OpenHRP::AutoBalanceStabilizerService::FootstepSequence& fs, const OpenHRP::AutoBalanceStabilizerService::StepParamSequence& sps, CORBA::Long overwrite_fs_idx);
    bool setFootStepsWithParam(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss, const OpenHRP::AutoBalanceStabilizerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx);
    bool setRunningFootSteps(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss);
    // TODO setRunningFootStepsWithParam ?
    bool setJumpingFootSteps(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss);
    // TODO setJumpingFootStepsWithParam ?
    void waitFootSteps();
    void waitFootStepsEarly(const double tm);
    bool setGaitGeneratorParam(const OpenHRP::AutoBalanceStabilizerService::GaitGeneratorParam& i_param);
    bool getGaitGeneratorParam(OpenHRP::AutoBalanceStabilizerService::GaitGeneratorParam& i_param);
    bool setAutoBalancerParam(const OpenHRP::AutoBalanceStabilizerService::AutoBalancerParam& i_param);
    bool getAutoBalancerParam(OpenHRP::AutoBalanceStabilizerService::AutoBalancerParam& i_param);
    bool getFootstepParam(OpenHRP::AutoBalanceStabilizerService::FootstepParam& i_param);

    bool adjustFootSteps(const OpenHRP::AutoBalanceStabilizerService::Footstep& rfootstep, const OpenHRP::AutoBalanceStabilizerService::Footstep& lfootstep);
    bool getRemainingFootstepSequence(OpenHRP::AutoBalanceStabilizerService::FootstepSequence_out o_footstep, CORBA::Long& o_current_fs_idx);
    bool getGoPosFootstepsSequence(const double& x, const double& y, const double& th, OpenHRP::AutoBalanceStabilizerService::FootstepsSequence_out o_footstep);
    bool releaseEmergencyStop();
    void distributeReferenceZMPToWrenches (const hrp::Vector3& _ref_zmp);

    /* Service for Stabilizer */
    void getStabilizerParam(OpenHRP::AutoBalanceStabilizerService::StabilizerParam& i_param);
    void setStabilizerParam(const OpenHRP::AutoBalanceStabilizerService::StabilizerParam& i_param);
    void startStabilizer(void);
    void stopStabilizer(void);
    void testMotion(const int test_number);

  protected:
    // Configuration variable declaration
    // <rtc-template block="config_declare">

    // DataInPort declaration
    // <rtc-template block="inport_declare">
    TimedDoubleSeq m_qCurrent;
    InPort<TimedDoubleSeq> m_qCurrentIn;
    TimedDoubleSeq m_qRef;
    InPort<TimedDoubleSeq> m_qRefIn;
    TimedOrientation3D m_rpy;
    InPort<TimedOrientation3D> m_rpyIn;
    TimedPoint3D m_basePos;
    InPort<TimedPoint3D> m_basePosIn;
    TimedOrientation3D m_baseRpy;
    InPort<TimedOrientation3D> m_baseRpyIn;
    TimedPoint3D m_refZmp;
    InPort<TimedPoint3D> m_refZmpIn;
    TimedDoubleSeq m_optionalData;
    InPort<TimedDoubleSeq> m_optionalDataIn;
    std::vector<TimedDoubleSeq> m_ref_force;
    std::vector<InPort<TimedDoubleSeq> *> m_ref_forceIn;
    TimedPoint3D m_refFootOriginExtMoment;
    InPort<TimedPoint3D> m_refFootOriginExtMomentIn;
    TimedBoolean m_refFootOriginExtMomentIsHoldValue;
    InPort<TimedBoolean> m_refFootOriginExtMomentIsHoldValueIn;
    std::vector<TimedDoubleSeq> m_wrenches;
    std::vector<InPort<TimedDoubleSeq> *> m_wrenchesIn;

    // DataOutPort declaration
    // <rtc-template block="outport_declare">
    OutPort<TimedDoubleSeq> m_qOut;
    TimedDoubleSeq m_tau;
    OutPort<TimedDoubleSeq> m_tauOut;
    OutPort<TimedPoint3D> m_basePosOut;
    OutPort<TimedOrientation3D> m_baseRpyOut;
    TimedPose3D m_basePose;
    OutPort<TimedPose3D> m_basePoseOut;
    TimedAcceleration3D m_accRef;
    OutPort<TimedAcceleration3D> m_accRefOut;
    TimedLongSeq m_refContactStates;
    OutPort<TimedLongSeq> m_refContactStatesOut;
    TimedDoubleSeq m_controlSwingSupportTime;
    OutPort<TimedDoubleSeq> m_controlSwingSupportTimeOut;
    TimedPoint3D m_sbpCogOffset;
    OutPort<TimedPoint3D> m_sbpCogOffsetOut;
    TimedPoint3D m_diffFootOriginExtMoment;
    OutPort<TimedPoint3D> m_diffFootOriginExtMomentOut;
    TimedLong m_emergencySignal;
    OutPort<TimedLong> m_emergencySignalOut;
    TimedDoubleSeq m_pgain;
    OutPort<TimedDoubleSeq> m_pgainOut;
    TimedDoubleSeq m_dgain;
    OutPort<TimedDoubleSeq> m_dgainOut;
    TimedDoubleSeq m_tqpgain;
    OutPort<TimedDoubleSeq> m_tqpgainOut;
    // TimedDoubleSeq m_tqdgain;
    // OutPort<TimedDoubleSeq> m_tqdgainOut;
    TimedDoubleSeq m_gainTransitionTime;
    OutPort<TimedDoubleSeq> m_gainTransitionTimeOut;

    // for debug
    TimedDoubleSeq m_baseTform;
    OutPort<TimedDoubleSeq> m_baseTformOut;
    OutPort<TimedPoint3D> m_refZmpOut;
    TimedPoint3D m_refCmp; // Calculated by cog trajectory
    OutPort<TimedPoint3D> m_refCmpOut;
    TimedPoint3D m_baseOriginRefZmp;
    OutPort<TimedPoint3D> m_baseOriginRefZmpOut;
    TimedPoint3D m_refCog;
    OutPort<TimedPoint3D> m_refCogOut;
    TimedPoint3D m_refCogVel;
    OutPort<TimedPoint3D> m_refCogVelOut;
    TimedPoint3D m_refCogAcc;
    OutPort<TimedPoint3D> m_refCogAccOut;
    TimedPoint3D m_refAngularMomentumRPY;
    OutPort<TimedPoint3D> m_refAngularMomentumRPYOut;
    // Data from Stabilizer
    TimedOrientation3D m_actBaseRpy;
    OutPort<TimedOrientation3D> m_actBaseRpyOut;
    TimedPoint3D m_baseOriginActZmp;
    OutPort<TimedPoint3D> m_baseOriginActZmpOut;
    TimedPoint3D m_originRefZmp;
    OutPort<TimedPoint3D> m_originRefZmpOut;
    TimedPoint3D m_originNewRefZmp;
    OutPort<TimedPoint3D> m_originNewRefZmpOut;
    TimedPoint3D m_originActZmp;
    OutPort<TimedPoint3D> m_originActZmpOut;
    TimedPoint3D m_footOriginRefCog;
    OutPort<TimedPoint3D> m_footOriginRefCogOut;
    TimedPoint3D m_footOriginActCog;
    OutPort<TimedPoint3D> m_footOriginActCogOut;
    TimedBooleanSeq m_actContactStates;
    OutPort<TimedBooleanSeq> m_actContactStatesOut;
    TimedDoubleSeq m_newRefWrenches;
    OutPort<TimedDoubleSeq> m_newRefWrenchesOut;
    TimedPoint3D m_refCP;
    OutPort<TimedPoint3D> m_refCPOut;
    TimedPoint3D m_actCP;
    OutPort<TimedPoint3D> m_actCPOut;
    TimedDoubleSeq m_COPInfo;
    OutPort<TimedDoubleSeq> m_COPInfoOut;

    // CORBA Port declaration
    // <rtc-template block="corbaport_declare">
    RTC::CorbaPort m_AutoBalanceStabilizerServicePort;

    // Service declaration
    // <rtc-template block="service_declare">
    AutoBalanceStabilizerService_impl m_service0;

  private:
    hrp::BodyPtr m_robot = boost::make_shared<hrp::Body>();
    hrp::BodyPtr m_act_robot;
    std::mutex m_mutex;
    double m_dt;
    unsigned int m_debugLevel = 0;
    size_t loop = 0;
    double g_acc = 9.80665;

    // Fullbody Inverse Kinematics Solver
    hrp::dvector q_prev_ik;
    hrp::Vector3 root_pos_prev_ik = hrp::Vector3::Zero();
    hrp::Matrix33 root_rot_prev_ik = hrp::Matrix33::Identity();

    size_t max_ik_iteration = 5;
    std::vector<hrp::IKConstraint> ik_constraints;
    std::unique_ptr<hrp::FullbodyInverseKinematicsSolver> fik;

    bool interpolating_to_flywheel = false;
    hrp::dvector default_q_weights;
    hrp::dvector flywheel_q_weights;
    std::unique_ptr<interpolator> q_weights_interpolator;

    hrp::Vector3 default_momentum_weights;
    hrp::Vector3 flywheel_momentum_weights;
    std::unique_ptr<interpolator> momentum_weights_interpolator;


    inline bool DEBUGP() { return (m_debugLevel == 1 && loop % 200 == 0) || m_debugLevel > 1; }
    bool loadModel(hrp::BodyPtr body, const string& model_path);

    // -- Functions for OpenRTM port --
    void setupBasicPort();
    inline void readInportData();
    inline void writeOutPortData(const hrp::Vector3& base_pos,
                                 const hrp::Matrix33& base_rot,
                                 const hrp::Vector3& ref_zmp_global,
                                 const hrp::Vector3& ref_zmp_base_frame,
                                 // const hrp::Vector3& ref_cmp,
                                 const hrp::Vector3& ref_cog,
                                 const hrp::Vector3& ref_cog_vel,
                                 const hrp::Vector3& ref_cog_acc,
                                 const hrp::Vector3& ref_momentum,
                                 const hrp::Vector3& sbp_cog_offset,
                                 const hrp::Vector3& acc_ref,
                                 const hrp::stabilizerPortData& st_port_data);
    // -- Functions for OpenRTM port --

    void updateBodyParams();
    std::vector<hrp::LinkConstraint> readContactPointsFromProps(const RTC::Properties& prop);

    // void addBodyConstraint(std::vector<hrp::LinkConstraint>& constraints,
    //                        const hrp::BodyPtr& _robot);
    void setupIKConstraints(const hrp::BodyPtr& _robot,
                            const std::vector<hrp::LinkConstraint>& constraints);
    void setIKConstraintsTarget();
    void storeRobotStatesForIK()
    {
        hrp::copyJointAnglesFromRobotModel(q_prev_ik, m_robot);
        root_pos_prev_ik = m_robot->rootLink()->p;
        root_rot_prev_ik = m_robot->rootLink()->R;
    }
    void restoreRobotStatesForIK()
    {
        hrp::copyJointAnglesToRobotModel(m_robot, q_prev_ik);
        m_robot->rootLink()->p = root_pos_prev_ik;
        m_robot->rootLink()->R = root_rot_prev_ik;
        m_robot->calcForwardKinematics();
    }

    void adjustCOPCoordToTarget();

    void writeOutportDataForLeggedRobot();
    void getTargetParameters();
    void getActualParameters();
    void solveFullbodyIK ();
    void startABCparam(const ::OpenHRP::AutoBalanceStabilizerService::StrSequence& limbs);
    void stopABCparam();
    void waitABCTransition();
    void calcOutputRefForcesFromRefZmp();

    // For service
    bool startWalking ();
    void stopWalking ();
    // static balance point offsetting
    void static_balance_point_proc_one(hrp::Vector3& tmp_input_sbp, const double ref_com_height);
    void calc_static_balance_point_from_forces(hrp::Vector3& sb_point, const hrp::Vector3& tmpcog, const double ref_com_height);
    // hrp::Vector3 calc_vel_from_hand_error (const rats::coordinates& tmp_fix_coords);
    // bool isOptionalDataContact (const std::string& ee_name)
    // {
    //     return (std::fabs(m_optionalData.data[contact_states_index_map[ee_name]] - 1.0) < 0.1) ? true : false;
    // }
    std::string getUseForceModeString ();

    // member variables to store values for data ports
    hrp::dvector q_act; // TODO: m_robot_act があればいらない ?
    hrp::dvector q_ref; // TODO: m_robotだけで十分?
    hrp::Vector3 act_rpy; // TODO: m_robot_act があればいらない ?
    hrp::Vector3 ref_base_pos; // TODO: m_robotだけで十分?
    hrp::Matrix33 ref_base_rot; // TODO: m_robotだけで十分?

    hrp::Vector3 input_ref_zmp;
    std::vector<hrp::dvector6> ref_wrenches;
    std::vector<hrp::dvector6> ref_wrenches_for_st;
    std::vector<hrp::dvector6> act_wrenches;

    std::vector<bool> ref_contact_states; // TODO: delete
    std::vector<double> control_swing_support_time; // TODO: delete
    hrp::Vector3 ref_foot_origin_ext_moment;
    bool is_ref_foot_origin_ext_moment_hold_value;

    std::unique_ptr<hrp::Stabilizer> st;

    // for gg
    std::unique_ptr<hrp::GaitGenerator> gg;
    bool gg_is_walking = false;

    // for abc
    hrp::Vector3 ref_zmp = hrp::Vector3::Zero();
    hrp::Vector3 prev_ref_zmp = hrp::Vector3::Zero();
    hrp::Vector3 ref_angular_momentum = hrp::Vector3::Zero();

    hrp::Vector3 prev_imu_sensor_pos = hrp::Vector3::Zero();
    hrp::Vector3 prev_imu_sensor_vel = hrp::Vector3::Zero();
    // enum {BIPED, TROT, PACE, CRAWL, GALLOP} gait_type;
    enum {MODE_IDLE, MODE_ABC, MODE_SYNC_TO_IDLE, MODE_SYNC_TO_ABC} control_mode = MODE_IDLE;
    std::map<std::string, hrp::VirtualForceSensorParam> m_vfs;
    std::vector<std::string> sensor_names, leg_names, ee_vec;
    Eigen::Isometry3d target_root;
    // rats::coordinates fix_leg_coords, fix_leg_coords2; // TODO: rename
    double d_pos_z_root, limb_stretch_avoidance_time_const, limb_stretch_avoidance_vlimit[2];
    bool use_limb_stretch_avoidance;

    double transition_interpolator_ratio = 0;
    double transition_time = 2.0;
    std::unique_ptr<interpolator> transition_interpolator;

    // static balance point offsetting
    hrp::Vector3 sbp_offset = hrp::Vector3::Zero();
    hrp::Vector3 sbp_cog_offset = hrp::Vector3::Zero();
    enum {MODE_NO_FORCE, MODE_REF_FORCE, MODE_REF_FORCE_WITH_FOOT, MODE_REF_FORCE_RFU_EXT_MOMENT} use_force = MODE_REF_FORCE;
    std::vector<hrp::Vector3> ref_forces, ref_moments;

    bool is_stop_mode = false;

    hrp::InvDynStateBuffer idsb;
    std::vector<IIRFilter> invdyn_zmp_filters;

    // Used for ref force balancing.
    hrp::Link* additional_force_applied_link; // TODO: raw pointer?
    hrp::Vector3 additional_force_applied_point_offset;
};


extern "C"
{
    void AutoBalanceStabilizerInit(RTC::Manager* manager);
};

#endif // AUTOBALANCESTABILIZER_H
