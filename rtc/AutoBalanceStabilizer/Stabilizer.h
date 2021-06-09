// -*- C++ -*-
/*!
 * @file  Stabilizer.h
 * @brief stabilizer filter
 * @date  $Date$
 *
 * $Id$
 */

#ifndef ABS_STABILIZER_H
#define ABS_STABILIZER_H

#include <memory>
#include <mutex>
#include <array>
#include <hrpModel/Body.h>
#include "../ImpedanceController/JointPathEx.h" // TODO: delete
#include "../ImpedanceController/RatsMatrix.h" // TODO: delete
#include "../TorqueFilter/IIRFilter.h"
#include "hrpsys/idl/AutoBalanceStabilizerService.hh"
#include "Utility.h"
#include "ZMPDistributor.h"

namespace hrp {

struct paramsFromAutoBalancer
{
    hrp::dvector q_ref;
    hrp::Vector3 zmp_ref;
    hrp::Vector3 base_pos_ref;
    hrp::Vector3 base_rpy_ref;
    bool is_walking;
    std::vector<bool> ref_contact_states;
    // std::vector<double> toe_heel_ratio;
    std::vector<double> control_swing_support_time;
    std::vector<hrp::dvector6> wrenches_ref;
    // std::vector<hrp::Vector3> limb_cop_offsets;
    hrp::Vector3 sbp_cog_offset;
};

struct paramsFromSensors
{
    hrp::dvector q_current;
    hrp::Vector3 rpy;
    std::vector<hrp::dvector6> wrenches;
};

struct stabilizerPortData
{
    hrp::Vector3 act_base_rpy;
    hrp::dvector joint_angles;
    hrp::dvector joint_torques;
    hrp::Vector3 ref_zmp;
    hrp::Vector3 new_ref_zmp;
    hrp::Vector3 act_zmp;
    hrp::Vector3 rel_act_zmp;
    hrp::Vector3 origin_ref_cog;
    hrp::Vector3 origin_act_cog;
    std::vector<double> servo_pgains;
    std::vector<double> servo_dgains;
    std::vector<double> servo_tqpgains;
    // std::vector<double> servo_tqdgains;
    std::vector<double> gains_transition_times;
    hrp::dvector ref_wrenches;
};

class Stabilizer
{
  public:
    Stabilizer(const hrp::BodyPtr& _robot, const std::string& _comp_name, const double _dt, std::mutex& _mutex);
    virtual ~Stabilizer() {};

    void initStabilizer(const RTC::Properties& prop, const size_t ee_num);
    void execStabilizer(const paramsFromAutoBalancer& abc_param,
                        const paramsFromSensors& sensor_param);

    // TODO: tmporarary function: delete this function after merging autobalancestabilizer IK and stabilizer IK
    void addSTIKParam(const std::string& ee_name, const std::string& target_name,
                      const std::string& ee_base, const std::string& sensor_name,
                      const hrp::Vector3& localp, const hrp::Matrix33& localR)
    {
        STIKParam ikp;
        ikp.ee_name = ee_name;
        ikp.target_name = target_name;
        ikp.ee_base = ee_base;
        ikp.sensor_name = sensor_name;
        ikp.localp = localp;
        ikp.localR = localR;

        if (ee_name.find("leg") == std::string::npos) { // Arm default
            ikp.eefm_ee_forcemoment_distribution_weight = hrp::dvector6::Zero();
        } else { // Leg default
            for (size_t i = 0; i < 3; i++) {
                ikp.eefm_ee_forcemoment_distribution_weight[i]     = 1.0; // Force
                ikp.eefm_ee_forcemoment_distribution_weight[i + 3] = 1e-2; // Moment
            }
        }

        for (hrp::Link* root = m_robot->link(ikp.target_name);
             !root->isRoot();
             root = root->parent) {
            ikp.max_limb_length += root->b.norm();
            ikp.parent_name = root->name;
        }

        // Set low pass filter (50 [Hz])
        ikp.target_ee_diff_p_filter = std::make_shared<FirstOrderLowPassFilter<hrp::Vector3>>(50.0, dt, hrp::Vector3::Zero());
        ikp.target_ee_diff_r_filter = std::make_shared<FirstOrderLowPassFilter<hrp::Vector3>>(50.0, dt, hrp::Vector3::Zero());

        stikp.push_back(ikp);
    }

    stabilizerPortData getStabilizerPortData() const
    {
        const size_t num_joints = m_robot->numJoints();
        hrp::dvector joint_angles(num_joints);
        hrp::dvector joint_torques(num_joints);
        for (size_t i = 0; i < num_joints; ++i) {
            joint_angles(i)  = m_robot->joint(i)->q;
            joint_torques(i) = m_robot->joint(i)->u;
        }

        const size_t stikp_size = stikp.size();
        hrp::dvector ref_wrenches(stikp_size*6);
        for (size_t i = 0; i < stikp_size; ++i) {
            for (size_t j = 0; j < 3; j++) {
                ref_wrenches[6*i+j] = stikp[i].ref_force(j);
                ref_wrenches[6*i+j+3] = stikp[i].ref_moment(j);
            }
        }

        return stabilizerPortData{act_base_rpy, joint_angles, joint_torques, ref_zmp, new_refzmp, act_zmp, rel_act_zmp, ref_cog, act_cog,
                servo_pgains, servo_dgains, servo_tqpgains, gains_transition_times, ref_wrenches};
                // servo_pgains, servo_dgains, servo_tqpgains, servo_tqdgains, gains_transition_times, ref_wrenches};
    }

    // Setter for AutoBalanceStabilizer
    void setIfChangeServoGains(const bool if_change) { change_servo_gains = if_change; }

    // Getter for AutoBalanceStabilizer
    std::vector<bool> getActContactStates() const { return act_contact_states; }
    std::vector<hrp::Vector3> getContactCOPInfo() const  { return contact_cop_info; }
    hrp::Vector3 getOriginRefCP() const { return rel_ref_cp; }
    hrp::Vector3 getOriginActCP() const { return rel_act_cp; }
    std::pair<bool, int> getEmergencySignal() const { return std::make_pair(whether_send_emergency_signal, emergency_signal); }
    bool getIfChangeServoGains() const { return change_servo_gains; }

    // Service
    void startStabilizer();
    void stopStabilizer();
    void getStabilizerParam(OpenHRP::AutoBalanceStabilizerService::StabilizerParam& i_stp);
    void setStabilizerParam(const OpenHRP::AutoBalanceStabilizerService::StabilizerParam& i_stp);
    void setBoolSequenceParam (std::vector<bool>& st_bool_values, const OpenHRP::AutoBalanceStabilizerService::BoolSequence& output_bool_values, const std::string& prop_name);
    void setBoolSequenceParamWithCheckContact (std::vector<bool>& st_bool_values, const OpenHRP::AutoBalanceStabilizerService::BoolSequence& output_bool_values, const std::string& prop_name);
    std::string getStabilizerAlgorithmString(const OpenHRP::AutoBalanceStabilizerService::STAlgorithm _st_algorithm);

  private:
    enum CONTROL_MODE {MODE_IDLE, MODE_AIR, MODE_ST, MODE_SYNC_TO_IDLE, MODE_SYNC_TO_AIR} control_mode = MODE_IDLE;
    enum CONTACT_PHASE {LANDING_PHASE=-1, SWING_PHASE=0, SUPPORT_PHASE=1};
    OpenHRP::AutoBalanceStabilizerService::JointControlMode joint_control_mode = OpenHRP::AutoBalanceStabilizerService::JOINT_POSITION;

    void storeCurrentStates();
    void calcTargetParameters(const paramsFromAutoBalancer& abc_param);
    void calcActualParameters(const paramsFromSensors& sensor_param);
    void calcFootOriginCoords (hrp::Vector3& foot_origin_pos, hrp::Matrix33& foot_origin_rot);
    void syncToSt();
    void syncToIdle();
    /**
     * @fn
     * @return being on ground or not
     */
    bool calcZMP(hrp::Vector3& ret_zmp, const double zmp_z);
    void moveBasePosRotForBodyRPYControl ();
    void calcSwingSupportLimbGain();
    void calcTPCC();
    void calcEEForceMomentControl();
    void calcSwingEEModification ();

    // Functions to calculate robot state
    bool calcIfCOPisOutside();
    bool calcIfCPisOutside();
    bool calcFalling();
    void calcStateForEmergencySignal();

    void waitSTTransition();
    // funcitons for calc final torque output
    void calcContactMatrix (hrp::dmatrix& tm, const std::vector<hrp::Vector3>& contact_p);
    void calcTorque();
    void fixLegToCoords (const std::string& leg, const rats::coordinates& coords);
    void getFootmidCoords (rats::coordinates& ret);
    double calcDampingControl (const double tau_d, const double tau, const double prev_d,
                               const double DD, const double TT);
    hrp::Vector3 calcDampingControl (const hrp::Vector3& prev_d, const hrp::Vector3& TT);
    double calcDampingControl (const double prev_d, const double TT);
    hrp::Vector3 calcDampingControl (const hrp::Vector3& tau_d, const hrp::Vector3& tau, const hrp::Vector3& prev_d,
                                     const hrp::Vector3& DD, const hrp::Vector3& TT);
    void setSwingSupportJointServoGains();
    void calcExternalForce(const hrp::Vector3& cog, const hrp::Vector3& zmp, const hrp::Matrix33& rot);
    void calcTorque(const hrp::Matrix33& rot);

    bool isContact (const size_t idx) // 0 = right, 1 = left
    {
        return (prev_act_force_z[idx] > 25.0);
    }
    int calcMaxTransitionCount ()
    {
        return (transition_time / dt);
    }
    hrp::Vector3 calcDiffCP() const { return ref_foot_origin_rot * (ref_cp - act_cp - cp_offset); }

    // Stabilizer Parameters
    struct STIKParam {
        std::string ee_name; // Name of ee (e.g., rleg, lleg, ...)
        std::string target_name; // Name of end link
        std::string ee_base; // temporary
        std::string sensor_name; // Name of force sensor in the limb
        std::string parent_name; // Name of parent ling in the limb

        hrp::Vector3  localp      = hrp::Vector3::Zero(); // Position of ee in end link frame (^{l}p_e = R_l^T (p_e - p_l))
        hrp::Vector3  localCOPPos = hrp::Vector3::Zero(); // Position offset of reference COP in end link frame (^{l}p_{cop} = R_l^T (p_{cop} - p_l) - ^{l}p_e)
        hrp::Matrix33 localR      = hrp::Matrix33::Identity(); // Rotation of ee in end link frame (^{l}R_e = R_l^T R_e)

        // For eefm
        hrp::Vector3 d_foot_pos                  = hrp::Vector3::Zero();
        hrp::Vector3 ee_d_foot_pos               = hrp::Vector3::Zero();
        hrp::Vector3 eefm_pos_damping_gain       = hrp::Vector3(3500*10, 3500*10, 3500);
        hrp::Vector3 eefm_pos_time_const_support = hrp::Vector3(1.5, 1.5, 1.5);
        hrp::Vector3 eefm_swing_pos_spring_gain  = hrp::Vector3(0.0, 0.0, 0.0);
        hrp::Vector3 eefm_swing_pos_time_const   = hrp::Vector3(1.5, 1.5, 1.5);
        double       eefm_pos_compensation_limit = 0.025;

        hrp::Vector3 d_foot_rpy                  = hrp::Vector3::Zero();
        hrp::Vector3 ee_d_foot_rpy               = hrp::Vector3::Zero();
        hrp::Vector3 eefm_rot_damping_gain       = hrp::Vector3(20*5, 20*5, 1e5);
        hrp::Vector3 eefm_rot_time_const         = hrp::Vector3(1.5, 1.5, 1.5);
        hrp::Vector3 eefm_swing_rot_spring_gain  = hrp::Vector3(0.0, 0.0, 0.0);
        hrp::Vector3 eefm_swing_rot_time_const   = hrp::Vector3(1.5, 1.5, 1.5);
        hrp::Vector3 eefm_ee_moment_limit        = hrp::Vector3(1e4, 1e4, 1e4); // Default limit [Nm] is too large. Same as no limit.
        double       eefm_rot_compensation_limit = rad2deg(10);

        hrp::dvector6 eefm_ee_forcemoment_distribution_weight = hrp::dvector6::Zero();

        hrp::Vector3 ref_force  = hrp::Vector3::Zero();
        hrp::Vector3 ref_moment = hrp::Vector3::Zero();
        double swing_support_gain = 1.0;
        double support_time = 0.0;

        // For swing ee modification
        hrp::Vector3  target_ee_diff_p = hrp::Vector3::Zero();
        hrp::Matrix33 target_ee_diff_r = hrp::Matrix33::Identity();
        hrp::Vector3  d_pos_swing      = hrp::Vector3::Zero();
        hrp::Vector3  prev_d_pos_swing = hrp::Vector3::Zero();
        hrp::Vector3  d_rpy_swing      = hrp::Vector3::Zero();
        hrp::Vector3  prev_d_rpy_swing = hrp::Vector3::Zero();
        // TODO: unique?
        std::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3>> target_ee_diff_p_filter;
        std::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3>> target_ee_diff_r_filter;

        // IK parameter
        double avoid_gain         = 0.001;
        double reference_gain     = 0.01;
        double max_limb_length    = 0.0;
        double limb_length_margin = 0.13;
        size_t ik_loop_count      = 3;

        // For jump
        CONTACT_PHASE contact_phase = SUPPORT_PHASE;
        double phase_time = 0;
        hrp::dvector support_pgain;
        hrp::dvector support_dgain;
        hrp::dvector landing_pgain;
        hrp::dvector landing_dgain;
    };

    hrp::BodyPtr m_robot;
    std::mutex& m_mutex; // This is the reference to the mutex of AutoBalanceStabilizer class
    const std::string comp_name;
    const double dt;
    unsigned int loop = 0;
    double g_acc = 9.80665; // [m/s^2]
    double total_mass;

    // Port data for AutoBalanceStabilizer
    int emergency_signal = 0;

    // TODO: 整理
    int transition_count = 0;
    double transition_time = 2.0;
    hrp::dvector transition_joint_q, qorg, qrefv;
    // std::map<std::string, hrp::VirtualForceSensorParam> m_vfs;
    std::vector<std::shared_ptr<hrp::JointPathEx>> jpe_v;
    std::vector<STIKParam> stikp;
    std::map<std::string, size_t> contact_states_index_map;
    std::vector<bool> ref_contact_states, prev_ref_contact_states, act_contact_states, prev_act_contact_states;
    std::vector<bool> is_ik_enable, is_feedback_control_enable, is_zmp_calc_enable;
    std::vector<double> toe_heel_ratio; // TODO: delete
    int m_is_falling_counter = 0;
    std::vector<int> m_will_fall_counter;
    size_t is_air_counter = 0;
    size_t detection_count_to_mode_air = 0;

    bool is_legged_robot = false;
    bool on_ground = false;
    bool is_seq_interpolating = false; // TODO
    bool initial_cp_too_large_error = true;
    bool use_limb_stretch_avoidance = false;
    bool use_zmp_truncation = false;
    bool is_walking = false;
    bool is_estop_while_walking = false;

    hrp::Vector3 current_root_p = hrp::Vector3::Zero();
    hrp::Vector3 target_root_p  = hrp::Vector3::Zero();
    hrp::Matrix33 current_root_R, target_root_R, prev_act_foot_origin_rot, prev_ref_foot_origin_rot, target_foot_origin_rot, ref_foot_origin_rot, act_Rs;
    std::vector <hrp::Vector3> target_ee_p, rel_ee_pos, act_ee_p, projected_normal, act_force, ref_force, ref_moment;
    std::vector <hrp::Matrix33> target_ee_R, rel_ee_rot, act_ee_R;
    std::vector<std::string> rel_ee_name;
    rats::coordinates target_foot_midcoords;

    hrp::Vector3 ref_zmp      = hrp::Vector3::Zero();
    hrp::Vector3 prev_ref_zmp = hrp::Vector3::Zero();
    hrp::Vector3 act_zmp      = hrp::Vector3::Zero();
    hrp::Vector3 rel_act_zmp  = hrp::Vector3::Zero();

    hrp::Vector3 ref_cog      = hrp::Vector3::Zero();
    hrp::Vector3 prev_ref_cog = hrp::Vector3::Zero();
    hrp::Vector3 act_cog      = hrp::Vector3::Zero();
    hrp::Vector3 prev_act_cog = hrp::Vector3::Zero();

    hrp::Vector3 ref_cogvel = hrp::Vector3::Zero();
    hrp::Vector3 prev_ref_cogvel = hrp::Vector3::Zero();
    hrp::Vector3 act_cogvel = hrp::Vector3::Zero();

    hrp::Vector3 ref_cogacc = hrp::Vector3::Zero();

    hrp::Vector3 foot_origin_pos_ = hrp::Vector3::Zero();

    hrp::Vector3 ref_cp     = hrp::Vector3::Zero();
    hrp::Vector3 rel_ref_cp = hrp::Vector3::Zero();
    hrp::Vector3 act_cp     = hrp::Vector3::Zero();
    hrp::Vector3 rel_act_cp = hrp::Vector3::Zero();
    hrp::Vector3 cp_offset  = hrp::Vector3::Zero();

    hrp::Vector3 act_base_rpy = hrp::Vector3::Zero();
    hrp::Vector3 sbp_cog_offset = hrp::Vector3::Zero();

    std::array<hrp::Vector3, 2> foot_origin_offset{{hrp::Vector3::Zero(), hrp::Vector3::Zero()}}; // TODO: delete
    std::vector<double> prev_act_force_z;
    double zmp_origin_off;
    double transition_smooth_gain;
    double d_pos_z_root;
    double limb_stretch_avoidance_time_const = 1.5;
    std::array<double, 2> limb_stretch_avoidance_vlimit; // TODO: 2脚のみ
    std::array<double, 2> root_rot_compensation_limit{{deg2rad(90.0), deg2rad(90.0)}};
    std::unique_ptr<FirstOrderLowPassFilter<hrp::Vector3>> act_cogvel_filter;
    // TODO: Paramという構造体にまとめて、Serviceはここに含めない
    OpenHRP::AutoBalanceStabilizerService::STAlgorithm st_algorithm = OpenHRP::AutoBalanceStabilizerService::EEFM;
    std::unique_ptr<SimpleZMPDistributor> szd;
    std::vector<std::vector<Eigen::Vector2d>> support_polygon_vertices, margined_support_polygon_vertices;
    std::vector<hrp::Vector3> contact_cop_info;
    std::vector<hrp::dvector6> wrenches;
    std::vector<double> control_swing_support_time;

    // TPCC
    std::array<double, 2> k_tpcc_p{{0.2, 0.2}};
    std::array<double, 2> k_tpcc_x{{4.0, 4.0}};
    std::array<double, 2> k_brot_p{{0.1, 0.1}};
    std::array<double, 2> k_brot_tc{{1.5, 1.5}};
    std::array<double, 2> d_rpy{{0, 0}};

    // EEFM ST
    constexpr static double K_RATIO = 0.9;
    std::array<double, 2> eefm_k1{{-1.41429  * K_RATIO, -1.41429  * K_RATIO}};
    std::array<double, 2> eefm_k2{{-0.404082 * K_RATIO, -0.404082 * K_RATIO}};
    std::array<double, 2> eefm_k3{{-0.18     * K_RATIO, -0.18     * K_RATIO}};
    std::array<double, 2> eefm_body_attitude_control_gain{{0.5, 0.5}};
    std::array<double, 2> eefm_body_attitude_control_time_const{{1e5, 1e5}};
    std::array<double, 2> eefm_zmp_delay_time_const{{0.055, 0.055}};

    hrp::Vector3 eefm_swing_pos_damping_gain = hrp::Vector3(20 * 5, 20 * 5, 1e5);
    hrp::Vector3 eefm_swing_rot_damping_gain = hrp::Vector3(33600, 33600, 7000);

    bool eefm_use_force_difference_control = true;
    bool eefm_use_swing_damping = false;

    double eefm_pos_time_const_swing = 0.08;
    double eefm_pos_transition_time = 0.01;
    double eefm_pos_margin_time = 0.02;
    std::vector<double> eefm_swing_damping_force_thre;
    std::vector<double> eefm_swing_damping_moment_thre;

    hrp::Vector3 new_refzmp = hrp::Vector3::Zero();
    hrp::Vector3 rel_cog = hrp::Vector3::Zero();
    hrp::Vector3 ref_zmp_aux = hrp::Vector3::Zero();

    hrp::Vector3 pos_ctrl = hrp::Vector3::Zero();
    hrp::Vector3 ref_total_force = hrp::Vector3::Zero();
    hrp::Vector3 ref_total_moment = hrp::Vector3::Zero();

    // Total foot moment around the foot origin coords (relative to foot origin coords)
    hrp::Vector3 ref_total_foot_origin_moment, act_total_foot_origin_moment;
    double cop_check_margin = 20.0 * 1e-3; // [m]
    double contact_decision_threshold = 50; // [N]
    std::vector<double> cp_check_margin;
    std::vector<double> tilt_margin;

    // Emergency
    bool is_emergency = false;
    bool reset_emergency_flag = false;
    bool whether_send_emergency_signal = false; // temporary variable to send emergency signal
    OpenHRP::AutoBalanceStabilizerService::EmergencyCheckMode emergency_check_mode = OpenHRP::AutoBalanceStabilizerService::NO_CHECK;

    // Jump
    size_t jump_time_count = 0;
    double jump_initial_velocity = 0;
    double swing2landing_transition_time = 0.05;
    double landing_phase_time = 0.1;
    double landing2support_transition_time = 0.5;

    // Servo gain
    bool change_servo_gains = false;
    std::vector<double> servo_pgains;
    std::vector<double> servo_dgains;
    std::vector<double> servo_tqpgains;
    // std::vector<double> servo_tqdgains;
    std::vector<double> gains_transition_times;
};

}
#endif // ABS_STABILIZER_H
