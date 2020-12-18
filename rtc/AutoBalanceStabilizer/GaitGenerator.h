// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  GaitGenerator.h
 * @brief
 * @date  $Date$
 */

#ifndef GAITGENERATORABS_H
#define GAITGENERATORABS_H

#include <vector>
#include <memory>
#include <mutex>

#include <hrpModel/Link.h>
#include <hrpModel/Body.h>

#include "Utility.h"
#include "LinkConstraint.h"
#include "RefZMPGenerator.h"
#include "LimbTrajectoryGenerator.h"
#include "COGTrajectoryGenerator.h"

namespace hrp {

class GaitGenerator
{
  public:
    enum LocomotionMode : size_t { WALK, RUN };
    enum WalkingMode : size_t { PREVIEW_CONTROL, FOOT_GUIDED_WALK }; // TODO: COGTrajectoryGeneratorとどちらか
    enum RunningMode : size_t { FOOT_GUIDED_RUN, EXTENDED_MATRIX };

  private:
    static constexpr double DEFAULT_GRAVITATIONAL_ACCELERATION = 9.80665; // [m/s^2]

    std::mutex& m_mutex; // This is the reference to the mutex of AutoBalanceStabilizer class
    size_t loop = 0;
    const double robot_mass;
    const size_t root_id = 0;
    Eigen::Isometry3d root_coord = Eigen::Isometry3d::Identity();
    bool if_compensate_cog_moment = true;
    hrp::Vector3 cog_moment = hrp::Vector3::Zero();

    LocomotionMode locomotion_mode = WALK;
    WalkingMode walking_mode = FOOT_GUIDED_WALK;
    RunningMode running_mode = FOOT_GUIDED_RUN;

    size_t cur_const_idx = 0; // To reduce calculation time

    /// Gait parameter
    size_t default_single_support_count;
    size_t default_double_support_count;
    size_t default_toe_support_count;  // memo: toeは single supportに含まれる
    size_t default_heel_support_count; // memo: heelはdouble supportに含まれる
    double default_step_height = 0.10; // [m]
    double max_stride = 0.1; // [m]
    double max_rot_angle = deg2rad(10); // [rad]

    // Run parameter
    // double default_take_off_z = 0.85;
    double default_take_off_z = 0.96;
    // double default_jump_height = 0.005;
    // double default_jump_height = 0.03;
    double default_jump_height = 0.05;
    double default_support_count_run;

    std::vector<std::pair<hrp::Vector3, size_t>> ref_zmp_goals;

    // Walking cycle
    std::vector<int> support_limb_cycle; // TODO: std::vector<std::vector<int>>
    std::vector<int> swing_limb_cycle;
    // std::vector<std::vector<int>> support_limb_cycle;
    // std::vector<std::vector<int>> swing_limb_cycle;
    size_t current_cycle = 0;

    // Toe-heel parameter
    bool default_use_toe_heel = false;
    double toe_kick_angle = deg2rad(15); // [rad]
    double heel_contact_angle = deg2rad(15); // [rad]

    // Velocity mode parameter
    bool is_velocity_mode = false; // TODO: enum?
    double vel_x = 0;
    double vel_y = 0;
    double vel_yaw = 0;

    /// Instance
    std::vector<ConstraintsWithCount> constraints_list;
    std::unique_ptr<RefZMPGenerator> zmp_gen;
    std::unique_ptr<COGTrajectoryGenerator> cog_gen;
    // TODO: STも持っても良いかも (AutoBalancerを上ではなくする)

    hrp::Vector3 ref_zmp = hrp::Vector3::Zero();
    hrp::Vector3 prev_ref_cog = hrp::Vector3::Zero();

    // -- COGTrajectoryGenerator --
    size_t preview_window = 800;
    // -- COGTrajectoryGenerator --

    // -- LimbTrajectoryGenerator --
    LimbTrajectoryGenerator::TrajectoryType default_traj_type = LimbTrajectoryGenerator::RECTANGLE;
    // -- LimbTrajectoryGenerator --

    void addFootStepVelocityMode(const size_t cur_count);
    hrp::Vector3 calcCogMomentFromCMP(const hrp::Vector3& ref_cmp, const double total_mass, const double z_acc = 0);

  public:
    GaitGenerator(const hrp::BodyPtr& _robot,
                  std::mutex& _mutex,
                  const double _dt,
                  std::vector<LinkConstraint>&& init_constraints,
                  const double preview_time = 1.6);

    void resetGaitGenerator(const hrp::BodyPtr& _robot,
                            const size_t cur_count,
                            const double _dt);

    void setCurrentLoop(const size_t _loop) { loop = _loop; }

    void forwardTimeStep(const size_t cur_count);
    void calcCogAndLimbTrajectory(const size_t cur_count, const double dt);
    void addLinkConstraint(); // TODO

    void setConstraintsList(const decltype(constraints_list)& _constraints) { constraints_list = _constraints; }
    void setConstraintsList(decltype(constraints_list)&& _constraints) { constraints_list = _constraints; }
    void addConstraintsList(const decltype(constraints_list)& _constraints)
    {
        constraints_list.reserve(constraints_list.size() + _constraints.size());
        for (const auto& constraints : _constraints) {
            constraints_list.push_back(constraints);
        }
    }
    const ConstraintsWithCount& getCurrentConstraints(const size_t count) const
    {
        const size_t index = getConstraintIndexFromCount(constraints_list, count);
        return constraints_list[index];
    }
    const std::vector<ConstraintsWithCount>& getConstraintsList() const { return constraints_list; }
    void modifyConstraintsTarget(const size_t cur_count,
                                 const size_t cwc_idx_from_current,
                                 const size_t modif_const_idx,
                                 const Eigen::Isometry3d& modif_mat,
                                 const int modif_count, // TODO: count or time
                                 const double dt);

    Eigen::Isometry3d& rootCoord() { return root_coord; }
    const Eigen::Isometry3d& rootCoord() const { return root_coord; }
    Eigen::Isometry3d::TranslationPart rootPos() { return root_coord.translation(); }
    Eigen::Isometry3d::ConstTranslationPart rootPos() const { return root_coord.translation(); }
    Eigen::Isometry3d::LinearPart rootRot() { return root_coord.linear(); }
    Eigen::Isometry3d::ConstLinearPart rootRot() const { return root_coord.linear(); }

    const hrp::Vector3& getCogMoment() const { return cog_moment; }
    const hrp::Vector3& getRefZMP() const { return ref_zmp; }

    // Todo: Private ?
    hrp::Vector3 calcReferenceCOPFromModel(const hrp::BodyPtr& _robot, const std::vector<LinkConstraint>& cur_consts) const;
    hrp::Matrix33 calcReferenceCOPRotFromModel(const hrp::BodyPtr& _robot, const std::vector<LinkConstraint>& cur_consts) const;
    void adjustCOPCoordToTarget(const hrp::BodyPtr& _robot, const size_t count);

    // -- RefZMPGenerator --
    void setRefZMPList(const size_t cur_count, const size_t start_index = 0)
    {
        zmp_gen->setRefZMPList(constraints_list, cur_count, start_index);
    }
    // const hrp::Vector3& getCurrentRefZMP() const { return zmp_gen->getCurrentRefZMP(); }
    // -- RefZMPGenerator --

    // -- COGTrajectoryGenerator --
    void resetCOGTrajectoryGenerator(const hrp::Vector3& init_cog, const double dt)
    {
        std::cerr << "init_cog: "<< init_cog.transpose() << std::endl;
        std::cerr << "init_zmp: " << zmp_gen->getCurrentRefZMP().transpose() << std::endl;
        cog_gen.reset(new COGTrajectoryGenerator(init_cog));
        cog_gen->initPreviewController(dt, zmp_gen->getCurrentRefZMP());
    }
    const hrp::Vector3& getCog()    const { return cog_gen->getCog(); }
    const hrp::Vector3& getCogVel() const { return cog_gen->getCogVel(); }
    const hrp::Vector3& getCogAcc() const { return cog_gen->getCogAcc(); }
    hrp::Vector3 calcCP() const { return cog_gen->calcCP(); }
    void setCogCalculationType(COGTrajectoryGenerator::CogCalculationType type)
    {
        cog_gen->setCogCalculationType(type);
    }
    // -- COGTrajectoryGenerator --

    // -- LimbTrajectoryGenerator --
    void modifyLimbViaPoints(const size_t constraint_idx,
                             const Eigen::Isometry3d& new_goal,
                             const size_t cur_count,
                             const size_t new_goal_count);
    // -- LimbTrajectoryGenerator --

    void startVelocityMode(const double _vel_x, const double _vel_y, const double _vel_yaw)
    {
        vel_x = _vel_x;
        vel_y = _vel_y;
        vel_yaw = _vel_yaw;
        is_velocity_mode = true;
    }

    void stopVelocityMode()
    {
        // TODO: addNextFootStepFromVelocity(0, 0, 0, ...) // ２脚ならこれでそろうが // cycleが終わるまでやる？
        is_velocity_mode = false;
    }

    // TODO: for debug
    void addCurrentConstraintForDummy(const size_t start_count)
    {
        const size_t cur_idx = getConstraintIndexFromCount(constraints_list, start_count);
        constraints_list.reserve(constraints_list.size() + 1);

        ConstraintsWithCount dummy_constraints(constraints_list[cur_idx]);
        dummy_constraints.start_count = start_count;
        constraints_list.push_back(std::move(dummy_constraints));
    }

    std::vector<ConstraintsWithCount>
    calcFootStepConstraints(const ConstraintsWithCount& last_constraints,
                            const std::vector<size_t>& swing_indices,
                            const std::vector<Eigen::Isometry3d>& targets,
                            const size_t swing_start_count,
                            const size_t one_step_count,
                            const bool use_toe_heel = false,
                            const std::vector<size_t>& toe_support_indices = std::vector<size_t>(),
                            const size_t toe_support_count = 0,
                            const size_t heel_support_count = 0);

    std::vector<ConstraintsWithCount>
    calcFootStepConstraintsForRun(const ConstraintsWithCount& last_constraints,
                                  const std::vector<size_t>& jump_indices,
                                  const std::vector<size_t>& land_indices,
                                  const std::vector<Eigen::Isometry3d>& targets,
                                  const size_t jump_start_count,
                                  const size_t jumping_count,
                                  const bool is_start = false,
                                  const size_t starting_count = 0);
    std::vector<ConstraintsWithCount>
    calcFootStepConstraintsForJump(const ConstraintsWithCount& last_constraints,
                                   const std::vector<Eigen::Isometry3d>& targets,
                                   const size_t jump_start_count,
                                   const size_t jumping_count);

    void addFootStep(const ConstraintsWithCount& last_constraints,
                     const std::vector<size_t>& swing_indices,
                     const std::vector<Eigen::Isometry3d>& targets,
                     const size_t swing_start_count,
                     const size_t one_step_count,
                     const bool use_toe_heel = false,
                     const std::vector<size_t>& toe_support_indices = std::vector<size_t>(),
                     const size_t toe_support_count = 0,
                     const size_t heel_support_count = 0);

    void addFootStep(const std::vector<int>& link_ids, const std::vector<Eigen::Isometry3d>& targets,
                     const size_t swing_start_count, const size_t one_step_count);

    void addFootStepFromVelocity(const double vel_x, const double vel_y, const double vel_yaw,
                                 const size_t swing_start_count, const double step_time, const double dt,
                                 const double max_step_length, const double max_rotate_angle /*[rad]*/,
                                 const int support_id, const int swing_id);


    /* Service below */
    std::vector<int> getConstraintLinkIds();
    void setDefaultSingleSupportTime(const double time, const double dt);
    void setDefaultDoubleSupportTime(const double time, const double dt);
    void setDefaultToeSupportTime(const double time, const double dt);
    void setDefaultHeelSupportTime(const double time, const double dt);
    void setDefaultStepHeight(const double height)   { default_step_height = height; }
    void setMaxStride(const double stride)           { max_stride = stride; }
    void setMaxRotAngle(const double angle_rad)      { max_rot_angle = angle_rad; }

    void setUseToeHeel(const bool use_toe_heel)      { default_use_toe_heel = use_toe_heel; }
    void setToeKickAngle(const double angle_rad)     { toe_kick_angle = angle_rad; }
    void setHeelContactAngle(const double angle_rad) { heel_contact_angle = angle_rad; }
    bool setToeContactPoints(const int link_id, const std::vector<hrp::Vector3>& contact_points);
    bool setHeelContactPoints(const int link_id, const std::vector<hrp::Vector3>& contact_points);
    // void setGaitGeneratorParam(); // TODO: 作るかも

    void setWalkingMode(const WalkingMode mode)      { walking_mode = mode; }

    bool goPos(const Eigen::Isometry3d& target,
               const std::vector<int>& support_link_cycle,
               const std::vector<int>& swing_link_cycle);
    bool setFootSteps(const std::vector<int>& support_link_cycle,
                      const std::vector<int>& swing_link_cycle,
                      hrp::Vector3 footsteps_pos[],
                      Eigen::Quaterniond footsteps_rot[],
                      int fs_side[],
                      int length);
    bool setRunningFootSteps(hrp::Vector3 footsteps_pos[],
                             Eigen::Quaterniond footsteps_rot[],
                             int fs_side[],
                             int length,
                             const double dt,
                             const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION);
    bool setJumpingFootSteps(const double dt, std::vector<std::vector<hrp::Vector3> > footsteps_pos,
                             std::vector<std::vector<Eigen::Quaterniond> > footsteps_rot,
                             std::vector<std::vector<int> > fs_side,
                             const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION);
    bool startRunning(const double dt, const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION);
    bool startJumping(const double dt, const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION);
    bool startRunJumpDemo(const double dt, const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION);

    // gopos: 接触のCycleを記述したい
    // void goPos(const rats::coordinates& target, const size_t one_step_count,
    //            const double max_step_length, const double max_rotate_angle,
    //            const std::vector<std::vector<int>>& support_link_cycle,
    //            const std::vector<std::vector<int>>& swing_link_cycle);
};

}

#endif // GAITGENERATORABS_H
