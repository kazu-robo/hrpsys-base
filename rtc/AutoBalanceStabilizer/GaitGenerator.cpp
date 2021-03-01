// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  GaitGenerator.cpp
 * @brief
 * @date  $Date$
 */

#include <boost/make_shared.hpp>
#include <tuple>
#include "GaitGenerator.h"
#include "Utility.h"
#include "EigenUtil.h"
#include "PlaneGeometry.h"

namespace hrp {

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )

GaitGenerator::GaitGenerator(const hrp::BodyPtr& _robot,
                             std::mutex& _mutex,
                             const double _dt,
                             std::vector<LinkConstraint>&& init_constraints,
                             const double preview_time) :
    m_mutex(_mutex),
    robot_mass(_robot->totalMass()),
    root_id(_robot->rootLink()->index),
    preview_window(static_cast<size_t>(std::round(preview_time / _dt))),
    default_single_support_count(static_cast<size_t>(1.0 / _dt)),
    default_double_support_count(static_cast<size_t>(0.25 / _dt)),
    default_toe_support_count(static_cast<size_t>(0.5 / _dt)),
    default_heel_support_count(static_cast<size_t>(0.15 / _dt)),
    // default_support_count_run(static_cast<size_t>(0.85 / _dt))
    default_support_count_run(static_cast<size_t>(0.65 / _dt))
    // default_support_count_run(static_cast<size_t>(0.235 / _dt))
    // default_support_count_run(static_cast<size_t>(0.335 / _dt))
    // default_support_count_run(static_cast<size_t>(0.285 / _dt))
{
    const size_t num_limbs = init_constraints.size();

    // Set initial position as target (need to be done calcForwardKinematics)
    // TODO: ここで良いか
    for (auto& constraint : init_constraints) {
        const hrp::Link* const link = _robot->link(constraint.getLinkId());
        constraint.targetPos() = constraint.calcActualTargetPosFromLinkState(link->p, link->R);
        // std::cerr << "link   " << link->name << " pos: " << link->p.transpose() << std::endl;
        // std::cerr << "target " << link->name << " pos: " << constraint.targetPos().transpose() << std::endl;
        // std::cerr << "local  " << link->name << " pos: " << constraint.localPos().transpose() << std::endl;
        constraint.targetRot() = constraint.calcActualTargetRotFromLinkState(link->R);
    }

    constraints_list.resize(1);
    constraints_list[0].constraints = std::move(init_constraints); // TODO: これでmoveできるかテストする
    constraints_list[0].start_count = 0;

    root_coord.translation() = _robot->rootLink()->p;
    root_coord.linear() = _robot->rootLink()->R;

    prev_ref_cog = _robot->calcCM();
    zmp_gen = std::make_unique<RefZMPGenerator>(_dt, preview_window, constraints_list[0]);
    cog_gen = std::make_unique<COGTrajectoryGenerator>(prev_ref_cog, zmp_gen->getCurrentRefZMP());

    cog_gen->initPreviewController(_dt, zmp_gen->getCurrentRefZMP());
    ref_zmp = zmp_gen->getCurrentRefZMP();
    ref_zmp_goals.push_back(std::make_pair(constraints_list[0].calcCOPFromConstraints(), constraints_list[0].start_count));
}

// TODO: cur_count か メンバ変数のloopかどちらかにする
void GaitGenerator::resetGaitGenerator(const hrp::BodyPtr& _robot,
                                       const size_t cur_count,
                                       const double _dt)
{
    // TODO: ここで中心を0にするようにtargetCoordsを移動するべき？
    //       start autobalancerをしたところ基準で考えたい気持ち
    //       いずれにせよodom resetみたいなのは必要

    constraints_list[0].constraints = constraints_list.back().constraints;
    constraints_list[0].start_count = cur_count;
    constraints_list.resize(1);
    cur_const_idx = 0;

    root_coord.translation() = _robot->rootLink()->p;
    root_coord.linear() = _robot->rootLink()->R;

    prev_ref_cog = _robot->calcCM();

    zmp_gen.reset(new RefZMPGenerator(_dt, preview_window, constraints_list[0]));
    resetCOGTrajectoryGenerator(prev_ref_cog, _dt);
    ref_zmp = zmp_gen->getCurrentRefZMP();
    ref_zmp_goals.push_back(std::make_pair(constraints_list[0].calcCOPFromConstraints(), constraints_list[0].start_count));
}

void GaitGenerator::forwardTimeStep(const size_t cur_count)
{
    zmp_gen->popAndPushRefZMP(constraints_list, cur_count);
    if (locomotion_mode != RUN) {
        ref_zmp = zmp_gen->getCurrentRefZMP();
    }

    cur_const_idx = getConstraintIndexFromCount(constraints_list, cur_count);

    if (cur_count == constraints_list[cur_const_idx].start_count &&
        constraints_list[cur_const_idx].is_stable) {
        ref_zmp_goals = zmp_gen->calcZMPGoalsFromConstraints(constraints_list, cur_const_idx, ref_zmp_goals.back().first, constraints_list[cur_const_idx].start_count);
    }

    ConstraintsWithCount& cur_cwc = constraints_list[cur_const_idx];
    if (cur_cwc.start_count != cur_count || cur_const_idx == 0) return;

    // if (cur_cwc.start_count == cur_count && cur_const_idx > 0) {
    // TODO: 最初のStateを変更しておかないと, ToeHeelの切り替え
    cur_cwc.copyLimbState(constraints_list[getPrevValidConstraintIndex(constraints_list, cur_const_idx)]);

    for (size_t idx = 0; idx < cur_cwc.constraints.size(); ++idx) {
        LinkConstraint& cur_const = cur_cwc.constraints[idx];
        if (cur_const.getConstraintType() == LinkConstraint::FIX ||
            cur_const.getConstraintType() == LinkConstraint::FREE ||
            cur_const.isLimbInterpolating(cur_count) // すでに跳躍時の遊脚
            ) continue;

        size_t goal_idx = cur_const_idx + 1;
        while (goal_idx < constraints_list.size()) {
            if (constraints_list[goal_idx].constraints[idx].getConstraintType() != LinkConstraint::FLOAT) break;
            ++goal_idx;
        }
        if (goal_idx == constraints_list.size()) continue;

        const ConstraintsWithCount goal_cwc = constraints_list[goal_idx];
        const Eigen::Isometry3d target_coord = goal_cwc.constraints[idx].targetCoord() * goal_cwc.constraints[idx].localCoord().inverse() * cur_const.localCoord(); // TODO: 確認

        if (cur_const.getConstraintType() == LinkConstraint::FLOAT) {
            cur_const.calcLimbViaPoints(default_traj_type,
                                        target_coord,
                                        cur_cwc.start_count,
                                        goal_cwc.start_count,
                                        default_step_height);
        } else if (cur_const.getConstraintType() == LinkConstraint::SLIDE) {
            std::cerr << "[GaitGenerator] SLIDE is not implemented yet." << std::endl;
        } else if (cur_const.getConstraintType() == LinkConstraint::ROTATE) {
            cur_const.calcLimbViaPoints(LimbTrajectoryGenerator::LINEAR,
                                        target_coord,
                                        cur_cwc.start_count,
                                        goal_cwc.start_count,
                                        0);
        }
    }
    // }
}

void GaitGenerator::calcCogAndLimbTrajectory(const size_t cur_count, const double dt)
{
    bool next_flight = false;
    if (cur_const_idx < constraints_list.size() - 1) {
        next_flight = constraints_list[cur_const_idx + 1].isFlightPhase();
    }

    bool last_flight = false;
    if (cur_const_idx > 0) {
        last_flight = constraints_list[cur_const_idx - 1].isFlightPhase();
    }

    // const std::vector<size_t> sup_indices = constraints_list[cur_const_idx].getConstraintIndicesFromType(LinkConstraint::FIX); // TODO: 平足
    const bool is_flight = constraints_list[cur_const_idx].isFlightPhase();
    // Update ref zmp TODO: わかりづらい
    // if (is_flight) ref_zmp = cog_gen->calcCogForFlightPhase(dt);
    if (is_flight) {
        ref_zmp = cog_gen->calcCogForFlightPhase(dt);
        if (DEBUGP) std::cerr << "[GaitGenerator] is_flight" << std::endl;
    }
    else if (!next_flight && !last_flight) {
        // if (walking_mode == PREVIEW_CONTROL) cog_gen->calcCogFromZMP(zmp_gen->getRefZMPList(), dt);
        if (walking_mode == PREVIEW_CONTROL) {
            cog_gen->calcCogFromZMP(zmp_gen->getRefZMPList(), dt);
            if (DEBUGP) std::cerr << "[GaitGenerator] preview_control" << std::endl;
        }
        else if (walking_mode == FOOT_GUIDED_WALK) {
            if (DEBUGP) std::cerr << "[GaitGenerator] foot_guided_walk" << std::endl;

            ref_zmp = cog_gen->calcFootGuidedCogWalk(constraints_list, ref_zmp_goals, cur_const_idx, cur_count, dt);
        }
    } else {
        const size_t count_to_jump = constraints_list[cur_const_idx + 1].start_count - cur_count;
        if (running_mode == FOOT_GUIDED_RUN) {
            if (DEBUGP) std::cerr << "[GaitGenerator] foot_guided_run" << std::endl;

            // const auto support_point = constraints_list[cur_const_idx].calcCOPFromConstraints();
            // const auto landing_point = constraints_list[cur_const_idx + 2].calcCOPFromConstraints();
            // const size_t supporting_count = constraints_list[cur_const_idx + 2].start_count - constraints_list[cur_const_idx + 1].start_count;
            ref_zmp = cog_gen->calcFootGuidedCog(constraints_list, default_jump_height, cur_const_idx, cur_count, dt);
        } else if (running_mode == EXTENDED_MATRIX) {
            if (DEBUGP) std::cerr << "[GaitGenerator] extended_matrix" << std::endl;

            if (cur_count == constraints_list[cur_const_idx].start_count) {
                const auto support_point = constraints_list[cur_const_idx].calcCOPFromConstraints();
                ref_zmp = support_point;
                // const double y_offset = 0.015;
                const double y_offset = 0.0;
                ref_zmp[1] += (support_point[1] < 0) ? y_offset : -y_offset; // TODO: Body相対か何かを使う

                std::cerr << "cur  cp: " << cog_gen->calcCP().transpose() << std::endl;
                std::cerr << "diff cp: " << (cog_gen->calcCP() - support_point).transpose() << std::endl;

                if (cur_const_idx < constraints_list.size() - 6) {
                    const auto next_landing_point = constraints_list[cur_const_idx + 2].calcCOPFromConstraints();
                    const auto last_landing_point = constraints_list[cur_const_idx + 4].calcCOPFromConstraints();

                    hrp::Vector3 next_ref_zmp = next_landing_point;
                    next_ref_zmp[1] += (next_landing_point[1] < 0) ? y_offset : -y_offset;

                    hrp::Vector3 last_ref_zmp = last_landing_point;
                    last_ref_zmp[1] += (last_landing_point[1] < 0) ? y_offset : -y_offset;
                    hrp::Vector3 target_cp = next_landing_point;
                    // target_cp[0] += (last_landing_point[0] - next_landing_point[0] > 0) ? 0.1 : 0.0;
                    // target_cp[1] += (landing_point[1] - support_point[1] > 0) ? -0.05 : 0.05;
                    // target_cp_offset[0] = (one_step[0] + start_zmp_offset[0]) + (one_step[0] + start_zmp_offset[0] - end_zmp_offset[0]) / (omega * flight_time) - one_step[0];

                    const size_t count_to_jump2 = constraints_list[cur_const_idx + 3].start_count - constraints_list[cur_const_idx + 2].start_count;

                    // Memo:  [ms] 程度
                    cog_gen->calcCogListForRun2Step(target_cp, ref_zmp, next_ref_zmp, last_ref_zmp,
                                                    count_to_jump, count_to_jump2, cur_count,
                                                    default_jump_height, default_jump_height,
                                                    default_take_off_z, default_take_off_z, dt);
                } else {
                    const auto landing_point = constraints_list[cur_const_idx + 2].calcCOPFromConstraints();

                    hrp::Vector3 next_ref_zmp = landing_point;
                    next_ref_zmp[1] += (landing_point[1] < 0) ? y_offset : -y_offset;

                    hrp::Vector3 target_cp = landing_point;
                    // target_cp[1] += (landing_point[1] - support_point[1] > 0) ? -0.05 : 0.05;
                    // target_cp_offset[0] = (one_step[0] + start_zmp_offset[0]) + (one_step[0] + start_zmp_offset[0] - end_zmp_offset[0]) / (omega * flight_time) - one_step[0];

                    // Memo: 0.03608 [ms] 程度
                    cog_gen->calcCogListForRun(target_cp, ref_zmp, next_ref_zmp, count_to_jump, cur_count,
                                               default_jump_height, default_take_off_z, dt);
                }
            }

            cog_gen->getCogFromCogList(cur_count, dt);
            ref_zmp = cog_gen->calcPointMassZMP();
        }
    }

    cog_moment = (if_compensate_cog_moment) ?
        calcCogMomentFromCMP(ref_zmp, robot_mass, cog_gen->getCogAcc()[2]) : hrp::Vector3::Zero();
    // std::cerr << "cog_moment: " << cog_moment.transpose() << std::endl;

    constraints_list[cur_const_idx].calcLimbTrajectory(cur_count, dt); // TODO: 跳躍直後の足を下す途中で一度着地してしまう

    root_coord.translation() += cog_gen->getCog() - prev_ref_cog;
    // TODO: 手が追加された時やその他の時にも対応できるように
    root_coord.linear() = constraints_list[cur_const_idx].calcCOPRotationFromConstraints(LinkConstraint::FREE); // TODO: toe-heelで回る
    prev_ref_cog = cog_gen->getCog();

    return;
}

hrp::Vector3 GaitGenerator::calcCogMomentFromCMP(const hrp::Vector3& ref_cmp, const double total_mass, const double z_acc)
{
    // TODO: 平面仮定してる
    std::vector<Eigen::Vector2d> convex_hull = constraints_list[cur_const_idx].calcContactConvexHullForAllConstraints();
    const size_t num_verts = convex_hull.size();

    if (num_verts == 0) return hrp::Vector3::Zero();

    const Eigen::Vector2d inner_p = (convex_hull[0] + convex_hull[num_verts / 3] + convex_hull[2 * num_verts / 3]) / 3.0;
    std::pair<size_t, size_t> closest_verts;
    if (isInsideConvexHull(convex_hull, ref_cmp.head<2>(), inner_p, closest_verts.first, closest_verts.second)) return hrp::Vector3::Zero();

    // TODO: これが最速かどうか，やり方を整理する
    Eigen::Vector2d closest_point = ref_cmp.head<2>();
    calcClosestBoundaryPoint(closest_point, convex_hull, closest_verts.first, closest_verts.second);
    const double f_z = (z_acc + DEFAULT_GRAVITATIONAL_ACCELERATION) * total_mass;
    std::cerr << "f_z: " << f_z << std::endl;
    return hrp::Vector3(-(ref_cmp[1] - closest_point[1]), ref_cmp[0] - closest_point[0], 0) * f_z;
}

// hrp::Vector3 GaitGenerator::calcCogMomentFromCMP(const hrp::Vector3& ref_cmp, const double total_mass, const double z_acc)
// {
//     // TODO: 平面仮定してる
//     std::vector<Eigen::Vector2d> convex_hull = constraints_list[cur_const_idx].calcContactConvexHullForAllConstraints();
//     const size_t num_verts = convex_hull.size();

//     if (num_verts == 0) return hrp::Vector3::Zero();

//     const Eigen::Vector2d inner_p = (convex_hull[0] + convex_hull[num_verts / 3] + convex_hull[2 * num_verts / 3]) / 3.0;
//     std::pair<size_t, size_t> closest_verts;
//     if (isInsideConvexHull(convex_hull, ref_cmp.head<2>(), inner_p, closest_verts.first, closest_verts.second)) return hrp::Vector3::Zero();

//     // TODO: これが最速かどうか，やり方を整理する
//     Eigen::Vector2d closest_point = ref_cmp.head<2>();
//     calcClosestBoundaryPoint(closest_point, convex_hull, closest_verts.first, closest_verts.second);
//     const double f_z = (z_acc + DEFAULT_GRAVITATIONAL_ACCELERATION) * total_mass;
//     return hrp::Vector3(-(ref_cmp[1] - closest_point[1]), ref_cmp[0] - closest_point[0], 0) * f_z;
// }

bool GaitGenerator::getSupportSwingIndex(int& support_idx, int& swing_idx, const ConstraintsWithCount& constraints, const size_t _cur_cycle, const std::vector<int>& support_link_cycle, const std::vector<int>& swing_link_cycle)
{
    support_idx = constraints.getConstraintIndexFromLinkId(support_link_cycle[_cur_cycle]);
    swing_idx   = constraints.getConstraintIndexFromLinkId(swing_link_cycle[_cur_cycle]);
    if (support_idx == -1 || swing_idx == -1) {
        std::cerr << "[GaitGenerator] Error" << std::endl; // TODO: error message
        return false;
    }
    return true;
}

void GaitGenerator::addNewFootSteps(std::vector<ConstraintsWithCount>& new_constraints, const ConstraintsWithCount& last_constraints, const size_t swing_idx, const size_t support_idx, const Eigen::Isometry3d& landing_target, const bool use_toe_heel)
{
    // Take longer time for first moving
    const size_t swing_start_count = last_constraints.start_count + default_double_support_count; // only double support count
    const std::vector<size_t> swing_indices{swing_idx};
    const std::vector<Eigen::Isometry3d> landing_targets{landing_target};
    const std::vector<size_t> toe_support_indices{support_idx};

    const std::vector<ConstraintsWithCount> footstep_constraints =
        calcFootStepConstraints(last_constraints, swing_indices, landing_targets,
                                swing_start_count, default_single_support_count,
                                use_toe_heel, toe_support_indices,
                                default_toe_support_count, default_heel_support_count);

    for (const auto& footstep : footstep_constraints) {
        new_constraints.push_back(footstep);
    }
}

void GaitGenerator::addNewRunningFootSteps(std::vector<ConstraintsWithCount>& new_constraints, const ConstraintsWithCount& last_constraints, const size_t jump_idx, const size_t land_idx, const Eigen::Isometry3d& landing_target, const size_t flight_phase_count, const bool is_start, const bool is_end)
{
    // Take longer time for first moving
    const size_t jump_start_count = last_constraints.start_count + default_support_count_run + (is_start ? default_double_support_count : 0);
    const size_t swing_start_count = last_constraints.start_count + default_double_support_count;
    const std::vector<size_t> jump_indices{jump_idx};
    const std::vector<size_t> land_indices{land_idx};
    const std::vector<Eigen::Isometry3d> landing_targets{landing_target};

    const std::vector<ConstraintsWithCount> footstep_constraints =
        calcFootStepConstraintsForRun(last_constraints, jump_indices, land_indices, landing_targets,
                                      jump_start_count, flight_phase_count, is_start, swing_start_count, is_end);

    for (const auto& footstep : footstep_constraints) {
        new_constraints.push_back(footstep);
    }
}

void GaitGenerator::finalizeFootSteps(std::vector<ConstraintsWithCount>& new_constraints)
{
    new_constraints.push_back(new_constraints.back());
    new_constraints.back().start_count += default_double_support_count;
    new_constraints.back().is_stable = true;

    // Update constraints_list
    std::lock_guard<std::mutex> lock(m_mutex);
    const size_t diff_count = loop - new_constraints[0].start_count + 1;
    for (auto& constraints : new_constraints) {
        constraints.start_count += diff_count;
    }
    setConstraintsList(std::move(new_constraints));
    setRefZMPList(loop);
}

void GaitGenerator::addFirstTwoConstraints(std::vector<ConstraintsWithCount>& new_constraints, const ConstraintsWithCount& cur_constraints)
{
    for (size_t i = 0; i < 2; ++i) {
        new_constraints.push_back(cur_constraints);
        ConstraintsWithCount& added_constraints = new_constraints.back();
        if (i == 1) { // 二足歩行の場合は最初の両足支持期に相当
            added_constraints.start_count = cur_constraints.start_count + default_single_support_count;
            added_constraints.is_stable = false;
        }
    }
}

void GaitGenerator::modifyConstraintsTarget(const size_t cur_count,
                                            const size_t cwc_idx_from_current,
                                            const size_t modif_const_idx,
                                            const Eigen::Isometry3d& modif_mat,
                                            const int modif_count,
                                            const double dt)
{
    size_t modif_cwc_idx = cur_const_idx + cwc_idx_from_current;

    // TODO: 例えば４足歩行だと，
    //       1. 今の遊脚のターゲットをかえる
    //       2. 次の遊脚のターゲットをかえ，他はそのまま...というふうになり，全てを変更したらそれ以降は変更してても良い
    // [0] float fix float fix
    //           変更
    // [1] fix float fix float
    //               変更

    const size_t cwc_size = constraints_list.size();
    while (modif_cwc_idx < cwc_size &&
           constraints_list[modif_cwc_idx].constraints[modif_const_idx].getConstraintType() > LinkConstraint::ROTATE) {
        ++modif_cwc_idx;
    }
    if (modif_cwc_idx == cwc_size) return;

    for (size_t cwc_idx = modif_cwc_idx; cwc_idx < cwc_size; ++cwc_idx) {
        constraints_list[cwc_idx].constraints[modif_const_idx].targetCoord() = modif_mat * constraints_list[cwc_idx].constraints[modif_const_idx].targetCoord();
        constraints_list[cwc_idx].start_count += modif_count;
    }

    // TODO: cur_const_idxがFLOATなら良いが，直接着地のidxを指定しているとこれではだめ
    if (constraints_list[cur_const_idx].constraints[modif_const_idx].getConstraintType() > LinkConstraint::ROTATE) {
        // TODO: regenerate
        const LinkConstraint& landing_constraint = constraints_list[modif_cwc_idx].constraints[modif_const_idx];
        const Eigen::Isometry3d target_coord = landing_constraint.targetCoord() * landing_constraint.localCoord().inverse() * constraints_list[cur_const_idx].constraints[modif_const_idx].localCoord(); // TODO: forwardTimeStepにも同じのがある．関数化

        constraints_list[cur_const_idx].constraints[modif_const_idx]
            .modifyLimbViaPoints(target_coord,
                                 cur_count,
                                 constraints_list[modif_cwc_idx].start_count, // TODO: goal count及びその後のずらし,
                                 dt);
    }

    const size_t const_size = constraints_list[modif_cwc_idx].constraints.size();
    for (size_t i = 0; i < const_size; ++i) {
        if (i == modif_const_idx) continue;

        size_t cur_modif_cwc_idx = modif_cwc_idx;
        while (cur_modif_cwc_idx < cwc_size &&
               constraints_list[cur_modif_cwc_idx].constraints[i].getConstraintType() <= LinkConstraint::ROTATE) {
            ++cur_modif_cwc_idx;
        }
        while (cur_modif_cwc_idx < cwc_size &&
               constraints_list[cur_modif_cwc_idx].constraints[i].getConstraintType() > LinkConstraint::ROTATE) {
            ++cur_modif_cwc_idx;
        }

        for (size_t j = cur_modif_cwc_idx; j < cwc_size; ++j) {
            constraints_list[j].constraints[i].targetCoord() = modif_mat * constraints_list[j].constraints[i].targetCoord();
        }
    }

    setRefZMPList(cur_count);
    // setRefZMPList(cur_count, constraints_list[cur_const_idx + cwc_idx_from_current].start_count - cur_count); // TODO
}

// This function is almost the same as ConstraintsWithCount.calcCOPFromConstraints()
hrp::Vector3 GaitGenerator::calcReferenceCOPFromModel(const hrp::BodyPtr& _robot, const std::vector<LinkConstraint>& cur_consts) const
{
    hrp::Vector3 cop_pos = hrp::Vector3::Zero();
    double sum_weight = 0;

    for (const LinkConstraint& constraint : cur_consts) {
        if (constraint.getConstraintType() >= LinkConstraint::FLOAT) continue;
        const double weight = constraint.getCOPWeight();
        const hrp::Link* const link = _robot->link(constraint.getLinkId());
        cop_pos += constraint.calcActualTargetPosFromLinkState(link->p, link->R) * weight;
        sum_weight += weight;
    }
    if (sum_weight > 0) cop_pos /= sum_weight;

    return cop_pos;
}

// This function is almost the same as ConstraintsWithCount.calcCOPRotationFromConstraints()
hrp::Matrix33 GaitGenerator::calcReferenceCOPRotFromModel(const hrp::BodyPtr& _robot, const std::vector<LinkConstraint>& cur_consts) const
{
    Eigen::Quaternion<double> cop_quat = Eigen::Quaternion<double>::Identity();
    double sum_weight = 0;

    for (const LinkConstraint& constraint : cur_consts) {
        const double weight = constraint.getCOPWeight();
        if (constraint.getConstraintType() >= LinkConstraint::FLOAT || weight == 0 /* to avoid zero division */) continue;
        sum_weight += weight;
        const hrp::Link* const link = _robot->link(constraint.getLinkId());
        const Eigen::Quaternion<double> contact_quat(constraint.calcActualTargetRotFromLinkState(link->R));
        cop_quat = cop_quat.slerp(weight / sum_weight, contact_quat);
    }

    return cop_quat.toRotationMatrix();
}

void GaitGenerator::adjustCOPCoordToTarget(const hrp::BodyPtr& _robot, const size_t count)
{
    const ConstraintsWithCount& cur_consts = getCurrentConstraints(count);
    const hrp::Vector3 cop_pos = calcReferenceCOPFromModel(_robot, cur_consts.constraints);
    const hrp::Matrix33 cop_rot = calcReferenceCOPRotFromModel(_robot, cur_consts.constraints);

    const hrp::Vector3 target_cop_pos = cur_consts.calcCOPFromConstraints();
    const hrp::Matrix33 target_cop_rot = cur_consts.calcCOPRotationFromConstraints();

    rootPos() = (target_cop_pos - cop_pos) + _robot->rootLink()->p;
    // rootRot() = target_cop_rot * cop_rot.transpose() * _robot->rootLink()->R;
    // TODO: rollとpitchの修正を0にする．これで良いのか
    hrp::Matrix33 diff_rot = target_cop_rot * cop_rot.transpose();
    hrp::Vector3 diff_rpy = hrp::rpyFromRot(diff_rot);
    diff_rpy[0] = diff_rpy[1] = 0.0;
    diff_rot = hrp::rotFromRpy(diff_rpy);
    rootRot() = diff_rot * _robot->rootLink()->R;
}

std::vector<ConstraintsWithCount>
GaitGenerator::calcFootStepConstraints(const ConstraintsWithCount& last_constraints,
                                       const std::vector<size_t>& swing_indices,
                                       const std::vector<Eigen::Isometry3d>& targets,
                                       const size_t swing_start_count,
                                       const size_t one_step_count,
                                       const bool use_toe_heel,
                                       const std::vector<size_t>& toe_support_indices,
                                       const size_t toe_support_count,
                                       const size_t heel_support_count)
{
    // TODO: Toe-heel
    //       float -> support toe -> landing heel -> landing plane -> float ...
    //       landingのheelがsupportのtoeより後ろの場合，toe-heelの意味ない? <- 平地はそうっぽいが，階段とか
    //       最初と最後だけ少し違う
    //       つま先関節の有無で追加の仕方が違いそう
    //       XやZ方向のthresholdでtoeのON/OFFを切り替える？
    //       最初のswing toeの時，重み0かも

    std::vector<ConstraintsWithCount> footstep_constraints;
    {
        const size_t num_constraints = use_toe_heel ? 4 : 2;
        footstep_constraints.reserve(num_constraints);
        footstep_constraints.push_back(last_constraints);
    }

    const size_t landing_count = swing_start_count + one_step_count;

    // TODO: 足上げた状態を最初にする？そうするとis_stableを追加するのが最後になって良い感じだが
    ConstraintsWithCount& swing_phase_constraints = footstep_constraints.back();
    {
        swing_phase_constraints.start_count = swing_start_count;
        swing_phase_constraints.clearLimbViaPoints();
        swing_phase_constraints.is_stable = true;
        for (const size_t swing_idx : swing_indices) {
            swing_phase_constraints.constraints[swing_idx].changeDefaultContacts();
            swing_phase_constraints.constraints[swing_idx].setConstraintType(LinkConstraint::FLOAT);
        }
    }

    if (use_toe_heel) {
        const size_t toe_start_count  = swing_start_count + (one_step_count - heel_support_count - toe_support_count);
        const size_t heel_start_count = toe_start_count + toe_support_count;

        // TODO BUG: Toe-Heel両方追加するとZMPが1制御周期で跳ぶ
        if (!toe_support_indices.empty()) {
            footstep_constraints.push_back(footstep_constraints.back());
            ConstraintsWithCount& toe_phase_constraints = footstep_constraints.back();

            toe_phase_constraints.start_count = toe_start_count;
            toe_phase_constraints.clearLimbViaPoints();
            toe_phase_constraints.is_stable = false;
            for (const size_t support_idx : toe_support_indices) {
                LinkConstraint& toe_constraint = toe_phase_constraints.constraints[support_idx];
                if (!toe_constraint.hasToeHeelContacts()) continue;
                toe_constraint.changeToeContacts();
                toe_constraint.setConstraintType(LinkConstraint::ROTATE);
            }
        }

        {
            footstep_constraints.push_back(footstep_constraints.back());
            ConstraintsWithCount& heel_phase_constraints = footstep_constraints.back();

            heel_phase_constraints.start_count = heel_start_count;
            heel_phase_constraints.clearLimbViaPoints();
            heel_phase_constraints.is_stable = false;
            // TODO: これはtoeに依存しているので，heelがなくても動くように ?
            for (const size_t support_idx : toe_support_indices) {
                heel_phase_constraints.constraints[support_idx].targetRot() = Eigen::AngleAxisd(toe_kick_angle, Eigen::Vector3d::UnitY()).toRotationMatrix() * heel_phase_constraints.constraints[support_idx].targetRot();
                heel_phase_constraints.constraints[support_idx].setConstraintType(LinkConstraint::FIX);
            }

            const size_t swing_indices_size = swing_indices.size();
            for (size_t i = 0; i < swing_indices_size; ++i) {
                LinkConstraint& heel_constraint = heel_phase_constraints.constraints[swing_indices[i]];
                if (!heel_constraint.hasToeHeelContacts()) continue;

                // TODO: heel contactの座標で計算して，changeDefaultContactsで良さそう？ <- どのconstraintでやるのか
                // const Eigen::Isometry3d heel_rotated_target =
                //     heel_constraint.calcRotatedTargetAroundHeel(targets[i], Eigen::AngleAxisd(-heel_contact_angle, Eigen::Vector3d::UnitY()));

                heel_constraint.changeDefaultContacts();
                heel_constraint.targetCoord() = targets[i];
                heel_constraint.changeHeelContacts();
                heel_constraint.targetRot() = heel_constraint.targetRot() * Eigen::AngleAxisd(-heel_contact_angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
                heel_constraint.setConstraintType(LinkConstraint::ROTATE);
            }
        }
    }

    {
        footstep_constraints.push_back(footstep_constraints.back());
        ConstraintsWithCount& landing_phase_constraints = footstep_constraints.back();
        landing_phase_constraints.start_count = landing_count;
        landing_phase_constraints.clearLimbViaPoints();
        landing_phase_constraints.is_stable = false;

        const size_t swing_indices_size = swing_indices.size();
        for (size_t i = 0; i < swing_indices_size; ++i) {
            LinkConstraint& landing_constraint = landing_phase_constraints.constraints[swing_indices[i]];
            landing_constraint.changeDefaultContacts();
            landing_constraint.targetCoord() = targets[i];
            landing_constraint.setConstraintType(LinkConstraint::FIX);
        }
    }

    return footstep_constraints;
}

std::vector<ConstraintsWithCount>
GaitGenerator::calcFootStepConstraintsForRun(const ConstraintsWithCount& last_constraints,
                                             const std::vector<size_t>& jump_indices,
                                             const std::vector<size_t>& land_indices,
                                             const std::vector<Eigen::Isometry3d>& targets,
                                             const size_t jump_start_count,
                                             const size_t jumping_count,
                                             const bool is_start,
                                             const size_t starting_count,
                                             const bool is_end)
{
    std::vector<ConstraintsWithCount> footstep_constraints;
    {
        const size_t num_constraints = is_start ? 3 : 2;
        footstep_constraints.reserve(num_constraints);
        footstep_constraints.push_back(last_constraints);
    }

    const size_t landing_count = jump_start_count + jumping_count;

    if (is_start) {
        ConstraintsWithCount& first_constraints = footstep_constraints.back();

        first_constraints.start_count = starting_count;
        first_constraints.clearLimbViaPoints();
        first_constraints.is_stable = true;
        for (const size_t up_idx : land_indices) {
            first_constraints.constraints[up_idx].changeDefaultContacts();
            first_constraints.constraints[up_idx].setConstraintType(LinkConstraint::FLOAT);
        }
    }

    if (!is_end) {
        if (is_start) footstep_constraints.push_back(footstep_constraints.back());
        ConstraintsWithCount& jumping_phase_constraints = footstep_constraints.back();

        jumping_phase_constraints.start_count = jump_start_count;
        jumping_phase_constraints.clearLimbViaPoints();
        jumping_phase_constraints.is_stable = false;
        for (const size_t jump_idx : jump_indices) {
            jumping_phase_constraints.constraints[jump_idx].changeDefaultContacts();
            jumping_phase_constraints.constraints[jump_idx].setConstraintType(LinkConstraint::FLOAT);
        }
    }

    {
        footstep_constraints.push_back(footstep_constraints.back()); // 歩行時のconstraintは毎歩2つ以上ほしいのでis_endのときは同じものを1つ多めに入れて2つにしている
        ConstraintsWithCount& landing_phase_constraints = footstep_constraints.back();

        landing_phase_constraints.start_count = landing_count;
        landing_phase_constraints.clearLimbViaPoints();
        landing_phase_constraints.is_stable = true;

        const size_t land_indices_size = land_indices.size();
        for (size_t i = 0; i < land_indices_size; ++i) {
            LinkConstraint& landing_constraint = landing_phase_constraints.constraints[land_indices[i]];
            landing_constraint.changeDefaultContacts();
            landing_constraint.targetCoord() = targets[i];
            landing_constraint.setConstraintType(LinkConstraint::FIX);
        }
    }

    return footstep_constraints;
}

std::vector<ConstraintsWithCount>
GaitGenerator::calcFootStepConstraintsForJump(const ConstraintsWithCount& last_constraints,
                                             const std::vector<Eigen::Isometry3d>& targets,
                                             const size_t jump_start_count,
                                              const size_t jumping_count)
{
    std::vector<ConstraintsWithCount> footstep_constraints;
    {
        // TODO: is_endも必要か
        constexpr size_t num_constraints = 2;
        footstep_constraints.reserve(num_constraints);
        footstep_constraints.push_back(last_constraints);
    }

    const size_t landing_count = jump_start_count + jumping_count;

    {
        ConstraintsWithCount& jumping_phase_constraints = footstep_constraints.back();
        jumping_phase_constraints.start_count = jump_start_count;
        jumping_phase_constraints.clearLimbViaPoints();
        jumping_phase_constraints.is_stable = false;

        for (auto& constraint : jumping_phase_constraints.constraints) {
            constraint.changeDefaultContacts();
            constraint.setConstraintType(LinkConstraint::FLOAT);
        }
    }

    {
        footstep_constraints.push_back(footstep_constraints.back());
        ConstraintsWithCount& landing_phase_constraints = footstep_constraints.back();
        landing_phase_constraints.start_count = landing_count;
        landing_phase_constraints.clearLimbViaPoints();
        landing_phase_constraints.is_stable = true;

        for (size_t i = 0; i < landing_phase_constraints.constraints.size(); ++i) {
            LinkConstraint& landing_constraint = landing_phase_constraints.constraints[i];
            landing_constraint.changeDefaultContacts();
            landing_constraint.targetCoord() = targets[i];
            landing_constraint.setConstraintType(LinkConstraint::FIX);
        }
    }

    return footstep_constraints;
}

void GaitGenerator::addFootStep(const ConstraintsWithCount& last_constraints,
                                const std::vector<size_t>& swing_indices,
                                const std::vector<Eigen::Isometry3d>& targets,
                                const size_t swing_start_count,
                                const size_t one_step_count,
                                const bool use_toe_heel,
                                const std::vector<size_t>& toe_support_indices,
                                const size_t toe_support_count,
                                const size_t heel_support_count)
{
    addConstraintsList(calcFootStepConstraints(last_constraints,
                                               swing_indices,
                                               targets,
                                               swing_start_count,
                                               one_step_count,
                                               use_toe_heel,
                                               toe_support_indices,
                                               toe_support_count,
                                               heel_support_count));
}

// TODO: 名前をFootStepではなくする
// Constraintsの存在チェックバージョン．
void GaitGenerator::addFootStep(const std::vector<int>& swing_link_ids, const std::vector<Eigen::Isometry3d>& targets,
                                const size_t swing_start_count, const size_t one_step_count)
{
    // TODO: Last決め打ちで良さそう
    const size_t cur_idx = getConstraintIndexFromCount(constraints_list, swing_start_count - 1);

    std::vector<size_t> swing_indices;
    std::vector<Eigen::Isometry3d> swing_targets;
    swing_indices.reserve(swing_link_ids.size());
    swing_targets.reserve(swing_link_ids.size());
    for (size_t i = 0; i < swing_link_ids.size(); ++i) {
        const int link_id = constraints_list[cur_idx].getConstraintIndexFromLinkId(swing_link_ids[i]);
        if (link_id == -1) continue;
        swing_indices.push_back(swing_link_ids[i]);
        swing_targets.push_back(targets[i]);
    }

    if (swing_indices.empty()) {
        std::cerr << "[GaitGenerator] Cannot find constraints" << std::endl; // TODO: error message
        return;
    }

    addFootStep(constraints_list[cur_idx], swing_indices, swing_targets, swing_start_count, one_step_count);
}

// TODO: foot stepの生成はm_robotを持っている上位に移すべきか
//       Use addFootStep (swing_indicesを与える関数を作ると最初の処理を省略できる)
void GaitGenerator::addFootStepFromVelocity(const double vel_x, const double vel_y, const double vel_yaw,
                                            const size_t swing_start_count, const double step_time, const double dt,
                                            const double max_step_length, const double max_rotate_angle /*[rad]*/,
                                            const int support_id, const int swing_id)
{
    const size_t cur_idx     = getConstraintIndexFromCount(constraints_list, swing_start_count - 1);
    const size_t support_idx = constraints_list[cur_idx].getConstraintIndexFromLinkId(support_id);
    const size_t swing_idx   = constraints_list[cur_idx].getConstraintIndexFromLinkId(swing_id);

    if (support_idx == -1 || swing_idx == -1) {
        std::cerr << "[GaitGenerator] Cannot find constraint " << support_id << "or constraint " << swing_id << std::endl; // TODO: error message
        return;
    }

    ConstraintsWithCount& current_constraints = constraints_list[cur_idx];
    LinkConstraint& support = current_constraints.constraints[support_idx];
    LinkConstraint& landing = current_constraints.constraints[swing_idx];

    // Support limb coordinate
    // TODO: ここあやしい
    double stride_x = vel_x / step_time;
    double stride_y = vel_y / step_time + (support.targetRot().transpose() * (landing.targetPos() - support.targetPos()))[1];
    double yaw_angle = vel_yaw / step_time;

    // TODO: stride limitation

    Eigen::Isometry3d trans_sup_local;
    trans_sup_local.translation() = Eigen::Vector3d(stride_x, stride_y, 0);
    trans_sup_local.linear() = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    std::vector<Eigen::Isometry3d> swing_target;
    swing_target.reserve(1);
    swing_target.push_back(trans_sup_local * support.targetCoord());

    std::vector<size_t> swing_idx_vec;
    swing_idx_vec.reserve(1);
    swing_idx_vec.push_back(swing_idx);

    addFootStep(current_constraints, swing_idx_vec, swing_target, swing_start_count, static_cast<size_t>(step_time / dt));
}

std::vector<int> GaitGenerator::getConstraintLinkIds()
{
    ConstraintsWithCount& last_constraints = constraints_list.back();
    std::vector<int> ids;
    ids.reserve(last_constraints.constraints.size());
    for (const LinkConstraint& constraint : last_constraints.constraints) {
        ids.push_back(constraint.getLinkId());
    }

    return ids;
}

bool GaitGenerator::setToeContactPoints(const int link_id, const std::vector<hrp::Vector3>& contact_points)
{
    ConstraintsWithCount& last_constraints = constraints_list.back();
    const int index = last_constraints.getConstraintIndexFromLinkId(link_id);
    if (index == -1) return false;

    last_constraints.constraints[index].setToeContactPoints(contact_points);
    return true;
}

bool GaitGenerator::setHeelContactPoints(const int link_id, const std::vector<hrp::Vector3>& contact_points)
{
    ConstraintsWithCount& last_constraints = constraints_list.back();
    const int index = last_constraints.getConstraintIndexFromLinkId(link_id);
    if (index == -1) return false;

    last_constraints.constraints[index].setHeelContactPoints(contact_points);
    return true;
}

// Rot は world base rotを使う ?
// 元の実装は，支持脚座標を入れ替えながらwhileで進んでいく
bool GaitGenerator::goPos(const Eigen::Isometry3d& target,
                          const std::vector<int>& support_link_cycle,
                          const std::vector<int>& swing_link_cycle)
{
    std::vector<ConstraintsWithCount> new_constraints;

    size_t cur_cycle = 0;
    const size_t cycle_size = support_link_cycle.size(); // TODO: エラーチェック support == swing
    hrp::Vector3 dp;
    Eigen::AngleAxisd dr;
    Eigen::Isometry3d sup_to_swing_trans;
    std::vector<Eigen::Isometry3d> translationFromLimbToCOP;

    const auto calcdpAnddr = [&dp, &dr, &target, &translationFromLimbToCOP](const Eigen::Isometry3d& support_coord,
                                                                            const int limb_idx) {
        const Eigen::Isometry3d diff_coord = (support_coord * translationFromLimbToCOP[limb_idx]).inverse() * target;
        dp = diff_coord.translation();
        dr = Eigen::AngleAxisd(diff_coord.linear());
    };

    {
        ConstraintsWithCount cur_constraints = getCurrentConstraints(loop);
        cur_constraints.start_count = loop;

        int support_idx, swing_idx;
        if (!getSupportSwingIndex(support_idx, swing_idx, cur_constraints, cur_cycle, support_link_cycle, swing_link_cycle)) return false;

        translationFromLimbToCOP.reserve(cur_constraints.constraints.size());
        const Eigen::Isometry3d init_cop = cur_constraints.calcCOPCoord();
        for (const auto& constraint : cur_constraints.constraints) {
            translationFromLimbToCOP.push_back(constraint.targetCoord().inverse() * init_cop);
        }

        const Eigen::Isometry3d& support_target = cur_constraints.constraints[support_idx].targetCoord();
        calcdpAnddr(support_target, support_idx);

        const Eigen::Isometry3d& swing_target = cur_constraints.constraints[swing_idx].targetCoord();
        sup_to_swing_trans = support_target.inverse() * swing_target;

        const size_t min_num_steps = std::max(static_cast<size_t>(std::ceil(dp.head<2>().norm() / max_stride)),
                                                  static_cast<size_t>(std::ceil(dr.angle() / max_rot_angle)));
        new_constraints.reserve(2 * min_num_steps + 5);
        addFirstTwoConstraints(new_constraints, cur_constraints);
    }

    const auto notdpZero = [&dp]() -> bool { return !eps_eq(dp.head<2>().squaredNorm(), 0.0, 1e-3 * 0.1); };
    const auto notdrZero = [&dr]() -> bool { return !eps_eq(dr.angle(), 0.0, deg2rad(0.5)); };

    const double squared_max_stride = max_stride * max_stride;

    while (notdrZero() || notdpZero()) {
        const ConstraintsWithCount& last_constraints = new_constraints.back();

        int support_idx, swing_idx;
        if (!getSupportSwingIndex(support_idx, swing_idx, last_constraints, cur_cycle, support_link_cycle, swing_link_cycle)) return false;

        if (notdpZero()) {
            std::cerr << "dp: " << dp.transpose() << std::endl;
            dp = dp.squaredNorm() > squared_max_stride ? safeNormalize(dp) * max_stride : dp;
        } else dp.setZero();

        if (notdrZero()) {
            dr.angle() = std::min(dr.angle(), max_rot_angle);
            std::cerr << "dr angle: " << dr.angle() << std::endl;
        } else dr.angle() = 0;

        // TODO: goPos(1, 0, 90)のときうまくいかない
        Eigen::Isometry3d landing_target = last_constraints.constraints[support_idx].targetCoord() * sup_to_swing_trans;
        Eigen::Isometry3d rotate_around_cop = translationFromLimbToCOP[swing_idx].inverse();
        rotate_around_cop.prerotate(dr);
        landing_target = landing_target * translationFromLimbToCOP[swing_idx] * rotate_around_cop; // TODO
        landing_target.translation() += dp;

        if (default_use_toe_heel) std::cerr << "default true" << std::endl;
        else std::cerr << "default false" << std::endl;
        addNewFootSteps(new_constraints, last_constraints, swing_idx, support_idx, landing_target, default_use_toe_heel);
        // Update to next step
        calcdpAnddr(landing_target, swing_idx);
        sup_to_swing_trans = sup_to_swing_trans.inverse(); // TODO: biped
        cur_cycle = (cur_cycle + 1) % cycle_size;
    }

    // Finalize, TODO: biped only
    const ConstraintsWithCount& last_constraints = new_constraints.back();
    int support_idx, swing_idx;
    if (!getSupportSwingIndex(support_idx, swing_idx, last_constraints, cur_cycle, support_link_cycle, swing_link_cycle)) return false;
    const Eigen::Isometry3d landing_target = last_constraints.constraints[support_idx].targetCoord() * sup_to_swing_trans;

    addNewFootSteps(new_constraints, last_constraints, swing_idx, support_idx, landing_target, false);
    new_constraints.back().is_stable = false;

    finalizeFootSteps(new_constraints);
    return true;
}

bool GaitGenerator::setFootSteps(const std::vector<int>& support_link_cycle,
                                 const std::vector<int>& swing_link_cycle,
                                 hrp::Vector3 footsteps_pos[],
                                 Eigen::Quaterniond footsteps_rot[],
                                 int fs_side[],
                                 int length)
{
    // TODO biped only
    std::vector<ConstraintsWithCount> new_constraints;

    size_t cur_cycle = fs_side[0];  // 0->右、1->左

    {                           // 一歩目？
        ConstraintsWithCount cur_constraints = getCurrentConstraints(loop);
        cur_constraints.start_count = loop;

        int support_idx, swing_idx;
        if (!getSupportSwingIndex(support_idx, swing_idx, cur_constraints, cur_cycle, support_link_cycle, swing_link_cycle)) return false;

        new_constraints.reserve(length);
        addFirstTwoConstraints(new_constraints, cur_constraints);
    }

    // footstep
    for (size_t step = 1; step < length; ++step) { // 踏み出すfootstepから
        const ConstraintsWithCount& next_constraints = new_constraints.back();
        int support_idx, swing_idx;
        cur_cycle = fs_side[step];
        if (!getSupportSwingIndex(support_idx, swing_idx, next_constraints, cur_cycle, support_link_cycle, swing_link_cycle)) return false;

        Eigen::Isometry3d landing_target;
        landing_target.linear() = footsteps_rot[step].normalized().toRotationMatrix();
        landing_target.translation().x() = footsteps_pos[step].x();
        landing_target.translation().y() = footsteps_pos[step].y();
        landing_target.translation().z() = footsteps_pos[step].z();

        addNewFootSteps(new_constraints, next_constraints, swing_idx, support_idx, landing_target, false);
    }
    new_constraints.back().is_stable = false;

    finalizeFootSteps(new_constraints);

    return true;
}

bool GaitGenerator::setRunningFootSteps(const std::vector<int>& support_link_cycle,
                                        const std::vector<int>& swing_link_cycle,
                                        hrp::Vector3 footsteps_pos[],
                                        Eigen::Quaterniond footsteps_rot[],
                                        int fs_side[],
                                        int length,
                                        const double dt,
                                        const double g_acc)
{
    // TODO biped only
    std::vector<ConstraintsWithCount> new_constraints;

    size_t cur_cycle = fs_side[0];  // 0->右、1->左

    {
        ConstraintsWithCount cur_constraints = getCurrentConstraints(loop);
        cur_constraints.start_count = loop;

        int support_idx, swing_idx;
        if (!getSupportSwingIndex(support_idx, swing_idx, cur_constraints, cur_cycle, support_link_cycle, swing_link_cycle)) return false;

        new_constraints.reserve(length);
        addFirstTwoConstraints(new_constraints, cur_constraints);
    }

    new_constraints.back().setDelayTimeOffsets(0.03);
    double take_off_z_vel = std::sqrt(2 * g_acc * default_jump_height);
    size_t flight_phase_count = static_cast<size_t>(2 * take_off_z_vel / g_acc / dt);

    // footstep
    for (size_t step = 1; step < length; ++step) { // 踏み出すfootstepから
        const ConstraintsWithCount& next_constraints = new_constraints.back();
        int jump_idx, land_idx;
        cur_cycle = fs_side[step];
        if (!getSupportSwingIndex(jump_idx, land_idx, next_constraints, cur_cycle, support_link_cycle, swing_link_cycle)) return false;

        Eigen::Isometry3d landing_target;
        landing_target.linear() = footsteps_rot[step].normalized().toRotationMatrix();
        landing_target.translation().x() = footsteps_pos[step].x();
        landing_target.translation().y() = footsteps_pos[step].y();
        landing_target.translation().z() = footsteps_pos[step].z();

        addNewRunningFootSteps(new_constraints, next_constraints, jump_idx, land_idx, landing_target, flight_phase_count, (step == 1), (step == length - 1));
    }
    new_constraints.back().is_stable = false;

    finalizeFootSteps(new_constraints);

    return true;
}

bool GaitGenerator::setJumpingFootSteps(const double dt,
                                        std::vector<std::vector<hrp::Vector3> > footsteps_pos,
                                        std::vector<std::vector<Eigen::Quaterniond> > footsteps_rot,
                                        std::vector<std::vector<int> > fs_side,
                                        const double g_acc)
{
    std::vector<ConstraintsWithCount> new_constraints;
    // new_constraints.reserve(42); // ?
    {
        ConstraintsWithCount cur_constraints = constraints_list[cur_const_idx];
        cur_constraints.start_count = loop;
        new_constraints.push_back(std::move(cur_constraints));
        new_constraints.back().setDelayTimeOffsets(0.03);
    }

    // std::vector<size_t> jump_indices{0, 1};
    // std::vector<size_t> land_indices{0, 1};
    // std::vector<Eigen::Isometry3d> targets;
    // targets.reserve(jump_indices.size());
    // for (const auto& constraint : constraints_list[cur_const_idx].constraints) {
    //     targets.push_back(constraint.targetCoord());
    // }

    running_mode = EXTENDED_MATRIX; // こいつが決定的か(run との違い)

    const double take_off_z_vel = std::sqrt(2 * g_acc * default_jump_height);
    const size_t flight_phase_count = static_cast<size_t>(2 * take_off_z_vel / g_acc / dt);

    std::vector<Eigen::Isometry3d> last_targets;

    for (size_t i = 1; i < fs_side.size(); ++i) { // todo step数がなぜrunと異なる
        const ConstraintsWithCount& last_constraints = new_constraints.back();
        std::vector<Eigen::Isometry3d> targets;
        std::vector<size_t> jump_indices; // 0 == rleg
        std::vector<size_t> land_indices;

        for (size_t j = 0; j < fs_side[i].size(); ++j) {
            Eigen::Isometry3d target;

            jump_indices.push_back(fs_side[i][j]);
            land_indices.push_back(fs_side[i][j]);

            target.linear() = footsteps_rot[i][j].normalized().toRotationMatrix();
            target.translation().x() = footsteps_pos[i][j].x();
            target.translation().y() = footsteps_pos[i][j].y();
            target.translation().z() = footsteps_pos[i][j].z();

            targets.push_back(target);
        }

        const std::vector<ConstraintsWithCount> run_constraints = calcFootStepConstraintsForRun(last_constraints,
                                                                                                jump_indices,
                                                                                                land_indices,
                                                                                                targets,
                                                                                                last_constraints.start_count + default_support_count_run,
                                                                                                flight_phase_count);
        for (const auto& constraints : run_constraints) {
            new_constraints.push_back(constraints);
        }

        std::cerr << "[GaitGenerator] add jump " << i << std::endl;
        last_targets = targets;
    }

    {                           // なんでこれないと落ちる？
        new_constraints.push_back(new_constraints.back());
        ConstraintsWithCount& final_constraints = new_constraints.back();
        final_constraints.start_count += default_support_count_run;
        final_constraints.clearLimbViaPoints();
        final_constraints.is_stable = true;

        for (size_t i = 0; i < fs_side[fs_side.size()-1].size(); ++i) {
            LinkConstraint& landing_constraint = final_constraints.constraints[fs_side[fs_side.size() - 1][i]];
            landing_constraint.changeDefaultContacts();
            landing_constraint.targetCoord() = last_targets[i];
            landing_constraint.setConstraintType(LinkConstraint::FIX);
        }
    }

    // Update constraints_list
    std::lock_guard<std::mutex> lock(m_mutex);
    default_step_height = default_jump_height + 0.02;
    locomotion_mode = RUN;
    const size_t diff_count = loop - new_constraints[0].start_count + 1;
    for (auto& constraints : new_constraints) {
        constraints.start_count += diff_count;
    }
    setConstraintsList(std::move(new_constraints));
    setRefZMPList(loop);
    std::cerr << "[GaitGenerator] add jump end" << std::endl;
}

bool GaitGenerator::startRunning(const double dt, const double g_acc)
{
    running_mode = EXTENDED_MATRIX;

    std::vector<ConstraintsWithCount> new_constraints;
    ConstraintsWithCount cur_constraints = constraints_list[cur_const_idx];
    cur_constraints.start_count = loop;
    new_constraints.push_back(std::move(cur_constraints));
    new_constraints.back().setDelayTimeOffsets(0.03);

    std::vector<size_t> jump_idx{0};
    std::vector<size_t> land_idx{1};
    std::vector<Eigen::Isometry3d> org_targets;
    for (const auto& constraint : constraints_list[cur_const_idx].constraints) {
        org_targets.push_back(constraint.targetCoord());
    }

    std::vector<Eigen::Isometry3d> targets{org_targets[land_idx[0]]};
    // std::vector<Eigen::Isometry3d> targets{org_targets[0], org_targets[1]}; // Jump
    const double take_off_z_vel = std::sqrt(2 * g_acc * default_jump_height);
    const size_t flight_phase_count = static_cast<size_t>(2 * take_off_z_vel / g_acc / dt);

    {
        const size_t starting_count = loop + 100;
        const std::vector<ConstraintsWithCount> run_constraints = calcFootStepConstraintsForRun(new_constraints.back(),
                                                                                                jump_idx,
                                                                                                land_idx,
                                                                                                targets,
                                                                                                starting_count + default_support_count_run,
                                                                                                flight_phase_count,
                                                                                                true,
                                                                                                starting_count);
        // const std::vector<ConstraintsWithCount> run_constraints = calcFootStepConstraintsForJump(constraints_list[cur_const_idx],
        //                                                                                          targets,
        //                                                                                          starting_count + default_support_count_run,
        //                                                                                          flight_phase_count);
        for (const auto& constraints : run_constraints) {
            new_constraints.push_back(constraints);
        }
        std::swap(jump_idx[0], land_idx[0]);
        targets[0] = org_targets[land_idx[0]];
        // targets[0].translation()[0] += 0.2;
        // targets[0].translation()[1] = org_targets[land_idx[0]].translation()[1];
        std::cerr << "add run first" << std::endl;
    }

    for (size_t i = 0; i < 30; ++i) {
        const ConstraintsWithCount& last_constraints = new_constraints.back();
        const std::vector<ConstraintsWithCount> run_constraints = calcFootStepConstraintsForRun(last_constraints,
                                                                                                jump_idx,
                                                                                                land_idx,
                                                                                                targets,
                                                                                                last_constraints.start_count + default_support_count_run,
                                                                                                flight_phase_count);
        // const std::vector<ConstraintsWithCount> run_constraints = calcFootStepConstraintsForJump(last_constraints,
        //                                                                                          targets,
        //                                                                                          last_constraints.start_count + default_support_count_run,
        //                                                                                          flight_phase_count);
        for (const auto& constraints : run_constraints) {
            new_constraints.push_back(constraints);
        }
        std::swap(jump_idx[0], land_idx[0]);
        // targets[0] = org_targets[land_idx[0]];
        targets[0].translation()[0] += 0.2;
        targets[0].translation()[1] = org_targets[land_idx[0]].translation()[1];
        std::cerr << "add run " << i << std::endl;
    }

    // Update constraints_list
    std::lock_guard<std::mutex> lock(m_mutex);
    // default_step_height = 0.15; // tmp
    default_step_height = 0.05; // tmp
    locomotion_mode = RUN;
    const size_t diff_count = loop - new_constraints[0].start_count + 1;
    for (auto& constraints : new_constraints) {
        constraints.start_count += diff_count;
    }
    setConstraintsList(std::move(new_constraints));
    setRefZMPList(loop);
    std::cerr << "add run end" << std::endl;
}

bool GaitGenerator::startRunJumpDemo(const double dt, const double g_acc)
{
    std::vector<ConstraintsWithCount> new_constraints;
    ConstraintsWithCount cur_constraints = constraints_list[cur_const_idx];
    cur_constraints.start_count = loop;
    new_constraints.push_back(std::move(cur_constraints));
    new_constraints.back().setDelayTimeOffsets(0.03);

    std::vector<size_t> jump_idx{0};
    std::vector<size_t> land_idx{1};
    std::vector<Eigen::Isometry3d> org_targets;
    for (const auto& constraint : constraints_list[cur_const_idx].constraints) {
        org_targets.push_back(constraint.targetCoord());
    }

    std::vector<Eigen::Isometry3d> targets{org_targets[land_idx[0]]};
    // std::vector<Eigen::Isometry3d> targets{org_targets[0], org_targets[1]}; // Jump

    const auto swapTarget = [&]() {
        std::swap(jump_idx[0], land_idx[0]);
        targets[0] = org_targets[land_idx[0]];
    };

    const double jump_height = 0.03;
    const double take_off_z_vel = std::sqrt(2 * g_acc * jump_height);
    const size_t flight_phase_count = static_cast<size_t>(2 * take_off_z_vel / g_acc / dt);

    {
        const size_t starting_count = loop + 100;
        const std::vector<ConstraintsWithCount> run_constraints = calcFootStepConstraintsForRun(new_constraints.back(),
                                                                                                jump_idx,
                                                                                                land_idx,
                                                                                                targets,
                                                                                                starting_count + default_support_count_run,
                                                                                                flight_phase_count,
                                                                                                true,
                                                                                                starting_count);
        for (const auto& constraints : run_constraints) {
            new_constraints.push_back(constraints);
        }
    }

    for (size_t i = 0; i < 3; ++i) {
        const ConstraintsWithCount& last_constraints = new_constraints.back();
        const std::vector<ConstraintsWithCount> run_constraints = calcFootStepConstraintsForRun(last_constraints,
                                                                                                std::vector<size_t>{0, 1},
                                                                                                land_idx,
                                                                                                targets,
                                                                                                last_constraints.start_count + default_support_count_run,
                                                                                                flight_phase_count);
        for (const auto& constraints : run_constraints) {
            new_constraints.push_back(constraints);
        }
    }

    swapTarget();

    for (size_t i = 0; i < 3; ++i) {
        const ConstraintsWithCount& last_constraints = new_constraints.back();
        const std::vector<ConstraintsWithCount> run_constraints = calcFootStepConstraintsForRun(last_constraints,
                                                                                                std::vector<size_t>{0, 1},
                                                                                                land_idx,
                                                                                                targets,
                                                                                                last_constraints.start_count + default_support_count_run,
                                                                                                flight_phase_count);
        for (const auto& constraints : run_constraints) {
            new_constraints.push_back(constraints);
        }
    }

    for (size_t i = 0; i < 5; ++i) {
        swapTarget();
        const ConstraintsWithCount& last_constraints = new_constraints.back();
        const std::vector<ConstraintsWithCount> run_constraints = calcFootStepConstraintsForRun(last_constraints,
                                                                                                jump_idx,
                                                                                                land_idx,
                                                                                                targets,
                                                                                                last_constraints.start_count + default_support_count_run,
                                                                                                flight_phase_count);
        for (const auto& constraints : run_constraints) {
            new_constraints.push_back(constraints);
        }
    }

    {
        new_constraints.push_back(new_constraints.back());
        ConstraintsWithCount& last_constraints = new_constraints.back();

        last_constraints.start_count += default_single_support_count;
        last_constraints.clearLimbViaPoints();
        last_constraints.is_stable = true;

        for (size_t i = 0; i < org_targets.size(); ++i) {
            last_constraints.constraints[i].targetCoord() = org_targets[i];
            last_constraints.constraints[i].setConstraintType(LinkConstraint::FIX);
        }
    }

    // Update constraints_list
    std::lock_guard<std::mutex> lock(m_mutex);
    // default_step_height = 0.15; // tmp
    default_step_height = 0.05; // tmp
    locomotion_mode = RUN;
    const size_t diff_count = loop - new_constraints[0].start_count + 1;
    for (auto& constraints : new_constraints) {
        constraints.start_count += diff_count;
    }
    setConstraintsList(std::move(new_constraints));
    setRefZMPList(loop);
}

bool GaitGenerator::startJumping(const double dt, const double g_acc)
{
    std::vector<ConstraintsWithCount> new_constraints;
    new_constraints.reserve(42);
    {
        ConstraintsWithCount cur_constraints = constraints_list[cur_const_idx];
        cur_constraints.start_count = loop;
        new_constraints.push_back(std::move(cur_constraints));
        new_constraints.back().setDelayTimeOffsets(0.03);
    }

    std::vector<size_t> jump_indices{0, 1};
    std::vector<size_t> land_indices{0, 1};
    std::vector<Eigen::Isometry3d> targets;
    targets.reserve(jump_indices.size());
    for (const auto& constraint : constraints_list[cur_const_idx].constraints) {
        targets.push_back(constraint.targetCoord());
    }

    const double take_off_z_vel = std::sqrt(2 * g_acc * default_jump_height);
    const size_t flight_phase_count = static_cast<size_t>(2 * take_off_z_vel / g_acc / dt);

    for (size_t i = 0; i < 20; ++i) {
        const ConstraintsWithCount& last_constraints = new_constraints.back();
        const std::vector<ConstraintsWithCount> run_constraints = calcFootStepConstraintsForRun(new_constraints.back(),
                                                                                                jump_indices,
                                                                                                land_indices,
                                                                                                targets,
                                                                                                last_constraints.start_count + default_support_count_run,
                                                                                                flight_phase_count);
        for (const auto& constraints : run_constraints) {
            new_constraints.push_back(constraints);
        }

        std::cerr << "add jump " << i << std::endl;
    }

    // Update constraints_list
    std::lock_guard<std::mutex> lock(m_mutex);
    default_step_height = default_jump_height + 0.02;
    locomotion_mode = RUN;
    const size_t diff_count = loop - new_constraints[0].start_count + 1;
    for (auto& constraints : new_constraints) {
        constraints.start_count += diff_count;
    }
    setConstraintsList(std::move(new_constraints));
    setRefZMPList(loop);
    std::cerr << "add jump end" << std::endl;
}

void GaitGenerator::setConstraintToFootCoord(const hrp::BodyPtr& _robot)
{
    auto& cur_constraints = constraints_list.back().constraints;
    for (auto& constraint : cur_constraints) {
        const hrp::Link* const link = _robot->link(constraint.getLinkId());
        constraint.targetPos() = constraint.calcActualTargetPosFromLinkState(link->p, link->R);
        constraint.targetRot() = constraint.calcActualTargetRotFromLinkState(link->R);
    }
}

}
