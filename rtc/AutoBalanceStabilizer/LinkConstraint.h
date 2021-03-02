// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  LinkConstraint.h
 * @brief
 * @date  $Date$
 */

#ifndef LINKCONSTRAINT_H
#define LINKCONSTRAINT_H

#include <vector>
#include <numeric>
#include <algorithm>
#include <memory>
#include <Eigen/Geometry>
#include <hrpUtil/EigenTypes.h>
#include "LimbTrajectoryGenerator.h"
#include <hrpModel/Link.h>
#include <hrpModel/Body.h>
#include "Utility.h"

// Memo: 着地時間が例えば遅くなった時，足は待っているのかどうか．どうやって修正するか

namespace hrp {

class LinkConstraint
{
  public:
    enum ConstraintType : size_t {FIX, ROTATE, SLIDE, FLOAT, FREE};

  private:
    int link_id;
    ConstraintType constraint_type = FIX;
    std::vector<hrp::Vector3> link_contact_points; // Link local
    Eigen::Isometry3d link_local_coord = Eigen::Isometry3d::Identity(); // Link local
    Eigen::Isometry3d target_coord = Eigen::Isometry3d::Identity(); // Global

    hrp::Vector3 pos_vel = hrp::Vector3::Zero();
    hrp::Vector3 pos_acc = hrp::Vector3::Zero();
    double rot_vel = 0; // TODO: 3d vec or axis-angle?
    double rot_acc = 0;

    // TODO: sensorやref forceなど入れる？
    bool is_zmp_calc_target = true;
    hrp::Vector3 local_pos_offset = hrp::Vector3::Zero(); // Link local
    hrp::Vector3 start_cop_offset = hrp::Vector3::Zero(); // Link local // TODO: cop_offsetやcop_weightを関数化してしまうのもあり
    hrp::Vector3 cop_offset = hrp::Vector3::Zero(); // Link local
    double cop_weight = 1.0; // TODO: FLOATではweightを0に?

    // // IK variables
    // hrp::dvector6 constraint_weight = hrp::dvector6::Ones(); // Pos + Rot (Axis-angle)
    // double pos_precision = 1e-4;
    // double rot_precision = deg2rad(0.1);

    // bool use_toe_heel = false; TODO: ?
    std::vector<hrp::Vector3> default_contact_points; // Link local
    std::vector<hrp::Vector3> toe_contact_points; // Link local
    std::vector<hrp::Vector3> heel_contact_points; // Link local

    LimbTrajectoryGenerator limb_traj;

    hrp::Vector3 calcLocalCop(const std::vector<hrp::Vector3>& points,
                              const hrp::Vector3& _pos_offset = hrp::Vector3::Zero())
    {
        return std::accumulate(points.begin(), points.end(), hrp::Vector3::Zero().eval()) /
            link_contact_points.size() + _pos_offset;
    }

  public:
    LinkConstraint(const int _link_id, const ConstraintType _constraint_type = FIX);
    // LinkConstraint(const LinkConstraint& lc);
    // LinkConstraint& operator=(const LinkConstraint& lc);
    // ~LinkConstraint();
    // LinkConstraint(LinkConstraint&& lc);
    // LinkConstraint& operator=(LinkConstraint&& lc);

    int getLinkId() const { return link_id; }
    void setLinkContactPoints(const std::vector<hrp::Vector3>& contact_points) { link_contact_points = contact_points; }
    const std::vector<hrp::Vector3>& getLinkContactPoints() const { return link_contact_points; }

    void addLinkContactPoint(const hrp::Vector3& pos) { link_contact_points.push_back(pos); }
    void clearLinkContactPoints() { link_contact_points.clear(); }

    void setDefaultContactPoints(const std::vector<hrp::Vector3>& contact_points) { default_contact_points = contact_points; }
    void setToeContactPoints(const std::vector<hrp::Vector3>& contact_points) { toe_contact_points = contact_points; }
    void setHeelContactPoints(const std::vector<hrp::Vector3>& contact_points) { heel_contact_points = contact_points; }

    void calcLinkLocalPos()
    {
        link_local_coord.translation() = calcLocalCop(link_contact_points, local_pos_offset);
    }

    const Eigen::Isometry3d& localCoord() const { return link_local_coord; }
    Eigen::Isometry3d::ConstTranslationPart localPos() const { return link_local_coord.translation(); }

    Eigen::Isometry3d::LinearPart localRot() { return link_local_coord.linear(); }
    Eigen::Isometry3d::ConstLinearPart localRot() const { return link_local_coord.linear(); }

    Eigen::Isometry3d& targetCoord() { return target_coord; }
    const Eigen::Isometry3d& targetCoord() const { return target_coord; }

    Eigen::Isometry3d::TranslationPart targetPos() { return target_coord.translation(); }
    Eigen::Isometry3d::ConstTranslationPart targetPos() const { return target_coord.translation(); }

    Eigen::Isometry3d::LinearPart targetRot() { return target_coord.linear(); }
    Eigen::Isometry3d::ConstLinearPart targetRot() const { return target_coord.linear(); }

    bool isZmpCalcTarget() const { return is_zmp_calc_target; }

    void setLocalPosOffset(const hrp::Vector3& _offset) { local_pos_offset = _offset; }
    const hrp::Vector3& getLocalPosOffset() const { return local_pos_offset; }

    void setStartCOPOffset(const hrp::Vector3& _offset) { start_cop_offset = _offset; }
    const hrp::Vector3& getStartCOPOffset() const { return start_cop_offset; }
    void setStartCOPOffsetToToe()  { start_cop_offset = calcLocalCop(toe_contact_points); }
    void setStartCOPOffsetToHeel() { start_cop_offset = calcLocalCop(heel_contact_points); }

    void setCOPOffset(const hrp::Vector3& _offset) { cop_offset = _offset; }
    const hrp::Vector3& getCOPOffset() const { return cop_offset; }
    void setCOPOffsetToToe()  { cop_offset = calcLocalCop(toe_contact_points); }
    void setCOPOffsetToHeel() { cop_offset = calcLocalCop(heel_contact_points); }

    void setCOPWeight(double _weight) { cop_weight = _weight; }
    double getCOPWeight() const { return cop_weight; }

    void setConstraintType(ConstraintType type) { constraint_type = type; }
    ConstraintType getConstraintType() const { return constraint_type; }

    std::vector<Eigen::Vector2d> calcContactConvexHull() const;

    hrp::Vector3 calcActualTargetPosFromLinkState(const hrp::Vector3& link_pos, const hrp::Matrix33& link_rot) const
    {
        return link_pos + link_rot * localPos();
    }

    hrp::Matrix33 calcActualTargetRotFromLinkState(const hrp::Matrix33& link_rot) const
    {
        return link_rot * localRot();
    }

    // TODO: toeとheelの時のlocal_pos_offset, cop_offsetの切り替え
    void changeContactPoints(const std::vector<hrp::Vector3>& points);

    // Toe-Heel methods
    bool hasToeHeelContacts() const { return !(toe_contact_points.empty() || heel_contact_points.empty()); }
    void changeDefaultContacts() { changeContactPoints(default_contact_points); }
    void changeToeContacts()     { changeContactPoints(toe_contact_points); }
    void changeHeelContacts()    { changeContactPoints(heel_contact_points); }
    Eigen::Isometry3d calcRotatedTargetAroundToe(const Eigen::Isometry3d& target,
                                                 const Eigen::AngleAxisd& local_rot);
    Eigen::Isometry3d calcRotatedTargetAroundHeel(const Eigen::Isometry3d& target,
                                                  const Eigen::AngleAxisd& local_rot);

    // LimbTrajectoryGenerator
    void copyLimbTrajectoryGenerator(const LinkConstraint& lc) { limb_traj = lc.limb_traj; }
    void copyLimbState(const LinkConstraint& lc) // TODO: 名前もはやこれじゃない
    {
        if (limb_traj.isViaPointsEmpty()) {
            double tmp_offset = limb_traj.getDelayTimeOffset();
            limb_traj = lc.limb_traj;
            limb_traj.setDelayTimeOffset(tmp_offset);
        }
        const hrp::Vector3 move_pos = lc.targetRot() * lc.localRot() * (localPos() - lc.localPos());
        // limb_traj.copyState(lc.limb_traj, move_pos);
        targetPos() = lc.targetPos() + move_pos;
        // TODO: rot_vel, rot_accを足す
        pos_vel = lc.pos_vel;
        pos_acc = lc.pos_acc;
        targetRot() = lc.targetRot();
        rot_vel = lc.rot_vel;
        rot_acc = lc.rot_acc;
    }
    void clearLimbViaPoints() { limb_traj.clearViaPoints(); }
    void calcLimbViaPoints(const LimbTrajectoryGenerator::TrajectoryType traj_type,
                           const Eigen::Isometry3d& goal,
                           const size_t start_count, const size_t goal_count,
                           const double step_height)
    {
        limb_traj.calcViaPoints(traj_type, target_coord, goal, start_count, goal_count, step_height);
    }
    void setLimbViaPoints(const LimbTrajectoryGenerator::TrajectoryType _traj_type,
                          const Eigen::Isometry3d& start,
                          const Eigen::Isometry3d& goal,
                          const std::vector<LimbTrajectoryGenerator::ViaPoint>& _via_points)
    {
        limb_traj.setViaPoints(_traj_type, start, goal, _via_points);
    }
    void modifyLimbViaPoints(const Eigen::Isometry3d& new_goal,
                             const size_t current_count,
                             const size_t new_goal_count,
                             const double dt)
    {
        limb_traj.modifyViaPoints(target_coord, new_goal, current_count, new_goal_count, dt);
    }
    void calcLimbTrajectory(const size_t cur_count, const double dt)
    {
        if (!limb_traj.isViaPointsEmpty()) {
            limb_traj.calcTrajectory(cur_count, dt, targetPos(), pos_vel, pos_acc, targetRot(), rot_vel, rot_acc);
        }
    }
    void setDelayTimeOffset(const double offset) { limb_traj.setDelayTimeOffset(offset); }
    bool isLimbInterpolating(const size_t count) { return limb_traj.isInterpolating(count); }
};

struct ConstraintsWithCount
{
    std::vector<LinkConstraint> constraints;
    size_t start_count = 0;
    bool is_stable = true;

    hrp::Vector3 calcCOPFromConstraints(const LinkConstraint::ConstraintType type_thre = LinkConstraint::FLOAT) const;
    hrp::Vector3 calcStartCOPFromConstraints(const LinkConstraint::ConstraintType type_thre = LinkConstraint::FLOAT) const;
    hrp::Matrix33 calcCOPRotationFromConstraints(const LinkConstraint::ConstraintType type_thre = LinkConstraint::FLOAT) const;
    Eigen::Isometry3d calcCOPCoord(const LinkConstraint::ConstraintType type_thre = LinkConstraint::FLOAT) const
    {
        Eigen::Isometry3d coord;
        coord.translation() = calcCOPFromConstraints(type_thre);
        coord.linear() = calcCOPRotationFromConstraints(type_thre);
        return coord;
    }
    Eigen::Isometry3d calcFootOriginCoord(const hrp::BodyPtr& _robot, const hrp::Vector3& n = hrp::Vector3::UnitZ()) const;
    inline Eigen::Isometry3d calcCOPOriginCoord() const
    {
        Eigen::Isometry3d coord;
        coord.translation() = calcCOPFromConstraints();
        coord.linear() = hrp::alignZaxis(calcCOPRotationFromConstraints());
        return coord;
    }

    int getConstraintIndexFromLinkId(const int id) const;
    std::vector<size_t> getConstraintIndicesFromType(const LinkConstraint::ConstraintType type) const;

    std::vector<Eigen::Vector2d> calcContactConvexHullForAllConstraints() const;

    // LimbTrajectoryGenerator
    void copyLimbTrajectoryGenerator(const ConstraintsWithCount& cwc);
    void copyLimbState(const ConstraintsWithCount& cwc);
    void clearLimbViaPoints()
    {
        for (auto& constraint : constraints) constraint.clearLimbViaPoints();
    }
    // void modifyLimbViaPoints(const size_t constraint_idx,
    //                          const Eigen::Isometry3d& new_goal,
    //                          const size_t new_goal_count);
    void calcLimbTrajectory(const size_t cur_count, const double dt)
    {
        for (auto& constraint : constraints) constraint.calcLimbTrajectory(cur_count, dt);
    }
    void setDelayTimeOffsets(const double offset)
    {
        for (auto& constraint : constraints) constraint.setDelayTimeOffset(offset);
    }
    bool isFlightPhase();
};


// Utility functions
// TODO: LinkConstraintなのかConstraintsWithCountなのかわかりづらい
inline size_t getConstraintIndexFromCount(const std::vector<ConstraintsWithCount>& constraints_list,
                                          const size_t count, const size_t search_start_idx = 0)
{
    size_t constraint_idx = search_start_idx;
    for (size_t i = search_start_idx; i < constraints_list.size() && constraints_list[i].start_count <= count; ++i) {
        constraint_idx = i;
    }
    return constraint_idx;
}

inline size_t getPrevValidConstraintIndex(const std::vector<ConstraintsWithCount>& constraints_list, const size_t cur_idx)
{
    size_t prev_constraint_idx = cur_idx - 1;
    for (size_t i = prev_constraint_idx; i >= 0 && constraints_list[i].start_count == constraints_list[cur_idx].start_count; --i) {
        prev_constraint_idx = i - 1;
    }
    return prev_constraint_idx;
}

// Return next stable constraints index if it exists. If not, return current index if it is stable, else return -1.
inline int getNextStableConstraints(const std::vector<ConstraintsWithCount>& constraints_list, const size_t current_idx = 0)
{
    const size_t constraint_list_size = constraints_list.size();
    if (constraint_list_size > 1 && current_idx == 0) return 2;

    for (int i = current_idx + 1; i < constraint_list_size; ++i) {
        if (constraints_list[i].is_stable) return i;
    }

    if (constraints_list[current_idx].is_stable) return current_idx;
    return -1;
}

}

#endif // LINKCONSTRAINT_H
