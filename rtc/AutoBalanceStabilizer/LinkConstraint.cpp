// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  LinkConstraint.cpp
 * @brief
 * @date  $Date$
 */

#include <iostream>
#include "LinkConstraint.h"
#include "LimbTrajectoryGenerator.h"
#include "PlaneGeometry.h"

namespace hrp {

LinkConstraint::LinkConstraint(const int _link_id, const ConstraintType _constraint_type)
    : link_id(_link_id),
      constraint_type(_constraint_type)
      // limb_traj_impl(std::make_unique<LimbTrajectoryGenerator>())
{}

// LinkConstraint::LinkConstraint(const LinkConstraint& lc)
//     : link_id               (lc.link_id),
//       link_contact_points   (lc.link_contact_points),
//       link_local_coord      (lc.link_local_coord),
//       target_coord          (lc.target_coord),
//       cop_offset            (lc.cop_offset),
//       cop_weight            (lc.cop_weight),
//       constraint_type       (lc.constraint_type),
//       default_contact_points(lc.default_contact_points),
//       toe_contact_points    (lc.toe_contact_points),
//       heel_contact_points   (lc.heel_contact_points),
//       limb_traj_impl        (std::make_unique<LimbTrajectoryGenerator>(*(lc.limb_traj_impl)))
// {}

// LinkConstraint& LinkConstraint::operator=(const LinkConstraint& lc)
// {
//     link_id                = lc.link_id;
//     link_contact_points    = lc.link_contact_points;
//     link_local_coord       = lc.link_local_coord;
//     target_coord           = lc.target_coord;
//     cop_offset             = lc.cop_offset;
//     cop_weight             = lc.cop_weight;
//     constraint_type        = lc.constraint_type;
//     default_contact_points = lc.default_contact_points;
//     toe_contact_points     = lc.toe_contact_points;
//     heel_contact_points    = lc.heel_contact_points;
//     limb_traj_impl         = std::make_unique<LimbTrajectoryGenerator>(*(lc.limb_traj_impl));
// }

// LinkConstraint::~LinkConstraint() = default;
// LinkConstraint::LinkConstraint(LinkConstraint&& lc) = default;
// LinkConstraint& LinkConstraint::operator=(LinkConstraint&& lc) = default;

std::vector<Eigen::Vector2d> LinkConstraint::calcContactConvexHull() const
{
    std::vector<Eigen::Vector2d> vertices;
    vertices.reserve(link_contact_points.size());
    for (const hrp::Vector3& point : link_contact_points) {
        vertices.push_back((targetPos() - targetRot() * localRot() * (localPos() + point)).head<2>()); // TODO: rot確認
    }
    return calcConvexHull(vertices); // TODO: 1点のときと2点のときの確認
}

void LinkConstraint::changeContactPoints(const std::vector<hrp::Vector3>& points)
{
    if (points.empty()) {
        std::cerr << "[LinkConstraint] The given vector is empty" << std::endl;
        return;
    }

    const hrp::Vector3 prev_local_pos = localPos();
    link_contact_points = points;
    calcLinkLocalPos();
    // std::cerr << "changed points:\n";
    // for (const auto& point : link_contact_points) std::cerr << point.transpose() << std::endl;
    // std::cerr << "prev local: " << prev_local_pos.transpose() << ", localpos:" << localPos().transpose() << std::endl;
    // std::cerr << "prev target: " << targetPos().transpose();
    const hrp::Vector3 move_pos = targetRot() * localRot() * (localPos() - prev_local_pos);
    targetPos() += move_pos;
    // std::cerr << ", move_pos: " << move_pos.transpose() << ", new target: " << targetPos().transpose() << std::endl;
    // TODO: toeの回転した時のtargetRot
}

// TODO: 要確認
Eigen::Isometry3d LinkConstraint::calcRotatedTargetAroundToe(const Eigen::Isometry3d& target,
                                                             const Eigen::AngleAxisd& local_rot)
{
    Eigen::Isometry3d rotated_target = target;
    const hrp::Vector3 toe_to_default = localPos() - calcLocalCop(toe_contact_points);
    rotated_target.translation() += target.linear() * localRot() * (local_rot.toRotationMatrix() * toe_to_default - toe_to_default);
    rotated_target.linear() = local_rot.toRotationMatrix() * target.linear();
    return rotated_target;
}

Eigen::Isometry3d LinkConstraint::calcRotatedTargetAroundHeel(const Eigen::Isometry3d& target,
                                                              const Eigen::AngleAxisd& local_rot)
{
    Eigen::Isometry3d rotated_target = target;
    const hrp::Vector3 heel_to_default = localPos() - calcLocalCop(heel_contact_points);
    rotated_target.translation() += target.linear() * localRot() * (local_rot.toRotationMatrix() * heel_to_default - heel_to_default);
    rotated_target.linear() = local_rot.toRotationMatrix() * target.linear(); // TODO: localRotを使わないと
    return rotated_target;
}

hrp::Vector3 ConstraintsWithCount::calcCOPFromConstraints(const LinkConstraint::ConstraintType type_thre) const
{
    hrp::Vector3 cop_pos = hrp::Vector3::Zero();
    double sum_weight = 0;

    for (const LinkConstraint& constraint : constraints) {
        if (constraint.getConstraintType() >= type_thre) continue;
        const double weight = constraint.getCOPWeight();
        // TODO:
        //       面接触してる時みたいに、target_coordと実際がずれているときは？
        cop_pos += (constraint.targetPos() + constraint.getCOPOffset()) * weight;
        sum_weight += weight;
    }
    if (sum_weight > 0) cop_pos /= sum_weight;

    return cop_pos;
}

hrp::Vector3 ConstraintsWithCount::calcStartCOPFromConstraints(const LinkConstraint::ConstraintType type_thre) const
{
    hrp::Vector3 cop_pos = hrp::Vector3::Zero();
    double sum_weight = 0;

    for (const LinkConstraint& constraint : constraints) {
        if (constraint.getConstraintType() >= type_thre) continue;
        const double weight = constraint.getCOPWeight();
        // TODO:
        //       面接触してる時みたいに、target_coordと実際がずれているときは？
        cop_pos += (constraint.targetPos() + constraint.getStartCOPOffset()) * weight;
        sum_weight += weight;
    }
    if (sum_weight > 0) cop_pos /= sum_weight;

    return cop_pos;
}

hrp::Matrix33 ConstraintsWithCount::calcCOPRotationFromConstraints(const LinkConstraint::ConstraintType type_thre) const
{
    Eigen::Quaternion<double> cop_quat = Eigen::Quaternion<double>::Identity();
    double sum_weight = 0;
    constexpr double EPS = 1e-6;

    for (const LinkConstraint& constraint : constraints) {
        const double weight = constraint.getCOPWeight();
        if (constraint.getConstraintType() >= type_thre || weight < EPS /* to avoid zero division */) continue;
        sum_weight += weight;
        const Eigen::Quaternion<double> contact_quat(constraint.targetRot());
        cop_quat = cop_quat.slerp(weight / sum_weight, contact_quat);
    }

    return cop_quat.toRotationMatrix();
}

Eigen::Isometry3d ConstraintsWithCount::calcFootOriginCoord(const hrp::BodyPtr& _robot, const hrp::Vector3& n) const
{
    Eigen::Isometry3d coord = Eigen::Isometry3d::Identity();
    Eigen::Quaternion<double> orig_quat = Eigen::Quaternion<double>::Identity();
    double sum_weight = 0;
    constexpr double EPS = 1e-6;
    for (const auto& constraint : constraints) {
        const int link_id = constraint.getLinkId();
        const double weight = constraint.getCOPWeight();
        if (constraint.getConstraintType() >= LinkConstraint::FLOAT || weight < EPS /* to avoid zero division */) continue;
        const hrp::Link* const target = dynamic_cast<hrp::Link*>(_robot->link(link_id));
        sum_weight += weight;

        // pos
        coord.translation() += constraint.calcActualTargetPosFromLinkState(target->p, target->R) * weight;

        // rot
        const Eigen::Quaternion<double> foot_quat(constraint.calcActualTargetRotFromLinkState(target->R));
        orig_quat = orig_quat.slerp(weight / sum_weight, foot_quat);
    }
    if (sum_weight > 0) coord.translation() /= sum_weight;
    coord.linear() = hrp::alignZaxis(orig_quat.toRotationMatrix());

    return coord;
}

int ConstraintsWithCount::getConstraintIndexFromLinkId(const int id) const
{
    for (size_t idx = 0; idx < constraints.size(); ++idx) {
        if (constraints[idx].getLinkId() == id) return idx;
    }

    std::cerr << "[LinkConstraint] Can't find constraints for link " << id << std::endl;
    return -1;
}

std::vector<size_t> ConstraintsWithCount::getConstraintIndicesFromType(const LinkConstraint::ConstraintType type) const
{
    std::vector<size_t> indices;
    indices.reserve(constraints.size());
    for (size_t i = 0; i < constraints.size(); ++i) {
        if (constraints[i].getConstraintType() == type) indices.push_back(i);
    }
    return indices;
}

std::vector<Eigen::Vector2d> ConstraintsWithCount::calcContactConvexHullForAllConstraints() const
{
    std::vector<Eigen::Vector2d> vertices;
    for (const LinkConstraint& constraint : constraints) {
        if (constraint.getConstraintType() >= LinkConstraint::FLOAT) continue;

        Eigen::Isometry3d::ConstTranslationPart target_pos = constraint.targetPos();
        Eigen::Isometry3d::ConstLinearPart target_rot = constraint.targetRot();
        Eigen::Isometry3d::ConstTranslationPart link_local_pos = constraint.localPos();
        Eigen::Isometry3d::ConstLinearPart link_local_rot = constraint.localRot();
        for (const hrp::Vector3& point : constraint.getLinkContactPoints()) {
            // vertices.push_back((target_pos - target_rot * link_local_rot * (link_local_pos + point)).head<2>()); // TODO: rot
            vertices.push_back((target_pos + target_rot * link_local_rot * (point - link_local_pos)).head<2>()); // TODO: check rot
        }
    }
    return calcConvexHull(vertices);
}

// TODO: 使う?
void ConstraintsWithCount::copyLimbTrajectoryGenerator(const ConstraintsWithCount& cwc)
{
    if (constraints.size() != cwc.constraints.size()) {
        std::cerr << "[ConstraintsWithCount] Two constraints must be save size" << std::endl;
        return;
    }

    for (size_t i = 0; i < constraints.size(); ++i) {
        constraints[i].copyLimbTrajectoryGenerator(cwc.constraints[i]);
    }
}

void ConstraintsWithCount::copyLimbState(const ConstraintsWithCount& cwc)
{
    if (constraints.size() != cwc.constraints.size()) {
        std::cerr << "[ConstraintsWithCount] Two constraints must be save size" << std::endl;
        return;
    }

    for (size_t i = 0; i < constraints.size(); ++i) {
        constraints[i].copyLimbState(cwc.constraints[i]);
    }
}

bool ConstraintsWithCount::isFlightPhase()
{
    for (const auto& constraint : constraints) {
        if (constraint.getConstraintType() < LinkConstraint::FLOAT) return false;
    }
    return true;
}

// void ConstraintsWithCount::modifyLimbViaPoints(const size_t constraint_idx,
//                                                const Eigen::Isometry3d& new_goal,
//                                                const size_t new_goal_count)
// {
// }

}
