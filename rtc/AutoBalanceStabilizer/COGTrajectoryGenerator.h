// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  COGTrajectoryGenerator.h
 * @brief
 * @date  $Date$
 */

#ifndef COGTRAJECTORYGENERATOR_H
#define COGTRAJECTORYGENERATOR_H

#include <vector>
#include <tuple>
#include <memory>
#include <hrpUtil/EigenTypes.h>
#include "PreviewController.h"

namespace hrp {

struct ConstraintsWithCount;

class COGTrajectoryGenerator
{
  public:
    enum CogCalculationType : size_t { PREVIEW_CONTROL, FOOT_GUIDED };
    enum FootGuidedRefZMPType : size_t { FIX, LINEAR, CUBIC };

  private:
    static constexpr double DEFAULT_GRAVITATIONAL_ACCELERATION = 9.80665; // [m/s^2]

    hrp::Vector3 cog     = hrp::Vector3::Zero();
    hrp::Vector3 cog_vel = hrp::Vector3::Zero();
    hrp::Vector3 cog_acc = hrp::Vector3::Zero();
    double ref_cog_z = 1.0;
    double diff_ref_cog_z = 0;
    double omega = std::sqrt(DEFAULT_GRAVITATIONAL_ACCELERATION / 1.0);
    double step_remain_time = 0;
    double const_remain_time = 0;

    // for log
    hrp::Vector3 nominal_zmp = hrp::Vector3::Zero();
    hrp::Vector3 ref_end_cp = hrp::Vector3::Zero();
    hrp::Vector3 new_ref_cp = hrp::Vector3::Zero();

    size_t cog_list_start_count = 0;
    std::vector<hrp::Vector3> cog_list;

    CogCalculationType calculation_type = PREVIEW_CONTROL;
    std::unique_ptr<ExtendedPreviewController> preview_controller;

    bool is_walking = false;

    void updateCogState(const hrp::Vector3& input_zmp, const double dt, const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION);

    // Foot guided run variables
  public:
    COGTrajectoryGenerator(const hrp::Vector3& init_cog,
                           const hrp::Vector3& cur_ref_zmp,
                           const CogCalculationType type = PREVIEW_CONTROL) :
        cog(init_cog), calculation_type(type)
    {
        setOmega(init_cog[2] - cur_ref_zmp[2]);
    }

    COGTrajectoryGenerator(const hrp::Vector3& init_cog,
                           const hrp::Vector3& init_cog_vel,
                           const hrp::Vector3& init_cog_acc,
                           const hrp::Vector3& cur_ref_zmp,
                           const CogCalculationType type = PREVIEW_CONTROL) :
        cog(init_cog), cog_vel(init_cog_vel), cog_acc(init_cog_acc), calculation_type(type)
    {
        setOmega(init_cog[2] - cur_ref_zmp[2]);
    }

    const hrp::Vector3& getCog()    const { return cog; }
    const hrp::Vector3& getCogVel() const { return cog_vel; }
    const hrp::Vector3& getCogAcc() const { return cog_acc; }
    const hrp::Vector3& getNominalZMP() const { return nominal_zmp; }
    const hrp::Vector3& getRefEndCP() const { return ref_end_cp; }
    const hrp::Vector3& getNewRefCP() const { return new_ref_cp; }
    const double& getStepRemainTime() const { return step_remain_time; }
    const double& getConstRemainTime() const { return const_remain_time; }
    const double getRefCogZ() const { return ref_cog_z; }
    const bool getWalkingState() { return is_walking; };
    void setWalkingState(const bool _walking) { is_walking = _walking; };
    hrp::Vector3 calcCP(const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION) const { return cog + cog_vel / omega; }
    hrp::Vector3 calcPointMassZMP(const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION) const
    {
        hrp::Vector3 zmp = cog;
        zmp.head<2>() -= cog_acc.head<2>() / (cog_acc[2] + g_acc) * cog[2];
        return zmp;
    }

    void setCogCalculationType(const CogCalculationType type) { calculation_type = type; }
    void setOmega(const double cog_z, const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION)
    {
        ref_cog_z = cog_z;
        omega = std::sqrt(g_acc / cog_z);
    }
    void retreiveCogZ(const double dt)
    {
        diff_ref_cog_z = -1 / 1.5 * diff_ref_cog_z * dt + diff_ref_cog_z;
        cog[2] = ref_cog_z + diff_ref_cog_z;
    }
    void initPreviewController(const double dt, const hrp::Vector3& cur_ref_zmp);
    void getCogFromCogList(const size_t cur_count, const double dt)
    {
        const size_t cog_idx = cur_count - cog_list_start_count;

        if (cog_list.empty() || cog_list.size() < cog_idx) {
            cog_vel.setZero();
            cog_acc.setZero();
            return;
        }

        for (size_t i = 0; i < 3; ++i) {
            const double ref_cog = cog_list[cur_count - cog_list_start_count][i];
            const double ref_vel = (ref_cog - cog[i]) / dt;
            cog_acc[i] = (ref_vel - cog_vel[i]) / dt;
            cog_vel[i] = ref_vel;
            cog[i]     = ref_cog;
        }
        std::cerr << "acc: " << cog_acc.transpose() << std::endl;
        new_ref_cp = calcCP();
    }

    void calcCogFromZMP(const std::deque<hrp::Vector3>& refzmp_list, const double dt);

    /**
     * @fn
     * @return reference zmp == (0, 0, 0)^T
     */
    hrp::Vector3 calcCogForFlightPhase(const double dt, const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION);

    /**
     * @fn
     * @return reference zmp
     */
    hrp::Vector3 calcCogForRun(const hrp::Vector3& support_point,
                               const hrp::Vector3& landing_point,
                               const hrp::Vector3& start_zmp_offset,
                               const hrp::Vector3& end_zmp_offset,
                               const hrp::Vector3& target_cp_offset,
                               const double take_off_z,
                               const double jump_height,
                               const size_t start_count,
                               const size_t supporting_count,
                               const size_t landing_count,
                               const size_t cur_count,
                               const double dt,
                               const FootGuidedRefZMPType ref_zmp_type = FIX,
                               const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION);
    /**
     * @fn
     * @return reference zmp
     */
    hrp::Vector3 calcFootGuidedCog(const std::vector<ConstraintsWithCount>& constraints_list,
                                   const double jump_height,
                                   const int cur_const_idx,
                                   const size_t cur_count,
                                   const double dt,
                                   const double takeoff_height_offset = 0,
                                   const double landing_height_offset = 0,
                                   const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION);


    void calcCogListForRun(const hrp::Vector3 target_cp,
                           const hrp::Vector3 ref_zmp,
                           const hrp::Vector3 next_ref_zmp,
                           const size_t count_to_jump,
                           const size_t cur_count,
                           const double jump_height,
                           const double take_off_z,
                           const double dt,
                           const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION);
    void calcCogListForRunLast(const hrp::Vector3 target_cp,
                               const hrp::Vector3 ref_zmp,
                               const hrp::Vector3 next_ref_zmp,
                               const size_t count_to_jump,
                               const size_t cur_count,
                               const double jump_height,
                               const double take_off_z,
                               const double dt,
                               const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION);

    void calcCogListForRun2Step(const hrp::Vector3 target_cp,
                                const hrp::Vector3 ref_zmp,
                                const hrp::Vector3 next_ref_zmp,
                                const hrp::Vector3 last_ref_zmp,
                                const int count_to_jump1,
                                const int count_to_jump2,
                                const size_t cur_count,
                                const double jump_height1,
                                const double jump_height2,
                                const double take_off_z1,
                                const double take_off_z2,
                                const double dt,
                                const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION);
    /**
     * @fn
     * @return reference zmp
     */
    hrp::Vector3 calcFootGuidedCogWalk(const std::vector<ConstraintsWithCount>& constraints_list,
                                       const std::vector<std::pair<hrp::Vector3, size_t>>& ref_zmp_goals,
                                       const int cur_const_idx,
                                       const size_t cur_count,
                                       const double dt,
                                       const hrp::Vector3& target_cp_offset = hrp::Vector3::Zero());
    /**
     * @fn
     * @return reference zmp
     */
    hrp::Vector3 calcFootGuidedCogWalk(const std::vector<ConstraintsWithCount>& constraints_list,
                                       const hrp::Vector3& ref_zmp,
                                       const hrp::Vector3& ref_zmp_vel,
                                       const int cur_const_idx,
                                       const size_t cur_count,
                                       const double dt,
                                       const hrp::Vector3& target_cp_offset = hrp::Vector3::Zero());

    void calcCogZForJump(const size_t count_to_jump,
                         const double jump_height,
                         const double take_off_z,
                         const double dt,
                         const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION);

    std::vector<std::tuple<double, double, double>> calcCogZListForJump(const size_t count_to_jump,
                                                                        const double jump_height,
                                                                        const double take_off_z,
                                                                        const double dt,
                                                                        const double g_acc = DEFAULT_GRAVITATIONAL_ACCELERATION);
};

}

#endif // COGTRAJECTORYGENERATOR_H
