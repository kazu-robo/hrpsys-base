// -*-C++-*-
#ifndef AUTOBALANCESTABILIZERSERVICESVC_IMPL_H
#define AUTOBALANCESTABILIZERSERVICESVC_IMPL_H

#include "hrpsys/idl/AutoBalanceStabilizerService.hh"

using namespace OpenHRP;

class AutoBalanceStabilizer;

class AutoBalanceStabilizerService_impl
  : public virtual POA_OpenHRP::AutoBalanceStabilizerService,
    public virtual PortableServer::RefCountServantBase
{
  private:
    AutoBalanceStabilizer *m_autobalancestabilizer;

  public:
    AutoBalanceStabilizerService_impl();
    virtual ~AutoBalanceStabilizerService_impl();
    void autobalancestabilizer(AutoBalanceStabilizer *i_autobalancestabilizer);

    /* Service for GaitGenerator */
    void printConstraintLinkIds();
    void setDefaultSingleSupportTime(const CORBA::Double time);
    void setDefaultDoubleSupportTime(const CORBA::Double time);
    void setDefaultToeSupportTime(const CORBA::Double time);
    void setDefaultHeelSupportTime(const CORBA::Double time);
    void setDefaultStepHeight(const CORBA::Double height);
    void setMaxStride(const CORBA::Double stride);
    void setMaxRotAngle(const CORBA::Double angle_rad);
    void setUseToeHeel(const CORBA::Boolean use_toe_heel);
    void setToeKickAngle(const CORBA::Double angle_rad);
    void setHeelContactAngle(const CORBA::Double angle_rad);
    CORBA::Boolean setToeContactPoints(const CORBA::Long link_id, const OpenHRP::AutoBalanceStabilizerService::DblSeq3Seq& contact_points);
    CORBA::Boolean setHeelContactPoints(const CORBA::Long link_id, const OpenHRP::AutoBalanceStabilizerService::DblSeq3Seq& contact_points);

    void setWalkingMode(const OpenHRP::AutoBalanceStabilizerService::WalkingMode mode);

    CORBA::Boolean goPos( CORBA::Double x,  CORBA::Double y,  CORBA::Double th);
    // CORBA::Boolean goVelocity( CORBA::Double vx,  CORBA::Double vy,  CORBA::Double vth);
    CORBA::Boolean goStop();
    // CORBA::Boolean emergencyStop();
    // CORBA::Boolean setFootSteps(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx);
    CORBA::Boolean setFootSteps(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss);
    // CORBA::Boolean setFootStepsWithParam(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss, const OpenHRP::AutoBalanceStabilizerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx);
    CORBA::Boolean setRunningFootSteps(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss);
    CORBA::Boolean setJumpingFootSteps(const OpenHRP::AutoBalanceStabilizerService::FootstepsSequence& fss);
    void waitFootSteps();
    // void waitFootStepsEarly(CORBA::Double tm);
    CORBA::Boolean startAutoBalancer(const OpenHRP::AutoBalanceStabilizerService::StrSequence& limbs);
    CORBA::Boolean stopAutoBalancer();

    CORBA::Boolean setGaitGeneratorParam(const OpenHRP::AutoBalanceStabilizerService::GaitGeneratorParam& i_param);
    CORBA::Boolean getGaitGeneratorParam(OpenHRP::AutoBalanceStabilizerService::GaitGeneratorParam_out i_param);
    CORBA::Boolean setAutoBalancerParam(const OpenHRP::AutoBalanceStabilizerService::AutoBalancerParam& i_param);
    CORBA::Boolean getAutoBalancerParam(OpenHRP::AutoBalanceStabilizerService::AutoBalancerParam_out i_param);
    CORBA::Boolean getFootstepParam(OpenHRP::AutoBalanceStabilizerService::FootstepParam_out i_param);
    // CORBA::Boolean adjustFootSteps(const OpenHRP::AutoBalanceStabilizerService::Footstep& rfootstep, const OpenHRP::AutoBalanceStabilizerService::Footstep& lfootstep);
    // CORBA::Boolean getRemainingFootstepSequence(OpenHRP::AutoBalanceStabilizerService::FootstepSequence_out o_footstep , CORBA::Long& o_current_fs_idx);
    // CORBA::Boolean getGoPosFootstepsSequence(CORBA::Double x, CORBA::Double y, CORBA::Double th, OpenHRP::AutoBalanceStabilizerService::FootstepsSequence_out o_footstep);
    CORBA::Boolean releaseEmergencyStop();
    void testMotion(const CORBA::Long test_number);

    /* Service for Stabilizer */
    void startStabilizer(void);
    void stopStabilizer(void);
    void setStabilizerParam(const OpenHRP::AutoBalanceStabilizerService::StabilizerParam& i_param);
    void getStabilizerParam(OpenHRP::AutoBalanceStabilizerService::StabilizerParam_out i_param);
};

#endif
