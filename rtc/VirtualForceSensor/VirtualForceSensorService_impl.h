// -*-C++-*-
#ifndef IMPEDANCESERVICESVC_IMPL_H
#define IMPEDANCESERVICESVC_IMPL_H

#include "hrpsys/idl/VirtualForceSensorService.hh"

using namespace OpenHRP;

class VirtualForceSensor;

class VirtualForceSensorService_impl 
  : public virtual POA_OpenHRP::VirtualForceSensorService,
    public virtual PortableServer::RefCountServantBase
{
public:
  VirtualForceSensorService_impl();
  virtual ~VirtualForceSensorService_impl();
  //
  void getParameter(OpenHRP::VirtualForceSensorService::vsParam_out i_param);
  void setParameter(const OpenHRP::VirtualForceSensorService::vsParam& i_param);
  CORBA::Boolean removeVirtualForceSensorOffset(const ::OpenHRP::VirtualForceSensorService::StrSequence& names, CORBA::Double tm);
  CORBA::Boolean removeExternalForceOffset(CORBA::Double tm);
  CORBA::Boolean startEstimation(const char *sensorName);
  CORBA::Boolean stopEstimation(const char *sensorName);
  //
  void vfsensor(VirtualForceSensor *i_vfsensor);
private:
  VirtualForceSensor *m_vfsensor;
};				 

#endif
