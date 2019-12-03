// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "VirtualForceSensorService_impl.h"
#include "VirtualForceSensor.h"

VirtualForceSensorService_impl::VirtualForceSensorService_impl() : m_vfsensor(NULL)
{
}

VirtualForceSensorService_impl::~VirtualForceSensorService_impl()
{
}

void VirtualForceSensorService_impl::getParameter(OpenHRP::VirtualForceSensorService::vsParam_out i_param)
{
  i_param = OpenHRP::VirtualForceSensorService::vsParam();
  return m_vfsensor->getParameter(i_param);
};

void VirtualForceSensorService_impl::setParameter(const OpenHRP::VirtualForceSensorService::vsParam& i_stp)
{
    m_vfsensor->setParameter(i_stp);
}

CORBA::Boolean VirtualForceSensorService_impl::removeVirtualForceSensorOffset(const ::OpenHRP::VirtualForceSensorService::StrSequence& sensorNames, CORBA::Double tm)
{
	return m_vfsensor->removeVirtualForceSensorOffset(sensorNames, tm);
}

CORBA::Boolean VirtualForceSensorService_impl::removeExternalForceOffset(CORBA::Double tm)
{
	return m_vfsensor->removeExternalForceOffset(tm);
}

CORBA::Boolean VirtualForceSensorService_impl::startEstimation(const char *sensorName)
{
	return m_vfsensor->startEstimation(std::string(sensorName));
}

CORBA::Boolean VirtualForceSensorService_impl::stopEstimation(const char *sensorName)
{
	return m_vfsensor->stopEstimation(std::string(sensorName));
}

void VirtualForceSensorService_impl::vfsensor(VirtualForceSensor *i_vfsensor)
{
	m_vfsensor = i_vfsensor;
}

