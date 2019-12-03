// -*- C++ -*-
/*!
 * @file  VirtualForceSensor.cpp
 * @brief virtual force sensor component
 * $Date$
 *
 * $Id$
 */

#include "VirtualForceSensor.h"
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpUtil/MatrixSolvers.h>

typedef coil::Guard<coil::Mutex> Guard;

#define VS_DEBUG false

// Module specification
// <rtc-template block="module_spec">
static const char* virtualforcesensor_spec[] =
  {
    "implementation_id", "VirtualForceSensor",
    "type_name",         "VirtualForceSensor",
    "description",       "null component",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debugLevel", "0",
    ""
  };
// </rtc-template>

VirtualForceSensor::VirtualForceSensor(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_tauInIn("tauIn", m_tauIn),
    m_baseRpyIn("baseRpy", m_baseRpy),
    m_VirtualForceSensorServicePort("VirtualForceSensorService"),
    // </rtc-template>
    m_debugLevel(0)
{
  m_service0.vfsensor(this);
}

VirtualForceSensor::~VirtualForceSensor()
{
}



RTC::ReturnCode_t VirtualForceSensor::onInitialize()
{
  std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qCurrent", m_qCurrentIn);
  addInPort("tauIn", m_tauInIn);
  addInPort("baseRpy", m_baseRpyIn);

  // Set service provider to Ports
  m_VirtualForceSensorServicePort.registerProvider("service0", "VirtualForceSensorService", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_VirtualForceSensorServicePort);
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();
  coil::stringTo(m_dt, prop["dt"].c_str());

  m_robot = hrp::BodyPtr(new hrp::Body());

  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
      comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(),
			       CosNaming::NamingContext::_duplicate(naming.getRootContext())
	  )){
    std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "] in "
              << m_profile.instance_name << std::endl;
    return RTC::RTC_ERROR;
  }
  m_robot->calcTotalMass();

  // Setting for wrench data ports (real)
  std::vector<std::string> force_sensor_names;
  // Find names for real force sensors
  int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
  for (unsigned int i=0; i<npforce; ++i) {
      force_sensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
  }
  m_wrenches.resize(npforce);
  m_wrenchesIn.resize(npforce);
  wrenchFilter.resize(npforce);
  std::cerr << "[" << m_profile.instance_name << "] force sensor ports (" << npforce << ")" << std::endl;
  for (unsigned int i=0; i<npforce; ++i) {
      std::string force_sensor_name = force_sensor_names[i];
      // actual inport
      m_wrenchesIn[i] = new RTC::InPort<RTC::TimedDoubleSeq>(force_sensor_name.c_str(), m_wrenches[i]);
      m_wrenches[i].data.length(6);
      registerInPort(force_sensor_name.c_str(), *m_wrenchesIn[i]);
      wrenchFilter[i] = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector6> >(new FirstOrderLowPassFilter<hrp::dvector6>(1.0, m_dt, hrp::dvector6::Zero())); // [Hz]
      std::cerr << "[" << m_profile.instance_name << "]   name = " << force_sensor_name << std::endl;
  }

  
  // virtual_force_sensor2: <name>, <target>, <localpos>, <localaxis>, <angle>, <friction_coefficient>, <rotation_friction_coefficient>, <upper_cop_x_margin>, <lower_cop_x_margin>, <upper_cop_y_margin>, <lower_cop_y_margin>
  coil::vstring virtual_force_sensor = coil::split(prop["virtual_force_sensor2"], ",");
  for(unsigned int i = 0; i < virtual_force_sensor.size()/15; i++ ){
    std::string name = virtual_force_sensor[i*15+0];
    boost::shared_ptr<VirtualForceSensorParam> p(new VirtualForceSensorParam());
    p->is_enable = false;
    p->target_name = virtual_force_sensor[i*15+1];
    hrp::dvector tr(7);
    for (int j = 0; j < 7; j++ ) {
      coil::stringTo(tr[j], virtual_force_sensor[i*15+2+j].c_str());
    }
    p->p = hrp::Vector3(tr[0], tr[1], tr[2]);
    p->R = Eigen::AngleAxis<double>(tr[6], hrp::Vector3(tr[3],tr[4],tr[5])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
    p->forceOffset = hrp::Vector3(0, 0, 0);
    p->momentOffset = hrp::Vector3(0, 0, 0);
    std::cerr << "[" << m_profile.instance_name << "] virtual force sensor : " << name << std::endl;
    std::cerr << "[" << m_profile.instance_name << "]               target : " << p->target_name << std::endl;
    std::cerr << "[" << m_profile.instance_name << "]                 T, R : " << p->p[0] << " " << p->p[1] << " " << p->p[2] << std::endl << p->R << std::endl;
    //p->path = hrp::JointPathPtr(new hrp::JointPath(m_robot->link(p->base_name), m_robot->link(p->target_name)));
    p->path = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->rootLink(), m_robot->link(p->target_name), m_dt, false, std::string(m_profile.instance_name)));
    coil::stringTo(p->friction_coefficient, virtual_force_sensor[i*15+9].c_str());
    coil::stringTo(p->rotation_friction_coefficient, virtual_force_sensor[i*15+10].c_str());
    coil::stringTo(p->upper_cop_x_margin, virtual_force_sensor[i*15+11].c_str());
    coil::stringTo(p->lower_cop_x_margin, virtual_force_sensor[i*15+12].c_str());
    coil::stringTo(p->upper_cop_y_margin, virtual_force_sensor[i*15+13].c_str());
    coil::stringTo(p->lower_cop_y_margin, virtual_force_sensor[i*15+14].c_str());
    std::cerr << "[" << m_profile.instance_name << "]               upper_cop_x_margin : " << p->upper_cop_x_margin << std::endl;
    std::cerr << "[" << m_profile.instance_name << "]               lower_cop_x_margin : " << p->lower_cop_x_margin << std::endl;
    std::cerr << "[" << m_profile.instance_name << "]               upper_cop_y_margin : " << p->upper_cop_y_margin << std::endl;
    std::cerr << "[" << m_profile.instance_name << "]               lower_cop_y_margin : " << p->lower_cop_y_margin << std::endl;
    p->off_sensor_force_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(10, m_dt, hrp::Vector3::Zero())); // [Hz]
    p->off_sensor_moment_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(10, m_dt, hrp::Vector3::Zero())); // [Hz]
    m_sensors[name] = p;
    // if ( m_sensors[name]->path->numJoints() == 0 ) {
    //   std::cerr << "[" << m_profile.instance_name << "] ERROR : Unknown link path " << m_sensors[name]->target_name  << std::endl;
    //   return RTC::RTC_ERROR;
    // }
  }
  int nforce = m_sensors.size();
  m_force.resize(nforce);
  m_forceOut.resize(nforce);
  m_offforce.resize(nforce);
  m_offforceOut.resize(nforce);
  int i = 0;
  std::map<std::string, boost::shared_ptr<VirtualForceSensorParam> >::iterator it = m_sensors.begin();
  while ( it != m_sensors.end() ) {
    m_forceOut[i] = new OutPort<TimedDoubleSeq>((*it).first.c_str(), m_force[i]);
    m_force[i].data.length(6);
    registerOutPort((*it).first.c_str(), *m_forceOut[i]);
    m_offforceOut[i] = new OutPort<TimedDoubleSeq>(("off_" + (*it).first).c_str(), m_offforce[i]);
    m_offforce[i].data.length(6);
    registerOutPort(("off_" + (*it).first).c_str(), *m_offforceOut[i]);
    it++; i++;
  }
  
  qCurrentFilter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(250, m_dt, hrp::dvector::Zero(m_robot->numJoints()))); // [Hz]
  dqCurrentFilter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, m_dt, hrp::dvector::Zero(m_robot->numJoints()))); // [Hz]
  ddqCurrentFilter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, m_dt, hrp::dvector::Zero(m_robot->numJoints()))); // [Hz]
  basewFilter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, m_dt, hrp::Vector3::Zero())); // [Hz]
  basedwFilter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, m_dt, hrp::Vector3::Zero())); // [Hz]
  tauFilter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(1, m_dt, hrp::dvector::Zero(m_robot->numJoints()))); // [Hz]
  qprev = hrp::dvector::Zero(m_robot->numJoints());
  dqprev = hrp::dvector::Zero(m_robot->numJoints());
  baseRprev = hrp::Matrix33::Identity();
  basewprev = hrp::Vector3::Zero();
  extforceOffset = hrp::Vector3::Zero();
  extmomentOffset = hrp::Vector3::Zero();
  exttorqueOffset = hrp::dvector::Zero(m_robot->numJoints());
  extforce_offset_calib_counter = 0;
  sem_init(&extforce_wait_sem, 0, 0);
  for (size_t i = 0; i < m_robot->numSensors(hrp::Sensor::FORCE); i++){
      jpe_v.push_back(hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->rootLink(), m_robot->sensor(hrp::Sensor::FORCE, i)->link, m_dt, false, std::string(m_profile.instance_name))));
  }

  root_force_weight = 10000.0;
  root_moment_weight = 100.0;
  joint_torque_weight = 1.0;
  weight = 1e-6;

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t VirtualForceSensor::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VirtualForceSensor::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VirtualForceSensor::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t VirtualForceSensor::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t VirtualForceSensor::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t VirtualForceSensor::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  static int loop = 0;
  loop ++;

  if (m_qCurrentIn.isNew()) {
    m_qCurrentIn.read();
  }
  if (m_tauInIn.isNew()) {
    m_tauInIn.read();
  }
  if (m_baseRpyIn.isNew()) {
    m_baseRpyIn.read();
  }
  for (size_t i = 0; i < m_wrenchesIn.size(); ++i) {
      if ( m_wrenchesIn[i]->isNew() ) {
          m_wrenchesIn[i]->read();
      }
  }

  RTC::Time tm = m_qCurrent.tm;

  if ( m_qCurrent.data.length() ==  m_robot->numJoints() &&
       m_tauIn.data.length() ==  m_robot->numJoints()) {
    // robotの状態の更新
    hrp::dvector qCurrent = hrp::dvector::Zero(m_robot->numJoints());
    for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
      qCurrent[i] = m_qCurrent.data[i];
    }
    qCurrent = qCurrentFilter->passFilter(qCurrent);
    for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q = qCurrent[i];
    }
    hrp::dvector dqCurrent(m_robot->numJoints());
    for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
        dqCurrent[i] = 0.0;//(qCurrent[i] - qprev[i])/dt; TODO
    }
    qprev = qCurrent;
    dqCurrent = dqCurrentFilter->passFilter(dqCurrent);
    for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->dq = dqCurrent[i];
    }
    hrp::dvector ddqCurrent = (dqCurrent - dqprev) / m_dt;
    dqprev = dqCurrent;
    ddqCurrent = ddqCurrentFilter->passFilter(ddqCurrent);
    for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->ddq = ddqCurrent[i];
    }
    hrp::Matrix33 baseR = hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
    //hrp::Vector3 basew = rats::matrix_log( baseR * baseRprev.transpose() ) / m_dt; TODO
    hrp::Vector3 basew = hrp::Vector3::Zero();
    basew = basewFilter->passFilter(basew);
    hrp::Vector3 basedw = (basew - basewprev) /m_dt;
    basedw = basedwFilter->passFilter(basedw);
    baseRprev = baseR;
    basewprev = basew;
    m_robot->rootLink()->R = baseR;
    m_robot->rootLink()->w = basew;
    m_robot->rootLink()->dw = basedw;
    
    m_robot->calcForwardKinematics();

    m_robot->rootLink()->p = hrp::Vector3::Zero();
    hrp::Vector3 basev = hrp::Vector3::Zero();//TODO
    m_robot->rootLink()->v = basev;
    hrp::Vector3 basedv = hrp::Vector3::Zero();//TODO
    m_robot->rootLink()->dv = basedv;
    m_robot->calcForwardKinematics(true,true);

    for(size_t i =0 ; i< m_robot->numLinks();i++){//voはFKで自動で計算されない
        m_robot->link(i)->vo = m_robot->link(i)->v - m_robot->link(i)->w.cross(m_robot->link(i)->p);
    }
    hrp::Vector3 g(0, 0, 9.80665);
    m_robot->rootLink()->dvo = g + m_robot->rootLink()->dv - m_robot->rootLink()->dw.cross(m_robot->rootLink()->p) - m_robot->rootLink()->w.cross(m_robot->rootLink()->vo + m_robot->rootLink()->w.cross(m_robot->rootLink()->p));

    hrp::dvector Tvirtual = hrp::dvector::Zero(6+m_robot->numJoints());/*actworld系 : actworld系,原点周り: 関節*/

    //反力無しの場合に要するトルク
    hrp::Vector3 base_f;/*actworld系*/
    hrp::Vector3 base_t;/*actworld系,原点周り*/
    m_robot->calcInverseDynamics(m_robot->rootLink(), base_f, base_t);
    hrp::dvector T0 = hrp::dvector::Zero(6+m_robot->numJoints());
    T0.block<3,1>(0,0) = base_f;
    T0.block<3,1>(3,0) = base_t;
    for (size_t i = 0; i < m_robot->numJoints() ; i++){
        T0[6+i] = m_robot->joint(i)->u;
    }
    Tvirtual += T0;

    if(VS_DEBUG){
        std::cout << "左辺" <<std::endl;
        std::cout << Tvirtual <<std::endl;
    }

    //実際のトルク
    hrp::dvector T = hrp::dvector::Zero(6+m_robot->numJoints());
    hrp::dvector tau = hrp::dvector::Zero(m_robot->numJoints());
    for (size_t i = 0; i < m_robot->numJoints() ; i++){
        tau[i] = m_tauIn.data[i];
    }
    tau = tauFilter->passFilter(tau);
    T.block(6,0,m_robot->numJoints(),1) = tau;
    Tvirtual -= T;

    if(VS_DEBUG){
        std::cout << "tau" <<std::endl;
        std::cout << Tvirtual <<std::endl;
    }

    //各反力に相当するトルク
    for (size_t i = 0; i < jpe_v.size() ; i++){
        hrp::Sensor* sensor = m_robot->sensor(hrp::Sensor::FORCE, i);
        hrp::dmatrix JJ;
        jpe_v[i]->calcJacobian(JJ,sensor->localPos);
        hrp::dmatrix J = hrp::dmatrix::Zero(6,6+m_robot->numJoints());
        J.block<3,3>(0,0) = hrp::Matrix33::Identity();
        J.block<3,3>(0,3) = - hrp::hat(sensor->link->p + sensor->link->R * sensor->localPos);
        J.block<3,3>(3,3) = hrp::Matrix33::Identity();
        for (int j = 0; j < jpe_v[i]->numJoints() ; j++){
            J.block<6,1>(0,6+jpe_v[i]->joint(j)->jointId) = JJ.block<6,1>(0,j);
        }

        hrp::dvector6 wrench_filtered;
        wrench_filtered << m_wrenches[i].data[0],m_wrenches[i].data[1],m_wrenches[i].data[2],m_wrenches[i].data[3],m_wrenches[i].data[4],m_wrenches[i].data[5];
        wrench_filtered = wrenchFilter[i]->passFilter(wrench_filtered);
        hrp::dvector6 wrench;
        wrench.block<3,1>(0,0) = (sensor->link->R * sensor->localR) * wrench_filtered.head<3>();
        wrench.block<3,1>(3,0) = (sensor->link->R * sensor->localR) * wrench_filtered.tail<3>();

        Tvirtual -= J.transpose() * wrench;
    }

    if(VS_DEBUG){
        std::cout << "Tvirtual" <<std::endl;
        std::cout << Tvirtual <<std::endl;
    }

    //外力のオフセット除去
    hrp::Vector3 CM/*actworld系*/ = m_robot->calcCM();
    hrp::Vector3 basecoords_p;/*actworld系*/
    hrp::Matrix33 basecoords_R;/*actworld系,Z軸鉛直*/
    {
        const hrp::Vector3 xv/*actworld系*/(m_robot->rootLink()->R * hrp::Vector3::UnitX()/*eef系*/);
        const hrp::Vector3 yv/*actworld系*/(m_robot->rootLink()->R * hrp::Vector3::UnitY()/*eef系*/);
        // atan2(y,x) = atan(y/x)
        const double yaw = atan2(xv[1]-yv[0],xv[0]+yv[1]);
        basecoords_p =m_robot->rootLink()->p;
        basecoords_R = hrp::rotFromRpy(hrp::Vector3(0.0,0.0,yaw));
    }
    if(extforce_offset_calib_counter > 0){// while calibrating
        extforceOffset_sum/*basecoords系*/ += basecoords_R/*actworld系*/.transpose() * Tvirtual.block<3,1>(0,0)/*actworld系*/;
        extmomentOffset_sum/*basecoords系, 重心周り*/ += basecoords_R/*actworld系*/.transpose() * (Tvirtual.block<3,1>(3,0)/*actworld系,原点周り*/ + (-CM/*actworld系*/).cross(Tvirtual.block<3,1>(0,0)/*actworld系*/));
        exttorqueOffset_sum += Tvirtual.block(6,0,m_robot->numJoints(),1);
        extforce_offset_calib_counter--;
        if (extforce_offset_calib_counter == 0){
            extforceOffset/*basecoords系*/ = extforceOffset_sum / max_extforce_offset_calib_counter;
            extmomentOffset/*basecoords系, 重心周り*/ = extmomentOffset_sum / max_extforce_offset_calib_counter;
            exttorqueOffset = exttorqueOffset_sum / max_extforce_offset_calib_counter;
            sem_post(&extforce_wait_sem);
        }
    }
    Tvirtual.block<3,1>(0,0)/*actworld系*/ -= basecoords_R/*actworld系*/ * extforceOffset/*basecoords系*/;
    Tvirtual.block<3,1>(3,0)/*actworld系,原点周り*/ -= basecoords_R/*actworld系*/ * extmomentOffset/*basecoords系, 重心周り*/ + CM/*actworld系*/.cross(basecoords_R/*actworld系*/ * extforceOffset/*basecoords系*/);
    Tvirtual.block(6,0,m_robot->numJoints(),1) -= exttorqueOffset;

    if(VS_DEBUG){
        std::cout << "off Tvirtual" <<std::endl;
        std::cout << Tvirtual <<std::endl;
    }


    std::vector<boost::shared_ptr<VirtualForceSensorParam> > enable_sensors;
    std::map<std::string, boost::shared_ptr<VirtualForceSensorParam> >::iterator it = m_sensors.begin();
    for (size_t i = 0 ; i < m_sensors.size(); i++){
        if((*it).second->is_enable){
            enable_sensors.push_back((*it).second);
        }else{
            (*it).second->sensor_force = hrp::Vector3::Zero();
            (*it).second->sensor_moment = hrp::Vector3::Zero();
        }
        it++;
    }

    //USE_QPOASES を ON にすること
    bool qp_solved=false;
    hrp::dvector virtual_wrench=hrp::dvector::Zero(6 * enable_sensors.size())/*sensor系,sensor周り*/;
    if(enable_sensors.size()!=0){
        /****************************************************************/
        //virtual sensor入力を推定する
        hrp::dmatrix H = hrp::dmatrix::Zero(6 * enable_sensors.size(),6 * enable_sensors.size());
        hrp::dmatrix g = hrp::dmatrix::Zero(1,6 * enable_sensors.size());
        std::vector<hrp::dmatrix> As;
        std::vector<hrp::dvector> lbAs;
        hrp::dvector lb;
        hrp::dvector ub;

        hrp::dmatrix J = hrp::dmatrix::Zero(6 * enable_sensors.size(),6+m_robot->numJoints());
        {
            for (size_t i = 0 ; i < enable_sensors.size(); i++){
                hrp::dmatrix JJ;
                enable_sensors[i]->path->calcJacobian(JJ, enable_sensors[i]->p);
                hrp::Matrix33 senRt = (m_robot->link(enable_sensors[i]->target_name)->R * enable_sensors[i]->R).transpose();
                J.block<3,3>(i*6,0) = senRt;
                J.block<3,3>(i*6,3) = senRt * - hrp::hat(m_robot->link(enable_sensors[i]->target_name)->p + m_robot->link(enable_sensors[i]->target_name)->R * enable_sensors[i]->p);
                J.block<3,3>(i*6+3,3) = senRt;
                for (int j = 0; j < enable_sensors[i]->path->numJoints() ; j++){
                    J.block<3,1>(i*6,6+enable_sensors[i]->path->joint(j)->jointId) = senRt * JJ.block<3,1>(0,j);
                    J.block<3,1>(i*6+3,6+enable_sensors[i]->path->joint(j)->jointId) = senRt * JJ.block<3,1>(3,j);
                }
            }
        }
        hrp::dmatrix W = hrp::dmatrix::Zero(6+m_robot->numJoints(),6+m_robot->numJoints());
        for(size_t i=0; i < 3; i++){
            W(i,i) = root_force_weight;
        }
        for(size_t i=0; i < 3; i++){
            W(3+i,3+i) = root_moment_weight;
        }
        for(size_t i=0; i < m_robot->numJoints(); i++){
            W(6+i,6+i) = joint_torque_weight;
        }

        H = J * W * J.transpose();
        g = - J * W * Tvirtual;

        for(size_t i = 0; i < H.cols(); i++){
            H(i,i) += weight;
        }

        {
            hrp::dmatrix A = hrp::dmatrix::Zero(11 * enable_sensors.size(),6 * enable_sensors.size());
            for (size_t i = 0 ; i < enable_sensors.size(); i++ ){
                A(i*11+0,i*6+2) = 1;
                A(i*11+1,i*6+0) = -1;
                A(i*11+1,i*6+2) = enable_sensors[i]->friction_coefficient;
                A(i*11+2,i*6+0) = 1;
                A(i*11+2,i*6+2) = enable_sensors[i]->friction_coefficient;
                A(i*11+3,i*6+1) = -1;
                A(i*11+3,i*6+2) = enable_sensors[i]->friction_coefficient;
                A(i*11+4,i*6+1) = 1;
                A(i*11+4,i*6+2) = enable_sensors[i]->friction_coefficient;
                A(i*11+5,i*6+3) = -1;
                A(i*11+5,i*6+2) = enable_sensors[i]->upper_cop_y_margin;
                A(i*11+6,i*6+3) = 1;
                A(i*11+6,i*6+2) = enable_sensors[i]->lower_cop_y_margin;
                A(i*11+7,i*6+4) = -1;
                A(i*11+7,i*6+2) = enable_sensors[i]->lower_cop_x_margin;
                A(i*11+8,i*6+4) = 1;
                A(i*11+8,i*6+2) = enable_sensors[i]->upper_cop_x_margin;
                A(i*11+9,i*6+5) = -1;
                A(i*11+9,i*6+2) = enable_sensors[i]->rotation_friction_coefficient;
                A(i*11+10,i*6+5) = 1;
                A(i*11+10,i*6+2) = enable_sensors[i]->rotation_friction_coefficient;
            }
            if(VS_DEBUG){
                std::cerr << "A" <<std::endl;
                std::cerr << A <<std::endl;
            }
            As.push_back(A);
        }
        {
            hrp::dvector lbA = hrp::dvector::Zero(enable_sensors.size() * 11);
            for (size_t i = 0; i < enable_sensors.size() * 11; i++){
                lbA[i] = 0.0;
            }
            lbAs.push_back(lbA);
        }

        /*****************************************************************/
        
        size_t state_len = 6 * enable_sensors.size();
        size_t inequality_len = 11 * enable_sensors.size();
        qpOASES::real_t* qp_H = new qpOASES::real_t[state_len * state_len];// 0.5 xt H x + xt g が目的関数であることに注意
        qpOASES::real_t* qp_A = new qpOASES::real_t[inequality_len * state_len];
        qpOASES::real_t* qp_g = new qpOASES::real_t[state_len];// 0.5 xt H x + xt g が目的関数であることに注意
        qpOASES::real_t* qp_ub = NULL;
        qpOASES::real_t* qp_lb = NULL;
        qpOASES::real_t* qp_ubA = NULL;
        qpOASES::real_t* qp_lbA = new qpOASES::real_t[inequality_len];

        for (size_t i = 0; i < state_len; i++) {
            for(size_t j = 0; j < state_len; j++){ 
                qp_H[i*state_len + j] = H(i,j);
            }
        }
        for (size_t i = 0; i < state_len; i++) {
            qp_g[i] = g(i,0);
            //qp_lb[i] = lb[i];
            //qp_ub[i] = ub[i];
        }
        {
            size_t inequality_idx = 0; 
            for (size_t i = 0; i < As.size(); i++) {
                for(size_t j = 0; j < As[i].rows() ; j++){
                    for(size_t k = 0; k < state_len; k++){ 
                        qp_A[state_len*inequality_idx + k] = As[i](j,k);
                    }
                    qp_lbA[inequality_idx] = lbAs[i][j];
                    inequality_idx++;
                }
            }
        }

        qpOASES::Options options;
        //options.enableFlippingBounds = qpOASES::BT_TRUE;
        options.initialStatusBounds = qpOASES::ST_INACTIVE;
        options.numRefinementSteps = 1;
        options.enableCholeskyRefactorisation = 1;
        if(VS_DEBUG){
            options.printLevel = qpOASES::PL_HIGH;
        }else{
            options.printLevel = qpOASES::PL_NONE;
        }
            
        //copied from eus_qpoases
        boost::shared_ptr<qpOASES::SQProblem> example;
        std::pair<int, int> tmp_pair(state_len, inequality_len);
        bool is_initial = true;
        bool internal_error = false;
        {
            std::map<std::pair<int, int>, boost::shared_ptr<qpOASES::SQProblem> >::iterator it = sqp_map.find(tmp_pair);
            is_initial = (it == sqp_map.end());
            if(!is_initial){
                example = it->second;
            }
        }
        if (!is_initial) {
            example->setOptions( options );
            int nWSR = 100;
            
            qpOASES::returnValue status = example->hotstart( qp_H,qp_g,qp_A,qp_lb,qp_ub,qp_lbA,qp_ubA, nWSR);
            
            if(qpOASES::getSimpleStatus(status)==0){
                if(VS_DEBUG){
                    std::cerr << "hotstart qp_solved" <<std::endl;
                }
                
                qp_solved=true;
                qpOASES::real_t* xOpt = new qpOASES::real_t[state_len];
                example->getPrimalSolution( xOpt );
                for(size_t i=0; i<state_len;i++){
                    virtual_wrench[i]=xOpt[i];
                }
                delete[] xOpt;
            }else{
                if(VS_DEBUG){
                    std::cerr << "hotstart qp fail" <<std::endl;
                }
                // Delete unsolved sqp
                sqp_map.erase(tmp_pair);
                if(qpOASES::getSimpleStatus(status)==-1){
                    if(VS_DEBUG){
                        std::cerr << "hotstart qp internal error" <<std::endl;
                    }
                    internal_error = true;
                }
            }
        }
        
        if(is_initial || internal_error){
            example = boost::shared_ptr<qpOASES::SQProblem>(new qpOASES::SQProblem ( state_len,inequality_len, qpOASES::HST_UNKNOWN));
            //sqp_map.insert(std::pair<std::pair<int, int>, boost::shared_ptr<qpOASES::SQProblem> >(tmp_pair, example));
            sqp_map[tmp_pair]=example;
            example->setOptions( options );
            int nWSR = 100;
            
            qpOASES::returnValue status = example->init( qp_H,qp_g,qp_A,qp_lb,qp_ub,qp_lbA,qp_ubA, nWSR);
            
            if(qpOASES::getSimpleStatus(status)==0){
                if(VS_DEBUG){
                    std::cerr << "initial qp_solved" <<std::endl;
                }
                
                qp_solved=true;
                qpOASES::real_t* xOpt = new qpOASES::real_t[state_len];
                example->getPrimalSolution( xOpt );
                for(size_t i=0; i<state_len;i++){
                    virtual_wrench[i]=xOpt[i];
                }
                delete[] xOpt;
            }else{
                if(VS_DEBUG){
                    std::cerr << "initial qp fail" <<std::endl;
                }
                // Delete unsolved sqp
                sqp_map.erase(tmp_pair);
            }
        }
        delete[] qp_H;
        delete[] qp_A;
        delete[] qp_g;
        delete[] qp_ub;
        delete[] qp_lb;
        delete[] qp_ubA;
        delete[] qp_lbA;
    }else{
        qp_solved=true;
    }

    if(qp_solved){
        if(VS_DEBUG){
            std::cerr << "virtual_wrench" <<std::endl;
            std::cerr << virtual_wrench <<std::endl;
        }

        for(size_t i = 0 ; i < enable_sensors.size(); i++){
            enable_sensors[i]->sensor_force = virtual_wrench.block<3,1>(i*6,0);
            enable_sensors[i]->sensor_moment = virtual_wrench.block<3,1>(i*6+3,0);
        }        
    }else{
        std::cerr << "[" << m_profile.instance_name << "] QP not solved" <<std::endl;
    }

    Guard guard(m_mutex);
    {
        std::map<std::string, boost::shared_ptr<VirtualForceSensorParam> >::iterator it = m_sensors.begin();
        for ( size_t i = 0; i < m_force.size(); i ++ ) {
            for (size_t j = 0; j < 3; j++){
                m_force[i].data[j+0] = (*it).second->sensor_force[j];
                m_force[i].data[j+3] = (*it).second->sensor_moment[j];
            }
            m_force[i].tm = tm; // put timestamp
            m_forceOut[i]->write();
            
            //offset
            if((*it).second->max_offset_calib_counter > 0){// while calibrating
                (*it).second->forceOffset_sum/*sensor系*/ += (*it).second->sensor_force/*sensor系*/;
                (*it).second->momentOffset_sum/*sensor系,sensor周り*/ += (*it).second->sensor_moment/*sensor系,sensor周り*/;
                
                (*it).second->offset_calib_counter--;
                if ((*it).second->offset_calib_counter == 0){
                    (*it).second->forceOffset/*sensor系*/ = (*it).second->forceOffset_sum / (*it).second->max_offset_calib_counter;
                    (*it).second->momentOffset/*sensor系, 重心周り*/ = (*it).second->momentOffset_sum / (*it).second->max_offset_calib_counter;
                    sem_post(&((*it).second->wait_sem));
                }
            }
            
            if((*it).second->is_enable){
                (*it).second->off_sensor_force = (*it).second->off_sensor_force_filter->passFilter( (*it).second->sensor_force - (*it).second->forceOffset);
                (*it).second->off_sensor_moment = (*it).second->off_sensor_moment_filter->passFilter( (*it).second->sensor_moment - (*it).second->momentOffset);
            }else{
                (*it).second->off_sensor_force = hrp::Vector3::Zero();
                (*it).second->off_sensor_moment = hrp::Vector3::Zero();
            }
            
            for (size_t j = 0; j < 3; j++){
                m_offforce[i].data[j+0] = (*it).second->off_sensor_force[j];
                m_offforce[i].data[j+3] = (*it).second->off_sensor_moment[j];
            }
            m_offforce[i].tm = tm; // put timestamp
            m_offforceOut[i]->write();
            it++;
        }
    }
    
  }
  return RTC::RTC_OK;  
}
  
/*
RTC::ReturnCode_t VirtualForceSensor::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VirtualForceSensor::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VirtualForceSensor::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VirtualForceSensor::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VirtualForceSensor::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

void VirtualForceSensor::setParameter(const OpenHRP::VirtualForceSensorService::vsParam& i_stp)
{
    root_force_weight = i_stp.root_force_weight;
    std::cerr << "[" << m_profile.instance_name << "] root_force_weight " << root_force_weight << std::endl;
    root_moment_weight = i_stp.root_moment_weight;
    std::cerr << "[" << m_profile.instance_name << "] root_moment_weight " << root_moment_weight << std::endl;
    joint_torque_weight = i_stp.joint_torque_weight;
    std::cerr << "[" << m_profile.instance_name << "] joint_torque_weight " << joint_torque_weight << std::endl;
    weight = i_stp.weight;
    std::cerr << "[" << m_profile.instance_name << "] weight " << weight << std::endl;

    qCurrentFilter->setCutOffFreq(i_stp.q_cutoff_freq);
    std::cerr << "[" << m_profile.instance_name << "]  q_cutoff_freq = " << qCurrentFilter->getCutOffFreq() << std::endl;

    tauFilter->setCutOffFreq(i_stp.tau_cutoff_freq);
    std::cerr << "[" << m_profile.instance_name << "]  tau_cutoff_freq = " << tauFilter->getCutOffFreq() << std::endl;

    for(size_t i=0; i < wrenchFilter.size() ; i++){
        wrenchFilter[i]->setCutOffFreq(i_stp.wrench_cutoff_freq);
    }
    std::cerr << "[" << m_profile.instance_name << "]  wrench_cutoff_freq = " << i_stp.wrench_cutoff_freq << std::endl;

    std::map<std::string, boost::shared_ptr<VirtualForceSensorParam> >::iterator it = m_sensors.begin();
    while ( it != m_sensors.end() ) {
        (*it).second->off_sensor_force_filter->setCutOffFreq(i_stp.out_cutoff_freq);
        (*it).second->off_sensor_moment_filter->setCutOffFreq(i_stp.out_cutoff_freq);
        it++;
    }
    std::cerr << "[" << m_profile.instance_name << "]  out_cutoff_freq = " << i_stp.out_cutoff_freq << std::endl;
}

void VirtualForceSensor::getParameter(OpenHRP::VirtualForceSensorService::vsParam& i_stp)
{
    i_stp.root_force_weight = root_force_weight;
    i_stp.root_moment_weight = root_moment_weight;
    i_stp.joint_torque_weight = joint_torque_weight;
    i_stp.weight = weight;

    i_stp.q_cutoff_freq = qCurrentFilter->getCutOffFreq();
    i_stp.tau_cutoff_freq = tauFilter->getCutOffFreq();
    if(wrenchFilter.size()>0){
        i_stp.wrench_cutoff_freq = wrenchFilter[0]->getCutOffFreq();
    }else{
        i_stp.wrench_cutoff_freq = 1.0;
    }
    if(m_sensors.size()>0){
        i_stp.out_cutoff_freq = (*m_sensors.begin()).second->off_sensor_force_filter->getCutOffFreq();
    }else{
        i_stp.out_cutoff_freq = 10;
    }
}

bool VirtualForceSensor::removeVirtualForceSensorOffset(const ::OpenHRP::VirtualForceSensorService::StrSequence& sensorNames, const double tm)
{
  std::cerr << "[" << m_profile.instance_name << "] removeVirtualForceSensorOffset..." << std::endl;

  // Check argument validity
  std::vector<std::string> valid_names, invalid_names, calibrating_names;
  bool is_valid_argument = true;
  {
      Guard guard(m_mutex);
      if ( sensorNames.length() == 0 ) { // If no sensor names are specified, calibrate all sensors.
          std::cerr << "[" << m_profile.instance_name << "]   No sensor names are specified, calibrate all sensors = [";
          for ( std::map<std::string, boost::shared_ptr<VirtualForceSensorParam> >::iterator it = m_sensors.begin(); it != m_sensors.end(); it++ ) {
              valid_names.push_back(it->first);
              std::cerr << it->first << " ";
          }
          std::cerr << "]" << std::endl;
      } else {
          for (size_t i = 0; i < sensorNames.length(); i++) {
              std::string name(sensorNames[i]);
              if ( m_sensors.find(name) != m_sensors.end() ) {
                  if ( m_sensors[name]->offset_calib_counter == 0 ) {
                      valid_names.push_back(name);
                  } else {
                      calibrating_names.push_back(name);
                      is_valid_argument = false;
                  }
              } else{
                  invalid_names.push_back(name);
                  is_valid_argument = false;
              }
          }
      }
  }
  // Return if invalid or calibrating
  if ( !is_valid_argument ) {
      std::cerr << "[" << m_profile.instance_name << "]   Cannot start removeVirtualForceSensorOffset, invalid = [";
      for (size_t i = 0; i < invalid_names.size(); i++) std::cerr << invalid_names[i] << " ";
      std::cerr << "], calibrating = [";
      for (size_t i = 0; i < calibrating_names.size(); i++) std::cerr << calibrating_names[i] << " ";
        std::cerr << "]" << std::endl;
        return false;
  }
  // Start calibration
  //   Print output force before calib
  std::cerr << "[" << m_profile.instance_name << "]   Calibrate sensor names = [";
  for (size_t i = 0; i < valid_names.size(); i++) std::cerr << valid_names[i] << " ";
  std::cerr << "]" << std::endl;
  {
      Guard guard(m_mutex);
      for (size_t i = 0; i < valid_names.size(); i++) {
          m_sensors[valid_names[i]]->max_offset_calib_counter = static_cast<int>(tm/m_dt);
          m_sensors[valid_names[i]]->forceOffset_sum = hrp::Vector3::Zero();
          m_sensors[valid_names[i]]->momentOffset_sum = hrp::Vector3::Zero();
          m_sensors[valid_names[i]]->offset_calib_counter = m_sensors[valid_names[i]]->max_offset_calib_counter;
      }
  }
  //   Wait
  for (size_t i = 0; i < valid_names.size(); i++) {
      sem_wait(&(m_sensors[valid_names[i]]->wait_sem));
  }
  //   Print output force and offset after calib
  {
      Guard guard(m_mutex);
      std::cerr << "[" << m_profile.instance_name << "]   Calibrate done (calib time = " << tm << "[s])" << std::endl;
      for (size_t i = 0; i < valid_names.size(); i++) {
          std::cerr << "[" << m_profile.instance_name << "]     Calibrated offset [" << valid_names[i] << "], ";
          std::cerr << "force_offset = " << m_sensors[valid_names[i]]->forceOffset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "][N]")) << ", ";
          std::cerr << "moment_offset = " << m_sensors[valid_names[i]]->momentOffset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "][Nm]")) << std::endl;
      }
  }
  std::cerr << "[" << m_profile.instance_name << "] removeVirtualForceSensorOffset...done" << std::endl;
  return true;
}

bool VirtualForceSensor::removeExternalForceOffset(const double tm)
{
  std::cerr << "[" << m_profile.instance_name << "] removeExternalForceOffset..." << std::endl;
  {
      Guard guard(m_mutex);
      if (extforce_offset_calib_counter!=0){
          std::cerr << "[" << m_profile.instance_name << "]   Cannot start removeExternalForceOffset" <<std::endl;
          return false;
      }
      max_extforce_offset_calib_counter = static_cast<size_t>(tm/m_dt);
      extforceOffset_sum = hrp::Vector3::Zero();
      extmomentOffset_sum = hrp::Vector3::Zero();
      exttorqueOffset_sum = hrp::dvector::Zero(m_robot->numJoints());
      extforce_offset_calib_counter = max_extforce_offset_calib_counter;
  }

  sem_wait(&extforce_wait_sem);

  {
      Guard guard(m_mutex);
      std::cerr << "[" << m_profile.instance_name << "]   Calibrate done (calib time = " << tm << "[s])" << std::endl;
      std::cerr << "force_offset = " << extforceOffset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "][N]")) << ", ";
      std::cerr << "moment_offset = " << extmomentOffset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "][Nm]")) << std::endl;
      std::cerr << "jointtorqueOffset = " << exttorqueOffset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "][Nm]")) << std::endl;
  }

  std::cerr << "[" << m_profile.instance_name << "] removeExternalForceOffset...done" << std::endl;
  
  return true;
}

bool VirtualForceSensor::startEstimation(const std::string& sensorName)
{
  // Check argument validity
  bool is_valid_argument = true;
  {
      Guard guard(m_mutex);
      if ( m_sensors.find(sensorName) != m_sensors.end() ) {
          m_sensors[sensorName]->is_enable = true;
      }else{
          is_valid_argument = false;
      }
  }
  if (is_valid_argument){
      std::cerr << "[" << m_profile.instance_name << "] startEstimation..." << sensorName << std::endl;
      return true;
  }else{
      std::cerr << "[" << m_profile.instance_name << "]   Cannot startEstimation, invalid = " << sensorName << std::endl;
      return false;
  }
}

bool VirtualForceSensor::stopEstimation(const std::string& sensorName)
{
  // Check argument validity
  bool is_valid_argument = true;
  {
      Guard guard(m_mutex);
      if ( m_sensors.find(sensorName) != m_sensors.end() ) {
          m_sensors[sensorName]->is_enable = false;
          m_sensors[sensorName]->off_sensor_force_filter->reset(hrp::Vector3::Zero());
          m_sensors[sensorName]->off_sensor_moment_filter->reset(hrp::Vector3::Zero());
      }else{
          is_valid_argument = false;
      }
  }
  if (is_valid_argument){
      std::cerr << "[" << m_profile.instance_name << "] stopEstimation..." << sensorName << std::endl;
      return true;
  }else{
      std::cerr << "[" << m_profile.instance_name << "]   Cannot stopEstimation, invalid = " << sensorName << std::endl;
      return false;
  }
}

extern "C"
{

  void VirtualForceSensorInit(RTC::Manager* manager)
  {
    RTC::Properties profile(virtualforcesensor_spec);
    manager->registerFactory(profile,
                             RTC::Create<VirtualForceSensor>,
                             RTC::Delete<VirtualForceSensor>);
  }

};


