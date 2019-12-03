if(USE_QPOASES)
  #add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0 ${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0)
  link_directories(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0/bin)
  include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0/include)
  add_definitions(-DUSE_QPOASES)
endif()

set(comp_sources VirtualForceSensor.cpp VirtualForceSensorService_impl.cpp ../ImpedanceController/JointPathEx.cpp ../ImpedanceController/RatsMatrix.cpp ../TorqueFilter/IIRFilter.h)
if(USE_QPOASES)
  set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub qpOASES)
else()
  set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)
endif()
add_library(VirtualForceSensor SHARED ${comp_sources})
target_link_libraries(VirtualForceSensor ${libs})
set_target_properties(VirtualForceSensor PROPERTIES PREFIX "")

add_executable(VirtualForceSensorComp VirtualForceSensorComp.cpp ${comp_sources})
target_link_libraries(VirtualForceSensorComp ${libs})

set(target VirtualForceSensor VirtualForceSensorComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
