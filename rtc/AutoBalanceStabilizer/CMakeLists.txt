if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.1)
  message(WARNING "[rtc/AutoBalanceStabilizer] Your gcc version is less than 5.1. Skip Compiling AutoBalanceStabilizer")
  return()
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(comp_sources AutoBalanceStabilizer.cpp AutoBalanceStabilizerService_impl.cpp ../SequencePlayer/interpolator.cpp ../TorqueFilter/IIRFilter.cpp)
set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)
set(abst_libs EigenUtil LinkConstraint GaitGenerator Stabilizer StateEstimator)

include_directories(${PROJECT_SOURCE_DIR}/rtc/SequencePlayer)

add_library(AutoBalanceStabilizer SHARED ${comp_sources})
target_link_libraries(AutoBalanceStabilizer PRIVATE ${abst_libs} ${libs})
set_target_properties(AutoBalanceStabilizer PROPERTIES PREFIX "")

add_library(EigenUtil SHARED EigenUtil.cpp)

add_library(LinkConstraint SHARED LinkConstraint.cpp LimbTrajectoryGenerator.cpp)

add_library(StateEstimator SHARED StateEstimator.cpp)
target_link_libraries(StateEstimator PRIVATE ${libs} LinkConstraint)

add_library(GaitGenerator SHARED GaitGenerator.cpp RefZMPGenerator.cpp LimbTrajectoryGenerator.cpp COGTrajectoryGenerator.cpp)
target_link_libraries(GaitGenerator PRIVATE ${libs} EigenUtil LinkConstraint)

add_library(Stabilizer SHARED Integrator.cpp TwoDofController.cpp Stabilizer.cpp ../ImpedanceController/JointPathEx.cpp ../ImpedanceController/RatsMatrix.cpp)
target_link_libraries(Stabilizer PRIVATE ${libs} EigenUtil)

add_executable(AutoBalanceStabilizerComp AutoBalanceStabilizerComp.cpp ${comp_sources})
target_link_libraries(AutoBalanceStabilizerComp PRIVATE ${abst_libs} ${libs})

add_executable(testRefZMPGenerator test/testRefZMPGenerator.cpp RefZMPGenerator.cpp ../SequencePlayer/interpolator.cpp)
target_link_libraries(testRefZMPGenerator PRIVATE ${libs} LinkConstraint)

add_executable(testLimbTrajectoryGenerator test/testLimbTrajectoryGenerator.cpp)
target_link_libraries(testLimbTrajectoryGenerator PRIVATE ${libs} LinkConstraint)

# add_executable(testCOGTrajectoryGenerator test/testCOGTrajectoryGenerator.cpp COGTrajectoryGenerator.cpp)
# target_link_libraries(testCOGTrajectoryGenerator PRIVATE ${libs} LinkConstraint)

add_executable(testTwoDofControllerABS testTwoDofControllerABS.cpp TwoDofController.cpp Integrator.cpp)
target_link_libraries(testTwoDofControllerABS ${libs})

add_executable(testZMPDistributorABS testZMPDistributorABS.cpp ZMPDistributor.h ../ImpedanceController/JointPathEx.cpp)
target_link_libraries(testZMPDistributorABS ${libs})

# add_test(testZMPDistributor_HRP2JSK_EEFM_Test0       testZMPDistributorABS --hrp2jsk --test0 --use-gnuplot false)
# add_test(testZMPDistributor_HRP2JSK_EEFM_Test1       testZMPDistributorABS --hrp2jsk --test1 --use-gnuplot false)
# add_test(testZMPDistributor_HRP2JSK_EEFM_Test2       testZMPDistributorABS --hrp2jsk --test2 --use-gnuplot false)
# add_test(testZMPDistributor_JAXONRED_EEFM_Test0      testZMPDistributorABS --jaxon_red --test0 --use-gnuplot false)
# add_test(testZMPDistributor_JAXONRED_EEFM_Test1      testZMPDistributorABS --jaxon_red --test1 --use-gnuplot false)
# add_test(testZMPDistributor_JAXONRED_EEFM_Test2      testZMPDistributorABS --jaxon_red --test2 --use-gnuplot false)
# add_test(testZMPDistributor_HRP2JSK_EEFMQPCOP_Test0  testZMPDistributorABS --hrp2jsk --test0 --use-gnuplot false --distribution-algorithm EEFMQPCOP)
# add_test(testZMPDistributor_HRP2JSK_EEFMQPCOP_Test1  testZMPDistributorABS --hrp2jsk --test1 --use-gnuplot false --distribution-algorithm EEFMQPCOP)
# add_test(testZMPDistributor_HRP2JSK_EEFMQPCOP_Test2  testZMPDistributorABS --hrp2jsk --test2 --use-gnuplot false --distribution-algorithm EEFMQPCOP)
# add_test(testZMPDistributor_JAXONRED_EEFMQPCOP_Test0 testZMPDistributorABS --jaxon_red --test0 --use-gnuplot false --distribution-algorithm EEFMQPCOP)
# add_test(testZMPDistributor_JAXONRED_EEFMQPCOP_Test1 testZMPDistributorABS --jaxon_red --test1 --use-gnuplot false --distribution-algorithm EEFMQPCOP)
# add_test(testZMPDistributor_JAXONRED_EEFMQPCOP_Test2 testZMPDistributorABS --jaxon_red --test2 --use-gnuplot false --distribution-algorithm EEFMQPCOP)

# set(target AutoBalanceStabilizer AutoBalanceStabilizerComp EigenUtil LinkConstraint GaitGenerator Stabilizer testRefZMPGenerator testLimbTrajectoryGenerator testCOGTrajectoryGenerator testTwoDofControllerABS testZMPDistributorABS)
set(target AutoBalanceStabilizer AutoBalanceStabilizerComp EigenUtil LinkConstraint GaitGenerator Stabilizer StateEstimator testRefZMPGenerator testLimbTrajectoryGenerator testTwoDofControllerABS testZMPDistributorABS)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
