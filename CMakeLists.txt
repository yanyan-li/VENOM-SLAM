cmake_minimum_required(VERSION 3.10)
project(PointLinePlaneParam)


SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo , Debug
set(CMAKE_CXX_STANDARD 11) 

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# installed in docker already 
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED) 

option(VERBOSE "Venom Similator Printer" OFF)

if(VERBOSE)
 message(STATUS "VERBOSE")
 add_definitions(-D__VERBOSE__)
endif()


# installed later if you need
find_package(Ceres REQUIRED)
find_package(GTSAM)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(
        ${CMAKE_SOURCE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}
)
add_subdirectory(thirdparty)

file(GLOB SOURCEFILES src/*.hpp)

add_library(PointLinePlaneParamLib SHARED ${SOURCEFILES})
target_link_libraries(PointLinePlaneParamLib
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${CERES_LIBRARIES}
        gtsam
        )

#########---show environment---###################
add_executable(show_estimator_env test/test_show_env.cc )
target_link_libraries(show_estimator_env PointLinePlaneParamLib ${CMAKE_THREAD_LIBS_INIT}) 

#########---Landmark optimization LBA---##########
add_executable(test_pointBA test/test_pointBA.cc )
target_link_libraries(test_pointBA PointLinePlaneParamLib ${CMAKE_THREAD_LIBS_INIT}) 


#########---show interface---###################
add_executable(test_interface test/test_interface.cc)
target_link_libraries(test_interface PointLinePlaneParamLib ${CMAKE_THREAD_LIBS_INIT}) 

