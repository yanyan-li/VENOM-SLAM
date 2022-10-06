cmake_minimum_required(VERSION 3.10)
project(Venom)


SET(CMAKE_BUILD_TYPE Debug) # Release, RelWithDebInfo , Debug
set(CMAKE_CXX_STANDARD 11) 

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# installed in docker already 
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED) 

option(VERBOSE "Venom Similator Printer" ON)

if(VERBOSE)
 message(STATUS "---Start Venom Simulator Printer---")
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

add_library(VenomLib SHARED ${SOURCEFILES})
target_link_libraries(VenomLib
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${CERES_LIBRARIES}
        gtsam
        )

#########---show environment---###################
add_executable(show_estimator_env test/test_show_env.cc )
target_link_libraries(show_estimator_env VenomLib ${CMAKE_THREAD_LIBS_INIT}) 

#########---Landmark optimization LBA---##########
add_executable(test_pointBA test/test_pointBA.cc )
target_link_libraries(test_pointBA VenomLib ${CMAKE_THREAD_LIBS_INIT}) 


#########---show interface---###################
add_executable(test_interface test/test_interface.cc)
target_link_libraries(test_interface VenomLib ${CMAKE_THREAD_LIBS_INIT}) 

#########---show interface---###################
add_executable(main_entrance test/main_entrance.cc)
target_link_libraries(main_entrance VenomLib ${CMAKE_THREAD_LIBS_INIT}) 
