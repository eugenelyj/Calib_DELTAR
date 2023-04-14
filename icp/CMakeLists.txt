# define new project
PROJECT(mv-lm-icp)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.0 FATAL_ERROR)

option(WITH_G2O "Compile with G2O support if found" ON)

#print var to screen
function(ECHO var)
MESSAGE(${var} ": " ${${var}})
endfunction(ECHO)

#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_DEBUG} -Wall") # -DCMAKE_BUILD_TYPE=Debug
#set(CMAKE_BUILD_TYPE "Release")

#this forces release build, otherwise ceres autodiff is really slow:
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
#to debug add this when runniing CMAKE: -DCMAKE_BUILD_TYPE=Debug
#set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)

if(UNIX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
endif(UNIX)

if(MSVC)
add_definitions(-D_USE_MATH_DEFINES)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

MESSAGE("---------------Sophus-------------------")
# use lovegroves version of sophus with ceres extensions!
#ADD_SUBDIRECTORY(ext/sophus-ceres)
SET(Sophus_INCLUDE_DIR ext/sophus-ceres)
INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIR})
MESSAGE("Sophus_INCLUDE_DIR ${Sophus_INCLUDE_DIR}")

MESSAGE("---------------Eigen-------------------")
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
SET(EIGEN_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR}) #needed for ceres for old version of cmake/FindEigen scripts
MESSAGE("Eigen3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR}")

MESSAGE("---------------Gflags-------------------")
FIND_PACKAGE(Gflags REQUIRED) # package name capitalization, is it really gflags (for windows) vs. Gflags (for linux)
INCLUDE_DIRECTORIES(${GFLAGS_INCLUDE_DIRS})
MESSAGE("gflags found ${gflags_FOUND} at ${gflags_INCLUDE_DIR}")

MESSAGE("---------------Ceres-------------------")
FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
MESSAGE("ceres found ${Ceres_FOUND} at ${Ceres_DIR}")
MESSAGE("ceres version ${Ceres_VERSION}")
MESSAGE("Ceres_INCLUDE_DIR ${CERES_INCLUDE_DIRS}")
MESSAGE("Ceres_LIBRARIES ${CERES_LIBRARIES}")



include_directories(include/)

file(GLOB HEADERS
    include/*.h
    include/*.hpp
)

file(GLOB SOURCES src/internal/*.cpp src/internal/*.c)

ADD_EXECUTABLE(pt2plane src/pt2plane.cpp ${HEADERS} ${SOURCES})
TARGET_LINK_LIBRARIES(pt2plane ${GFLAGS_LIBRARIES} ${CERES_LIBRARIES})
