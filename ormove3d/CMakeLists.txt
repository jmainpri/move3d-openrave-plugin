cmake_minimum_required(VERSION 2.4.6)

project (or-move3d)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing ")
endif()

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

cmake_minimum_required (VERSION 2.4)
if(POLICY CMP0015)
   cmake_policy(SET CMP0015 NEW)
endif()

message(STATUS "CMAKE_SOURCE_DIR : " ${CMAKE_SOURCE_DIR})
message(STATUS "CMAKE_MODULE_PATH : " ${CMAKE_MODULE_PATH})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../cmake")

if( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )
  set(PLUGIN_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../plugins")
else( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )
    set(PLUGIN_INSTALL_DIR "$ENV{PLUGIN_INSTALL_DIR}")
endif( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )

message(STATUS "PLUGIN_INSTALL_DIR : " ${PLUGIN_INSTALL_DIR})

#-------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# --- Compilation Flags --------------------------------------------------------
# ------------------------------------------------------------------------------
#-------------------------------------------------------------------------------

SET(Move3D_CompilationFlags "")
SET(Move3D_LinkedCompilationFlags "")

MACRO(Move3D_flag flagName description status)
  SET(flagName "${flagName}")
  SET(description "${description}")
  SET(status "${status}")
  OPTION(${flagName} ${description} ${status})
  LIST(APPEND Move3D_CompilationFlags ${flagName})
  FOREACH(linkFlag ${ARGN})
    LIST(APPEND Move3D_LinkedCompilationFlags "${flagName}:${linkFlag}")
  ENDFOREACH(linkFlag)
#  set_property(CACHE ANIMATION PROPERTY VALUE ON)
ENDMACRO(Move3D_flag)

MACRO(Move3D_CheckCompilationFlags)
  SET(Move3D_ActiveFlags "")
  #MESSAGE ("Active Flags :")
  FILE(WRITE ${CMAKE_CURRENT_BINARY_DIR}/smpFlags "#Generated File Do not modify!!!\n" )
  FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/smpFlags "Move3D_COMPILATION_FLAGS = ")
  FOREACH(flag ${Move3D_CompilationFlags})
    IF(${flag})
      FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/Move3DFlags "-D${flag} ")
      #MESSAGE("${flag}")
      ADD_DEFINITIONS("-D${flag}")
      LIST(APPEND Move3D_ActiveFlags ${flag})
    ENDIF()
  ENDFOREACH(flag)
ENDMACRO(Move3D_CheckCompilationFlags)

Move3D_flag( QT_LIBRARY "Sets QtLibrary compiling mode" ON)
Move3D_flag( QT_GL "Sets QtLibrary compiling mode" ON QT_LIBRARY)
Move3D_flag( QT_UI_XML_FILES "Sets QtLibrary compiling mode" ON QT_LIBRARY)

Move3D_flag( HRI_COSTSPACE "With the hri work" ON)


Move3D_flag( P3D_CONSTRAINTS "Compilation with module Constraints" ON)
Move3D_flag( P3D_COLLISION_CHECKING "Compilation with module Collision Checking" ON)
Move3D_flag( P3D_LOCALPATH "Compilation with module Localpath" ON)
Move3D_flag( P3D_PLANNER "Compilation with module Planner" ON)
Move3D_flag( LIGHT_PLANNER "Compilation with module Light Planner" ON)
Move3D_flag( GRASP_PLANNING "Compilation with module Manipulation Planner" ON)
Move3D_flag( MULTILOCALPATH "Compilation with module Multilocalpath" ON)
Move3D_flag( MULTIGRAPH "Compilation with module Multi-Graph" ON)

Move3D_flag( MOVE3D_CORE "Compiles the planning API and planners" ON)

Move3D_flag( HRI_PLANNER "Human Robot Interaction planner module" ON USE_GSL PQP)

Move3D_flag( USE_GBM "Compilation with module Multilocalpath" OFF)
Move3D_flag( USE_GLUT "Compilation with a glut window" OFF)
Move3D_flag( USE_GSL "Mathematic Library" OFF)
Move3D_flag( USE_QWT "Compilation with technical widget" ON)

# Find important packages
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
#-------------------------------------------------------------------------------

find_package (PkgConfig REQUIRED)

#-------------- Openrave -------------------------------------------------------

find_package(OpenRAVE REQUIRED)
include_directories(${OPENRAVE_INCLUDE_DIRS})
message(STATUS "OPENRAVE_INCLUDE_DIRS : " ${OPENRAVE_INCLUDE_DIRS})
message(STATUS "OPENRAVE_LIBRARY : " ${OPENRAVE_LIBRARY})

# ------------------- libmove3d ----------------------------------------------

PKG_CHECK_MODULES(MOVE3D REQUIRED libmove3d)
INCLUDE_DIRECTORIES(${MOVE3D_INCLUDE_DIRS})
LINK_DIRECTORIES( ${MOVE3D_LIBRARY_DIRS} )
SET(LIBS ${LIBS} ${MOVE3D_LIBRARIES})
ADD_DEFINITIONS(${MOVE3D_CFLAGS})

#MESSAGE( MOVE3D_LIBRARIES = ${MOVE3D_LIBRARIES})

#-------------- Move3D-motionPlanner-libs --------------------------------------

FIND_PACKAGE(MOVE3D-PLANNERS REQUIRED)
INCLUDE_directories(${MOVE3D-PLANNERS_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${MOVE3D-PLANNERS_INCLUDE_DIR})
SET(LIBS ${LIBS} ${MOVE3D-PLANNERS_LIBRARIES})

#-------------- Move3D-Qt-Gui-libs --------------------------------------

FIND_PACKAGE(MOVE3D-QTGUI REQUIRED)
INCLUDE_directories(${MOVE3D-QTGUI_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${MOVE3D-QTGUI_INCLUDE_DIR})
SET(LIBS ${LIBS} ${MOVE3D-QTGUI_LIBRARIES})

#-------------- Eigen ----------------------------------------------------------

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
message(Eigen3_INCLUDE_DIR : ${EIGEN3_INCLUDE_DIR})

#-------------- Boost ----------------------------------------------------------

# boost is used e.g. for managing the cost functions (T-RRT).
#  FIND_PACKAGE(Boost REQUIRED)
#  INCLUDE_directories(${BOOST_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${BOOST_INCLUDE_DIR})

# boost is used e.g. for managing the cost functions (T-RRT).
find_package(Boost REQUIRED)
include_directories(/opt/local/include/)
set(INCLUDE_DIRS ${INCLUDE_DIRS} /opt/local/include/ ${BOOST_INCLUDE_DIRS})


IF( MULTILOCALPATH )

#-------------- GBM ------------------------------------------------------------
    FIND_PACKAGE(GBM REQUIRED)
    include_directories(${GBM_INCLUDE_DIR})
    SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GBM_INCLUDE_DIR})
    SET(LIBS ${LIBS} ${GBM_LIBRARIES})

#-------------- SOFTMOTION -----------------------------------------------------
    FIND_PACKAGE(SOFTMOTION REQUIRED)
    INCLUDE_directories(${SOFTMOTION_INCLUDE_DIR})
    SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${SOFTMOTION_INCLUDE_DIR})
    SET(LIBS ${LIBS} ${SOFTMOTION_LIBRARIES})

ENDIF( MULTILOCALPATH )

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
#-------------------------------------------------------------------------------

if( NOT OPENRAVE_FOUND )
    message(FATAL_ERROR "could not find openrave, check for openrave-config in path")
endif( NOT OPENRAVE_FOUND )

add_library(${PROJECT_NAME} SHARED move3d-main.cpp openrave_move3d_api.cpp planner_functions.cpp )

link_directories(${OPENRAVE_LINK_DIRS})

target_link_libraries(${PROJECT_NAME} ${LIBS} ${OPENRAVE_LIBRARY} )

set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS ${OPENRAVE_LINK_FLAGS} COMPILE_FLAGS ${OPENRAVE_CXXFLAGS})

install(TARGETS ${PROJECT_NAME} DESTINATION ${PLUGIN_INSTALL_DIR} )

