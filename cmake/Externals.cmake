################################################################################
#
# Settings for finding files and directories
#
################################################################################
SET(HGR   $ENV{SIT})
SET(MKPLT $ENV{MAKEFILE_PLATFORM})


################################################################################
#
# Settings for different Vortex versions
#
################################################################################
IF(USE_VORTEX STREQUAL ESSENTIALS)

    IF(DEFINED ENV{SIT})
        # SIT available, use that version by default
        SET(VORTEX_ESSENTIALS_DIR "${HGR}/External/Vortex/Essentials/${MKPLT}" CACHE PATH "Vortex install directory")
    ELSE()
        # SIT not available, path must be set
        SET(VORTEX_ESSENTIALS_DIR "" CACHE PATH "Vortex install directory")
    ENDIF()

    # Validate vortex dir
    IF(NOT EXISTS ${VORTEX_ESSENTIALS_DIR})
        MESSAGE(FATAL_ERROR "Set to use Vortex, but VORTEX_ESSENTIALS_DIR is not set or does not exist")
    ENDIF()

    SET(VORTEX_INCLUDE_DIR ${VORTEX_ESSENTIALS_DIR}/include)
    SET(VORTEX_LIBRARY_DIR ${VORTEX_ESSENTIALS_DIR}/lib)
    SET(VORTEX_LIBRARIES
      ${VORTEX_LIBRARY_DIR}/libVxCore.so
      ${VORTEX_LIBRARY_DIR}/libVxDynamics.so)
    SET(VORTEX_DEFINITIONS -DLINUX)

ENDIF()

################################################################################
#
# Settings for bullet physics
#
################################################################################
IF(UNIX)
  SET(BT_LIB_PREFIX "lib")
  SET(BT_LIB_SUFFIX "so")
ELSEIF(WIN32)
  SET(BT_LIB_PREFIX "")
  SET(BT_LIB_SUFFIX "lib")
ENDIF()

##############################################################################
# Bullet 2.83 double version
##############################################################################
IF (USE_BULLET STREQUAL 2.83_double)

  IF(DEFINED ENV{SIT})
    # SIT available, use that version by default
    SET(BT_LIB_DIR ${HGR}/External/Bullet/2.83/lib/${MKPLT})
    SET(BULLET_INCLUDE_DIRS ${HGR}/External/Bullet/2.83/include)
  ELSE()
    # SIT not available, path must be set
    SET(BT_LIB_DIR "" CACHE PATH "Bullet library directory")
    SET(BULLET_INCLUDE_DIRS "" CACHE PATH "Bullet include directory")
  ENDIF()

  SET(BULLET_DEFINITIONS -DUSE_BULLET -DBT_USE_DOUBLE_PRECISION)
  SET(BULLET_CXX_FLAGS "-isystem ${BULLET_INCLUDE_DIRS}")

  IF(UNIX)
    SET(BULLET_LIBRARIES
      ${BT_LIB_DIR}/libLinearMath.so
      ${BT_LIB_DIR}/libBulletCollision.so
      ${BT_LIB_DIR}/libBulletDynamics.so
      ${BT_LIB_DIR}/libBulletSoftBody.so
      )
  ELSEIF(WIN32)
    SET(BULLET_LIBRARIES
      ${BT_LIB_DIR}/BulletCollision.lib
      ${BT_LIB_DIR}/BulletDynamics.lib
      ${BT_LIB_DIR}/BulletFileLoader.lib
      ${BT_LIB_DIR}/BulletSoftBody.lib
      ${BT_LIB_DIR}/BulletInverseDynamics.lib
      ${BT_LIB_DIR}/BulletInverseDynamicsUtils.lib
      ${BT_LIB_DIR}/BulletWorldImporter.lib
      ${BT_LIB_DIR}/BulletXmlWorldImporter.lib
      ${BT_LIB_DIR}/ConvexDecomposition.lib
      ${BT_LIB_DIR}/GIMPACTUtils.lib
      ${BT_LIB_DIR}/LinearMath.lib
      opengl32 glu32
      ${BT_LIB_DIR}/glut32.lib
      )
  ENDIF()

ELSEIF(USE_BULLET STREQUAL 2.83_float)

  FIND_PACKAGE(Bullet REQUIRED)

ENDIF()

################################################################################
#
# Settings for OpenSceneGraph
#
################################################################################
IF(WIN32)

  SET(OPENSCENEGRAPH_INCLUDE_DIRS ${HGR}/External/OpenSceneGraph/3.0/${MKPLT}/include)
  SET(OSG_BINARY_DIR ${HGR}/External/OpenSceneGraph/3.0/${MKPLT}/bin)
  SET(OSG_LIBRARY_DIR ${HGR}/External/OpenSceneGraph/3.0/${MKPLT}/lib)

  SET(OPENSCENEGRAPH_LIBRARIES
    ${OSG_LIBRARY_DIR}/osgAnimation${RCS_DEBUG_SUFFIX}.lib
    ${OSG_LIBRARY_DIR}/osg${RCS_DEBUG_SUFFIX}.lib
    ${OSG_LIBRARY_DIR}/osgDB${RCS_DEBUG_SUFFIX}.lib
    ${OSG_LIBRARY_DIR}/osgFX${RCS_DEBUG_SUFFIX}.lib
    ${OSG_LIBRARY_DIR}/osgGA${RCS_DEBUG_SUFFIX}.lib
    ${OSG_LIBRARY_DIR}/osgManipulator${RCS_DEBUG_SUFFIX}.lib
    ${OSG_LIBRARY_DIR}/osgParticle${RCS_DEBUG_SUFFIX}.lib
    ${OSG_LIBRARY_DIR}/osgPresentation${RCS_DEBUG_SUFFIX}.lib
    ${OSG_LIBRARY_DIR}/osgShadow${RCS_DEBUG_SUFFIX}.lib
    ${OSG_LIBRARY_DIR}/osgSim${RCS_DEBUG_SUFFIX}.lib
    ${OSG_LIBRARY_DIR}/osgTerrain${RCS_DEBUG_SUFFIX}.lib
    ${OSG_LIBRARY_DIR}/osgText${RCS_DEBUG_SUFFIX}.lib
    ${OSG_LIBRARY_DIR}/osgUtil${RCS_DEBUG_SUFFIX}.lib
    ${OSG_LIBRARY_DIR}/osgViewer${RCS_DEBUG_SUFFIX}.lib
    ${OSG_LIBRARY_DIR}/osgVolume${RCS_DEBUG_SUFFIX}.lib
    ${OSG_LIBRARY_DIR}/osgWidget${RCS_DEBUG_SUFFIX}.lib
    ${OSG_LIBRARY_DIR}/OpenThreads${RCS_DEBUG_SUFFIX}.lib
    )

ELSE()

  SET(OpenSceneGraph_MARK_AS_ADVANCED TRUE)
  FIND_PACKAGE(OpenSceneGraph REQUIRED
    OpenThreads
    osgDB
    osg
    osgManipulator
    osgShadow
    osgText
    osgUtil
    osgViewer
    osgFX
    osgGA)

ENDIF()

################################################################################
#
# Settings for GeometricTools 5.10
#
################################################################################
SET(WM5_DEFINITIONS USE_WM5)
# SET(WM5_INCLUDE_DIR ${HGR}/External/GeometricTools/5.10/include)
# SET(WM5_LIBRARY_DIR ${HGR}/External/GeometricTools/5.10/lib/${MKPLT})

SET(WM5_INCLUDE_DIR /home/theo/software/tools/GeometricTools/WildMagic5/SDK/Include)
SET(WM5_LIBRARY_DIR /home/theo/software/tools/GeometricTools/WildMagic5/SDK/Library/ReleaseDynamic)


IF(WIN32)
  SET(WM5_LIBRARIES
    ${WM5_LIBRARY_DIR}/Wm5Mathematics${RCS_DEBUG_SUFFIX}.lib
    ${WM5_LIBRARY_DIR}/Wm5Core${RCS_DEBUG_SUFFIX}.lib)
ELSE(WIN32)
  SET(WM5_FLAGS "-isystem ${WM5_INCLUDE_DIR}")
  SET(WM5_LIBRARIES
    ${WM5_LIBRARY_DIR}/libWm5Mathematics.so
    ${WM5_LIBRARY_DIR}/libWm5Core.so)
ENDIF(WIN32)

################################################################################
#
# Settings for libxml2
#
################################################################################
IF(WIN32)
  SET(LIBXML2_LIBRARIES ${HGR}/External/libxml2-win/libxml2-2.7.8.win32/lib/libxml2.lib)
  SET(LIBXML2_INCLUDE_DIR ${HGR}/External/libxml2-win/libxml2-2.7.8.win32/include)
ELSE(WIN32)
  FIND_PACKAGE(LibXml2 REQUIRED)
ENDIF(WIN32)

################################################################################
#
# Settings for pthreads.
#
################################################################################
IF(WIN32)
  SET(PTHREAD_LIBRARIES ${HGR}/External/pthreads-win/pthreads-w32-2-9-1-release/Pre-built.2/lib/x86/pthreadVC2.lib)
  SET(PTHREAD_INCLUDE_DIR ${HGR}/External/pthreads-win/pthreads-w32-2-9-1-release/Pre-built.2/include)
ELSE(WIN32)
  SET(PTHREAD_LIBRARIES pthread)
ENDIF()

################################################################################
#
# Settings for qwt
#
################################################################################
IF(WIN32)

  SET(QWT_INCLUDE_DIRS ${HGR}/External/qwt/5.2/include)
  SET(QWT_INCLUDE_DIR ${HGR}/External/qwt/5.2/include)
  SET(QWT_LIBRARY_DIR ${HGR}/External/qwt/5.2/lib/${MKPLT})
  SET(QWT_MAJOR_VERSION 5)

  ADD_LIBRARY(libqwt STATIC IMPORTED)
  SET_PROPERTY(TARGET libqwt PROPERTY IMPORTED_LOCATION ${QWT_LIBRARY_DIR}/qwt5${RCS_DEBUG_SUFFIX}.lib)
  SET_PROPERTY(TARGET libqwt PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${HGR}/External/qwt/5.2/include)
  SET(QWT_LIBRARIES libqwt)

ELSE(WIN32)

  FIND_PACKAGE(Qwt REQUIRED)

ENDIF(WIN32)

################################################################################
#
# Octomap
#
################################################################################
SET(OCTOMAP_DEFINITIONS -DUSE_OCTOMAP)
SET(OCTOMAP_INCLUDE_DIR ${HGR}/External/octomap/1.6.8/include)
SET(OCTOMAP_LIBRARY_DIR ${HGR}/External/octomap/1.6.8/lib/${MKPLT})

IF(WIN32)
  SET(OCTOMAP_LIBRARIES
    ${OCTOMAP_LIBRARY_DIR}/octomap.lib
    ${OCTOMAP_LIBRARY_DIR}/octomath.lib)
ELSE(WIN32)
  SET(OCTOMAP_LIBRARIES
    ${OCTOMAP_LIBRARY_DIR}/liboctomap.so
    ${OCTOMAP_LIBRARY_DIR}/liboctomath.so)
ENDIF(WIN32)

################################################################################
#
# Eigen3 math library
#
################################################################################
IF(USE_EIGEN3)
  FIND_PACKAGE (Eigen3 3.2.0 REQUIRED)
ENDIF()
