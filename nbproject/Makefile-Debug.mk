#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux-x86
CND_DLIB_EXT=so
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/src/AutonomousXplorationAndMapping.o \
	${OBJECTDIR}/src/CompareASR.o \
	${OBJECTDIR}/src/CompareASRClass.o \
	${OBJECTDIR}/src/Comparison.o \
	${OBJECTDIR}/src/ExploringandReturn.o \
	${OBJECTDIR}/src/GXplorationAndMapping.o \
	${OBJECTDIR}/src/GeometricOp.o \
	${OBJECTDIR}/src/GeometryFuncs.o \
	${OBJECTDIR}/src/GroupS.o \
	${OBJECTDIR}/src/GuidedXplorationAndMapping.o \
	${OBJECTDIR}/src/Laser2Surface.o \
	${OBJECTDIR}/src/Map.o \
	${OBJECTDIR}/src/Mapping.o \
	${OBJECTDIR}/src/Minfo.o \
	${OBJECTDIR}/src/NavigationToExit.o \
	${OBJECTDIR}/src/Object.o \
	${OBJECTDIR}/src/OfflineNavigationToExit.o \
	${OBJECTDIR}/src/PathPlanning.o \
	${OBJECTDIR}/src/PerceptualMapping.o \
	${OBJECTDIR}/src/Plotting.o \
	${OBJECTDIR}/src/Point.o \
	${OBJECTDIR}/src/PointAndSurface.o \
	${OBJECTDIR}/src/Polygon.o \
	${OBJECTDIR}/src/PolygonChoice.o \
	${OBJECTDIR}/src/StereoMapping.o \
	${OBJECTDIR}/src/Testboost.o \
	${OBJECTDIR}/src/Transporter.o \
	${OBJECTDIR}/src/asr.o \
	${OBJECTDIR}/src/asrOp.o \
	${OBJECTDIR}/src/clipper.o \
	${OBJECTDIR}/src/convertlog.o \
	${OBJECTDIR}/src/convexPathPlanner.o \
	${OBJECTDIR}/src/crossExits_test.o \
	${OBJECTDIR}/src/demo.o \
	${OBJECTDIR}/src/display.o \
	${OBJECTDIR}/src/errorMap.o \
	${OBJECTDIR}/src/errorMapWholeFloor.o \
	${OBJECTDIR}/src/localmapAndglobal.o \
	${OBJECTDIR}/src/localmapGen.o \
	${OBJECTDIR}/src/localmap_Generation.o \
	${OBJECTDIR}/src/mapWholeFloor.o \
	${OBJECTDIR}/src/mfisOp.o \
	${OBJECTDIR}/src/mrpt.o \
	${OBJECTDIR}/src/newClass.o \
	${OBJECTDIR}/src/offlineMapping.o \
	${OBJECTDIR}/src/readAndwriteASCII.o \
	${OBJECTDIR}/src/robotControl.o \
	${OBJECTDIR}/src/segmentgeneration.o \
	${OBJECTDIR}/src/space.o \
	${OBJECTDIR}/src/thesis.o \
	${OBJECTDIR}/src/wander.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/oldmappingproject

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/oldmappingproject: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/oldmappingproject ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/src/AutonomousXplorationAndMapping.o: src/AutonomousXplorationAndMapping.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/AutonomousXplorationAndMapping.o src/AutonomousXplorationAndMapping.cpp

${OBJECTDIR}/src/CompareASR.o: src/CompareASR.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/CompareASR.o src/CompareASR.cpp

${OBJECTDIR}/src/CompareASRClass.o: src/CompareASRClass.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/CompareASRClass.o src/CompareASRClass.cpp

${OBJECTDIR}/src/Comparison.o: src/Comparison.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/Comparison.o src/Comparison.cpp

${OBJECTDIR}/src/ExploringandReturn.o: src/ExploringandReturn.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/ExploringandReturn.o src/ExploringandReturn.cpp

${OBJECTDIR}/src/GXplorationAndMapping.o: src/GXplorationAndMapping.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/GXplorationAndMapping.o src/GXplorationAndMapping.cpp

${OBJECTDIR}/src/GeometricOp.o: src/GeometricOp.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/GeometricOp.o src/GeometricOp.cpp

${OBJECTDIR}/src/GeometryFuncs.o: src/GeometryFuncs.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/GeometryFuncs.o src/GeometryFuncs.cpp

${OBJECTDIR}/src/GroupS.o: src/GroupS.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/GroupS.o src/GroupS.cpp

${OBJECTDIR}/src/GuidedXplorationAndMapping.o: src/GuidedXplorationAndMapping.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/GuidedXplorationAndMapping.o src/GuidedXplorationAndMapping.cpp

${OBJECTDIR}/src/Laser2Surface.o: src/Laser2Surface.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/Laser2Surface.o src/Laser2Surface.cpp

${OBJECTDIR}/src/Map.o: src/Map.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/Map.o src/Map.cpp

${OBJECTDIR}/src/Mapping.o: src/Mapping.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/Mapping.o src/Mapping.cpp

${OBJECTDIR}/src/Minfo.o: src/Minfo.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/Minfo.o src/Minfo.cpp

${OBJECTDIR}/src/NavigationToExit.o: src/NavigationToExit.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/NavigationToExit.o src/NavigationToExit.cpp

${OBJECTDIR}/src/Object.o: src/Object.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/Object.o src/Object.cpp

${OBJECTDIR}/src/OfflineNavigationToExit.o: src/OfflineNavigationToExit.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/OfflineNavigationToExit.o src/OfflineNavigationToExit.cpp

${OBJECTDIR}/src/PathPlanning.o: src/PathPlanning.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/PathPlanning.o src/PathPlanning.cpp

${OBJECTDIR}/src/PerceptualMapping.o: src/PerceptualMapping.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/PerceptualMapping.o src/PerceptualMapping.cpp

${OBJECTDIR}/src/Plotting.o: src/Plotting.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/Plotting.o src/Plotting.cpp

${OBJECTDIR}/src/Point.o: src/Point.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/Point.o src/Point.cpp

${OBJECTDIR}/src/PointAndSurface.o: src/PointAndSurface.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/PointAndSurface.o src/PointAndSurface.cpp

${OBJECTDIR}/src/Polygon.o: src/Polygon.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/Polygon.o src/Polygon.cpp

${OBJECTDIR}/src/PolygonChoice.o: src/PolygonChoice.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/PolygonChoice.o src/PolygonChoice.cpp

${OBJECTDIR}/src/StereoMapping.o: src/StereoMapping.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/StereoMapping.o src/StereoMapping.cpp

${OBJECTDIR}/src/Testboost.o: src/Testboost.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/Testboost.o src/Testboost.cpp

${OBJECTDIR}/src/Transporter.o: src/Transporter.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/Transporter.o src/Transporter.cpp

${OBJECTDIR}/src/asr.o: src/asr.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/asr.o src/asr.cpp

${OBJECTDIR}/src/asrOp.o: src/asrOp.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/asrOp.o src/asrOp.cpp

${OBJECTDIR}/src/clipper.o: src/clipper.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/clipper.o src/clipper.cpp

${OBJECTDIR}/src/convertlog.o: src/convertlog.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/convertlog.o src/convertlog.cpp

${OBJECTDIR}/src/convexPathPlanner.o: src/convexPathPlanner.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/convexPathPlanner.o src/convexPathPlanner.cpp

${OBJECTDIR}/src/crossExits_test.o: src/crossExits_test.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/crossExits_test.o src/crossExits_test.cpp

${OBJECTDIR}/src/demo.o: src/demo.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/demo.o src/demo.cpp

${OBJECTDIR}/src/display.o: src/display.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/display.o src/display.cpp

${OBJECTDIR}/src/errorMap.o: src/errorMap.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/errorMap.o src/errorMap.cpp

${OBJECTDIR}/src/errorMapWholeFloor.o: src/errorMapWholeFloor.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/errorMapWholeFloor.o src/errorMapWholeFloor.cpp

${OBJECTDIR}/src/localmapAndglobal.o: src/localmapAndglobal.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/localmapAndglobal.o src/localmapAndglobal.cpp

${OBJECTDIR}/src/localmapGen.o: src/localmapGen.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/localmapGen.o src/localmapGen.cpp

${OBJECTDIR}/src/localmap_Generation.o: src/localmap_Generation.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/localmap_Generation.o src/localmap_Generation.cpp

${OBJECTDIR}/src/mapWholeFloor.o: src/mapWholeFloor.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/mapWholeFloor.o src/mapWholeFloor.cpp

${OBJECTDIR}/src/mfisOp.o: src/mfisOp.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/mfisOp.o src/mfisOp.cpp

${OBJECTDIR}/src/mrpt.o: src/mrpt.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/mrpt.o src/mrpt.cpp

${OBJECTDIR}/src/newClass.o: src/newClass.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/newClass.o src/newClass.cpp

${OBJECTDIR}/src/offlineMapping.o: src/offlineMapping.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/offlineMapping.o src/offlineMapping.cpp

${OBJECTDIR}/src/readAndwriteASCII.o: src/readAndwriteASCII.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/readAndwriteASCII.o src/readAndwriteASCII.cpp

${OBJECTDIR}/src/robotControl.o: src/robotControl.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/robotControl.o src/robotControl.cpp

${OBJECTDIR}/src/segmentgeneration.o: src/segmentgeneration.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/segmentgeneration.o src/segmentgeneration.cpp

${OBJECTDIR}/src/space.o: src/space.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/space.o src/space.cpp

${OBJECTDIR}/src/thesis.o: src/thesis.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/thesis.o src/thesis.cpp

${OBJECTDIR}/src/wander.o: src/wander.cpp 
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/wander.o src/wander.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/oldmappingproject

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
