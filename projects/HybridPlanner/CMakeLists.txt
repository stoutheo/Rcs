SET(RCS_HYBRIDPLANNER_SRCS ${PROJECT_SOURCE_DIR}/projects/HybridPlanner)


include_directories(${RCS_HYBRIDPLANNER_SRCS}/modules  ${PROJECT_SOURCE_DIR}/src/RcsCore)
#add_library(MotionPlan SHARED ${RCS_HYBRIDPLANNER_SRCS}/modules/MotionPlanning.cpp)

##############################################################################
#
# Targets
#
##############################################################################
ADD_EXECUTABLE(DexBot_Kin ${RCS_HYBRIDPLANNER_SRCS}/demos/DexBot/DexBot_Kin.cpp)
TARGET_LINK_LIBRARIES(DexBot_Kin RcsGui RcsGraphics)
