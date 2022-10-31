project(plansys2_problem_expert)

cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)

set(DEPS
  roscpp
  lifecycle
  plansys2_pddl_parser
  plansys2_core
  plansys2_msgs
  plansys2_domain_expert
  std_msgs
  lifecycle_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${DEPS}
)  

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${DEPS}
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(PROBLEM_EXPERT_SOURCES
  src/plansys2_problem_expert/ProblemExpert.cpp
  src/plansys2_problem_expert/ProblemExpertClient.cpp
  src/plansys2_problem_expert/ProblemExpertNode.cpp
  src/plansys2_problem_expert/Utils.cpp
)

add_library(${PROJECT_NAME} SHARED ${PROBLEM_EXPERT_SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(problem_expert_node
  src/problem_expert_node.cpp
)
target_link_libraries(problem_expert_node ${PROJECT_NAME})

foreach( dir config launch)
   install( DIRECTORY ${dir}/
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir} )
endforeach(dir)

install(TARGETS
  ${PROJECT_NAME}
  problem_expert_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto_find_test_dependencies()

#  find_package(ament_cmake_gtest REQUIRED)
#  add_subdirectory(test)
#endif()

