cmake_minimum_required(VERSION 2.8.3)
project(rq_gripper_2F140)

add_executable(${PROJECT_NAME} src/rq_gripper_2F140.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

