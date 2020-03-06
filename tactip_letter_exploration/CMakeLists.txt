cmake_minimum_required(VERSION 2.8.3)
project(tactip_letter_exploration)

add_executable(${PROJECT_NAME} src/tactip_letter_exploration.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

