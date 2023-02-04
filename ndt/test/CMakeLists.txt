ament_add_gtest(ndt_test ndt_test.cpp)
ament_target_dependencies(ndt_test ${dependencies})
target_link_libraries(ndt_test ${PROJECT_NAME} ${PCL_LIBRARIES})
