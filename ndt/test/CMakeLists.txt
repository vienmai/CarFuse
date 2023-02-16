ament_add_gtest(ndt_test ndt_test.cpp)
ament_target_dependencies(ndt_test ${dependencies})
target_link_libraries(ndt_test ${PROJECT_NAME} ${PCL_LIBRARIES})

# ament_add_gtest(map_publisher_test map_publisher_test.cpp)
# ament_target_dependencies(map_publisher_test ${dependencies})
# target_link_libraries(map_publisher_test ${PROJECT_NAME} ${PCL_LIBRARIES})