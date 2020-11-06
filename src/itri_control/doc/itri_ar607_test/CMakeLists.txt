add_executable(itri_ar607_test src/itri_ar607_test.cpp)
target_link_libraries(itri_ar607_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS itri_ar607_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

