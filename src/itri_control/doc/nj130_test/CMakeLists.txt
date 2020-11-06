add_executable(nj130_test src/nj130_test.cpp)
target_link_libraries(nj130_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS nj130_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

