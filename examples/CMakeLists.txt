add_executable(runner runner.cc)
add_executable(runner_3d runner.cc)
target_link_libraries(runner PUBLIC ffea)
target_link_libraries(runner_3d PUBLIC ffea)