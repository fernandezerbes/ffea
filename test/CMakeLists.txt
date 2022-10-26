enable_testing()

add_executable(geometric_entity_test geometric_entity_test.cc)

target_link_libraries(geometric_entity_test PRIVATE ffea GTest::gtest_main)

target_link_libraries(geometric_entity_test PRIVATE ffea gmock)

include(GoogleTest)

set(BUILD_GMOCK ON)

add_test(geometric_entity_gtest geometric_entity_test)