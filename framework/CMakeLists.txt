add_library(ffea
            src/analysis/analysis.cc
            src/assembly/assembler.cc
            src/math/utils.cc
            src/math/shape_functions.cc
            src/mesh/coordinates.cc
            src/mesh/degree_of_freedom.cc
            src/mesh/element.cc
            src/mesh/integration_point.cc
            src/mesh/mesh.cc
            src/mesh/node.cc
            src/model/boundary_condition.cc
            src/processor/operator.cc
)

target_include_directories(ffea INTERFACE
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/analysis"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/assembly"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/math"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/mesh"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/model"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/processor"
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

target_link_libraries(ffea Eigen3::Eigen)
