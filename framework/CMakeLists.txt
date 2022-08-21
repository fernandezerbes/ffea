add_library(ffea
            src/analysis/analysis.cc
            src/assembly/assembler.cc
            src/math/utils.cc
            src/math/shape_functions.cc
            src/mesh/coordinates.cc
            src/mesh/degree_of_freedom.cc
            src/mesh/element.cc
            src/mesh/element_factory.cc
            src/mesh/integration_point.cc
            src/mesh/mesh.cc
            src/mesh/mesh_builder.cc
            src/mesh/node.cc
            src/model/boundary_condition.cc
            src/model/model.cc
            src/fileio/output_writer.cc
            src/fileio/mesh_parser.cc
            src/processor/operator.cc
            src/postprocessor/postprocessor.cc
)

target_include_directories(ffea INTERFACE
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/analysis"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/assembly"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/math"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/mesh"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/model"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/fileio"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/processor"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/postprocessor"
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

target_link_libraries(ffea Eigen3::Eigen)
