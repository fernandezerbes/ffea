add_library(ffea
            src/analysis/analysis.cc
            src/math/utils.cc
            src/math/shape_functions.cc
            src/geometry/coordinates.cc
            src/geometry/geometry.cc
            src/geometry/geometric_entity.cc
            src/geometry/geometric_entity_factory.cc
            src/geometry/geometry_builder.cc
            src/geometry/node.cc
            src/mesh/degree_of_freedom.cc
            src/mesh/element.cc
            src/mesh/element_factory.cc
            src/mesh/integration_point.cc
            src/mesh/integration_points_provider.cc
            src/mesh/mesh.cc
            src/mesh/mesh_builder.cc
            src/model/boundary_condition.cc
            src/model/computational_domain.cc
            src/model/constitutive_model.cc
            src/model/integrand.cc
            src/model/model.cc
            src/fileio/output_writer.cc
            src/fileio/geometry_parser.cc
            src/postprocessor/postprocessor.cc
)

target_include_directories(ffea INTERFACE
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/analysis"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/math"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/geometry"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/mesh"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/model"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/fileio"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/postprocessor"
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

target_link_libraries(ffea Eigen3::Eigen)
