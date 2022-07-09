add_library(ffea
            src/analysis/Analysis.cc
            src/assembly/Assembler.cc
            src/math/Utils.cc
            src/mesh/Coordinates.cc
            src/mesh/Line2.cc
            src/mesh/DegreeOfFreedom.cc
            src/mesh/Element.cc
            src/mesh/IntegrationPoint.cc
            src/mesh/Node.cc
            src/mesh/NodeElement.cc
            src/mesh/SurfaceElement.cc
            src/mesh/VolumeElement.cc
)

target_include_directories(ffea INTERFACE
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/analysis"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/assembly"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/math"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/mesh"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/model"
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc/processor"
)
