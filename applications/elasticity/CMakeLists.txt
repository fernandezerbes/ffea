add_library(ffea_elasticity
            src/constitutive_model.cc
            src/postprocessor.cc
)

target_include_directories(ffea_elasticity INTERFACE
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc"
)

target_link_libraries(ffea_elasticity ffea)
