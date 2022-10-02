add_library(ffea_quasi_harmonic
            src/constitutive_model.cc
            src/physics_processor.cc
)

target_include_directories(ffea_quasi_harmonic INTERFACE
                           "${CMAKE_CURRENT_SOURCE_DIR}/inc"
)

target_link_libraries(ffea_quasi_harmonic ffea)
