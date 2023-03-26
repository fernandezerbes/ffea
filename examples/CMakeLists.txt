add_executable(cantilever_beam cantilever_beam.cc)
add_executable(heat_sink heat_sink.cc)

target_link_libraries(cantilever_beam PUBLIC ffea)
target_link_libraries(heat_sink PUBLIC ffea)

target_link_libraries(cantilever_beam PUBLIC ffea_elasticity)
target_link_libraries(heat_sink PUBLIC ffea_quasi_harmonic)
