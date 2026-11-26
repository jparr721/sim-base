add_library(sim_scenes STATIC Catalog.h Catalog.cc)
target_include_directories(sim_scenes PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(sim_scenes PUBLIC sim)
target_compile_options(sim_scenes PRIVATE ${SIM_WARNINGS})
