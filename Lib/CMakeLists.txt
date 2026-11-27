file(GLOB HEADERS "*.h" "Sim/*.h")
file(GLOB SOURCES "*.cc" "Sim/*.cc")

file(GLOB COLLISION_HEADERS "Collision/*.h")
file(GLOB COLLISION_SOURCES "Collision/*.cc")

file(GLOB INTEGRATOR_HEADERS "Integrator/*.h")
file(GLOB INTEGRATOR_SOURCES "Integrator/*.cc")

file(GLOB ENERGY_STRAND_HEADERS "Energy/Strand/*.h")
file(GLOB ENERGY_STRAND_SOURCES "Energy/Strand/*.cc")

# Listed by name: the other Energy/Volume files build as their own modules below.
set(ENERGY_VOLUME_HEADERS Energy/Volume/HyperelasticMaterial.h
        Energy/Volume/SNH.h Energy/Volume/STVK.h)
set(ENERGY_VOLUME_SOURCES Energy/Volume/SNH.cc Energy/Volume/STVK.cc)

file(GLOB ENERGY_COLLISION_HEADERS "Energy/Collision/*.h")
file(GLOB ENERGY_COLLISION_SOURCES "Energy/Collision/*.cc")

include(GNUInstallDirs)

# An energy module links the core math only (LibMath.h and Eigen).
add_library(sim_energy_mass_spring STATIC
        Energy/ElementEnergy.h
        Energy/Spring/MassSpring.h Energy/Spring/MassSpring.cc)
target_include_directories(sim_energy_mass_spring PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_options(sim_energy_mass_spring PRIVATE ${SIM_WARNINGS})
target_link_libraries(sim_energy_mass_spring PUBLIC Eigen3::Eigen)

# A volume energy module: Energy/Volume/<file>.{h,cc} as sim_energy_<name>.
function(sim_volume_energy name file)
    add_library(sim_energy_${name} STATIC Energy/Volume/Energy.h
            Energy/Volume/${file}.h Energy/Volume/${file}.cc)
    target_include_directories(sim_energy_${name} PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
    target_compile_options(sim_energy_${name} PRIVATE ${SIM_WARNINGS})
    target_link_libraries(sim_energy_${name} PUBLIC Eigen3::Eigen)
endfunction()
sim_volume_energy(stable_neo_hookean StableNeoHookean)

add_library(sim STATIC
        ${HEADERS} ${SOURCES}
        ${COLLISION_HEADERS} ${COLLISION_SOURCES}
        ${INTEGRATOR_HEADERS} ${INTEGRATOR_SOURCES}
        ${ENERGY_STRAND_HEADERS} ${ENERGY_STRAND_SOURCES}
        ${ENERGY_VOLUME_HEADERS} ${ENERGY_VOLUME_SOURCES}
        ${ENERGY_COLLISION_HEADERS} ${ENERGY_COLLISION_SOURCES})
target_include_directories(sim PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_options(sim PRIVATE ${SIM_WARNINGS})
install(TARGETS sim sim_energy_mass_spring EXPORT sim)
target_link_libraries(
        sim
        PUBLIC
        sim_energy_mass_spring
        igl::core
        igl_copyleft::tetgen
        Eigen3::Eigen
)
add_library(sim::sim ALIAS sim)
