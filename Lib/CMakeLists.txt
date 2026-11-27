file(GLOB HEADERS "*.h" "Sim/*.h")
file(GLOB SOURCES "*.cc" "Sim/*.cc")
# TetMesh is sim_solid's geometry.
list(REMOVE_ITEM HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/TetMesh.h")
list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/TetMesh.cc")

file(GLOB COLLISION_HEADERS "Collision/*.h")
file(GLOB COLLISION_SOURCES "Collision/*.cc")

file(GLOB INTEGRATOR_HEADERS "Integrator/*.h")
file(GLOB INTEGRATOR_SOURCES "Integrator/*.cc")

file(GLOB ENERGY_STRAND_HEADERS "Energy/Strand/*.h")
file(GLOB ENERGY_STRAND_SOURCES "Energy/Strand/*.cc")

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
sim_volume_energy(stvk StVenantKirchhoff)

# Tet geometry and its global assembly: links the volume energies, never sim.
add_library(sim_solid STATIC
        TetMesh.h TetMesh.cc
        Solid/System.h
        Solid/Constraints.h Solid/Constraints.cc
        Solid/ElasticSystem.h Solid/ElasticSystem.cc
        Energy/Volume/Material.h)
target_include_directories(sim_solid PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_options(sim_solid PRIVATE ${SIM_WARNINGS})
target_link_libraries(sim_solid PUBLIC
        sim_energy_stable_neo_hookean
        sim_energy_stvk
        igl::core
        igl_copyleft::tetgen
        Eigen3::Eigen)

add_library(sim STATIC
        ${HEADERS} ${SOURCES}
        ${COLLISION_HEADERS} ${COLLISION_SOURCES}
        ${INTEGRATOR_HEADERS} ${INTEGRATOR_SOURCES}
        ${ENERGY_STRAND_HEADERS} ${ENERGY_STRAND_SOURCES}
        ${ENERGY_COLLISION_HEADERS} ${ENERGY_COLLISION_SOURCES})
target_include_directories(sim PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_options(sim PRIVATE ${SIM_WARNINGS})
install(TARGETS sim sim_solid sim_energy_mass_spring
        sim_energy_stable_neo_hookean sim_energy_stvk EXPORT sim)
target_link_libraries(
        sim
        PUBLIC
        sim_energy_mass_spring
        sim_solid
        igl::core
        Eigen3::Eigen
)
add_library(sim::sim ALIAS sim)
