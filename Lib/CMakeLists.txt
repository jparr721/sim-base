file(GLOB HEADERS "*.h")
file(GLOB SOURCES "*.cc")

file(GLOB SCENES_HEADERS "Scenes/*.h")
file(GLOB SCENES_SOURCES "Scenes/*.cc")

file(GLOB ENERGY_STRAND_HEADERS "Energy/Strand/*.h")
file(GLOB ENERGY_STRAND_SOURCES "Energy/Strand/*.cc")

file(GLOB ENERGY_VOLUME_HEADERS "Energy/Volume/*.h")
file(GLOB ENERGY_VOLUME_SOURCES "Energy/Volume/*.cc")

file(GLOB ENERGY_SPRING_HEADERS "Energy/Spring/*.h")
file(GLOB ENERGY_SPRING_SOURCES "Energy/Spring/*.cc")

add_library(base STATIC
        ${HEADERS} ${SOURCES}
        ${SCENES_HEADERS} ${SCENES_SOURCES}
        ${ENERGY_STRAND_HEADERS} ${ENERGY_STRAND_SOURCES}
        ${ENERGY_VOLUME_HEADERS} ${ENERGY_VOLUME_SOURCES}
        ${ENERGY_SPRING_HEADERS} ${ENERGY_SPRING_SOURCES})
include(GNUInstallDirs)
target_include_directories(base PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
install(TARGETS base EXPORT base)
target_link_libraries(
        base
        PUBLIC
        igl::core
        igl_copyleft::tetgen
        Eigen3::Eigen
        GLUT::GLUT
)