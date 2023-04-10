file(GLOB HEADERS "*.h")
file(GLOB SOURCES "*.cc")

add_library(base STATIC ${HEADERS} ${SOURCES})
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
)