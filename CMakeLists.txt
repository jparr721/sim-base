# To use VCPKG, uncomment
#set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
#message("Using Toolchain: ${CMAKE_TOOLCHAIN_FILE}")

cmake_minimum_required(VERSION 3.12)
project(base VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

file(GLOB SOURCES *.cc)

# Eigen
option(EIGEN_BUILD_DOC "Enable creation of Eigen documentation" OFF)
option(EIGEN_BUILD_TESTING "Enable creation of Eigen tests." OFF)
add_subdirectory(ext/eigen)
add_subdirectory(ext/tetgen)
add_executable(${PROJECT_NAME} ${SOURCES} main.cc)

target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen GLUT::GLUT tetgen "-framework OpenGL")
