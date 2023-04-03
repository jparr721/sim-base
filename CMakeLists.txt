set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
message("Using Toolchain: ${CMAKE_TOOLCHAIN_FILE}")

cmake_minimum_required(VERSION 3.12)
project(base VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

find_package(Eigen3 CONFIG REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

file(GLOB SOURCES *.cc)

add_executable(${PROJECT_NAME} ${SOURCES} main.cc)

target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen GLUT::GLUT "-framework OpenGL")
