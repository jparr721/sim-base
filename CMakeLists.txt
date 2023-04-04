# To use VCPKG, uncomment
#set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
#message("Using Toolchain: ${CMAKE_TOOLCHAIN_FILE}")

cmake_minimum_required(VERSION 3.12)
project(base VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

file(GLOB SOURCES *.cc)
file(GLOB HEADERS *.h)

include(FetchContent)

option(LIBIGL_COPYLEFT_TETGEN "Build target igl_copyleft::tetgen" ON)
FetchContent_Declare(
        libigl
        GIT_REPOSITORY https://github.com/libigl/libigl.git
        GIT_TAG 66b3ef2253e765d0ce0db74cec91bd706e5ba176
)
FetchContent_MakeAvailable(libigl)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECT_NAME} PRIVATE
        igl::core
        igl_copyleft::tetgen
        Eigen3::Eigen
        GLUT::GLUT
        "-framework OpenGL")
