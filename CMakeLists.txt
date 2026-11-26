# To use VCPKG, uncomment
#set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
#message("Using Toolchain: ${CMAKE_TOOLCHAIN_FILE}")

cmake_minimum_required(VERSION 3.25)
project(base VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SIM_WARNINGS -Wall -Wextra -Wpedantic)

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

set(GL_SILENCE_DEPRECATION ON)

# libigl pulls deps (tetgen) whose cmake_minimum_required predates what CMake 4 accepts.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
include(FetchContent)

option(LIBIGL_COPYLEFT_TETGEN "Build target igl_copyleft::tetgen" ON)
FetchContent_Declare(
        libigl
        GIT_REPOSITORY https://github.com/libigl/libigl.git
        GIT_TAG v2.6.4
        SYSTEM
)
FetchContent_MakeAvailable(libigl)
# Third-party sources are not ours to fix; keep their warnings out of the build log.
target_compile_options(tetgen PRIVATE -w)

option(BUILD_TESTING "Build tests" ON)
if (BUILD_TESTING)
    enable_testing()
    include(GoogleTest)
    add_subdirectory(Tests)
endif ()

find_package(OpenGL REQUIRED)
add_subdirectory(Lib)
add_subdirectory(SceneCatalog)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
add_subdirectory(viewer)


