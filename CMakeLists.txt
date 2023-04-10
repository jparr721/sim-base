# To use VCPKG, uncomment
#set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
#message("Using Toolchain: ${CMAKE_TOOLCHAIN_FILE}")

cmake_minimum_required(VERSION 3.12)
project(base VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

set(CMAKE_BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(TARGET_NAME ${PROJECT_NAME}_${CMAKE_BUILD_TYPE})
set(GL_SILENCE_DEPRECATION ON)
set(INSTALL_LIB_DIR lib CACHE PATH "Install directory for library code")
set(INSTALL_BIN_DIR CACHE PATH "Install directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Install directory for header files")

include(FetchContent)

option(LIBIGL_COPYLEFT_TETGEN "Build target igl_copyleft::tetgen" ON)
FetchContent_Declare(
        libigl
        GIT_REPOSITORY https://github.com/libigl/libigl.git
        GIT_TAG 66b3ef2253e765d0ce0db74cec91bd706e5ba176
)
FetchContent_MakeAvailable(libigl)

option(BUILD_TESTING "Build tests" ON)
if (BUILD_TESTING)
    enable_testing()
    include(GoogleTest)
    add_subdirectory(Tests)
endif ()

find_package(OpenGL REQUIRED)
add_subdirectory(Lib)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
add_executable(${TARGET_NAME} main.cc)
target_link_libraries(${TARGET_NAME} PRIVATE ${OPENGL_LIBRARIES} GLUT::GLUT ${PROJECT_NAME})
