include(FetchContent)

set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG 3.4
        SYSTEM
)

# Same loader libigl's recipe pins (cmake/recipes/external/glad.cmake).
FetchContent_Declare(
        glad
        GIT_REPOSITORY https://github.com/libigl/libigl-glad.git
        GIT_TAG 651a425101365aa6e8504988ef9bb363d066c5ee
        SYSTEM
)

# ImGui ships no CMake project; we compile its core and two backends ourselves.
FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.93.1
        SYSTEM
)
FetchContent_MakeAvailable(glfw glad imgui)
target_compile_options(glfw PRIVATE -w)
target_compile_options(glad PRIVATE -w)

add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/misc/cpp/imgui_stdlib.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
target_include_directories(imgui SYSTEM PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(imgui PUBLIC glfw)
target_compile_options(imgui PRIVATE -w)

add_executable(sim_viewer
        main.cc
        App.cc
        Window.cc
        Geometry.cc
        Renderer.cc
        Export.cc
        ViewerState.cc
        ui/Panels.cc
        ui/ScenePicker.cc
        ui/Playback.cc
        ui/Stats.cc
        ui/Export.cc)
target_include_directories(sim_viewer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(sim_viewer PRIVATE sim sim_scenes glfw glad imgui)
target_compile_options(sim_viewer PRIVATE ${SIM_WARNINGS})
# sim and glfw also arrive transitively; let CMake drop the repeats Apple ld warns about.
set_target_properties(sim_viewer PROPERTIES LINK_LIBRARIES_STRATEGY REORDER_FREELY)
