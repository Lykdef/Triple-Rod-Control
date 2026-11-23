cmake_minimum_required(VERSION 3.20)

project(triple VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(eigen)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.3.8
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(glfw)

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.89.9
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(imgui)
find_package(OpenGL REQUIRED)

# Physics, sensing and control are independent of the windowing libraries.
add_library(triple_core
    src/model.cpp
    src/integrator.cpp
    src/collision.cpp
    src/sensors.cpp
    src/lqr.cpp
    src/automatic_controller.cpp
    src/swing_up_data.cpp
    src/simulation.cpp
)
target_include_directories(triple_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(triple_core PUBLIC Eigen3::Eigen)

add_executable(triple
    src/main.cpp
    src/scene.cpp
    src/window.cpp
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(triple PRIVATE
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(triple PRIVATE triple_core glfw OpenGL::GL)

enable_testing()
add_executable(triple_tests tests/control_tests.cpp)
target_link_libraries(triple_tests PRIVATE triple_core)
add_test(NAME control_tests COMMAND triple_tests)
