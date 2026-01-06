cmake_minimum_required(VERSION 3.15)
project(PathForge CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCASCADE REQUIRED)

find_package(VTK REQUIRED COMPONENTS
    CommonCore
    CommonDataModel
    CommonColor
    RenderingCore
    RenderingOpenGL2
    RenderingFreeType
    InteractionStyle
    FiltersCore
    IOPLY
    IOXML
)

file(GLOB SOURCES_ROOT      "*.cpp")
file(GLOB SOURCES_GEOMETRY  "Geometry/*.cpp")
file(GLOB SOURCES_TOPOLOGY  "Topology/*.cpp")
file(GLOB SOURCES_FEATURE  "Feature/*.cpp")

set(ALL_SOURCES
    ${SOURCES_ROOT}
    ${SOURCES_GEOMETRY}
    ${SOURCES_TOPOLOGY}
    ${SOURCES_FEATURE}
)

add_executable(PathForge ${ALL_SOURCES})

target_include_directories(PathForge PRIVATE
    ${OpenCASCADE_INCLUDE_DIR}
    ${VTK_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry
    ${CMAKE_CURRENT_SOURCE_DIR}/Topology
    ${CMAKE_CURRENT_SOURCE_DIR}/Feature
)

target_link_libraries(PathForge PRIVATE
    ${OpenCASCADE_LIBRARIES}
    VTK::CommonCore
    VTK::CommonDataModel
    VTK::CommonColor
    VTK::RenderingCore
    VTK::RenderingOpenGL2
    VTK::RenderingFreeType
    VTK::InteractionStyle
    VTK::FiltersCore
    VTK::IOPLY
    VTK::IOXML
)

if(WIN32)
    target_link_libraries(PathForge PRIVATE opengl32)
endif()

vtk_module_autoinit(
    TARGETS PathForge
    MODULES ${VTK_LIBRARIES}
)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${ALL_SOURCES})

message(STATUS "OpenCASCADE 版本: ${OpenCASCADE_VERSION}")
message(STATUS "VTK 版本: ${VTK_VERSION}")
