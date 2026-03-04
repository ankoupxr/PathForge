cmake_minimum_required(VERSION 3.15)
project(PathForge CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 使用 UTF-8 编码处理源文件
if(MSVC)
    add_compile_options(/utf-8)
endif()

find_package(OpenCASCADE REQUIRED)

find_package(VTK REQUIRED COMPONENTS
    CommonCore
    CommonDataModel
    CommonColor
    RenderingCore
    RenderingOpenGL2
    RenderingFreeType
    RenderingAnnotation
    RenderingUI
    InteractionStyle
    InteractionImage
    FiltersCore
    IOPLY
    IOXML
)

file(GLOB SOURCES_ROOT      "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB SOURCES_GEOMETRY  "${CMAKE_CURRENT_SOURCE_DIR}/Geometry/*.cpp")
file(GLOB SOURCES_TOPOLOGY  "${CMAKE_CURRENT_SOURCE_DIR}/Topology/*.cpp")
file(GLOB SOURCES_PATH      "${CMAKE_CURRENT_SOURCE_DIR}/Path/*.cpp")
file(GLOB SOURCES_PATH_OPERATIONS "${CMAKE_CURRENT_SOURCE_DIR}/Path/Operations/*.cpp")
file(GLOB SOURCES_PATH_POSTPROCESS "${CMAKE_CURRENT_SOURCE_DIR}/Path/PostProcess/*.cpp")
file(GLOB SOURCES_PATH_STRATEGIES "${CMAKE_CURRENT_SOURCE_DIR}/Path/Strategies/*.cpp")
file(GLOB SOURCES_PATH_PROJECTION "${CMAKE_CURRENT_SOURCE_DIR}/Path/Projection/*.cpp")
file(GLOB SOURCES_PATH_MULTIAXIS "${CMAKE_CURRENT_SOURCE_DIR}/Path/MultiAxis/*.cpp")
file(GLOB SOURCES_PATH_OPTIMIZATION "${CMAKE_CURRENT_SOURCE_DIR}/Path/Optimization/*.cpp")

set(ALL_SOURCES
    ${SOURCES_ROOT}
    ${SOURCES_GEOMETRY}
    ${SOURCES_TOPOLOGY}
    ${SOURCES_PATH}
    ${SOURCES_PATH_OPERATIONS}
    ${SOURCES_PATH_POSTPROCESS}
    ${SOURCES_PATH_STRATEGIES}
    ${SOURCES_PATH_PROJECTION}
    ${SOURCES_PATH_MULTIAXIS}
    ${SOURCES_PATH_OPTIMIZATION}
)

# 打印用于诊断的源文件列表
message(STATUS "ALL_SOURCES count: ${ALL_SOURCES}")

add_executable(PathForge ${ALL_SOURCES})

target_include_directories(PathForge PRIVATE
    ${OpenCASCADE_INCLUDE_DIR}
    ${VTK_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry
    ${CMAKE_CURRENT_SOURCE_DIR}/Topology
    ${CMAKE_CURRENT_SOURCE_DIR}/Path
    ${CMAKE_CURRENT_SOURCE_DIR}/Path/Strategies
    ${CMAKE_CURRENT_SOURCE_DIR}/Path/Projection
)

target_link_libraries(PathForge PRIVATE
    ${OpenCASCADE_LIBRARIES}
    VTK::CommonCore
    VTK::CommonDataModel
    VTK::CommonColor
    VTK::RenderingCore
    VTK::RenderingOpenGL2
    VTK::RenderingFreeType
    VTK::RenderingAnnotation
    VTK::RenderingUI
    VTK::InteractionStyle
    VTK::InteractionImage
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

# 复制 DLL 到输出目录
if(WIN32)
    set(OUTPUT_DIR "${CMAKE_BINARY_DIR}")
    set(OCC_DLL_DIR "D:/myself/OCC/occt-vc14-64/win64/vc14/bin")
    set(OCCT_3RDPARTY "D:/myself/OCC/3rdparty-vc14-64")

    # OpenCASCADE DLL
    add_custom_command(TARGET PathForge POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${OCC_DLL_DIR}" "${OUTPUT_DIR}"
    )

    # VTK
    if(EXISTS "${OCCT_3RDPARTY}/vtk-9.4.1-x64/bin")
        add_custom_command(TARGET PathForge POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${OCCT_3RDPARTY}/vtk-9.4.1-x64/bin" "${OUTPUT_DIR}"
        )
    endif()

    # TBB
    if(EXISTS "${OCCT_3RDPARTY}/tbb-2021.13.0-x64/bin")
        add_custom_command(TARGET PathForge POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${OCCT_3RDPARTY}/tbb-2021.13.0-x64/bin" "${OUTPUT_DIR}"
        )
    endif()

    # MSVC Runtime
    if(EXISTS "${OCCT_3RDPARTY}/msvc-vc14-64")
        add_custom_command(TARGET PathForge POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${OCCT_3RDPARTY}/msvc-vc14-64" "${OUTPUT_DIR}"
        )
    endif()

    # FreeImage
    if(EXISTS "${OCCT_3RDPARTY}/freeimage-3.18.0-x64/bin")
        add_custom_command(TARGET PathForge POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${OCCT_3RDPARTY}/freeimage-3.18.0-x64/bin" "${OUTPUT_DIR}"
        )
    endif()

    # lzma
    if(EXISTS "${OCCT_3RDPARTY}/lzma-5.2.2-vc14-64/bin")
        add_custom_command(TARGET PathForge POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${OCCT_3RDPARTY}/lzma-5.2.2-vc14-64/bin" "${OUTPUT_DIR}"
        )
    endif()

    # gl2ps
    if(EXISTS "${OCCT_3RDPARTY}/gl2ps-1.3.8-vc14-64/bin")
        add_custom_command(TARGET PathForge POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${OCCT_3RDPARTY}/gl2ps-1.3.8-vc14-64/bin" "${OUTPUT_DIR}"
        )
    endif()

    # jemalloc
    if(EXISTS "${OCCT_3RDPARTY}/jemalloc-vc14-64/bin")
        add_custom_command(TARGET PathForge POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${OCCT_3RDPARTY}/jemalloc-vc14-64/bin" "${OUTPUT_DIR}"
        )
    endif()

    # Qt5
    if(EXISTS "${OCCT_3RDPARTY}/qt5.11.2-vc14-64/bin")
        add_custom_command(TARGET PathForge POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${OCCT_3RDPARTY}/qt5.11.2-vc14-64/bin" "${OUTPUT_DIR}"
        )
    endif()

    # GLFW
    if(EXISTS "${OCCT_3RDPARTY}/glfw-3.4-x64/lib-vc2022")
        add_custom_command(TARGET PathForge POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${OCCT_3RDPARTY}/glfw-3.4-x64/lib-vc2022" "${OUTPUT_DIR}"
        )
    endif()

    # ffmpeg
    if(EXISTS "${OCCT_3RDPARTY}/ffmpeg-3.3.4-64/bin")
        add_custom_command(TARGET PathForge POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${OCCT_3RDPARTY}/ffmpeg-3.3.4-64/bin" "${OUTPUT_DIR}"
        )
    endif()

    # draco
    if(EXISTS "${OCCT_3RDPARTY}/draco-1.4.1-vc14-64/lib")
        add_custom_command(TARGET PathForge POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${OCCT_3RDPARTY}/draco-1.4.1-vc14-64/lib" "${OUTPUT_DIR}"
        )
    endif()

    # openvr
    if(EXISTS "${OCCT_3RDPARTY}/openvr-1.14.15-64/bin/win64")
        add_custom_command(TARGET PathForge POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${OCCT_3RDPARTY}/openvr-1.14.15-64/bin/win64" "${OUTPUT_DIR}"
        )
    endif()

    # angle
    if(EXISTS "${OCCT_3RDPARTY}/angle-gles2-2.1.0-vc14-64/bin")
        add_custom_command(TARGET PathForge POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${OCCT_3RDPARTY}/angle-gles2-2.1.0-vc14-64/bin" "${OUTPUT_DIR}"
        )
    endif()
endif()

message(STATUS "OpenCASCADE 版本：${OpenCASCADE_VERSION}")
message(STATUS "VTK 版本：${VTK_VERSION}")
