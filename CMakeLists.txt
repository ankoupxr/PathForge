	cmake_minimum_required(VERSION 3.15)
	project(PathForge CXX)
	set(CMAKE_CXX_STANDARD 17)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
	if(MSVC)
	    add_compile_options(/utf-8)
	endif()
	find_package(OpenCASCADE REQUIRED)

	find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

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
	    GUISupportQt
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

	message(STATUS "ALL_SOURCES count: ${ALL_SOURCES}")
	add_executable(PathForge ${ALL_SOURCES})
	target_include_directories(PathForge PRIVATE
	    ${OpenCASCADE_INCLUDE_DIR}
	    ${VTK_INCLUDE_DIRS}
	    ${Qt5Core_INCLUDE_DIRS}
	    ${Qt5Widgets_INCLUDE_DIRS}
	    ${CMAKE_CURRENT_SOURCE_DIR}
	    ${CMAKE_CURRENT_SOURCE_DIR}/Geometry
	    ${CMAKE_CURRENT_SOURCE_DIR}/Topology
	    ${CMAKE_CURRENT_SOURCE_DIR}/Path
	    ${CMAKE_CURRENT_SOURCE_DIR}/Path/Strategies
	    ${CMAKE_CURRENT_SOURCE_DIR}/Path/Projection
	)

	target_link_libraries(PathForge PRIVATE
	    TKernel
		TKMath
		TKBRep
		TKGeomBase
		TKGeomAlgo
		TKTopAlgo
		TKPrim
		TKShHealing
		TKMesh
		TKService
		TKV3d
		TKXSBase  # <--- ÐÂÔö
		TKDESTEP
		TKDESTL
		TKOffset
		TKDEIGES

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
	    VTK::GUISupportQt # <--- é“¾æŽ¥ Qt æ”¯æŒæ¨¡å—
	    Qt5::Core
	    Qt5::Widgets
	    Qt5::Gui
	)
	if(WIN32)
	    target_link_libraries(PathForge PRIVATE opengl32)
	endif()
	# Qt MOC
	set_target_properties(PathForge PROPERTIES
	    AUTOMOC ON
	    AUTOUIC ON
	    AUTORCC ON
	)
	source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${ALL_SOURCES})
	# å¤åˆ¶ DLL åˆ°è¾“å‡ºç›®å½?
	if(WIN32)
		set(OUTPUT_DIR "${CMAKE_BINARY_DIR}")
		set(OCC_DLL_DIR "G:/OpenCASCADE-7.7.0-vc14-64/opencascade-7.9.3-vc14-64/win64/vc14/bin")
		set(OCCT_3RDPARTY "G:/OpenCASCADE-7.7.0-vc14-64/3rdparty-vc14-64")
	    # OpenCASCADE DLL
	    add_custom_command(TARGET PathForge POST_BUILD
	        COMMAND ${CMAKE_COMMAND} -E copy_directory "${OCC_DLL_DIR}" "${OUTPUT_DIR}"
	    )
	    # VTK
	    if(EXISTS "G:/env/vtk/bin")
	        add_custom_command(TARGET PathForge POST_BUILD
	            COMMAND ${CMAKE_COMMAND} -E copy_directory "G:/env/vtk/bin" "${OUTPUT_DIR}"
	        )
	    endif()
		# Qt5
			set(QT_DIR "F:/qt/5.15.2/msvc2019_64")
		if(EXISTS "${QT_DIR}/bin")
			# ¸´ÖÆºËÐÄ DLL
			add_custom_command(TARGET PathForge POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_directory "${QT_DIR}/bin" "${OUTPUT_DIR}"
			)
			
			# ÐÞ¸Ä 2: ÏÔÊ½¸´ÖÆ platforms ²å¼þÄ¿Â¼£¬½â¾ö "platform plugin" ´íÎó
			add_custom_command(TARGET PathForge POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E make_directory "${OUTPUT_DIR}/platforms"
				COMMAND ${CMAKE_COMMAND} -E copy_if_different "${QT_DIR}/plugins/platforms/qwindows.dll" "${OUTPUT_DIR}/platforms"
				COMMAND ${CMAKE_COMMAND} -E copy_if_different "${QT_DIR}/plugins/platforms/qwindowsd.dll" "${OUTPUT_DIR}/platforms"
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
	endif()
	message(STATUS "OpenCASCADE ç‰ˆæœ¬ï¼?{OpenCASCADE_VERSION}")
	message(STATUS "VTK ç‰ˆæœ¬ï¼?{VTK_VERSION}")
