cmake_minimum_required(VERSION 2.8)
project(Aquila)
CMAKE_POLICY(SET CMP0020 NEW)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_DEBUG_POSTFIX d)
include(cmake/utilities.cmake)
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
IF(WIN32)
  add_definitions(-DWIN32)
  add_definitions(-D_WIN32)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
  ADD_DEFINITIONS(-D_SCL_SECURE_NO_WARNINGS)
  ADD_DEFINITIONS(-D_WIN32_WINNT=0x0601)
  ADD_DEFINITIONS(-D_ENABLE_ATOMIC_ALIGNMENT_FIX)
ELSE(WIN32)
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-reorder -Wno-sign-compare -Wl,--no-as-needed -O0")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -g")
  SET(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin/Plugins" CACHE PATH "Library output path" FORCE)
  SET(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin" CACHE PATH "Executable output path" FORCE)
ENDIF(WIN32)

ADD_SUBDIRECTORY("dependencies/MetaObject")
set(RCC_PERMODULE_INTERFACE_SOURCE
    ${RCC_PERMODULE_INTERFACE_SOURCE}
    CACHE PATH "" FORCE
)
include_directories(${RCC_INCLUDE_DIRS})
ADD_SUBDIRECTORY("dependencies/pplx")

find_package(OpenCV 3.0 QUIET COMPONENTS  core imgproc highgui cudaimgproc cudawarping cudafeatures2d cudaoptflow cudacodec cudabgsegm)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)

find_package(CUDA REQUIRED)
  set(CUDA_ARCH_NAME "Manual")
  set(CUDA_ARCH_BIN "30 35 50 60")
  set(CUDA_ARCH_PTX "30 35 50 60")
get_filename_component(_cuda_lib_dir "${CUDA_CUDART_LIBRARY}" DIRECTORY)
link_directories(${_cuda_lib_dir})
find_package(Qt5 COMPONENTS Core Gui Widgets Network OpenGL)
cmake_policy(SET CMP0012 NEW)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

IF(WIN32)
    SET(CUDA_DEVRT ${CUDA_TOOLKIT_ROOT_DIR}/lib/x64/cudadevrt.lib)
    SET(CUDA_LIB_DIR ${CUDA_TOOLKIT_ROOT_DIR}/lib/x64/)
ELSE(WIN32)
    SET(CUDA_DEVRT ${CUDA_TOOLKIT_ROOT_DIR}/lib64/cudadevrt)
    SET(CUDA_LIB_DIR ${CUDA_TOOLKIT_ROOT_DIR}/lib64/)
ENDIF(WIN32)
SET(LINK_LIBS "")
ADD_DEFINITIONS(-DCVAPI_EXPORTS)
ADD_DEFINITIONS(-DOPENCV_FOUND)
ADD_DEFINITIONS(-DAquila_EXPORTS)
ADD_DEFINITIONS(-DRMT_DLL)
ADD_DEFINITIONS(-DBUILD_DIR=${CMAKE_BINARY_DIR})
ADD_DEFINITIONS(-DCVAPI_EXPORTS)
ADD_DEFINITIONS(-DOPENCV_FOUND)
ADD_DEFINITIONS(-DAquila_EXPORTS)
IF(${NV_TOOLS_EXT_FOUND})
  ADD_DEFINITIONS(-DUSE_NVTX)
  list(APPEND LINK_LIBS ${NV_TOOLS_EXT_LIB})
  include_directories(${NV_TOOLS_EXT_INCLUDE_DIR})
  SET(NVTX_BIN_DIR_OPT "${NV_TOOLS_EXT_BIN_DIR}" CACHE PATH "")
  SET(NVTX_BIN_DIR_DBG "${NV_TOOLS_EXT_BIN_DIR}" CACHE PATH "")
  SET(BIN_DIRS "${BIN_DIRS};NVTX" CACHE STRING "" FORCE)
endif()
LIST(APPEND defs_ -DRMT_USE_CUDA)
LIST(APPEND defs_ -DRMT_USE_OPENGL)
IF(${Aquila_JETSON})
    LIST(APPEND defs_ "-DJETSON")
ENDIF(${Aquila_JETSON})

set(BOOST_REQUIRED_MODULES 
	system 
	filesystem 
	thread 
	date_time 
	iostreams 
	chrono 
	regex 
	log 
	log_setup 
	atomic 
    unit_test_framework
)
set(Boost_USE_STATIC_LIBS        OFF)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     OFF)
ADD_DEFINITIONS(-DBOOST_ALL_DYN_LINK)
find_package(Boost 1.47.0 QUIET COMPONENTS ${BOOST_REQUIRED_MODULES}) 

FILE(GLOB_RECURSE src "src/*.cpp" "remotery/lib/*.c")
FILE(GLOB_RECURSE hdr "remotery/lib/*.h" "include/*.hpp" "include/*.h" "src/*.hpp")
FILE(GLOB_RECURSE ext_inc "include/Aquila/rcc/external_includes")
FILE(GLOB_RECURSE knl "src/*.cu")


find_package(mxnet QUIET)
if(${MXNET_FOUND})
  INCLUDE_DIRECTORIES(${MXNET_INCLUDE_DIRS})
  add_definitions(-DHAVE_MXNET)
  add_definitions(-DMSHADOW_USE_CBLAS=1)
  add_definitions(-DMSHADOW_USE_CUDA=1)
  add_definitions(-DMSHADOW_USE_MKL=0)
  add_definitions(-DMSHADOW_USE_CUDNN=1)
endif()


include_directories("${CMAKE_CURRENT_SOURCE_DIR}/dependencies/MetaObject/dependencies/cereal/include")

INCLUDE_DIRECTORIES(
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    ${Parameters_INCLUDE}
    ${Qt5Gui_INCLUDE_DIRS} 
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Network_INCLUDE_DIRS}
    ${PPLX_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/remotery/lib
    )


set_target_properties(pplx PROPERTIES FOLDER Dependencies)
set_target_properties(MetaObject PROPERTIES FOLDER Dependencies)
IF(NOT WIN32)
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -D_DEBUG")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")
ENDIF()


#===================================================================
#Export variables for other projects to use

SET(LIB_BUILD_DIR_RELEASE "")
SET(LIB_BUILD_DIR_DEBUG "")

IF(WIN32)
    SET(LIB_BUILD_DIR_RELEASE "${CMAKE_CURRENT_BINARY_DIR}/RelWithDebInfo")
    SET(LIB_BUILD_DIR_DEBUG "${CMAKE_CURRENT_BINARY_DIR}/Debug")
ELSE(WIN32)
    SET(LIB_BUILD_DIR_RELEASE "${LIBRARY_OUTPUT_PATH};${CMAKE_BINARY_DIR}")
    SET(LIB_BUILD_DIR_DEBUG "${LIBRARY_OUTPUT_PATH};${CMAKE_BINARY_DIR}")
ENDIF(WIN32)

set(Aquila_INCLUDE_DIRECTORIES
    "${CMAKE_CURRENT_SOURCE_DIR}/MetaObject/dependencies/cereal/include;"
    "${CMAKE_CURRENT_SOURCE_DIR}/MetaObject/instantiations;"
    "${CMAKE_CURRENT_SOURCE_DIR}/include;"
    "${CMAKE_CURRENT_SOURCE_DIR}/remotery/lib;"
    "${Boost_INCLUDE_DIRS};${OpenCV_INCLUDE_DIRS};"
    "${RCC_INCLUDE_DIRS};"
    "${Qt5Gui_INCLUDE_DIRS};"
    "${Qt5Widgets_INCLUDE_DIRS};"
    "${Qt5Network_INCLUDE_DIRS};"
	"${EIGEN3_INCLUDE_DIR};"
    "${PPLX_INCLUDE_DIRS}" CACHE PATH "" FORCE
)

set(Aquila_LIBRARIES
    "Aquila;"
    "${Boost_LIBRARIES};"
    "${Parameters_LIBRARIES};"
    "pplx" CACHE PATH "" FORCE
)

set(Aquila_LIBS
    "Aquila;"
    "${Parameters_LIBRARIES};"
    "pplx"
)

SET(Aquila_LIB_DIRS_DEBUG
    "${LIB_BUILD_DIR_DEBUG};"
    "${Boost_LIBRARY_DIR};"
    "${Boost_LIBRARY_DIR_DEBUG};"
    "${CUDA_LIB_DIR}"
    CACHE PATH "" FORCE
)

SET(Aquila_LIB_DIRS_RELEASE
    "${LIB_BUILD_DIR_RELEASE};"
    "${Boost_LIBRARY_DIR};"
    "${Boost_LIBRARY_DIR_RELEASE};"
    "${CUDA_LIB_DIR}"
    CACHE PATH "" FORCE
)

SET(Aquila_DEFS "${defs_}"
    CACHE PATH "" FORCE
)

CONFIGURE_FILE(AquilaConfig.cmake.in
    ${CMAKE_BINARY_DIR}/AquilaConfig.cmake @ONLY
)

# Link directories
LINK_DIRECTORIES(${Parameters_LIB_DIRS_DEBUG})
LINK_DIRECTORIES(${Parameters_LIB_DIRS_RELEASE})
LINK_DIRECTORIES(${Boost_LIBRARY_DIR})
LINK_DIRECTORIES(${Boost_LIBRARY_DIR_RELEASE})
# Reporting section

if(RCC_VERBOSE_CONFIG)
message(STATUS
"================ Aquila ===============
    Current source dir: ${CMAKE_CURRENT_SOURCE_DIR}
    OpenCV found at: ${OpenCV_CONFIG_PATH}
    NVCC Flags: ${CUDA_NVCC_FLAGS}
    Jetson: ${Aquila_JETSON}
    Bulid Type: ${CMAKE_BUILD_TYPE}

    Includes: ${Aquila_INCLUDE_DIRECTORIES}

    Libs: ${Aquila_LIBRARIES}

    Lib dirs:
    ${Aquila_LIB_DIRS_DEBUG}

    ${Aquila_LIB_DIRS_RELEASE}

    Definitions: ${defs_}


++++++++++ RCC++ Defines ++++++++++++++++++
    OpenCV include dir: ${OpenCV_INCLUDE_DIR}
    Boost library dir: ${Boost_LIBRARY_DIRS}
    OpenCV library dir: ${CV_LIB_DIR}
    Cuda include dir: ${CUDA_INCLUDE_DIRS}
    NVCC: ${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc
    Cuda RT libs: ${CUDA_LIBRARIES}
    Cuda Dev RT libs: ${CUDA_DEVRT}
    Cuda root dir: ${CUDA_TOOLKIT_ROOT_DIR}
    Binary Dir: ${CMAKE_BINARY_DIR}

    Boost LIBS: ${Boost_LIBRARIES}

    PCL_found: ${PCL_FOUND}
    PCL_LIB_DIR: ${PCL_LIBRARY_DIRS}
    Eigen inc: ${EIGEN3_INCLUDE_DIR}
++++++++++++ Parameters ++++++++++++++++++++
    Includes: ${Parameters_INCLUDE}
    UI: ${Parameters_UI_qt}
")
endif()
ADD_DEFINITIONS(${defs_})


macro(GroupSources curdir)
file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir} ${PROJECT_SOURCE_DIR}/${curdir}/*)
    foreach(child ${children})
        if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
            GroupSources(${curdir}/${child})
        else()
            string(REPLACE "/" "\\" groupname ${curdir})
            source_group(${groupname} FILES ${PROJECT_SOURCE_DIR}/${curdir}/${child})
        endif()
   endforeach()
endmacro()
 
GroupSources(include)
GroupSources(src)

cuda_add_library(Aquila SHARED
    ${src}  
    ${hdr}
    ${knl}
    ${RuntimeObjectSystem_SRCS}
    src/precompiled.hpp
)
ocv_add_precompiled_header_to_target(Aquila src/precompiled.hpp)
SOURCE_GROUP("Header Files\\External" FILES ${ext_inc})


TARGET_LINK_LIBRARIES(Aquila
    ${OpenCV_LIBS}
    pplx
    ${Parameters_LIBRARIES}
    ${CUDA_CUDART_LIBRARY}
    Qt5::OpenGL
    RuntimeObjectSystem
    RuntimeCompiler
    ${LINK_LIBS}
    MetaObject
    ${Boost_LIBRARIES}
)

if(MXNET_FOUND)
  TARGET_LINK_LIBRARIES(Aquila ${MXNET_LIBRARY})
endif()

if(WIN32)
    target_link_libraries(Aquila OpenGL32)
else(WIN32)
  if(UNIX)
    target_link_libraries(Aquila GL GLU)
  endif(UNIX)
endif(WIN32)

ADD_DEPENDENCIES(Aquila
    pplx
    RuntimeObjectSystem
    RuntimeCompiler
)

install(TARGETS Aquila
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.hpp"
)

install(FILES ${CMAKE_BINARY_DIR}/AquilaConfig.cmake
        DESTINATION ./
)

if(WIN32)
    install(DIRECTORY ${CMAKE_BINARY_DIR}/Debug/
        DESTINATION bin
        FILES_MATCHING PATTERN "*.pdb")
    install(DIRECTORY ${CMAKE_BINARY_DIR}/RelWithDebInfo/
        DESTINATION bin
        FILES_MATCHING PATTERN "*.pdb")
endif(WIN32)


if(UNIX)
    TARGET_LINK_LIBRARIES(Aquila dl)
endif()
if(WITH_PYTHON)
  add_subdirectory("python")
endif()

if(Boost_UNIT_TEST_FRAMEWORK_FOUND AND BUILD_TESTS)
    include(CTest)
    enable_testing()
    SUBDIRLIST(tests "${CMAKE_CURRENT_LIST_DIR}/tests")
    foreach(test ${tests})
        file(GLOB_RECURSE test_srcs "tests/${test}/*.cpp")
        file(GLOB_RECURSE test_knl "tests/${test}/*.cu")
        LIST(LENGTH test_knl num_knl)
        IF(${num_knl} GREATER 0)
            cuda_add_executable(${test} ${test_srcs} ${test_knl})
        else()
            add_executable(${test} ${test_srcs})
        endif()
        TARGET_LINK_LIBRARIES(${test} MetaObject Aquila MetaParameters ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
        ADD_DEPENDENCIES(${test} MetaObject Aquila)
        set_target_properties(${test} PROPERTIES FOLDER Tests/Aquila)
        add_test(NAME ${test} COMMAND $<TARGET_FILE:${test}>)
        if(MSVC)
          CONFIGURE_FILE("tests/Test.vcxproj.user.in" ${CMAKE_BINARY_DIR}/Aquila/${test}.vcxproj.user @ONLY)
        endif()
    endforeach()
endif()
message(STATUS
    "-- Aquila config\n"
    "  MetaObject\t\t${MetaObject_VERSION}\n"
    "  MxNet\t\t${MXNET_FOUND}\n"
    "  Eigen3\t\t${EIGEN_MAJOR_VERSION}.${EIGEN_MINOR_VERSION}\n"
    "  Tests\t\t${Boost_UNIT_TEST_FRAMEWORK_FOUND}"
)
