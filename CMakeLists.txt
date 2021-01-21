cmake_minimum_required(VERSION 3.1.0)

project(bullet_plugin)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(BULLET_PLUGIN_COMPILER_FLAGS "")
set(BULLET_PLUGIN_DEFINITIONS "")
if(MSVC)
  list(APPEND BULLET_PLUGIN_COMPILER_FLAGS /EHsc /MP)
  if(CMAKE_BUILD_TYPE MATCHES DEBUG)
    list(APPEND BULLET_PLUGIN_COMPILER_FLAGS /EHsc /MP)
  endif()
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug OR CMAKE_BUILD_TYPE MATCHES RelWithDebInfo OR ICEENGINE_ENABLE_DEBUG_LOGGING)
  list(APPEND BULLET_PLUGIN_DEFINITIONS -DICEENGINE_ENABLE_DEBUG_LOGGING)
endif()

if(ICEENGINE_ENABLE_TRACE_LOGGING)
  list(APPEND BULLET_PLUGIN_DEFINITIONS -DICEENGINE_ENABLE_TRACE_LOGGING)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_CPU_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_OPENGL3_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_EXTRAS OFF CACHE BOOL "" FORCE)
set(BUILD_BULLET3 OFF CACHE BOOL "" FORCE)
set(BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(INSTALL_CMAKE_FILES OFF CACHE BOOL "" FORCE)
set(INSTALL_LIBS OFF CACHE BOOL "" FORCE)

# disable floating-point optimizations
set(USE_MSVC_FAST_FLOATINGPOINT OFF CACHE BOOL "" FORCE)
set(USE_MSVC_SSE OFF CACHE BOOL "" FORCE)
set(USE_MSVC_SSE2 OFF CACHE BOOL "" FORCE)

set(USE_MSVC_STRING_POOLING OFF CACHE BOOL "" FORCE)
set(USE_MSVC_FUNCTION_LEVEL_LINKING OFF CACHE BOOL "" FORCE)
set(USE_MSVC_COMDAT_FOLDING OFF CACHE BOOL "" FORCE)
set(USE_MSVC_DISABLE_RTTI OFF CACHE BOOL "" FORCE)
set(CMAKE_CL_64 ON CACHE BOOL "" FORCE)

#if(NOT MSVC)
#  set(BUILD_SHARED_LIBS ON CACHE BOOL "Shared Libs" FORCE)
#endif()
set(USE_MSVC_RUNTIME_LIBRARY_DLL ON CACHE BOOL "" FORCE)
set(CMAKE_DEBUG_POSTFIX "" CACHE STRING "Adds a postfix for debug-built libraries." FORCE)
set(CMAKE_MINSIZEREL_POSTFIX "" CACHE STRING "Adds a postfix for MinsizeRelease-built libraries." FORCE)
set(CMAKE_RELWITHDEBINFO_POSTFIX "" CACHE STRING "Adds a postfix for ReleaseWithDebug-built libraries." FORCE)
set(LIB_DESTINATION "" CACHE STRING "Library directory name")
set(LIBRARY_OUTPUT_PATH "" CACHE PATH "Single output directory for building all libraries." FORCE)

add_subdirectory(./bullet/bullet3)

unset(LIBRARY_OUTPUT_PATH CACHE)
unset(LIBRARY_OUTPUT_PATH)
unset(LIB_DESTINATION CACHE)
unset(LIB_DESTINATION)
unset(BUILD_SHARED_LIBS CACHE)
unset(BUILD_SHARED_LIBS)

find_package(glm REQUIRED)
find_package(Boost REQUIRED)

# Source
file(GLOB_RECURSE SOURCES "src/*.cpp")
list(APPEND SOURCES "${ICEENGINE_BASE_DIR}/src/exceptions/Exception.cpp")
list(APPEND SOURCES "${ICEENGINE_BASE_DIR}/src/exceptions/Stacktrace.cpp")

add_library(bullet_plugin SHARED ${SOURCES})

# Headers
target_include_directories(bullet_plugin PRIVATE include)

target_include_directories(bullet_plugin PRIVATE ${ICEENGINE_INCLUDE_DIRS})
target_include_directories(bullet_plugin PRIVATE ${Boost_INCLUDE_DIRS})
target_include_directories(bullet_plugin PRIVATE ./bullet/bullet3/src)

target_compile_definitions(bullet_plugin PRIVATE ${BULLET_PLUGIN_DEFINITIONS})
target_compile_options(bullet_plugin PRIVATE ${BULLET_PLUGIN_COMPILER_FLAGS})

target_link_libraries(bullet_plugin PRIVATE glm::glm)
target_link_libraries(bullet_plugin PRIVATE Bullet3Common)
target_link_libraries(bullet_plugin PRIVATE BulletDynamics)
target_link_libraries(bullet_plugin PRIVATE BulletCollision)
target_link_libraries(bullet_plugin PRIVATE LinearMath)