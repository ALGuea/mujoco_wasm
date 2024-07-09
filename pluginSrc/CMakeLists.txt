cmake_minimum_required(VERSION 3.8)
project(PidPlugin)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
# Define source files for the static library
set(SOURCE_FILES_ACTUATOR ${CMAKE_SOURCE_DIR}/src/Pid.cpp)
set(SOURCE_FILES_PLUGIN plugin.cpp)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src)

# Add compile options
add_compile_options(-pthread)

# Linker flags for the shared library (plugin)
set(EMCC_LINKER_FLAGS "-s SIDE_MODULE=1 -s EXPORT_ALL=1 -s ERROR_ON_UNDEFINED_SYMBOLS=1")

# Add the static TactileSensor library
add_library(Pid STATIC ${SOURCE_FILES_ACTUATOR})
target_include_directories(
  Pid PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)
target_include_directories(Pid PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_libraries(Pid PUBLIC ${LIB_MUJOCO})

# Add the shared TactileSensorPlugin library
add_library(PidPlugin SHARED ${SOURCE_FILES_PLUGIN})
set_target_properties(PidPlugin PROPERTIES LINK_FLAGS "${EMCC_LINKER_FLAGS}")
target_include_directories(PidPlugin PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_libraries(PidPlugin PUBLIC Pid)

# Install the libraries
install(
  TARGETS Pid PidPlugin
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
        DESTINATION "${INCLUDE_INSTALL_DIR}")
