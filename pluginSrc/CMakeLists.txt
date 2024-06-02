cmake_minimum_required(VERSION 3.2)
project(TactileSensorPlugin)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)

# Define source files for the static library
set(SOURCE_FILES_SENSOR ${CMAKE_SOURCE_DIR}/src/TactileSensor.cpp)
set(SOURCE_FILES_PLUGIN plugin.cpp)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src)

# Add compile options
add_compile_options(-pthread)

# Linker flags for the shared library (plugin)
set(EMCC_LINKER_FLAGS "-s SIDE_MODULE=1 -s EXPORT_ALL=1 -s ERROR_ON_UNDEFINED_SYMBOLS=0")

# Add the static TactileSensor library
add_library(TactileSensor STATIC ${SOURCE_FILES_SENSOR})
target_include_directories(
  TactileSensor PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)
target_include_directories(TactileSensor PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_libraries(TactileSensor PUBLIC ${LIB_MUJOCO})

# Add the shared TactileSensorPlugin library
add_library(TactileSensorPlugin SHARED ${SOURCE_FILES_PLUGIN})
set_target_properties(TactileSensorPlugin PROPERTIES LINK_FLAGS "${EMCC_LINKER_FLAGS}")
target_include_directories(TactileSensorPlugin PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_libraries(TactileSensorPlugin PUBLIC TactileSensor)

# Install the libraries
install(
  TARGETS TactileSensor TactileSensorPlugin
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
        DESTINATION "${INCLUDE_INSTALL_DIR}")
