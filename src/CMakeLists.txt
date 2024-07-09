cmake_minimum_required(VERSION 3.8)
project(mujoco_wasm)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)

# Define source files for the main executable
set(SOURCE_FILES main.genned.cc)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/pluginSrc)

# Add compile options
add_compile_options(-pthread)

# Linker flags for the main module
set(EMCC_LINKER_FLAGS "-s USE_WEBGPU=1 -s MAIN_MODULE=1 -s ASSERTIONS=1 --bind -s ALLOW_MEMORY_GROWTH=1 -s EXPORT_ES6=1 -s MODULARIZE=1 -s FORCE_FILESYSTEM=1 -s EXPORTED_RUNTIME_METHODS=['FS','MEMFS','loadDynamicLibrary','addFunction','removeFunction','cwrap'] -s EXPORT_NAME='load_mujoco' -s EXCEPTION_CATCHING_ALLOWED=['load_from_xml'] -s ERROR_ON_UNDEFINED_SYMBOLS=0 -s ALLOW_UNIMPLEMENTED_SYSCALLS=1 -s DYNAMIC_EXECUTION=1")

# Set linker flags for the main module
set(CMAKE_REQUIRED_FLAGS "${EMCC_LINKER_FLAGS}")

# Add the main executable
add_executable(mujoco_wasm ${SOURCE_FILES})
set_target_properties(mujoco_wasm PROPERTIES LINK_FLAGS "${EMCC_LINKER_FLAGS}")

# Link libraries for the main module, including the static myCustomSensor library
target_link_libraries(mujoco_wasm ccd elasticity lodepng mujoco tinyxml2 qhullstatic_r PidPlugin)

# Install the main module
install(TARGETS mujoco_wasm DESTINATION ${CMAKE_INSTALL_PREFIX}/dist)
