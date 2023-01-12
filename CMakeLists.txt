# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

add_compile_options(-fdiagnostics-color=always)
add_compile_options(-Wno-unused-variable)
add_compile_options(-Wno-unused-function)
add_compile_options(-Wno-maybe-uninitialized)
add_compile_options(-Wunused-but-set-parameter)
add_compile_options(-Wformat=0)
add_compile_options(-mlongcalls)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(esp32s3_dc_motor_controller)
