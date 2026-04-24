# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Thêm component path ngoài
# set(EXTRA_COMPONENT_DIRS my_component)
list(APPEND SDKCONFIG_DEFAULTS
                "${CMAKE_SOURCE_DIR}/sdkconfig.nimbledf"
                # "${CMAKE_SOURCE_DIR}/sdkconfig.mesh_defaults"
                )
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(test_component)
