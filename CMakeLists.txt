cmake_minimum_required(VERSION 3.28.3)
project(ropod_sim)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED COMPONENTS profiler)
find_package(gz-sim8 REQUIRED)

# =========
# LIBRARIES
# =========
add_library(HolonomicControl SHARED
    src/HolonomicControl.cpp
)
target_include_directories(HolonomicControl PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(HolonomicControl PRIVATE
    gz-sim8::gz-sim8
)

# =======
# INSTALL
# =======
install(
    DIRECTORY
        launch
        config
        scripts
        description
        world_description
    DESTINATION
        share/${PROJECT_NAME}
)
install(
    TARGETS
        HolonomicControl
    DESTINATION
        lib/${PROJECT_NAME}
)

ament_package()
