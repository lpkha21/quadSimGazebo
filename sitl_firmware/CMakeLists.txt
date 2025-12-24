cmake_minimum_required(VERSION 3.10)

project(sitl_firmware LANGUAGES C CXX)

# --------------------------------------------------
# Language standards
# --------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

# --------------------------------------------------
# Gazebo Transport (via pkg-config)
# --------------------------------------------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(GZ_TRANSPORT REQUIRED gz-transport)

# --------------------------------------------------
# Sources
# --------------------------------------------------
set(SOURCES
    main.cpp
    motor_sender.cpp
    rc.cpp
    pid.cpp
    imu.cpp
)

set(HEADERS
    motor_sender.hh
    rc.hh
    pid.hh
    imu.hh
)

# --------------------------------------------------
# Executable
# --------------------------------------------------
add_executable(sitl_firmware
    ${SOURCES}
    ${HEADERS}
)

# --------------------------------------------------
# Include paths
# --------------------------------------------------
target_include_directories(sitl_firmware
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${GZ_TRANSPORT_INCLUDE_DIRS}
)

# --------------------------------------------------
# Link libraries
# --------------------------------------------------
target_link_libraries(sitl_firmware
    PRIVATE
        ${GZ_TRANSPORT_LIBRARIES}
        pthread
        m
)

# --------------------------------------------------
# Extra compile flags from Gazebo
# --------------------------------------------------
target_compile_options(sitl_firmware
    PRIVATE
        ${GZ_TRANSPORT_CFLAGS_OTHER}
)
