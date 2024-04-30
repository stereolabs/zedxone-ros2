cmake_minimum_required(VERSION 3.5)
project(zedxone_components)

################################################
## Generate symbols for IDE indexer (VSCode)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

################################################
# Check the ROS2 version

set(ROS2_FOUND FALSE)
if(DEFINED ENV{ROS_DISTRO})
    set(FOUND_ROS2_DISTRO $ENV{ROS_DISTRO})
    set(ROS2_FOUND TRUE)
    #message("* Found ROS2 ${FOUND_ROS2_DISTRO}")
else()
    message("* ROS2 distro variable not set. Trying to figure it out...")
    set(ROS2_DISTROS "ardent;crystal;dashing;eloquent;foxy;galactic;humble;rolling")
    set(ROS2_FOUND FALSE)
    foreach(distro ${ROS2_DISTROS})
        if(NOT ROS2_FOUND)
            find_path(RCLCPP_H rclcpp.hpp PATHS /opt/ros/${distro}/include/rclcpp)
            if(RCLCPP_H)
                #message("* Found ROS2 ${distro}")
                set(FOUND_ROS2_DISTRO ${distro})
                set(ROS2_FOUND TRUE)
            endif()
        endif()
    endforeach()
endif()

if(ROS2_FOUND)
    if(${FOUND_ROS2_DISTRO} STREQUAL "foxy")
        #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
        add_definitions(-DFOUND_FOXY)
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "iron")
        #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
	add_definitions(-DFOUND_IRON)
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "humble")
        #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
        add_definitions(-DFOUND_HUMBLE)
    else()
        message("*** WARNING *** ROS2 ${FOUND_ROS2_DISTRO} is not officially supported by the package '${PROJECT_NAME}'. Correct operation is not guaranteed.")
    endif()
else()
    message("*** WARNING *** ROS2 distro is unknown. This package could not work correctly.")
endif()
################################################

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE MATCHES Release)
    #message(" * Release Mode")
    add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    #message(" * Release with Debug Info Mode")
    add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
    message(" * Debug Mode")
endif()

#############################################
# Dependencies

set(DEPENDENCIES
    rclcpp
    rclcpp_components
    image_transport
    builtin_interfaces
    std_msgs    
    sensor_msgs
    std_srvs
    diagnostic_msgs
    diagnostic_updater
    ${TM_DEP}
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

###############################################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
file(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})

###############################################################################
# LIBS


# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

###############################################################################
# SOURCES

# "EXCLUDE_FROM_ALL" is used to ignore the install command of the "arguscapture" lib
add_subdirectory(libs/zedx-one-capture/lib EXCLUDE_FROM_ALL)

set(ZEDXONE_CAMERA_INC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/visibility_control.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zedxone_camera/include/sl_types.hpp   
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zedxone_camera/include/zedxone_camera_component.hpp
)

set(ZEDXONE_CAMERA_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zedxone_camera/src/zedxone_camera_component.cpp
)

###############################################################################
# Bin and Install

add_library(zedxone_camera_component SHARED
    ${ZEDXONE_CAMERA_INC}
    ${ZEDXONE_CAMERA_SRC}
)
target_include_directories(zedxone_camera_component PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zedxone_camera/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/zedx-one-capture/lib/include
)
target_compile_definitions(zedxone_camera_component
    PRIVATE "COMPOSITION_BUILDING_DLL"
)

add_dependencies (zedxone_camera_component 
    arguscapture
)

target_link_libraries(zedxone_camera_component
    arguscapture
)
ament_target_dependencies(zedxone_camera_component
    ${DEPENDENCIES}
)

rclcpp_components_register_nodes(zedxone_camera_component "stereolabs::ZedXOneCamera")
set(node_plugins "${node_plugins}stereolabs::ZedXOneCamera;$<TARGET_FILE:zedxone_camera_component>\n")

# Install components
install(TARGETS zedxone_camera_component arguscapture
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zedxone_camera/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/
    DESTINATION include/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(zedxone_camera_component)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
