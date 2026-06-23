cmake_minimum_required(VERSION 3.1.2)
project(bridgex)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE "Release")

# 定义PTHREAD_STACK_MIN宏，解决Boost库编译错误
add_definitions(-DPTHREAD_STACK_MIN=16384)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  move_base_msgs
  actionlib
  base_switch_map
  tf
  message_generation
)
add_subdirectory(src/thirdparty/log)
# 查找ZMQ库
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
message(STATUS "Found libzmq version: ${ZMQ_VERSION}")

# 解析ZMQ版本并定义编译时宏
if(ZMQ_VERSION)
    # 解析版本号 (例如 "4.3.4" -> 4, 3, 4)
    string(REPLACE "." ";" VERSION_LIST ${ZMQ_VERSION})
    list(GET VERSION_LIST 0 ZMQ_MAJOR_VERSION)
    list(GET VERSION_LIST 1 ZMQ_MINOR_VERSION)
    list(GET VERSION_LIST 2 ZMQ_PATCH_VERSION)
    
    # 根据版本设置使用新旧API的宏
    if(ZMQ_MAJOR_VERSION EQUAL 4 AND ZMQ_MINOR_VERSION GREATER_EQUAL 7)
        add_definitions(-DUSE_NEW_ZMQ_API=1)  # 4.7.0+ 使用旧API因为setsockopt被弃用
    else()
        add_definitions(-DUSE_NEW_ZMQ_API=0)  # 其他版本使用旧API
    endif()
    
    message(STATUS "ZMQ version: ${ZMQ_MAJOR_VERSION}.${ZMQ_MINOR_VERSION}.${ZMQ_PATCH_VERSION}")
    message(STATUS "USE_NEW_ZMQ_API: ${USE_NEW_ZMQ_API}")
endif()

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs std_srvs nav_msgs geometry_msgs move_base_msgs actionlib tf
  LIBRARIES ${PROJECT_NAME}
)
# BridgeX节点（包含ZMQ管理功能）
add_executable(${PROJECT_NAME}_node
    src/bridgex.cpp
    src/zmq/message_manager.cpp
    src/zmq/message_base.cpp
    src/message/cmd_vel_message.cpp
    src/message/imu_message.cpp
    src/message/loc_message.cpp
    src/message/amclpose_message.cpp
    src/ex/ex_client.cpp
    src/ex/file_storage.cpp
    src/ex/robot/m20_controller.cpp
    src/ex/robot/lite3_controller.cpp
    src/ex/robot/agv_controller.cpp
    src/ex/task/task_manager.cpp
    src/ex/cam/camera.cpp
    src/ex/map/bag_recording_api.cpp
    src/ex/map/mapping_api.cpp
    src/log/log.cpp
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node PUBLIC
  ${catkin_LIBRARIES}
  ${ZMQ_LIBRARIES}
  fmtlog
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
        ${catkin_INCLUDE_DIRS}
        src/zmq
        src/message
        src/thirdparty
        src/log
        src/message
        src/ex
        src/ex/task
        src/ex/map
        src/thirdparty/log/fmt/include
        src/thirdparty/log/fmt/
        src/thirdparty/log/
)
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS fmtlog
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/

  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"  # 只安装launch文件
)

install(DIRECTORY src/zmq/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/zmq
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY src/message/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/message
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY src/ex/task/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/ex/task
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY src/ex/map/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/ex/map
  FILES_MATCHING PATTERN "*.h"
)

install(FILES src/bridgex.h
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)