cmake_minimum_required(VERSION 3.22)

project(testNode LANGUAGES CXX)

set(I2W_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../i2w"
    CACHE PATH
    "Path to the i2w source tree")
set(CRAWLER_I2W_MSGS_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../crawler-i2w-msgs"
    CACHE PATH
    "Path to the crawler i2w message package")

if(NOT EXISTS "${I2W_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "i2w source not found at '${I2W_SOURCE_DIR}'")
endif()
if(NOT EXISTS "${CRAWLER_I2W_MSGS_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "crawler-i2w-msgs source not found at '${CRAWLER_I2W_MSGS_SOURCE_DIR}'")
endif()

add_subdirectory("${I2W_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/i2w")
add_subdirectory("${CRAWLER_I2W_MSGS_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/crawler_i2w_msgs")

add_executable(joy_sub_node
src/joyToCmd_vel.cpp
)
target_compile_features(joy_sub_node PRIVATE cxx_std_20)
target_include_directories(joy_sub_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(joy_sub_node PRIVATE
  i2w::i2w
  crawler_i2w_msgs::crawler_i2w_msgs
)

install(TARGETS joy_sub_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

  add_executable(joy_pub_node
src/joy.cpp
)
target_compile_features(joy_pub_node PRIVATE cxx_std_20)
target_include_directories(joy_pub_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(joy_pub_node PRIVATE
  i2w::i2w
  crawler_i2w_msgs::crawler_i2w_msgs
)

install(TARGETS joy_pub_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})


# Go to Gole 
  add_executable(gotoGole_node
src/GotoGole.cpp
)
target_compile_features(gotoGole_node PRIVATE cxx_std_20)
target_include_directories(gotoGole_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(gotoGole_node PRIVATE
  i2w::i2w
  crawler_i2w_msgs::crawler_i2w_msgs
)

install(TARGETS gotoGole_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})


  # Chmnel Switching Client 
  add_executable(channelSwitchingClient_node
src/chnnelSwitchingClient.cpp
)
target_compile_features(channelSwitchingClient_node PRIVATE cxx_std_20)
target_include_directories(channelSwitchingClient_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(channelSwitchingClient_node PRIVATE
  i2w::i2w
  crawler_i2w_msgs::crawler_i2w_msgs
)

install(TARGETS channelSwitchingClient_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

