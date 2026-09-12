cmake_minimum_required(VERSION 3.22)

project(testNode LANGUAGES CXX)

# ============================================================
# Dependencies
# ============================================================

set(I2W_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../i2w"
    CACHE PATH
    "Path to the i2w source tree")

set(CRAWLER_I2W_MSGS_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../crawler-i2w-msgs"
    CACHE PATH
    "Path to the crawler i2w message package")


if(NOT EXISTS "${I2W_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
        "i2w source not found at '${I2W_SOURCE_DIR}'")
endif()

if(NOT EXISTS "${CRAWLER_I2W_MSGS_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
        "crawler-i2w-msgs source not found at "
        "'${CRAWLER_I2W_MSGS_SOURCE_DIR}'")
endif()


add_subdirectory(
    "${I2W_SOURCE_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/i2w"
)

add_subdirectory(
    "${CRAWLER_I2W_MSGS_SOURCE_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/crawler_i2w_msgs"
)


# ============================================================
# Common function for test nodes
# ============================================================

function(add_test_node TARGET_NAME SOURCE_FILE)

    add_executable(${TARGET_NAME}
        ${SOURCE_FILE}
    )

    target_compile_features(${TARGET_NAME}
        PRIVATE
        cxx_std_20
    )

    target_include_directories(${TARGET_NAME}
        PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
    )

    target_link_libraries(${TARGET_NAME}
        PRIVATE
        i2w::i2w
        crawler_i2w_msgs::crawler_i2w_msgs
    )

    install(TARGETS ${TARGET_NAME}
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )

    target_compile_definitions(${TARGET_NAME}
    PRIVATE
    CONFIG_DIR="${CMAKE_SOURCE_DIR}/config"
)


endfunction()


# ============================================================
# Executables
# ============================================================

# ------------------------------------------------------------
# Joystick
# ------------------------------------------------------------

add_test_node(
    joy_sub_node
    src/joyToCmd_vel.cpp
)

add_test_node(
    joy_pub_node
    src/joy.cpp
)



# ------------------------------------------------------------
# Go To Goal
# ------------------------------------------------------------

add_test_node(
    gotoGole_node
    src/GotoGole.cpp
)


# ------------------------------------------------------------
# Channel Switching
# ------------------------------------------------------------

add_test_node(
    channelSwitchingClient_node
    src/chnnelSwitchingClient.cpp
)


# ------------------------------------------------------------
# UI Robot Connection Check
# ------------------------------------------------------------

add_test_node(
    uiRobotConnectionCheckClient_node
    src/uiRobotConnectionCheckClient.cpp
)

target_compile_definitions(uiRobotConnectionCheckClient_node
    PRIVATE
    TRIDRISHTI_TESTNODE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)


add_test_node(
    uiRobotConnectionCheckServer_node
    src/uiRobotConnectionCheckServer.cpp
)


# ------------------------------------------------------------
# Motor State
# ------------------------------------------------------------

add_test_node(
    randomMotorState_node
    src/randomMotorState.cpp
)


# ------------------------------------------------------------
# Joy Network -> Local
# ------------------------------------------------------------

add_test_node(
    joy_network_to_local_node
    src/joy_network_to_local.cpp
)


# ------------------------------------------------------------
# Random RCS Publisher
# ------------------------------------------------------------

add_test_node(
    randomPubRcs_node
    src/randomPubRcs.cpp
)


# ------------------------------------------------------------
# RCS Motor State Subscriber
# ------------------------------------------------------------

add_test_node(
    subscriberRCSMotorState_node
    src/subscriberRCSMotorState.cpp
)


# ------------------------------------------------------------
# UI Robot Connection Check Server
# Network -> Local
# ------------------------------------------------------------

add_test_node(
    uiRobotConnectionCheckServer_network_to_local_node
    src/uiRobotConnectionCheckServer_network_to_local.cpp
)