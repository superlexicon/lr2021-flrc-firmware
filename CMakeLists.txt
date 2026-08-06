# ----------------------------------------------------------------------------
# LR2021 USB <-> FLRC bridge firmware - Zephyr application
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(lr2021_flrc_firmware LANGUAGES C)

if(NOT DEFINED ZEPHYR_USP_MODULE_DIR)
    message(FATAL_ERROR
        "ZEPHYR_USP_MODULE_DIR is not defined. The Semtech USP module is missing.")
endif()

zephyr_include_directories(
    ${ZEPHYR_USP_ZEPHYR_CMAKE_DIR}/include/zephyr/usp
)

target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/flrc_burst_protocol.c
)
