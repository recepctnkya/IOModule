# The following four lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Hexnet IO Module — aktif geliştirme: IOModule-v0-50 (v0-30/v0-40 donduruldu)
project(mqtt_tcp_custom_outbox VERSION "0.50.0")

# Add custom outbox implementation to mqtt component
idf_component_get_property(mqtt mqtt COMPONENT_LIB)
target_sources(${mqtt} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/main/custom_outbox.cpp)


# Our C++ needs an extra dependency to mqtt component, so we add it to mqtt component.
# This is needed because we are adding another source to the mqtt component and the build
# system needs to be aware of it to be able to compile and link the mqtt component.
# First we get our dependency
idf_component_get_property(pthread pthread COMPONENT_LIB)
# And them we link the components
target_link_libraries(${mqtt} ${pthread})

# Derleme çıktısı: vango_medium_v0_50.bin (platform katalog / OTA dosya adı)
set(HEXNET_HW_SLUG "vango_medium")
string(REPLACE "." ";" _hexnet_ver_parts "${PROJECT_VERSION}")
list(GET _hexnet_ver_parts 0 _hexnet_ver_maj)
list(GET _hexnet_ver_parts 1 _hexnet_ver_min)
set(HEXNET_RELEASE_BIN "${HEXNET_HW_SLUG}_v${_hexnet_ver_maj}_${_hexnet_ver_min}.bin")

if(TARGET ${CMAKE_PROJECT_NAME}.elf)
    # Note: Temporarily disabled due to missing binary generation step
    # add_custom_command(
    #     TARGET ${CMAKE_PROJECT_NAME}.elf
    #     POST_BUILD
    #     COMMAND ${CMAKE_COMMAND} -E copy_if_different
    #         "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin"
    #         "${CMAKE_BINARY_DIR}/${HEXNET_RELEASE_BIN}"
    #     COMMENT "Hexnet release: ${HEXNET_RELEASE_BIN}"
    # )
endif()
