# add_compile_definitions(SIMULATOR)

add_subdirectory(deb)
add_subdirectory(rpm)
add_subdirectory(windows)

include(cpack_config.cmake)
