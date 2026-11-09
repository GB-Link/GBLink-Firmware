zephyr_library_sources(usbLayer.cpp)
zephyr_library_sources(serialLayer.cpp)
zephyr_library_sources(transport.cpp)

add_subdirectory(link)
add_subdirectory(packet)