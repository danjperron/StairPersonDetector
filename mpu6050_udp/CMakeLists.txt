set(PROJECT mpu6050_udp)
cmake_minimum_required(VERSION 3.12)
set(PICO_BOARD pico_w)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
add_executable(${PROJECT} ${PROJECT}.c)
add_library(kiss_fftr kiss_fftr.c)
add_library(kiss_fft kiss_fft.c)


project(${PROJECT} C CXX)
pico_sdk_init()

target_sources(${PROJECT} PRIVATE
        ${PROJECT}.c
        )

target_sources(${PROJECT} PRIVATE
        ${PROJECT}.c
        )
target_compile_definitions(mpu6050_udp PRIVATE
            WIFI_SSID=\"${WIFI_SSID}\"
            WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
            )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(${PROJECT}  pico_stdlib hardware_i2c kiss_fftr kiss_fft)
target_link_libraries(kiss_fftr kiss_fft)


target_include_directories(${PROJECT} PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(${PROJECT}
     pico_cyw43_arch_lwip_threadsafe_background
     kiss_fftr
     pico_stdlib
     pico_multicore
)

pico_add_extra_outputs(${PROJECT})
pico_enable_stdio_usb(${PROJECT} 1)
pico_enable_stdio_uart(${PROJECT} 0)


