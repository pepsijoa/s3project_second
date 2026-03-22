cmake_minimum_required(VERSION 3.16)
project(rpi_uart_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 경고 활성화
add_compile_options(-Wall -Wextra -Wpedantic)

add_executable(uart_test
    src/main.cpp
    src/uart_manager.cpp
    src/protocol.cpp
)

target_include_directories(uart_test PRIVATE src)

# timerfd / epoll 는 리눅스 glibc 에 포함 — 별도 링크 불필요
# target_link_libraries(uart_test pthread) ← 사용 금지 (CLAUDE.md §3)

install(TARGETS uart_test DESTINATION bin)
