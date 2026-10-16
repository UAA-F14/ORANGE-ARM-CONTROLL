cmake_minimum_required(VERSION 3.16)
project(P8PS4 C)

add_executable(P8PS4
    src/ps4.c
    src/main.c
)
target_include_directories(P8PS4
    PRIVATE
    include
)
