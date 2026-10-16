cmake_minimum_required(VERSION 3.16)
project(P6PWMLIBRARY C)

add_executable(P6PWMLIBRARY
    src/gpio.c
    src/pwm.c
    src/main.c
)
target_include_directories(P6PWMLIBRARY
    PRIVATE
    include
)
target_link_libraries(P6PWMLIBRARY
    gpiod
)
