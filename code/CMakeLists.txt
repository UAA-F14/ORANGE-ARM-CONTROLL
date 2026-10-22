cmake_minimum_required(VERSION 3.16)
project(EX2BrazoPS4 C)

add_executable(EX2BrazoPS4
    main.c
    src/gpio.c
    src/pwm.c
    src/ps4.c
)
target_include_directories(EX2BrazoPS4
    PRIVATE
    inc
)
target_link_libraries(EX2BrazoPS4
    gpiod
)
