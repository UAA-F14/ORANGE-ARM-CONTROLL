cmake_minimum_required(VERSION 3.16)
project(EX2BrazoPS4 C)

add_executable(EX2BrazoPS4
    src/main.c
    ../P6PWMLIBRARY/src/pwm.c
    ../P8PS4/src/ps4.c
)
target_include_directories(EX2BrazoPS4
    PRIVATE
    ../P6PWMLIBRARY/include
    ../P8PS4/include
)
