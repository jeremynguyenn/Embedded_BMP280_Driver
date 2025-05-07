cmake_minimum_required(VERSION 3.10)

project(main)

add_executable(main main.c GY_86.c GY_86.h)

target_link_libraries(main PRIVATE m wiringPi)
