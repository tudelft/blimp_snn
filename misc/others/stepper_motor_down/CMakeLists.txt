cmake_minimum_required(VERSION 2.6)

include_directories("include")

add_library(StepperMotor "src/StepperMotor.cpp")

add_executable(Test_1 "test/Test_1.cpp")
add_executable(Test_2 "test/Test_2.cpp")
add_executable(Test_3 "test/Test_3.cpp")

target_link_libraries(Test_1 StepperMotor wiringPi)
target_link_libraries(Test_2 StepperMotor wiringPi)
target_link_libraries(Test_3 StepperMotor wiringPi pthread)
