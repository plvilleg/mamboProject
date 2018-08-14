cmake_minimum_required(VERSION 2.6)
project(TestADXL345)
#The version number.
set(TestADXL345_VERSION_MAJOR 1)
set(TestADXL345_VERSION_MINOR 0)

#set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(CMAKE_C_FLAGS "-Wall -Wextra -Wno-strict-prototypes -Wno-unknown-warning-option -std=gnu99 ${CMAKE_C_FLAGS}")

#add the executable
add_executable(TestADXL345 testADXL345.cpp)

#wiringPi
link_directories("/usr/local/lib/")
include_directories("/usr/local/lib/")


#ADXL345
link_directories("/home/pi/submarino/test")
include_directories("/home/pi/submarino/test/libs/")
add_library(ADXL345 /home/pi/submarino/test/libs/ADXL345.cpp)

#HMC5883L
link_directories("/home/pi/submarino/")
include_directories("/home/pi/submarino/test/libs/")
add_library(HMC5883L /home/pi/submarino/test/libs/hmc5883l.cpp)

target_link_libraries(TestADXL345 wiringPi ADXL345)
