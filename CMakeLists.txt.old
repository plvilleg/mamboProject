cmake_minimum_required(VERSION 2.6)
project(LogSubmarine)
#The version number.
set(LogSubmarine_VERSION_MAJOR 1)
set(LogSubmarine_VERSION_MINOR 0)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

if(NOT CMAKE_VERSION VERSION_LESS 3.1)
	set(CMAKE_CXX_STANDARD 11)
else()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -pthread -g -O0 -fprofile-arcs -ftest-coverage -Wno-strict-prototypes")

#add the executable
add_executable(TestDestination destinationPoint.cpp)

#libSerialPort
find_package(PkgConfig)
pkg_check_modules(LIBSERIALPORT libserialport)
link_directories(${LIBSERIALPORT_LIBRARY_DIRS})
include_directories(${LIBSERIALPORT_INCLUDE_DIRS})

find_library(M_LIB m)

#wiringPi
link_directories("/usr/local/lib/")
include_directories("/usr/local/lib/")

#SBP
include_directories("/usr/local/include/libsbp")

#armadillo
link_directories("/home/pi/armadillo-8.600.0")
include_directories("/home/pi/armadillo-8.600.0")

#ADXL345
link_directories("/home/pi/submarino/")
include_directories("/home/pi/submarino/lib/")
add_library(ADXL345 /home/pi/submarino/lib/ADXL345.cpp)

#HMC5843
link_directories("/home/pi/libsbp/c/submarino/")
include_directories("/home/pi/submarino/lib/")
add_library(HMC5843 /home/pi/submarino/lib/hmc5843.cpp)

#COMPASS
link_directories("/home/pi/libsbp/c/submarino/")
include_directories("/home/pi/submarino/lib/")
add_library(COMPASS /home/pi/submarino/lib/compass.cpp)

#ITG3205
link_directories("/home/pi/libsbp/c/submarino/")
include_directories("/home/pi/libsbp/c/submarino/lib/")
add_library(ITG3205 /home/pi/submarino/lib/ITG3205.cpp)

target_link_libraries(TestDestination armadillo wiringPi COMPASS ADXL345 HMC5843 ITG3205 serialport sbp ${M_LIB})
