cmake_minimum_required(VERSION 2.6)
project(LogSubmarine)
#The version number.
set(LogSubmarine_VERSION_MAJOR 1)
set(LogSubmarine_VERSION_MINOR 0)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

#if(NOT CMAKE_VERSION VERSION_LESS 3.1)
#	set(CMAKE_CXX_STANDARD 11)
#else()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu99")   #-std=c++11")
#endif()


set(GCC_ARMADILLO_FLAGS "-O2  -I /home/pi/armadillo-8.600.0/include/ -DARMA_DONT_USE_WRAPPER -lblas -llapack -lgfortran")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_ARMADILLO_FLAGS} -Wall -Wextra -pedantic -pthread -g -O0 -fprofile-arcs -ftest-coverage")


#add the executable
add_executable(TestGPS_point GPS_point.cpp)


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
link_directories("/usr/local/lib/")
include_directories("/usr/local/include/")


#armadillo  
find_package(Armadillo)
include_directories(${ARMADILLO_INCLUDE-DIRS})


#ADXL345
link_directories("/home/pi/mamboProject/")
include_directories("/home/pi/mamboProject/lib/")
add_library(ADXL345 /home/pi/mamboProject/lib/ADXL345.cpp)


#HMC5843
link_directories("/home/pi/mamboProject/")
include_directories("/home/pi/mamboProject/lib/")
add_library(HMC5843 /home/pi/mamboProject/lib/HMC5843.cpp)


#DISTANCE g++ example1.cpp -o example1 -O2 -larmadillo -lgfortran  LIB_FLAGS = -larmadillo -lgfortran
include_directories("/home/pi/mamboProject/lib/")
add_library(RELATIVE_DISTANCE /home/pi/mamboProject/lib/RELATIVE_DISTANCE.cpp)


#COMPASS
link_directories("/home/pi/mamboProject/")
include_directories("/home/pi/mamboProject/lib/")
add_library(COMPASS /home/pi/mamboProject/lib/COMPASS.cpp)


#ITG3205
#link_directories("/home/pi/libsbp/c/submarino/")
#include_directories("/home/pi/libsbp/c/submarino/lib/")
#add_library(ITG3205 /home/pi/submarino/lib/ITG3205.cpp)


target_link_libraries(TestGPS_point wiringPi COMPASS ADXL345 HMC5843 RELATIVE_DISTANCE ${ARMADILLO_LIBRARY} serialport sbp ${M_LIB})
