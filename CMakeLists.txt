cmake_minimum_required(VERSION 3.10)
project(FR3_visual_servo_examples CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find your installed ViSP (from source or packages)
find_package(VISP REQUIRED)

include_directories(${VISP_INCLUDE_DIRS})

# CHRPS executable is optional (source may be removed). Guard its creation.
if(EXISTS "${CMAKE_SOURCE_DIR}/src/servoFrankaIBVS_CHRPS.cpp")
  add_executable(servoFrankaIBVS_CHRPS src/servoFrankaIBVS_CHRPS.cpp)
  target_link_libraries(servoFrankaIBVS_CHRPS ${VISP_LIBRARIES})
else()
  message(STATUS "Skipping servoFrankaIBVS_CHRPS target: src/servoFrankaIBVS_CHRPS.cpp not found")
endif()

add_executable(servoFrankaIBVS_combined src/servoFrankaIBVS_combined.cpp)
target_link_libraries(servoFrankaIBVS_combined ${VISP_LIBRARIES})
