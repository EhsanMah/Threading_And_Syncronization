cmake_minimum_required(VERSION 2.8.3)
project(a3_skeleton)

# Importing the necessary external packages
find_package( Threads REQUIRED )
find_package( OpenCV REQUIRED )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

set(SIMULATOR_LIB ${CMAKE_SOURCE_DIR}/dep/lib/)
set(SIMULATOR_INCLUDE ${CMAKE_SOURCE_DIR}/dep/include)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${SIMULATOR_INCLUDE}
)

message ( STATUS "Looking for libray in ${SIMULATOR_LIB}" )
link_directories(${SIMULATOR_LIB})

###################################################################################################


# Here we declare a C++ executable with only one cpp forming part of it, you will need to change to cater for more files.
add_executable(${PROJECT_NAME}_example main.cpp Surveillance.cpp DetectBogies.cpp BogieChaser.cpp)

# Link the library
target_link_libraries(${PROJECT_NAME}_example ${OpenCV_LIBS} simulator)



#####################################
# Link to gtests
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

if(NOT GTEST_LIBRARY)
  message("GTest library not found")
endif()
###################################

if(GTEST_LIBRARY)
    enable_testing()

    #For each test (cpp file you have), you will need to repeat below    

    #1) Add the unit test executable against the source code of the test
    add_executable(dfsTest test/utest_dfs.cpp)
    #2) Linking to gtest  
    target_link_libraries(dfsTest ${GTEST_LIBRARIES} pthread)
    #3) Link the the libraries
    target_link_libraries(dfsTest ${OpenCV_LIBS} simulator )

    #You can run the test individually by name (in this example it is ./dfsTest)
    #OR (make test) will run them as well
    add_test(dfsUnitTest dfsTest)
endif()

