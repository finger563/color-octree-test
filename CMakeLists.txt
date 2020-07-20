cmake_minimum_required(VERSION 2.8.3)
project(color-octree-test)

add_compile_options(-std=c++14)

SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-error ")
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-error ")
SET (CMAKE_CXX_FLAGS_RELEASE "-O3 -funroll-loops -DNDEBUG")
SET (CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
# Shared object compilation under 64bit (vtable)
ADD_DEFINITIONS(-fPIC)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fconcepts")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# Octomap
find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

# Set OS-specific things here
if(WIN32)
elseif(CYGWIN)
elseif(APPLE)
elseif(UNIX)
  add_definitions(-DUNIX)
  add_compile_options(-Wno-deprecated-declarations)
  add_compile_options(-Wno-unused-result)
  add_definitions(-O3)
endif(WIN32)

find_package(OpenMP)
list(APPEND OpenMP_LIBRARIES OpenMP::OpenMP_CXX)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${OpenMP_LIBRARIES}
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ThirdParty/json/single_include
  ${OCTOMAP_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/ColorOcTree.cpp
  src/MapServer.cpp
  )
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})

add_executable(test_mapserver_insertion_performance
  src/test_mapserver_insertion_performance.cpp)
target_link_libraries(test_mapserver_insertion_performance
  ${PROJECT_NAME}
  ${LINK_LIBS})
