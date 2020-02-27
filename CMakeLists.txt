cmake_minimum_required(VERSION 3.15)

set(PROJECT_NAME osm-fapra)

set(default_build_type "Release")

project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 20)
if( NOT CMAKE_BUILD_TYPE )
    set( CMAKE_BUILD_TYPE Release)
endif()
set(SOURCES
    src/Graph.cpp
    src/GraphBuilder.cpp
    src/main.cpp
    src/Grid.cpp
    src/Config.cpp
    src/Dijkstra.cpp
    src/CHDijkstra.cpp
    src/CHConstructor.cpp
    src/EdgeListGraph.cpp)

set(HEADERS
    include/Graph.h
    include/GraphBuilder.h
    include/Grid.h
    include/Config.h
    include/Dijkstra.h
    include/CHDijkstra.h
    include/CHConstructor.h
    include/EdgeListGraph.h)

find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(GeographicLib 1.34 REQUIRED COMPONENTS SHARED)
set (GeographicLib_USE_STATIC_LIBS ON)

include_directories(${Boost_INCLUDE_DIRS} ${GeographicLib_INCLUDE_DIRS} ${HEADERS})
add_definitions (${GeographicLib_DEFINITIONS})
add_executable(${PROJECT_NAME} ${SOURCES})
add_subdirectory(vendor/osmpbf)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${GeographicLib_LIBRARIES} osmpbf)









