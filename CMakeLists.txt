cmake_minimum_required(VERSION 3.15)

set(PROJECT_NAME osm-fapra)

project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 20)


set(SOURCES
    src/Graph.cpp
    src/GraphBuilder.cpp
    src/main.cpp
    src/Grid.cpp
    src/Config.cpp
        src/Dijkstra.cpp src/CHDijkstra.cpp)

set(HEADERS
    include/Graph.h
    include/GraphBuilder.h
    include/Grid.h
    include/Config.h include/Dijkstra.h include/CHDijkstra.h)

find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(GeographicLib 1.34 REQUIRED COMPONENTS SHARED)
set (GeographicLib_USE_STATIC_LIBS ON)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS} ${GeographicLib_INCLUDE_DIRS})
    add_definitions (${GeographicLib_DEFINITIONS})
    add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
    target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${GeographicLib_LIBRARIES})
endif()
add_compile_options("")



add_subdirectory(vendor/osmpbf)

target_include_directories(${PROJECT_NAME} PRIVATE include)

target_link_libraries(${PROJECT_NAME} osmpbf)
