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
        src/Dijkstra.cpp)

set(HEADERS
    include/Graph.h
    include/GraphBuilder.h
    include/Grid.h
    include/Config.h include/Dijkstra.h)

find_package(Boost REQUIRED COMPONENTS system iostreams)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
    target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
endif()
add_compile_options("")



add_subdirectory(vendor/osmpbf)

target_include_directories(${PROJECT_NAME} PRIVATE include)

target_link_libraries(${PROJECT_NAME} osmpbf)
