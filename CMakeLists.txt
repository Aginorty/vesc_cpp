cmake_minimum_required(VERSION 3.10)
project(vesc_test LANGUAGES CXX)

find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(spdlog REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

list(APPEND SOURCES main.cpp
        src/VescUart.cpp
        src/crc.cpp
        src/buffer.cpp)

file(GLOB_RECURSE HEADERS **/*.h)

add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} spdlog::spdlog)