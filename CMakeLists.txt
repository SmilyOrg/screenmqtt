cmake_minimum_required(VERSION 3.2)
include(ExternalProject)

project(screenmqtt VERSION 0.2 LANGUAGES CXX)

add_definitions(
)

set(includes
	src/
	src/vendor/paho.mqtt.c/src/
	src/vendor/yaml-cpp/include/
)

file(GLOB sources
	src/screenmqtt.cpp
	src/msgqueue.cpp
)

add_subdirectory(src/vendor/paho.mqtt.c)
add_subdirectory(src/vendor/yaml-cpp)

add_executable(screenmqtt ${sources})
target_link_libraries(screenmqtt dxva2 paho-mqtt3c yaml-cpp)
target_include_directories(screenmqtt PRIVATE ${includes})

add_custom_command(TARGET screenmqtt POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_if_different
		"${PROJECT_SOURCE_DIR}/build/src/vendor/paho.mqtt.c/src/$<CONFIGURATION>/paho-mqtt3c.dll"
		$<TARGET_FILE_DIR:screenmqtt>/paho-mqtt3c.dll)
