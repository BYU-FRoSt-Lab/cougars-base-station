cmake_minimum_required(VERSION 3.8)
project(base_station_coms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(seatrac_interfaces REQUIRED)
find_package(base_station_interfaces REQUIRED)


include_directories(include)

add_executable(base_station_coms src/base_station_coms.cpp)
ament_target_dependencies(base_station_coms rclcpp std_srvs seatrac_interfaces base_station_interfaces)

add_executable(modem_pinger src/modem_pinger.cpp)
ament_target_dependencies(modem_pinger rclcpp std_srvs seatrac_interfaces)

install(DIRECTORY include/
DESTINATION include/${PROJECT_NAME})

install(TARGETS
base_station_coms
modem_pinger
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
set(ament_cmake_copyright_FOUND TRUE)
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()

ament_package()