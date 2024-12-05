cmake_minimum_required(VERSION 3.6)
project(seatrac_ping_example VERSION 0.1)

find_package(seatrac_driver QUIET)
if(NOT TARGET seatrac_driver)
    include(FetchContent)
    FetchContent_Declare(seatrac_driver
        GIT_REPOSITORY https://Clayton314@bitbucket.org/frostlab/seatrac_driver.git
        GIT_TAG        main
    )
    FetchContent_MakeAvailable(seatrac_driver)
endif()

include_directories(include)

add_executable(topside_modem src/topside_modem.cpp)
target_link_libraries(topside_modem PRIVATE seatrac_driver)

