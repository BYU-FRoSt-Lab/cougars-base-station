cmake_minimum_required(VERSION 3.6)
project(seatrac_setup_tool VERSION 0.1)

find_package(seatrac_driver QUIET)
if(NOT TARGET seatrac_driver)
    include(FetchContent)
    FetchContent_Declare(seatrac_driver
        GIT_REPOSITORY https://Clayton314@bitbucket.org/frostlab/seatrac_driver.git
        GIT_TAG        main
    )
    FetchContent_MakeAvailable(seatrac_driver)
endif()

add_executable(setup_seatrac_beacon src/seatrac_beacon_setup_tool.cpp)
target_link_libraries(setup_seatrac_beacon PRIVATE seatrac_driver)
