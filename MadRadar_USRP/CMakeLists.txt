cmake_minimum_required(VERSION 3.0.0)
project(MadRadar_USRP VERSION 0.1.0)

include(CTest)
enable_testing()

#UHD specific code ######################################
#commands included from the uhd github website
set(CMAKE_CXX_STANDARD 11)

if(CMAKE_SYSTEM_NAME STREQUAL "FreeBSD" AND ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_EXE_LINKER_FLAGS "-lthr ${CMAKE_EXE_LINKER_FLAGS}")
    set(CMAKE_CXX_FLAGS "-stdlib=libc++ ${CMAKE_CXX_FLAGS}")
endif()

option(UHD_USE_STATIC_LIBS OFF)

find_package(UHD 4.1.0 REQUIRED)

set(UHD_BOOST_REQUIRED_COMPONENTS
    program_options
    system
    thread
)
set(BOOST_MIN_VERSION 1.65)
include(UHDBoost)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${UHD_INCLUDE_DIRS}
)
link_directories(${Boost_LIBRARY_DIRS})
#end of code from uhd github website

#User Defined Code ######################################################
#add libraries and subdirectories
add_subdirectory(src)
find_package(nlohmann_json 3.10.5 REQUIRED)

#addexecurable
add_executable(MadRadar_USRP main.cpp)

#perform linking
target_link_libraries(MadRadar_USRP    PRIVATE JSONHandler
                                            PRIVATE USRPHandler
                                            PRIVATE nlohmann_json::nlohmann_json
                                            PRIVATE ${UHD_LIBRARIES} 
                                            PRIVATE ${BoostLIBRARIES}
                                            PRIVATE BufferHandler
                                            PRIVATE RADAR
                                            PRIVATE ATTACKER
                                            PRIVATE SpectrogramHandler
                                            PRIVATE EnergyDetector
                                            PRIVATE FMCWHandler)

#include directories
target_link_directories(MadRadar_USRP PRIVATE 
                        ${PROJECT_BINARY_DIR}
                        ${PROJECT_BINARY_DIR}/src
                        ${PROJECT_BINARY_DIR}/src/sensing_subsystem
                        ${PROJECT_BINARY_DIR}/src/sensing_subsystem/pocketfft
                        ${PROJECT_BINARY_DIR}/src/sensing_subsystem/eigen/Eigen
                        )

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)