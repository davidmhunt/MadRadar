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


#add subdirectories/dependencies
find_package(nlohmann_json 3.10.5 REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

#declare libraries
add_library(RADAR RADAR.cpp)
add_library(ATTACKER Attacker.cpp)
add_library(JSONHandler JSONHandler.cpp)
add_library(USRPHandler USRPHandler.cpp)
add_library(BufferHandler BufferHandler.cpp)
add_library(FMCWHandler FMCWHandler.cpp)

#add in subdirectories
add_subdirectory(attacking_subsystem)
add_subdirectory(sensing_subsystem)


#link libraries as needed
target_link_libraries(JSONHandler PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(USRPHandler   PRIVATE nlohmann_json::nlohmann_json
                                    PRIVATE ${UHD_LIBRARIES} 
                                    PRIVATE ${BoostLIBRARIES}
                                    PRIVATE BufferHandler
                                    PRIVATE Threads::Threads
                                    PRIVATE EnergyDetector
                                    PRIVATE SpectrogramHandler)
target_link_libraries(BufferHandler PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(RADAR     PRIVATE nlohmann_json::nlohmann_json
                                PRIVATE ${UHD_LIBRARIES} 
                                PRIVATE ${BoostLIBRARIES}
                                PRIVATE BufferHandler
                                PRIVATE USRPHandler)
target_link_libraries(ATTACKER  PRIVATE nlohmann_json::nlohmann_json
                                PRIVATE ${UHD_LIBRARIES} 
                                PRIVATE ${BoostLIBRARIES}
                                PRIVATE BufferHandler
                                PRIVATE USRPHandler
                                PRIVATE SensingSubsystem
                                PRIVATE AttackingSubsystem
                                PRIVATE Threads::Threads)
target_link_libraries(FMCWHandler   PRIVATE RADAR
                                    PRIVATE ATTACKER
                                    PRIVATE nlohmann_json::nlohmann_json)

target_link_directories(ATTACKER PRIVATE 
                            ${WORKING_DIRECTORY}/sensing_subsystem
                            ${WORKING_DIRECTORY}/sensing_subsystem/pocketfft
                            ${WORKING_DIRECTORY}/sensing_subsystem/eigen/Eigen
                            ${WORKING_DIRECTORY}/attacking_subsystem)

