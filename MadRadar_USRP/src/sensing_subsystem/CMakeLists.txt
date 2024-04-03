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


#enable threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

#declare libraries
add_library(SpectrogramHandler SpectrogramHandler.cpp)
add_library(EnergyDetector EnergyDetector.cpp)
add_library(SensingSubsystem SensingSubsystem.cpp)
add_library(CrossCorr CrossCorr.cpp)

#add in subdirectories
add_subdirectory(eigen)

#link libraries as needed
target_link_libraries(CrossCorr             PRIVATE BufferHandler)    
target_link_libraries(SpectrogramHandler    PRIVATE Threads::Threads
                                            PRIVATE BufferHandler
                                            PRIVATE nlohmann_json::nlohmann_json
                                            PRIVATE CrossCorr
                                            PRIVATE Eigen3:Eigen)
target_link_libraries(EnergyDetector        PRIVATE nlohmann_json::nlohmann_json
                                            PRIVATE BufferHandler)
target_link_libraries(SensingSubsystem      PRIVATE Threads::Threads
                                            PRIVATE SpectrogramHandler
                                            PRIVATE EnergyDetector
                                            PRIVATE nlohmann_json::nlohmann_json
                                            PRIVATE ${UHD_LIBRARIES} 
                                            PRIVATE ${BoostLIBRARIES}
                                            PRIVATE BufferHandler
                                            PRIVATE USRPHandler
                                            PRIVATE AttackingSubsystem
                                            PRIVATE Eigen3:Eigen)

#include directories
target_link_directories(SpectrogramHandler PRIVATE 
                        ${WORKING_DIRECTORY}/pocketfft
                        ${WORKING_DIRECTORY}/eigen/Eigen
                        ${PARENT_DIRECTORY}
                        ${PARENT_DIRECTORY}/attacking_subsystem
                        )