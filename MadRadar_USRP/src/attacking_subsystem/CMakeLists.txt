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
add_library(AttackingSubsystem AttackingSubsystem.cpp)


#link libraries as needed
target_link_libraries(AttackingSubsystem    PRIVATE Threads::Threads
                                            PRIVATE nlohmann_json::nlohmann_json
                                            PRIVATE ${UHD_LIBRARIES} 
                                            PRIVATE ${BoostLIBRARIES}
                                            PRIVATE BufferHandler
                                            PRIVATE USRPHandler)

#include directories
target_link_directories(AttackingSubsystem PRIVATE 
                        ${PARENT_DIRECTORY}
                        )