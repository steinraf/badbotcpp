cmake_minimum_required(VERSION 3.8)
project(BadBot++ LANGUAGES CXX)



add_subdirectory(RLBotCPP)

include_directories(/bot /brain /util /sequence)

add_executable(BadBot++ main.cc badbot.cc badbot.h bot/state.cc bot/state.h brain/path.cpp brain/path.h util/util.cpp util/util.h sequence/sequence.cpp sequence/sequence.h)

target_include_directories(BadBot++ PRIVATE RLBotCPP/inc RLBotCPP/lib/inc)
target_include_directories(BadBot++ PRIVATE lib/eigen-3.3.9)

if (UNIX)
    target_link_libraries(BadBot++ RLBotCPP pthread)
else (UNIX)
    target_link_libraries(BadBot++ RLBotCPP)
endif (UNIX)


target_compile_features(BadBot++ PRIVATE cxx_std_17)

# Copy port config to output folder
add_custom_command(TARGET BadBot++ POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PROJECT_SOURCE_DIR}/port.cfg"
        $<TARGET_FILE_DIR:BadBot++>)