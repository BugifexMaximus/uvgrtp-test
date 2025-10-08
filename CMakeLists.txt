cmake_minimum_required(VERSION 3.16)
project(uvgrtp_sender LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavutil libswscale libavformat)

set(UVGRTP_FROM_SOURCE OFF)
if(EXISTS "${CMAKE_SOURCE_DIR}/uvgRTP/CMakeLists.txt")
    set(UVGRTP_FROM_SOURCE ON)
endif()

if(UVGRTP_FROM_SOURCE)
    # Ensure we do not accidentally install or test uvgRTP when using the vendored tree
    set(UVGRTP_DISABLE_INSTALL ON CACHE BOOL "" FORCE)
    set(UVGRTP_DISABLE_EXAMPLES ON CACHE BOOL "" FORCE)
    set(UVGRTP_DISABLE_TESTS ON CACHE BOOL "" FORCE)
    add_subdirectory(uvgRTP EXCLUDE_FROM_ALL)
else()
    pkg_check_modules(UVGRTP REQUIRED IMPORTED_TARGET libuvgrtp)
endif()

add_executable(sender src/sender.cpp)
target_compile_features(sender PRIVATE cxx_std_20)

target_include_directories(sender PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${FFMPEG_INCLUDE_DIRS}
)

if(NOT UVGRTP_FROM_SOURCE)
    target_include_directories(sender PRIVATE ${UVGRTP_INCLUDE_DIRS})
endif()

target_link_directories(sender PRIVATE
    ${FFMPEG_LIBRARY_DIRS}
)

if(NOT UVGRTP_FROM_SOURCE)
    target_link_directories(sender PRIVATE ${UVGRTP_LIBRARY_DIRS})
endif()

target_link_libraries(sender
    PRIVATE
        ${OpenCV_LIBS}
        PkgConfig::FFMPEG
)

if(UVGRTP_FROM_SOURCE)
    target_link_libraries(sender PRIVATE uvgrtp)
else()
    target_link_libraries(sender PRIVATE PkgConfig::UVGRTP)
endif()

if(FFMPEG_LDFLAGS_OTHER)
    target_link_options(sender PRIVATE ${FFMPEG_LDFLAGS_OTHER})
endif()

if(NOT UVGRTP_FROM_SOURCE AND UVGRTP_LDFLAGS_OTHER)
    target_link_options(sender PRIVATE ${UVGRTP_LDFLAGS_OTHER})
endif()

install(TARGETS sender)
