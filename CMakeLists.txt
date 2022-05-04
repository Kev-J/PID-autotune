cmake_minimum_required(VERSION 3.8)

project(PID-autotune)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Core Widgets PrintSupport REQUIRED)
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Qt5PrintSupport_INCLUDE_DIRS})
set(POSITION_INDEPENDENT_CODE ON)

file(
        GLOB
        source_files
        src/*.cpp
        third_party/qcustomplot/*.cpp
)

file(
        GLOB
        header_files
        src/*.hpp
        third_party/qcustomplot/*.h
)

#QT4_WRAP_CPP(header_files_moc ${header_files})

#INCLUDE(${QT_USE_FILE})

add_executable(
        pid-autotune
        ${source_files}
        ${header_files_moc}
)

target_link_libraries(
        pid-autotune
        Qt5::Core Qt5::Widgets Qt5::PrintSupport
)
