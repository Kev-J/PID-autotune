cmake_minimum_required(VERSION 2.6)

project(PID-autotune)

FIND_PACKAGE(Qt4 REQUIRED)

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

QT4_WRAP_CPP(header_files_moc ${header_files})

INCLUDE(${QT_USE_FILE})

ADD_DEFINITIONS(${QT_DEFINITIONS})

add_executable(
	pid-autotune
	${source_files}
	${header_files_moc}
)

target_link_libraries(
	pid-autotune
	${QT_LIBRARIES}
)
