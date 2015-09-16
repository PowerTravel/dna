cmake_minimum_required (VERSION 2.8)
project (DNA)

add_executable(dna
	src/main.cpp
)

#find_package(OpenGL REQUIRED)

if( CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR )
    message( FATAL_ERROR "Please select another Build Directory !" )
endif()
if( CMAKE_SOURCE_DIR MATCHES " " )
	message( "Your Source Directory contains spaces. If you experience problems when compiling, this can be the cause." )
endif()
if( CMAKE_BINARY_DIR MATCHES " " )
	message( "Your Build Directory contains spaces. If you experience problems when compiling, this can be the cause." )
endif()


## Compile dependancies ## 
add_subdirectory(external)

## Include Directories @@
include_directories(
	external/eigen-3.2.5
#	include
)

## LINK TO LIBRARIES ##
#set(GLFW_LIBS
#	glfw
#	${GLFW_LIBRARIES}
#)
#set(GLEW_LIBS
#	GLEW
#	${GLEW_LIBRARIES}

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
#target_link_libraries(dna
	#${OPENGL_LIBRARY}
	#${GLFW_LIBS}
	#${GLEW_LIBS}
	#assimp
	#openmp
#)



 #set compiler flags
list( APPEND CMAKE_CXX_FLAGS 
	"-std=c++0x ${CMAKE_CXX_FLAGS} -g")

install(TARGETS dna DESTINATION bin)

