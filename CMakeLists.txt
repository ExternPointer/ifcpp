cmake_minimum_required( VERSION 3.20 )
project( ifcpp )

set( CMAKE_CXX_STANDARD 23 )

add_definitions( -DIFCQUERY_STATIC_LIB )

file( GLOB_RECURSE IFC_HEADERS include/ifcpp *.h )
file( GLOB_RECURSE IFC_SOURCES src *.cpp )

set( SOURCES
        ${IFC_SOURCES} )

set( HEADERS
        ${IFC_HEADERS} )



add_library( ${PROJECT_NAME} STATIC ${SOURCES} ${HEADERS} )
target_include_directories( ${PROJECT_NAME} PUBLIC include )
if( MSVC )
    target_compile_options( ${PROJECT_NAME} PRIVATE "/bigobj" )
endif()
