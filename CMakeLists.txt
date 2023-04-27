cmake_minimum_required( VERSION 3.20 )
project( ifcpp )

set( CMAKE_CXX_STANDARD 23 )

add_definitions( -DIFCQUERY_STATIC_LIB )

file( GLOB_RECURSE IFC_HEADERS include/ifcpp/Ifc *.h )
file( GLOB_RECURSE IFC_SOURCES src/Ifc *.cpp )

set( SOURCES
        src/Model/AttributeObject.cpp
        src/Model/BuildingGuid.cpp
        src/Model/BuildingModel.cpp
        src/Model/UnitConverter.cpp
        src/Reader/ReaderSTEP.cpp
        src/Reader/ReaderUtil.cpp
        src/Writer/WriterSTEP.cpp
        src/Writer/WriterUtil.cpp
        ${IFC_SOURCES} )

set( HEADERS
        include/ifcpp/Geometry/Matrix.h
        include/ifcpp/Model/AttributeObject.h
        include/ifcpp/Model/BasicTypes.h
        include/ifcpp/Model/BuildingException.h
        include/ifcpp/Model/BuildingGuid.h
        include/ifcpp/Model/BuildingModel.h
        include/ifcpp/Model/BuildingObject.h
        include/ifcpp/Model/GlobalDefines.h
        include/ifcpp/Model/OpenMPIncludes.h
        include/ifcpp/Model/StatusCallback.h
        include/ifcpp/Model/UnitConverter.h
        include/ifcpp/Model/UnknownEntityException.h
        include/ifcpp/Geometry/GeometryGenerator.h
        include/ifcpp/Geometry/CAdapter.h
        include/ifcpp/Geometry/GeometryConverter.h
        include/ifcpp/Geometry/PrimitiveTypesConverter.h
        include/ifcpp/Geometry/VectorAdapter.h
        include/ifcpp/Geometry/Parameters.h
        include/ifcpp/Geometry/CurveConverter.h
        include/ifcpp/Geometry/ProfileConverter.h
        include/ifcpp/Geometry/GeomUtils.h
        include/ifcpp/Geometry/SolidConverter.h
        include/ifcpp/Geometry/SplineConverter.h
        include/ifcpp/Geometry/Extruder.h
        include/ifcpp/Geometry/Helpers.h
        ${IFC_HEADERS} )



add_library( ${PROJECT_NAME} STATIC ${SOURCES} ${HEADERS} )
target_include_directories( ${PROJECT_NAME} PUBLIC include )
if( MSVC )
    target_compile_options( ${PROJECT_NAME} PRIVATE "/bigobj" )
endif()
