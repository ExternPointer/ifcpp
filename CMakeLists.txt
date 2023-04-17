cmake_minimum_required( VERSION 3.20 )
project( ifcpp )

set( CMAKE_CXX_STANDARD 23 )

add_definitions( -DIFCQUERY_STATIC_LIB )

file( GLOB_RECURSE IFC_HEADERS include/ifcpp/Ifc *.h )
file( GLOB_RECURSE IFC_SOURCES src/Ifc *.cpp )
file( GLOB_RECURSE CARVE third_party/carve *.* )
file( GLOB_RECURSE EARCUT third_party/earcut *.* )

set( SOURCES
        src/Geometry/csgjs.cpp
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
        include/ifcpp/Geometry/csgjs.h
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
        include/ifcpp/Geometry/Adapter.h
        include/ifcpp/Geometry/GeometryConverter.h
        include/ifcpp/Geometry/PrimitivesConverter.h
        include/ifcpp/Geometry/VectorAdapter.h
        include/ifcpp/Geometry/Parameters.h
        include/ifcpp/Geometry/CurveConverter.h
        include/ifcpp/Geometry/ProfileConverter.h
        include/ifcpp/Geometry/GeomUtils.h
        include/ifcpp/Geometry/SolidConverter.h
        include/ifcpp/Geometry/SplineConverter.h
        include/ifcpp/Geometry/Extruder.h
        ${IFC_HEADERS} )



add_library( ${PROJECT_NAME} STATIC ${SOURCES} ${HEADERS} ${CARVE} ${EARCUT} )
target_include_directories( ${PROJECT_NAME} PUBLIC include )
include_directories( third_party/carve/include )
include_directories( third_party/earcut/include )
if( MSVC )
    target_compile_options( ${PROJECT_NAME} PRIVATE "/bigobj" )
endif()
