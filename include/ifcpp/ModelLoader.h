#pragma once

#include "ifcpp/Geometry/CAdapter.h"
#include "ifcpp/Model/BuildingModel.h"
#include "ifcpp/Reader/ReaderSTEP.h"
#include <ifcpp/Geometry/GeometryGenerator.h>

namespace ifcpp {

template<CAdapter TAdapter>
std::vector<typename TAdapter::TEntity> LoadModel( const std::string& filePath, const std::shared_ptr<Parameters>& parameters ) {
    auto ifcModel = std::make_shared<BuildingModel>();
    auto reader = std::make_shared<ReaderSTEP>();
    reader->loadModelFromFile( filePath, ifcModel );
    auto adapter = std::make_shared<TAdapter>();
    auto styleConverter = std::make_shared<ifcpp::StyleConverter>();
    auto geomUtils = std::make_shared<ifcpp::GeomUtils<typename TAdapter::TVector>>( parameters );
    auto primitivesConverter = std::make_shared<ifcpp::PrimitiveTypesConverter<typename TAdapter::TVector>>();
    auto splineConverter = std::make_shared<ifcpp::SplineConverter<typename TAdapter::TVector>>( primitivesConverter, geomUtils, parameters );
    auto curveConverter = std::make_shared<ifcpp::CurveConverter<typename TAdapter::TVector>>( primitivesConverter, geomUtils, splineConverter, parameters );
    auto extruder = std::make_shared<ifcpp::Extruder<typename TAdapter::TVector>>( geomUtils, parameters );
    auto profileConverter = std::make_shared<ifcpp::ProfileConverter<typename TAdapter::TVector>>( curveConverter, geomUtils, primitivesConverter, parameters );
    auto geometryConverter = std::make_shared<ifcpp::GeometryConverter<typename TAdapter::TVector>>( curveConverter, primitivesConverter, splineConverter,
                                                                                                     geomUtils, extruder, profileConverter, parameters );
    auto solidConverter = std::make_shared<ifcpp::SolidConverter<TAdapter>>( primitivesConverter, curveConverter, profileConverter, extruder, geometryConverter,
                                                                             adapter, geomUtils, styleConverter, parameters );
    auto geometryGenerator =
        std::make_shared<ifcpp::GeometryGenerator<TAdapter>>( ifcModel, adapter, curveConverter, extruder, geometryConverter, geomUtils, primitivesConverter,
                                                              profileConverter, solidConverter, splineConverter, styleConverter, parameters );

    return geometryGenerator->GenerateGeometry();
}

}