#pragma once

#include <atomic>
#include <thread>

#include "ifcpp/Geometry/CAdapter.h"
#include "ifcpp/Model/BuildingModel.h"
#include "ifcpp/Reader/ReaderSTEP.h"
#include <ifcpp/Geometry/GeometryGenerator.h>

namespace ifcpp {

template<CAdapter TAdapter>
std::vector<typename TAdapter::TEntity> LoadModel( const std::string& filePath, const std::shared_ptr<Parameters>& parameters,
                                                   const std::function<void( double )>& onProgressChanged, const std::atomic<bool>& isCancellationRequest = false ) {

    auto readerMessageCallback = [ onProgressChanged ]( const std::shared_ptr<StatusCallback::Message>& message ) {
        if( message->m_type == StatusCallback::PROGRESS_CHANGED ) {
            onProgressChanged( message->m_progress * 0.5 );
        }
    };

    auto readerIsCancellationRequested = [ &isCancellationRequest ] () {
        return (bool)isCancellationRequest;
    };

    auto geometryGeneratorProgressChangedCallback = [ onProgressChanged ]( double progress ) { onProgressChanged( 0.5 + progress * 0.5 ); };

    auto ifcModel = std::make_shared<BuildingModel>();
    auto reader = std::make_shared<ReaderSTEP>();
    reader->SetMessageCallBack( readerMessageCallback );
    reader->SetIsCancellationRequestedMethod( readerIsCancellationRequested );
    reader->loadModelFromFile( filePath, ifcModel );
    if( isCancellationRequest ) {
        return {};
    }
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
        std::shared_ptr<ifcpp::GeometryGenerator<TAdapter>>( new ifcpp::GeometryGenerator<TAdapter>( ifcModel, adapter, curveConverter, extruder, geometryConverter, geomUtils, primitivesConverter,
                                                              profileConverter, solidConverter, splineConverter, styleConverter, parameters ), [] (ifcpp::GeometryGenerator<TAdapter>* d) {
                                                                 std::thread( [ d ]() {
                                                                     delete d;
                                                                 } ).detach();
                                                             } );

    return geometryGenerator->GenerateGeometry( geometryGeneratorProgressChangedCallback, isCancellationRequest );
}

}