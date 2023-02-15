#pragma once

#include <vector>
#include <carve/geom2d.hpp>
#include <carve/geom3d.hpp>
#include <carve/matrix.hpp>

#include "ifcpp/Geometry/Curve.h"
#include "ifcpp/Geometry/Utils.h"
#include "ifcpp/Geometry/Matrix.h"

#include "ifcpp/Ifc/IfcProfileDef.h"
#include "ifcpp/Ifc/IfcBoundedCurve.h"
#include "ifcpp/Ifc/IfcParameterizedProfileDef.h"
#include "ifcpp/Ifc/IfcArbitraryClosedProfileDef.h"
#include "ifcpp/Ifc/IfcArbitraryOpenProfileDef.h"
#include "ifcpp/Ifc/IfcCompositeProfileDef.h"
#include "ifcpp/Ifc/IfcCenterLineProfileDef.h"
#include "ifcpp/Ifc/IfcDerivedProfileDef.h"
#include "ifcpp/Ifc/IfcArbitraryProfileDefWithVoids.h"
#include "ifcpp/Ifc/IfcCircleProfileDef.h"
#include "ifcpp/Ifc/IfcRectangleHollowProfileDef.h"
#include "ifcpp/Ifc/IfcPositiveLengthMeasure.h"
#include "ifcpp/Ifc/IfcNonNegativeLengthMeasure.h"
#include "ifcpp/Ifc/IfcRoundedRectangleProfileDef.h"
#include "ifcpp/Ifc/IfcTrapeziumProfileDef.h"
#include "ifcpp/Ifc/IfcCircleHollowProfileDef.h"
#include "ifcpp/Ifc/IfcEllipseProfileDef.h"
#include "ifcpp/Ifc/IfcIShapeProfileDef.h"
#include "ifcpp/Ifc/IfcAsymmetricIShapeProfileDef.h"
#include "ifcpp/Ifc/IfcLShapeProfileDef.h"
#include "ifcpp/Ifc/IfcUShapeProfileDef.h"
#include "ifcpp/Ifc/IfcPlaneAngleMeasure.h"
#include "ifcpp/Ifc/IfcCShapeProfileDef.h"
#include "ifcpp/Ifc/IfcZShapeProfileDef.h"
#include "ifcpp/Ifc/IfcTShapeProfileDef.h"
#include "ifcpp/Ifc/IfcTrimmingSelect.h"
#include "ifcpp/Ifc/IfcCartesianTransformationOperator.h"
#include "ifcpp/Ifc/IfcCartesianTransformationOperator2D.h"
#include "ifcpp/Ifc/IfcCartesianPoint.h"
#include "ifcpp/Ifc/IfcReal.h"
#include "ifcpp/Ifc/IfcDirection.h"
#include "ifcpp/Ifc/IfcCartesianTransformationOperator2DnonUniform.h"
#include "ifcpp/Ifc/IfcCartesianTransformationOperator3D.h"
#include "ifcpp/Ifc/IfcCartesianTransformationOperator3DnonUniform.h"
#include "ifcpp/Ifc/IfcAxis2Placement2D.h"

namespace ifcpp {

using namespace IFC4X3;
namespace Curve2D {

    inline void convertIfcCurve2D(const shared_ptr<IfcCurve>& ifc_curve, std::vector<carve::geom::vector<2>>& target_vec, std::vector<carve::geom::vector<2>>& segment_start_points,
                            std::vector<shared_ptr<IfcTrimmingSelect> >& trim1_vec, std::vector<shared_ptr<IfcTrimmingSelect> >& trim2_vec, bool senseAgreement, float angleFactor)
    {
        typedef carve::geom::vector<2> vec2;
        typedef carve::geom::vector<3> vec3;
        std::vector<csgjscpp::Vector> target_vec_3d;
        std::vector<csgjscpp::Vector> segment_start_points_3d;
        ConvertCurveInternal(ifc_curve, target_vec_3d, segment_start_points_3d, senseAgreement, angleFactor );

        for( size_t i = 0; i < target_vec_3d.size(); ++i )
        {
            const auto& point_3d = target_vec_3d[i];
            target_vec.push_back(carve::geom::VECTOR(point_3d.x, point_3d.y));
        }
        for( size_t i = 0; i < segment_start_points_3d.size(); ++i )
        {
            const auto& point_3d = segment_start_points_3d[i];
            segment_start_points.push_back(carve::geom::VECTOR(point_3d.x, point_3d.y));
        }
    }

    inline void convertIfcCurve2D(const shared_ptr<IfcCurve>& ifc_curve, std::vector<carve::geom::vector<2>>& loops, std::vector<carve::geom::vector<2>>& segment_start_points, bool senseAgreement, float angleFactor)
    {
        typedef carve::geom::vector<2> vec2;
        std::vector<shared_ptr<IfcTrimmingSelect> > trim1_vec;
        std::vector<shared_ptr<IfcTrimmingSelect> > trim2_vec;
        convertIfcCurve2D(ifc_curve, loops, segment_start_points, trim1_vec, trim2_vec, senseAgreement, angleFactor );
    }
}

namespace TransformConverter {
    void convertTransformationOperator( const shared_ptr<IfcCartesianTransformationOperator>& transform_operator, carve::math::Matrix& resulting_matrix )
    {
        typedef carve::geom::vector<3> vec3;
        // ENTITY IfcCartesianTransformationOperator  ABSTRACT SUPERTYPE OF(ONEOF(IfcCartesianTransformationOperator2D, IfcCartesianTransformationOperator3D))
        vec3  translate( carve::geom::VECTOR( 0.0, 0.0, 0.0 ) );
        vec3  local_x( carve::geom::VECTOR( 1.0, 0.0, 0.0 ) );
        vec3  local_y( carve::geom::VECTOR( 0.0, 1.0, 0.0 ) );
        vec3  local_z( carve::geom::VECTOR( 0.0, 0.0, 1.0 ) );

        double scale = 1.0;
        double scale_y = 1.0;
        double scale_z = 1.0;

        shared_ptr<IfcCartesianTransformationOperator2D> trans_operator_2d = dynamic_pointer_cast<IfcCartesianTransformationOperator2D>( transform_operator );
        if( trans_operator_2d )
        {
            // ENTITY IfcCartesianTransformationOperator2D SUPERTYPE OF(IfcCartesianTransformationOperator2DnonUniform)
            if( !trans_operator_2d->m_LocalOrigin )
            {
                // messageCallback( "LocalOrigin not given", StatusCallback::MESSAGE_TYPE_ERROR, __FUNC__, trans_operator_2d.get() );
                // TODO: Log error
                return;
            }
            if( trans_operator_2d->m_LocalOrigin->m_Coordinates.size() < 2 )
            {
                //messageCallback( "LocalOrigin is not valid", StatusCallback::MESSAGE_TYPE_ERROR, __FUNC__, trans_operator_2d.get() );
                // TODO: Log error
                return;
            }
            double x = trans_operator_2d->m_LocalOrigin->m_Coordinates[0]->m_value;
            double y = trans_operator_2d->m_LocalOrigin->m_Coordinates[1]->m_value;
            translate = carve::geom::VECTOR( x, y, 0.0 );

            if( trans_operator_2d->m_Scale )
            {
                scale = trans_operator_2d->m_Scale->m_value;
            }
            scale_y = scale;
            scale_z = scale;
            if( trans_operator_2d->m_Axis1 && trans_operator_2d->m_Axis2 )
            {
                if( trans_operator_2d->m_Axis1->m_DirectionRatios.size() < 2 )
                {
                    //messageCallback( "Axis1 is not valid", StatusCallback::MESSAGE_TYPE_ERROR, __FUNC__, trans_operator_2d.get() );
                    // TODO: Log error
                    return;
                }
                if( trans_operator_2d->m_Axis2->m_DirectionRatios.size() < 2 )
                {
                    //messageCallback( "Axis2 is not valid", StatusCallback::MESSAGE_TYPE_ERROR, __FUNC__, trans_operator_2d.get() );
                    // TODO: Log error
                    return;
                }

                local_x.x = trans_operator_2d->m_Axis1->m_DirectionRatios[0]->m_value;
                local_x.y = trans_operator_2d->m_Axis1->m_DirectionRatios[1]->m_value;

                local_y.x = trans_operator_2d->m_Axis2->m_DirectionRatios[0]->m_value;
                local_y.y = trans_operator_2d->m_Axis2->m_DirectionRatios[1]->m_value;
            }

            shared_ptr<IfcCartesianTransformationOperator2DnonUniform> non_uniform = dynamic_pointer_cast<IfcCartesianTransformationOperator2DnonUniform>( transform_operator );
            if( non_uniform )
            {
                if( non_uniform->m_Scale2 )
                {
                    scale_y = non_uniform->m_Scale2->m_value;
                }
            }
        }
        else
        {
            // ENTITY IfcCartesianTransformationOperator3D SUPERTYPE OF(IfcCartesianTransformationOperator3DnonUniform)
            shared_ptr<IfcCartesianTransformationOperator3D> trans_operator_3d = dynamic_pointer_cast<IfcCartesianTransformationOperator3D>( transform_operator );
            if( !trans_operator_3d )
            {
                // messageCallback( "IfcCartesianTransformationOperator is not valid", StatusCallback::MESSAGE_TYPE_ERROR, __FUNC__, trans_operator_3d.get() );
                // TODO: Log error
                return;
            }
            if( !trans_operator_3d->m_LocalOrigin )
            {
                // messageCallback( "LocalOrigin not given", StatusCallback::MESSAGE_TYPE_ERROR, __FUNC__, trans_operator_3d.get() );
                // TODO: Log error
                return;
            }
            if( trans_operator_3d->m_LocalOrigin->m_Coordinates.size() < 3 )
            {
                //messageCallback( "LocalOrigin is not valid", StatusCallback::MESSAGE_TYPE_ERROR, __FUNC__, trans_operator_3d.get() );
                // TODO: Log error
                return;
            }
            if( AllPointersValid( trans_operator_3d->m_LocalOrigin->m_Coordinates ) )
            {
                translate.x = trans_operator_3d->m_LocalOrigin->m_Coordinates[0]->m_value;
                translate.y = trans_operator_3d->m_LocalOrigin->m_Coordinates[1]->m_value;
                translate.z = trans_operator_3d->m_LocalOrigin->m_Coordinates[2]->m_value;
            }
            if( trans_operator_3d->m_Scale )
            {
                scale = trans_operator_3d->m_Scale->m_value;
            }
            scale_y = scale;
            scale_z = scale;
            if( trans_operator_3d->m_Axis1 && trans_operator_3d->m_Axis2 && trans_operator_3d->m_Axis3 )
            {
                shared_ptr<IfcDirection> axis1 = trans_operator_3d->m_Axis1;
                shared_ptr<IfcDirection> axis2 = trans_operator_3d->m_Axis2;
                shared_ptr<IfcDirection> axis3 = trans_operator_3d->m_Axis3;
                if( axis1->m_DirectionRatios.size() < 2 )
                {
                    // messageCallback( "Axis1 is not valid", StatusCallback::MESSAGE_TYPE_ERROR, __FUNC__, trans_operator_3d.get() );
                    // TODO: Log error
                    return;
                }
                if( axis2->m_DirectionRatios.size() < 2 )
                {
                    // messageCallback( "Axis2 is not valid", StatusCallback::MESSAGE_TYPE_ERROR, __FUNC__, trans_operator_3d.get() );
                    // TODO: Log error
                    return;
                }
                if( axis3->m_DirectionRatios.size() < 2 )
                {
                    // messageCallback( "Axis3 is not valid", StatusCallback::MESSAGE_TYPE_ERROR, __FUNC__, trans_operator_3d.get() );
                    // TODO: Log error
                    return;
                }
                local_x.x = axis1->m_DirectionRatios[0]->m_value;
                local_x.y = axis1->m_DirectionRatios[1]->m_value;
                local_x.z = axis1->m_DirectionRatios[2]->m_value;

                local_y.x = axis2->m_DirectionRatios[0]->m_value;
                local_y.y = axis2->m_DirectionRatios[1]->m_value;
                local_y.z = axis2->m_DirectionRatios[2]->m_value;

                local_z.x = axis3->m_DirectionRatios[0]->m_value;
                local_z.y = axis3->m_DirectionRatios[1]->m_value;
                local_z.z = axis3->m_DirectionRatios[2]->m_value;
            }

            shared_ptr<IfcCartesianTransformationOperator3DnonUniform> non_uniform = dynamic_pointer_cast<IfcCartesianTransformationOperator3DnonUniform>( transform_operator );
            if( non_uniform )
            {
                if( non_uniform->m_Scale2 )
                {
                    scale_y = non_uniform->m_Scale2->m_value;
                }
                if( non_uniform->m_Scale3 )
                {
                    scale_z = non_uniform->m_Scale3->m_value;
                }
            }
        }
        local_x.normalize();
        local_y.normalize();
        local_z.normalize();

        carve::math::Matrix rotate_translate(
            local_x.x, local_y.x, local_z.x, translate.x,
            local_x.y, local_y.y, local_z.y, translate.y,
            local_x.z, local_y.z, local_z.z, translate.z,
            0, 0, 0, 1 );
        resulting_matrix = rotate_translate*carve::math::Matrix::SCALE( scale, scale_y, scale_z ); // scale is applied first, rotate second
    }
    void convertIfcAxis2Placement2D( const shared_ptr<IfcAxis2Placement2D>& axis2placement2d, carve::math::Matrix& resulting_matrix, bool only_rotation = false )
    {
        typedef carve::geom::vector<2> vec2;
        typedef carve::geom::vector<3> vec3;
        vec3  translate( carve::geom::VECTOR( 0.0, 0.0, 0.0 ) );
        vec3  local_x( carve::geom::VECTOR( 1.0, 0.0, 0.0 ) );
        vec3  local_y( carve::geom::VECTOR( 0.0, 1.0, 0.0 ) );
        vec3  local_z( carve::geom::VECTOR( 0.0, 0.0, 1.0 ) );
        vec3  ref_direction( carve::geom::VECTOR( 1.0, 0.0, 0.0 ) );

        if( !only_rotation )
        {
            if( axis2placement2d->m_Location )
            {
                shared_ptr<IfcCartesianPoint> cartesianPoint = dynamic_pointer_cast<IfcCartesianPoint>(axis2placement2d->m_Location);
                if( cartesianPoint )
                {
                    std::vector<shared_ptr<IfcLengthMeasure> >& coords = cartesianPoint->m_Coordinates;
                    if( coords.size() > 1 )
                    {
                        translate = carve::geom::VECTOR(coords[0]->m_value, coords[1]->m_value, 0.0);
                    }
                }
                else
                {
                }
            }
        }

        if( axis2placement2d->m_RefDirection )
        {
            if( axis2placement2d->m_RefDirection->m_DirectionRatios.size() > 1 )
            {
                if( axis2placement2d->m_RefDirection->m_DirectionRatios[0] )
                {
                    ref_direction.x = axis2placement2d->m_RefDirection->m_DirectionRatios[0]->m_value;
                }
                if( axis2placement2d->m_RefDirection->m_DirectionRatios[1] )
                {
                    ref_direction.y = axis2placement2d->m_RefDirection->m_DirectionRatios[1]->m_value;
                }
                ref_direction.z = 0;
            }
        }

        local_x = ref_direction;
        vec3  z_axis( carve::geom::VECTOR( 0.0, 0.0, 1.0 ) );
        local_y = carve::geom::cross( z_axis, local_x );
        // ref_direction can be just in the x-z-plane, not perpendicular to y and z. so re-compute local x
        local_x = carve::geom::cross( local_y, local_z );

        local_x.normalize();
        local_y.normalize();
        local_z.normalize();

        resulting_matrix = carve::math::Matrix(
            local_x.x, local_y.x, local_z.x, translate.x,
            local_x.y, local_y.y, local_z.y, translate.y,
            local_x.z, local_y.z, local_z.z, translate.z,
            0, 0, 0, 1 );
    }
}

class ProfileConverter {
    typedef carve::geom::vector<2> vec2;
    typedef carve::geom::vector<3> vec3;

public:
    const std::vector<std::vector<vec2>>& getCoordinates() {
        return m_paths;
    }
    std::vector<std::vector<csgjscpp::Vector>> getCsgjscppCoords() {
        std::vector<std::vector<csgjscpp::Vector>> result;
        for(const auto& loop: m_paths) {
            std::vector<csgjscpp::Vector> l;
            for(const auto& v: loop) {
                l.emplace_back((float)v.x, (float)v.y, 0.0f);
            }
            result.push_back(l);
        }
        return result;
    }
    void clearProfileConverter() {
        m_paths.clear();
    }

protected:
    std::vector<std::vector<vec2>> m_paths;
    float m_angleFactor;

public:
    explicit ProfileConverter(float angleFactor) : m_angleFactor(angleFactor) {}
    void computeProfile( std::shared_ptr<IfcProfileDef> profile_def ) {
        // ENTITY IfcProfileDef SUPERTYPE OF(ONEOF(IfcArbitraryClosedProfileDef, IfcArbitraryOpenProfileDef, IfcCompositeProfileDef, IfcDerivedProfileDef, IfcParameterizedProfileDef));
        shared_ptr<IfcParameterizedProfileDef> parameterized = dynamic_pointer_cast<IfcParameterizedProfileDef>( profile_def );
        if( parameterized ) {
            convertIfcParameterizedProfileDefWithPosition( parameterized, m_paths );
            GeomUtils::removeDuplicates( m_paths );
            return;
        }

        shared_ptr<IfcArbitraryClosedProfileDef> arbitrary_closed = dynamic_pointer_cast<IfcArbitraryClosedProfileDef>( profile_def );
        if( arbitrary_closed ) {
            convertIfcArbitraryClosedProfileDef( arbitrary_closed, m_paths );
            GeomUtils::removeDuplicates( m_paths );
            return;
        }

        shared_ptr<IfcArbitraryOpenProfileDef> arbitrary_open = dynamic_pointer_cast<IfcArbitraryOpenProfileDef>( profile_def );
        if( arbitrary_open ) {
            convertIfcArbitraryOpenProfileDef( arbitrary_open, m_paths );
            GeomUtils::removeDuplicates( m_paths );
            return;
        }

        shared_ptr<IfcCompositeProfileDef> composite = dynamic_pointer_cast<IfcCompositeProfileDef>( profile_def );
        if( composite ) {
            convertIfcCompositeProfileDef( composite, m_paths );
            GeomUtils::removeDuplicates( m_paths );
            return;
        }

        shared_ptr<IfcDerivedProfileDef> derived = dynamic_pointer_cast<IfcDerivedProfileDef>( profile_def );
        if( derived ) {
            convertIfcDerivedProfileDef( derived, m_paths );
            GeomUtils::removeDuplicates( m_paths );
            return;
        }

        // messageCallback( "Profile not supported", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__, profile_def.get() );
        // TODO: Log error
    }
    void addAvoidingDuplicates( const std::vector<vec2>& polygon, std::vector<std::vector<vec2>>& paths ) {
        if( polygon.size() < 1 ) {
            return;
        }

        std::vector<vec2> polygon_add;
        polygon_add.push_back( polygon[ 0 ] );
        for( size_t i = 1; i < polygon.size(); ++i ) {
            const vec2& point = polygon[ i ];
            const vec2& point_previous = polygon[ i - 1 ];

            // omit duplicate points
            if( std::abs( point.x - point_previous.x ) > 0.00001 ) {
                polygon_add.push_back( point );
                continue;
            }

            if( std::abs( point.y - point_previous.y ) > 0.00001 ) {
                polygon_add.push_back( point );
                continue;
            }
        }
        paths.push_back( polygon_add );
    }
    void convertIfcArbitraryClosedProfileDef( const shared_ptr<IfcArbitraryClosedProfileDef>& profile, std::vector<std::vector<vec2>>& paths ) {
        shared_ptr<IfcCurve> outer_curve = profile->m_OuterCurve;
        if( outer_curve ) {
            std::vector<vec2> curve_polygon;
            std::vector<vec2> segment_start_points;
            Curve2D::convertIfcCurve2D( outer_curve, curve_polygon, segment_start_points, true, m_angleFactor );
            deleteLastPointIfEqualToFirst( curve_polygon );
            addAvoidingDuplicates( curve_polygon, paths );
        }

        // IfcArbitraryProfileDefWithVoids
        shared_ptr<IfcArbitraryProfileDefWithVoids> profile_with_voids = dynamic_pointer_cast<IfcArbitraryProfileDefWithVoids>( profile );
        if( profile_with_voids ) {
            std::vector<shared_ptr<IfcCurve>> inner_curves = profile_with_voids->m_InnerCurves;
            for( size_t i = 0; i < inner_curves.size(); ++i ) {
                shared_ptr<IfcCurve> inner_ifc_curve = inner_curves[ i ];
                std::vector<vec2> inner_curve_polygon;
                std::vector<vec2> segment_start_points;

                Curve2D::convertIfcCurve2D( inner_ifc_curve, inner_curve_polygon, segment_start_points, true, m_angleFactor );
                deleteLastPointIfEqualToFirst( inner_curve_polygon );
                addAvoidingDuplicates( inner_curve_polygon, paths );
            }
        }
    }
    void convertIfcArbitraryOpenProfileDef( const shared_ptr<IfcArbitraryOpenProfileDef>& profile, std::vector<std::vector<vec2>>& paths ) {
        // ENTITY IfcArbitraryOpenProfileDef
        //	SUPERTYPE OF(IfcCenterLineProfileDef)
        //	SUBTYPE OF IfcProfileDef;
        //	Curve	 :	IfcBoundedCurve;

        shared_ptr<IfcCurve> ifc_curve = profile->m_Curve;

        // IfcCenterLineProfileDef
        shared_ptr<IfcCenterLineProfileDef> center_line_profile_def = dynamic_pointer_cast<IfcCenterLineProfileDef>( profile );
        if( center_line_profile_def ) {
            if( center_line_profile_def->m_Thickness ) {
                std::vector<csgjscpp::Vector> segment_start_points;
                std::vector<csgjscpp::Vector> basis_curve_points;
                ConvertCurveInternal( ifc_curve, basis_curve_points, segment_start_points, true, m_angleFactor );

                size_t num_base_points = basis_curve_points.size();
                if( num_base_points < 2 ) {
                    return;
                }

                auto matrix_sweep = Matrix::GetIdentity();
                csgjscpp::Vector local_z( 0, 0, 1 );
                std::vector<csgjscpp::Vector> left_points;
                std::vector<csgjscpp::Vector> right_points;
                csgjscpp::Vector point_left( 0.0, -0.5, 0.0 );
                csgjscpp::Vector point_right( 0.0, 0.5, 0.0 );

                for( size_t ii = 0; ii < num_base_points; ++ii ) {
                    auto vertex_current = basis_curve_points[ ii ];
                    csgjscpp::Vector vertex_next;
                    csgjscpp::Vector vertex_before;
                    if( ii == 0 ) {
                        // first point
                        vertex_next = basis_curve_points[ ii + 1 ];
                        auto delta_element = vertex_next - vertex_current;
                        vertex_before = vertex_current - ( delta_element );
                    } else if( ii == num_base_points - 1 ) {
                        // last point
                        vertex_before = basis_curve_points[ ii - 1 ];
                        auto delta_element = vertex_current - vertex_before;
                        vertex_next = vertex_before + ( delta_element );
                    } else {
                        // inner point
                        vertex_next = basis_curve_points[ ii + 1 ];
                        vertex_before = basis_curve_points[ ii - 1 ];
                    }

                    csgjscpp::Vector bisecting_normal = BisectingPlane( vertex_before, vertex_current, vertex_next );

                    if( ii == num_base_points - 1 ) {
                        bisecting_normal = -bisecting_normal;
                    }

                    local_z.x = 0;
                    local_z.y = 0;
                    local_z.z = -1;
                    matrix_sweep = ConvertPlane2Matrix( bisecting_normal, vertex_current, local_z );

                    left_points.push_back( matrix_sweep.GetTransformed( point_left ) );
                    right_points.push_back( matrix_sweep.GetTransformed( point_right ) );
                }

                std::reverse( right_points.begin(), right_points.end() );
                std::vector<vec2> polygon;
                for( size_t i2 = 0; i2 < left_points.size(); ++i2 ) {
                    const auto& point3d = left_points[ i2 ];
                    polygon.push_back( carve::geom::VECTOR( point3d.x, point3d.y ) );
                }
                for( size_t i2 = 0; i2 < right_points.size(); ++i2 ) {
                    const auto& point3d = right_points[ i2 ];
                    polygon.push_back( carve::geom::VECTOR( point3d.x, point3d.y ) );
                }
                addAvoidingDuplicates( polygon, paths );
            }
        } else {
            std::vector<vec2> polygon;
            std::vector<vec2> segment_start_points;
            Curve2D::convertIfcCurve2D( ifc_curve, polygon, segment_start_points, true, m_angleFactor );
            addAvoidingDuplicates( polygon, paths );
        }
    }
    void convertIfcCompositeProfileDef( const shared_ptr<IfcCompositeProfileDef>& composite_profile, std::vector<std::vector<vec2>>& paths ) {
        std::vector<int> temploop_counts;
        std::vector<int> tempcontour_counts;

        std::vector<shared_ptr<IfcProfileDef>>& vec_profiles = composite_profile->m_Profiles;
        for( size_t ii = 0; ii < vec_profiles.size(); ++ii ) {
            shared_ptr<IfcProfileDef>& profile_def = vec_profiles[ ii ];

            shared_ptr<IfcParameterizedProfileDef> parameterized = dynamic_pointer_cast<IfcParameterizedProfileDef>( profile_def );
            if( parameterized ) {
                convertIfcParameterizedProfileDefWithPosition( parameterized, paths );
                continue;
            }

            shared_ptr<IfcArbitraryOpenProfileDef> open = dynamic_pointer_cast<IfcArbitraryOpenProfileDef>( profile_def );
            if( open ) {
                convertIfcArbitraryOpenProfileDef( open, paths );
                continue;
            }

            shared_ptr<IfcArbitraryClosedProfileDef> closed = dynamic_pointer_cast<IfcArbitraryClosedProfileDef>( profile_def );
            if( closed ) {
                convertIfcArbitraryClosedProfileDef( closed, paths );
                continue;
            }

            shared_ptr<IfcCompositeProfileDef> composite = dynamic_pointer_cast<IfcCompositeProfileDef>( profile_def );
            if( composite ) {
                convertIfcCompositeProfileDef( composite, paths );
                continue;
            }

            shared_ptr<IfcDerivedProfileDef> derived = dynamic_pointer_cast<IfcDerivedProfileDef>( profile_def );
            if( derived ) {
                convertIfcDerivedProfileDef( derived, paths );
                continue;
            }

            // messageCallback( "Profile not supported", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__, profile_def.get() );
            // TODO: Log error
        }
    }
    void convertIfcDerivedProfileDef( const shared_ptr<IfcDerivedProfileDef>& derived_profile, std::vector<std::vector<vec2>>& paths ) {
        ProfileConverter temp_profiler( m_angleFactor );
        temp_profiler.computeProfile( derived_profile->m_ParentProfile );
        const std::vector<std::vector<vec2>>& parent_paths = temp_profiler.getCoordinates();

        shared_ptr<IfcCartesianTransformationOperator2D> transf_op_2D = derived_profile->m_Operator;

        carve::math::Matrix transform =  carve::math::Matrix::IDENT();
        TransformConverter::convertTransformationOperator( transf_op_2D, transform );
        for( size_t i = 0; i < parent_paths.size(); ++i ) {
            const std::vector<vec2>& loop_parent = parent_paths[ i ];
            std::vector<vec2> loop;

            for( size_t j = 0; j < loop_parent.size(); ++j ) {
                const vec2& pt = loop_parent[ j ];
                vec3 pt3d( carve::geom::VECTOR( pt.x, pt.y, 0 ) );
                    pt3d = transform * pt3d;
                loop.push_back( carve::geom::VECTOR( pt3d.x, pt3d.y ) );
            }
            paths.push_back( loop );
        }
    }
    void convertIfcParameterizedProfileDefWithPosition( const shared_ptr<IfcParameterizedProfileDef>& parameterized, std::vector<std::vector<vec2>>& paths ) {
        std::vector<std::vector<vec2>> temp_paths;
        convertIfcParameterizedProfileDef( parameterized, temp_paths );

        // local coordinate system
        if( parameterized->m_Position ) {
            shared_ptr<IfcAxis2Placement2D> axis2Placement2D = parameterized->m_Position;
            carve::math::Matrix transform = carve::math::Matrix::IDENT();
            TransformConverter::convertIfcAxis2Placement2D( axis2Placement2D, transform );

            for( size_t i = 0; i < temp_paths.size(); ++i ) {
                std::vector<vec2>& path_loop = temp_paths[ i ];
                for( size_t j = 0; j < path_loop.size(); ++j ) {
                    vec2& pt = path_loop[ j ];
                    vec3 pt_3d( carve::geom::VECTOR( pt.x, pt.y, 0 ) );
                        pt_3d = transform * pt_3d;
                    pt.x = pt_3d.x;
                    pt.y = pt_3d.y;
                }
                paths.push_back( path_loop );
            }
        } else {
            for( size_t i = 0; i < temp_paths.size(); ++i ) {
                std::vector<vec2>& path_loop = temp_paths[ i ];
                paths.push_back( path_loop );
            }
        }
    }
    void convertIfcParameterizedProfileDef( const shared_ptr<IfcParameterizedProfileDef>& profile, std::vector<std::vector<vec2>>& paths ) {
        // IfcParameterizedProfileDef ABSTRACT SUPERTYPE OF (ONEOF
        //	(IfcCShapeProfileDef, IfcCircleProfileDef, IfcEllipseProfileDef, IfcIShapeProfileDef, IfcLShapeProfileDef,
        //	IfcRectangleProfileDef, IfcTShapeProfileDef, IfcTrapeziumProfileDef, IfcUShapeProfileDef, IfcZShapeProfileDef))

        const double length_factor = 1.0;
        std::vector<vec2> outer_loop;

        // Rectangle profile
        shared_ptr<IfcRectangleProfileDef> rectangle_profile = dynamic_pointer_cast<IfcRectangleProfileDef>( profile );
        if( rectangle_profile ) {
            if( rectangle_profile->m_XDim && rectangle_profile->m_YDim ) {
                double x_dim = rectangle_profile->m_XDim->m_value * length_factor;
                double y_dim = rectangle_profile->m_YDim->m_value * length_factor;

                shared_ptr<IfcRectangleHollowProfileDef> hollow = dynamic_pointer_cast<IfcRectangleHollowProfileDef>( rectangle_profile );
                if( hollow ) {
                    if( hollow->m_WallThickness ) {
                        const double t = hollow->m_WallThickness->m_value * length_factor;
                        double outer_fillet_radius = 0;
                        if( hollow->m_OuterFilletRadius ) {
                            outer_fillet_radius = hollow->m_OuterFilletRadius->m_value * length_factor;
                        }

                        double inner_fillet_radius = 0;
                        if( hollow->m_InnerFilletRadius ) {
                            inner_fillet_radius = hollow->m_InnerFilletRadius->m_value * length_factor;
                        }

                        // Outer
                        if( outer_fillet_radius != 0 ) {
                            addArc( outer_loop, outer_fillet_radius, 0, M_PI_2, x_dim * 0.5 - outer_fillet_radius, y_dim * 0.5 - outer_fillet_radius );
                            addArc( outer_loop, outer_fillet_radius, M_PI_2, M_PI_2, -x_dim * 0.5 + outer_fillet_radius, y_dim * 0.5 - outer_fillet_radius );
                            addArc( outer_loop, outer_fillet_radius, M_PI, M_PI_2, -x_dim * 0.5 + outer_fillet_radius, -y_dim * 0.5 + outer_fillet_radius );
                            addArc( outer_loop, outer_fillet_radius, 3 * M_PI_2, M_PI_2, x_dim * 0.5 - outer_fillet_radius,
                                    -y_dim * 0.5 + outer_fillet_radius );
                        } else {
                            outer_loop.push_back( carve::geom::VECTOR( -x_dim * 0.5, -y_dim * 0.5 ) );
                            outer_loop.push_back( carve::geom::VECTOR( x_dim * 0.5, -y_dim * 0.5 ) );
                            outer_loop.push_back( carve::geom::VECTOR( x_dim * 0.5, y_dim * 0.5 ) );
                            outer_loop.push_back( carve::geom::VECTOR( -x_dim * 0.5, y_dim * 0.5 ) );
                        }

                        // Inner
                        std::vector<vec2> inner_loop;
                        x_dim -= 2 * t;
                        y_dim -= 2 * t;
                        if( inner_fillet_radius != 0 ) {
                            addArc( inner_loop, inner_fillet_radius, 0, M_PI_2, x_dim * 0.5 - inner_fillet_radius, y_dim * 0.5 - inner_fillet_radius );
                            addArc( inner_loop, inner_fillet_radius, M_PI_2, M_PI_2, -x_dim * 0.5 + inner_fillet_radius, y_dim * 0.5 - inner_fillet_radius );
                            addArc( inner_loop, inner_fillet_radius, M_PI, M_PI_2, -x_dim * 0.5 + inner_fillet_radius, -y_dim * 0.5 + inner_fillet_radius );
                            addArc( inner_loop, inner_fillet_radius, 3 * M_PI_2, M_PI_2, x_dim * 0.5 - inner_fillet_radius,
                                    -y_dim * 0.5 + inner_fillet_radius );
                        } else {
                            inner_loop.push_back( carve::geom::VECTOR( -x_dim * 0.5, -y_dim * 0.5 ) );
                            inner_loop.push_back( carve::geom::VECTOR( x_dim * 0.5, -y_dim * 0.5 ) );
                            inner_loop.push_back( carve::geom::VECTOR( x_dim * 0.5, y_dim * 0.5 ) );
                            inner_loop.push_back( carve::geom::VECTOR( -x_dim * 0.5, y_dim * 0.5 ) );
                        }
                        paths.push_back( outer_loop );
                        paths.push_back( inner_loop );
                    }
                    return;
                }

                // RoundedRectangle
                shared_ptr<IfcRoundedRectangleProfileDef> rounded_rectangle = dynamic_pointer_cast<IfcRoundedRectangleProfileDef>( rectangle_profile );
                if( rounded_rectangle ) {
                    if( rounded_rectangle->m_RoundingRadius ) {
                        double rr = rounded_rectangle->m_RoundingRadius->m_value * length_factor;
                        addArc( outer_loop, rr, 0, M_PI_2, x_dim * 0.5 - rr, y_dim * 0.5 - rr );
                        addArc( outer_loop, rr, M_PI_2, M_PI_2, -x_dim * 0.5 + rr, y_dim * 0.5 - rr );
                        addArc( outer_loop, rr, M_PI, M_PI_2, -x_dim * 0.5 + rr, -y_dim * 0.5 + rr );
                        addArc( outer_loop, rr, 3 * M_PI_2, M_PI_2, x_dim * 0.5 - rr, -y_dim * 0.5 + rr );
                        paths.push_back( outer_loop );
                    }
                    return;
                }


                // else it's a standard rectangle
                outer_loop.push_back( carve::geom::VECTOR( -x_dim * 0.5, -y_dim * 0.5 ) );
                outer_loop.push_back( carve::geom::VECTOR( x_dim * 0.5, -y_dim * 0.5 ) );
                outer_loop.push_back( carve::geom::VECTOR( x_dim * 0.5, y_dim * 0.5 ) );
                outer_loop.push_back( carve::geom::VECTOR( -x_dim * 0.5, y_dim * 0.5 ) );
                paths.push_back( outer_loop );
                return;
            }
        }

        // Trapezium profile
        shared_ptr<IfcTrapeziumProfileDef> trapezium = dynamic_pointer_cast<IfcTrapeziumProfileDef>( profile );
        if( trapezium ) {
            if( trapezium->m_BottomXDim && trapezium->m_TopXDim && trapezium->m_TopXOffset && trapezium->m_YDim ) {
                double x_bottom = trapezium->m_BottomXDim->m_value * length_factor;
                double x_top = trapezium->m_TopXDim->m_value * length_factor;
                double x_offset = trapezium->m_TopXOffset->m_value * length_factor;
                double y_dim = trapezium->m_YDim->m_value * length_factor;
                outer_loop.push_back( carve::geom::VECTOR( -x_bottom * 0.5, -y_dim * 0.5 ) );
                outer_loop.push_back( carve::geom::VECTOR( x_bottom * 0.5, -y_dim * 0.5 ) );
                outer_loop.push_back( carve::geom::VECTOR( -x_bottom * 0.5 + x_offset + x_top, y_dim * 0.5 ) );
                outer_loop.push_back( carve::geom::VECTOR( -x_bottom * 0.5 + x_offset, y_dim * 0.5 ) );
                paths.push_back( outer_loop );
            }
            return;
        }

        // Circle profile
        shared_ptr<IfcCircleProfileDef> circle_profile_def = dynamic_pointer_cast<IfcCircleProfileDef>( profile );
        if( circle_profile_def ) {
            shared_ptr<IfcPositiveLengthMeasure> radius_measure = circle_profile_def->m_Radius;
            if( radius_measure ) {
                double radius = radius_measure->m_value * length_factor;
                if( radius < 0.000001 ) {
                    return;
                }
                int num_segments = 14;
                double angle = 0;
                for( int i = 0; i < num_segments; ++i ) {
                    outer_loop.push_back( carve::geom::VECTOR( ( radius * cos( angle ) ), ( radius * sin( angle ) ) ) );
                    angle += 2.0 * M_PI / double( num_segments );
                }
                paths.push_back( outer_loop );

                // CircleHollow
                std::vector<vec2> inner_loop;
                shared_ptr<IfcCircleHollowProfileDef> hollow = dynamic_pointer_cast<IfcCircleHollowProfileDef>( profile );
                if( hollow ) {
                    angle = 0;
                    radius -= hollow->m_WallThickness->m_value * length_factor;

                    int num_segments2 = 14;
                    for( int i = 0; i < num_segments2; ++i ) {
                        inner_loop.push_back( carve::geom::VECTOR( ( radius * cos( angle ) ), ( radius * sin( angle ) ) ) );
                        angle += 2.0 * M_PI / double( num_segments2 );
                    }
                    paths.push_back( inner_loop );
                }
            }
            return;
        }

        // Ellipse profile
        shared_ptr<IfcEllipseProfileDef> ellipse_profile_def = dynamic_pointer_cast<IfcEllipseProfileDef>( profile );
        if( ellipse_profile_def ) {
            if( ellipse_profile_def->m_SemiAxis1 ) {
                if( ellipse_profile_def->m_SemiAxis2 ) {
                    double x_radius = ellipse_profile_def->m_SemiAxis1->m_value * length_factor;
                    double y_radius = ellipse_profile_def->m_SemiAxis2->m_value * length_factor;
                    int num_segments = 14;
                    double angle = 0;
                    for( int i = 0; i < num_segments; ++i ) {
                        outer_loop.push_back( carve::geom::VECTOR( ( x_radius * cos( angle ) ), ( y_radius * sin( angle ) ) ) );
                        angle += 2.0 * M_PI / double( num_segments );
                    }
                    paths.push_back( outer_loop );
                }
            }
            return;
        }

        // I-shaped profile
        shared_ptr<IfcIShapeProfileDef> i_shape = dynamic_pointer_cast<IfcIShapeProfileDef>( profile );
        if( i_shape ) {
            if( i_shape->m_OverallDepth && i_shape->m_OverallWidth && i_shape->m_WebThickness && i_shape->m_FlangeThickness ) {
                const double h = i_shape->m_OverallDepth->m_value * length_factor;
                const double width = i_shape->m_OverallWidth->m_value * length_factor;
                const double tw = i_shape->m_WebThickness->m_value * length_factor;
                const double tf = i_shape->m_FlangeThickness->m_value * length_factor;
                double fillet_radius = 0;
                if( i_shape->m_FilletRadius ) {
                    fillet_radius = i_shape->m_FilletRadius->m_value * length_factor;
                }

                outer_loop.push_back( carve::geom::VECTOR( width * 0.5, -h * 0.5 ) );

                // TODO: implement flange slope

                double flange_edge_radius = 0;
                if( i_shape->m_FlangeEdgeRadius ) {
                    flange_edge_radius = i_shape->m_FlangeEdgeRadius->m_value * length_factor;
                    addArc( outer_loop, flange_edge_radius, 0, M_PI_2, width * 0.5 - flange_edge_radius, -h * 0.5 + tf - flange_edge_radius );
                } else {
                    outer_loop.push_back( carve::geom::VECTOR( width * 0.5, ( -h * 0.5 + tf ) ) );
                }

                if( fillet_radius != 0 ) {
                    addArc( outer_loop, fillet_radius, 3 * M_PI_2, -M_PI_2, tw * 0.5 + fillet_radius, -h * 0.5 + tf + fillet_radius );
                } else {
                    outer_loop.push_back( carve::geom::VECTOR( tw * 0.5, ( -h * 0.5 + tf ) ) );
                }

                shared_ptr<IfcAsymmetricIShapeProfileDef> asym_I_profile = dynamic_pointer_cast<IfcAsymmetricIShapeProfileDef>( i_shape );
                if( asym_I_profile ) {
                    if( asym_I_profile->m_TopFlangeWidth ) {
                        const double width_top = asym_I_profile->m_TopFlangeWidth->m_value * length_factor;
                        double tfTop = tf;

                        if( asym_I_profile->m_TopFlangeThickness ) {
                            tfTop = asym_I_profile->m_TopFlangeThickness->m_value * length_factor;
                        }
                        double rTop = fillet_radius;
                        if( asym_I_profile->m_TopFlangeFilletRadius ) {
                            rTop = asym_I_profile->m_TopFlangeFilletRadius->m_value * length_factor;
                        }

                        if( rTop != 0 ) {
                            addArc( outer_loop, rTop, M_PI, -M_PI_2, tw * 0.5 + rTop, h * 0.5 - tfTop - rTop );
                        } else {
                            outer_loop.push_back( carve::geom::VECTOR( tw * 0.5, ( h * 0.5 - tfTop ) ) );
                        }
                        outer_loop.push_back( carve::geom::VECTOR( width_top * 0.5, ( h * 0.5 - tfTop ) ) );
                        outer_loop.push_back( carve::geom::VECTOR( width_top * 0.5, h * 0.5 ) );
                    }
                } else {
                    // symmetric: mirror horizontally along x-Axis
                    mirrorCopyPathReverse( outer_loop, false, true );
                }

                // mirror vertically along y-axis
                mirrorCopyPathReverse( outer_loop, true, false );
                paths.push_back( outer_loop );
            }
            return;
        }

        // L-shaped profile
        shared_ptr<IfcLShapeProfileDef> l_shape = dynamic_pointer_cast<IfcLShapeProfileDef>( profile );
        if( l_shape ) {
            if( l_shape->m_Depth && l_shape->m_Thickness ) {
                const double h = l_shape->m_Depth->m_value * length_factor;
                double w = h;

                if( l_shape->m_Width ) {
                    w = l_shape->m_Width->m_value * length_factor;
                }

                double t = l_shape->m_Thickness->m_value * length_factor;

                double fillet_radius = 0;
                if( l_shape->m_FilletRadius ) {
                    fillet_radius = l_shape->m_FilletRadius->m_value * length_factor;
                }

                double edge_radius = 0;
                if( l_shape->m_EdgeRadius ) {
                    edge_radius = l_shape->m_EdgeRadius->m_value * length_factor;
                }

                double leg_slope = 0;
                if( l_shape->m_LegSlope ) {
                    const double angle_factor = m_angleFactor;
                    leg_slope = l_shape->m_LegSlope->m_value * angle_factor;
                }

                outer_loop.push_back( carve::geom::VECTOR( -w * 0.5, -h * 0.5 ) );
                outer_loop.push_back( carve::geom::VECTOR( w * 0.5, -h * 0.5 ) );

                if( edge_radius != 0 ) {
                    double y_edge_radius_start = -h * 0.5 + t - edge_radius;
                    addArc( outer_loop, edge_radius, 0, M_PI_2 - leg_slope, w * 0.5 - edge_radius, y_edge_radius_start );
                } else {
                    outer_loop.push_back( carve::geom::VECTOR( w * 0.5, ( -h * 0.5 + t ) ) );
                }

                const double s = sin( leg_slope );
                const double c = cos( leg_slope );
                const double z1 = ( -s * ( ( c - s ) * ( fillet_radius + edge_radius + t ) - c * w + s * h ) ) / ( 2 * c * c - 1 );
                const double z2 = ( -s * ( ( c - s ) * ( fillet_radius + edge_radius + t ) - c * h + s * w ) ) / ( 2 * c * c - 1 );
                if( fillet_radius != 0 ) {
                    addArc( outer_loop, fillet_radius, 3 * M_PI_2 - leg_slope, -M_PI_2 + 2 * leg_slope, -w * 0.5 + t + z2 + fillet_radius,
                            -h * 0.5 + t + z1 + fillet_radius );
                } else {
                    outer_loop.push_back( carve::geom::VECTOR( ( -w * 0.5 + t + z2 ), ( -h * 0.5 + t + z1 ) ) );
                }

                if( edge_radius != 0 ) {
                    addArc( outer_loop, edge_radius, leg_slope, M_PI_2 - leg_slope, -w * 0.5 + t - edge_radius, h * 0.5 - edge_radius );
                } else {
                    outer_loop.push_back( carve::geom::VECTOR( ( -w * 0.5 + t ), h * 0.5 ) );
                }

                outer_loop.push_back( carve::geom::VECTOR( -w * 0.5, h * 0.5 ) );
                paths.push_back( outer_loop );
            }
            return;
        }

        // U-shaped profile
        shared_ptr<IfcUShapeProfileDef> u_shape = dynamic_pointer_cast<IfcUShapeProfileDef>( profile );
        if( u_shape ) {
            if( u_shape->m_Depth && u_shape->m_FlangeWidth && u_shape->m_WebThickness && u_shape->m_FlangeThickness ) {
                const double height = u_shape->m_Depth->m_value * length_factor;
                const double width = u_shape->m_FlangeWidth->m_value * length_factor;
                const double tw = u_shape->m_WebThickness->m_value * length_factor;
                const double tf = u_shape->m_FlangeThickness->m_value * length_factor;
                double fillet_radius = 0;
                if( u_shape->m_FilletRadius ) {
                    fillet_radius = u_shape->m_FilletRadius->m_value * length_factor;
                }
                double edge_radius = 0;
                if( u_shape->m_EdgeRadius ) {
                    edge_radius = u_shape->m_EdgeRadius->m_value * length_factor;
                }
                double fs = 0;
                if( u_shape->m_FlangeSlope ) {
                    const double angle_factor = m_angleFactor;
                    fs = u_shape->m_FlangeSlope->m_value * angle_factor;
                }

                outer_loop.push_back( carve::geom::VECTOR( -width * 0.5, -height * 0.5 ) );
                outer_loop.push_back( carve::geom::VECTOR( width * 0.5, -height * 0.5 ) );

                double z = tan( fs ) * ( width * 0.5 - edge_radius );
                if( edge_radius != 0 ) {
                    addArc( outer_loop, edge_radius, 0, M_PI_2 - fs, width * 0.5 - edge_radius, -height * 0.5 + tf - z - edge_radius );
                } else {
                    outer_loop.push_back( carve::geom::VECTOR( width * 0.5, ( -height * 0.5 + tf - z ) ) );
                }

                z = tan( fs ) * ( width * 0.5 - tw - fillet_radius );
                if( fillet_radius != 0 ) {
                    addArc( outer_loop, fillet_radius, 3 * M_PI_2 - fs, -M_PI_2 + fs, -width * 0.5 + tw + fillet_radius,
                            -height * 0.5 + tf + z + fillet_radius );
                } else {
                    outer_loop.push_back( carve::geom::VECTOR( ( -width * 0.5 + tw ), ( -height * 0.5 + tf + z ) ) );
                }

                // mirror horizontally along x-Axis
                mirrorCopyPathReverse( outer_loop, false, true );
                paths.push_back( outer_loop );
            }
            return;
        }

        // C-shaped profile
        shared_ptr<IfcCShapeProfileDef> c_shape = dynamic_pointer_cast<IfcCShapeProfileDef>( profile );
        if( c_shape ) {
            if( c_shape->m_Depth && c_shape->m_Width && c_shape->m_Girth && c_shape->m_WallThickness ) {
                const double h = c_shape->m_Depth->m_value * length_factor;
                const double width = c_shape->m_Width->m_value * length_factor;
                const double g = c_shape->m_Girth->m_value * length_factor;
                const double t = c_shape->m_WallThickness->m_value * length_factor;
                double fillet_radius = 0;
                if( c_shape->m_InternalFilletRadius ) {
                    fillet_radius = c_shape->m_InternalFilletRadius->m_value * length_factor;
                }

                if( fillet_radius != 0 ) {
                    addArc( outer_loop, fillet_radius + t, M_PI, M_PI_2, -width * 0.5 + t + fillet_radius, -h * 0.5 + t + fillet_radius );
                } else {
                    outer_loop.push_back( carve::geom::VECTOR( -width * 0.5, -h * 0.5 ) );
                }

                if( fillet_radius != 0 ) {
                    addArc( outer_loop, fillet_radius + t, 3 * M_PI_2, M_PI_2, width * 0.5 - t - fillet_radius, -h * 0.5 + t + fillet_radius );
                } else {
                    outer_loop.push_back( carve::geom::VECTOR( width * 0.5, -h * 0.5 ) );
                }

                outer_loop.push_back( carve::geom::VECTOR( width * 0.5, ( -h * 0.5 + g ) ) );
                outer_loop.push_back( carve::geom::VECTOR( ( width * 0.5 - t ), ( -h * 0.5 + g ) ) );

                if( fillet_radius != 0 ) {
                    addArc( outer_loop, fillet_radius, 0, -M_PI_2, width * 0.5 - t - fillet_radius, -h * 0.5 + t + fillet_radius );
                } else {
                    outer_loop.push_back( carve::geom::VECTOR( ( width * 0.5 - t ), ( -h * 0.5 + t ) ) );
                }

                if( fillet_radius != 0 ) {
                    addArc( outer_loop, fillet_radius, 3 * M_PI_2, -M_PI_2, -width * 0.5 + t + fillet_radius, -h * 0.5 + t + fillet_radius );
                } else {
                    outer_loop.push_back( carve::geom::VECTOR( ( -width * 0.5 + t ), ( -h * 0.5 + t ) ) );
                }
                // mirror horizontally along x-Axis
                mirrorCopyPathReverse( outer_loop, false, true );
                paths.push_back( outer_loop );
            }
            return;
        }

        // Z-shape profile
        shared_ptr<IfcZShapeProfileDef> z_shape = dynamic_pointer_cast<IfcZShapeProfileDef>( profile );
        if( z_shape ) {
            if( z_shape->m_Depth && z_shape->m_FlangeWidth && z_shape->m_WebThickness && z_shape->m_FlangeThickness ) {
                const double h = z_shape->m_Depth->m_value * length_factor;
                const double width = z_shape->m_FlangeWidth->m_value * length_factor;
                const double tw = z_shape->m_WebThickness->m_value * length_factor;
                const double tf = z_shape->m_FlangeThickness->m_value * length_factor;
                double fillet_radius = 0;
                if( z_shape->m_FilletRadius ) {
                    fillet_radius = z_shape->m_FilletRadius->m_value * length_factor;
                }

                double edge_radius = 0;
                if( z_shape->m_EdgeRadius ) {
                    edge_radius = z_shape->m_EdgeRadius->m_value * length_factor;
                }

                outer_loop.push_back( carve::geom::VECTOR( ( -tw * 0.5 ), -h * 0.5 ) );
                outer_loop.push_back( carve::geom::VECTOR( ( width - tw * 0.5 ), -h * 0.5 ) );

                if( edge_radius != 0 ) {
                    addArc( outer_loop, edge_radius, 0, M_PI_2, width - tw * 0.5 - edge_radius, -h * 0.5 + tf - edge_radius );
                } else {
                    outer_loop.push_back( carve::geom::VECTOR( ( width - tw * 0.5 ), ( -h * 0.5 + tf ) ) );
                }

                if( fillet_radius != 0 ) {
                    addArc( outer_loop, fillet_radius, 3 * M_PI_2, -M_PI_2, tw * 0.5 + fillet_radius, -h * 0.5 + tf + fillet_radius );
                } else {
                    outer_loop.push_back( carve::geom::VECTOR( ( tw * 0.5 ), ( -h * 0.5 + tf ) ) );
                }

                // mirror horizontally and vertically
                mirrorCopyPath( outer_loop, true, true );
                paths.push_back( outer_loop );
            }
            return;
        }

        // T-shape profile
        shared_ptr<IfcTShapeProfileDef> t_shape = dynamic_pointer_cast<IfcTShapeProfileDef>( profile );
        if( t_shape ) {
            const double h = t_shape->m_Depth->m_value * length_factor;
            const double width = t_shape->m_FlangeWidth->m_value * length_factor;
            const double tw = t_shape->m_WebThickness->m_value * length_factor * 0.5;
            const double tf = t_shape->m_FlangeThickness->m_value * length_factor;

            double fillet_radius = 0;
            if( t_shape->m_FilletRadius ) {
                fillet_radius = t_shape->m_FilletRadius->m_value * length_factor;
            }

            double flange_edge_radius = 0;
            if( t_shape->m_FlangeEdgeRadius ) {
                flange_edge_radius = t_shape->m_FlangeEdgeRadius->m_value * length_factor;
            }

            double web_edge_radius = 0;
            if( t_shape->m_WebEdgeRadius ) {
                web_edge_radius = t_shape->m_WebEdgeRadius->m_value * length_factor;
            }
            double flange_slope = 0;

            if( t_shape->m_FlangeSlope ) {
                const double angle_factor = m_angleFactor;
                flange_slope = t_shape->m_FlangeSlope->m_value * angle_factor;
            }

            double web_slope = 0;
            if( t_shape->m_WebSlope ) {
                const double angle_factor = m_angleFactor;
                web_slope = t_shape->m_WebSlope->m_value * angle_factor;
            }

            outer_loop.push_back( carve::geom::VECTOR( -width * 0.5, h * 0.5 ) );

            const double zf = tan( flange_slope ) * ( width * 0.25 - flange_edge_radius );
            const double zw = tan( web_slope ) * ( h * 0.5 - web_edge_radius );
            if( flange_edge_radius != 0 ) {
                addArc( outer_loop, flange_edge_radius, M_PI, M_PI_2 - flange_slope, -width * 0.5 + flange_edge_radius,
                        h * 0.5 - tf + zf + flange_edge_radius );
            } else {
                outer_loop.push_back( carve::geom::VECTOR( -width * 0.5, ( h * 0.5 - tf + zf ) ) );
            }

            const double cf = cos( flange_slope );
            const double sf = sin( flange_slope );
            const double cw = cos( web_slope );
            const double sw = sin( web_slope );
            const double z1 =
                ( sf *
                  ( ( width - 2 * ( fillet_radius + flange_edge_radius + tw - zw ) ) * cw - 2 * ( h - web_edge_radius - fillet_radius - tf + zf ) * sw ) ) /
                ( 2 * ( cf * cw - sf * sw ) );
            const double z2 = tan( web_slope ) * ( h - web_edge_radius - fillet_radius - z1 - tf + zf );
            if( fillet_radius != 0 ) {
                addArc( outer_loop, fillet_radius, M_PI_2 - flange_slope, -M_PI_2 + flange_slope + web_slope, -tw + zw - z2 - fillet_radius,
                        h * 0.5 - tf + zf - z1 - fillet_radius );
            } else {
                outer_loop.push_back( carve::geom::VECTOR( ( -tw + zw - z2 ), ( h * 0.5 - tf + zf - z1 ) ) );
            }

            if( web_edge_radius != 0 ) {
                double x_center = -tw + zw + web_edge_radius;
                if( x_center > 0 ) {
                    x_center = 0;
                }
                addArc( outer_loop, web_edge_radius, M_PI + web_slope, M_PI_2 - web_slope, x_center, -h * 0.5 + web_edge_radius );
                while( outer_loop.size() > 0 ) {
                    if( outer_loop.back().x < 0 ) {
                        break;
                    }
                    outer_loop.pop_back();
                }
                outer_loop.push_back( carve::geom::VECTOR( 0, -h * 0.5 ) );
            } else {
                outer_loop.push_back( carve::geom::VECTOR( ( -tw + zw ), -h * 0.5 ) );
            }

            // mirror vertically along y-Axis
            mirrorCopyPathReverse( outer_loop, true, false );
            paths.push_back( outer_loop );
            return;
        }

        //messageCallback( "Profile not supported", StatusCallback::MESSAGE_TYPE_WARNING, __FUNC__, profile.get() );
        // TODO: Log error
    }

    static void deleteLastPointIfEqualToFirst( std::vector<vec2>& coords ) {
        while( coords.size() > 2 ) {
            vec2& first = coords.front();
            vec2& last = coords.back();

            if( std::abs( first.x - last.x ) < 0.00000001 ) {
                if( std::abs( first.y - last.y ) < 0.00000001 ) {
                    coords.pop_back();
                    continue;
                }
            }
            break;
        }
    }

    void simplifyPaths() {
        simplifyPaths( m_paths );
    }

    static void simplifyPaths( std::vector<std::vector<vec2>>& paths ) {
        for( std::vector<std::vector<vec2>>::iterator it_paths = paths.begin(); it_paths != paths.end(); ++it_paths ) {
            std::vector<vec2>& path = ( *it_paths );
            if( path.size() < 3 ) {
                continue;
            }
            simplifyPath( path );
        }
    }

    static void simplifyPath( std::vector<vec2>& path ) {
        if( path.size() < 3 ) {
            return;
        }

        for( size_t i = 1; i < path.size(); ) {
            vec2& previous = path[ i - 1 ];
            vec2& current = path[ i ];

            vec2 segment1 = current - previous;
            if( segment1.length2() < 0.000000001 ) {
                path.erase( path.begin() + i );
                continue;
            }
            ++i;
        }

        for( size_t i = 1; i < path.size() - 1; ) {
            vec2& previous = path[ i - 1 ];
            vec2& current = path[ i ];
            vec2& next = path[ i + 1 ];

            vec2 segment1 = current - previous;
            segment1.normalize();
            vec2 segment2 = next - current;
            segment2.normalize();
            double angle = std::abs( segment1.x * segment2.x + segment1.y * segment2.y );
            if( std::abs( angle - 1 ) < 0.00001 ) {
                // points are colinear, current point can be removed
                path.erase( path.begin() + i );
                continue;
            }
            ++i;
        }

        // 1-----0 5-----4      0-----------3         1---0 4---3      0-----------2
        // |             |  ->  |           |         |   _ ---    ->  |   _ ---
        // 2-------------3      1-----------2         2--              1---

        if( path.size() > 4 ) {
            vec2& first = path.front();
            vec2& last = path.back();

            if( ( last - first ).length2() < 0.000001 ) {
                vec2 first_segment = path[ 1 ] - first;
                first_segment.normalize();
                vec2 last_segment = last - path[ path.size() - 2 ];
                last_segment.normalize();
                double angle = std::abs( first_segment.x * last_segment.x + first_segment.y * last_segment.y );
                if( std::abs( angle - 1 ) < 0.00001 ) {
                    // remove first and last point
                    path.erase( path.begin() );
                    path.pop_back();
                }
            }
        }
    }

    void addArc( std::vector<vec2>& coords, double radius, double start_angle, double opening_angle, double x_center, double y_center,
                 int num_segments = 4 ) const {
        // int num_segments = (int)( std::abs( opening_angle ) / ( 2.0*M_PI )*gs->getNumVerticesPerCircle() ); // TODO: adapt to model size and complexity
        if( num_segments < 5 ) {
            num_segments = 5;
        }

        if( num_segments > 100 ) {
            num_segments = 100;
        }

        double angle = start_angle;
        double angle_delta = opening_angle / (double)( num_segments );
        for( int i = 0; i < num_segments + 1; ++i ) {
            coords.push_back( carve::geom::VECTOR( radius * cos( angle ) + x_center, radius * sin( angle ) + y_center ) );
            angle += angle_delta;
        }
    }

    static void mirrorCopyPath( std::vector<vec2>& coords, bool mirror_on_y_axis, bool mirror_on_x_axis ) {
        int points_count = coords.size();
        double x, y;
        for( int i = 0; i < points_count; ++i ) {
            vec2& p = coords[ i ];
            if( mirror_on_y_axis ) {
                x = -p.x;
            } else {
                x = p.x;
            }
            if( mirror_on_x_axis ) {
                y = -p.y;
            } else {
                y = p.y;
            }
            coords.push_back( carve::geom::VECTOR( x, y ) );
        }
    }

    static void mirrorCopyPathReverse( std::vector<vec2>& coords, bool mirror_on_y_axis, bool mirror_on_x_axis ) {
        int points_count = coords.size();
        double x, y;
        for( int i = points_count - 1; i >= 0; --i ) {
            vec2& p = coords[ i ];
            if( mirror_on_y_axis ) {
                x = -p.x;
            } else {
                x = p.x;
            }
            if( mirror_on_x_axis ) {
                y = -p.y;
            } else {
                y = p.y;
            }

            coords.push_back( carve::geom::VECTOR( x, y ) );
        }
    }
};

}