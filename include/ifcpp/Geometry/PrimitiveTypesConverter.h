#pragma once

#include "ifcpp/Geometry/CAdapter.h"
#include "ifcpp/Geometry/Matrix.h"
#include "ifcpp/Geometry/VectorAdapter.h"

#include "ifcpp/Ifc/IfcAxis1Placement.h"
#include "ifcpp/Ifc/IfcAxis2Placement2D.h"
#include "ifcpp/Ifc/IfcAxis2Placement3D.h"
#include "ifcpp/Ifc/IfcCartesianPoint.h"
#include "ifcpp/Ifc/IfcCartesianTransformationOperator.h"
#include "ifcpp/Ifc/IfcCartesianTransformationOperator2D.h"
#include "ifcpp/Ifc/IfcCartesianTransformationOperator2DnonUniform.h"
#include "ifcpp/Ifc/IfcCartesianTransformationOperator3D.h"
#include "ifcpp/Ifc/IfcCartesianTransformationOperator3DnonUniform.h"
#include "ifcpp/Ifc/IfcDirection.h"
#include "ifcpp/Ifc/IfcGridPlacement.h"
#include "ifcpp/Ifc/IfcLengthMeasure.h"
#include "ifcpp/Ifc/IfcLocalPlacement.h"
#include "ifcpp/Ifc/IfcObjectPlacement.h"
#include "ifcpp/Ifc/IfcPlacement.h"
#include "ifcpp/Ifc/IfcReal.h"
#include "ifcpp/Ifc/IfcVertex.h"
#include "ifcpp/Ifc/IfcVertexPoint.h"
#include "ifcpp/Ifc/IfcVector.h"


namespace ifcpp {
using namespace IFC4X3;

template<CVector TVector>
class PrimitiveTypesConverter {
    using TMatrix = Matrix<TVector>;
    using AVector = VectorAdapter<TVector>;

public:
    TVector ConvertVector( const std::shared_ptr<IfcVector>& vector ) {
        if( !vector || !vector->m_Orientation || !vector->m_Magnitude ) {
            // TODO: Log error
            return AVector::New();
        }
        return this->ConvertPoint( vector->m_Orientation->m_DirectionRatios ) * vector->m_Magnitude->m_value;
    }
    TVector ConvertPoint( const std::vector<std::shared_ptr<IfcLengthMeasure>>& coords ) {
        if( coords.size() > 2 ) {
            return AVector::New( coords[ 0 ]->m_value, coords[ 1 ]->m_value, coords[ 2 ]->m_value );
        } else if( coords.size() > 1 ) {
            return AVector::New( coords[ 0 ]->m_value, coords[ 1 ]->m_value, 0.0 );
        }
        // TODO: Log error
        return AVector::New();
    }
    TVector ConvertPoint( const std::vector<std::shared_ptr<IfcReal>>& coords ) {
        if( coords.size() > 2 ) {
            return AVector::New( coords[ 0 ]->m_value, coords[ 1 ]->m_value, coords[ 2 ]->m_value );
        } else if( coords.size() > 1 ) {
            return AVector::New( coords[ 0 ]->m_value, coords[ 1 ]->m_value, 0.0 );
        }
        // TODO: Log error
        return AVector::New();
    }
    TVector ConvertPoint( const std::shared_ptr<IfcCartesianPoint>& cartesianPoint ) {
        if( !cartesianPoint ) {
            // TODO: Log error
            return AVector::New();
        }
        TVector result;
        if( cartesianPoint->m_size > 0 ) {
            result.x = cartesianPoint->m_Coordinates[0];
        }
        if( cartesianPoint->m_size > 1 ) {
            result.y = cartesianPoint->m_Coordinates[1];
        }
        if( cartesianPoint->m_size > 2 ) {
            result.z = cartesianPoint->m_Coordinates[2];
        }
        return result;
    }
    TVector ConvertPoint( const std::shared_ptr<IfcPoint>& point ) {
        if( const auto cartesianPoint = std::dynamic_pointer_cast<IfcCartesianPoint>( point ) ) {
            return ConvertPoint( cartesianPoint );
        }
        // TODO: Log error
        return AVector::New();
    }
    std::vector<TVector> ConvertPoints( const std::vector<std::shared_ptr<IfcCartesianPoint>>& points ) {
        std::vector<TVector> result;
        for( const auto& point: points ) {
            result.push_back( ConvertPoint( point ) );
        }
        return result;
    }
    std::vector<TVector> ConvertPoints( const std::vector<std::vector<shared_ptr<IfcLengthMeasure>>>& points ) {
        std::vector<TVector> result;
        for( const auto& coords: points ) {
            result.push_back( ConvertPoint( coords ) );
        }
        return result;
    }
    TVector ConvertVertex( const std::shared_ptr<IfcVertex>& vertex ) {
        if( auto v = dynamic_pointer_cast<IfcVertexPoint>( vertex ) ) {
            return ConvertPoint( dynamic_pointer_cast<IfcCartesianPoint>( v->m_VertexGeometry ) );
        }
        // TODO: Log error
        return AVector::New();
    }

    TMatrix ConvertPlacement( const std::shared_ptr<IfcAxis2Placement2D>& placement, bool onlyRotation = false ) {
        if( !placement ) {
            return TMatrix::GetIdentity();
        }
        auto translate = AVector::New( 0.0, 0.0, 0.0 );
        auto local_x = AVector::New( 1.0, 0.0, 0.0 );
        auto local_y = AVector::New( 0.0, 1.0, 0.0 );
        auto local_z = AVector::New( 0.0, 0.0, 1.0 );
        auto ref_direction = AVector::New( 1.0, 0.0, 0.0 );

        if( !onlyRotation ) {
            translate = ConvertPoint( dynamic_pointer_cast<IfcCartesianPoint>( placement->m_Location ) );
        }

        if( placement->m_RefDirection ) {
            if( placement->m_RefDirection->m_DirectionRatios.size() > 1 ) {
                if( placement->m_RefDirection->m_DirectionRatios[ 0 ] ) {
                    ref_direction.x = placement->m_RefDirection->m_DirectionRatios[ 0 ]->m_value;
                }
                if( placement->m_RefDirection->m_DirectionRatios[ 1 ] ) {
                    ref_direction.y = placement->m_RefDirection->m_DirectionRatios[ 1 ]->m_value;
                }
                ref_direction.z = 0;
            }
        }

        local_x = ref_direction;
        auto z_axis = AVector::New( 0.0, 0.0, 1.0 );
        local_y = AVector::Cross( z_axis, local_x );
        local_x = AVector::Cross( local_y, local_z );

        AVector::Normalize( &local_x );
        AVector::Normalize( &local_y );
        AVector::Normalize( &local_z );

        return TMatrix::CreateFromAxises( local_x, local_y, local_z, translate );
    }
    TMatrix ConvertPlacement( const std::shared_ptr<IfcAxis2Placement3D>& placement, bool onlyRotation = false ) {
        if( !placement ) {
            return TMatrix::GetIdentity();
        }
        auto translate = AVector::New( 0.0, 0.0, 0.0 );
        auto local_x = AVector::New( 1.0, 0.0, 0.0 );
        auto local_y = AVector::New( 0.0, 1.0, 0.0 );
        auto local_z = AVector::New( 0.0, 0.0, 1.0 );
        auto ref_direction = AVector::New( 1.0, 0.0, 0.0 );

        if( !onlyRotation ) {
            translate = this->ConvertPoint( placement->m_Location );
        }

        if( placement->m_Axis ) {
            local_z = this->ConvertPoint( placement->m_Axis->m_DirectionRatios );
        }

        if( placement->m_RefDirection ) {
            ref_direction = this->ConvertPoint( placement->m_RefDirection->m_DirectionRatios );
        }

        local_x = ref_direction;
        local_y = AVector::Cross( local_z, local_x );
        local_x = AVector::Cross( local_y, local_z );

        AVector::Normalize( &local_x );
        AVector::Normalize( &local_y );
        AVector::Normalize( &local_z );

        return TMatrix::CreateFromAxises( local_x, local_y, local_z, translate );
    }
    TMatrix ConvertPlacement( const std::shared_ptr<IfcObjectPlacement>& placement, bool onlyRotation = false ) {
        // FIXME: infinite recurision
        if( !placement ) {
            return TMatrix::GetIdentity();
        }
        auto relativeTo = TMatrix::GetIdentity();
        auto result = TMatrix::GetIdentity();

        auto localPlacement = dynamic_pointer_cast<IfcLocalPlacement>( placement );
        if( localPlacement ) {
            if( localPlacement->m_PlacementRelTo ) {
                shared_ptr<IfcObjectPlacement> relativeToPlacement = localPlacement->m_PlacementRelTo;
                relativeTo = ConvertPlacement( relativeToPlacement, onlyRotation );
            }
            const auto& relativePlacement = localPlacement->m_RelativePlacement;
            if( relativePlacement ) {
                if( dynamic_pointer_cast<IfcPlacement>( relativePlacement ) ) {
                    result = ConvertPlacement( dynamic_pointer_cast<IfcPlacement>( relativePlacement ), onlyRotation );
                    TMatrix::Multiply( &result, relativeTo );
                } else {
                    // TODO: Log error
                }
            } else {
                // TODO: Log error
            }
        }

        auto gridPlacement = dynamic_pointer_cast<IfcGridPlacement>( placement );
        if( gridPlacement ) {
            // TODO: Implement
            // NOT IMPLEMENTED
        }

        return result;
    }
    TMatrix ConvertPlacement( const std::shared_ptr<IfcPlacement>& placement, bool onlyRotation = false ) {
        if( !placement ) {
            return TMatrix::GetIdentity();
        }
        if( dynamic_pointer_cast<IfcAxis1Placement>( placement ) ) {
            // TODO: Implement
            // NOT IMPLEMENTED
            return TMatrix::GetIdentity();
        } else if( auto placement2d = dynamic_pointer_cast<IfcAxis2Placement2D>( placement ) ) {
            return ConvertPlacement( placement2d, onlyRotation );
        } else if( auto placement3d = dynamic_pointer_cast<IfcAxis2Placement3D>( placement ) ) {
            return ConvertPlacement( placement3d, onlyRotation );
        }
        // TODO: Log error
        return TMatrix::GetIdentity();
    }
    TMatrix ConvertPlacement( const std::shared_ptr<IfcAxis2Placement>& placement, bool onlyRotation = false ) {
        if( !placement ) {
            return TMatrix::GetIdentity();
        }
        if( dynamic_pointer_cast<IfcAxis1Placement>( placement ) ) {
            // TODO: Implement
            // NOT IMPLEMENTED
            return TMatrix::GetIdentity();
        } else if( auto placement2d = dynamic_pointer_cast<IfcAxis2Placement2D>( placement ) ) {
            return ConvertPlacement( placement2d, onlyRotation );
        } else if( auto placement3d = dynamic_pointer_cast<IfcAxis2Placement3D>( placement ) ) {
            return ConvertPlacement( placement3d, onlyRotation );
        }
        // TODO: Log error
        return TMatrix::GetIdentity();
    }
    TMatrix ConvertTransformationOperator( const shared_ptr<IfcCartesianTransformationOperator>& transformationOperator ) {
        // ENTITY IfcCartesianTransformationOperator ABSTRACT SUPERTYPE OF(ONEOF(IfcCartesianTransformationOperator2D, IfcCartesianTransformationOperator3D))

        if( !transformationOperator ) {
            return TMatrix::GetIdentity();
        }

        auto translate = AVector::New( 0.0, 0.0, 0.0 );
        auto local_x = AVector::New( 1.0, 0.0, 0.0 );
        auto local_y = AVector::New( 0.0, 1.0, 0.0 );
        auto local_z = AVector::New( 0.0, 0.0, 1.0 );

        double scale = 1.0;
        double scale_y = 1.0;
        double scale_z = 1.0;

        shared_ptr<IfcCartesianTransformationOperator2D> trans_operator_2d =
            dynamic_pointer_cast<IfcCartesianTransformationOperator2D>( transformationOperator );
        if( trans_operator_2d ) {
            // ENTITY IfcCartesianTransformationOperator2D SUPERTYPE OF(IfcCartesianTransformationOperator2DnonUniform)
            if( !trans_operator_2d->m_LocalOrigin || trans_operator_2d->m_LocalOrigin->m_size < 2 ) {
                // TODO: Log error
                return TMatrix::GetIdentity();
            }
            double x = trans_operator_2d->m_LocalOrigin->m_Coordinates[ 0 ];
            double y = trans_operator_2d->m_LocalOrigin->m_Coordinates[ 1 ];
            translate = AVector::New( x, y, 0.0 );

            if( trans_operator_2d->m_Scale ) {
                scale = trans_operator_2d->m_Scale->m_value;
            }
            scale_y = scale;
            scale_z = scale;
            if( trans_operator_2d->m_Axis1 && trans_operator_2d->m_Axis2 ) {
                if( trans_operator_2d->m_Axis1->m_DirectionRatios.size() < 2 || trans_operator_2d->m_Axis2->m_DirectionRatios.size() < 2 ) {
                    // TODO: Log error
                    return TMatrix::GetIdentity();
                }

                local_x.x = trans_operator_2d->m_Axis1->m_DirectionRatios[ 0 ]->m_value;
                local_x.y = trans_operator_2d->m_Axis1->m_DirectionRatios[ 1 ]->m_value;

                local_y.x = trans_operator_2d->m_Axis2->m_DirectionRatios[ 0 ]->m_value;
                local_y.y = trans_operator_2d->m_Axis2->m_DirectionRatios[ 1 ]->m_value;
            }

            shared_ptr<IfcCartesianTransformationOperator2DnonUniform> non_uniform =
                dynamic_pointer_cast<IfcCartesianTransformationOperator2DnonUniform>( transformationOperator );
            if( non_uniform && non_uniform->m_Scale2 ) {
                scale_y = non_uniform->m_Scale2->m_value;
            }
        } else {
            // ENTITY IfcCartesianTransformationOperator3D SUPERTYPE OF(IfcCartesianTransformationOperator3DnonUniform)
            shared_ptr<IfcCartesianTransformationOperator3D> trans_operator_3d =
                dynamic_pointer_cast<IfcCartesianTransformationOperator3D>( transformationOperator );
            if( !trans_operator_3d || !trans_operator_3d->m_LocalOrigin || trans_operator_3d->m_LocalOrigin->m_size < 3 ) {
                // TODO: Log error
                return TMatrix::GetIdentity();
            }
            translate.x = trans_operator_3d->m_LocalOrigin->m_Coordinates[ 0 ];
            translate.y = trans_operator_3d->m_LocalOrigin->m_Coordinates[ 1 ];
            translate.z = trans_operator_3d->m_LocalOrigin->m_Coordinates[ 2 ];
            if( trans_operator_3d->m_Scale ) {
                scale = trans_operator_3d->m_Scale->m_value;
            }
            scale_y = scale;
            scale_z = scale;
            if( trans_operator_3d->m_Axis1 && trans_operator_3d->m_Axis2 && trans_operator_3d->m_Axis3 ) {
                const auto& axis1 = trans_operator_3d->m_Axis1;
                const auto& axis2 = trans_operator_3d->m_Axis2;
                const auto& axis3 = trans_operator_3d->m_Axis3;
                if( axis1->m_DirectionRatios.size() < 2 || axis2->m_DirectionRatios.size() < 2 || axis3->m_DirectionRatios.size() < 2 ) {
                    // TODO: Log error
                    return {};
                }
                local_x.x = axis1->m_DirectionRatios[ 0 ]->m_value;
                local_x.y = axis1->m_DirectionRatios[ 1 ]->m_value;
                local_x.z = axis1->m_DirectionRatios[ 2 ]->m_value;

                local_y.x = axis2->m_DirectionRatios[ 0 ]->m_value;
                local_y.y = axis2->m_DirectionRatios[ 1 ]->m_value;
                local_y.z = axis2->m_DirectionRatios[ 2 ]->m_value;

                local_z.x = axis3->m_DirectionRatios[ 0 ]->m_value;
                local_z.y = axis3->m_DirectionRatios[ 1 ]->m_value;
                local_z.z = axis3->m_DirectionRatios[ 2 ]->m_value;
            }

            shared_ptr<IfcCartesianTransformationOperator3DnonUniform> non_uniform =
                dynamic_pointer_cast<IfcCartesianTransformationOperator3DnonUniform>( transformationOperator );
            if( non_uniform ) {
                if( non_uniform->m_Scale2 ) {
                    scale_y = non_uniform->m_Scale2->m_value;
                }
                if( non_uniform->m_Scale3 ) {
                    scale_z = non_uniform->m_Scale3->m_value;
                }
            }
        }
        AVector::Normalize( &local_x );
        AVector::Normalize( &local_y );
        AVector::Normalize( &local_z );

        return TMatrix::GetMultiplied( TMatrix::CreateFromAxises( local_x, local_y, local_z, translate ), TMatrix::GetScale( scale, scale_y, scale_z ) );
    }
};

}