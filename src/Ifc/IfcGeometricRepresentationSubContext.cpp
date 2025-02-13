/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcAxis2Placement.h"
#include "ifcpp/Ifc/IfcCoordinateOperation.h"
#include "ifcpp/Ifc/IfcDimensionCount.h"
#include "ifcpp/Ifc/IfcDirection.h"
#include "ifcpp/Ifc/IfcGeometricProjectionEnum.h"
#include "ifcpp/Ifc/IfcGeometricRepresentationContext.h"
#include "ifcpp/Ifc/IfcGeometricRepresentationSubContext.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcPositiveRatioMeasure.h"
#include "ifcpp/Ifc/IfcReal.h"
#include "ifcpp/Ifc/IfcRepresentation.h"

// ENTITY IfcGeometricRepresentationSubContext 
IFC4X3::IfcGeometricRepresentationSubContext::IfcGeometricRepresentationSubContext( int tag ) { m_tag = tag; }
void IFC4X3::IfcGeometricRepresentationSubContext::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCGEOMETRICREPRESENTATIONSUBCONTEXT" << "(";
	if( m_ContextIdentifier ) { m_ContextIdentifier->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ContextType ) { m_ContextType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_CoordinateSpaceDimension ) { m_CoordinateSpaceDimension->getStepParameter( stream ); } else { stream << "*"; }
	stream << ",";
	if( m_Precision ) { m_Precision->getStepParameter( stream ); } else { stream << "*"; }
	stream << ",";
	if( m_WorldCoordinateSystem ) { m_WorldCoordinateSystem->getStepParameter( stream, true ); } else { stream << "*" ; }
	stream << ",";
	if( m_TrueNorth ) { stream << "#" << m_TrueNorth->m_tag; } else { stream << "*"; }
	stream << ",";
	if( m_ParentContext ) { stream << "#" << m_ParentContext->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_TargetScale ) { m_TargetScale->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_TargetView ) { m_TargetView->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_UserDefinedTargetView ) { m_UserDefinedTargetView->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcGeometricRepresentationSubContext::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcGeometricRepresentationSubContext::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 10 ){ std::stringstream err; err << "Wrong parameter count for entity IfcGeometricRepresentationSubContext, expecting 10, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_ContextIdentifier = IfcLabel::createObjectFromSTEP( args[0], map, errorStream );
	m_ContextType = IfcLabel::createObjectFromSTEP( args[1], map, errorStream );
	m_CoordinateSpaceDimension = IfcDimensionCount::createObjectFromSTEP( args[2], map, errorStream );
	m_Precision = IfcReal::createObjectFromSTEP( args[3], map, errorStream );
	m_WorldCoordinateSystem = IfcAxis2Placement::createObjectFromSTEP( args[4], map, errorStream );
	readEntityReference( args[5], m_TrueNorth, map, errorStream );
	readEntityReference( args[6], m_ParentContext, map, errorStream );
	m_TargetScale = IfcPositiveRatioMeasure::createObjectFromSTEP( args[7], map, errorStream );
	m_TargetView = IfcGeometricProjectionEnum::createObjectFromSTEP( args[8], map, errorStream );
	m_UserDefinedTargetView = IfcLabel::createObjectFromSTEP( args[9], map, errorStream );
}
void IFC4X3::IfcGeometricRepresentationSubContext::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcGeometricRepresentationContext::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "ParentContext", m_ParentContext ) );
	vec_attributes.emplace_back( std::make_pair( "TargetScale", m_TargetScale ) );
	vec_attributes.emplace_back( std::make_pair( "TargetView", m_TargetView ) );
	vec_attributes.emplace_back( std::make_pair( "UserDefinedTargetView", m_UserDefinedTargetView ) );
}
void IFC4X3::IfcGeometricRepresentationSubContext::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcGeometricRepresentationContext::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcGeometricRepresentationSubContext::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcGeometricRepresentationContext::setInverseCounterparts( ptr_self_entity );
	shared_ptr<IfcGeometricRepresentationSubContext> ptr_self = dynamic_pointer_cast<IfcGeometricRepresentationSubContext>( ptr_self_entity );
	if( !ptr_self ) { throw BuildingException( "IfcGeometricRepresentationSubContext::setInverseCounterparts: type mismatch" ); }
	if( m_ParentContext )
	{
		m_ParentContext->m_HasSubContexts_inverse.emplace_back( ptr_self );
	}
}
void IFC4X3::IfcGeometricRepresentationSubContext::unlinkFromInverseCounterparts()
{
	IfcGeometricRepresentationContext::unlinkFromInverseCounterparts();
	if( m_ParentContext )
	{
		std::vector<weak_ptr<IfcGeometricRepresentationSubContext> >& HasSubContexts_inverse = m_ParentContext->m_HasSubContexts_inverse;
		for( auto it_HasSubContexts_inverse = HasSubContexts_inverse.begin(); it_HasSubContexts_inverse != HasSubContexts_inverse.end(); )
		{
			weak_ptr<IfcGeometricRepresentationSubContext> self_candidate_weak = *it_HasSubContexts_inverse;
			if( self_candidate_weak.expired() )
			{
				++it_HasSubContexts_inverse;
				continue;
			}
			shared_ptr<IfcGeometricRepresentationSubContext> self_candidate( *it_HasSubContexts_inverse );
			if( self_candidate.get() == this )
			{
				it_HasSubContexts_inverse= HasSubContexts_inverse.erase( it_HasSubContexts_inverse );
			}
			else
			{
				++it_HasSubContexts_inverse;
			}
		}
	}
}
