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
#include "ifcpp/Ifc/IfcGeometricRepresentationContext.h"
#include "ifcpp/Ifc/IfcGeometricRepresentationSubContext.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcReal.h"
#include "ifcpp/Ifc/IfcRepresentation.h"

// ENTITY IfcGeometricRepresentationContext 
IFC4X3::IfcGeometricRepresentationContext::IfcGeometricRepresentationContext( int tag ) { m_tag = tag; }
void IFC4X3::IfcGeometricRepresentationContext::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCGEOMETRICREPRESENTATIONCONTEXT" << "(";
	if( m_ContextIdentifier ) { m_ContextIdentifier->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ContextType ) { m_ContextType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_CoordinateSpaceDimension ) { m_CoordinateSpaceDimension->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Precision ) { m_Precision->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_WorldCoordinateSystem ) { m_WorldCoordinateSystem->getStepParameter( stream, true ); } else { stream << "$" ; }
	stream << ",";
	if( m_TrueNorth ) { stream << "#" << m_TrueNorth->m_tag; } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcGeometricRepresentationContext::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcGeometricRepresentationContext::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 6 ){ std::stringstream err; err << "Wrong parameter count for entity IfcGeometricRepresentationContext, expecting 6, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_ContextIdentifier = IfcLabel::createObjectFromSTEP( args[0], map, errorStream );
	m_ContextType = IfcLabel::createObjectFromSTEP( args[1], map, errorStream );
	m_CoordinateSpaceDimension = IfcDimensionCount::createObjectFromSTEP( args[2], map, errorStream );
	m_Precision = IfcReal::createObjectFromSTEP( args[3], map, errorStream );
	m_WorldCoordinateSystem = IfcAxis2Placement::createObjectFromSTEP( args[4], map, errorStream );
	readEntityReference( args[5], m_TrueNorth, map, errorStream );
}
void IFC4X3::IfcGeometricRepresentationContext::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcRepresentationContext::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "CoordinateSpaceDimension", m_CoordinateSpaceDimension ) );
	vec_attributes.emplace_back( std::make_pair( "Precision", m_Precision ) );
	vec_attributes.emplace_back( std::make_pair( "WorldCoordinateSystem", m_WorldCoordinateSystem ) );
	vec_attributes.emplace_back( std::make_pair( "TrueNorth", m_TrueNorth ) );
}
void IFC4X3::IfcGeometricRepresentationContext::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcRepresentationContext::getAttributesInverse( vec_attributes_inverse );
	shared_ptr<AttributeObjectVector> HasSubContexts_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_HasSubContexts_inverse.size(); ++i )
	{
		if( !m_HasSubContexts_inverse[i].expired() )
		{
			HasSubContexts_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcGeometricRepresentationSubContext>( m_HasSubContexts_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "HasSubContexts_inverse", HasSubContexts_inverse_vec_obj ) );
	shared_ptr<AttributeObjectVector> HasCoordinateOperation_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_HasCoordinateOperation_inverse.size(); ++i )
	{
		if( !m_HasCoordinateOperation_inverse[i].expired() )
		{
			HasCoordinateOperation_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcCoordinateOperation>( m_HasCoordinateOperation_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "HasCoordinateOperation_inverse", HasCoordinateOperation_inverse_vec_obj ) );
}
void IFC4X3::IfcGeometricRepresentationContext::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcRepresentationContext::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcGeometricRepresentationContext::unlinkFromInverseCounterparts()
{
	IfcRepresentationContext::unlinkFromInverseCounterparts();
}
