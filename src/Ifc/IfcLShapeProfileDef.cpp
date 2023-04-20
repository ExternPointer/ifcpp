/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcAxis2Placement2D.h"
#include "ifcpp/Ifc/IfcExternalReferenceRelationship.h"
#include "ifcpp/Ifc/IfcLShapeProfileDef.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcNonNegativeLengthMeasure.h"
#include "ifcpp/Ifc/IfcPlaneAngleMeasure.h"
#include "ifcpp/Ifc/IfcPositiveLengthMeasure.h"
#include "ifcpp/Ifc/IfcProfileProperties.h"
#include "ifcpp/Ifc/IfcProfileTypeEnum.h"

// ENTITY IfcLShapeProfileDef 
IFC4X3::IfcLShapeProfileDef::IfcLShapeProfileDef( int tag ) { m_tag = tag; }
void IFC4X3::IfcLShapeProfileDef::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCLSHAPEPROFILEDEF" << "(";
	if( m_ProfileType ) { m_ProfileType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ProfileName ) { m_ProfileName->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Position ) { stream << "#" << m_Position->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Depth ) { m_Depth->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Width ) { m_Width->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Thickness ) { m_Thickness->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_FilletRadius ) { m_FilletRadius->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_EdgeRadius ) { m_EdgeRadius->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_LegSlope ) { m_LegSlope->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcLShapeProfileDef::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcLShapeProfileDef::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 9 ){ std::stringstream err; err << "Wrong parameter count for entity IfcLShapeProfileDef, expecting 9, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_ProfileType = IfcProfileTypeEnum::createObjectFromSTEP( args[0], map, errorStream );
	m_ProfileName = IfcLabel::createObjectFromSTEP( args[1], map, errorStream );
	readEntityReference( args[2], m_Position, map, errorStream );
	m_Depth = IfcPositiveLengthMeasure::createObjectFromSTEP( args[3], map, errorStream );
	m_Width = IfcPositiveLengthMeasure::createObjectFromSTEP( args[4], map, errorStream );
	m_Thickness = IfcPositiveLengthMeasure::createObjectFromSTEP( args[5], map, errorStream );
	m_FilletRadius = IfcNonNegativeLengthMeasure::createObjectFromSTEP( args[6], map, errorStream );
	m_EdgeRadius = IfcNonNegativeLengthMeasure::createObjectFromSTEP( args[7], map, errorStream );
	m_LegSlope = IfcPlaneAngleMeasure::createObjectFromSTEP( args[8], map, errorStream );
}
void IFC4X3::IfcLShapeProfileDef::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcParameterizedProfileDef::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "Depth", m_Depth ) );
	vec_attributes.emplace_back( std::make_pair( "Width", m_Width ) );
	vec_attributes.emplace_back( std::make_pair( "Thickness", m_Thickness ) );
	vec_attributes.emplace_back( std::make_pair( "FilletRadius", m_FilletRadius ) );
	vec_attributes.emplace_back( std::make_pair( "EdgeRadius", m_EdgeRadius ) );
	vec_attributes.emplace_back( std::make_pair( "LegSlope", m_LegSlope ) );
}
void IFC4X3::IfcLShapeProfileDef::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcParameterizedProfileDef::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcLShapeProfileDef::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcParameterizedProfileDef::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcLShapeProfileDef::unlinkFromInverseCounterparts()
{
	IfcParameterizedProfileDef::unlinkFromInverseCounterparts();
}