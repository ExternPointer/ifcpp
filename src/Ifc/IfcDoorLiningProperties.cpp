/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcDoorLiningProperties.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcLengthMeasure.h"
#include "ifcpp/Ifc/IfcNonNegativeLengthMeasure.h"
#include "ifcpp/Ifc/IfcOwnerHistory.h"
#include "ifcpp/Ifc/IfcPositiveLengthMeasure.h"
#include "ifcpp/Ifc/IfcRelAssociates.h"
#include "ifcpp/Ifc/IfcRelDeclares.h"
#include "ifcpp/Ifc/IfcRelDefinesByProperties.h"
#include "ifcpp/Ifc/IfcRelDefinesByTemplate.h"
#include "ifcpp/Ifc/IfcShapeAspect.h"
#include "ifcpp/Ifc/IfcText.h"
#include "ifcpp/Ifc/IfcTypeObject.h"

// ENTITY IfcDoorLiningProperties 
IFC4X3::IfcDoorLiningProperties::IfcDoorLiningProperties( int tag ) { m_tag = tag; }
void IFC4X3::IfcDoorLiningProperties::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCDOORLININGPROPERTIES" << "(";
	if( m_GlobalId ) { m_GlobalId->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OwnerHistory ) { stream << "#" << m_OwnerHistory->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_LiningDepth ) { m_LiningDepth->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_LiningThickness ) { m_LiningThickness->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ThresholdDepth ) { m_ThresholdDepth->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ThresholdThickness ) { m_ThresholdThickness->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_TransomThickness ) { m_TransomThickness->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_TransomOffset ) { m_TransomOffset->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_LiningOffset ) { m_LiningOffset->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ThresholdOffset ) { m_ThresholdOffset->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_CasingThickness ) { m_CasingThickness->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_CasingDepth ) { m_CasingDepth->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ShapeAspectStyle ) { stream << "#" << m_ShapeAspectStyle->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_LiningToPanelOffsetX ) { m_LiningToPanelOffsetX->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_LiningToPanelOffsetY ) { m_LiningToPanelOffsetY->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcDoorLiningProperties::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcDoorLiningProperties::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 17 ){ std::stringstream err; err << "Wrong parameter count for entity IfcDoorLiningProperties, expecting 17, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	m_LiningDepth = IfcPositiveLengthMeasure::createObjectFromSTEP( args[4], map, errorStream );
	m_LiningThickness = IfcNonNegativeLengthMeasure::createObjectFromSTEP( args[5], map, errorStream );
	m_ThresholdDepth = IfcPositiveLengthMeasure::createObjectFromSTEP( args[6], map, errorStream );
	m_ThresholdThickness = IfcNonNegativeLengthMeasure::createObjectFromSTEP( args[7], map, errorStream );
	m_TransomThickness = IfcNonNegativeLengthMeasure::createObjectFromSTEP( args[8], map, errorStream );
	m_TransomOffset = IfcLengthMeasure::createObjectFromSTEP( args[9], map, errorStream );
	m_LiningOffset = IfcLengthMeasure::createObjectFromSTEP( args[10], map, errorStream );
	m_ThresholdOffset = IfcLengthMeasure::createObjectFromSTEP( args[11], map, errorStream );
	m_CasingThickness = IfcPositiveLengthMeasure::createObjectFromSTEP( args[12], map, errorStream );
	m_CasingDepth = IfcPositiveLengthMeasure::createObjectFromSTEP( args[13], map, errorStream );
	readEntityReference( args[14], m_ShapeAspectStyle, map, errorStream );
	m_LiningToPanelOffsetX = IfcLengthMeasure::createObjectFromSTEP( args[15], map, errorStream );
	m_LiningToPanelOffsetY = IfcLengthMeasure::createObjectFromSTEP( args[16], map, errorStream );
}
void IFC4X3::IfcDoorLiningProperties::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcPreDefinedPropertySet::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "LiningDepth", m_LiningDepth ) );
	vec_attributes.emplace_back( std::make_pair( "LiningThickness", m_LiningThickness ) );
	vec_attributes.emplace_back( std::make_pair( "ThresholdDepth", m_ThresholdDepth ) );
	vec_attributes.emplace_back( std::make_pair( "ThresholdThickness", m_ThresholdThickness ) );
	vec_attributes.emplace_back( std::make_pair( "TransomThickness", m_TransomThickness ) );
	vec_attributes.emplace_back( std::make_pair( "TransomOffset", m_TransomOffset ) );
	vec_attributes.emplace_back( std::make_pair( "LiningOffset", m_LiningOffset ) );
	vec_attributes.emplace_back( std::make_pair( "ThresholdOffset", m_ThresholdOffset ) );
	vec_attributes.emplace_back( std::make_pair( "CasingThickness", m_CasingThickness ) );
	vec_attributes.emplace_back( std::make_pair( "CasingDepth", m_CasingDepth ) );
	vec_attributes.emplace_back( std::make_pair( "ShapeAspectStyle", m_ShapeAspectStyle ) );
	vec_attributes.emplace_back( std::make_pair( "LiningToPanelOffsetX", m_LiningToPanelOffsetX ) );
	vec_attributes.emplace_back( std::make_pair( "LiningToPanelOffsetY", m_LiningToPanelOffsetY ) );
}
void IFC4X3::IfcDoorLiningProperties::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcPreDefinedPropertySet::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcDoorLiningProperties::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcPreDefinedPropertySet::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcDoorLiningProperties::unlinkFromInverseCounterparts()
{
	IfcPreDefinedPropertySet::unlinkFromInverseCounterparts();
}
