/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcDuctSegmentType.h"
#include "ifcpp/Ifc/IfcDuctSegmentTypeEnum.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcIdentifier.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcOwnerHistory.h"
#include "ifcpp/Ifc/IfcPropertySetDefinition.h"
#include "ifcpp/Ifc/IfcRelAggregates.h"
#include "ifcpp/Ifc/IfcRelAssigns.h"
#include "ifcpp/Ifc/IfcRelAssignsToProduct.h"
#include "ifcpp/Ifc/IfcRelAssociates.h"
#include "ifcpp/Ifc/IfcRelDeclares.h"
#include "ifcpp/Ifc/IfcRelDefinesByType.h"
#include "ifcpp/Ifc/IfcRelNests.h"
#include "ifcpp/Ifc/IfcRepresentationMap.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcDuctSegmentType 
IFC4X3::IfcDuctSegmentType::IfcDuctSegmentType( int tag ) { m_tag = tag; }
void IFC4X3::IfcDuctSegmentType::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCDUCTSEGMENTTYPE" << "(";
	if( m_GlobalId ) { m_GlobalId->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OwnerHistory ) { stream << "#" << m_OwnerHistory->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ApplicableOccurrence ) { m_ApplicableOccurrence->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_HasPropertySets );
	stream << ",";
	writeEntityList( stream, m_RepresentationMaps );
	stream << ",";
	if( m_Tag ) { m_Tag->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ElementType ) { m_ElementType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_PredefinedType ) { m_PredefinedType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcDuctSegmentType::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcDuctSegmentType::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 10 ){ std::stringstream err; err << "Wrong parameter count for entity IfcDuctSegmentType, expecting 10, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	m_ApplicableOccurrence = IfcIdentifier::createObjectFromSTEP( args[4], map, errorStream );
	readEntityReferenceList( args[5], m_HasPropertySets, map, errorStream );
	readEntityReferenceList( args[6], m_RepresentationMaps, map, errorStream );
	m_Tag = IfcLabel::createObjectFromSTEP( args[7], map, errorStream );
	m_ElementType = IfcLabel::createObjectFromSTEP( args[8], map, errorStream );
	m_PredefinedType = IfcDuctSegmentTypeEnum::createObjectFromSTEP( args[9], map, errorStream );
}
void IFC4X3::IfcDuctSegmentType::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcFlowSegmentType::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "PredefinedType", m_PredefinedType ) );
}
void IFC4X3::IfcDuctSegmentType::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcFlowSegmentType::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcDuctSegmentType::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcFlowSegmentType::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcDuctSegmentType::unlinkFromInverseCounterparts()
{
	IfcFlowSegmentType::unlinkFromInverseCounterparts();
}