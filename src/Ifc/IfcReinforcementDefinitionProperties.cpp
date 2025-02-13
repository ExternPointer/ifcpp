/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcOwnerHistory.h"
#include "ifcpp/Ifc/IfcReinforcementDefinitionProperties.h"
#include "ifcpp/Ifc/IfcRelAssociates.h"
#include "ifcpp/Ifc/IfcRelDeclares.h"
#include "ifcpp/Ifc/IfcRelDefinesByProperties.h"
#include "ifcpp/Ifc/IfcRelDefinesByTemplate.h"
#include "ifcpp/Ifc/IfcSectionReinforcementProperties.h"
#include "ifcpp/Ifc/IfcText.h"
#include "ifcpp/Ifc/IfcTypeObject.h"

// ENTITY IfcReinforcementDefinitionProperties 
IFC4X3::IfcReinforcementDefinitionProperties::IfcReinforcementDefinitionProperties( int tag ) { m_tag = tag; }
void IFC4X3::IfcReinforcementDefinitionProperties::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCREINFORCEMENTDEFINITIONPROPERTIES" << "(";
	if( m_GlobalId ) { m_GlobalId->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OwnerHistory ) { stream << "#" << m_OwnerHistory->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_DefinitionType ) { m_DefinitionType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_ReinforcementSectionDefinitions );
	stream << ");";
}
void IFC4X3::IfcReinforcementDefinitionProperties::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcReinforcementDefinitionProperties::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 6 ){ std::stringstream err; err << "Wrong parameter count for entity IfcReinforcementDefinitionProperties, expecting 6, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	m_DefinitionType = IfcLabel::createObjectFromSTEP( args[4], map, errorStream );
	readEntityReferenceList( args[5], m_ReinforcementSectionDefinitions, map, errorStream );
}
void IFC4X3::IfcReinforcementDefinitionProperties::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcPreDefinedPropertySet::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "DefinitionType", m_DefinitionType ) );
	shared_ptr<AttributeObjectVector> ReinforcementSectionDefinitions_vec_object( new AttributeObjectVector() );
	std::copy( m_ReinforcementSectionDefinitions.begin(), m_ReinforcementSectionDefinitions.end(), std::back_inserter( ReinforcementSectionDefinitions_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "ReinforcementSectionDefinitions", ReinforcementSectionDefinitions_vec_object ) );
}
void IFC4X3::IfcReinforcementDefinitionProperties::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcPreDefinedPropertySet::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcReinforcementDefinitionProperties::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcPreDefinedPropertySet::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcReinforcementDefinitionProperties::unlinkFromInverseCounterparts()
{
	IfcPreDefinedPropertySet::unlinkFromInverseCounterparts();
}
