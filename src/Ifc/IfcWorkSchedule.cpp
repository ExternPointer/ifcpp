/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcDateTime.h"
#include "ifcpp/Ifc/IfcDuration.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcIdentifier.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcOwnerHistory.h"
#include "ifcpp/Ifc/IfcPerson.h"
#include "ifcpp/Ifc/IfcRelAggregates.h"
#include "ifcpp/Ifc/IfcRelAssigns.h"
#include "ifcpp/Ifc/IfcRelAssignsToControl.h"
#include "ifcpp/Ifc/IfcRelAssociates.h"
#include "ifcpp/Ifc/IfcRelDeclares.h"
#include "ifcpp/Ifc/IfcRelDefinesByObject.h"
#include "ifcpp/Ifc/IfcRelDefinesByProperties.h"
#include "ifcpp/Ifc/IfcRelDefinesByType.h"
#include "ifcpp/Ifc/IfcRelNests.h"
#include "ifcpp/Ifc/IfcText.h"
#include "ifcpp/Ifc/IfcWorkSchedule.h"
#include "ifcpp/Ifc/IfcWorkScheduleTypeEnum.h"

// ENTITY IfcWorkSchedule 
IFC4X3::IfcWorkSchedule::IfcWorkSchedule( int tag ) { m_tag = tag; }
void IFC4X3::IfcWorkSchedule::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCWORKSCHEDULE" << "(";
	if( m_GlobalId ) { m_GlobalId->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OwnerHistory ) { stream << "#" << m_OwnerHistory->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ObjectType ) { m_ObjectType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Identification ) { m_Identification->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_CreationDate ) { m_CreationDate->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_Creators );
	stream << ",";
	if( m_Purpose ) { m_Purpose->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Duration ) { m_Duration->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_TotalFloat ) { m_TotalFloat->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_StartTime ) { m_StartTime->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_FinishTime ) { m_FinishTime->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_PredefinedType ) { m_PredefinedType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcWorkSchedule::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcWorkSchedule::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 14 ){ std::stringstream err; err << "Wrong parameter count for entity IfcWorkSchedule, expecting 14, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	m_ObjectType = IfcLabel::createObjectFromSTEP( args[4], map, errorStream );
	m_Identification = IfcIdentifier::createObjectFromSTEP( args[5], map, errorStream );
	m_CreationDate = IfcDateTime::createObjectFromSTEP( args[6], map, errorStream );
	readEntityReferenceList( args[7], m_Creators, map, errorStream );
	m_Purpose = IfcLabel::createObjectFromSTEP( args[8], map, errorStream );
	m_Duration = IfcDuration::createObjectFromSTEP( args[9], map, errorStream );
	m_TotalFloat = IfcDuration::createObjectFromSTEP( args[10], map, errorStream );
	m_StartTime = IfcDateTime::createObjectFromSTEP( args[11], map, errorStream );
	m_FinishTime = IfcDateTime::createObjectFromSTEP( args[12], map, errorStream );
	m_PredefinedType = IfcWorkScheduleTypeEnum::createObjectFromSTEP( args[13], map, errorStream );
}
void IFC4X3::IfcWorkSchedule::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcWorkControl::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "PredefinedType", m_PredefinedType ) );
}
void IFC4X3::IfcWorkSchedule::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcWorkControl::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcWorkSchedule::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcWorkControl::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcWorkSchedule::unlinkFromInverseCounterparts()
{
	IfcWorkControl::unlinkFromInverseCounterparts();
}
