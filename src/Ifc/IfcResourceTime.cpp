/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcBoolean.h"
#include "ifcpp/Ifc/IfcDataOriginEnum.h"
#include "ifcpp/Ifc/IfcDateTime.h"
#include "ifcpp/Ifc/IfcDuration.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcPositiveRatioMeasure.h"
#include "ifcpp/Ifc/IfcResourceTime.h"

// ENTITY IfcResourceTime 
IFC4X3::IfcResourceTime::IfcResourceTime( int tag ) { m_tag = tag; }
void IFC4X3::IfcResourceTime::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCRESOURCETIME" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_DataOrigin ) { m_DataOrigin->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_UserDefinedDataOrigin ) { m_UserDefinedDataOrigin->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ScheduleWork ) { m_ScheduleWork->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ScheduleUsage ) { m_ScheduleUsage->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ScheduleStart ) { m_ScheduleStart->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ScheduleFinish ) { m_ScheduleFinish->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ScheduleContour ) { m_ScheduleContour->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_LevelingDelay ) { m_LevelingDelay->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_IsOverAllocated ) { m_IsOverAllocated->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_StatusTime ) { m_StatusTime->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ActualWork ) { m_ActualWork->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ActualUsage ) { m_ActualUsage->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ActualStart ) { m_ActualStart->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ActualFinish ) { m_ActualFinish->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_RemainingWork ) { m_RemainingWork->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_RemainingUsage ) { m_RemainingUsage->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Completion ) { m_Completion->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcResourceTime::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcResourceTime::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 18 ){ std::stringstream err; err << "Wrong parameter count for entity IfcResourceTime, expecting 18, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Name = IfcLabel::createObjectFromSTEP( args[0], map, errorStream );
	m_DataOrigin = IfcDataOriginEnum::createObjectFromSTEP( args[1], map, errorStream );
	m_UserDefinedDataOrigin = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_ScheduleWork = IfcDuration::createObjectFromSTEP( args[3], map, errorStream );
	m_ScheduleUsage = IfcPositiveRatioMeasure::createObjectFromSTEP( args[4], map, errorStream );
	m_ScheduleStart = IfcDateTime::createObjectFromSTEP( args[5], map, errorStream );
	m_ScheduleFinish = IfcDateTime::createObjectFromSTEP( args[6], map, errorStream );
	m_ScheduleContour = IfcLabel::createObjectFromSTEP( args[7], map, errorStream );
	m_LevelingDelay = IfcDuration::createObjectFromSTEP( args[8], map, errorStream );
	m_IsOverAllocated = IfcBoolean::createObjectFromSTEP( args[9], map, errorStream );
	m_StatusTime = IfcDateTime::createObjectFromSTEP( args[10], map, errorStream );
	m_ActualWork = IfcDuration::createObjectFromSTEP( args[11], map, errorStream );
	m_ActualUsage = IfcPositiveRatioMeasure::createObjectFromSTEP( args[12], map, errorStream );
	m_ActualStart = IfcDateTime::createObjectFromSTEP( args[13], map, errorStream );
	m_ActualFinish = IfcDateTime::createObjectFromSTEP( args[14], map, errorStream );
	m_RemainingWork = IfcDuration::createObjectFromSTEP( args[15], map, errorStream );
	m_RemainingUsage = IfcPositiveRatioMeasure::createObjectFromSTEP( args[16], map, errorStream );
	m_Completion = IfcPositiveRatioMeasure::createObjectFromSTEP( args[17], map, errorStream );
}
void IFC4X3::IfcResourceTime::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcSchedulingTime::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "ScheduleWork", m_ScheduleWork ) );
	vec_attributes.emplace_back( std::make_pair( "ScheduleUsage", m_ScheduleUsage ) );
	vec_attributes.emplace_back( std::make_pair( "ScheduleStart", m_ScheduleStart ) );
	vec_attributes.emplace_back( std::make_pair( "ScheduleFinish", m_ScheduleFinish ) );
	vec_attributes.emplace_back( std::make_pair( "ScheduleContour", m_ScheduleContour ) );
	vec_attributes.emplace_back( std::make_pair( "LevelingDelay", m_LevelingDelay ) );
	vec_attributes.emplace_back( std::make_pair( "IsOverAllocated", m_IsOverAllocated ) );
	vec_attributes.emplace_back( std::make_pair( "StatusTime", m_StatusTime ) );
	vec_attributes.emplace_back( std::make_pair( "ActualWork", m_ActualWork ) );
	vec_attributes.emplace_back( std::make_pair( "ActualUsage", m_ActualUsage ) );
	vec_attributes.emplace_back( std::make_pair( "ActualStart", m_ActualStart ) );
	vec_attributes.emplace_back( std::make_pair( "ActualFinish", m_ActualFinish ) );
	vec_attributes.emplace_back( std::make_pair( "RemainingWork", m_RemainingWork ) );
	vec_attributes.emplace_back( std::make_pair( "RemainingUsage", m_RemainingUsage ) );
	vec_attributes.emplace_back( std::make_pair( "Completion", m_Completion ) );
}
void IFC4X3::IfcResourceTime::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcSchedulingTime::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcResourceTime::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcSchedulingTime::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcResourceTime::unlinkFromInverseCounterparts()
{
	IfcSchedulingTime::unlinkFromInverseCounterparts();
}
