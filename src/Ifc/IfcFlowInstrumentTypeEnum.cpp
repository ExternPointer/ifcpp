/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcFlowInstrumentTypeEnum.h"

// TYPE IfcFlowInstrumentTypeEnum = ENUMERATION OF	(AMMETER	,COMBINED	,FREQUENCYMETER	,PHASEANGLEMETER	,POWERFACTORMETER	,PRESSUREGAUGE	,THERMOMETER	,VOLTMETER	,VOLTMETER_PEAK	,VOLTMETER_RMS	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcFlowInstrumentTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCFLOWINSTRUMENTTYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_AMMETER:	stream << ".AMMETER."; break;
		case ENUM_COMBINED:	stream << ".COMBINED."; break;
		case ENUM_FREQUENCYMETER:	stream << ".FREQUENCYMETER."; break;
		case ENUM_PHASEANGLEMETER:	stream << ".PHASEANGLEMETER."; break;
		case ENUM_POWERFACTORMETER:	stream << ".POWERFACTORMETER."; break;
		case ENUM_PRESSUREGAUGE:	stream << ".PRESSUREGAUGE."; break;
		case ENUM_THERMOMETER:	stream << ".THERMOMETER."; break;
		case ENUM_VOLTMETER:	stream << ".VOLTMETER."; break;
		case ENUM_VOLTMETER_PEAK:	stream << ".VOLTMETER_PEAK."; break;
		case ENUM_VOLTMETER_RMS:	stream << ".VOLTMETER_RMS."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcFlowInstrumentTypeEnum> IFC4X3::IfcFlowInstrumentTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcFlowInstrumentTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcFlowInstrumentTypeEnum>(); }
	shared_ptr<IfcFlowInstrumentTypeEnum> type_object( new IfcFlowInstrumentTypeEnum() );
	if( std_iequal( arg, ".AMMETER." ) )
	{
		type_object->m_enum = IfcFlowInstrumentTypeEnum::ENUM_AMMETER;
	}
	else if( std_iequal( arg, ".COMBINED." ) )
	{
		type_object->m_enum = IfcFlowInstrumentTypeEnum::ENUM_COMBINED;
	}
	else if( std_iequal( arg, ".FREQUENCYMETER." ) )
	{
		type_object->m_enum = IfcFlowInstrumentTypeEnum::ENUM_FREQUENCYMETER;
	}
	else if( std_iequal( arg, ".PHASEANGLEMETER." ) )
	{
		type_object->m_enum = IfcFlowInstrumentTypeEnum::ENUM_PHASEANGLEMETER;
	}
	else if( std_iequal( arg, ".POWERFACTORMETER." ) )
	{
		type_object->m_enum = IfcFlowInstrumentTypeEnum::ENUM_POWERFACTORMETER;
	}
	else if( std_iequal( arg, ".PRESSUREGAUGE." ) )
	{
		type_object->m_enum = IfcFlowInstrumentTypeEnum::ENUM_PRESSUREGAUGE;
	}
	else if( std_iequal( arg, ".THERMOMETER." ) )
	{
		type_object->m_enum = IfcFlowInstrumentTypeEnum::ENUM_THERMOMETER;
	}
	else if( std_iequal( arg, ".VOLTMETER." ) )
	{
		type_object->m_enum = IfcFlowInstrumentTypeEnum::ENUM_VOLTMETER;
	}
	else if( std_iequal( arg, ".VOLTMETER_PEAK." ) )
	{
		type_object->m_enum = IfcFlowInstrumentTypeEnum::ENUM_VOLTMETER_PEAK;
	}
	else if( std_iequal( arg, ".VOLTMETER_RMS." ) )
	{
		type_object->m_enum = IfcFlowInstrumentTypeEnum::ENUM_VOLTMETER_RMS;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcFlowInstrumentTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcFlowInstrumentTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
