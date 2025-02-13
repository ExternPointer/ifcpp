/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcRampTypeEnum.h"

// TYPE IfcRampTypeEnum = ENUMERATION OF	(HALF_TURN_RAMP	,QUARTER_TURN_RAMP	,SPIRAL_RAMP	,STRAIGHT_RUN_RAMP	,TWO_QUARTER_TURN_RAMP	,TWO_STRAIGHT_RUN_RAMP	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcRampTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCRAMPTYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_HALF_TURN_RAMP:	stream << ".HALF_TURN_RAMP."; break;
		case ENUM_QUARTER_TURN_RAMP:	stream << ".QUARTER_TURN_RAMP."; break;
		case ENUM_SPIRAL_RAMP:	stream << ".SPIRAL_RAMP."; break;
		case ENUM_STRAIGHT_RUN_RAMP:	stream << ".STRAIGHT_RUN_RAMP."; break;
		case ENUM_TWO_QUARTER_TURN_RAMP:	stream << ".TWO_QUARTER_TURN_RAMP."; break;
		case ENUM_TWO_STRAIGHT_RUN_RAMP:	stream << ".TWO_STRAIGHT_RUN_RAMP."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcRampTypeEnum> IFC4X3::IfcRampTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcRampTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcRampTypeEnum>(); }
	shared_ptr<IfcRampTypeEnum> type_object( new IfcRampTypeEnum() );
	if( std_iequal( arg, ".HALF_TURN_RAMP." ) )
	{
		type_object->m_enum = IfcRampTypeEnum::ENUM_HALF_TURN_RAMP;
	}
	else if( std_iequal( arg, ".QUARTER_TURN_RAMP." ) )
	{
		type_object->m_enum = IfcRampTypeEnum::ENUM_QUARTER_TURN_RAMP;
	}
	else if( std_iequal( arg, ".SPIRAL_RAMP." ) )
	{
		type_object->m_enum = IfcRampTypeEnum::ENUM_SPIRAL_RAMP;
	}
	else if( std_iequal( arg, ".STRAIGHT_RUN_RAMP." ) )
	{
		type_object->m_enum = IfcRampTypeEnum::ENUM_STRAIGHT_RUN_RAMP;
	}
	else if( std_iequal( arg, ".TWO_QUARTER_TURN_RAMP." ) )
	{
		type_object->m_enum = IfcRampTypeEnum::ENUM_TWO_QUARTER_TURN_RAMP;
	}
	else if( std_iequal( arg, ".TWO_STRAIGHT_RUN_RAMP." ) )
	{
		type_object->m_enum = IfcRampTypeEnum::ENUM_TWO_STRAIGHT_RUN_RAMP;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcRampTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcRampTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
