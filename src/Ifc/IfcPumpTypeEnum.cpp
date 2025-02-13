/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcPumpTypeEnum.h"

// TYPE IfcPumpTypeEnum = ENUMERATION OF	(CIRCULATOR	,ENDSUCTION	,SPLITCASE	,SUBMERSIBLEPUMP	,SUMPPUMP	,VERTICALINLINE	,VERTICALTURBINE	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcPumpTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCPUMPTYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_CIRCULATOR:	stream << ".CIRCULATOR."; break;
		case ENUM_ENDSUCTION:	stream << ".ENDSUCTION."; break;
		case ENUM_SPLITCASE:	stream << ".SPLITCASE."; break;
		case ENUM_SUBMERSIBLEPUMP:	stream << ".SUBMERSIBLEPUMP."; break;
		case ENUM_SUMPPUMP:	stream << ".SUMPPUMP."; break;
		case ENUM_VERTICALINLINE:	stream << ".VERTICALINLINE."; break;
		case ENUM_VERTICALTURBINE:	stream << ".VERTICALTURBINE."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcPumpTypeEnum> IFC4X3::IfcPumpTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcPumpTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcPumpTypeEnum>(); }
	shared_ptr<IfcPumpTypeEnum> type_object( new IfcPumpTypeEnum() );
	if( std_iequal( arg, ".CIRCULATOR." ) )
	{
		type_object->m_enum = IfcPumpTypeEnum::ENUM_CIRCULATOR;
	}
	else if( std_iequal( arg, ".ENDSUCTION." ) )
	{
		type_object->m_enum = IfcPumpTypeEnum::ENUM_ENDSUCTION;
	}
	else if( std_iequal( arg, ".SPLITCASE." ) )
	{
		type_object->m_enum = IfcPumpTypeEnum::ENUM_SPLITCASE;
	}
	else if( std_iequal( arg, ".SUBMERSIBLEPUMP." ) )
	{
		type_object->m_enum = IfcPumpTypeEnum::ENUM_SUBMERSIBLEPUMP;
	}
	else if( std_iequal( arg, ".SUMPPUMP." ) )
	{
		type_object->m_enum = IfcPumpTypeEnum::ENUM_SUMPPUMP;
	}
	else if( std_iequal( arg, ".VERTICALINLINE." ) )
	{
		type_object->m_enum = IfcPumpTypeEnum::ENUM_VERTICALINLINE;
	}
	else if( std_iequal( arg, ".VERTICALTURBINE." ) )
	{
		type_object->m_enum = IfcPumpTypeEnum::ENUM_VERTICALTURBINE;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcPumpTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcPumpTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
