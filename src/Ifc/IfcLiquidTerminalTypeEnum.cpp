/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcLiquidTerminalTypeEnum.h"

// TYPE IfcLiquidTerminalTypeEnum = ENUMERATION OF	(HOSEREEL	,LOADINGARM	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcLiquidTerminalTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCLIQUIDTERMINALTYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_HOSEREEL:	stream << ".HOSEREEL."; break;
		case ENUM_LOADINGARM:	stream << ".LOADINGARM."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcLiquidTerminalTypeEnum> IFC4X3::IfcLiquidTerminalTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcLiquidTerminalTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcLiquidTerminalTypeEnum>(); }
	shared_ptr<IfcLiquidTerminalTypeEnum> type_object( new IfcLiquidTerminalTypeEnum() );
	if( std_iequal( arg, ".HOSEREEL." ) )
	{
		type_object->m_enum = IfcLiquidTerminalTypeEnum::ENUM_HOSEREEL;
	}
	else if( std_iequal( arg, ".LOADINGARM." ) )
	{
		type_object->m_enum = IfcLiquidTerminalTypeEnum::ENUM_LOADINGARM;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcLiquidTerminalTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcLiquidTerminalTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
