/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcElectricTimeControlTypeEnum.h"

// TYPE IfcElectricTimeControlTypeEnum = ENUMERATION OF	(RELAY	,TIMECLOCK	,TIMEDELAY	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcElectricTimeControlTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCELECTRICTIMECONTROLTYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_RELAY:	stream << ".RELAY."; break;
		case ENUM_TIMECLOCK:	stream << ".TIMECLOCK."; break;
		case ENUM_TIMEDELAY:	stream << ".TIMEDELAY."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcElectricTimeControlTypeEnum> IFC4X3::IfcElectricTimeControlTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcElectricTimeControlTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcElectricTimeControlTypeEnum>(); }
	shared_ptr<IfcElectricTimeControlTypeEnum> type_object( new IfcElectricTimeControlTypeEnum() );
	if( std_iequal( arg, ".RELAY." ) )
	{
		type_object->m_enum = IfcElectricTimeControlTypeEnum::ENUM_RELAY;
	}
	else if( std_iequal( arg, ".TIMECLOCK." ) )
	{
		type_object->m_enum = IfcElectricTimeControlTypeEnum::ENUM_TIMECLOCK;
	}
	else if( std_iequal( arg, ".TIMEDELAY." ) )
	{
		type_object->m_enum = IfcElectricTimeControlTypeEnum::ENUM_TIMEDELAY;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcElectricTimeControlTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcElectricTimeControlTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}