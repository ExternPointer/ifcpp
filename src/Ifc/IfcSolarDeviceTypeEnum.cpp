/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcSolarDeviceTypeEnum.h"

// TYPE IfcSolarDeviceTypeEnum = ENUMERATION OF	(SOLARCOLLECTOR	,SOLARPANEL	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcSolarDeviceTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCSOLARDEVICETYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_SOLARCOLLECTOR:	stream << ".SOLARCOLLECTOR."; break;
		case ENUM_SOLARPANEL:	stream << ".SOLARPANEL."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcSolarDeviceTypeEnum> IFC4X3::IfcSolarDeviceTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcSolarDeviceTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcSolarDeviceTypeEnum>(); }
	shared_ptr<IfcSolarDeviceTypeEnum> type_object( new IfcSolarDeviceTypeEnum() );
	if( std_iequal( arg, ".SOLARCOLLECTOR." ) )
	{
		type_object->m_enum = IfcSolarDeviceTypeEnum::ENUM_SOLARCOLLECTOR;
	}
	else if( std_iequal( arg, ".SOLARPANEL." ) )
	{
		type_object->m_enum = IfcSolarDeviceTypeEnum::ENUM_SOLARPANEL;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcSolarDeviceTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcSolarDeviceTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
