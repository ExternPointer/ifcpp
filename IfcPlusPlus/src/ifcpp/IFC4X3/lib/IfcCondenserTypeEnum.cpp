/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/IFC4X3/include/IfcCondenserTypeEnum.h"

// TYPE IfcCondenserTypeEnum = ENUMERATION OF	(AIRCOOLED	,EVAPORATIVECOOLED	,WATERCOOLED	,WATERCOOLEDBRAZEDPLATE	,WATERCOOLEDSHELLCOIL	,WATERCOOLEDSHELLTUBE	,WATERCOOLEDTUBEINTUBE	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcCondenserTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCCONDENSERTYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_AIRCOOLED:	stream << ".AIRCOOLED."; break;
		case ENUM_EVAPORATIVECOOLED:	stream << ".EVAPORATIVECOOLED."; break;
		case ENUM_WATERCOOLED:	stream << ".WATERCOOLED."; break;
		case ENUM_WATERCOOLEDBRAZEDPLATE:	stream << ".WATERCOOLEDBRAZEDPLATE."; break;
		case ENUM_WATERCOOLEDSHELLCOIL:	stream << ".WATERCOOLEDSHELLCOIL."; break;
		case ENUM_WATERCOOLEDSHELLTUBE:	stream << ".WATERCOOLEDSHELLTUBE."; break;
		case ENUM_WATERCOOLEDTUBEINTUBE:	stream << ".WATERCOOLEDTUBEINTUBE."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcCondenserTypeEnum> IFC4X3::IfcCondenserTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcCondenserTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcCondenserTypeEnum>(); }
	shared_ptr<IfcCondenserTypeEnum> type_object( new IfcCondenserTypeEnum() );
	if( std_iequal( arg, ".AIRCOOLED." ) )
	{
		type_object->m_enum = IfcCondenserTypeEnum::ENUM_AIRCOOLED;
	}
	else if( std_iequal( arg, ".EVAPORATIVECOOLED." ) )
	{
		type_object->m_enum = IfcCondenserTypeEnum::ENUM_EVAPORATIVECOOLED;
	}
	else if( std_iequal( arg, ".WATERCOOLED." ) )
	{
		type_object->m_enum = IfcCondenserTypeEnum::ENUM_WATERCOOLED;
	}
	else if( std_iequal( arg, ".WATERCOOLEDBRAZEDPLATE." ) )
	{
		type_object->m_enum = IfcCondenserTypeEnum::ENUM_WATERCOOLEDBRAZEDPLATE;
	}
	else if( std_iequal( arg, ".WATERCOOLEDSHELLCOIL." ) )
	{
		type_object->m_enum = IfcCondenserTypeEnum::ENUM_WATERCOOLEDSHELLCOIL;
	}
	else if( std_iequal( arg, ".WATERCOOLEDSHELLTUBE." ) )
	{
		type_object->m_enum = IfcCondenserTypeEnum::ENUM_WATERCOOLEDSHELLTUBE;
	}
	else if( std_iequal( arg, ".WATERCOOLEDTUBEINTUBE." ) )
	{
		type_object->m_enum = IfcCondenserTypeEnum::ENUM_WATERCOOLEDTUBEINTUBE;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcCondenserTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcCondenserTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
