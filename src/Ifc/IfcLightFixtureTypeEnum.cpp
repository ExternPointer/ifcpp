/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcLightFixtureTypeEnum.h"

// TYPE IfcLightFixtureTypeEnum = ENUMERATION OF	(DIRECTIONSOURCE	,POINTSOURCE	,SECURITYLIGHTING	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcLightFixtureTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCLIGHTFIXTURETYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_DIRECTIONSOURCE:	stream << ".DIRECTIONSOURCE."; break;
		case ENUM_POINTSOURCE:	stream << ".POINTSOURCE."; break;
		case ENUM_SECURITYLIGHTING:	stream << ".SECURITYLIGHTING."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcLightFixtureTypeEnum> IFC4X3::IfcLightFixtureTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcLightFixtureTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcLightFixtureTypeEnum>(); }
	shared_ptr<IfcLightFixtureTypeEnum> type_object( new IfcLightFixtureTypeEnum() );
	if( std_iequal( arg, ".DIRECTIONSOURCE." ) )
	{
		type_object->m_enum = IfcLightFixtureTypeEnum::ENUM_DIRECTIONSOURCE;
	}
	else if( std_iequal( arg, ".POINTSOURCE." ) )
	{
		type_object->m_enum = IfcLightFixtureTypeEnum::ENUM_POINTSOURCE;
	}
	else if( std_iequal( arg, ".SECURITYLIGHTING." ) )
	{
		type_object->m_enum = IfcLightFixtureTypeEnum::ENUM_SECURITYLIGHTING;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcLightFixtureTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcLightFixtureTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
