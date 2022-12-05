/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/IFC4X3/include/IfcFacilityUsageEnum.h"

// TYPE IfcFacilityUsageEnum = ENUMERATION OF	(LATERAL	,LONGITUDINAL	,REGION	,VERTICAL	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcFacilityUsageEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCFACILITYUSAGEENUM("; }
	switch( m_enum )
	{
		case ENUM_LATERAL:	stream << ".LATERAL."; break;
		case ENUM_LONGITUDINAL:	stream << ".LONGITUDINAL."; break;
		case ENUM_REGION:	stream << ".REGION."; break;
		case ENUM_VERTICAL:	stream << ".VERTICAL."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcFacilityUsageEnum> IFC4X3::IfcFacilityUsageEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcFacilityUsageEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcFacilityUsageEnum>(); }
	shared_ptr<IfcFacilityUsageEnum> type_object( new IfcFacilityUsageEnum() );
	if( std_iequal( arg, ".LATERAL." ) )
	{
		type_object->m_enum = IfcFacilityUsageEnum::ENUM_LATERAL;
	}
	else if( std_iequal( arg, ".LONGITUDINAL." ) )
	{
		type_object->m_enum = IfcFacilityUsageEnum::ENUM_LONGITUDINAL;
	}
	else if( std_iequal( arg, ".REGION." ) )
	{
		type_object->m_enum = IfcFacilityUsageEnum::ENUM_REGION;
	}
	else if( std_iequal( arg, ".VERTICAL." ) )
	{
		type_object->m_enum = IfcFacilityUsageEnum::ENUM_VERTICAL;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcFacilityUsageEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcFacilityUsageEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
