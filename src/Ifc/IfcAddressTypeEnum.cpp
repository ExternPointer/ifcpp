/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcAddressTypeEnum.h"

// TYPE IfcAddressTypeEnum = ENUMERATION OF	(DISTRIBUTIONPOINT	,HOME	,OFFICE	,SITE	,USERDEFINED);
void IFC4X3::IfcAddressTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCADDRESSTYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_DISTRIBUTIONPOINT:	stream << ".DISTRIBUTIONPOINT."; break;
		case ENUM_HOME:	stream << ".HOME."; break;
		case ENUM_OFFICE:	stream << ".OFFICE."; break;
		case ENUM_SITE:	stream << ".SITE."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcAddressTypeEnum> IFC4X3::IfcAddressTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcAddressTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcAddressTypeEnum>(); }
	shared_ptr<IfcAddressTypeEnum> type_object( new IfcAddressTypeEnum() );
	if( std_iequal( arg, ".DISTRIBUTIONPOINT." ) )
	{
		type_object->m_enum = IfcAddressTypeEnum::ENUM_DISTRIBUTIONPOINT;
	}
	else if( std_iequal( arg, ".HOME." ) )
	{
		type_object->m_enum = IfcAddressTypeEnum::ENUM_HOME;
	}
	else if( std_iequal( arg, ".OFFICE." ) )
	{
		type_object->m_enum = IfcAddressTypeEnum::ENUM_OFFICE;
	}
	else if( std_iequal( arg, ".SITE." ) )
	{
		type_object->m_enum = IfcAddressTypeEnum::ENUM_SITE;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcAddressTypeEnum::ENUM_USERDEFINED;
	}
	return type_object;
}
